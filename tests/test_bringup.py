"""hexapod_bringup: gerçek robotun başlatma dosyası, robotsuz uçtan uca (dry_run).

Yalnız ROS 2 ortamında (rclpy + ros2 CLI, çalışma alanı derlenmiş); Windows'ta
atlanır. robot.launch.py `ros2 launch` ile gerçek süreç olarak açılır: sensör
düğümü taklit cihazlarla /imu ve /range0..2 yayınlar, politika düğümü
/cmd_vel ile yürür, sürücü düğümü komutları kabul edip /joint_states'e
yansıtır. Config ve kalibrasyon UYDURMA değerli kopyalar (tmp); depodaki
robot.yaml'a dokunulmaz.
"""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import pytest
import yaml

os.environ["ROS_DOMAIN_ID"] = "86"   # açık bir simle ya da öteki testlerle karışmasın

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (ROS 2 Lyrical kaynaklı ortam gerekir)")
if shutil.which("ros2") is None:
    pytest.skip("ros2 komutu yok", allow_module_level=True)

from geometry_msgs.msg import Twist  # noqa: E402
from sensor_msgs.msg import Imu, JointState, Range  # noqa: E402

from hexapod_description.interface import STATE_TOPIC  # noqa: E402
from test_driver_controller import make_calibration, wired_config  # noqa: E402  (UYDURMA kablolama)

REPO = Path(__file__).resolve().parent.parent
LAUNCH = REPO / "src" / "hexapod_bringup" / "launch" / "robot.launch.py"
STARTUP_S = 40.0


def full_fake_config(tmp_path: Path) -> tuple[str, str]:
    """UYDURMA ama eksiksiz config: kablolama + IMU + mesafe sensörü yerleşimi
    (DENEYSEL, eğitimdeki). Yalnız test; gerçek değerler D5/D8'de girilir."""
    from hexapod_rl.deneysel_yerlesim import experimental_config

    config = wired_config(tmp_path)
    make_calibration(config).save(tmp_path / "calibration.yaml")
    path = experimental_config(tmp_path / "robot_tam.yaml", config_path=config.path)
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    imu = raw["sensors"]["imu"]
    imu["address"] = {"value": 0x28, "source": "UYDURMA (test)", "measured": False}
    imu["mount_rotation_deg"] = {"value": 0.0, "source": "UYDURMA (test)", "measured": False}
    for i, dev in enumerate(raw["sensors"]["range_finders"]["devices"]):
        dev["xshut_gpio"] = (17, 27, 22)[i]
        dev["address"] = 0x30 + i
    path.write_text(yaml.safe_dump(raw, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return str(path), str(tmp_path / "calibration.yaml")


def launch(*args: str) -> subprocess.Popen:
    # Kendi süreç grubunda: kapatırken bütün düğümler gider.
    return subprocess.Popen(["ros2", "launch", str(LAUNCH), *args], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, start_new_session=True)


def stop(proc: subprocess.Popen) -> tuple[int, str]:
    """Terminalde Ctrl+C gibi: SIGINT bütün gruba gider, launch da her düğüme bir
    daha iletir (düğüm iki sinyal alır). 2026-09-29'a kadar düğümler burada
    sinyalle ölüyordu (çıkış -2), sürücünün servoları bırakması yarıda
    kalabiliyordu (hexapod_driver.stop_signals)."""
    os.killpg(proc.pid, signal.SIGINT)
    try:
        out, _ = proc.communicate(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        out, _ = proc.communicate()
        pytest.fail(f"launch SIGINT'e 30 sn'de kapanmadı:\n{out[-3000:]}")
    return proc.returncode, out


def spin_until(node, predicate, timeout_s, each=None) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if each is not None:
            each()
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return False


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    node = rclpy.create_node("test_bringup")
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


def test_robot_tek_komutla_kalkar_komutla_yurur_temiz_kapanir(ros, tmp_path):
    config, calibration = full_fake_config(tmp_path)
    proc = launch("dry_run:=true", "reflex:=true", f"config:={config}",
                  f"calibration:={calibration}")
    try:
        states: list[list[float]] = []
        imus, ranges = [], {0: [], 1: [], 2: []}
        ros.create_subscription(JointState, STATE_TOPIC, lambda m: states.append(list(m.position)), 10)
        ros.create_subscription(Imu, "/imu", imus.append, 10)
        for i in ranges:
            ros.create_subscription(Range, f"/range{i}", ranges[i].append, 10)
        cmd = ros.create_publisher(Twist, "/cmd_vel", 10)

        # Komut yokken: sensörler yayında, politika ayakta duruş, sürücü kabul ediyor.
        assert spin_until(ros, lambda: len(states) >= 20 and imus and all(ranges.values()),
                          STARTUP_S), "sistem kalkmadı (/joint_states, /imu, /range0..2)"
        assert proc.poll() is None
        stand = states[-1]
        assert len(stand) == 18
        assert all(r.range == pytest.approx(0.4, abs=0.01) for r in ranges[0][-3:])  # taklit sensör

        # İleri komut: politika yürür, sürücüye ulaşan açılar duruştan ayrılır.
        twist = Twist()
        twist.linear.x = 0.1
        n0 = len(states)
        assert spin_until(ros, lambda: len(states) > n0 + 50
                          and max(abs(a - b) for a, b in zip(states[-1], stand)) > 0.05,
                          15.0, lambda: cmd.publish(twist)), "komutla yürüme sürücüye ulaşmadı"
        assert proc.poll() is None, "bir düğüm çıktı"
    finally:
        code, out = stop(proc)

    assert "Traceback" not in out, out[-3000:]
    for name in ("hexapod_sensors hazır", "hexapod_hardware hazır", "hexapod_policy hazır"):
        assert name in out, f"{name} yok:\n{out[-3000:]}"
    assert "refleks: görüyor" in out, out[-3000:]
    assert "durum: yürüyor" in out, out[-3000:]
    assert "process has died" not in out, out[-3000:]           # sinyalle ölen düğüm yok
    for proc_name in ("sensors-1", "driver-2", "policy-3"):
        assert f"[{proc_name}]: process has finished cleanly" in out, out[-3000:]


def test_kamera_yayini_acilir_kamera_cokse_de_robot_calisir(ros, tmp_path):
    """camera:=deneme: kamera yayını (deneme deseni) robotla birlikte açılır.
    Kamera yalnız izlemek için; yayın düğümü ölünce (kamera arızası gibi)
    robotun geri kalanı "biri çıkarsa hepsi kapansın" kuralıyla KAPANMAMALI."""
    pytest.importorskip("PIL", reason="deneme deseni Pillow ister")
    import http.client
    import socket

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    config, calibration = full_fake_config(tmp_path)
    proc = launch("dry_run:=true", f"config:={config}", f"calibration:={calibration}",
                  "camera:=deneme", f"camera_port:={port}")
    try:
        states = []
        ros.create_subscription(JointState, STATE_TOPIC, lambda m: states.append(1), 10)
        assert spin_until(ros, lambda: len(states) >= 20, STARTUP_S), "robot kalkmadı"

        def camera_pids() -> list[int]:
            out = subprocess.run(["pgrep", "-g", str(proc.pid), "-f", "hexapod_camera/stream"],
                                 capture_output=True, text=True).stdout
            return [int(p) for p in out.split()]

        assert spin_until(ros, lambda: bool(camera_pids()), 10.0), "kamera yayını açılmadı"
        end = time.monotonic() + 15
        status = 0
        while time.monotonic() < end and status != 200:
            try:
                conn = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
                conn.request("GET", "/kare.jpg")
                r = conn.getresponse()
                status, body = r.status, r.read()
                conn.close()
            except OSError:
                time.sleep(0.2)
        assert status == 200 and body[:2] == b"\xff\xd8", "tarayıcı yayını kare vermiyor"

        for pid in camera_pids():                   # kamera arızası: yayın düğümü ölür
            os.kill(pid, signal.SIGKILL)
        n = len(states)
        assert spin_until(ros, lambda: len(states) > n + 100, 10.0), "robot durdu"
        assert proc.poll() is None, "kamera ölünce bütün sistem kapandı"
    finally:
        code, out = stop(proc)
    assert "hexapod_camera hazır: kaynak deneme deseni" in out, out[-3000:]
    for proc_name in ("sensors-1", "driver-2", "policy-3"):
        assert f"[{proc_name}]: process has finished cleanly" in out, out[-3000:]


def test_eksik_configte_eksigi_soyleyip_sistemi_kapatir(tmp_path):
    """Depodaki robot.yaml'da kablolama boş: sürücü çıkış 2 verir, launch
    bütün sistemi kapatır (yarım çalışan robot bırakmaz)."""
    proc = launch("dry_run:=true", f"config:={REPO / 'config' / 'robot.yaml'}")
    try:
        out, _ = proc.communicate(timeout=STARTUP_S + 20)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        out, _ = proc.communicate()
        pytest.fail(f"eksik config'te sistem kapanmadı:\n{out[-3000:]}")
    assert "drivers[" in out or "sensors." in out, out[-3000:]     # eksik alanın adı
    assert "Traceback" not in out, out[-3000:]                     # kapanış sessiz
    assert "shutting down launched system" in out, out[-3000:]     # biri çıkınca hepsi
    assert "exit code -2" not in out, out[-3000:]                  # sinyalle ölen yok
    assert proc.returncode is not None
