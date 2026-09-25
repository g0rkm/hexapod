"""ROS düğümlerinin (node.py) gerçek süreç olarak testi — GOREVLER.md S3, S4.

Yalnızca rclpy kurulu ortamda çalışır (WSL, ROS 2 Lyrical kaynaklı); Windows'ta atlanır.

Neden var: ROS'suz çekirdek testleri (controller.py) düğüm kabuğuna dokunmaz.
hexapod_hardware düğümü bu yüzden ilk reddedilen komutta çöktü (Lyrical'da
`logger.warn` yok, `warning` var) ve hiçbir birim testi bunu görmedi; kapanışta
(SIGTERM) hata izi basıp çıkış kodu 1 vermesi de öyle. Burada düğümler
`python -m ...` ile gerçekten başlatılıp konulardan sürülür. İki hata da
geri konarak bu testlerin yakaladığı doğrulandı (2026-09-25).
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest
import yaml

# Çalışan başka bir ROS ağıyla (ör. açık bir simülasyon) karışmasın; rclpy.init'ten önce.
os.environ["ROS_DOMAIN_ID"] = "87"

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (ROS 2 Lyrical kaynaklı ortam gerekir)")

from geometry_msgs.msg import Twist  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

from hexapod_description.interface import COMMAND_TOPIC, STATE_TOPIC, joint_names  # noqa: E402
from test_driver_controller import make_calibration, wired_config  # noqa: E402  (aynı UYDURMA kablolama)

SRC = Path(__file__).resolve().parent.parent / "src"
PACKAGES = ("hexapod_driver", "hexapod_kinematics", "hexapod_gait", "hexapod_description",
            "hexapod_teleop", "hexapod_hardware")
STARTUP_S = 25.0  # WSL'de ilk keşif ve süreç açılışı yavaş olabiliyor


# ---------------------------------------------------------------------------
# Yardımcılar
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    node = rclpy.create_node("test_ros_nodes")
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


def start(module: str, *ros_args: str) -> subprocess.Popen:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [str(SRC / p) for p in PACKAGES] + [env.get("PYTHONPATH", "")])
    return subprocess.Popen(
        [sys.executable, "-m", module, "--ros-args", *ros_args],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def stop(proc: subprocess.Popen) -> tuple[int, str]:
    """SIGTERM gönder (robotta servis durdurma yolu), (çıkış kodu, stderr) döndür.

    SIGINT değil: doğrudan başlatılan süreçte Ctrl+C `KeyboardInterrupt`
    yolundan geçer ve düzeltme olmasa da temiz görünür. SIGTERM'de rclpy
    bağlamı kapatır, spin ExternalShutdownException fırlatır; düzeltme
    yokken hata izi ve çıkış kodu 1 verir (deneyle doğrulandı).
    """
    proc.send_signal(signal.SIGTERM)
    try:
        _, err = proc.communicate(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()
        _, err = proc.communicate()
        pytest.fail(f"düğüm SIGINT'e 15 sn'de kapanmadı:\n{err}")
    return proc.returncode, err


def spin_until(node, predicate, timeout_s: float, each=None) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if each is not None:
            each()
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def write_wiring(tmp_path: Path) -> tuple[str, str]:
    config = wired_config(tmp_path)
    make_calibration(config).save(tmp_path / "calibration.yaml")
    return str(config.path), str(tmp_path / "calibration.yaml")


# ---------------------------------------------------------------------------
# hexapod_hardware (S4)
# ---------------------------------------------------------------------------


def test_surucu_dugumu_komutu_yansitiyor_bozuk_komutta_cokmuyor_temiz_kapaniyor(ros, tmp_path):
    config, calibration = write_wiring(tmp_path)
    proc = start("hexapod_hardware.node", "-p", "dry_run:=true",
                 "-p", f"config:={config}", "-p", f"calibration:={calibration}")
    try:
        pub = ros.create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)
        states: list[JointState] = []
        ros.create_subscription(JointState, STATE_TOPIC, states.append, 10)

        assert spin_until(ros, lambda: pub.get_subscription_count() >= 1, STARTUP_S), \
            "sürücü düğümü komut konusunu dinlemeye başlamadı"

        # Bozuk komut (3 değer): reddedilmeli ama düğüm ÇÖKMEMELİ (warn/warning hatası).
        for _ in range(5):
            pub.publish(Float64MultiArray(data=[0.0, 0.1, 0.2]))
            spin_until(ros, lambda: False, 0.1)
        spin_until(ros, lambda: False, 1.5)  # çökecekse çökmeye vakit tanı
        assert proc.poll() is None, "düğüm bozuk komutta çöktü"
        assert states == [], "hiç geçerli komut yokken /joint_states yayınlanmamalı"

        # Geçerli komut: /joint_states'e yansımalı.
        good = [0.05 * i for i in range(18)]
        assert spin_until(ros, lambda: len(states) > 0, 10.0,
                          each=lambda: pub.publish(Float64MultiArray(data=good))), \
            "geçerli komut /joint_states'e yansımadı"
        assert list(states[-1].name) == joint_names(range(6))
        assert list(states[-1].position) == pytest.approx(good)

        # Limit dışı komut (2.0 rad ~ 115 derece > 90): reddedilmeli, durum bozulmamalı.
        over = list(good)
        over[4] = 2.0
        for _ in range(5):
            pub.publish(Float64MultiArray(data=over))
            spin_until(ros, lambda: False, 0.1)
        spin_until(ros, lambda: False, 1.5)
        assert proc.poll() is None, "düğüm limit dışı komutta çöktü"
        assert list(states[-1].position) == pytest.approx(good)
    finally:
        code, err = stop(proc)

    assert "Traceback" not in err, f"kapanışta hata izi:\n{err}"
    assert code == 0, f"çıkış kodu {code}:\n{err}"


def test_surucu_dugumu_eksik_kablolamada_temiz_hata_verip_cikiyor(tmp_path):
    config, calibration = write_wiring(tmp_path)
    raw = yaml.safe_load(Path(config).read_text(encoding="utf-8"))
    raw["drivers"][1]["address"]["value"] = None  # ikinci kartın adresi bilinmiyor
    Path(config).write_text(yaml.safe_dump(raw, allow_unicode=True), encoding="utf-8")

    proc = start("hexapod_hardware.node", "-p", "dry_run:=true",
                 "-p", f"config:={config}", "-p", f"calibration:={calibration}")
    try:
        _, err = proc.communicate(timeout=STARTUP_S)
    except subprocess.TimeoutExpired:
        proc.kill()
        pytest.fail("eksik kablolamada düğüm kendiliğinden çıkmadı")
    assert proc.returncode == 2
    assert "drivers[1].address" in err, "eksik alan adı söylenmeli"
    assert "Traceback" not in err


# ---------------------------------------------------------------------------
# hexapod_teleop (S3)
# ---------------------------------------------------------------------------


def test_teleop_dugumu_cmd_vel_dinleyip_18_deger_yayinliyor_temiz_kapaniyor(ros):
    proc = start("hexapod_teleop.node")
    try:
        pub = ros.create_publisher(Twist, "/cmd_vel", 10)
        commands: list[Float64MultiArray] = []
        ros.create_subscription(Float64MultiArray, COMMAND_TOPIC, commands.append, 10)

        twist = Twist()
        twist.linear.x = 0.08
        assert spin_until(ros, lambda: len(commands) >= 20, STARTUP_S,
                          each=lambda: pub.publish(twist)), "teleop komut yayınlamadı"
        assert proc.poll() is None
        for msg in commands:
            assert len(msg.data) == 18
            assert all(math.isfinite(v) for v in msg.data)
    finally:
        code, err = stop(proc)

    assert "Traceback" not in err, f"kapanışta hata izi:\n{err}"
    assert code == 0, f"çıkış kodu {code}:\n{err}"
