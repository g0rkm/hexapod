"""hexapod_policy düğümü gerçek süreç olarak (GOREVLER.md G8; test_ros_nodes.py deseni).

Yalnız rclpy ve stable_baselines3 olan ortamda (WSL: ROS 2 Lyrical + ~/hexapod_venv).
Düğüm `python -m hexapod_policy.node` ile başlatılır, /imu ve /cmd_vel
yayınlanıp /leg_controller/commands dinlenir.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

os.environ["ROS_DOMAIN_ID"] = "88"   # açık bir simülasyonla ya da öteki testlerle karışmasın

rclpy = pytest.importorskip("rclpy", reason="rclpy yok (ROS 2 Lyrical kaynaklı ortam gerekir)")
pytest.importorskip("stable_baselines3", reason="SB3 yok (tools/wsl/rl_kurulum.sh)")

from geometry_msgs.msg import Twist  # noqa: E402
from sensor_msgs.msg import Imu, Range  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

from hexapod_description.interface import COMMAND_TOPIC, IMU_TOPIC  # noqa: E402
from hexapod_policy import MlpPolicy  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "src"
PACKAGES = ("hexapod_driver", "hexapod_kinematics", "hexapod_description", "hexapod_policy")
STARTUP_S = 25.0


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    node = rclpy.create_node("test_policy_node")
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


@pytest.fixture(scope="module")
def policy_file(tmp_path_factory):
    from hexapod_rl.export import export
    return export(REPO / "models" / "taklit_bc_v4" / "model.zip",
                  tmp_path_factory.mktemp("pol") / "policy.npz")


def start(*ros_args: str) -> subprocess.Popen:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(SRC / p) for p in PACKAGES]
                                        + [env.get("PYTHONPATH", "")])
    return subprocess.Popen([sys.executable, "-m", "hexapod_policy.node", "--ros-args", *ros_args],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def stop(proc: subprocess.Popen) -> tuple[int, str]:
    proc.send_signal(signal.SIGTERM)
    try:
        _, err = proc.communicate(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()
        _, err = proc.communicate()
        pytest.fail(f"düğüm SIGTERM'e 15 sn'de kapanmadı:\n{err}")
    return proc.returncode, err


def spin_until(node, predicate, timeout_s, each=None) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if each is not None:
            each()
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return False


def test_politika_dosyasi_yoksa_cikis_kodu_2():
    proc = subprocess.run([sys.executable, "-m", "hexapod_policy.node"], capture_output=True,
                          text=True, timeout=60,
                          env={**os.environ, "PYTHONPATH": os.pathsep.join(
                              [str(SRC / p) for p in PACKAGES] + [os.environ.get("PYTHONPATH", "")])})
    assert proc.returncode == 2
    assert "policy" in proc.stderr and "hexapod_rl.export" in proc.stderr


def test_dugum_ayakta_bekler_komutla_yurur_temiz_kapanir(ros, policy_file):
    stand = list(MlpPolicy.load(policy_file).contract.default_rad)
    proc = start("-p", f"policy:={policy_file}")
    try:
        imu_pub = ros.create_publisher(Imu, IMU_TOPIC, 10)
        cmd_pub = ros.create_publisher(Twist, "/cmd_vel", 10)
        got: list[list[float]] = []
        ros.create_subscription(Float64MultiArray, COMMAND_TOPIC, lambda m: got.append(list(m.data)), 10)

        # Komut yokken: 18 değerlik ayakta duruş yayınlanır.
        assert spin_until(ros, lambda: len(got) >= 5, STARTUP_S), "düğüm komut yayınlamadı"
        assert len(got[-1]) == 18 and got[-1] == pytest.approx(stand)

        imu = Imu()
        imu.orientation.w = 1.0
        twist = Twist()
        twist.linear.x = 0.1

        def feed():
            imu_pub.publish(imu)
            cmd_pub.publish(twist)

        # IMU + ileri komut: politika koşar, duruştan farklı hedefler gelir.
        n0 = len(got)
        assert spin_until(ros, lambda: len(got) > n0 + 20
                          and max(abs(a - b) for a, b in zip(got[-1], stand)) > 0.01, 10.0, feed), \
            "politika yürümeye başlamadı"
    finally:
        code, err = stop(proc)
    assert code == 0, err
    assert "Traceback" not in err


REFLEX_POLICY = REPO / "models" / "ppo_kaldirma35_250k" / "policy.npz"   # robota aday (artık eylem)


def test_refleks_yerlesimi_bilinmiyorsa_eksik_alani_soyleyip_cikar():
    """Depodaki robot.yaml'da yerleşim D8'e kadar null: -p reflex:=true ile düğüm
    başlamamalı, neyin eksik olduğunu söylemeli (değer uydurulmaz)."""
    env = {**os.environ, "PYTHONPATH": os.pathsep.join(
        [str(SRC / p) for p in PACKAGES] + [os.environ.get("PYTHONPATH", "")])}
    proc = subprocess.run([sys.executable, "-m", "hexapod_policy.node", "--ros-args",
                           "-p", f"policy:={REFLEX_POLICY}", "-p", "reflex:=true",
                           "-p", f"config:={REPO / 'config' / 'robot.yaml'}"],
                          capture_output=True, text=True, timeout=60, env=env)
    if proc.returncode == 0:
        pytest.skip("robot.yaml'da yerleşim girilmiş (D8 sonrası)")
    assert proc.returncode == 2, proc.stderr
    assert "sensors.range_finders" in proc.stderr and "Traceback" not in proc.stderr


def test_refleksli_dugum_mesafe_konularini_dinler(ros, tmp_path):
    """-p reflex:=true + DENEYSEL yerleşimli config kopyası: /range0..2'den gelen
    ölçümle refleks "görüyor"; mesafe kesilince "mesafe bayat" (kör davranış)."""
    from hexapod_rl.deneysel_yerlesim import experimental_config

    cfg = experimental_config(tmp_path / "robot.yaml")
    proc = start("-p", f"policy:={REFLEX_POLICY}", "-p", "reflex:=true", "-p", f"config:={cfg}")
    try:
        imu_pub = ros.create_publisher(Imu, IMU_TOPIC, 10)
        cmd_pub = ros.create_publisher(Twist, "/cmd_vel", 10)
        range_pubs = [ros.create_publisher(Range, f"/range{i}", 10) for i in range(3)]
        got: list[list[float]] = []
        ros.create_subscription(Float64MultiArray, COMMAND_TOPIC, lambda m: got.append(list(m.data)), 10)
        assert spin_until(ros, lambda: all(p.get_subscription_count() >= 1 for p in range_pubs),
                          STARTUP_S), "düğüm /range0..2'yi dinlemiyor"

        imu = Imu()
        imu.orientation.w = 1.0
        twist = Twist()
        twist.linear.x = 0.1
        near = Range()
        near.max_range, near.range = 1.2, 0.25          # önde yakın bir engel

        def feed(with_ranges=True):
            imu_pub.publish(imu)
            cmd_pub.publish(twist)
            if with_ranges:
                for p in range_pubs:
                    p.publish(near)

        n0 = len(got)
        assert spin_until(ros, lambda: len(got) > n0 + 50, 10.0, feed), "düğüm yayınlamadı"
        spin_until(ros, lambda: False, 1.0, lambda: feed(with_ranges=False))   # mesafe kesildi
        assert proc.poll() is None, "düğüm çöktü"
    finally:
        code, err = stop(proc)
    assert code == 0, err
    assert "Traceback" not in err
    assert "/range0, /range1, /range2" in err, err
    assert "refleks: görüyor" in err and "refleks: mesafe bayat" in err, err
