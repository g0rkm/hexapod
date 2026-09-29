"""WASD klavye kumandası: çekirdek (her yerde) + düğüm gerçek süreç olarak (Linux, rclpy).

Düğüm testi sahte bir terminal (pty) açıp tuşları oraya yazar, /cmd_vel'i
dinler: gerçek kullanımdaki gibi (ders 19: düğümü birim testiyle yetinmeden
canlı çalıştır).
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

from hexapod_teleop import TeleopLimits
from hexapod_teleop.wasd import DEFAULT_LEVEL, LEVELS, WasdTeleop

LIM = TeleopLimits()


def test_baslangicta_durur_ve_hiz_010():
    t = WasdTeleop()
    assert t.command() == (0.0, 0.0, 0.0)
    v, w = t.speeds()
    assert math.isclose(v, 0.10) and math.isclose(w, LIM.wz_max * LEVELS[DEFAULT_LEVEL])


@pytest.mark.parametrize("key, expected", [
    ("w", (1, 0)), ("s", (-1, 0)), ("a", (0, 1)), ("d", (0, -1)),
    ("W", (1, 0)), ("S", (-1, 0)), ("A", (0, 1)), ("D", (0, -1)),   # Caps Lock
])
def test_yon_tuslari(key, expected):
    t = WasdTeleop()
    assert t.press(key)
    vx, vy, wz = t.command()
    v, w = t.speeds()
    assert (vx, vy, wz) == (expected[0] * v, 0.0, expected[1] * w)
    assert vy == 0.0


def test_yon_tusu_kalici_k_durdurur_yeni_yon_oncekinin_yerine_gecer():
    t = WasdTeleop()
    t.press("w")
    for _ in range(3):                      # tuş bırakılsa da (başka tuş yok) sürer
        assert t.command()[0] > 0
    t.press("a")                            # ileri + dönüş değil, yerinde dönüş
    assert t.command()[0] == 0.0 and t.command()[2] > 0
    assert t.press("k") and t.command() == (0.0, 0.0, 0.0)
    assert t.press("K")


def test_q_e_kademe_sinirlarda_durur_hareket_yeni_hizla_surer():
    t = WasdTeleop()
    t.press("w")
    for _ in range(10):
        t.press("q")
    assert t.level == len(LEVELS) - 1
    assert math.isclose(t.command()[0], LIM.vx_max)           # en yüksek = TeleopLimits
    for _ in range(10):
        t.press("e")
    assert t.level == 0
    assert math.isclose(t.command()[0], LIM.vx_max * LEVELS[0])
    t.press("d")
    assert math.isclose(t.command()[2], -LIM.wz_max * LEVELS[0])


def test_en_dusuk_kademe_politikanin_olu_bolgesinin_ustunde():
    """Politika düğümü aralığın 1/6'sının altındaki komutta ayakta bekler
    (hexapod_rl.export: command_deadband); en düşük kademe de yürümeli."""
    t = WasdTeleop(level=0)
    v, w = t.speeds()
    assert v > LIM.vx_max / 6 * 1.5 and w > LIM.wz_max / 6 * 1.5


def test_bilinmeyen_tus_yok_sayilir():
    t = WasdTeleop()
    t.press("w")
    before = t.command()
    for key in ("x", " ", "\x1b", "[", "1", "ş"):
        assert not t.press(key)
    assert t.command() == before


def test_gecersiz_kademe():
    with pytest.raises(ValueError):
        WasdTeleop(level=len(LEVELS))


def test_durum_satiri():
    t = WasdTeleop()
    assert t.status().startswith("DUR")
    t.press("s")
    assert t.status().startswith("GERİ")
    t.press("w")
    assert t.status().startswith("İLERİ")
    assert "3/5" in t.status()


# ---------------------------------------------------------------------------
# Düğüm: gerçek süreç, sahte terminal (yalnız Linux + rclpy)
# ---------------------------------------------------------------------------

SRC = Path(__file__).resolve().parent.parent / "src"
PACKAGES = ("hexapod_driver", "hexapod_kinematics", "hexapod_gait", "hexapod_description",
            "hexapod_teleop")
STARTUP_S = 25.0


def _env() -> dict:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [str(SRC / p) for p in PACKAGES] + [env.get("PYTHONPATH", "")])
    env["ROS_DOMAIN_ID"] = "88"
    return env


@pytest.fixture(scope="module")
def ros():
    if sys.platform == "win32":
        pytest.skip("düğüm Linux terminali ister (termios)")
    os.environ["ROS_DOMAIN_ID"] = "88"
    rclpy = pytest.importorskip("rclpy", reason="rclpy yok (ROS 2 Lyrical kaynaklı ortam gerekir)")
    rclpy.init()
    node = rclpy.create_node("test_wasd")
    yield rclpy, node
    node.destroy_node()
    rclpy.try_shutdown()


def test_dugum_terminal_yoksa_aciklayip_cikar(ros):
    proc = subprocess.run([sys.executable, "-m", "hexapod_teleop.wasd_node"], env=_env(),
                          stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=30)
    assert proc.returncode == 2
    assert "terminalde" in proc.stderr


def test_dugum_tuslarla_cmd_vel_yayinlar_ctrl_c_de_durdurur(ros):
    import pty

    from geometry_msgs.msg import Twist

    rclpy, node = ros
    msgs: list[Twist] = []
    node.create_subscription(Twist, "/cmd_vel", msgs.append, 10)

    def spin_until(pred, timeout_s):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if pred():
                return True
        return False

    master, slave = pty.openpty()
    proc = subprocess.Popen([sys.executable, "-m", "hexapod_teleop.wasd_node"], env=_env(),
                            stdin=slave, stdout=slave, stderr=slave, start_new_session=True)
    os.close(slave)
    try:
        # dururken de sürekli sıfır yayınlar (politikanın zaman aşımı dolmasın)
        assert spin_until(lambda: len(msgs) >= 3, STARTUP_S), "wasd /cmd_vel yayınlamadı"
        assert all(m.linear.x == 0.0 and m.angular.z == 0.0 for m in msgs)

        def last_is(vx, wz):
            return lambda: bool(msgs) and math.isclose(msgs[-1].linear.x, vx, abs_tol=1e-6) \
                and math.isclose(msgs[-1].angular.z, wz, abs_tol=1e-6)

        os.write(master, b"w")
        assert spin_until(last_is(0.10, 0.0), 5.0), msgs[-1]
        n = len(msgs)
        assert spin_until(lambda: len(msgs) >= n + 5, 5.0)     # tuş yokken de sürüyor
        assert math.isclose(msgs[-1].linear.x, 0.10, abs_tol=1e-6)

        os.write(master, b"q")
        assert spin_until(last_is(0.125, 0.0), 5.0), msgs[-1]
        os.write(master, b"a")
        assert spin_until(last_is(0.0, 0.5 * 5 / 6), 5.0), msgs[-1]
        os.write(master, b"k")
        assert spin_until(last_is(0.0, 0.0), 5.0), msgs[-1]
        os.write(master, b"s")
        assert spin_until(last_is(-0.125, 0.0), 5.0), msgs[-1]

        # hareket ederken Ctrl+C: temiz çıkış, son komut dur
        proc.send_signal(signal.SIGINT)
        assert proc.wait(timeout=15) == 0
        assert spin_until(last_is(0.0, 0.0), 3.0), msgs[-1]
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
        os.close(master)
