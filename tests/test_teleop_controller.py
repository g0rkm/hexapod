"""hexapod_teleop.TeleopController testleri (saf Python, ROS'suz) — GOREVLER.md S3.

ROS düğümünün (node.py) kendisi Gazebo'da elle doğrulanır (GOREVLER.md'de
sonuçları not edilir, G5'in kendi doğrulaması gibi); burada yalnızca
komut/zaman aşımı/hız sınırlama/ReachError mantığı test edilir.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from hexapod_driver import RobotConfig
from hexapod_kinematics import HexapodKinematics, ReachError

from hexapod_teleop import TeleopController, TeleopLimits

REAL_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"
DT = 1.0 / 50.0


@pytest.fixture(scope="module")
def kin() -> HexapodKinematics:
    return HexapodKinematics.from_config(RobotConfig.load(REAL_CONFIG))


@pytest.fixture
def controller(kin) -> TeleopController:
    return TeleopController(kin, timeout_s=0.5)


# ---------------------------------------------------------------------------
# Temel çıktı
# ---------------------------------------------------------------------------


def test_ilk_tick_18_deger_donduruyor(controller):
    out = controller.tick(now=0.0, dt=DT)
    assert out is not None
    assert len(out) == 18
    assert all(isinstance(v, float) for v in out)


def test_yururken_her_tick_18_deger_radyan(controller):
    controller.on_command(vx=0.08, vy=0.0, wz=0.0, now=0.0)
    for i in range(1, 100):
        out = controller.tick(now=i * DT, dt=DT)
        assert out is not None and len(out) == 18


# ---------------------------------------------------------------------------
# Hız sınırlama
# ---------------------------------------------------------------------------


def test_hiz_siniri_asilan_komut_kirpiliyor(kin):
    controller = TeleopController(kin, limits=TeleopLimits(vx_max=0.1, vy_max=0.05, wz_max=0.3))
    controller.on_command(vx=10.0, vy=-10.0, wz=10.0, now=0.0)
    assert controller._command == (0.1, -0.05, 0.3)


def test_sinir_icindeki_komut_degismiyor(kin):
    controller = TeleopController(kin, limits=TeleopLimits(vx_max=0.1, vy_max=0.05, wz_max=0.3))
    controller.on_command(vx=0.05, vy=-0.02, wz=0.1, now=0.0)
    assert controller._command == pytest.approx((0.05, -0.02, 0.1))


# ---------------------------------------------------------------------------
# Zaman aşımı (deadman)
# ---------------------------------------------------------------------------


def test_komut_hic_gelmezse_sifir_hizla_ilerliyor(controller):
    for i in range(20):
        controller.tick(now=i * DT, dt=DT)
    assert controller.gait._pose.x == pytest.approx(0.0, abs=1e-9)
    assert controller.gait._pose.y == pytest.approx(0.0, abs=1e-9)


def test_zaman_asiminda_ilerleme_duruyor(controller):
    controller.on_command(vx=0.1, vy=0.0, wz=0.0, now=0.0)
    t = 0.0
    for _ in range(30):
        t += DT
        controller.tick(now=t, dt=DT)
    x_before = controller.gait._pose.x
    assert x_before > 0.0  # komut aktifken gerçekten ilerlemiş

    t += 1.0  # timeout_s=0.5'i aş
    for _ in range(30):
        t += DT
        controller.tick(now=t, dt=DT)
    x_after = controller.gait._pose.x

    assert x_after == pytest.approx(x_before, abs=1e-6)


def test_taze_komut_gelince_tekrar_ilerliyor(controller):
    controller.on_command(vx=0.1, vy=0.0, wz=0.0, now=0.0)
    t = 0.0
    for _ in range(30):
        t += DT
        controller.tick(now=t, dt=DT)
    x_before = controller.gait._pose.x

    t += 1.0
    controller.on_command(vx=0.1, vy=0.0, wz=0.0, now=t)  # taze komut
    for _ in range(30):
        t += DT
        controller.tick(now=t, dt=DT)
    x_after = controller.gait._pose.x

    assert x_after > x_before + 1.0  # yeniden ilerledi (mm cinsinden)


# ---------------------------------------------------------------------------
# Erişilemeyen hedef: çökme yok, son iyi komut korunuyor
# ---------------------------------------------------------------------------


def test_erisim_hatasinda_son_gecerli_komut_korunuyor(kin, monkeypatch):
    controller = TeleopController(kin)
    controller.on_command(vx=0.05, vy=0.0, wz=0.0, now=0.0)
    good = controller.tick(now=0.02, dt=DT)
    assert good is not None

    def patlar(*args, **kwargs):
        raise ReachError("test: erişilemez hedef")

    monkeypatch.setattr(controller.gait, "step", patlar)
    result = controller.tick(now=0.04, dt=DT)
    assert result == good
