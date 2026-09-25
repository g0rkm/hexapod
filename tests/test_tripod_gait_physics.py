"""hexapod_gait.TripodGait'i gerçek fizikte dener (GOREVLER.md S2/S3, G6 notu).

test_tripod_gait.py saf kinematik doğruluyor (IK, erişim, çapa sabitliği);
bu dosya aynı yürüyüşü hexapod_rl.sim.HexapodSim'e (gz.sim, tork tabanlı
servo modeli) verip robotun GERÇEKTEN devrilmeden ve beklenen hıza yakın
yürüdüğünü ölçer. Yalnızca gz.sim kurulu Linux'ta çalışır (ROS 2 Lyrical).

Görkem'in notu (GOREVLER.md G6, 2026-09-25): "Samet için (S2/S3):
tripod'unu hexapod_rl.sim ile de dene; ROS'lu simülasyon (sim.launch.py)
hâlâ eski servo modelini kullanıyor, orada ayaklar kayar."
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from hexapod_description import RobotModel
from hexapod_driver import RobotConfig
from hexapod_kinematics import HexapodKinematics

REAL_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"

gz_sim = pytest.importorskip("gz.sim", reason="gz.sim yok (ROS 2 Lyrical kurulu Linux gerekir)")

from hexapod_rl.sim import HexapodSim  # noqa: E402

from hexapod_gait import GaitParams, TripodGait  # noqa: E402


def _angles_to_targets(angles) -> list[float]:
    """{bacak: JointAngles} (derece) -> 18 hedef (radyan), interface sırasıyla."""
    return [math.radians(v) for leg in range(6) for v in angles[leg].as_dict().values()]


@pytest.fixture(scope="module")
def kin() -> HexapodKinematics:
    return HexapodKinematics.from_config(RobotConfig.load(REAL_CONFIG))


@pytest.fixture(scope="module")
def model() -> RobotModel:
    return RobotModel.from_config(RobotConfig.load(REAL_CONFIG))


@pytest.fixture
def sim(model, tmp_path_factory) -> HexapodSim:
    return HexapodSim(model, workdir=tmp_path_factory.mktemp("tripod_gait_rl"))


def test_tripod_gercek_fizikte_devrilmeden_beklenen_hiza_yakin_yuruyor(kin, sim):
    """Görkem'in test_simulasyon_yurumeye_izin_veriyor'daki %70 eşiğiyle aynı fikir:
    burada elle yazılmış bir yörünge değil, hexapod_gait.TripodGait'in
    ürettiği gerçek komut kullanılıyor.
    """
    vx = 0.08  # m/s
    gait = TripodGait(kin, GaitParams(cycle_hz=1.5))
    gait.reset()

    sim.reset()
    # 1 sn duruşa yerleş (gait de kendi içinde phase=0'da duruyor sayılır).
    stand = _angles_to_targets(gait.step(0.0, 0.0, 0.0, sim.dt))
    for _ in range(int(round(1.0 / sim.dt)) - 1):
        state = sim.step(stand)

    x0 = state.base_pos[0]
    seconds = 5.0
    for _ in range(int(round(seconds / sim.dt))):
        targets = _angles_to_targets(gait.step(vx, 0.0, 0.0, sim.dt))
        state = sim.step(targets)
        assert state.base_pos[2] > 0.05, "robot devrildi / çöktü"

    speed = (state.base_pos[0] - x0) / seconds
    assert speed > 0.7 * vx, f"gerçek hız {speed:.3f} m/s, komut {vx:.3f} m/s"
    assert abs(state.base_pos[2] - 0.100) < 0.02
