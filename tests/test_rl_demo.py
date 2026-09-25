"""hexapod_rl.demo — taklit için gösterim tripod'u. Saf Python, her yerde koşar."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from hexapod_driver import RobotConfig
from hexapod_gazebo.pose import standing_pose
from hexapod_kinematics import HexapodKinematics
from hexapod_rl.demo import TripodDemo
from hexapod_rl.task import TaskConfig, tripod_groups

REAL_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"


@pytest.fixture(scope="module")
def demo() -> TripodDemo:
    task = TaskConfig()
    kin = HexapodKinematics.from_config(RobotConfig.load(REAL_CONFIG))
    pose = standing_pose(kin, task.stand_reach_mm, task.stand_height_mm)
    default = [math.radians(v) for leg in sorted(pose) for v in pose[leg].as_dict().values()]
    return TripodDemo(kin, default, task.action_scale,
                      tripod_groups({leg: m.yaw for leg, m in kin.mounts.items()}),
                      task.gait_hz, task.stand_reach_mm, task.stand_height_mm)


def test_komutsuz_saat_basinda_eylem_sifir(demo):
    assert demo.action(0.0, (0.0, 0.0, 0.0)) == pytest.approx([0.0] * 18, abs=1e-9)


def test_saatin_ilk_yarisinda_ilk_grup_havada(demo):
    """task.gait_score ile aynı sözleşme: ilk yarıda groups[0] salınımda."""
    home_z = -demo.height_mm
    for phase, up in ((0.25, demo.groups[0]), (0.75, demo.groups[1])):
        feet = demo.feet(phase, (0.1, 0.0, 0.0))
        for leg, (_, _, z) in feet.items():
            if leg in up:
                assert z == pytest.approx(home_z + demo.lift_mm)
            else:
                assert z == pytest.approx(home_z)


def test_destekte_ayak_hizla_orantili_geri_kayar(demo):
    """Bir destek fazında (0.5/hz s) ayak gövdeye göre v x 0.5/hz geri gider."""
    vx = 0.12
    leg = demo.groups[1][0]                  # ilk yarıda yerde
    x0 = demo.feet(0.0, (vx, 0.0, 0.0))[leg][0]
    x1 = demo.feet(0.4999999, (vx, 0.0, 0.0))[leg][0]
    assert x0 - x1 == pytest.approx(vx * 1000.0 * 0.5 / demo.gait_hz, rel=1e-5)


def test_donuste_destek_ayagi_teget_kayar(demo):
    """wz > 0 (saat yönünün tersi) iken yerdeki ayak gövdeye göre ters yönde,
    merkeze teğet gider; merkeze uzaklığı değişmez."""
    leg = demo.groups[1][0]
    a = demo.feet(0.0, (0.0, 0.0, 0.5))[leg]
    b = demo.feet(0.4999999, (0.0, 0.0, 0.5))[leg]
    d = (b[0] - a[0], b[1] - a[1])
    mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    cos = (d[0] * mid[0] + d[1] * mid[1]) / (math.hypot(*d) * math.hypot(*mid))
    assert cos == pytest.approx(0.0, abs=1e-6)                             # teğet
    assert mid[0] * d[1] - mid[1] * d[0] < 0                               # saat yönünde


@pytest.mark.parametrize("vx", [0.0, 0.05, 0.10, 0.15])
def test_komut_araliginda_eylem_kirpilmiyor(demo, vx):
    """Eğitimdeki hız aralığında gösterim eylem sınırlarına ([-1, 1]) sığmalı,
    yoksa taklit edilen yürüyüş kırpılmış hâli olur."""
    for k in range(100):
        a = demo.action(k / 100, (vx, 0.0, 0.0))
        assert max(abs(v) for v in a) <= 1.0
