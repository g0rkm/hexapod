"""hexapod_gazebo.pose — simülasyon kontrol duruşu. ROS gerektirmez."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from hexapod_description import RobotModel
from hexapod_driver import RobotConfig
from hexapod_gazebo.pose import standing_pose
from hexapod_kinematics import HexapodKinematics, ReachError

REAL_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"


@pytest.fixture(scope="module")
def config():
    return RobotConfig.load(REAL_CONFIG)


def test_ayaklar_yerde_ve_istenen_uzaklikta(config):
    kin = HexapodKinematics.from_config(config)
    angles = standing_pose(kin, reach_mm=130.0, height_mm=100.0)
    feet = kin.forward(angles)
    for leg, (x, y, z) in feet.items():
        m = kin.mounts[leg]
        assert z == pytest.approx(-100.0, abs=1e-6)
        assert math.hypot(x - m.x, y - m.y) == pytest.approx(130.0, abs=1e-6)


def test_varsayilan_durus_eklem_limitlerinin_icinde(config):
    model = RobotModel.from_config(config)
    angles = standing_pose(HexapodKinematics.from_config(config), 130.0, 100.0)
    for leg, a in angles.items():
        for part, deg in a.as_dict().items():
            lim = model.limits[(leg, part)]
            assert lim.lower <= math.radians(deg) <= lim.upper, (leg, part, deg)


def test_erisilemeyen_durus_kirpilmaz(config):
    with pytest.raises(ReachError):
        standing_pose(HexapodKinematics.from_config(config), 130.0, 400.0)
