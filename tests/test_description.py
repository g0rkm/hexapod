"""Simülasyon modeli (hexapod_description) testleri. Donanım ve ROS gerektirmez."""

from __future__ import annotations

import copy
import math
from pathlib import Path

import pytest
import yaml

from hexapod_description import RobotModel
from hexapod_driver import RobotConfig
from hexapod_driver.errors import ConfigError, MissingValue
from hexapod_kinematics import HexapodKinematics

REAL_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"


def config_with(tmp_path: Path, edit) -> RobotConfig:
    """Gerçek robot.yaml'ın değiştirilmiş bir kopyası (değişiklik yalnızca testte)."""
    raw = copy.deepcopy(yaml.safe_load(REAL_CONFIG.read_text(encoding="utf-8")))
    edit(raw)
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(raw, allow_unicode=True), encoding="utf-8")
    return RobotConfig.load(path)


@pytest.fixture(scope="module")
def model() -> RobotModel:
    return RobotModel.from_config(RobotConfig.load(REAL_CONFIG))


def test_geometri_ik_ile_ayni(model):
    kin = HexapodKinematics.from_config(RobotConfig.load(REAL_CONFIG))
    assert model.coxa == pytest.approx(kin.leg.coxa / 1000)
    assert model.femur == pytest.approx(kin.leg.femur / 1000)
    assert model.tibia == pytest.approx(kin.leg.tibia / 1000)
    for i, m in kin.mounts.items():
        got = model.mounts[i]
        assert (got.x, got.y, got.z) == pytest.approx((m.x / 1000, m.y / 1000, m.z / 1000))
        assert got.yaw == pytest.approx(m.yaw)


def test_kutleler_ve_ataletler_fiziksel(model):
    for name, link in model.links.items():
        ixx, iyy, izz, ixy, ixz, iyz = link.inertial.inertia
        assert link.inertial.mass > 0, name
        # pozitif tanımlı (Sylvester) ve üçgen eşitsizliği
        assert ixx > 0 and ixx * iyy - ixy * ixy > 0, name
        det = (ixx * (iyy * izz - iyz * iyz) - ixy * (ixy * izz - iyz * ixz)
               + ixz * (ixy * iyz - iyy * ixz))
        assert det > 0, name
        assert ixx + iyy >= izz and iyy + izz >= ixx and ixx + izz >= iyy, name
        assert all(s > 0 for box in link.boxes for s in box.size), name
    # 2 kg civarı; tahminin kaba bir akıl kontrolü
    assert 1.5 < model.total_mass() < 3.0


def test_aynali_bacakta_y_ters(model):
    normal = [i for i, m in model.mirrored.items() if not m][0]
    aynali = [i for i, m in model.mirrored.items() if m][0]
    for name in ("coxa", "femur", "tibia"):
        a = model.leg_link(normal, name)
        b = model.leg_link(aynali, name)
        assert b.inertial.com[1] == pytest.approx(-a.inertial.com[1])
        assert b.inertial.com[0] == pytest.approx(a.inertial.com[0])
        assert b.inertial.inertia[3] == pytest.approx(-a.inertial.inertia[3])  # ixy
        assert b.inertial.inertia[5] == pytest.approx(-a.inertial.inertia[5])  # iyz
        assert b.boxes[0].center[1] == pytest.approx(-a.boxes[0].center[1])


def test_kalibrasyon_limiti_yoksa_gecici_limit(model):
    assert len(model.provisional_joints()) == 18
    lim = model.limits[(0, "femur")]
    assert (lim.lower, lim.upper) == pytest.approx((-math.pi / 2, math.pi / 2))


def test_kalibrasyon_limiti_varsa_o_kullanilir(tmp_path):
    def edit(raw):
        raw["joints"][1]["limits_deg"] = {"min": -40.0, "max": 70.0}  # bacak 0 femur

    model = RobotModel.from_config(config_with(tmp_path, edit))
    lim = model.limits[(0, "femur")]
    assert not lim.provisional
    assert (lim.lower, lim.upper) == pytest.approx((math.radians(-40), math.radians(70)))
    assert len(model.provisional_joints()) == 17


def test_eksik_simulasyon_degeri_uydurulmaz(tmp_path):
    def edit(raw):
        raw["simulation"]["links"]["tibia"]["mass_kg"]["value"] = None

    with pytest.raises(MissingValue, match="simulation.links.tibia.mass_kg"):
        RobotModel.from_config(config_with(tmp_path, edit))


def test_ters_limit_reddedilir(tmp_path):
    def edit(raw):
        raw["joints"][0]["limits_deg"] = {"min": 30.0, "max": -30.0}

    with pytest.raises(ConfigError):
        RobotModel.from_config(config_with(tmp_path, edit))
