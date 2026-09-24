"""Eklem komut arayüzü + Gazebo ekleri + kontrolcü ayarı. ROS gerektirmez.

Arayüz dört tarafın (simülasyon, gerçek sürücü, tripod, politika) ortak
sözleşmesi; buradaki testler sıranın, birimin ve adların her yerde aynı
olduğunu garanti eder.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from hexapod_description import RobotModel
from hexapod_description.control import controllers_config, write_controllers_yaml
from hexapod_description.interface import (
    COMMAND_TOPIC,
    CONTROLLER_NAME,
    IMU_FRAME,
    IMU_TOPIC,
    foot_contact_topic,
    from_command,
    joint_names,
    to_command,
)
from hexapod_description.urdf import GazeboOptions, build_urdf, foot_collision_name
from hexapod_driver import RobotConfig
from hexapod_driver.errors import ConfigError
from hexapod_kinematics import JointAngles

REAL_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"


@pytest.fixture(scope="module")
def model() -> RobotModel:
    return RobotModel.from_config(RobotConfig.load(REAL_CONFIG))


@pytest.fixture(scope="module")
def gz_urdf(model) -> ET.Element:
    return ET.fromstring(build_urdf(model, gazebo=GazeboOptions("/tmp/controllers.yaml")))


# --- arayüz -----------------------------------------------------------------


def test_eklem_sirasi_bacak_bacak_coxa_femur_tibia():
    names = joint_names()
    assert len(names) == 18
    assert names[:4] == ["leg0_coxa_joint", "leg0_femur_joint", "leg0_tibia_joint",
                         "leg1_coxa_joint"]
    assert names[-1] == "leg5_tibia_joint"


def test_komut_konusu_kontrolcu_adindan():
    # ForwardCommandController ~/commands açar; ad değişirse konu da değişmeli.
    assert COMMAND_TOPIC == f"/{CONTROLLER_NAME}/commands"


def test_derece_radyan_gidis_donus():
    angles = {leg: JointAngles(10.0 * leg, -20.0 + leg, 30.5 - leg) for leg in range(6)}
    data = to_command(angles)
    assert len(data) == 18
    assert data[3] == pytest.approx(math.radians(10.0))  # bacak 1 coxa
    back = from_command(data)
    for leg in range(6):
        assert back[leg].as_dict() == pytest.approx(angles[leg].as_dict())


def test_eksik_bacak_sessizce_sifirlanmaz():
    with pytest.raises(ValueError):
        to_command({leg: JointAngles(0, 0, 0) for leg in range(5)})


def test_bozuk_komut_reddedilir():
    with pytest.raises(ValueError):
        from_command([0.0] * 17)
    with pytest.raises(ValueError):
        from_command([0.0] * 17 + [float("nan")])


# --- Gazebo ekleri ------------------------------------------------------------


def test_ros2_control_eklem_sirasi_arayuzle_ayni(model, gz_urdf):
    rc = gz_urdf.find("ros2_control")
    assert rc.find("hardware/plugin").text == "gz_ros2_control/GazeboSimSystem"
    assert [j.get("name") for j in rc.findall("joint")] == joint_names()
    for j in rc.findall("joint"):
        leg, part = int(j.get("name")[3]), j.get("name").split("_")[1]
        lim = model.limits[(leg, part)]
        cmd = j.find("command_interface[@name='position']")
        assert float(cmd.find("param[@name='min']").text) == pytest.approx(lim.lower)
        assert float(cmd.find("param[@name='max']").text) == pytest.approx(lim.upper)


def test_konum_kazanci_zaman_sabitinden(model, gz_urdf):
    plugin = gz_urdf.find("gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
    assert plugin.find("parameters").text == "/tmp/controllers.yaml"
    gain = float(plugin.find("position_proportional_gain").text)
    # T = 1 / (kazanç x hız)
    assert 1.0 / (gain * model.control_rate) == pytest.approx(model.servo_time_constant)
    assert 0.0 < gain <= 1.0


def test_tutarsiz_zaman_sabiti_reddedilir(model):
    bad = RobotModel(**{**model.__dict__, "servo_time_constant": 0.001})  # T x hız = 0.1
    with pytest.raises(ConfigError):
        bad.position_gain()


def test_imu_sensoru(gz_urdf):
    sensor = gz_urdf.find(f"gazebo[@reference='{IMU_FRAME}']/sensor[@type='imu']")
    assert sensor.find("topic").text == IMU_TOPIC
    assert sensor.find("gz_frame_id").text == IMU_FRAME


def test_temas_sensoru_ayak_kuresine_bagli(gz_urdf):
    """sdformat çarpışmayı '<ad>_collision' (+ sıra 0 değilse '_<sıra>') diye adlandırır.
    Sensör o adı kullanıyor; ayak küresi tibia'nın İLK çarpışması olmalı."""
    for leg in range(6):
        tibia = gz_urdf.find(f"link[@name='leg{leg}_tibia']")
        first = tibia.findall("collision")[0]
        assert first.get("name") == foot_collision_name(leg)
        assert first.find("geometry/sphere") is not None
        assert first.get("name").startswith(f"leg{leg}_tibia")  # sdformat link adını bekler
        sensor = gz_urdf.find(f"gazebo[@reference='leg{leg}_tibia']/sensor[@type='contact']")
        assert sensor.find("contact/collision").text == foot_collision_name(leg) + "_collision"
        assert sensor.find("contact/topic").text == foot_contact_topic(leg)


def test_gazebo_ekleri_istenmezse_yok(model):
    root = ET.fromstring(build_urdf(model))
    assert root.find("ros2_control") is None
    assert root.find("gazebo") is None


# --- kontrolcü ayarı ------------------------------------------------------------


def test_kontrolcu_ayari(model, tmp_path):
    cfg = controllers_config(model)
    cm = cfg["controller_manager"]["ros__parameters"]
    assert cm["update_rate"] == round(model.control_rate)
    assert cm[CONTROLLER_NAME]["type"] == "forward_command_controller/ForwardCommandController"
    params = cfg[CONTROLLER_NAME]["ros__parameters"]
    assert params["joints"] == joint_names()
    assert params["interface_name"] == "position"
    path = write_controllers_yaml(model, tmp_path / "c.yaml")
    assert yaml.safe_load(path.read_text(encoding="utf-8")) == cfg
