"""ros2_control kontrolcü ayarları (gz_ros2_control'ün okuduğu YAML).

Elle yazılmış bir YAML yerine üretiliyor: eklem listesi ve sırası
interface.joint_names()'den gelsin, komut dizisinin sırası hiçbir yerde
ayrışmasın. Kontrol döngüsü hızı robot.yaml'dan (simulation.control).

Kontrolcüler:
  joint_state_broadcaster   /joint_states yayınlar
  leg_controller            ForwardCommandController, konum; ~/commands
                            (= interface.COMMAND_TOPIC) Float64MultiArray

Not: ros2_controllers'ın position_controllers paketi ROS 2 Lyrical'da yok;
aynı işi forward_command_controller + interface_name: position yapıyor.
"""

from __future__ import annotations

from pathlib import Path

import yaml

from .interface import CONTROLLER_NAME, joint_names
from .model import RobotModel


def controllers_config(model: RobotModel) -> dict:
    return {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": int(round(model.control_rate)),
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster",
                },
                CONTROLLER_NAME: {
                    "type": "forward_command_controller/ForwardCommandController",
                },
            },
        },
        CONTROLLER_NAME: {
            "ros__parameters": {
                "joints": joint_names(model.mounts),
                "interface_name": "position",
            },
        },
    }


def write_controllers_yaml(model: RobotModel, path: Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ("# ÜRETİLDİ: hexapod_description.control, config/robot.yaml'dan. "
              "Elle düzenlemeyin.\n")
    path.write_text(header + yaml.safe_dump(controllers_config(model), sort_keys=False),
                    encoding="utf-8", newline="\n")
    return path
