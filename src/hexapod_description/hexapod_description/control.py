"""ros2_control kontrolcü ayarları (gz_ros2_control'ün okuduğu YAML).

Elle yazılmış bir YAML yerine üretiliyor: eklem listesi ve sırası
interface.joint_names()'den gelsin, komut dizisinin sırası hiçbir yerde
ayrışmasın. Kontrol döngüsü hızı robot.yaml'dan (simulation.control).

Kontrolcüler:
  joint_state_broadcaster   /joint_states yayınlar
  leg_controller            ForwardCommandController, konum; ~/commands
                            (= interface.COMMAND_TOPIC) Float64MultiArray
  servo_controller          (yalnız tork modeli) pid_controller: leg_controller
                            ona zincirlenir; hedef konumdan eklem EFORU üretir

Tork modeli (servo="torque", urdf.GazeboOptions): leg_controller'ın yazdığı
hedef, servo_controller'ın referansıdır (zincirleme: "servo_controller/<eklem>/
position"). PID yalnız P (sertlik), çıkış durma torkuyla sınırlı; sönüm URDF'te
eklem sönümü. Döngü TORQUE_LOOP_HZ'de: hexapod_rl.sim modeli her fizik adımında
hesaplıyor, burada da öyle (ROS'lu simin fizik adımı 1 ms). Komut konusu ve
sırası değişmez; yayınlayan taraf (teleop, politika) farkı görmez.

Not: ros2_controllers'ın position_controllers paketi ROS 2 Lyrical'da yok;
aynı işi forward_command_controller + interface_name: position yapıyor.
"""

from __future__ import annotations

from pathlib import Path

import yaml

from .interface import CONTROLLER_NAME, joint_names
from .model import RobotModel

#: Tork modelinde servo döngüsü (ROS'lu simin fizik adımı 1 ms; modül açıklaması).
TORQUE_LOOP_HZ = 1000
#: Tork modelinde hedef konumdan efor üreten pid_controller'ın adı.
SERVO_CONTROLLER = "servo_controller"


def controllers_config(model: RobotModel, servo: str = "torque") -> dict:
    if servo == "torque":
        return _torque_config(model)
    if servo != "velocity":
        raise ValueError(f"servo modeli 'torque' ya da 'velocity' olmalı, {servo!r} verildi")
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


def _torque_config(model: RobotModel) -> dict:
    joints = joint_names(model.mounts)
    gains = {j: {"p": float(model.servo_stiffness), "i": 0.0, "d": 0.0,
                 "u_clamp_max": float(model.effort), "u_clamp_min": -float(model.effort)}
             for j in joints}
    return {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": TORQUE_LOOP_HZ,
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster",
                },
                SERVO_CONTROLLER: {"type": "pid_controller/PidController"},
                CONTROLLER_NAME: {
                    "type": "forward_command_controller/ForwardCommandController",
                },
            },
        },
        SERVO_CONTROLLER: {
            "ros__parameters": {
                "dof_names": joints,
                "command_interface": "effort",
                "reference_and_state_interfaces": ["position"],
                # komut gelene kadar doğduğu duruşu tutsun (NaN referansla boşta kalmasın)
                "set_current_state_as_first_setpoint": True,
                "gains": gains,
            },
        },
        CONTROLLER_NAME: {
            "ros__parameters": {
                "joints": [f"{SERVO_CONTROLLER}/{j}" for j in joints],
                "interface_name": "position",
            },
        },
    }


def write_controllers_yaml(model: RobotModel, path: Path, servo: str = "torque") -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ("# ÜRETİLDİ: hexapod_description.control, config/robot.yaml'dan. "
              "Elle düzenlemeyin.\n")
    path.write_text(header + yaml.safe_dump(controllers_config(model, servo), sort_keys=False),
                    encoding="utf-8", newline="\n")
    return path
