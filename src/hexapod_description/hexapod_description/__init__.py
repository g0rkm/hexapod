"""hexapod_description — robotun simülasyon modeli (URDF'in girdisi).

Katman sırası:
    Gazebo / RL ortamı          (ileride)
    Simülasyon modeli           <- burası
    Ters kinematik (hexapod_kinematics)

Her şey config/robot.yaml'dan gelir; geometri ve bacak yerleşimi
hexapod_kinematics'ten alınır ki simülasyon ile IK ayrışmasın.
"""

from .model import (
    LINK_NAMES,
    CollisionBox,
    Inertial,
    JointLimit,
    LinkModel,
    RobotModel,
)

__all__ = [
    "CollisionBox",
    "Inertial",
    "JointLimit",
    "LINK_NAMES",
    "LinkModel",
    "RobotModel",
]

__version__ = "0.1.0"
