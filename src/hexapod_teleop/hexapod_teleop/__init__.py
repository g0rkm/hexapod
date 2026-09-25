"""hexapod_teleop — hız komutundan tripod yürüyüşüne ROS 2 köprüsü (GOREVLER.md S3).

controller.py  ROS'suz çekirdek: hız komutu, zaman aşımı, hız sınırlama,
               ReachError yakalama; hexapod_gait.TripodGait'i sarar.
node.py        ince rclpy kabuğu: /cmd_vel dinler,
               hexapod_description.interface.COMMAND_TOPIC'e yayınlar.

Kullanım: ros2 run hexapod_teleop teleop
"""

from .controller import Command, TeleopController, TeleopLimits

__all__ = ["Command", "TeleopController", "TeleopLimits"]

__version__ = "0.1.0"
