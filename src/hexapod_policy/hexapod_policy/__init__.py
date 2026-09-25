"""hexapod_policy — eğitilmiş RL politikasını çalıştıran ROS 2 düğümü (GOREVLER.md G8).

mlp.py         numpy ile MLP çıkarımı + politikanın eğitim sözleşmesi (.npz)
controller.py  ROS'suz çekirdek: IMU + hız komutu -> gözlem -> eklem hedefi
node.py        ince rclpy kabuğu: /imu, /cmd_vel -> /leg_controller/commands

Politika dosyası `python -m hexapod_rl.export models/<ad>/model.zip` ile üretilir.
"""

from .controller import PolicyController, gravity_in_base
from .mlp import MlpPolicy, PolicyContract

__all__ = ["MlpPolicy", "PolicyContract", "PolicyController", "gravity_in_base"]

__version__ = "0.1.0"
