"""Politika düğümü (GOREVLER.md G8).

    ros2 run hexapod_policy policy --ros-args -p policy:=/yol/models/ppo_v4_4M/policy.npz
    ros2 run hexapod_policy policy --ros-args -p policy:=... -p use_sim_time:=true   # simde

/imu (sensor_msgs/Imu) ve /cmd_vel (geometry_msgs/Twist) dinler, eğitilmiş
politikayı numpy ile çalıştırıp hexapod_description.interface.COMMAND_TOPIC'e
(docs/ARAYUZ.md) 50 Hz'de yayınlar. Asıl mantık controller.py'de (ROS'suz,
testli); bu dosya yalnızca subscribe/publish/timer bağlıyor.

Politika dosyası (.npz) `python -m hexapod_rl.export` ile üretilir; yoksa
düğüm başlamaz (çıkış kodu 2). Eklem limitleri robot.yaml'dan (kalibrasyon
limitleri, yoksa geçici limitler); politikanın hedefi bunlara kırpılır.
Durum değişimleri (yürüyor / dur / komut zaman aşımı / IMU yok / devrildi)
günlüğe yazılır.
"""

from __future__ import annotations

import sys

from .controller import PolicyController
from .mlp import MlpPolicy


def main(argv: list[str] | None = None) -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float64MultiArray

    from hexapod_description.interface import (
        COMMAND_RATE_HZ, COMMAND_TOPIC, IMU_TOPIC, joint_names)
    from hexapod_description.model import RobotModel
    from hexapod_driver import HexapodError, RobotConfig

    class StartupError(Exception):
        """Düğüm kurulamadı (politika dosyası yok/bozuk, config eksik)."""

    class PolicyNode(Node):
        def __init__(self) -> None:
            super().__init__("hexapod_policy")
            self.declare_parameter("policy", "")
            self.declare_parameter("config", "")
            self.declare_parameter("cmd_vel_topic", "/cmd_vel")
            self.declare_parameter("cmd_timeout_s", 0.5)
            self.declare_parameter("imu_timeout_s", 0.2)
            path = str(self.get_parameter("policy").value)
            config_path = str(self.get_parameter("config").value) or None
            topic = str(self.get_parameter("cmd_vel_topic").value)

            self._controller = self._build(path, config_path,
                                           float(self.get_parameter("cmd_timeout_s").value),
                                           float(self.get_parameter("imu_timeout_s").value))
            c = self._controller.policy.contract
            if abs(c.control_hz - COMMAND_RATE_HZ) > 1e-9:
                raise StartupError(f"politika {c.control_hz:g} Hz için eğitilmiş, "
                                   f"komut hızı {COMMAND_RATE_HZ:g} Hz")

            self._pub = self.create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)
            self._subs = [self.create_subscription(Twist, topic, self._on_twist, 10),
                          self.create_subscription(Imu, IMU_TOPIC, self._on_imu, 10)]
            self._timer = self.create_timer(1.0 / COMMAND_RATE_HZ, self._tick)
            self._last_status = None
            self.get_logger().info(
                f"hexapod_policy hazır: {path} | {topic} + {IMU_TOPIC} -> {COMMAND_TOPIC} | "
                f"eğitim aralığı vx {c.command_ranges['vx']}")

        @staticmethod
        def _build(path, config_path, cmd_timeout, imu_timeout) -> PolicyController:
            if not path:
                raise StartupError("politika dosyası verilmedi: -p policy:=/yol/policy.npz "
                                   "(üretmek için: python -m hexapod_rl.export <model.zip>)")
            try:
                policy = MlpPolicy.load(path)
            except (OSError, ValueError, KeyError) as exc:
                raise StartupError(f"politika okunamadı ({path}): {exc}") from exc
            try:
                model = RobotModel.from_config(RobotConfig.load(config_path))
            except HexapodError as exc:
                raise StartupError(str(exc)) from exc
            limits = [(model.limits[(int(n[3]), n.split("_")[1])].lower,
                       model.limits[(int(n[3]), n.split("_")[1])].upper)
                      for n in joint_names(model.mounts)]
            return PolicyController(policy, limits, cmd_timeout, imu_timeout)

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

        def _on_twist(self, msg: Twist) -> None:
            self._controller.on_command(msg.linear.x, msg.linear.y, msg.angular.z, self._now())

        def _on_imu(self, msg: Imu) -> None:
            q, w = msg.orientation, msg.angular_velocity
            self._controller.on_imu((q.w, q.x, q.y, q.z), (w.x, w.y, w.z), self._now())

        def _tick(self) -> None:
            targets = self._controller.tick(self._now())
            self._pub.publish(Float64MultiArray(data=targets))
            status = self._controller.status
            if status != self._last_status:
                self.get_logger().info(f"durum: {status}")
                self._last_status = status
            if self._controller.clipped_command:
                self.get_logger().warning(
                    "hız komutu politikanın eğitim aralığına kırpıldı", throttle_duration_sec=5.0)

    rclpy.init(args=argv)
    try:
        node = PolicyNode()
    except StartupError as exc:
        print(f"HATA: politika düğümü başlatılamadı: {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        return 2

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # Ctrl+C / SIGTERM rclpy bağlamını kapatır; spin bu sırada Lyrical'da
        # KeyboardInterrupt değil ExternalShutdownException ya da RCLError ile çıkar
        # (PROJE_DEVIR §12.19). Bağlam kapandıysa normal çıkış, değilse gerçek hata.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
