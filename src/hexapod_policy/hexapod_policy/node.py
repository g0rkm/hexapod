"""Politika düğümü (GOREVLER.md G8).

    ros2 run hexapod_policy policy --ros-args -p policy:=/yol/models/ppo_omni_250k/policy.npz
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

Mesafe sensörlü kaldırma refleksi (-p reflex:=true; yalnız artık eylem
politikalarında): sensör yerleşimi robot.yaml -> sensors.range_finders'tan
(ranges.range_sensors_from_config); bilinmiyorsa düğüm başlamaz ve eksik
alanı söyler (çıkış kodu 2). /range<kimlik> (sensor_msgs/Range,
hexapod_sensors düğümü) dinlenir; her sensörden birer ölçüm gelince
denetleyiciye verilir (ranges.RangeCollector). Mesafe gelmezse ya da bayatsa
refleks devre dışı, politika kör davranışına döner; durum ("refleks: ...")
günlüğe yazılır.
"""

from __future__ import annotations

import sys

from .controller import PolicyController
from .lift_reflex import LiftReflex
from .mlp import MlpPolicy
from .ranges import RangeCollector, range_sensors_from_config


def _run(argv: list[str] | None = None) -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, Range
    from std_msgs.msg import Float64MultiArray

    from hexapod_description.interface import (
        COMMAND_RATE_HZ, COMMAND_TOPIC, IMU_TOPIC, joint_names)
    from hexapod_description.model import RobotModel
    from hexapod_driver import HexapodError, RobotConfig
    from hexapod_kinematics import HexapodKinematics

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
            self.declare_parameter("reflex", False)
            path = str(self.get_parameter("policy").value)
            config_path = str(self.get_parameter("config").value) or None
            topic = str(self.get_parameter("cmd_vel_topic").value)
            reflex = bool(self.get_parameter("reflex").value)

            self._controller, range_ids = self._build(
                path, config_path, float(self.get_parameter("cmd_timeout_s").value),
                float(self.get_parameter("imu_timeout_s").value), reflex)
            c = self._controller.policy.contract
            if abs(c.control_hz - COMMAND_RATE_HZ) > 1e-9:
                raise StartupError(f"politika {c.control_hz:g} Hz için eğitilmiş, "
                                   f"komut hızı {COMMAND_RATE_HZ:g} Hz")

            self._pub = self.create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)
            self._subs = [self.create_subscription(Twist, topic, self._on_twist, 10),
                          self.create_subscription(Imu, IMU_TOPIC, self._on_imu, 10)]
            self._ranges = None
            if range_ids:
                self._ranges = RangeCollector(self._controller.range_sensors)
                self._subs += [self.create_subscription(
                    Range, f"/range{i}", lambda msg, k=k: self._on_range(k, msg), 10)
                    for k, i in enumerate(range_ids)]
            self._timer = self.create_timer(1.0 / COMMAND_RATE_HZ, self._tick)
            self._last_status = None
            self._last_reflex = None
            ranges = (" + " + ", ".join(f"/range{i}" for i in range_ids) + " (refleks "
                      f"'{self._controller.reflex.reference}')") if range_ids else ""
            self.get_logger().info(
                f"hexapod_policy hazır: {path} | {topic} + {IMU_TOPIC}{ranges} -> "
                f"{COMMAND_TOPIC} | eğitim aralığı vx {c.command_ranges['vx']} | "
                f"eylem modu {c.action_mode}")

        @staticmethod
        def _build(path, config_path, cmd_timeout, imu_timeout,
                   reflex: bool) -> tuple[PolicyController, list[int]]:
            if not path:
                raise StartupError("politika dosyası verilmedi: -p policy:=/yol/policy.npz "
                                   "(üretmek için: python -m hexapod_rl.export <model.zip>)")
            try:
                policy = MlpPolicy.load(path)
            except (OSError, ValueError, KeyError) as exc:
                raise StartupError(f"politika okunamadı ({path}): {exc}") from exc
            try:
                config = RobotConfig.load(config_path)
                model = RobotModel.from_config(config)
                kin = HexapodKinematics.from_config(config)
                placed = range_sensors_from_config(config) if reflex else []
            except HexapodError as exc:
                raise StartupError(str(exc)) from exc
            except ValueError as exc:
                raise StartupError(f"mesafe sensörü yerleşimi geçersiz: {exc}") from exc
            limits = [(model.limits[(int(n[3]), n.split("_")[1])].lower,
                       model.limits[(int(n[3]), n.split("_")[1])].upper)
                      for n in joint_names(model.mounts)]
            kw = (dict(range_sensors=[s for _, s in placed], reflex=LiftReflex())
                  if placed else {})
            try:
                controller = PolicyController(policy, limits, cmd_timeout, imu_timeout,
                                              kin=kin, **kw)
            except ValueError as exc:
                raise StartupError(f"politika bu robota uymuyor: {exc}") from exc
            return controller, [i for i, _ in placed]

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

        def _on_twist(self, msg: Twist) -> None:
            self._controller.on_command(msg.linear.x, msg.linear.y, msg.angular.z, self._now())

        def _on_imu(self, msg: Imu) -> None:
            q, w = msg.orientation, msg.angular_velocity
            self._controller.on_imu((q.w, q.x, q.y, q.z), (w.x, w.y, w.z), self._now())

        def _on_range(self, k: int, msg: Range) -> None:
            full = self._ranges.add(k, msg.range, msg.max_range)
            if full is not None:
                self._controller.on_ranges(full, self._now())

        def _tick(self) -> None:
            targets = self._controller.tick(self._now())
            self._pub.publish(Float64MultiArray(data=targets))
            status = self._controller.status
            if status != self._last_status:
                self.get_logger().info(f"durum: {status}")
                self._last_status = status
            reflex = self._controller.reflex_status
            if reflex is not None and reflex != self._last_reflex:
                self.get_logger().info(f"refleks: {reflex}")
                self._last_reflex = reflex
            if self._controller.clipped_command:
                self.get_logger().warning(
                    "hız komutu politikanın eğitim aralığına kırpıldı", throttle_duration_sec=5.0)

    # rclpy'nin sinyal işleyicisi kapalı: Ctrl+C / SIGTERM'i main (run_node)
    # karşılar; ikinci sinyal kapanışı yarıda kesmesin (hexapod_driver.stop_signals).
    from rclpy.signals import SignalHandlerOptions

    try:
        rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
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


def main(argv: list[str] | None = None) -> int:
    from hexapod_driver.stop_signals import run_node
    return run_node(_run, argv)


if __name__ == "__main__":
    raise SystemExit(main())
