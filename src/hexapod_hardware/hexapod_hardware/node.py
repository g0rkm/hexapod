"""Gerçek robot sürücü düğümü (GOREVLER.md S4).

    ros2 run hexapod_hardware driver --ros-args -p dry_run:=true
    ros2 run hexapod_hardware driver                       # gerçek donanım (Pi)
    ros2 run hexapod_hardware driver --ros-args -p dry_run:=true \\
        -p config:=/yol/robot.yaml -p calibration:=/yol/calibration.yaml

Simülasyondaki ros2_control kontrolcüsünün yerine geçer: aynı konuyu dinler
(hexapod_description.interface.COMMAND_TOPIC, docs/ARAYUZ.md), servoyu sürer,
/joint_states yayınlar. Asıl mantık controller.py'de (ROS'suz, testli).

Kablolama (robot.yaml: kart adresleri, kanallar, limitler) ve kalibrasyon
eksikse düğüm başlamaz ve neyin eksik olduğunu listeler; varsayılan
uydurulmaz. Düğüm kapanınca bütün servolar serbest bırakılır (tork kesilir).
"""

from __future__ import annotations

import sys

from .controller import DriverController


def main(argv: list[str] | None = None) -> int:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    from hexapod_description.interface import COMMAND_RATE_HZ, COMMAND_TOPIC, STATE_TOPIC
    from hexapod_driver import Calibration, HexapodError, RobotConfig, ServoBus

    class StartupError(Exception):
        """Düğüm kurulamadı (eksik kablolama, kart yok...). Mesaj kullanıcıya gösterilir."""

    class DriverNode(Node):
        def __init__(self) -> None:
            super().__init__("hexapod_hardware")
            self.declare_parameter("dry_run", False)
            self.declare_parameter("config", "")
            self.declare_parameter("calibration", "")
            dry_run = bool(self.get_parameter("dry_run").value)
            config_path = str(self.get_parameter("config").value) or None
            calibration_path = str(self.get_parameter("calibration").value) or None

            self._controller = self._build_controller(config_path, calibration_path, dry_run)

            self._sub = self.create_subscription(
                Float64MultiArray, COMMAND_TOPIC, self._on_command, 10)
            self._pub = self.create_publisher(JointState, STATE_TOPIC, 10)
            self._timer = self.create_timer(1.0 / COMMAND_RATE_HZ, self._publish_state)
            mode = "DRY-RUN (donanıma yazılmıyor)" if dry_run else "GERÇEK DONANIM"
            self.get_logger().info(f"hexapod_hardware hazır [{mode}]: {COMMAND_TOPIC} dinleniyor")

        @staticmethod
        def _build_controller(config_path, calibration_path, dry_run) -> DriverController:
            try:
                config = RobotConfig.load(config_path)
                if not config.wiring_is_complete():
                    lines = ["kablolama bilgisi eksik; config/robot.yaml içinde şu alanlar "
                             "doldurulmalı:"]
                    for gap in config.wiring_gaps():
                        note = f"  ({gap.source})" if gap.source else ""
                        lines.append(f"  - {gap.path}{note}")
                    raise StartupError("\n".join(lines))
                calibration = Calibration.load(calibration_path)
                controller = DriverController(ServoBus(config, calibration, dry_run=dry_run))
                controller.start()
                return controller
            except HexapodError as exc:
                raise StartupError(str(exc)) from exc

        def shutdown(self) -> None:
            self._controller.stop()  # servolar serbest, tork kesilir

        def _on_command(self, msg: Float64MultiArray) -> None:
            result = self._controller.on_command(msg.data)
            if not result.accepted:
                self.get_logger().warning(
                    f"komut reddedildi, servoya gitmedi: {result.reason}",
                    throttle_duration_sec=2.0)

        def _publish_state(self) -> None:
            state = self._controller.joint_state()
            if state is None:
                return  # henüz komut yok; konum bilinmiyor, uydurulmaz
            names, positions = state
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = names
            msg.position = positions
            self._pub.publish(msg)

    rclpy.init(args=argv)
    try:
        node = DriverNode()
    except StartupError as exc:
        print(f"HATA: sürücü başlatılamadı: {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        return 2

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # Ctrl+C / SIGTERM rclpy bağlamını kapatır; spin bu sırada Lyrical'da
        # KeyboardInterrupt değil ExternalShutdownException ya da RCLError ile çıkar.
        # Bağlam kapandıysa bu normal çıkıştır, değilse gerçek hatadır.
        if rclpy.ok():
            raise
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
