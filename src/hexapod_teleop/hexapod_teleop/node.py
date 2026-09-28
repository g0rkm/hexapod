"""hexapod_teleop ROS 2 düğümü (GOREVLER.md S3).

    ros2 run hexapod_teleop teleop
    ros2 run hexapod_teleop teleop --ros-args -p cmd_vel_topic:=/cmd_vel -p cycle_hz:=1.5

/cmd_vel (geometry_msgs/Twist) dinler, hexapod_gait.TripodGait ile eklem
açısı üretip hexapod_description.interface.COMMAND_TOPIC'e (docs/ARAYUZ.md)
yayınlar. Asıl mantık controller.py'de (ROS'suz, testli); bu dosya yalnızca
subscribe/publish/timer bağlıyor — hexapod_gazebo/stand.py ile aynı desen.
"""

from __future__ import annotations

from .controller import TeleopController, TeleopLimits


def _run(argv: list[str] | None = None) -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray

    from hexapod_description.interface import COMMAND_RATE_HZ, COMMAND_TOPIC
    from hexapod_driver.config import RobotConfig
    from hexapod_gait import GaitParams
    from hexapod_kinematics import HexapodKinematics

    class TeleopNode(Node):
        def __init__(self) -> None:
            super().__init__("hexapod_teleop")
            self.declare_parameter("cmd_vel_topic", "/cmd_vel")
            self.declare_parameter("cmd_timeout_s", 0.5)
            self.declare_parameter("cycle_hz", 1.5)

            topic = str(self.get_parameter("cmd_vel_topic").value)
            timeout = float(self.get_parameter("cmd_timeout_s").value)
            cycle_hz = float(self.get_parameter("cycle_hz").value)

            kin = HexapodKinematics.from_config(RobotConfig.load())
            self._controller = TeleopController(
                kin, GaitParams(cycle_hz=cycle_hz), TeleopLimits(), timeout_s=timeout)

            self._pub = self.create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)
            self._sub = self.create_subscription(Twist, topic, self._on_twist, 10)
            self._last_tick = self._now()
            self._timer = self.create_timer(1.0 / COMMAND_RATE_HZ, self._tick)
            self.get_logger().info(
                f"hexapod_teleop hazır: {topic} dinleniyor -> {COMMAND_TOPIC} "
                f"({cycle_hz:g} Hz adım, {timeout:g} sn zaman aşımı)")

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

        def _on_twist(self, msg: Twist) -> None:
            self._controller.on_command(msg.linear.x, msg.linear.y, msg.angular.z, self._now())

        def _tick(self) -> None:
            now = self._now()
            dt = now - self._last_tick
            self._last_tick = now
            if dt <= 0.0:
                return  # sim saati sıfırlandı/durdu, bekle
            command = self._controller.tick(now, dt)
            if command is not None:
                self._pub.publish(Float64MultiArray(data=command))

    # rclpy'nin sinyal işleyicisi kapalı: Ctrl+C / SIGTERM'i main (run_node)
    # karşılar; ikinci sinyal kapanışı yarıda kesmesin (hexapod_driver.stop_signals).
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = TeleopNode()
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
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def main(argv: list[str] | None = None) -> int:
    from hexapod_driver.stop_signals import run_node
    return run_node(_run, argv)


if __name__ == "__main__":
    raise SystemExit(main())
