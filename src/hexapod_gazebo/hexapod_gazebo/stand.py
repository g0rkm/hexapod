"""Robotu simülasyonda ayağa kaldıran komut (G5 kontrolü).

    ros2 run hexapod_gazebo stand
    ros2 run hexapod_gazebo stand --height 90 --reach 120

Eklem komut arayüzüne (hexapod_description.interface.COMMAND_TOPIC) bir
duruş gönderir. Varsayılanlar robot parametresi değil, yalnızca gösterim
için seçilmiş bir duruş (femur yataya yakın, gövde 100 mm yüksekte).
"""

from __future__ import annotations

import argparse
import sys

from hexapod_description.interface import COMMAND_TOPIC, to_command
from hexapod_driver.config import RobotConfig
from hexapod_kinematics import HexapodKinematics, ReachError

from .pose import standing_pose


def main(argv: list[str] | None = None) -> int:
    import rclpy
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import Float64MultiArray

    parser = argparse.ArgumentParser(description="Simülasyonda ayağa kalk")
    parser.add_argument("--height", type=float, default=100.0, help="gövde yüksekliği, mm")
    parser.add_argument("--reach", type=float, default=130.0,
                        help="ayağın coxa ekseninden yatay uzaklığı, mm")
    parser.add_argument("--seconds", type=float, default=2.0, help="kaç saniye yayınlansın")
    args = parser.parse_args(remove_ros_args(argv if argv is not None else sys.argv)[1:])

    kin = HexapodKinematics.from_config(RobotConfig.load())
    try:
        angles = standing_pose(kin, args.reach, args.height)
    except ReachError as exc:
        print(f"HATA: bu duruşa erişilemez: {exc}", file=sys.stderr)
        return 1
    msg = Float64MultiArray(data=to_command(angles))

    rclpy.init(args=argv)
    node = rclpy.create_node("hexapod_stand")
    pub = node.create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)
    period = 0.1
    remaining = [int(args.seconds / period)]

    def tick():
        pub.publish(msg)
        remaining[0] -= 1

    node.create_timer(period, tick)
    node.get_logger().info(
        f"duruş: gövde {args.height:g} mm, ayak {args.reach:g} mm -> {COMMAND_TOPIC}")
    try:
        while rclpy.ok() and remaining[0] > 0:
            rclpy.spin_once(node, timeout_sec=period)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
