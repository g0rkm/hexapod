"""RViz'de robot modeli: URDF + eklem kaydırıcıları.

    ros2 launch hexapod_description display.launch.py
    ros2 launch hexapod_description display.launch.py meshes:=false

URDF her açılışta config/robot.yaml'dan üretilir (dosya olarak tutulmaz).
config/ klasörü bulunamazsa HEXAPOD_CONFIG_DIR ortam değişkenini ayarlayın.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hexapod_description.model import RobotModel
from hexapod_description.urdf import MeshSet, build_urdf
from hexapod_driver.config import RobotConfig


def _nodes(context):
    share = get_package_share_directory("hexapod_description")
    use_meshes = LaunchConfiguration("meshes").perform(context).lower() in ("true", "1")
    meshes = None
    if use_meshes:
        if os.path.isdir(os.path.join(share, "meshes")):
            meshes = MeshSet.load("package://hexapod_description/meshes/")
        else:
            print("[display] UYARI: pakette meshes/ yok, kutularla çiziliyor. "
                  "Önce: python tools/cad_sim_model.py --copy-meshes, sonra colcon build.")
    urdf = build_urdf(RobotModel.from_config(RobotConfig.load()), meshes)

    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", os.path.join(share, "rviz", "display.rviz")]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("meshes", default_value="true",
                              description="CAD mesh'leri (true) ya da çarpışma kutuları (false)"),
        OpaqueFunction(function=_nodes),
    ])
