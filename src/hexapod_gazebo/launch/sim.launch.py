"""Hexapod'u Gazebo'da başlat (GOREVLER.md G5).

    ros2 launch hexapod_gazebo sim.launch.py
    ros2 launch hexapod_gazebo sim.launch.py gui:=false        # pencere yok (RL, test)
    ros2 launch hexapod_gazebo sim.launch.py world:=/yol/dunya.sdf

Sonra ayağa kaldırmak için:  ros2 run hexapod_gazebo stand

Akış: robot.yaml -> RobotModel -> kontrolcü YAML'ı + Gazebo ekli URDF ->
Gazebo -> robotu doğur -> joint_state_broadcaster -> leg_controller.
Konular hexapod_description.interface'te (belge: docs/ARAYUZ.md).
"""

import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hexapod_description.control import write_controllers_yaml
from hexapod_description.interface import CONTROLLER_NAME, IMU_TOPIC, foot_contact_topic
from hexapod_description.model import RobotModel
from hexapod_description.urdf import GazeboOptions, MeshSet, build_urdf
from hexapod_driver.config import RobotConfig

# Doğarken ayaklar yerin bu kadar üstünde (sıfır duruşunda); robot düşüp oturur.
_SPAWN_CLEARANCE_M = 0.01


def _setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    gui = arg("gui").lower() in ("true", "1")
    use_meshes = arg("meshes").lower() in ("true", "1")

    desc_share = get_package_share_directory("hexapod_description")
    model = RobotModel.from_config(RobotConfig.load())

    controllers = write_controllers_yaml(
        model, Path(tempfile.gettempdir()) / "hexapod" / "controllers.yaml")
    meshes = None
    if use_meshes and os.path.isdir(os.path.join(desc_share, "meshes")):
        meshes = MeshSet.load("package://hexapod_description/meshes/")
    urdf = build_urdf(model, meshes, GazeboOptions(str(controllers)))

    world = arg("world")
    if not os.path.isabs(world):
        world = os.path.join(get_package_share_directory("hexapod_gazebo"), "worlds", world)
    gz_args = f"-r -v 1 {world}" + ("" if gui else " -s --headless-rendering")

    # Sıfır duruşunda ayak ucu gövde çerçevesinde z = montaj z - tibia.
    feet_z = min(m.z for m in model.mounts.values()) - model.tibia
    spawn_z = -feet_z + _SPAWN_CLEARANCE_M

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": urdf, "use_sim_time": True}], output="screen",
    )
    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", model.name,
                   "-z", f"{spawn_z:.4f}"],
    )
    bridges = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
               f"{IMU_TOPIC}@sensor_msgs/msg/Imu[gz.msgs.IMU"]
    bridges += [f"{foot_contact_topic(leg)}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
                for leg in sorted(model.mounts)]
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                  arguments=bridges, output="screen")

    def spawner(name):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "--param-file", str(controllers)], output="screen")

    broadcaster = spawner("joint_state_broadcaster")
    legs = spawner(CONTROLLER_NAME)

    return [
        gz, state_publisher, spawn, bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=[legs])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="flat.sdf",
                              description="dünya dosyası (adı ya da mutlak yolu)"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="Gazebo penceresi (false: yalnız sunucu)"),
        DeclareLaunchArgument("meshes", default_value="true",
                              description="CAD mesh'leri (true) ya da kutular (false)"),
        OpaqueFunction(function=_setup),
    ])
