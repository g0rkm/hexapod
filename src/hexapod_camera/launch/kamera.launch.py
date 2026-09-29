"""Kamera: Pi Camera Module V2 -> camera_ros -> tarayıcıda canlı görüntü.

    ros2 launch hexapod_camera kamera.launch.py                  # kamera takılıyken (Pi)
    ros2 launch hexapod_camera kamera.launch.py deneme:=true     # kamerasız: deneme deseni
    ros2 launch hexapod_camera kamera.launch.py port:=8081 width:=1280 height:=720

Sonra aynı Wi-Fi'deki telefonda ya da bilgisayarda tarayıcıyla:
http://<Pi'nin IP'si>:8080  (adres hexapod_camera'nın günlüğünde yazar).

Açılanlar:
  camera          camera_ros (libcamera) kamera sürücüsü; /camera/image_raw ve
                  /camera/image_raw/compressed (JPEG) yayınlar. deneme:=true'da açılmaz.
  hexapod_camera  JPEG kareleri tarayıcıya akıtır (hexapod_camera.node).

camera_ros Pi'de tools/pi/pi_kurulum.sh ile kurulur (ros-lyrical-camera-ros;
libcamera'sında Pi 4 ve IMX219 = Camera V2 desteği var, 2026-09-29'da paket
içinden doğrulandı). GERÇEK KAMERADA DENENMEDİ: kamera takılınca ilk deneme
donanım vardiyasının işi. Kamera bulunamazsa camera_ros hata verip çıkar;
robot.launch.py camera:=true ile açıldıysa robotun geri kalanı çalışmaya
devam eder.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("deneme", default_value="false",
                              description="kamera yerine hareketli deneme deseni (kamerasız)"),
        DeclareLaunchArgument("port", default_value="8080",
                              description="tarayıcı adresi http://<IP>:<port>"),
        DeclareLaunchArgument("width", default_value="640", description="görüntü genişliği"),
        DeclareLaunchArgument("height", default_value="480", description="görüntü yüksekliği"),
        DeclareLaunchArgument("jpeg_quality", default_value="80",
                              description="JPEG kalitesi (düşük = az Wi-Fi yükü)"),
    ]

    def num(name: str) -> ParameterValue:
        return ParameterValue(LaunchConfiguration(name), value_type=int)

    nodes = [
        Node(package="camera_ros", executable="camera_node", name="camera",
             output="screen", emulate_tty=True,
             condition=UnlessCondition(LaunchConfiguration("deneme")),
             parameters=[{"width": num("width"), "height": num("height"),
                          "jpeg_quality": num("jpeg_quality")}]),
        Node(package="hexapod_camera", executable="stream", name="hexapod_camera",
             output="screen", emulate_tty=True,
             parameters=[{"port": num("port"),
                          "test_pattern": ParameterValue(LaunchConfiguration("deneme"),
                                                         value_type=bool),
                          "width": num("width"), "height": num("height"),
                          "topic": "/camera/image_raw/compressed"}]),
    ]
    return LaunchDescription(args + nodes)
