"""Gerçek robotu başlat: sensörler + servo sürücü + politika (GOREVLER.md D9, D11).

    ros2 launch hexapod_bringup robot.launch.py                      # robotta
    ros2 launch hexapod_bringup robot.launch.py reflex:=true         # + kaldırma refleksi (D8 sonrası)
    ros2 launch hexapod_bringup robot.launch.py dry_run:=true \\
        config:=/yol/robot.yaml calibration:=/yol/calibration.yaml   # robotsuz deneme

Açılan düğümler (konular: docs/ARAYUZ.md):
  hexapod_sensors   /imu + /range0..2 yayınlar (BNO055, VL53L0X)
  hexapod_policy    /imu + /cmd_vel (+ /range*) -> /leg_controller/commands, 50 Hz
  hexapod_hardware  /leg_controller/commands -> servolar (PCA9685), /joint_states

Yürütmek için /cmd_vel gerekir: aynı ağda başka bir terminalden
`ros2 run teleop_twist_keyboard teleop_twist_keyboard` (ya da
`ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"`).
Komut yokken politika ayakta duruş yayınlar.

Değer uydurulmaz: kablolama, kalibrasyon, IMU montajı ve (reflex:=true ise)
mesafe sensörü yerleşimi robot.yaml / calibration.yaml'da yoksa ilgili düğüm
eksik alanı söyleyip çıkış kodu 2 ile çıkar. Düğümlerden biri çıkarsa bütün
sistem kapanır: yarım çalışan bir robot (ör. IMU'suz politika) bırakılmaz.
Sürücü kapanırken servoları serbest bırakır (robot çöker; bilerek).

DİKKAT (S4'ten devredilen risk, GOREVLER.md): ilk açılışta 18 servo aynı anda
ayakta duruşa gider, akım sıçrar. İlk denemeyi robot havada ve güç kaynağı
akım sınırlıyken yapın.

dry_run:=true: sürücü servoya yazmaz, sensör düğümü taklit cihazlarla koşar
(sabit IMU, 0.4 m mesafe). Robotsuz uçtan uca deneme için; config'te
kablolama yine dolu olmalı (testte uydurma değerli bir kopya kullanılır).

policy: verilmezse depodaki robota aday model (models/ppo_kaldirma35_250k).
Depo yolu bu dosyanın gerçek yerinden bulunur (colcon --symlink-install ile
depoya bağlıdır); bulunamazsa policy:= açıkça verilmeli.
"""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#: Robota aday model (PROJE_DEVIR §3.8), depo köküne göre.
DEFAULT_POLICY = Path("models") / "ppo_kaldirma35_250k" / "policy.npz"


def _default_policy() -> str:
    """Depodaki aday modelin yolu; bu dosya depodan çağrılmıyorsa boş."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / DEFAULT_POLICY
        if candidate.is_file():
            return str(candidate)
    return ""


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="servoya yazma, sensörleri taklit et (robotsuz deneme)"),
        DeclareLaunchArgument("reflex", default_value="false",
                              description="mesafe sensörlü kaldırma refleksi (yerleşim D8'de girilince)"),
        DeclareLaunchArgument("sensors", default_value="true",
                              description="sensör düğümünü aç (kapalıysa /imu başka yerden gelmeli)"),
        DeclareLaunchArgument("policy", default_value=_default_policy(),
                              description="politika dosyası (.npz, hexapod_rl.export)"),
        DeclareLaunchArgument("config", default_value="",
                              description="robot.yaml (boş: config/robot.yaml aranır)"),
        DeclareLaunchArgument("calibration", default_value="",
                              description="calibration.yaml (boş: config/calibration.yaml)"),
    ]

    def flag(name: str) -> ParameterValue:
        return ParameterValue(LaunchConfiguration(name), value_type=bool)

    def text(name: str) -> ParameterValue:
        return ParameterValue(LaunchConfiguration(name), value_type=str)

    # Biri çıkarsa hepsi kapansın (docstring).
    stop_all = [Shutdown(reason="bir düğüm çıktı; sistem kapatılıyor")]
    nodes = [
        Node(package="hexapod_sensors", executable="sensors", name="hexapod_sensors",
             output="screen", emulate_tty=True, on_exit=stop_all, condition=IfCondition(LaunchConfiguration("sensors")),
             parameters=[{"dry_run": flag("dry_run"), "config": text("config")}]),
        Node(package="hexapod_hardware", executable="driver", name="hexapod_hardware",
             output="screen", emulate_tty=True, on_exit=stop_all,
             parameters=[{"dry_run": flag("dry_run"), "config": text("config"),
                          "calibration": text("calibration")}]),
        Node(package="hexapod_policy", executable="policy", name="hexapod_policy",
             output="screen", emulate_tty=True, on_exit=stop_all,
             parameters=[{"policy": text("policy"), "config": text("config"),
                          "reflex": flag("reflex")}]),
    ]
    return LaunchDescription(args + nodes)
