"""Eklem komut arayüzü: simülasyon, gerçek sürücü, yürüyüş ve politika
düğümlerinin ortak sözleşmesi (GOREVLER.md G5; belge: docs/ARAYUZ.md).

Aynı konuyu dört taraf konuşur:
  - Gazebo'daki ros2_control kontrolcüsü (G5)      komutu dinler
  - gerçek robot sürücü düğümü (S4)                 komutu dinler
  - tripod yürüyüş düğümü (S3)                      komut yayınlar
  - RL politika düğümü (G8)                         komut yayınlar
Simülasyondan robota geçişte yalnızca dinleyen taraf değişir; yayınlayan
kod aynı kalır. Bu dosya değişirse dört taraf da etkilenir.

Birimler: ROS tarafında RADYAN (REP-103). hexapod_kinematics DERECE
kullanır; dönüşüm yalnızca bu dosyadaki yardımcılarla yapılır.
İşaretler IK ↔ kalibrasyon sözleşmesiyle aynı (CLAUDE.md): femur +
bacağı kaldırır, tibia + ayağı dışarı götürür, coxa + yukarıdan bakınca
saat yönünün tersi.

Saf Python; ROS'a bağımlı değil (mesaj tipleri yalnızca adıyla geçiyor).
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence

from hexapod_kinematics import JointAngles

JOINT_PARTS = ("coxa", "femur", "tibia")

# --- Konular -----------------------------------------------------------------

#: Eklem hedefleri. std_msgs/msg/Float64MultiArray, 18 değer, radyan,
#: joint_names() sırasıyla. ros2_control ForwardCommandController'ın
#: "leg_controller" adıyla açtığı ~/commands konusu.
COMMAND_TOPIC = "/leg_controller/commands"
COMMAND_TYPE = "std_msgs/msg/Float64MultiArray"
CONTROLLER_NAME = "leg_controller"

#: Eklem durumu. sensor_msgs/msg/JointState (ad listesiyle, sıra garanti değil).
#: DİKKAT: MG996R konum geri bildirimi vermez. Gerçek robotta bu konu ÖLÇÜM
#: değil, son gönderilen komuttur. Politika gözlemini buna göre tasarla
#: (simülasyonda da ölçülen değil komut edilen açıyı gözle ya da gürültü ekle).
STATE_TOPIC = "/joint_states"

#: IMU. sensor_msgs/msg/Imu, çerçeve imu_link (base_link ile aynı yönelim).
#: Gerçek sürücü, BNO055'in ham verisini montaj yönelimine
#: (robot.yaml sensors.imu.mount_rotation_deg) göre bu çerçeveye döndürür.
IMU_TOPIC = "/imu"
IMU_FRAME = "imu_link"

#: Komut hızı. MG996R 50 Hz PWM ile sürülür; daha sık komutun anlamı yok.
COMMAND_RATE_HZ = 50.0


def foot_contact_topic(leg: int) -> str:
    """Ayak teması, YALNIZCA simülasyonda (ros_gz_interfaces/msg/Contacts).

    Gerçek robotta ayak temas sensörü yok. RL politikası bunu gözlem olarak
    KULLANMAMALI; ödül ve değerlendirme için var.
    """
    return f"/leg{leg}/foot_contact"


# --- Eklem sırası ------------------------------------------------------------


def joint_name(leg: int, part: str) -> str:
    """URDF eklem adı, ör. leg0_coxa_joint."""
    if part not in JOINT_PARTS:
        raise ValueError(f"bilinmeyen eklem: {part!r}")
    return f"leg{leg}_{part}_joint"


def joint_names(leg_ids: Iterable[int] = range(6)) -> list[str]:
    """Komut dizisinin sırası: bacak bacak, her bacakta coxa, femur, tibia.

    [leg0_coxa, leg0_femur, leg0_tibia, leg1_coxa, ..., leg5_tibia]
    """
    return [joint_name(leg, part) for leg in sorted(leg_ids) for part in JOINT_PARTS]


# --- Dönüşümler ----------------------------------------------------------------


def to_command(angles: dict[int, JointAngles]) -> list[float]:
    """Bacak başına eklem açıları (DERECE) -> komut dizisi (RADYAN).

    Altı bacağın da verilmesi gerekir; eksik bacak için sessizce 0
    gönderilmez.
    """
    if sorted(angles) != list(range(6)):
        raise ValueError(f"altı bacağın açısı gerekli, verilen: {sorted(angles)}")
    out = []
    for leg in range(6):
        a = angles[leg]
        out.extend(math.radians(v) for v in (a.coxa, a.femur, a.tibia))
    return out


def from_command(data: Sequence[float]) -> dict[int, JointAngles]:
    """Komut dizisi (RADYAN) -> bacak başına eklem açıları (DERECE)."""
    if len(data) != 18:
        raise ValueError(f"18 değer bekleniyordu, {len(data)} geldi")
    if not all(math.isfinite(v) for v in data):
        raise ValueError("komut dizisinde sonlu olmayan değer var")
    deg = [math.degrees(v) for v in data]
    return {leg: JointAngles(*deg[3 * leg:3 * leg + 3]) for leg in range(6)}
