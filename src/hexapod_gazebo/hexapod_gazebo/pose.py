"""Basit duruşlar (saf Python). Simülasyonu kontrol etmek için; yürüyüş DEĞİL.

Yürüyüş Samet'in işi (GOREVLER.md S2, S3). Buradaki tek duruş, robotu
Gazebo'da ayağa kaldırıp eklem arayüzünün çalıştığını görmek için.
"""

from __future__ import annotations

import math

from hexapod_kinematics import HexapodKinematics, JointAngles


def standing_pose(kin: HexapodKinematics, reach_mm: float,
                  height_mm: float) -> dict[int, JointAngles]:
    """Altı ayak yerde, gövde height_mm yüksekte.

    reach_mm: ayağın coxa ekseninden yatay uzaklığı (bacak dümdüz dışarı).
    height_mm: gövde çerçevesinin (bacak montaj düzlemi) yerden yüksekliği.
    Erişilemeyen bir duruş istenirse ReachError fırlar (kırpılmaz).
    """
    feet = {}
    for leg, mount in kin.mounts.items():
        x = mount.x + reach_mm * math.cos(mount.yaw)
        y = mount.y + reach_mm * math.sin(mount.yaw)
        feet[leg] = (x, y, -height_mm)
    return kin.inverse(feet)
