"""IMU'nun montaj yönelimi: sensör çerçevesi -> gövde çerçevesi (GOREVLER.md S7).

Politika ve refleks gövde çerçevesinde veri bekliyor (docs/ARAYUZ.md madde 3:
"/imu her zaman base_link yöneliminde yayınlanır; gerçek sürücü BNO055'in ham
verisini montaj yönelimine göre döndürür"). IMU karta nasıl takıldıysa
eksenleri gövdeninkiyle aynı olmak zorunda değil.

Bu dönüşüm YANLIŞSA sessizce bozulur: robot eğik olmadığı hâlde politika eğik
sanır, ya da sağa yatmayı sola yatma diye okur. Simde öğrenilen her şey
bozulur ama hata "politika kötü" gibi görünür. Bu yüzden değer uydurulmaz:
robot.yaml `sensors.imu.mount_rotation_deg` null (D8'in ölçümü), verilmezse
MissingValue.

Dönme sırası ZYX (önce yaw, sonra pitch, sonra roll) — hexapod_kinematics.body
ile aynı sözleşme, iki yerde farklı olmasın.
"""

from __future__ import annotations

import math
from typing import Sequence

from hexapod_driver.config import RobotConfig, Value
from hexapod_driver.errors import ConfigError

Vec3 = tuple[float, float, float]
Matrix = tuple[Vec3, Vec3, Vec3]

#: Dönüşsüz montaj (sensör eksenleri gövdeyle aynı).
IDENTITY: Matrix = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def rotation(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Matrix:
    """ZYX dönme matrisi: sensör çerçevesindeki vektörü gövdeye taşır."""
    r, p, y = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def parse_mount(value) -> Matrix:
    """robot.yaml'daki mount_rotation_deg -> dönme matrisi.

    Tek sayı: yalnız yaw (kart güverteye düz ama döndürülmüş takılmış).
    Üç sayı: (roll, pitch, yaw) — kart yan ya da ters takılmışsa.
    Hangisi olacağı D8'de belli olur; ikisi de kabul ediliyor ki o zaman
    burada değişiklik gerekmesin.
    """
    if isinstance(value, (int, float)):
        return rotation(0.0, 0.0, float(value))
    if isinstance(value, Sequence) and len(value) == 3:
        roll, pitch, yaw = (float(v) for v in value)
        return rotation(roll, pitch, yaw)
    raise ConfigError(
        "sensors.imu.mount_rotation_deg tek sayı (yaw) ya da üç sayı "
        f"(roll, pitch, yaw) olmalı; {value!r} verildi")


def mount_from_config(config: RobotConfig) -> Matrix:
    """robot.yaml'dan montaj dönüşü. Girilmemişse MissingValue."""
    imu = ((config.raw.get("sensors") or {}).get("imu") or {})
    value = Value.parse(imu.get("mount_rotation_deg"), "sensors.imu.mount_rotation_deg")
    return parse_mount(value.require())


def rotate_to_base(matrix: Matrix, vector: Vec3) -> Vec3:
    """Sensör çerçevesindeki vektörü gövde çerçevesine taşı."""
    return tuple(sum(matrix[i][j] * vector[j] for j in range(3))  # type: ignore[return-value]
                 for i in range(3))


Quat = tuple[float, float, float, float]   # w, x, y, z


def quaternion_to_matrix(q: Quat) -> Matrix:
    """Birim quaternion -> dönme matrisi."""
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def matrix_to_quaternion(m: Matrix) -> Quat:
    """Dönme matrisi -> birim quaternion (w, x, y, z).

    En büyük bileşenden çözülür; küçük bileşenden çözmek sayısal olarak
    kötüdür (sıfıra bölmeye yaklaşır).
    """
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s,
                (m[0][2] + m[2][0]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s,
                (m[1][2] + m[2][1]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s,
            0.25 * s)


def quaternion_to_base(matrix: Matrix, quat: Quat) -> Quat:
    """Sensörün yönelimini gövdenin yönelimine çevir.

    IMU'nun verdiği quaternion sensörün yönelimidir (dünya <- sensör).
    Politika gövdenin yönelimini bekliyor (dünya <- gövde,
    hexapod_rl.state.gravity_in_base ile aynı sözleşme). M sensörü gövdeye
    taşıyorsa (v_gövde = M v_sensör), gövde->dünya = (sensör->dünya) . Mᵀ.

    Bu dönüşüm ATLANIRSA robot, IMU dönük takılıysa eğikliğini yanlış okur
    ve hata "politika kötü" gibi görünür.
    """
    r_sensor = quaternion_to_matrix(quat)
    m_t = tuple(tuple(matrix[j][i] for j in range(3)) for i in range(3))
    product = tuple(tuple(sum(r_sensor[i][k] * m_t[k][j] for k in range(3))
                          for j in range(3)) for i in range(3))
    return matrix_to_quaternion(product)  # type: ignore[arg-type]


__all__ = ["IDENTITY", "Matrix", "Quat", "matrix_to_quaternion", "mount_from_config",
           "parse_mount", "quaternion_to_base", "quaternion_to_matrix", "rotate_to_base",
           "rotation"]
