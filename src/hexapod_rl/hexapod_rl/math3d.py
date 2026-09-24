"""Küçük 3B yardımcılar (saf Python; gz.sim gerektirmez, her yerde test edilir)."""

from __future__ import annotations

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # w, x, y, z


def rotate_inverse(q: Quat, v: Vec3) -> Vec3:
    """v'yi q'nun tersiyle döndür (dünya -> gövde). q = (w, x, y, z), birim.

    Gövde çerçevesindeki IMU'nun gördüğü büyüklükleri (yerçekimi yönü,
    açısal hız) dünya çerçevesindeki değerlerden hesaplamak için.
    """
    w, x, y, z = q
    # R(q)^T v
    r00, r01, r02 = 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)
    r10, r11, r12 = 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)
    r20, r21, r22 = 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)
    return (r00 * v[0] + r10 * v[1] + r20 * v[2],
            r01 * v[0] + r11 * v[1] + r21 * v[2],
            r02 * v[0] + r12 * v[1] + r22 * v[2])
