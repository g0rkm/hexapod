"""Katı cisim dönüşümleri: 3x4 matris (üç satır, her biri [r0, r1, r2, öteleme]).

Numpy'sız, çünkü araçlar yalnızca pyyaml ile çalışabilsin diye (README
kurulumu). Matrisler tuple; p' = R·p + t.
"""

from __future__ import annotations

import math

Mat = tuple  # ((r00, r01, r02, tx), (r10, r11, r12, ty), (r20, r21, r22, tz))
Vec3 = tuple

IDENT: Mat = ((1.0, 0, 0, 0), (0, 1.0, 0, 0), (0, 0, 1.0, 0))
MIRROR_Y: Mat = ((1, 0, 0, 0), (0, -1, 0, 0), (0, 0, 1, 0))


def rigid(rot, origin) -> Mat:
    """3x3 dönme (satırlar) + öteleme -> 3x4."""
    return tuple(tuple(rot[i]) + (origin[i],) for i in range(3))


def mat_from_axis(loc, zdir, xdir) -> Mat:
    """STEP AXIS2_PLACEMENT_3D (konum, z yönü, x yönü) -> 3x4."""
    if zdir is None:
        zdir = (0.0, 0.0, 1.0)
    if xdir is None:
        xdir = (1.0, 0.0, 0.0)
    z = normalize(zdir)
    x = normalize(sub(xdir, scale(z, dot(xdir, z))))
    y = cross(z, x)
    return ((x[0], y[0], z[0], loc[0]),
            (x[1], y[1], z[1], loc[1]),
            (x[2], y[2], z[2], loc[2]))


def mat_mul(a: Mat, b: Mat) -> Mat:
    out = []
    for i in range(3):
        row = [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        row.append(sum(a[i][k] * b[k][3] for k in range(3)) + a[i][3])
        out.append(tuple(row))
    return tuple(out)


def mat_inv(a: Mat) -> Mat:
    """Katı dönüşümün tersi (dönme kısmı ortonormal varsayılır)."""
    out = []
    for i in range(3):
        row = [a[0][i], a[1][i], a[2][i]]
        row.append(-(a[0][i] * a[0][3] + a[1][i] * a[1][3] + a[2][i] * a[2][3]))
        out.append(tuple(row))
    return tuple(out)


def apply(m: Mat, p) -> Vec3:
    return tuple(m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3] for i in range(3))


def rotate(m: Mat, v) -> Vec3:
    """Yalnız dönme kısmı (yön vektörleri için)."""
    return tuple(m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] for i in range(3))


def rot_about_neg_y(theta: float):
    """-y ekseni etrafında theta: +x yukarı (+z) döner. URDF femur/tibia ekseni."""
    c, s = math.cos(theta), math.sin(theta)
    return ((c, 0.0, -s), (0.0, 1.0, 0.0), (s, 0.0, c))


def rpy_of(m: Mat) -> tuple[float, float, float]:
    """Dönme kısmı -> URDF rpy (R = Rz(yaw) · Ry(pitch) · Rx(roll))."""
    r20 = max(-1.0, min(1.0, m[2][0]))
    pitch = -math.asin(r20)
    if abs(r20) < 1 - 1e-9:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    else:  # gimbal kilidi; yaw'ı sıfır al
        roll = math.atan2(-m[1][2], m[1][1])
        yaw = 0.0
    return roll, pitch, yaw


def sub(a, b) -> Vec3:
    return tuple(a[i] - b[i] for i in range(3))


def dot(a, b) -> float:
    return sum(a[i] * b[i] for i in range(3))


def cross(a, b) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def scale(a, s) -> Vec3:
    return tuple(v * s for v in a)


def normalize(a) -> Vec3:
    n = math.sqrt(dot(a, a)) or 1.0
    return tuple(v / n for v in a)
