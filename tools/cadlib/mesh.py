"""STL okuma ve kütle özellikleri (hacim, ağırlık merkezi, atalet).

Parçanın en uç noktası ya da hacmi STEP yüzeylerinden değil, STL mesh
köşelerinden alınır: B-spline kontrol noktaları yüzeyin dışında durur
(PROJE_DEVIR §12).
"""

from __future__ import annotations

import math
import re
import struct
from pathlib import Path

from .transform import Mat, apply, cross


def read_stl(path: Path) -> list[tuple[float, float, float]]:
    """Binary ya da ASCII STL'den köşe noktaları; her üç nokta bir üçgen."""
    data = Path(path).read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        found = re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)
        return [tuple(float(x) for x in v) for v in found]
    count = struct.unpack_from("<I", data, 80)[0]
    points = []
    for i in range(count):
        offset = 84 + i * 50 + 12
        for k in range(3):
            points.append(struct.unpack_from("<3f", data, offset + 12 * k))
    return points


def load_stl(path: Path, m: Mat) -> list[tuple[float, float, float]]:
    """STL köşeleri, m dönüşümü uygulanmış."""
    return [apply(m, p) for p in read_stl(path)]


def flip_winding(points: list) -> list:
    """Üçgen sırasını ters çevir. Yansıtılmış mesh'te hacim işareti düzelsin diye."""
    return [q for k in range(0, len(points) - 2, 3) for q in (points[k], points[k + 2], points[k + 1])]


def aabb(points) -> list[float]:
    """Eksenlere hizalı sınır kutusu: [merkez x, y, z, boy x, y, z]."""
    lo = [min(p[i] for p in points) for i in range(3)]
    hi = [max(p[i] for p in points) for i in range(3)]
    return [(lo[i] + hi[i]) / 2 for i in range(3)] + [hi[i] - lo[i] for i in range(3)]


class Box:
    """Herhangi bir yönde duran kutu: merkez, üç birim eksen, eksen başına boy (mm)."""

    def __init__(self, center, axes, size) -> None:
        self.center = tuple(center)
        self.axes = [tuple(a) for a in axes]
        self.size = tuple(size)

    @classmethod
    def aligned(cls, center, size) -> "Box":
        return cls(center, ((1, 0, 0), (0, 1, 0), (0, 0, 1)), size)

    def corners(self):
        out = []
        for sx in (-0.5, 0.5):
            for sy in (-0.5, 0.5):
                for sz in (-0.5, 0.5):
                    f = (sx * self.size[0], sy * self.size[1], sz * self.size[2])
                    out.append(tuple(self.center[i] + sum(f[k] * self.axes[k][i] for k in range(3))
                                     for i in range(3)))
        return out

    def moved(self, m: Mat) -> "Box":
        turn = [tuple(sum(m[i][k] * a[k] for k in range(3)) for i in range(3)) for a in self.axes]
        return Box(apply(m, self.center), turn, self.size)

    @classmethod
    def servo(cls, center, horn, size) -> "Box":
        """Servo gövdesi, CAD'deki merkezinden ve horn'unun yerinden.

        Servo modelinin orijini gövdenin merkezinde; horn mil ekseni üzerinde,
        merkezden mil yönünde ~17 mm ve gövdenin uzun kenarı boyunca ~10 mm
        kaçık (MG996R'da mil gövdenin bir ucuna yakın). Yani merkezden horn'a
        giden vektörün baskın bileşeni mil (yükseklik) eksenini, kalanı uzun
        kenarı verir. size = [uzunluk, genişlik, yükseklik(mil yönü)].
        """
        d = [horn[i] - center[i] for i in range(3)]
        k = max(range(3), key=lambda i: abs(d[i]))
        shaft = [0.0, 0.0, 0.0]
        shaft[k] = math.copysign(1.0, d[k])
        rest = [d[i] - (d[k] if i == k else 0.0) for i in range(3)]
        n = math.sqrt(sum(v * v for v in rest))
        length = [v / n for v in rest]
        return cls(center, (length, cross(shaft, length), shaft), size)


class MassProps:
    """Kütle, birinci moment ve ikinci moment (∫ x xᵀ dm) toplayıcısı.

    Hepsi aynı çerçevede tutulur; atalet en sonda ağırlık merkezine taşınır.
    Birimler: kg, mm.
    """

    def __init__(self) -> None:
        self.m = 0.0
        self.s = [0.0, 0.0, 0.0]
        self.c = [[0.0] * 3 for _ in range(3)]

    def add_mesh(self, points, density_kg_mm3: float) -> None:
        """Kapalı üçgen ağ; her üçgen orijinle bir dörtyüzlü oluşturur.

        Hacim işareti üçgen yönünden gelir: yansıtılmış bir mesh'i eklemeden
        önce flip_winding() uygula, yoksa kütle eklenmez, çıkarılır.
        """
        for k in range(0, len(points) - 2, 3):
            a, b, c = points[k], points[k + 1], points[k + 2]
            v = (a[0] * (b[1] * c[2] - b[2] * c[1])
                 - a[1] * (b[0] * c[2] - b[2] * c[0])
                 + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
            dm = v * density_kg_mm3
            tot = [a[i] + b[i] + c[i] for i in range(3)]
            self.m += dm
            for i in range(3):
                self.s[i] += dm * tot[i] / 4.0
                for j in range(3):
                    self.c[i][j] += dm / 20.0 * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j]
                                                 + tot[i] * tot[j])

    def add_box(self, box: Box, mass: float) -> None:
        """Düzgün yoğunluklu kutu."""
        self.m += mass
        for i in range(3):
            self.s[i] += mass * box.center[i]
            for j in range(3):
                # kendi ekseninde ∫ u² dm = m·boy²/12; çerçeveye döndür
                own = sum(box.axes[k][i] * box.axes[k][j] * mass * box.size[k] ** 2 / 12.0
                          for k in range(3))
                self.c[i][j] += mass * box.center[i] * box.center[j] + own

    def com(self):
        return tuple(v / self.m for v in self.s)

    def inertia_at_com(self):
        """(ixx, iyy, izz, ixy, ixz, iyz), kg·mm², ağırlık merkezinde."""
        g = self.com()
        cc = [[self.c[i][j] - self.m * g[i] * g[j] for j in range(3)] for i in range(3)]
        tr = cc[0][0] + cc[1][1] + cc[2][2]
        return (tr - cc[0][0], tr - cc[1][1], tr - cc[2][2],
                -cc[0][1], -cc[0][2], -cc[1][2])
