"""Tek bacak kinematiği: 3 DOF (coxa yaw, femur pitch, tibia pitch).

Bacak çerçevesi
---------------
Orijin coxa'nın dikey dönme ekseni üzerinde, femur ekleminin yüksekliğinde.
  +x : bacak dümdüz dışarı (coxa = 0 iken)
  +y : +x'in yukarıdan bakınca saat yönünün tersine 90 derece solu
  +z : yukarı

Sıfır duruşu (bütün açılar 0)
-----------------------------
Kalibrasyonda "merkez" (center_us) olarak kaydedilen konum budur; ikisi
aynı olmak ZORUNDA, yoksa her ayak sistematik olarak kayık basar.
  coxa  = 0 : bacak gövdeden dümdüz dışarı bakıyor
  femur = 0 : femur yere paralel
  tibia = 0 : tibia femura dik (femur yataysa tibia dümdüz aşağı)

Pozitif yönler
--------------
  coxa  + : yukarıdan bakınca saat yönünün tersine
  femur + : bacak yukarı kalkar
  tibia + : diz açılır, ayak dışarı gider

Aynı kural altı bacak için de geçerli; aynalı bacaklarda servonun ters
dönmesi calibration.yaml'daki direction ile giderilir, burada değil.

Çözüm dalı
----------
Ters kinematiğin her hedef için iki çözümü var (diz yukarıda / diz aşağıda).
Hexapod her zaman diz yukarıda çalışır; yalnızca o döndürülür.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .errors import ReachError

Vec3 = tuple[float, float, float]

_EPS = 1e-6


@dataclass(frozen=True)
class LegGeometry:
    """Segment uzunlukları, mm."""

    coxa: float
    femur: float
    tibia: float

    @property
    def max_reach(self) -> float:
        """Femur ekleminden ayağa en uzak erişim (bacak dümdüz)."""
        return self.femur + self.tibia

    @property
    def min_reach(self) -> float:
        """Femur ekleminden ayağa en yakın erişim (bacak tamamen katlı)."""
        return abs(self.femur - self.tibia)


@dataclass(frozen=True)
class JointAngles:
    """Bir bacağın üç eklem açısı, derece."""

    coxa: float
    femur: float
    tibia: float

    def as_dict(self) -> dict[str, float]:
        return {"coxa": self.coxa, "femur": self.femur, "tibia": self.tibia}


ZERO = JointAngles(0.0, 0.0, 0.0)


def forward(geom: LegGeometry, angles: JointAngles) -> Vec3:
    """Eklem açılarından ayak ucunun bacak çerçevesindeki konumu."""
    q1 = math.radians(angles.coxa)
    q2 = math.radians(angles.femur)
    q3 = math.radians(angles.tibia)
    psi = q2 + q3 - math.pi / 2  # tibia'nın yataya göre mutlak açısı
    r = geom.coxa + geom.femur * math.cos(q2) + geom.tibia * math.cos(psi)
    z = geom.femur * math.sin(q2) + geom.tibia * math.sin(psi)
    return (r * math.cos(q1), r * math.sin(q1), z)


def inverse(geom: LegGeometry, x: float, y: float, z: float) -> JointAngles:
    """Ayak ucunun bacak çerçevesindeki hedefinden eklem açıları.

    Hedef erişilemiyorsa ReachError fırlatır; en yakın noktaya kırpmaz.
    Kırpmak, ayağın istenmeyen bir yere basmasını sessizce kabul etmek olur.

    Varsayım: ayak coxa ekseninin dışında (gövdeden uzakta). Coxa ekseninin
    arkasındaki bir hedef, bacağın 180 derece dönmüş hali olarak yorumlanır;
    bu iki durum geometrik olarak ayırt edilemez. Gerçek robotta o bölge
    gövdenin altı olduğu için pratikte sorun değil.
    """
    q1 = math.atan2(y, x)
    r = math.hypot(x, y) - geom.coxa  # femur ekleminden yatay uzaklık
    d = math.hypot(r, z)  # femur ekleminden ayağa düz uzaklık

    if d > geom.max_reach + _EPS:
        raise ReachError(
            f"Ayak çok uzakta: femur ekleminden {d:.1f} mm, "
            f"en fazla {geom.max_reach:.1f} mm erişilebilir."
        )
    if d < geom.min_reach - _EPS or d < _EPS:
        raise ReachError(
            f"Ayak çok yakında: femur ekleminden {d:.1f} mm, "
            f"en az {geom.min_reach:.1f} mm olmalı."
        )

    f, t = geom.femur, geom.tibia
    # Femur, ayağa bakan doğrudan bu açı kadar yukarıda (diz yukarıda çözüm).
    alpha = math.acos(_clamp((f * f + d * d - t * t) / (2 * f * d)))
    q2 = math.atan2(z, r) + alpha
    # Ayak femur ekleminin gerisine düştüğünde (r < 0) toplam 180 dereceyi
    # aşabilir; -60 yerine 300 dönmesin diye (-180, 180] aralığına sar.
    q2 = math.atan2(math.sin(q2), math.cos(q2))
    # Dizdeki iç açı; sıfır duruşunda 90 derece.
    gamma = math.acos(_clamp((f * f + t * t - d * d) / (2 * f * t)))
    q3 = gamma - math.pi / 2

    return JointAngles(math.degrees(q1), math.degrees(q2), math.degrees(q3))


def _clamp(value: float) -> float:
    """acos'a giden değeri kayan nokta gürültüsüne karşı [-1, 1]'e sıkıştır.

    Erişim kontrolü yukarıda yapıldığı için buraya gerçek bir taşma gelmez.
    """
    return max(-1.0, min(1.0, value))
