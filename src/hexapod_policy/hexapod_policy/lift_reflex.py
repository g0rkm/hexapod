"""Mesafe sensörlü ayak kaldırma refleksi (saf Python; eğitimde ve robotta aynı kod).

Neden (2026-09-27): kör politika ayak kaldırmayı zemine göre seçemiyor
(PROJE_DEVIR ders 33, 35): 25 mm düzde verimli ama 45 mm engelde takılıyor,
50 mm engeli geçiyor ama düzde pahalı. Önüne bakan mesafe sensörleri engeli
görürse kaldırmayı yükseltir, görmezse düşük tutar. Simde (ideal sensör,
rastgeleleştirme açık) düzde 25 mm'nin enerjisinde kalıp engellerde sabit
50 mm'den iyi geçiyor (models/README, "Mesafe sensörlü kaldırma refleksi").

RangeSensor yerleşimi robot.yaml'dan gelmeli (D8); orada şu an bilinmiyor
(`sensors.range_finders.devices[*].direction_deg: null`). Bu modül yerleşim
için varsayılan KOYMAZ: her kullanımda açıkça verilir. Simdeki denemeler
DENEYSEL bir yerleşimle yapıldı; D8'e öneri olarak yazıldı.

Kural: her sensörün ölçümünden ışının çarptığı noktanın robotun durduğu
zemine göre yüksekliği (obstacle_height: mesafe + yerleşim + IMU yerçekimi
yönü). Bir sensör art arda `persist` okumada `threshold_m`'yi aşarsa engel
kaydedilir; son `hold_s` saniyede kaydedilen en yüksek engel + `margin_m`
kaldırma olur (low_mm..high_mm), kayıt yoksa low_mm. Tutma süresi, ön
sensör engeli geçtikten sonra orta ve arka bacakların da geçmesi için.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class RangeSensor:
    """Gövde çerçevesinde (+x ileri, +y sol, +z yukarı) tek ışınlı bir mesafe sensörü."""

    x: float           # m
    y: float           # m
    z: float           # m
    yaw_deg: float     # bakış yönü; + sola (yukarıdan bakınca saat yönünün tersi)
    pitch_deg: float   # + aşağı bakar
    max_m: float       # bu mesafede bir şey yoksa ölçüm max_m ("görmüyor")

    def direction(self) -> Vec3:
        """Bakış yönü, gövde çerçevesinde birim vektör."""
        yaw, pitch = math.radians(self.yaw_deg), math.radians(self.pitch_deg)
        return (math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw),
                -math.sin(pitch))


def obstacle_height(sensor: RangeSensor, distance: float, gravity_in_base: Vec3,
                    stand_height: float) -> float | None:
    """Işının çarptığı noktanın, robotun durduğu zemine göre yüksekliği, m.

    Çarpma noktası gövde çerçevesinde sensör yeri + mesafe x yön; yerçekimi
    yönüne (IMU, birim vektör) izdüşümü gövdeye göre derinliği verir; gövde
    durduğu zeminden stand_height yukarıda varsayılır. Işın bir şey
    görmediyse (mesafe >= max_m) None.
    """
    if distance >= sensor.max_m:
        return None
    d = sensor.direction()
    hit = (sensor.x + distance * d[0], sensor.y + distance * d[1], sensor.z + distance * d[2])
    g = gravity_in_base
    return stand_height - (hit[0] * g[0] + hit[1] * g[1] + hit[2] * g[2])


@dataclass
class LiftReflex:
    """update(t, yükseklikler) -> ayak kaldırma, mm. Ayarlar simde seçildi (deney A)."""

    low_mm: float = 25.0
    high_mm: float = 60.0
    threshold_m: float = 0.02
    margin_m: float = 0.015
    hold_s: float = 3.0
    persist: int = 2
    _streak: list[int] = field(default_factory=list, repr=False)
    _seen: list[tuple[float, float]] = field(default_factory=list, repr=False)

    def reset(self) -> None:
        self._streak, self._seen = [], []

    def update(self, t: float, heights: Sequence[float | None]) -> float:
        """t: s (tekdüze artan); heights: sensör başına obstacle_height ya da None."""
        if len(self._streak) != len(heights):
            self._streak = [0] * len(heights)
        for i, h in enumerate(heights):
            if h is not None and h > self.threshold_m:
                self._streak[i] += 1
                if self._streak[i] >= self.persist:
                    self._seen.append((t, h))
            else:
                self._streak[i] = 0
        self._seen = [(ts, h) for ts, h in self._seen if t - ts <= self.hold_s]
        if not self._seen:
            return self.low_mm
        top = max(h for _, h in self._seen)
        return min(self.high_mm, max(self.low_mm, 1000.0 * (top + self.margin_m)))


__all__ = ["LiftReflex", "RangeSensor", "obstacle_height"]
