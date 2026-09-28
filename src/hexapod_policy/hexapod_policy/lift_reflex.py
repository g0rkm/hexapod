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
zemine göre yüksekliği (LiftReflex.heights -> obstacle_height: mesafe +
yerleşim). Bir sensör art arda `persist` okumada `threshold_m`'yi aşarsa engel
kaydedilir; son `hold_s` saniyede kaydedilen en yüksek engel + `margin_m`
kaldırma olur (low_mm..high_mm), kayıt yoksa low_mm. Tutma süresi, ön
sensör engeli geçtikten sonra orta ve arka bacakların da geçmesi için.

Yükseklik neye göre (reference, 2026-09-28), yani "aşağı" hangi yön:
  - "yercekimi": IMU'nun yerçekimi yönü (ilk sürüm). Düzgün bir yokuş da 20°
    aşağı bakan ışının çarptığı yerde "yükselmiş zemin" görünüyor, refleks
    ayağı boşuna kaldırıyor (S6: 10° yokuşta 47.3 J/m, düzde 36.5).
  - "govde": gövdenin -z'si. Gövde eğimde zemine paralel durduğu için düzgün
    eğim düz görünür (10° yokuş 39.3 J/m); ama basamağa çıkarken burun
    kalkınca basamağın üstü alçak görünüyor, refleks kaldırmayı erken
    indiriyor ve arka bacaklar takılabiliyor (kapalı döngü 45 mm: 0.94 ->
    0.45 m).
  - "egim" (VARSAYILAN): ikisinin birleşimi. Yerçekimi yönünün slope_tau_s'lik
    ortalaması zeminin eğimi sayılır; "aşağı", gövde -z'sinin bu yavaş
    eğimden şimdiki yerçekimine olan dönüşle çevrilmişi. Kalıcı yokuşta gövde
    gibi, basamaktaki kısa burun kalkmasında yerçekimi gibi davranır.
    Ölçüldü (S6 zeminleri, ppo_kaldirma35_250k, J/m; yerçekimi -> egim): 10°
    yokuş 47.3 -> 39.3 (tripod 39.4; 8 tohumda 49.4 -> 43.4), 20° yokuş 53.7
    -> 46.5, yan eğim 44.6 -> 37.5, merdiven 58.4 -> 56.9; düz, çukur ve 45 mm
    basamak aynı; 60 mm basamak iki kipte de 8 tohumun 6'sında geçiliyor.
    Kapalı döngü 45 mm basamak testini (gövde kipinin kaldığı) geçiyor.
    ppo_refleks_1500k yerçekimi kipiyle eğitildi; 60 mm basamakta onunla
    biraz daha iyi (models/README).
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


#: Gövde çerçevesinde "aşağı": gövde düzlemine göre yükseklik (LiftReflex.reference "govde").
BODY_DOWN: Vec3 = (0.0, 0.0, -1.0)

#: LiftReflex.reference değerleri (modül açıklaması).
REFERENCES = ("yercekimi", "govde", "egim")


def rotate_between(a: Vec3, b: Vec3, v: Vec3) -> Vec3:
    """Birim a'yı birim b'ye çeviren en kısa dönüşü v'ye uygula (Rodrigues)."""
    c = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    k = (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
    if c <= -1.0 + 1e-9:
        raise ValueError("zıt yönler arasında dönüş tanımsız")
    kxv = (k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0])
    kv = (k[0] * v[0] + k[1] * v[1] + k[2] * v[2]) / (1.0 + c)
    return tuple(v[i] * c + kxv[i] + k[i] * kv for i in range(3))


def obstacle_height(sensor: RangeSensor, distance: float, gravity_in_base: Vec3,
                    stand_height: float) -> float | None:
    """Işının çarptığı noktanın, robotun durduğu zemine göre yüksekliği, m.

    Çarpma noktası gövde çerçevesinde sensör yeri + mesafe x yön; "aşağı"
    yönüne (gravity_in_base, birim vektör: IMU'nun yerçekimi yönü ya da gövde
    düzlemi için BODY_DOWN) izdüşümü gövdeye göre derinliği verir; gövde
    durduğu zeminden stand_height yukarıda varsayılır. Işın bir şey
    görmediyse (mesafe >= max_m) None.
    """
    if distance >= sensor.max_m:
        return None
    d = sensor.direction()
    hit = (sensor.x + distance * d[0], sensor.y + distance * d[1], sensor.z + distance * d[2])
    g = gravity_in_base
    return stand_height - (hit[0] * g[0] + hit[1] * g[1] + hit[2] * g[2])


#: Bir sensör, bakış yönünden en çok bu kadar sapan yürüyüş yönünü "görür".
#: Simde (20° aşağı bakan ışın, 45 mm basamak): 30° sapmada refleks zamanında
#: tetikleniyor, 60°'de tetiklenmiyor (models/README, yerleşim karşılaştırması).
COVERAGE_DEG = 45.0


def covers(sensors: Sequence[RangeSensor], vx: float, vy: float,
           max_angle_deg: float = COVERAGE_DEG) -> bool:
    """Yürüyüş yönü (vx, vy; gövde çerçevesi) bir sensörün bakışına yeterince yakın
    mı? Öteleme yoksa (yerinde dönüş) True: refleks karar verebilir."""
    if math.hypot(vx, vy) < 1e-9:
        return True
    walk = math.degrees(math.atan2(vy, vx))
    return any(abs((walk - s.yaw_deg + 180.0) % 360.0 - 180.0) <= max_angle_deg for s in sensors)


@dataclass
class LiftReflex:
    """update(t, yükseklikler) -> ayak kaldırma, mm. Ayarlar simde seçildi (deney A)."""

    low_mm: float = 25.0
    high_mm: float = 60.0
    threshold_m: float = 0.02
    margin_m: float = 0.015
    hold_s: float = 3.0
    persist: int = 2
    reference: str = "egim"        # engel yüksekliği neye göre (REFERENCES; modül açıklaması)
    slope_tau_s: float = 5.0       # "egim": zemin eğiminin (yerçekimi yönü) ortalama süresi
    _streak: list[int] = field(default_factory=list, repr=False)
    _seen: list[tuple[float, float]] = field(default_factory=list, repr=False)
    _slope: tuple | None = field(default=None, repr=False)   # (t, yavaş yerçekimi yönü)

    def __post_init__(self) -> None:
        if self.reference not in REFERENCES:
            raise ValueError(f"reference {REFERENCES} içinden olmalı: {self.reference!r}")
        if not self.slope_tau_s > 0:
            raise ValueError(f"slope_tau_s pozitif olmalı: {self.slope_tau_s}")

    def reset(self) -> None:
        self._streak, self._seen, self._slope = [], [], None

    def heights(self, sensors: Sequence[RangeSensor], distances: Sequence[float],
                gravity_in_base: Vec3, stand_height: float, t: float = 0.0) -> list[float | None]:
        """Sensör başına engel yüksekliği (obstacle_height), m; "aşağı" reference'a
        göre (modül açıklaması). t: s, tekdüze artan ("egim" eğimi zamanla ortalar)."""
        down = self._down(gravity_in_base, t)
        return [obstacle_height(s, d, down, stand_height) for s, d in zip(sensors, distances)]

    def _down(self, g: Vec3, t: float) -> Vec3:
        if self.reference == "yercekimi":
            return g
        if self.reference == "govde":
            return BODY_DOWN
        if self._slope is None:
            slow = g                   # ilk ölçüm: durduğu zemin eğim sayılır
        else:
            t0, prev = self._slope
            a = 1.0 - math.exp(-max(0.0, t - t0) / self.slope_tau_s)
            mix = tuple(p + a * (c - p) for p, c in zip(prev, g))
            n = math.sqrt(sum(v * v for v in mix))
            slow = tuple(v / n for v in mix)
        self._slope = (t, slow)
        return rotate_between(slow, g, BODY_DOWN)

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


__all__ = ["BODY_DOWN", "COVERAGE_DEG", "LiftReflex", "REFERENCES", "RangeSensor", "covers",
           "obstacle_height", "rotate_between"]
