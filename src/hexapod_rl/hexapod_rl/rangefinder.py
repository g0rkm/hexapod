"""Mesafe sensörü (VL53L0X gibi tek ışınlı) simülasyonu — DENEYSEL yerleşim (G7).

Neden (2026-09-27): kör politika ayak kaldırmayı zemine göre seçemiyor (PROJE_DEVIR
ders 33, 35); önündeki engeli görmesi gerekiyor. Robotta 3 VL53L0X var ama
nereye ve hangi yöne bakacak şekilde takılacakları bilinmiyor: robot.yaml
`sensors.range_finders.devices[*].direction_deg` null (D8). Bu modül o değeri
DOLDURMAZ ve robot.yaml'dan okumaz. Simde bir yerleşimi deneyip "engeli gören
denetleyici kaldırmayı zemine göre ayarlayabiliyor mu, hangi bakış açısıyla"
sorusunu cevaplamak ve D8'e öneri çıkarmak için; yerleşim her deneyde açıkça
verilir (RangeSensor), varsayılanı yok. RangeSensor ve obstacle_height robotta
da çalışan hexapod_policy.lift_reflex'te (eğitim ve robot aynı hesap).

Model: gövdeye bağlı tek ışın; ışın boyunca zemin yüksekliği fonksiyonuna
(sim.TerrainHeight) çarpana kadar ilerlenir, sonra ikiye bölmeyle incelir.
Görüş konisi, gürültü, yansıma ve ölçüm süresi YOK (ideal ışın); gerçek
sensörün bunlarını D8/S7'de ölçmeden eklemiyoruz. Engel yüksekliği
(obstacle_height) robotta da hesaplanabilen büyüklüklerden çıkar: mesafe,
sensörün gövdedeki yeri ve IMU'nun verdiği yerçekimi yönü.
"""

from __future__ import annotations

from typing import Callable, Sequence

from hexapod_policy.lift_reflex import RangeSensor, obstacle_height

from .math3d import Quat, Vec3, rotate

TerrainHeight = Callable[[float, float], float]


def ray_distance(origin: Vec3, direction: Vec3, height: TerrainHeight, max_m: float,
                 step_m: float = 0.005) -> float:
    """origin'den direction boyunca zemine (z <= height(x, y)) uzaklık, m; yoksa max_m.
    step_m'den ince engeller (basamak kenarı dahil değil) kaçabilir."""
    ox, oy, oz = origin
    dx, dy, dz = direction

    def below(s: float) -> bool:
        return oz + s * dz <= height(ox + s * dx, oy + s * dy)

    if below(0.0):
        return 0.0
    prev, s = 0.0, step_m
    while s <= max_m + 1e-12:
        if below(s):
            lo, hi = prev, s
            for _ in range(12):   # 5 mm / 2^12 ~ 1 µm
                mid = 0.5 * (lo + hi)
                lo, hi = (lo, mid) if below(mid) else (mid, hi)
            return hi
        prev, s = s, s + step_m
    return max_m


def read(sensors: Sequence[RangeSensor], base_pos: Vec3, base_quat: Quat,
         height: TerrainHeight) -> list[float]:
    """Her sensörün ölçtüğü mesafe, m (gövde pozu dünyada: pos, quat dünya <- gövde)."""
    out = []
    for s in sensors:
        mount = rotate(base_quat, (s.x, s.y, s.z))
        origin = (base_pos[0] + mount[0], base_pos[1] + mount[1], base_pos[2] + mount[2])
        out.append(ray_distance(origin, rotate(base_quat, s.direction()), height, s.max_m))
    return out


__all__ = ["RangeSensor", "obstacle_height", "ray_distance", "read"]
