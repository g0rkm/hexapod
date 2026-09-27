"""Adım saatinin fonksiyonu olan açık döngü tripod (saf Python; iç durumu yok).

İki yerde aynı yörünge gerekiyor, o yüzden tek uygulama burada:
  - eğitim: taklit gösterimi (hexapod_rl.demo) ve "artık eylem" modunun tabanı
    (TaskConfig.action_mode = "residual": hedef = bu tripod + politikanın düzeltmesi),
  - robot: politika düğümü artık eylem modunda aynı tabanı hesaplar.

Samet'in TripodGait'inden (hexapod_gait, GOREVLER.md S2) farkı: dünya
çerçevesinde çapa tutmaz; ayak hedefleri yalnız adım saatine ve hız
komutuna bağlı. Politika gözlemi bu ikisini içerdiği için taban gözlemden
çıkarılabilir; RL'nin üstüne binmesi için gereken bu. RL ortamında ikisi
neredeyse aynı ölçülüyor (2026-09-25).

Yörünge: saatin ilk yarısında groups[0] havada, groups[1] yerde
(hexapod_rl.task.gait_score ile aynı sözleşme). Destekte ayak gövdeye göre
istenen hareketin tersine kayar; bir destek fazı 0.5/hz s sürdüğünden
destekte yer değiştirme = hız x 0.5/hz. Dönüş komutu (wz) ayağı gövde
merkezi etrafında kaydırır. Salınımda ayak yarım sinüsle kalkıp öne gelir.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from hexapod_kinematics import HexapodKinematics

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class PhaseTripod:
    kin: HexapodKinematics
    groups: tuple             # iki üçlü grup; ilk yarıda groups[0] havada
    gait_hz: float            # adım saati frekansı
    reach_mm: float           # ayağın coxa ekseninden yatay uzaklığı
    height_mm: float          # gövde yüksekliği
    lift_mm: float = 25.0     # salınımda ayak kaldırma

    @staticmethod
    def swing_group(phase: float) -> int:
        """phase anında havadaki grup: 0 -> groups[0], 1 -> groups[1]. Değiştiği
        adım yeni bir salınımın başı (öğrenilmiş kaldırma orada seçilir)."""
        return 0 if phase % 1.0 < 0.5 else 1

    def feet(self, phase: float, command, lift_mm: float | None = None) -> dict[int, Vec3]:
        """Adım saatinin phase ([0, 1)) anında ayak hedefleri, gövde çerçevesi, mm.
        lift_mm: bu an için ayak kaldırma (öğrenilmiş kaldırma; verilmezse
        self.lift_mm)."""
        lift = self.lift_mm if lift_mm is None else lift_mm
        vx, vy, wz = command
        stance_s = 0.5 / self.gait_hz
        out = {}
        for leg, m in self.kin.mounts.items():
            hx = m.x + self.reach_mm * math.cos(m.yaw)
            hy = m.y + self.reach_mm * math.sin(m.yaw)
            # destekte ayağın gövdeye göre gidişi, mm (hız m/s -> mm/s)
            dx = (vx * 1000.0 - wz * hy) * stance_s
            dy = (vy * 1000.0 + wz * hx) * stance_s
            p = (phase + 0.5) % 1.0 if leg in self.groups[0] else phase
            if p < 0.5:                        # destek: +d/2'den -d/2'ye
                s, dz = 0.5 - 2.0 * p, 0.0
            else:                              # salınım: -d/2'den +d/2'ye, kalkarak
                q = (p - 0.5) * 2.0
                s, dz = -0.5 + q, lift * math.sin(math.pi * q)
            out[leg] = (hx + s * dx, hy + s * dy, -self.height_mm + dz)
        return out

    def targets(self, phase: float, command, lift_mm: float | None = None) -> list[float]:
        """Eklem hedefleri, rad, interface.joint_names() sırasıyla."""
        angles = self.kin.inverse(self.feet(phase, command, lift_mm))
        return [math.radians(v) for leg in sorted(angles)
                for v in angles[leg].as_dict().values()]


__all__ = ["PhaseTripod"]
