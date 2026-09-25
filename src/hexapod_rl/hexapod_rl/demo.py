"""Gösterim yürüyüşü: taklit ile başlatma için açık döngü, simetrik tripod (saf Python).

Neden (2026-09-25): ödül v2 ile öğrenilen politika (models/tork_v2_10M)
saniyede ~12° dönerek yürüdü; ödül v3 ile 1.75M adımlık devam eğitimi bunu
düzeltemedi (ödül yatay, yön sapması aynı). Aynı v3 ödülünde bu gösterim adım
başı 2.76, politika 1.82 alıyor: ödül doğru, politika yerel bir tepeye
takılmış. Politika önce bunu taklit etmeyi öğrenirse (pretrain.py) PPO dümdüz
yürüyen bir başlangıçtan iyileştirir.

Bu Samet'in tripod'u (GOREVLER.md S2, hexapod_gait) DEĞİL: parametreleri
sabit, geri beslemesiz, en basit yörünge; yalnızca RL'ye başlangıç noktası.
RL ortamında ikisi neredeyse aynı ölçülüyor (baseline.TripodPolicy ile,
2026-09-25). Taklit için bu tutuldu, çünkü eylemi yalnız adım saatine ve
hız komutuna bağlı (iç durumu yok); taklit edilen şeyin gözlemden
çıkarılabilmesi gerekir. TripodGait ise dünya çerçevesinde çapa tutar.

Yörünge: adım saati ortamınkiyle aynı; saatin ilk yarısında groups[0]
havada, groups[1] yerde (task.gait_score ile aynı sözleşme). Destek fazında
ayak gövdeye göre istenen hareketin tersine kayar; bir destek fazı 0.5/hz s
sürdüğünden destekte ayağın yer değiştirmesi = hız x 0.5/hz. Dönüş komutu
(wz) ayağı gövde merkezi etrafında kaydırır. Salınımda ayak yarım sinüsle
kalkıp öne gelir.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from hexapod_kinematics import HexapodKinematics

from .task import ACTION_SIZE


@dataclass(frozen=True)
class TripodDemo:
    kin: HexapodKinematics
    default: list[float]      # rad, ortamın varsayılan duruşu (eylem 0)
    action_scale: float       # rad, eylem 1 -> varsayılandan bu kadar
    groups: tuple             # tripod grupları, task.tripod_groups
    gait_hz: float            # adım saati frekansı
    reach_mm: float           # duruş: ayağın coxa ekseninden yatay uzaklığı
    height_mm: float          # duruş: gövde yüksekliği
    lift_mm: float = 25.0     # salınımda ayak kaldırma

    def feet(self, phase: float, command) -> dict[int, tuple[float, float, float]]:
        """Adım saatinin phase ([0, 1)) anında ayak hedefleri, gövde çerçevesi, mm."""
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
                s, dz = -0.5 + q, self.lift_mm * math.sin(math.pi * q)
            out[leg] = (hx + s * dx, hy + s * dy, -self.height_mm + dz)
        return out

    def targets(self, phase: float, command) -> list[float]:
        """Eklem hedefleri, rad, interface.joint_names() sırasıyla."""
        angles = self.kin.inverse(self.feet(phase, command))
        return [math.radians(v) for leg in sorted(angles)
                for v in angles[leg].as_dict().values()]

    def action(self, phase: float, command) -> list[float]:
        """Ortamın eylemi: (hedef - varsayılan) / ölçek. Kırpılmaz; aralık dışına
        çıkıyorsa çağıran bilsin (komut aralığı testle denetleniyor)."""
        out = [(t - d) / self.action_scale
               for t, d in zip(self.targets(phase, command), self.default)]
        assert len(out) == ACTION_SIZE
        return out
