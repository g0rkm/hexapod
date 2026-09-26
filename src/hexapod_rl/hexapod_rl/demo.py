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

Yörüngenin kendisi hexapod_policy.tripod.PhaseTripod'da (robottaki politika
düğümü "artık eylem" modunda aynısını kullanır). Adım saati ortamınkiyle
aynı; saatin ilk yarısında groups[0] havada, groups[1] yerde
(task.gait_score ile aynı sözleşme).
"""

from __future__ import annotations

from dataclasses import dataclass, field

from hexapod_kinematics import HexapodKinematics
from hexapod_policy.tripod import PhaseTripod

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
    base: PhaseTripod = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "base", PhaseTripod(
            self.kin, self.groups, self.gait_hz, self.reach_mm, self.height_mm, self.lift_mm))

    def feet(self, phase: float, command):
        """Adım saatinin phase ([0, 1)) anında ayak hedefleri, gövde çerçevesi, mm."""
        return self.base.feet(phase, command)

    def targets(self, phase: float, command) -> list[float]:
        """Eklem hedefleri, rad, interface.joint_names() sırasıyla."""
        return self.base.targets(phase, command)

    def action(self, phase: float, command) -> list[float]:
        """Ortamın eylemi: (hedef - varsayılan) / ölçek. Kırpılmaz; aralık dışına
        çıkıyorsa çağıran bilsin (komut aralığı testle denetleniyor)."""
        out = [(t - d) / self.action_scale
               for t, d in zip(self.targets(phase, command), self.default)]
        assert len(out) == ACTION_SIZE
        return out
