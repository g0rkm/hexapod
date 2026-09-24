"""Görev tanımı: eylem -> eklem hedefi, gözlem, ödül, devrilme (saf Python).

Gymnasium ortamı (env.py) bunları yalnızca numpy dizilerine sarar; mantık
burada, gz.sim ve gymnasium olmadan test edilebilir.

Gözlem — YALNIZCA gerçek robotta da olanlar (docs/ARAYUZ.md):
    yerçekimi yönü, gövde çerçevesinde (3)   IMU yöneliminden
    açısal hız, gövde çerçevesinde (3)      IMU jiroskobu
    eklem hedefleri, normalize (18)          son komut (MG996R geri bildirim vermez)
    hız komutu: vx, vy, wz (3)               politikaya ne istendiği
    adım saati: sin, cos (2)                 periyodik yürüyüşü öğrenmeyi kolaylaştırır
Ölçülen eklem açısı ve ayak teması gözleme GİRMEZ (gerçek robotta yok);
ayak teması ancak ödülde kullanılabilir.

Eylem: 18 değer [-1, 1] -> hedef = varsayılan duruş + ölçek x eylem,
eklem limitlerine kırpılır. Varsayılan duruş ayakta duruş.

Ödül — TÜBİTAK başvurusundaki tanım: devrilmeden, en az enerjiyle, en hızlı
ilerleme. Terimler ve ağırlıkları TaskConfig'te; bunlar robot parametresi
değil, eğitim ayarı (G7'de ayarlanacak). Enerji için gerçek tork yok (servo
hız kontrollü modelleniyor); vekil olarak eklem hızlarının karesi ve eylem
değişimi cezalandırılıyor.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .state import SimState

OBS_SIZE = 3 + 3 + 18 + 3 + 2
ACTION_SIZE = 18


@dataclass(frozen=True)
class TaskConfig:
    # eylem
    action_scale: float = 0.5  # rad; eylem 1 -> varsayılandan 0.5 rad
    # duruş
    stand_reach_mm: float = 130.0
    stand_height_mm: float = 100.0
    # komut aralıkları (bölüm başında rastgele); başlangıçta yalnız ileri
    vx_range: tuple[float, float] = (0.0, 0.15)   # m/s
    vy_range: tuple[float, float] = (0.0, 0.0)
    wz_range: tuple[float, float] = (0.0, 0.0)    # rad/s
    gait_hz: float = 1.5                           # adım saati frekansı
    # bölüm
    episode_s: float = 20.0
    settle_s: float = 1.0          # bölüm başında duruşa yerleşme (ödülsüz)
    # devrilme
    min_height_m: float = 0.045    # gövde bundan alçaksa yere değmiştir
    max_tilt_deg: float = 45.0
    # ödül ağırlıkları
    w: dict = field(default_factory=lambda: {
        "lin_vel": 1.0,        # hız komutunu izleme (exp)
        "yaw_rate": 0.5,
        "orientation": -2.0,   # gövdenin yatması
        "height": -20.0,       # yükseklik sapması (m^2)
        "joint_vel": -5e-4,    # enerji vekili
        "action_rate": -0.01,  # sarsıntı
        "fall": -10.0,         # devrilince bir kez
    })
    lin_vel_sigma: float = 0.05    # m/s
    yaw_rate_sigma: float = 0.5    # rad/s


def action_to_targets(action, default: list[float], scale: float,
                      limits: list[tuple[float, float]]) -> list[float]:
    if len(action) != ACTION_SIZE:
        raise ValueError(f"{ACTION_SIZE} eylem bekleniyordu, {len(action)} geldi")
    out = []
    for a, d, (lo, hi) in zip(action, default, limits):
        a = max(-1.0, min(1.0, float(a)))
        out.append(max(lo, min(hi, d + scale * a)))
    return out


def observation(state: SimState, command: tuple[float, float, float], phase: float,
                default: list[float], scale: float) -> list[float]:
    """phase: adım saati, [0, 1)."""
    g = state.gravity_in_base()
    w = state.ang_vel_in_base()
    targets = [(t - d) / scale for t, d in zip(state.joint_target, default)]
    clock = [math.sin(2 * math.pi * phase), math.cos(2 * math.pi * phase)]
    obs = [*g, *w, *targets, *command, *clock]
    assert len(obs) == OBS_SIZE
    return obs


def fallen(state: SimState, cfg: TaskConfig) -> bool:
    if state.base_pos[2] < cfg.min_height_m:
        return True
    # gövde +z'si ile dünya yukarısı arasındaki açı: yerçekiminin gövdedeki z'si
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -state.gravity_in_base()[2]))))
    return tilt > cfg.max_tilt_deg


def reward(state: SimState, action, prev_action, command: tuple[float, float, float],
           cfg: TaskConfig, fell: bool) -> tuple[float, dict[str, float]]:
    vx, vy, _ = state.lin_vel_in_base()
    wz = state.ang_vel_in_base()[2]
    g = state.gravity_in_base()
    ex, ey = command[0] - vx, command[1] - vy
    terms = {
        "lin_vel": math.exp(-(ex * ex + ey * ey) / cfg.lin_vel_sigma ** 2),
        "yaw_rate": math.exp(-((command[2] - wz) ** 2) / cfg.yaw_rate_sigma ** 2),
        "orientation": g[0] ** 2 + g[1] ** 2,
        "height": (state.base_pos[2] - cfg.stand_height_mm / 1000.0) ** 2,
        "joint_vel": sum(v * v for v in state.joint_vel if math.isfinite(v)),
        "action_rate": sum((a - b) ** 2 for a, b in zip(action, prev_action)),
        "fall": 1.0 if fell else 0.0,
    }
    weighted = {k: cfg.w[k] * v for k, v in terms.items()}
    return sum(weighted.values()), weighted
