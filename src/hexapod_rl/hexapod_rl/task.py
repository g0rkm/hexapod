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
değil, eğitim ayarı (G7'de ayarlanacak). Enerji: servonun uyguladığı tork
simülasyonda bilindiği için gerçek mekanik güç Σ|tork x açısal hız| (W)
cezalandırılıyor; ayrıca eylem değişimi (sarsıntı).

Ödül v2 (ilk 1M adımlık eğitimden sonra): v1'de robot yerinde durmayı
öğrendi, çünkü durarak da adım başı ~0.8 puan alıyordu (hız komutu 0'a
yakınken hız izleme ~1, dönüş komutu hep 0 olduğundan dönüş terimi bedava
0.5). 1000 adımda ~800 = eğitimin ulaştığı ~810. Düzeltmeler:
  - "progress": komut yönünde gidilen hıza doğrudan puan (komutla sınırlı)
  - hız izleme toleransı 0.05 -> 0.10 m/s (uzaktayken de iyileşme görünsün)
  - "gait": adım saatine göre tripod ritmi — saatin ilk yarısında bir üçlü
    grup havada, diğeri yerde; ikinci yarıda tersi. Ayak teması YALNIZCA
    ödülde (gözlemde değil). Yol gösterici; hareketi politika öğreniyor.
  - hız komutu en az 0.05 m/s; dönüş ağırlığı 0.5 -> 0.2
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
    vx_range: tuple[float, float] = (0.05, 0.15)  # m/s; en az 0.05: durmak hep kaybettirsin
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
        "progress": 10.0,      # komut yönünde hız (m/s), komutla sınırlı
        "gait": 0.5,           # tripod ritmi (0..1)
        "yaw_rate": 0.2,
        "orientation": -2.0,   # gövdenin yatması
        "height": -20.0,       # yükseklik sapması (m^2)
        "power": -0.02,        # mekanik güç, W (enerji)
        "action_rate": -0.01,  # sarsıntı
        "fall": -10.0,         # devrilince bir kez
    })
    lin_vel_sigma: float = 0.10    # m/s
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


def tripod_groups(yaws: dict[int, float]) -> tuple[tuple[int, ...], tuple[int, ...]]:
    """Bacakları gövde etrafındaki açıya göre dizip sırayla iki üçlüye ayır.

    yaws: bacak kimliği -> montaj açısı (rad). Ayak numaralarına elle
    bağlanmıyor; komşu iki bacak hiçbir zaman aynı grupta olmaz.
    """
    order = sorted(yaws, key=lambda leg: yaws[leg])
    return tuple(sorted(order[0::2])), tuple(sorted(order[1::2]))


def gait_score(contact, phase: float, groups) -> float:
    """Tripod ritmine uyum, 0..1: saatin ilk yarısında groups[0] havada,
    groups[1] yerde; ikinci yarısında tersi. contact: bacak sırasıyla (0..5)."""
    swing, stance = (groups[0], groups[1]) if phase < 0.5 else (groups[1], groups[0])
    ok = sum(not contact[i] for i in swing) + sum(bool(contact[i]) for i in stance)
    return ok / (len(swing) + len(stance))


def reward(state: SimState, action, prev_action, command: tuple[float, float, float],
           cfg: TaskConfig, fell: bool, phase: float = 0.0,
           groups=((0, 2, 4), (1, 3, 5))) -> tuple[float, dict[str, float]]:
    vx, vy, _ = state.lin_vel_in_base()
    wz = state.ang_vel_in_base()[2]
    g = state.gravity_in_base()
    ex, ey = command[0] - vx, command[1] - vy
    terms = {
        "lin_vel": math.exp(-(ex * ex + ey * ey) / cfg.lin_vel_sigma ** 2),
        "progress": _progress(vx, vy, command),
        "gait": gait_score(state.foot_contact, phase, groups),
        "yaw_rate": math.exp(-((command[2] - wz) ** 2) / cfg.yaw_rate_sigma ** 2),
        "orientation": g[0] ** 2 + g[1] ** 2,
        "height": (state.base_pos[2] - cfg.stand_height_mm / 1000.0) ** 2,
        "power": sum(abs(t * v) for t, v in zip(state.joint_effort, state.joint_vel)
                     if math.isfinite(v)),
        "action_rate": sum((a - b) ** 2 for a, b in zip(action, prev_action)),
        "fall": 1.0 if fell else 0.0,
    }
    weighted = {k: cfg.w[k] * v for k, v in terms.items()}
    return sum(weighted.values()), weighted


def _progress(vx: float, vy: float, command) -> float:
    """Komut yönündeki hız, [0, |komut|] aralığına kırpılmış (m/s)."""
    speed = math.hypot(command[0], command[1])
    if speed < 1e-9:
        return 0.0
    along = (vx * command[0] + vy * command[1]) / speed
    return max(0.0, min(speed, along))
