"""Basit deneme zeminleri: eğim ve basamakta tripod ile politikaları karşılaştır (G7).

    python -m hexapod_rl.terrain_probe tripod models/ppo_omni_250k/model.zip
    python -m hexapod_rl.terrain_probe tripod phase AD/best_model.zip:residual --vx 0.1

Model tanımı: "tripod" (Samet'in TripodGait'i), "phase" (düzeltmesiz
PhaseTripod = artık eylem modunda eylem 0; "phase:50" ayağı 50 mm
kaldıran), "<zip>" ya da "<zip>:residual". Modelin yanında gorev.json varsa
(train.py yazar) eylem modu ve taban yürüyüş ondan okunur.

Bu S5'in (Samet, zemin üreteci) yerine geçmez: G7'nin zeminli eğitimi ve
"bitti" ölçümü S5 + S6 ile yapılacak. Buradaki zeminler yalnız beklenti
oluşturmak için (PROJE_DEVIR §14, madde 5) ve zeminli eğitim altyapısının
ilk denemesi için (TRAIN_SETS, train.py --terrains); her biri terrain_sdf +
terrain_height çifti (sim.py), S5'in üreteci de aynı biçimde vermeli.

S5 geldi (2026-09-28): "s5" adlı eğitim seti, müfredat ve ara kayıt ölçümü
hexapod_terrain'in üreteçlerini kullanır (TRAIN_SETS/EVAL_CASES/CURRICULA
["s5"]). Buradaki eski deneme zeminleri ("deneme", "deneme2") depodaki
modellerin eğitildiği ve models/README tablolarının ölçüldüğü zeminler
oldukları için yeniden üretilebilsinler diye kalıyor; aynı adlı S5 zemininden
farklılar (docs/olcumler/README).
"""

from __future__ import annotations

import argparse
import math
from functools import partial

from hexapod_terrain import sets as s5_sets
from hexapod_terrain import terrain as s5

#: Zemin kutularının kalınlığı ve yatay boyu (m); robot 10 s'de ~1-1.5 m gider.
_THICK, _SIZE = 0.2, 8.0


def _surface(mu: float | None) -> str:
    """Zeminin sürtünme katsayısı (<surface><friction>); None: Gazebo varsayılanı
    (1). Ölçüldü (2026-09-26): düz zeminde 1.0-0.15 arası yürüyüşü etkilemiyor,
    kaygan zemin ancak eğimle anlamlı."""
    if mu is None:
        return ""
    if not mu > 0:
        raise ValueError(f"sürtünme katsayısı pozitif olmalı: {mu}")
    return (f"<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction>"
            "</surface>")


def slope(deg: float, axis: str = "x", mu: float | None = None):
    """Orijinden geçen düz eğim. axis "x": +x (ileri) yönünde ALÇALIR (deg > 0
    yokuş aşağı, deg < 0 yokuş yukarı); "y": +y (sol) yönünde alçalır.
    mu: sürtünme (kaygan eğim)."""
    t = math.radians(deg)
    if axis == "x":
        n, rpy = (math.sin(t), 0.0, math.cos(t)), (0.0, t, 0.0)
    elif axis == "y":
        n, rpy = (0.0, math.sin(t), math.cos(t)), (-t, 0.0, 0.0)
    else:
        raise ValueError(f"eksen x ya da y olmalı: {axis!r}")
    c = tuple(-0.5 * _THICK * v for v in n)   # üst yüz orijinden geçsin
    sdf = f"""<model name="ground"><static>true</static><link name="link">
      <collision name="collision"><pose>{c[0]} {c[1]} {c[2]} {rpy[0]} {rpy[1]} {rpy[2]}</pose>
        <geometry><box><size>{_SIZE} {_SIZE} {_THICK}</size></box></geometry>{_surface(mu)}</collision>
      <visual name="visual"><pose>{c[0]} {c[1]} {c[2]} {rpy[0]} {rpy[1]} {rpy[2]}</pose>
        <geometry><box><size>{_SIZE} {_SIZE} {_THICK}</size></box></geometry></visual>
    </link></model>"""
    k = math.tan(t)

    def height(x: float, y: float) -> float:
        return -k * (x if axis == "x" else y)

    return sdf, height


def step(height_m: float, at_x: float = 0.3, angle_deg: float = 0.0):
    """Düz zemin; x >= at_x'te height_m yüksekliğinde bir basamak (robot ileri
    yürürken önce ön ayaklar çıkar). angle_deg: basamak o yöne döndürülür
    (90: +y'de, sola yürüyünce; 180: arkada); at_x o yönde uzaklık."""
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    r = at_x + _SIZE / 2
    cx, cy = r * ca, r * sa
    sdf = f"""<model name="ground"><static>true</static><link name="link">
      <collision name="plane"><geometry><plane><normal>0 0 1</normal>
        <size>100 100</size></plane></geometry></collision>
      <collision name="step"><pose>{cx} {cy} {height_m / 2} 0 0 {a}</pose>
        <geometry><box><size>{_SIZE} {_SIZE} {height_m}</size></box></geometry></collision>
      <visual name="step"><pose>{cx} {cy} {height_m / 2} 0 0 {a}</pose>
        <geometry><box><size>{_SIZE} {_SIZE} {height_m}</size></box></geometry></visual>
    </link></model>"""

    def height(x: float, y: float) -> float:
        u = x * ca + y * sa
        v = -x * sa + y * ca
        return height_m if at_x <= u <= at_x + _SIZE and abs(v) <= _SIZE / 2 else 0.0

    return sdf, height


def _boxes_sdf(boxes, with_plane: bool = True, mu: float | None = None) -> str:
    """boxes: (cx, cy, cz, sx, sy, sz) listesi -> tek bağlantılı statik model."""
    parts = []
    surface = _surface(mu)
    if with_plane:
        parts.append('<collision name="plane"><geometry><plane><normal>0 0 1</normal>'
                     f'<size>100 100</size></plane></geometry>{surface}</collision>')
    for i, (cx, cy, cz, sx, sy, sz) in enumerate(boxes):
        geo = f"<pose>{cx} {cy} {cz} 0 0 0</pose><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
        parts.append(f'<collision name="box{i}">{geo}{surface}</collision>'
                     f'<visual name="box{i}">{geo}</visual>')
    return ('<model name="ground"><static>true</static><link name="link">'
            + "".join(parts) + "</link></model>")


def pit(height_m: float, half: float = 0.35):
    """Çukur: robot |x|, |y| < half karesinde doğar; her yönde height_m'lik
    basamak çıkar (her yöne komutla eğitimde her yön basamağa varır)."""
    L = 4.0
    h = height_m
    sdf = _boxes_sdf([(half + L / 2, 0, h / 2, L, 2 * (half + L), h),
                      (-half - L / 2, 0, h / 2, L, 2 * (half + L), h),
                      (0, half + L / 2, h / 2, 2 * half, L, h),
                      (0, -half - L / 2, h / 2, 2 * half, L, h)])

    def height(x: float, y: float) -> float:
        return h if max(abs(x), abs(y)) >= half else 0.0

    return sdf, height


def plateau(height_m: float, half: float = 0.5):
    """Yayla: robot height_m yüksekliğinde bir karenin üstünde doğar, her
    yönde basamak iner."""
    h = height_m
    sdf = _boxes_sdf([(0, 0, h / 2, 2 * half, 2 * half, h)])

    def height(x: float, y: float) -> float:
        return h if max(abs(x), abs(y)) < half else 0.0

    return sdf, height


def flat(mu: float | None = None):
    """Düz zemin (isteğe bağlı sürtünmeyle: kaygan düz)."""
    return _boxes_sdf([], mu=mu), lambda x, y: 0.0


def rough(height_m: float, seed: int = 0, cell: float = 0.12, extent: float = 1.5,
          mu: float | None = None):
    """Engebe: [-extent, extent]² karesi cell x cell bloklara bölünür, her bloğun
    üstü [0, height_m] aralığından rastgele (aynı tohum aynı zemin); dışı düz.
    Blokların yanları dik: yükseklik fonksiyonu SDF ile birebir aynı."""
    import numpy as np

    n = int(round(2 * extent / cell))
    tops = np.random.default_rng(seed).uniform(0.0, height_m, (n, n))
    base = 0.05                                     # blok z=-base'den üst yüzüne
    boxes = []
    for i in range(n):
        for j in range(n):
            x = -extent + (i + 0.5) * cell
            y = -extent + (j + 0.5) * cell
            h = float(tops[i, j])
            boxes.append((round(x, 6), round(y, 6), round((h - base) / 2, 6), cell, cell,
                          round(h + base, 6)))
    sdf = _boxes_sdf(boxes, mu=mu)

    def height(x: float, y: float) -> float:
        i = math.floor((x + extent) / cell)
        j = math.floor((y + extent) / cell)
        if 0 <= i < n and 0 <= j < n:
            return float(tops[i, j])
        return 0.0

    return sdf, height


def world_sdf(terrain_sdf: str, name: str = "zemin") -> str:
    """Deneme zeminini ROS'lu simin (sim.launch.py world:=...) dünya dosyasına
    göm: hexapod_gazebo/worlds/flat.sdf'teki fizik ve sistem eklentileri
    (IMU, Contact...) aynı, yalnız zemin modeli değişir. ROS'lu sim robotu
    düz zemine göre doğurur: orijini z=0'da olan zeminler (step, pit)."""
    return f"""<?xml version="1.0"?>
<!-- ÜRETİLDİ: hexapod_rl.terrain_probe.world_sdf (deneme zemini; S5'in yerine geçmez) -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    {terrain_sdf}
  </world>
</sdf>
"""


#: Zeminli eğitim denemesi (G7, 2026-09-26): 16 ortama birer zemin. S5 gelince
#: onun üreteciyle değişecek; bu yalnız "politika tripod'un çıkamadığı 45 mm'yi
#: öğrenebiliyor mu" sorusu için. Eğimler her yöne komutla hem inilir hem çıkılır.
TRAIN_SETS = {
    "deneme": (
        ("düz", lambda: ("", None)),
        ("düz", lambda: ("", None)),
        ("çukur 20 mm", lambda: pit(0.020)),
        ("çukur 30 mm", lambda: pit(0.030)),
        ("çukur 40 mm", lambda: pit(0.040)),
        ("çukur 45 mm", lambda: pit(0.045)),
        ("çukur 50 mm", lambda: pit(0.050)),
        ("çukur 55 mm", lambda: pit(0.055)),
        ("çukur 60 mm", lambda: pit(0.060)),
        ("yayla 30 mm", lambda: plateau(0.030)),
        ("yayla 50 mm", lambda: plateau(0.050)),
        ("eğim 10° x", lambda: slope(10.0)),
        ("eğim 20° x", lambda: slope(20.0)),
        ("eğim 15° y", lambda: slope(15.0, "y")),
        ("eğim 25° x", lambda: slope(25.0)),
        ("çukur 45 mm", lambda: pit(0.045)),
    ),
    # G7 "bitti" şartının üç türü (eğim, engebe, kaygan) + basamaklar
    # (2026-09-26). Ölçüm: 15° kaygan yokuşu (μ 0.3) tripod çıkamıyor, geriye
    # kayıp devriliyor; deneme zemini politikası tutunuyor ama ilerlemiyor.
    "deneme2": (
        ("düz", lambda: ("", None)),
        ("kaygan düz μ0.2", lambda: flat(0.2)),
        ("çukur 45 mm", lambda: pit(0.045)),
        ("çukur 55 mm", lambda: pit(0.055)),
        ("çukur 60 mm", lambda: pit(0.060)),
        ("yayla 50 mm", lambda: plateau(0.050)),
        ("eğim 10° x", lambda: slope(10.0)),
        ("eğim 20° x", lambda: slope(20.0)),
        ("kaygan eğim 10° x μ0.25", lambda: slope(10.0, mu=0.25)),
        ("kaygan eğim 15° x μ0.3", lambda: slope(15.0, mu=0.3)),
        ("kaygan eğim 15° y μ0.3", lambda: slope(15.0, "y", mu=0.3)),
        ("kaygan eğim 20° x μ0.4", lambda: slope(20.0, mu=0.4)),
        ("engebe 20 mm", lambda: rough(0.020, seed=11)),
        ("engebe 40 mm", lambda: rough(0.040, seed=12)),
        ("engebe 50 mm", lambda: rough(0.050, seed=13)),
        ("engebe 60 mm", lambda: rough(0.060, seed=14)),
    ),
    # S5'in zeminleri (hexapod_terrain, 2026-09-28): G7'nin asıl eğitimi. 8'lik
    # liste: eski PC 8 ortamla eğitir, 16 ortamda liste iki kez döner. Seviyeler
    # S6'nın ölçüm listesindekilerin (sets.evaluation_set) AYNISI DEĞİL, arası
    # ya da biraz ötesi: ölçüm, eğitimde görülen zeminin ezberini değil
    # genellemeyi göstersin. Engebe yok: kutu sayısı en yavaş ortamı ~2 kat
    # yavaşlatıyor ve paralel eğitim onu bekliyor (ders 42); S6 tablosunda da
    # engebe 40/60'ta bütün denetleyiciler 3/3 geçiyor, zemin görmemiş
    # politikalar dahil. Düz zemin tek ortam (1/8), deneme setlerindeki oranda.
    "s5": (
        ("düz", lambda: s5.flat()),
        ("çukur 55 mm", lambda: s5.pit(0.055)),
        ("basamak 55 mm", lambda: s5.step(0.055)),
        ("merdiven 6x45 mm", lambda: s5.stairs(0.045, 0.28)),
        ("yokuş yukarı 25°", lambda: s5.slope(-25.0)),
        ("kaygan yokuş 12° μ0.3", lambda: s5.slope(-12.0, mu=0.3)),
        ("yayla 55 mm", lambda: s5.plateau(0.055)),
        ("yan eğim 15°", lambda: s5.slope(15.0, "y")),
    ),
}


#: Eğitim içinde ara kayıt seçiminde ölçülen zemin durumları (train.py): her
#: set için (ad, zemin, komut). Ölçüt düz zemin setine ek olarak bunların
#: adım başı ödülü; ödül hedef hızı aşmayı zaten cezalandırıyor (lin_vel).
EVAL_CASES = {
    "deneme": (
        ("çukur 45 ileri", lambda: pit(0.045), (0.10, 0.0, 0.0)),
        ("çukur 45 yana", lambda: pit(0.045), (0.0, 0.06, 0.0)),
        ("çukur 60 geri", lambda: pit(0.060), (-0.10, 0.0, 0.0)),
        ("yayla 50 ileri", lambda: plateau(0.050), (0.10, 0.0, 0.0)),
        ("yokuş yukarı 20", lambda: slope(-20.0), (0.10, 0.0, 0.0)),
    ),
    "deneme2": (
        ("çukur 60 geri", lambda: pit(0.060), (-0.10, 0.0, 0.0)),
        ("çukur 45 yana", lambda: pit(0.045), (0.0, 0.06, 0.0)),
        ("kaygan yokuş 15 μ0.3", lambda: slope(-15.0, mu=0.3), (0.10, 0.0, 0.0)),
        ("kaygan yokuş 20 μ0.4", lambda: slope(-20.0, mu=0.4), (0.10, 0.0, 0.0)),
        ("engebe 40 ileri", lambda: rough(0.040, seed=101), (0.10, 0.0, 0.0)),
        ("engebe 60 yana", lambda: rough(0.060, seed=102), (0.0, 0.06, 0.0)),
    ),
    # Seçim de S6'nın ölçüm zeminlerine bakmasın (ara kayıt seçimi ölçüm setine
    # ayarlanırsa tablo iyimser olur): ara seviyeler ve öteki yönler.
    "s5": (
        ("basamak 50 ileri", lambda: s5.step(0.050), (0.10, 0.0, 0.0)),
        ("merdiven 6x40 ileri", lambda: s5.stairs(0.040, 0.25), (0.10, 0.0, 0.0)),
        ("çukur 50 yana", lambda: s5.pit(0.050), (0.0, 0.06, 0.0)),
        ("çukur 60 geri", lambda: s5.pit(0.060), (-0.10, 0.0, 0.0)),
        ("yayla 45 ileri", lambda: s5.plateau(0.045), (0.10, 0.0, 0.0)),
        ("kaygan yokuş 12 μ0.3", lambda: s5.slope(-12.0, mu=0.3), (0.10, 0.0, 0.0)),
    ),
}


def eval_cases(name: str) -> list[tuple[str, str, object, tuple[float, float, float]]]:
    """Zemin setinin ölçüm durumları: (ad, terrain_sdf, terrain_height, komut)."""
    if name not in EVAL_CASES:
        raise ValueError(f"{name!r} için ölçüm durumu yok; olanlar: {sorted(EVAL_CASES)}")
    return [(label, *make(), cmd) for label, make, cmd in EVAL_CASES[name]]


#: Müfredat (train.py --curriculum; env.HexapodEnv terrain_levels): ortam başına
#: bir zemin türü ve o türün kolaydan zora seviyeleri. Tür listesi ortamlara
#: sırayla dağıtılır; seviyesi None olan tür (düz) müfredatsız ortamdır.
#: "deneme": TRAIN_SETS["deneme"]'nin türleri ve oranları (16 ortamda 8 çukur,
#: 2 yayla, 4 eğim, 2 düz), ama sabit yükseklikler yerine 10 mm'den 60 mm'ye.
_OBSTACLE_MM = (10, 20, 30, 35, 40, 45, 50, 55, 60)
_PITS = tuple(partial(pit, h / 1000) for h in _OBSTACLE_MM)
_PLATEAUS = tuple(partial(plateau, h / 1000) for h in _OBSTACLE_MM)
CURRICULA = {
    "deneme": (
        ("düz", None),
        ("çukur", _PITS),
        ("çukur", _PITS),
        ("yayla", _PLATEAUS),
        ("çukur", _PITS),
        ("eğim x", tuple(partial(slope, d) for d in (5.0, 10.0, 15.0, 20.0, 25.0))),
        ("çukur", _PITS),
        ("eğim y", tuple(partial(slope, d, "y") for d in (5.0, 10.0, 15.0, 20.0))),
    ),
    # S5'in seviyeleri (hexapod_terrain.sets.LEVELS, kolaydan zora); türler ve
    # oranlar TRAIN_SETS["s5"] gibi (engebe yok, düz 1/8).
    "s5": (("düz", None),) + tuple(
        (kind, s5_sets.LEVELS[kind])
        for kind in ("çukur", "basamak", "merdiven", "eğim", "kaygan", "yayla", "yan eğim")),
}


def curriculum_levels(name: str, n: int) -> list[tuple[str, tuple | None]]:
    """n ortama (tür adı, seviye üreteçleri ya da None): listeyi sırayla dağıt."""
    if name not in CURRICULA:
        raise ValueError(f"bilinmeyen müfredat {name!r}; olanlar: {sorted(CURRICULA)}")
    items = CURRICULA[name]
    return [items[i % len(items)] for i in range(n)]


def training_terrains(name: str, n: int) -> list[tuple[str, str, object]]:
    """n ortama (ad, terrain_sdf, terrain_height): listeyi sırayla dağıt."""
    if name not in TRAIN_SETS:
        raise ValueError(f"bilinmeyen zemin seti {name!r}; olanlar: {sorted(TRAIN_SETS)}")
    items = TRAIN_SETS[name]
    out = []
    for i in range(n):
        label, make = items[i % len(items)]
        sdf, height = make()
        out.append((label, sdf, height))
    return out


#: (ad, zemin) — yön robotun ileri (+x) yürüyüşüne göre.
TERRAINS = (
    ("düz", None),
    ("yokuş yukarı 10°", lambda: slope(-10.0)),
    ("yokuş aşağı 10°", lambda: slope(10.0)),
    ("yan eğim 10°", lambda: slope(10.0, "y")),
    ("yokuş yukarı 20°", lambda: slope(-20.0)),
    ("yokuş aşağı 20°", lambda: slope(20.0)),
    ("basamak 15 mm", lambda: step(0.015)),
    ("basamak 30 mm", lambda: step(0.030)),
    ("basamak 45 mm", lambda: step(0.045)),
    ("basamak 60 mm", lambda: step(0.060)),
    ("kaygan yokuş 15° μ0.3", lambda: slope(-15.0, mu=0.3)),
    ("kaygan yokuş 20° μ0.4", lambda: slope(-20.0, mu=0.4)),
    ("engebe 40 mm", lambda: rough(0.040, seed=1)),
    ("engebe 60 mm", lambda: rough(0.060, seed=1)),
)


class _ZeroResidual:
    """Artık eylem modunda eylem 0: PhaseTripod'un kendisi."""

    def predict(self, obs, deterministic=True):
        import numpy as np
        return np.zeros(18, dtype=np.float32), None


def load(spec: str):
    """Model tanımı -> (model, görev ayarı; rastgeleleştirmesiz).

    "tripod" (Samet'in; "tripod:50" adım yüksekliği 50 mm), "phase"
    (düzeltmesiz PhaseTripod), "phase:50" (ayak 50 mm kalkan), "<zip>" ya da
    "<zip>:residual". Modelin yanında gorev.json varsa
    görev ondan (eylem modu, taban yürüyüş); yoksa ":residual" eki."""
    from dataclasses import replace

    from .task import TaskConfig, find_task, standard_reward

    if spec.startswith("tripod"):   # "tripod" ya da "tripod:50" (adım yüksekliği, mm)
        from .baseline import TripodPolicy
        _, _, h = spec.partition(":")
        policy = TripodPolicy(step_height_mm=float(h) if h else None)
        return policy, policy.task   # kendi eylem ölçeğiyle (baseline.ACTION_SCALE)
    if spec.startswith("phase"):
        _, _, lift = spec.partition(":")
        task = TaskConfig(action_mode="residual")
        return _ZeroResidual(), replace(task, lift_mm=float(lift)) if lift else task
    from stable_baselines3 import PPO
    path, _, mode = spec.partition(":")
    trained = find_task(path)
    if trained is not None:
        return PPO.load(path, device="cpu"), standard_reward(replace(trained, randomization=None))
    return PPO.load(path, device="cpu"), TaskConfig(
        action_mode="residual" if mode == "residual" else "absolute")


def main(argv: list[str] | None = None) -> int:
    from .evaluate import evaluate

    parser = argparse.ArgumentParser(description="Eğim ve basamakta karşılaştırma")
    parser.add_argument("models", nargs="+",
                        help='"tripod", "phase[:lift_mm]", "<zip>" ya da "<zip>:residual"')
    parser.add_argument("--vx", type=float, default=0.1)
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args(argv)

    print("| Zemin | " + " | ".join(args.models) + " |")
    print("|---|" + "---|" * len(args.models))
    loaded = [load(m) for m in args.models]
    for name, make in TERRAINS:
        sdf, height = make() if make else ("", None)
        cells = []
        for model, task in loaded:
            r = evaluate(model, args.seconds, args.vx, task=task, terrain_sdf=sdf,
                         terrain_height=height)
            cell = (f"{r['ortalama_hiz_m_s']:.3f} m/s, {r['yon_sapmasi_derece']:+.0f}°, "
                    f"ödül {r['adim_basi_odul']:.2f}, {r['ortalama_guc_w']:.1f} W")
            if r["devrildi"]:
                cell = f"**devrildi** {r['sure_s']:.1f} s'de ({r['alinan_yol_m']:.2f} m)"
            cells.append(cell)
        print(f"| {name} | " + " | ".join(cells) + " |", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
