"""Basit deneme zeminleri: eğim ve basamakta tripod ile politikaları karşılaştır (G7).

    python -m hexapod_rl.terrain_probe tripod models/ppo_res_250k/model.zip:residual
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
"""

from __future__ import annotations

import argparse
import math

#: Zemin kutularının kalınlığı ve yatay boyu (m); robot 10 s'de ~1-1.5 m gider.
_THICK, _SIZE = 0.2, 8.0


def slope(deg: float, axis: str = "x"):
    """Orijinden geçen düz eğim. axis "x": +x (ileri) yönünde ALÇALIR (deg > 0
    yokuş aşağı, deg < 0 yokuş yukarı); "y": +y (sol) yönünde alçalır."""
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
        <geometry><box><size>{_SIZE} {_SIZE} {_THICK}</size></box></geometry></collision>
      <visual name="visual"><pose>{c[0]} {c[1]} {c[2]} {rpy[0]} {rpy[1]} {rpy[2]}</pose>
        <geometry><box><size>{_SIZE} {_SIZE} {_THICK}</size></box></geometry></visual>
    </link></model>"""
    k = math.tan(t)

    def height(x: float, y: float) -> float:
        return -k * (x if axis == "x" else y)

    return sdf, height


def step(height_m: float, at_x: float = 0.3):
    """Düz zemin; x >= at_x'te height_m yüksekliğinde bir basamak (robot ileri
    yürürken önce ön ayaklar çıkar)."""
    cx = at_x + _SIZE / 2
    sdf = f"""<model name="ground"><static>true</static><link name="link">
      <collision name="plane"><geometry><plane><normal>0 0 1</normal>
        <size>100 100</size></plane></geometry></collision>
      <collision name="step"><pose>{cx} 0 {height_m / 2} 0 0 0</pose>
        <geometry><box><size>{_SIZE} {_SIZE} {height_m}</size></box></geometry></collision>
      <visual name="step"><pose>{cx} 0 {height_m / 2} 0 0 0</pose>
        <geometry><box><size>{_SIZE} {_SIZE} {height_m}</size></box></geometry></visual>
    </link></model>"""

    def height(x: float, y: float) -> float:
        return height_m if x >= at_x else 0.0

    return sdf, height


def _boxes_sdf(boxes, with_plane: bool = True) -> str:
    """boxes: (cx, cy, cz, sx, sy, sz) listesi -> tek bağlantılı statik model."""
    parts = []
    if with_plane:
        parts.append('<collision name="plane"><geometry><plane><normal>0 0 1</normal>'
                     '<size>100 100</size></plane></geometry></collision>')
    for i, (cx, cy, cz, sx, sy, sz) in enumerate(boxes):
        geo = f"<pose>{cx} {cy} {cz} 0 0 0</pose><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
        parts.append(f'<collision name="box{i}">{geo}</collision><visual name="box{i}">{geo}</visual>')
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
}


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

    from .task import TaskConfig, find_task

    if spec.startswith("tripod"):   # "tripod" ya da "tripod:50" (adım yüksekliği, mm)
        from .baseline import TripodPolicy
        _, _, h = spec.partition(":")
        return TripodPolicy(step_height_mm=float(h) if h else None), TaskConfig()
    if spec.startswith("phase"):
        _, _, lift = spec.partition(":")
        task = TaskConfig(action_mode="residual")
        return _ZeroResidual(), replace(task, lift_mm=float(lift)) if lift else task
    from stable_baselines3 import PPO
    path, _, mode = spec.partition(":")
    trained = find_task(path)
    if trained is not None:
        return PPO.load(path, device="cpu"), replace(trained, randomization=None,
                                                     w=dict(TaskConfig().w))
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
