"""Basit deneme zeminleri: eğim ve basamakta tripod ile politikaları karşılaştır (G7).

    python -m hexapod_rl.terrain_probe tripod models/ppo_res_250k/model.zip:residual
    python -m hexapod_rl.terrain_probe tripod phase AD/best_model.zip:residual --vx 0.1

Model tanımı: "tripod" (Samet'in TripodGait'i), "phase" (düzeltmesiz
PhaseTripod = artık eylem modunda eylem 0), "<zip>" (mutlak eylem) ya da
"<zip>:residual" (artık eylem).

Bu S5'in (Samet, zemin üreteci) yerine geçmez: G7'nin zeminli eğitimi ve
"bitti" ölçümü S5 + S6 ile yapılacak. Buradaki iki zemin yalnız beklenti
oluşturmak için (PROJE_DEVIR §14, madde 5); her biri terrain_sdf +
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
)


class _ZeroResidual:
    """Artık eylem modunda eylem 0: PhaseTripod'un kendisi."""

    def predict(self, obs, deterministic=True):
        import numpy as np
        return np.zeros(18, dtype=np.float32), None


def load(spec: str):
    """Model tanımı -> (model, artık eylem mi)."""
    if spec == "tripod":
        from .baseline import TripodPolicy
        return TripodPolicy(), False
    if spec == "phase":
        return _ZeroResidual(), True
    from stable_baselines3 import PPO
    path, _, mode = spec.partition(":")
    return PPO.load(path, device="cpu"), mode == "residual"


def main(argv: list[str] | None = None) -> int:
    from .evaluate import evaluate
    from .task import TaskConfig

    parser = argparse.ArgumentParser(description="Eğim ve basamakta karşılaştırma")
    parser.add_argument("models", nargs="+", help='"tripod", "phase", "<zip>" ya da "<zip>:residual"')
    parser.add_argument("--vx", type=float, default=0.1)
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args(argv)

    print("| Zemin | " + " | ".join(args.models) + " |")
    print("|---|" + "---|" * len(args.models))
    loaded = [load(m) for m in args.models]
    for name, make in TERRAINS:
        sdf, height = make() if make else ("", None)
        cells = []
        for model, residual in loaded:
            task = TaskConfig(action_mode="residual" if residual else "absolute")
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
