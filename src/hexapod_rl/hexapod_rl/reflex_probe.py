"""Mesafe sensörlü kaldırma refleksi ile sabit kaldırmaları deneme zeminlerinde karşılaştır.

    python -m hexapod_rl.reflex_probe
    python -m hexapod_rl.reflex_probe --pitch 20 25 30 --noise 5 --drop 10 --every 2
    python -m hexapod_rl.reflex_probe --model AD/model.zip --fixed 25 35 50

Model öğrenilmiş kaldırmalı olmalı (19 çıkış; varsayılan ppo_kaldirma35_250k:
kaldırmayı 25-55 mm arasında değiştirince düzde de yürüyor). Kaldırma çıkışı
her adımda ya sabit bir değerle ya da hexapod_policy.lift_reflex.LiftReflex'in
kararıyla ezilir; eklemler modelin.

Sensör yerleşimi DENEYSEL (robot.yaml'da null, D8): gövde önünde üç sensör,
x 0.10/0.095 m, y 0/±0.03 m, z 0.02 m, bakış 0/±25°, --pitch derece aşağı,
menzil 1 m. Sensör gürültüsü ve düşmesi bilinmiyor; --noise (bağıl std, %),
--drop (okumanın gelmeme olasılığı, %), --every (kaç kontrol adımında bir
okuma) taranan büyüklükler; --offset σ: eklem başına kalibrasyon ofseti
(robustness.perturbation("ofset", σ, tohum)). Rastgeleleştirme açık, 3 tohum, 0.1 m/s ileri,
10 s. Hücre: ortalama yol (m), 0.4 m'den fazla ilerleyen tohum, ortalama güç,
ortalama kaldırma; D = devrilme.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import replace

DEFAULT_MODEL = "models/ppo_kaldirma35_250k/model.zip"
TERRAINS = (("düz", None, {}), ("basamak 45", "step", {"height_m": 0.045}),
            ("basamak 60", "step", {"height_m": 0.060}),
            ("çukur 45 ileri", "pit", {"height_m": 0.045}),
            ("engebe 40", "rough", {"height_m": 0.04, "seed": 7}),
            ("engebe 60", "rough", {"height_m": 0.06, "seed": 8}),
            ("yokuş 20", "slope", {"deg": -20.0}))
SEEDS = (1, 2, 3)


def front_sensors(pitch_deg: float):
    """DENEYSEL yerleşim: gövde önünde üç sensör (D8'e öneri, robotta bilinmiyor)."""
    from hexapod_policy.lift_reflex import RangeSensor
    return (RangeSensor(0.10, 0.0, 0.02, 0.0, pitch_deg, 1.0),
            RangeSensor(0.095, 0.03, 0.02, 25.0, pitch_deg, 1.0),
            RangeSensor(0.095, -0.03, 0.02, -25.0, pitch_deg, 1.0))


def ring_sensors(yaws_deg, pitch_deg: float, radius: float = 0.10):
    """DENEYSEL yerleşim: gövde kenarında (yarıçap radius) verilen yönlere bakan
    sensörler (her yöne yürüyüşte yerleşim karşılaştırması için)."""
    import math

    from hexapod_policy.lift_reflex import RangeSensor
    return tuple(RangeSensor(radius * math.cos(math.radians(y)), radius * math.sin(math.radians(y)),
                             0.02, float(y), pitch_deg, 1.0) for y in yaws_deg)


def run_case(model_spec: str, mode, terrain: int, seed: int, noise: float = 0.0,
             drop: float = 0.0, every: int = 1, seconds: float = 10.0,
             offset_deg: float = 0.0, command=(0.1, 0.0, 0.0), sensors=None,
             terrain_spec=None) -> dict:
    """mode: sabit kaldırma (mm, float) ya da ("refleks", pitch_deg). sensors:
    yerleşim (verilmezse front_sensors(pitch)); terrain_spec: (üreteç, argümanlar)
    TERRAINS[terrain] yerine. Dönen yol komut yönünde (m)."""
    import numpy as np

    from hexapod_policy.lift_reflex import LiftReflex, covers

    from . import terrain_probe as tp
    from .env import HexapodEnv
    from .robustness import perturbation
    from .task import Randomization, action_for_lift

    model, task = tp.load(model_spec)
    if task.lift_action is None:
        raise ValueError(f"{model_spec}: öğrenilmiş kaldırma çıkışı yok (19 çıkışlı model gerekir)")
    task = replace(task, randomization=Randomization())
    gen, kw = terrain_spec if terrain_spec is not None else TERRAINS[terrain][1:]
    sdf, height = ("", None) if gen is None else getattr(tp, gen)(**kw)
    reflex = isinstance(mode, tuple)
    if sensors is None:
        sensors = front_sensors(mode[1] if reflex else 30.0)
    env = HexapodEnv(task=task, terrain_sdf=sdf, terrain_height=height, range_sensors=sensors,
                     perturbation=perturbation("ofset", offset_deg, seed) if offset_deg else None)
    rng = np.random.default_rng(seed + 1000)
    lift_reflex = LiftReflex()
    try:
        obs, info = env.reset(seed=seed, options={"command": tuple(command)})
        p0 = env._state.base_pos
        speed = math.hypot(command[0], command[1])
        ux, uy = (command[0] / speed, command[1] / speed) if speed > 0 else (1.0, 0.0)
        lift = lift_reflex.low_mm if reflex else float(mode)
        lifts, power, n, fell = [], 0.0, 0, False
        for k in range(int(round(seconds / env.dt))):
            if reflex and k % every == 0:
                ds = []
                for s, d in zip(sensors, info["ranges_m"]):
                    if rng.uniform() < drop:
                        d = s.max_m                       # okuma gelmedi: "görmüyor"
                    elif d < s.max_m:
                        d *= 1.0 + rng.normal(0.0, noise)
                    ds.append(d)
                lift = lift_reflex.update(k * env.dt, lift_reflex.heights(
                    sensors, ds, env._state.gravity_in_base(), task.stand_height_mm / 1000.0,
                    k * env.dt))
            action, _ = model.predict(obs, deterministic=True)
            action = np.array(action)
            if not reflex or covers(sensors, command[0], command[1]):
                action[-1] = action_for_lift(lift, task.lift_action)
            # görmediği yönde (denetleyicideki gibi) politikanın kendi kaldırması
            obs, _, terminated, _, info = env.step(action)
            lifts.append(info["lift_mm"])
            power += info["reward_terms"]["power"] / env.task.w["power"]
            n += 1
            if terminated:
                fell = True
                break
        p1 = env._state.base_pos
        return {"yol_m": (p1[0] - p0[0]) * ux + (p1[1] - p0[1]) * uy, "guc_w": power / n,
                "kaldirma_mm": sum(lifts) / n, "devrildi": fell}
    finally:
        env.close()


def _job(args):
    return args, run_case(*args)


def table(model_spec: str, modes, noise: float, drop: float, every: int,
          workers: int = 16, offset_deg: float = 0.0) -> str:
    jobs = [(model_spec, m, ti, s, noise, drop, every, 10.0, offset_deg) for m in modes
            for ti in range(len(TERRAINS)) for s in SEEDS]
    from .paralel import job_pool

    with job_pool(workers) as pool:
        out = pool.map(_job, jobs, chunksize=1)
    res: dict = {}
    for (_, m, ti, *_), r in out:
        res.setdefault((m, ti), []).append(r)
    lines = ["| Kaldırma | " + " | ".join(t[0] for t in TERRAINS) + " |",
             "|---|" + "---|" * len(TERRAINS)]
    for m in modes:
        cells = []
        for ti in range(len(TERRAINS)):
            rs = res[(m, ti)]
            k = len(rs)
            fell = sum(r["devrildi"] for r in rs)
            cells.append(f"{sum(r['yol_m'] for r in rs) / k:.2f} "
                         f"({sum(r['yol_m'] > 0.4 for r in rs)}/{k}) "
                         f"{sum(r['guc_w'] for r in rs) / k:.2f} W "
                         f"{sum(r['kaldirma_mm'] for r in rs) / k:.0f} mm"
                         + (f" D{fell}" if fell else ""))
        label = f"refleks, {m[1]:g}° aşağı" if isinstance(m, tuple) else f"sabit {m:g} mm"
        lines.append(f"| {label} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Mesafe sensörlü kaldırma refleksi denemesi")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--fixed", type=float, nargs="*", default=[25.0, 50.0],
                        help="karşılaştırma için sabit kaldırmalar, mm")
    parser.add_argument("--pitch", type=float, nargs="*", default=[25.0],
                        help="sensörlerin aşağı bakış açıları, derece")
    parser.add_argument("--noise", type=float, default=0.0, help="bağıl ölçüm gürültüsü, %%")
    parser.add_argument("--drop", type=float, default=0.0, help="okuma gelmeme olasılığı, %%")
    parser.add_argument("--every", type=int, default=1, help="kaç kontrol adımında bir okuma")
    parser.add_argument("--offset", type=float, default=0.0,
                        help="eklem başına kalibrasyon ofseti σ, derece (tohumlu)")
    parser.add_argument("--workers", type=int, default=16)
    args = parser.parse_args(argv)
    modes = list(args.fixed) + [("refleks", p) for p in args.pitch]
    print(table(args.model, modes, args.noise / 100, args.drop / 100, args.every, args.workers,
                args.offset), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
