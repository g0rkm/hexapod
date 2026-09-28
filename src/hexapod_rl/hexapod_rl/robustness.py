"""Dayanıklılık taraması: robota geçişte beklenen ama eğitimde rastgeleleştirilmeyen
hatalarla modeller nerede bozuluyor?

    python -m hexapod_rl.robustness tripod models/ppo_omni_250k/model.zip \
        models/ppo_lift50_3750k/model.zip [--seconds 10] [--workers 16]

Bozulmalar (task.Perturbation): servo sıfırının kalibrasyon hatası (eklem
başına rastgele, std σ), IMU'nun eğik takılması (büyüklük θ, yönü rastgele),
kontrol adımından (20 ms) uzun komut gecikmesi, zayıf servo (durma torku
çarpanı; akü gerilimi düşünce). Eğitimdeki rastgeleleştirme bunların yalnız
bir kısmını kapsıyor (gecikme 0-18 ms, servo gücü x0.8-1.1); ofset ve IMU
eğikliği hiç yok.

Değerler ölçüm değil, TARANAN büyüklükler: gerçek robotta ne kadar
olacakları bilinmiyor (D9, D10). Amaç hangi hataya karşı hassas olunduğunu
bulmak; hassas çıkan hata eğitime rastgeleleştirme olarak eklenir ya da
donanım vardiyasında o değer öncelikle ölçülür.

Her model x zemin x bozulma: 0.1 m/s ileri, deterministik, rastgeleleştirme
kapalı (bozulmanın kendi etkisi). Rastgele yönlü bozulmalar (ofset, IMU)
--seeds tohumla. Hücre: ortalama alınan yol (m) ve devrilme sayısı (D).
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

#: (ad, tür, büyüklük). tür: "yok", "ofset" (σ, derece), "imu" (θ, derece),
#: "gecikme" (ms), "servo" (durma torku çarpanı).
CASES = (
    ("yok", "yok", 0.0),
    ("ofset σ1°", "ofset", 1.0), ("ofset σ2°", "ofset", 2.0), ("ofset σ4°", "ofset", 4.0),
    ("IMU 3°", "imu", 3.0), ("IMU 6°", "imu", 6.0), ("IMU 10°", "imu", 10.0),
    ("gecikme 20 ms", "gecikme", 20.0), ("gecikme 40 ms", "gecikme", 40.0),
    ("gecikme 60 ms", "gecikme", 60.0), ("gecikme 80 ms", "gecikme", 80.0),
    ("servo x0.7", "servo", 0.7), ("servo x0.6", "servo", 0.6), ("servo x0.5", "servo", 0.5),
)

#: (ad, terrain_probe üreteci, argümanlar); None düz zemin.
TERRAINS = (
    ("düz", None, {}),
    ("basamak 45", "step", {"height_m": 0.045}),
    ("engebe 40", "rough", {"height_m": 0.04, "seed": 7}),
)


def perturbation(kind: str, size: float, seed: int):
    """Tarama durumunun bozulması; rastgele yönlüler tohumdan belirlenimci."""
    import numpy as np

    from .task import Perturbation

    rng = np.random.default_rng(seed)
    if kind == "yok":
        return Perturbation()
    if kind == "ofset":
        return Perturbation(joint_offset_deg=tuple(float(v) for v in rng.normal(0.0, size, 18)))
    if kind == "imu":
        azimuth = float(rng.uniform(0.0, 2.0 * math.pi))
        return Perturbation(imu_tilt_deg=(size * math.cos(azimuth), size * math.sin(azimuth)))
    if kind == "gecikme":
        return Perturbation(delay_ms=size)
    if kind == "servo":
        return Perturbation(servo_strength=size)
    raise ValueError(f"bilinmeyen bozulma: {kind}")


def seeds_for(kind: str, n: int) -> tuple[int, ...]:
    """Rastgelelik yalnız ofset ve IMU yönünde; ötekiler tek koşu."""
    return tuple(range(1, n + 1)) if kind in ("ofset", "imu") else (1,)


def _job(args):
    spec, ti, ci, seed, seconds = args
    from . import terrain_probe as tp
    from .evaluate import evaluate

    _, gen, kw = TERRAINS[ti]
    sdf, height = ("", None) if gen is None else getattr(tp, gen)(**kw)
    model, task = tp.load(spec)
    _, kind, size = CASES[ci]
    r = evaluate(model, seconds, vx=0.1, task=task, terrain_sdf=sdf, terrain_height=height,
                 perturbation=perturbation(kind, size, seed))
    return spec, ti, ci, r["alinan_yol_m"], r["devrildi"]


def _name(spec: str) -> str:
    if spec.startswith(("tripod", "phase")):
        return spec
    p = Path(spec.partition(":")[0])
    return p.parent.name if p.name in ("model.zip", "best_model.zip") else p.stem


def run(specs, seconds: float = 10.0, seeds: int = 3, workers: int = 16) -> str:
    """Tarama tablosu (Markdown)."""
    jobs = [(s, ti, ci, seed, seconds) for s in specs for ti in range(len(TERRAINS))
            for ci, (_, kind, _) in enumerate(CASES) for seed in seeds_for(kind, seeds)]
    from .paralel import job_pool

    with job_pool(workers) as pool:
        out = pool.map(_job, jobs, chunksize=1)
    res: dict = {}
    for s, ti, ci, dist, fell in out:
        res.setdefault((s, ti, ci), []).append((dist, fell))
    lines = ["| Model | Zemin | " + " | ".join(c[0] for c in CASES) + " |",
             "|---|---|" + "---|" * len(CASES)]
    for s in specs:
        for ti, (tname, _, _) in enumerate(TERRAINS):
            cells = []
            for ci in range(len(CASES)):
                runs = res[(s, ti, ci)]
                mean = sum(d for d, _ in runs) / len(runs)
                fell = sum(f for _, f in runs)
                cells.append(f"{mean:.2f}" + (f" D{fell}" if fell else ""))
            lines.append(f"| {_name(s)} | {tname} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Dayanıklılık taraması (sabit bozulmalar)")
    parser.add_argument("models", nargs="+",
                        help='"tripod", "phase[:lift_mm]", "<zip>" ya da "<zip>:residual"')
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--seeds", type=int, default=3, help="ofset ve IMU yönü için tohum sayısı")
    parser.add_argument("--workers", type=int, default=16)
    args = parser.parse_args(argv)
    print(run(args.models, args.seconds, args.seeds, args.workers), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
