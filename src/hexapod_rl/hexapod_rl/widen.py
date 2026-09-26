"""Eğitilmiş bir artık eylem modelinin ağına öğrenilmiş ayak kaldırma çıkışı ekle.

    python -m hexapod_rl.widen models/ppo_lift50_3750k/model.zip --lift-range 20 60 \
        --out ~/hexapod_runs/w_lift/model.zip
    python -m hexapod_rl.train ... --lift-range 20 60 --init-from ~/hexapod_runs/w_lift/model.zip

Neden (2026-09-27): taban tripod'un ayak kaldırması sabitken bir ödünleşim
var: 25 mm düzde verimli ama 45 mm'lik engelde takılıyor, 50 mm engelleri
geçiyor ama düzde ~%60 fazla enerji harcıyor (PROJE_DEVIR ders 29, 33).
Politika kaldırmayı kendisi seçerse (task.TaskConfig.lift_action) düzde
düşük, engele takılınca yüksek kaldırmayı öğrenebilir. 18 eklemin ayrı
ayrı keşfiyle bulunamayan bu hareket tek boyutlu bir düğmeyle kolay.

Sıcak başlangıç: aktör ağının son katmanına bir satır eklenir; ağırlıkları 0,
sapması modelin eğitildiği sabit kaldırmaya (gorev.json lift_mm) karşılık
gelen eylem. Eklem çıkışları, kritik ve std'ler kopyalanır: başlangıç
davranışı eski modelle birebir aynı (test). Yeni boyutun std'si --lift-std.
Yanına kaldırma aralığı eklenmiş gorev.json yazılır.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import replace
from pathlib import Path


def widen(src: Path, out: Path, lift_range: tuple[float, float], lift_std: float = 0.3):
    import torch
    from stable_baselines3 import PPO

    from .pretrain import _spaces_only_env
    from .task import (ACTION_SIZE, TASK_FILE, action_dim, action_for_lift, find_task,
                       task_to_json)
    from .train import PPO_KWARGS

    task = find_task(src)
    if task is None:
        raise ValueError(f"{src}: yanında gorev.json yok; taban ayak kaldırması bilinmeden "
                         "sıcak başlangıç kurulamaz")
    if task.action_mode != "residual":
        raise ValueError("öğrenilmiş ayak kaldırma yalnız artık eylem modunda")
    if task.lift_action is not None:
        raise ValueError(f"{src} zaten ayak kaldırma çıkışlı")
    lo, hi = lift_range
    if not lo <= task.lift_mm <= hi:
        raise ValueError(f"modelin kaldırması {task.lift_mm} mm aralığın ({lo}, {hi}) dışında")

    old = PPO.load(src, device="cpu")
    new_task = replace(task, lift_action=(float(lo), float(hi)))
    kwargs = dict(PPO_KWARGS, policy_kwargs=old.policy_kwargs)
    new = PPO("MlpPolicy", _spaces_only_env(action_dim(new_task)), device="cpu", verbose=0,
              **kwargs)
    old_sd, new_sd = old.policy.state_dict(), new.policy.state_dict()
    with torch.no_grad():
        for key, value in old_sd.items():
            if key in ("action_net.weight", "action_net.bias", "log_std"):
                continue
            new_sd[key].copy_(value)
        new_sd["action_net.weight"].zero_()
        new_sd["action_net.weight"][:ACTION_SIZE].copy_(old_sd["action_net.weight"])
        new_sd["action_net.bias"][:ACTION_SIZE].copy_(old_sd["action_net.bias"])
        new_sd["action_net.bias"][ACTION_SIZE] = action_for_lift(task.lift_mm, (lo, hi))
        new_sd["log_std"][:ACTION_SIZE].copy_(old_sd["log_std"])
        new_sd["log_std"][ACTION_SIZE] = math.log(lift_std)
    new.policy.load_state_dict(new_sd)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    new.save(out)
    (out.parent / TASK_FILE).write_text(task_to_json(new_task), encoding="utf-8")
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Modele öğrenilmiş ayak kaldırma çıkışı ekle")
    parser.add_argument("model", type=Path)
    parser.add_argument("--lift-range", type=float, nargs=2, required=True,
                        metavar=("EN_AZ", "EN_COK"))
    parser.add_argument("--lift-std", type=float, default=0.3)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    out = widen(args.model, args.out, tuple(args.lift_range), args.lift_std)
    print(f"yazıldı -> {out} (+ gorev.json)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
