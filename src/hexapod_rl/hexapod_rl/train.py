"""PPO eğitimi (Stable-Baselines3), paralel süreç içi Gazebo ortamlarıyla.

    python -m hexapod_rl.train --steps 1000000 --envs 8 --name deneme1

Çıktılar ~/hexapod_runs/<ad>/ altında (OneDrive'a senkronlanmasın diye
depoda değil): model.zip, ara kayıtlar (checkpoints/), progress.csv
(SB3 günlüğü), degerlendirme.txt. Ortam: tools/wsl/rl_kurulum.sh.

Sonda kısa bir değerlendirme yapılır: öğrenilen politika ileri hız
komutuyla (vx) deterministik koşturulur; alınan yol, ortalama hız ve
devrilme yazılır. Ayrıntılı ölçüm aracı Samet'in işi (GOREVLER.md S6).
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np


def evaluate(model, seconds: float = 10.0, vx: float = 0.1) -> dict:
    from .env import HexapodEnv

    env = HexapodEnv()
    obs, _ = env.reset(seed=123, options={"command": (vx, 0.0, 0.0)})
    x0 = env._state.base_pos[0]
    steps = int(round(seconds / env.dt))
    fell = False
    for _ in range(steps):
        action, _ = model.predict(obs, deterministic=True)
        obs, _, terminated, truncated, info = env.step(action)
        if terminated:
            fell = True
            break
    dx = info["base_pos"][0] - x0
    env.close()
    return {"komut_vx": vx, "sure_s": seconds, "alinan_yol_m": dx,
            "ortalama_hiz_m_s": dx / seconds, "devrildi": fell}


def main(argv: list[str] | None = None) -> int:
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor

    from .env import make_env

    parser = argparse.ArgumentParser(description="Hexapod PPO eğitimi")
    parser.add_argument("--steps", type=int, default=1_000_000)
    parser.add_argument("--envs", type=int, default=8)
    parser.add_argument("--name", default=time.strftime("%Y%m%d-%H%M%S"))
    parser.add_argument("--out", type=Path, default=Path.home() / "hexapod_runs")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)

    out = args.out / args.name
    out.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(1)  # paralellik süreçlerde; torch'un iş parçacıkları yarışmasın

    venv = VecMonitor(SubprocVecEnv([make_env(args.seed * 100 + i) for i in range(args.envs)],
                                    start_method="fork"))
    model = PPO(
        "MlpPolicy", venv, device="cpu", seed=args.seed, verbose=0,
        n_steps=256, batch_size=512, n_epochs=5, learning_rate=3e-4,
        gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.0,
        policy_kwargs={"net_arch": [128, 128]},
    )
    model.set_logger(configure(str(out), ["csv", "stdout"]))
    every = max(50_000 // args.envs, 1)
    t0 = time.time()
    model.learn(total_timesteps=args.steps,
                callback=CheckpointCallback(every, str(out / "checkpoints"), "ppo"))
    wall = time.time() - t0
    model.save(out / "model")
    venv.close()

    result = evaluate(model)
    lines = [f"adım: {args.steps}, ortam: {args.envs}, süre: {wall / 60:.1f} dk "
             f"({args.steps / wall:.0f} adım/s)"]
    lines += [f"{k}: {v:.3f}" if isinstance(v, float) else f"{k}: {v}" for k, v in result.items()]
    (out / "degerlendirme.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"kaydedildi -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
