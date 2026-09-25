"""PPO eğitimi (Stable-Baselines3), paralel süreç içi Gazebo ortamlarıyla.

    python -m hexapod_rl.train --steps 1000000 --envs 8 --name deneme1
    python -m hexapod_rl.train --steps 5000000 --name v3 --init-from models/tork_v2_10M/model.zip

--init-from: sıfırdan değil, eğitilmiş bir modelin ağırlıklarından devam
eder (ödül değişince yeniden öğrenmek yerine uyum sağlasın diye). PPO'nun
ayarları o modelden gelir; yalnızca ortam ve günlük yeni. Taklit ile
başlatılmış model de böyle verilir (pretrain.py).

Çıktılar ~/hexapod_runs/<ad>/ altında (OneDrive'a senkronlanmasın diye
depoda değil): model.zip, ara kayıtlar (checkpoints/), progress.csv
(SB3 günlüğü), degerlendirme.txt. Ortam: tools/wsl/rl_kurulum.sh.

Sonda kısa bir değerlendirme yapılır (evaluate.py); ara kayıtlar da
`python -m hexapod_rl.evaluate <zip>` ile değerlendirilebilir.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from .evaluate import evaluate, format_result

# Sıfırdan eğitimin PPO ayarları; pretrain.py de modelini bunlarla kurar.
PPO_KWARGS = dict(
    n_steps=256, batch_size=512, n_epochs=5, learning_rate=3e-4,
    gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.0,
    policy_kwargs={"net_arch": [128, 128]},
)


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
    parser.add_argument("--init-from", type=Path, default=None,
                        help="eğitilmiş model.zip'ten devam et")
    args = parser.parse_args(argv)

    out = args.out / args.name
    out.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(1)  # paralellik süreçlerde; torch'un iş parçacıkları yarışmasın

    venv = VecMonitor(SubprocVecEnv([make_env(args.seed * 100 + i) for i in range(args.envs)],
                                    start_method="fork"))
    if args.init_from:
        model = PPO.load(args.init_from, env=venv, device="cpu", seed=args.seed)
        (out / "baslangic.txt").write_text(str(args.init_from.resolve()) + "\n",
                                           encoding="utf-8")
    else:
        model = PPO("MlpPolicy", venv, device="cpu", seed=args.seed, verbose=0, **PPO_KWARGS)
    model.set_logger(configure(str(out), ["csv", "stdout"]))
    every = max(250_000 // args.envs, 1)  # 250 bin adımda bir ara kayıt
    t0 = time.time()
    model.learn(total_timesteps=args.steps, reset_num_timesteps=True,
                callback=CheckpointCallback(every, str(out / "checkpoints"), "ppo"))
    wall = time.time() - t0
    model.save(out / "model")
    venv.close()

    result = evaluate(model)
    lines = [f"adım: {args.steps}, ortam: {args.envs}, süre: {wall / 60:.1f} dk "
             f"({args.steps / wall:.0f} adım/s)"]
    lines.append(format_result(result))
    (out / "degerlendirme.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"kaydedildi -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
