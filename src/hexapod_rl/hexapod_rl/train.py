"""PPO eğitimi (Stable-Baselines3), paralel süreç içi Gazebo ortamlarıyla.

    python -m hexapod_rl.train --steps 1000000 --envs 8 --name deneme1
    python -m hexapod_rl.train --steps 5000000 --name v3 --init-from models/tork_v2_10M/model.zip
    python -m hexapod_rl.train --steps 10000000 --name v5 --init-from ~/hexapod_runs/bc/model.zip --randomize

--init-from: sıfırdan değil, eğitilmiş bir modelin ağırlıklarından devam
eder (ödül değişince yeniden öğrenmek yerine uyum sağlasın diye). PPO'nun
ayarları o modelden gelir; yalnızca ortam ve günlük yeni. Taklit ile
başlatılmış model de böyle verilir (pretrain.py). --randomize: alan
rastgeleleştirme açık (task.Randomization; servo, gecikme, itme, IMU gürültüsü;
gövde kütlesi ortamlara eşit dağıtılır, task.body_mass_scales).
--power-weight: ödülün güç cezası ağırlığını (W başına) bu eğitim için
değiştirir; varsayılan ödül (task.TaskConfig) değişmez. Enerji deneyleri için.
--std: keşif gürültüsünü (eylem biriminde) kurar; devam eğitiminde
gürültüyü küçültmek için (gSDE'li modelde desteklenmez).
--lr, --target-kl: modelden gelen ayarların üstüne yazar. Neden (2026-09-26):
gSDE'li taklitten 3e-4 ile başlayan PPO'da güncellemeler çok büyüktü (KL
0.06-0.23, kırpılma 0.5-0.7) ve ödül 200 bin adımda 2200'den 1570'e düştü.
target_kl, bir güncellemedeki dönemleri KL bu değeri aşınca keser.

--omni: her yöne yürüyüş (task.OMNI_COMMANDS: ileri/geri, yana, dönüş);
taklit de --omni ile yapılmalı.

--terrains AD: ortam başına zemin (terrain_probe.TRAIN_SETS[AD]; liste
ortamlara sırayla dağıtılır, kütle çarpanları karıştırılır). Zemin ortamın
ömrü boyunca sabit (dünya kurulurken yazılıyor). S5 gelince onun üreteci
aynı biçimde (terrain_sdf, terrain_height) bir liste verecek.

En iyi ara kayıt (2026-09-26): uzun eğitimde politika yine hedef hızı
aşmaya kayıyordu, en iyisi 250k ara kaydıydı (ders 25). Her ara kayıtta
(250 bin adımda bir) politika deterministik ölçülür (evaluate.eval_commands:
ileride 0.05/0.10/0.15, her yönde yedi komut; düz zemin, rastgeleleştirme
yok, 10 s); adım başı ödüllerin ortalaması en yüksek olan best_model.zip
olarak saklanır, bütün ölçümler ara_degerlendirme.csv'ye yazılır.

Çıktılar ~/hexapod_runs/<ad>/ altında (OneDrive'a senkronlanmasın diye
depoda değil): model.zip (son), best_model.zip (en iyi ara kayıt), ara
kayıtlar (checkpoints/), progress.csv (SB3 günlüğü), ara_degerlendirme.csv,
degerlendirme.txt. Ortam: tools/wsl/rl_kurulum.sh.

Sonda kısa bir değerlendirme yapılır (evaluate.py); ara kayıtlar da
`python -m hexapod_rl.evaluate <zip>` ile değerlendirilebilir.
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

from .evaluate import eval_commands, evaluate, evaluate_set, format_result

# Sıfırdan eğitimin PPO ayarları; pretrain.py de modelini bunlarla kurar.
PPO_KWARGS = dict(
    n_steps=256, batch_size=512, n_epochs=5, learning_rate=3e-4,
    gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.0,
    policy_kwargs={"net_arch": [128, 128]},
)


def best_checkpoint_callback(every: int, out: Path, task, seconds: float = 10.0):
    """Her `every` çağrıda politikayı ölçüp en iyisini best_model.zip olarak saklar."""
    from stable_baselines3.common.callbacks import BaseCallback

    from .env import HexapodEnv

    commands = eval_commands(task)

    class BestCheckpoint(BaseCallback):
        def __init__(self) -> None:
            super().__init__()
            self.best = -math.inf
            self.best_step = None
            self.env = None       # ilk ölçümde kurulur (alt süreçler çatallandıktan sonra)
            self.csv = out / "ara_degerlendirme.csv"

        def _on_step(self) -> bool:
            if self.n_calls % every == 0:
                self.measure()
            return True

        def measure(self) -> None:
            if self.env is None:
                self.env = HexapodEnv(task=task)
            results = evaluate_set(self.model, commands, seconds, env=self.env)
            score = sum(r["adim_basi_odul"] for r in results) / len(results)
            row = {"adim": self.num_timesteps, "skor": round(score, 4),
                   "devrilen": sum(r["devrildi"] for r in results)}
            for c, r in zip(commands, results):
                row[f"odul_{c[0]:+.2f}_{c[1]:+.2f}_{c[2]:+.2f}"] = round(r["adim_basi_odul"], 4)
            new = not self.csv.exists()
            with self.csv.open("a", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=list(row), lineterminator="\n")
                if new:
                    w.writeheader()
                w.writerow(row)
            if score > self.best:
                self.best, self.best_step = score, self.num_timesteps
                self.model.save(out / "best_model")
            self.logger.record("eval/skor", score)
            self.logger.record("eval/en_iyi_adim", self.best_step)

        def _on_training_end(self) -> None:
            if self.env is not None:
                self.env.close()

    return BestCheckpoint()


def main(argv: list[str] | None = None) -> int:
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor

    from .env import make_env
    from .task import TaskConfig, body_mass_scales, task_from_flags

    parser = argparse.ArgumentParser(description="Hexapod PPO eğitimi")
    parser.add_argument("--steps", type=int, default=1_000_000)
    parser.add_argument("--envs", type=int, default=8)
    parser.add_argument("--name", default=time.strftime("%Y%m%d-%H%M%S"))
    parser.add_argument("--out", type=Path, default=Path.home() / "hexapod_runs")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--randomize", action="store_true", help="alan rastgeleleştirme")
    parser.add_argument("--lr", type=float, default=None, help="öğrenme hızı (modeldekini ezer)")
    parser.add_argument("--target-kl", type=float, default=None,
                        help="güncelleme KL'si bunu aşınca dönemleri kes")
    parser.add_argument("--residual", action="store_true",
                        help="artık eylem modu (tripod + düzeltme); model de öyle olmalı")
    parser.add_argument("--omni", action="store_true",
                        help="her yöne komut (task.OMNI_COMMANDS)")
    parser.add_argument("--terrains", default=None,
                        help="ortam başına zemin seti (terrain_probe.TRAIN_SETS; S5 gelince onunki)")
    parser.add_argument("--power-weight", type=float, default=None,
                        help="güç cezası ağırlığı, W başına (varsayılan: TaskConfig)")
    parser.add_argument("--std", type=float, default=None,
                        help="keşif gürültüsü std'si (eylem birimi; modeldekini ezer)")
    parser.add_argument("--init-from", type=Path, default=None,
                        help="eğitilmiş model.zip'ten devam et")
    args = parser.parse_args(argv)

    out = args.out / args.name
    out.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(1)  # paralellik süreçlerde; torch'un iş parçacıkları yarışmasın

    weights = dict(TaskConfig().w)
    if args.power_weight is not None:
        weights["power"] = args.power_weight
    task = task_from_flags(args.residual, args.omni, args.randomize, w=weights)
    eval_task = task_from_flags(args.residual, args.omni)   # ölçüm: rastgeleleştirmesiz
    masses = body_mass_scales(args.envs, task)   # ortam başına gövde kütlesi çarpanı
    terrains = [("düz", "", None)] * args.envs
    if args.terrains:
        from .terrain_probe import training_terrains
        terrains = training_terrains(args.terrains, args.envs)
        # kütle sıralı, zemin listesi de sıralı: aynı zemin hep aynı uç kütleye düşmesin
        step = next(k for k in range(args.envs // 2 + 1, args.envs + 1)
                    if math.gcd(k, args.envs) == 1)   # n ile aralarında asal: permütasyon
        masses = [masses[(i * step) % args.envs] for i in range(args.envs)]
    venv = VecMonitor(SubprocVecEnv([make_env(args.seed * 100 + i, task, body_mass_scale=m,
                                              terrain_sdf=sdf, terrain_height=h)
                                     for i, (m, (_, sdf, h)) in enumerate(zip(masses, terrains))],
                                    start_method="fork"))
    if args.init_from:
        model = PPO.load(args.init_from, env=venv, device="cpu", seed=args.seed)
        (out / "baslangic.txt").write_text(str(args.init_from.resolve()) + "\n",
                                           encoding="utf-8")
    else:
        model = PPO("MlpPolicy", venv, device="cpu", seed=args.seed, verbose=0, **PPO_KWARGS)
    if args.lr is not None:
        model.learning_rate = args.lr
        model._setup_lr_schedule()
    if args.target_kl is not None:
        model.target_kl = args.target_kl
    if args.std is not None:
        if model.use_sde:
            parser.error("--std gSDE'li modelde desteklenmiyor (pretrain.py --sde --std kullan)")
        with torch.no_grad():
            model.policy.log_std.fill_(math.log(args.std))
    (out / "ayarlar.txt").write_text(
        f"learning_rate: {model.learning_rate}\ntarget_kl: {model.target_kl}\n"
        f"use_sde: {model.use_sde}\nrandomize: {args.randomize}\nstd: {args.std}\n"
        f"power_weight: {task.w['power']}\naction_mode: {task.action_mode}\n"
        f"omni: {args.omni}\nenvs: {args.envs}\n"
        f"body_mass_scales: {[round(m, 3) for m in masses]}\n"
        f"terrains: {[t[0] for t in terrains]}\n",
        encoding="utf-8")
    model.set_logger(configure(str(out), ["csv", "stdout"]))
    every = max(250_000 // args.envs, 1)  # 250 bin adımda bir ara kayıt
    best = best_checkpoint_callback(every, out, eval_task)
    t0 = time.time()
    model.learn(total_timesteps=args.steps, reset_num_timesteps=True,
                callback=CallbackList([CheckpointCallback(every, str(out / "checkpoints"), "ppo"),
                                       best]))
    wall = time.time() - t0
    model.save(out / "model")
    venv.close()

    result = evaluate(model, task=eval_task)
    lines = [f"adım: {args.steps}, ortam: {args.envs}, süre: {wall / 60:.1f} dk "
             f"({args.steps / wall:.0f} adım/s)"]
    if best.best_step is not None:
        lines.append(f"en iyi ara kayıt: {best.best_step} adım, skor {best.best:.3f} "
                     f"-> best_model.zip (ara_degerlendirme.csv)")
    lines.append("son model, komut vx 0.10:")
    lines.append(format_result(result))
    (out / "degerlendirme.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"kaydedildi -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
