"""PPO eğitimi (Stable-Baselines3), paralel süreç içi Gazebo ortamlarıyla.

    python -m hexapod_rl.train --steps 1000000 --envs 8 --name deneme1
    python -m hexapod_rl.train --steps 3000000 --envs 16 --name v3 --residual --omni --init-from models/ppo_omni_250k/model.zip
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

--curriculum AD: kolaydan zora müfredat (terrain_probe.CURRICULA[AD]; ortam
başına bir zemin türü ve seviyeleri, env.HexapodEnv terrain_levels). Her
ortam en kolay seviyeden başlar, başardıkça zorlaşır. Ortalama seviye her
güncellemede progress.csv'ye yazılır (mufredat/<tür>, 0 en kolay).
--terrains ile birlikte verilmez; ara kayıt seçimi aynı adın EVAL_CASES'ini
ölçer.

--reflex AÇI: mesafe sensörlü kaldırma refleksi açık (hexapod_policy.lift_reflex;
DENEYSEL yerleşim: gövde kenarında ileri ve ±90°, AÇI derece aşağı;
reflex_probe.ring_sensors). Taban tripod'un kaldırmasını refleks seçer,
eklemler buna uyum sağlar; yürüyüş yönüne sensör bakmıyorsa politikanın
kaldırma çıkışı (robottaki PolicyController ile aynı). --range-noise /
--range-drop (%): ölçüm gürültüsü ve düşen okuma. Ara kayıt seçimi de
refleksle ölçer.

En iyi ara kayıt (2026-09-26): uzun eğitimde politika yine hedef hızı
aşmaya kayıyordu, en iyisi 250k ara kaydıydı (ders 25). Her ara kayıtta
(250 bin adımda bir) politika deterministik ölçülür (evaluate.eval_commands:
ileride 0.05/0.10/0.15, her yönde yedi komut; düz zemin, rastgeleleştirme
yok, 10 s); adım başı ödüllerin ortalaması en yüksek olan best_model.zip
olarak saklanır, bütün ölçümler ara_degerlendirme.csv'ye yazılır.
--terrains ile eğitimde zemin durumları da ölçülür (terrain_probe.EVAL_CASES;
her biri kendi Gazebo dünyasında, kurulup kapatılır): yalnız düz zemine
bakan seçim zeminde en iyi ara kaydı kaçırıyordu (v13: 250k seçildi, zeminde
en iyisi 2.25M; bu ölçütle 2.25M seçilir).

Çıktılar ~/hexapod_runs/<ad>/ altında (OneDrive'a senkronlanmasın diye
depoda değil): model.zip (son), best_model.zip (en iyi ara kayıt), ara
kayıtlar (checkpoints/), progress.csv (SB3 günlüğü), ara_degerlendirme.csv,
degerlendirme.txt, gorev.json (görev ayarı: eylem modu, komut aralıkları,
taban yürüyüş, ödül, rastgeleleştirme; evaluate/export/terrain_probe modeli
bununla ölçer ve aktarır, bayrak gerekmez). Ortam: tools/wsl/rl_kurulum.sh.

--lift-mm: artık eylemde taban tripod'un ayak kaldırması (varsayılan 25).
Ölçüldü (2026-09-26, düzeltmesiz tripod): 25 mm'de 45 mm basamak ve 50 mm
yayladan iniş takılıyor; 40 mm'de ikisi de geçiliyor, 60 mm'de 60 mm basamak
da; düz zeminde ödül aynı, güç 1.9 -> 3.3 W.

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


def measure_model(model, task, commands, terrain_cases=(), seconds: float = 10.0,
                  env=None, env_kwargs: dict | None = None) -> tuple[float, dict]:
    """Ara kayıt seçiminin ölçütü: düz zemindeki komutlar (env yeniden kullanılır)
    + zemin durumları (her biri kendi dünyasında, kurulup kapatılır), hepsi
    deterministik ve rastgeleleştirmesiz; skor adım başı ödüllerin ortalaması.
    Ödül hedef hızı aşmayı (lin_vel) ve enerjiyi cezalandırdığı için yalnız
    ilerlemeye bakan bir ölçütün düştüğü tuzağa düşmez (ders 30)."""
    from .env import HexapodEnv

    results = evaluate_set(model, commands, seconds, env=env, task=task)
    row = {f"odul_{c[0]:+.2f}_{c[1]:+.2f}_{c[2]:+.2f}": r["adim_basi_odul"]
           for c, r in zip(commands, results)}
    fell = sum(r["devrildi"] for r in results)
    for label, sdf, height, cmd in terrain_cases:
        e = HexapodEnv(task=task, terrain_sdf=sdf, terrain_height=height, **(env_kwargs or {}))
        try:
            r = evaluate(model, seconds, vx=cmd[0], vy=cmd[1], wz=cmd[2], env=e)
        finally:
            e.close()
        row[f"zemin_{label}"] = r["adim_basi_odul"]
        fell += r["devrildi"]
    score = sum(row.values()) / len(row)
    return score, {"skor": round(score, 4), "devrilen": fell,
                   **{k: round(v, 4) for k, v in row.items()}}


def best_checkpoint_callback(every: int, out: Path, task, seconds: float = 10.0,
                             terrain_cases=(), env_kwargs: dict | None = None):
    """Her `every` çağrıda politikayı ölçüp (measure_model) en iyisini
    best_model.zip olarak saklar. terrain_cases: (ad, sdf, yükseklik, komut).
    env_kwargs: ölçüm ortamlarına (ör. mesafe sensörü ve refleks)."""
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
                self.env = HexapodEnv(task=task, **(env_kwargs or {}))
            score, cols = measure_model(self.model, task, commands, terrain_cases, seconds,
                                        env=self.env, env_kwargs=env_kwargs)
            row = {"adim": self.num_timesteps, **cols}
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


def curriculum_log_callback(labels: list[str]):
    """Her güncelleme turunda müfredat türü başına ortalama zemin seviyesini
    günlüğe yazar (mufredat/<tür>; 0 en kolay). labels: ortam başına tür adı,
    None seviyeli (müfredatsız) ortamlar için None."""
    from stable_baselines3.common.callbacks import BaseCallback

    class CurriculumLog(BaseCallback):
        def _on_step(self) -> bool:
            return True

        def _on_rollout_end(self) -> None:
            levels = self.training_env.get_attr("level")
            for label in sorted({lb for lb in labels if lb is not None}):
                vals = [lv for lv, lb in zip(levels, labels) if lb == label]
                self.logger.record(f"mufredat/{label}", sum(vals) / len(vals))

    return CurriculumLog()


def main(argv: list[str] | None = None) -> int:
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor

    from dataclasses import replace

    from .env import make_env
    from .task import (TASK_FILE, TaskConfig, body_mass_scales, standard_reward, task_from_flags,
                       task_to_json)

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
    parser.add_argument("--curriculum", default=None,
                        help="kolaydan zora zemin müfredatı (terrain_probe.CURRICULA)")
    parser.add_argument("--reflex", type=float, default=None, metavar="AÇI",
                        help="mesafe sensörlü kaldırma refleksi; sensörler bu kadar derece aşağı")
    parser.add_argument("--range-noise", type=float, default=0.0,
                        help="mesafe ölçümüne bağıl gürültü, %%")
    parser.add_argument("--range-drop", type=float, default=0.0,
                        help="mesafe okumasının gelmeme olasılığı, %%")
    parser.add_argument("--lift-mm", type=float, default=None,
                        help="artık eylemde taban tripod'un ayak kaldırması (varsayılan TaskConfig)")
    parser.add_argument("--lift-range", type=float, nargs=2, default=None, metavar=("EN_AZ", "EN_COK"),
                        help="öğrenilmiş ayak kaldırma aralığı, mm (eylem 19 boyutlu; "
                             "başlangıç modeli hexapod_rl.widen ile genişletilmeli)")
    parser.add_argument("--lift-std", type=float, default=None,
                        help="ayak kaldırma eyleminin keşif std'si (--std'den sonra uygulanır)")
    parser.add_argument("--overshoot", type=float, default=None,
                        help="ödül v7: komutu aşan hızın progress'ten düşülme katsayısı (1: simetrik)")
    parser.add_argument("--power-weight", type=float, default=None,
                        help="güç cezası ağırlığı, W başına (varsayılan: TaskConfig)")
    parser.add_argument("--std", type=float, default=None,
                        help="keşif gürültüsü std'si (eylem birimi; modeldekini ezer)")
    parser.add_argument("--init-from", type=Path, default=None,
                        help="eğitilmiş model.zip'ten devam et")
    args = parser.parse_args(argv)
    if args.terrains and args.curriculum:
        parser.error("--terrains ile --curriculum birlikte verilmez")

    out = args.out / args.name
    out.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(1)  # paralellik süreçlerde; torch'un iş parçacıkları yarışmasın

    weights = dict(TaskConfig().w)
    if args.power_weight is not None:
        weights["power"] = args.power_weight
    extra = {} if args.overshoot is None else {"progress_overshoot": args.overshoot}
    if args.lift_range is not None:
        extra["lift_action"] = tuple(args.lift_range)
    task = task_from_flags(args.residual, args.omni, args.randomize, args.lift_mm, w=weights,
                           **extra)
    # Ara kayıt seçimi: rastgeleleştirmesiz; ağırlıklar standart ama progress
    # biçimi (v7 aşma cezası) eğitimdeki gibi, seçim hedefle tutarlı olsun.
    eval_task = replace(task, randomization=None, w=dict(TaskConfig().w))
    # Görev ayarı modelin yanında: evaluate/export/terrain_probe bayraksız okur.
    (out / TASK_FILE).write_text(task_to_json(task), encoding="utf-8")
    masses = body_mass_scales(args.envs, task)   # ortam başına gövde kütlesi çarpanı
    terrains = [("düz", "", None)] * args.envs
    levels = [None] * args.envs            # müfredat: ortam başına seviye üreteçleri
    if args.terrains:
        from .terrain_probe import training_terrains
        terrains = training_terrains(args.terrains, args.envs)
    if args.curriculum:
        from .terrain_probe import curriculum_levels
        pairs = curriculum_levels(args.curriculum, args.envs)
        terrains = [(label, "", None) for label, _ in pairs]
        levels = [lv for _, lv in pairs]
    if args.terrains or args.curriculum:
        # kütle sıralı, zemin listesi de sıralı: aynı zemin hep aynı uç kütleye düşmesin
        step = next(k for k in range(args.envs // 2 + 1, args.envs + 1)
                    if math.gcd(k, args.envs) == 1)   # n ile aralarında asal: permütasyon
        masses = [masses[(i * step) % args.envs] for i in range(args.envs)]
    sensor_kwargs = {}
    if args.reflex is not None:
        from hexapod_policy.lift_reflex import LiftReflex

        from .reflex_probe import ring_sensors
        if task.action_mode != "residual":
            parser.error("--reflex yalnız artık eylem modunda (--residual)")
        sensor_kwargs = dict(range_sensors=ring_sensors((0.0, 90.0, -90.0), args.reflex),
                             lift_reflex=LiftReflex(), range_noise=args.range_noise / 100,
                             range_drop=args.range_drop / 100)
    venv = VecMonitor(SubprocVecEnv([make_env(args.seed * 100 + i, task, body_mass_scale=m,
                                              terrain_sdf=sdf, terrain_height=h, terrain_levels=lv,
                                              **sensor_kwargs)
                                     for i, (m, (_, sdf, h), lv)
                                     in enumerate(zip(masses, terrains, levels))],
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
    if args.lift_std is not None:
        if task.lift_action is None:
            parser.error("--lift-std için --lift-range gerekir")
        with torch.no_grad():
            model.policy.log_std[-1] = math.log(args.lift_std)
    (out / "ayarlar.txt").write_text(
        f"learning_rate: {model.learning_rate}\ntarget_kl: {model.target_kl}\n"
        f"use_sde: {model.use_sde}\nrandomize: {args.randomize}\nstd: {args.std}\n"
        f"power_weight: {task.w['power']}\naction_mode: {task.action_mode}\n"
        f"omni: {args.omni}\nenvs: {args.envs}\n"
        f"body_mass_scales: {[round(m, 3) for m in masses]}\n"
        f"terrains: {[t[0] for t in terrains]}\n"
        f"curriculum: {args.curriculum}\n"
        f"reflex: {args.reflex}, range_noise: {args.range_noise} %, "
        f"range_drop: {args.range_drop} %\n",
        encoding="utf-8")
    model.set_logger(configure(str(out), ["csv", "stdout"]))
    every = max(250_000 // args.envs, 1)  # 250 bin adımda bir ara kayıt
    cases = []
    set_name = args.terrains or args.curriculum
    if set_name:   # ara kayıt seçimi zemini de görsün (ders 30)
        from .terrain_probe import EVAL_CASES, eval_cases
        cases = eval_cases(set_name) if set_name in EVAL_CASES else []
    eval_kwargs = {k: v for k, v in sensor_kwargs.items()
                   if k in ("range_sensors", "lift_reflex")}   # ölçüm gürültüsüz
    best = best_checkpoint_callback(every, out, eval_task, terrain_cases=cases,
                                    env_kwargs=eval_kwargs)
    callbacks = [CheckpointCallback(every, str(out / "checkpoints"), "ppo"), best]
    if args.curriculum:
        callbacks.append(curriculum_log_callback(
            [t[0] if lv is not None else None for t, lv in zip(terrains, levels)]))
    t0 = time.time()
    model.learn(total_timesteps=args.steps, reset_num_timesteps=True,
                callback=CallbackList(callbacks))
    wall = time.time() - t0
    model.save(out / "model")
    venv.close()

    result = evaluate(model, task=standard_reward(eval_task))   # rapor: ortak ödül
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
