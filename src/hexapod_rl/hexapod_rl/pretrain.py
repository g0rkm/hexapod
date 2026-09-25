"""Taklit ile başlatma (behavior cloning): politikaya önce gösterim tripod'unu öğret.

    python -m hexapod_rl.pretrain --name bc
    python -m hexapod_rl.train --steps 5000000 --name v3_bc --init-from ~/hexapod_runs/bc/model.zip

Neden: demo.py açıklaması (dönerek yürüme yerel tepesi). Adımlar:
1. Veri: gösterim (demo.TripodDemo) ortamda koşar. Uygulanan eyleme gürültü
   eklenir, etiket gürültüsüz gösterim eylemidir: gözlemdeki son komutlar
   gösterimden sapsın, politika onları kopyalamayı değil adım saatine ve hız
   komutuna bakmayı öğrensin. Hız komutu her bölümde ortamın aralığından.
2. Aktör: eylem ortalaması gösterim eylemine MSE ile yaklaştırılır.
3. Kritik: aynı bölümlerin indirimli getirileriyle (ortamın ödülü). Yoksa
   PPO'nun değer tahmini sıfırdan başlar, ilk güncellemeler aktörü bozar.
   Süreyle kesilen bölümün son adımları dışarıda (getirileri eksik kalır).
4. Politikanın std'si --std'ye kurulur: PPO'nun keşif gürültüsü.

Model train.py'nin sıfırdan kurduğu PPO ayarlarıyla (PPO_KWARGS) kaydedilir;
çıktılar ~/hexapod_runs/<ad>/: model.zip, ozet.txt (veri, kayıp,
değerlendirme).
"""

from __future__ import annotations

import argparse
import math
import multiprocessing
import time
import warnings
from pathlib import Path

import numpy as np

from .demo import TripodDemo
from .evaluate import evaluate, format_result
from .task import ACTION_SIZE, OBS_SIZE, Randomization, TaskConfig


def demo_for(env) -> TripodDemo:
    """Ortamın duruşu, eylem ölçeği ve adım saatiyle uyumlu gösterim."""
    from hexapod_driver.config import RobotConfig
    from hexapod_kinematics import HexapodKinematics

    t = env.task
    return TripodDemo(HexapodKinematics.from_config(RobotConfig.load()), env.default,
                      t.action_scale, env.groups, t.gait_hz, t.stand_reach_mm,
                      t.stand_height_mm)


def discounted_returns(rewards: list[float], gamma: float) -> list[float]:
    out, g = [0.0] * len(rewards), 0.0
    for i in range(len(rewards) - 1, -1, -1):
        g = rewards[i] + gamma * g
        out[i] = g
    return out


def _collect(job) -> list[dict]:
    """Bir işçi süreç: kendi Gazebo'suyla verilen tohumlardaki bölümleri koşar."""
    from .env import HexapodEnv

    seeds, noise, gamma, tail, task = job
    env = HexapodEnv(task=task)
    demo = demo_for(env)
    episodes = []
    for seed in seeds:
        rng = np.random.default_rng(seed)
        obs, info = env.reset(seed=seed)
        command = info["command"]
        ep = {"obs": [], "act": [], "rew": [], "command": command, "fell": False}
        done = False
        while not done:
            a = np.asarray(demo.action(env._phase, command), dtype=np.float32)
            ep["obs"].append(obs)
            ep["act"].append(a)
            noisy = np.clip(a + rng.normal(0.0, noise, ACTION_SIZE), -1.0, 1.0)
            obs, r, terminated, truncated, _ = env.step(noisy)
            ep["rew"].append(r)
            ep["fell"] = bool(terminated)
            done = terminated or truncated
        ret = discounted_returns(ep["rew"], gamma)
        keep = len(ret) if ep["fell"] else max(len(ret) - tail, 0)
        ep["vobs"], ep["ret"] = ep["obs"][:keep], ret[:keep]
        episodes.append(ep)
    env.close()
    return episodes


def _spaces_only_env():
    """PPO'yu Gazebo kurmadan oluşturmak için yalnız uzayları olan ortam
    (uzaylar env.HexapodEnv ile aynı; train.py yüklerken denetler)."""
    import gymnasium as gym

    class SpacesOnly(gym.Env):
        observation_space = gym.spaces.Box(-np.inf, np.inf, (OBS_SIZE,), np.float32)
        action_space = gym.spaces.Box(-1.0, 1.0, (ACTION_SIZE,), np.float32)

        def reset(self, *, seed=None, options=None):
            return np.zeros(OBS_SIZE, np.float32), {}

        def step(self, action):
            raise NotImplementedError("yalnız uzaylar")

    return SpacesOnly()


def fit(model, data: dict, epochs: int, lr: float, batch: int, log) -> dict:
    """Aktörü gösterim eylemine, kritiği getirilere oturt. Doğrulama ölçüleri döner."""
    import torch
    import torch.nn.functional as F

    p = model.policy
    p.set_training_mode(True)
    t = {k: torch.as_tensor(np.asarray(v), dtype=torch.float32) for k, v in data.items()}
    opt_pi = torch.optim.Adam([*p.mlp_extractor.policy_net.parameters(),
                               *p.action_net.parameters()], lr=lr)
    opt_vf = torch.optim.Adam([*p.mlp_extractor.value_net.parameters(),
                               *p.value_net.parameters()], lr=lr)
    with torch.no_grad():
        p.value_net.bias.fill_(float(t["ret"].mean()))

    def metrics() -> dict:
        with torch.no_grad():
            mean = p.get_distribution(t["val_obs"]).distribution.mean
            v = p.predict_values(t["val_vobs"]).squeeze(-1)
            ret = t["val_ret"]
            return {"aktor_rmse": float(torch.sqrt(F.mse_loss(mean, t["val_act"]))),
                    "kritik_aciklanan_varyans": float(1 - torch.var(ret - v) / torch.var(ret))}

    for epoch in range(1, epochs + 1):
        for idx in torch.randperm(len(t["obs"])).split(batch):
            loss = F.mse_loss(p.get_distribution(t["obs"][idx]).distribution.mean, t["act"][idx])
            opt_pi.zero_grad()
            loss.backward()
            opt_pi.step()
        for idx in torch.randperm(len(t["vobs"])).split(batch):
            loss = F.mse_loss(p.predict_values(t["vobs"][idx]).squeeze(-1), t["ret"][idx])
            opt_vf.zero_grad()
            loss.backward()
            opt_vf.step()
        if epoch % 10 == 0 or epoch == epochs:
            log(f"tur {epoch}: " + ", ".join(f"{k} {v:.4f}" for k, v in metrics().items()))
    p.set_training_mode(False)
    return metrics()


def main(argv: list[str] | None = None) -> int:
    import torch
    from stable_baselines3 import PPO

    from .train import PPO_KWARGS

    parser = argparse.ArgumentParser(description="Taklit ile başlatma (gösterim tripod'u)")
    parser.add_argument("--name", default="bc")
    parser.add_argument("--out", type=Path, default=Path.home() / "hexapod_runs")
    parser.add_argument("--episodes", type=int, default=48)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--noise", type=float, default=0.1, help="uygulanan eyleme gürültü")
    parser.add_argument("--std", type=float, default=0.3, help="PPO'nun başlangıç std'si")
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--batch", type=int, default=1024)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--randomize", action="store_true",
                        help="veriyi alan rastgeleleştirme açık topla (kritik aynı ortamı öğrensin)")
    args = parser.parse_args(argv)

    out = args.out / args.name
    out.mkdir(parents=True, exist_ok=True)
    lines: list[str] = []

    def log(s: str) -> None:
        print(s, flush=True)
        lines.append(s)

    torch.manual_seed(args.seed)
    torch.set_num_threads(1)
    gamma = PPO_KWARGS["gamma"]
    tail = int(round(3 / (1 - gamma)))    # getirinin %95'i bu kadar adımda birikir
    seeds = [args.seed * 1000 + i for i in range(args.episodes)]
    task = TaskConfig(randomization=Randomization()) if args.randomize else TaskConfig()
    jobs = [(seeds[i::args.workers], args.noise, gamma, tail, task) for i in range(args.workers)]
    t0 = time.time()
    with multiprocessing.get_context("fork").Pool(args.workers) as pool:
        episodes = [ep for part in pool.map(_collect, jobs) for ep in part]
    episodes.sort(key=lambda ep: ep["command"])
    steps = sum(len(ep["rew"]) for ep in episodes)
    log(f"veri: {len(episodes)} bölüm, {steps} adım, {time.time() - t0:.0f} s; "
        f"gürültü {args.noise}; devrilen {sum(ep['fell'] for ep in episodes)}; "
        f"rastgeleleştirme {'açık' if args.randomize else 'kapalı'}; "
        f"gösterimin adım başı ödülü {sum(sum(ep['rew']) for ep in episodes) / steps:.3f}")

    val = set(range(len(episodes))[::10])          # her 10 bölümden biri doğrulama

    def stack(key, which):
        return np.concatenate([np.asarray(episodes[i][key], dtype=np.float32)
                               for i in range(len(episodes)) if (i in val) == which])
    data = {"obs": stack("obs", False), "act": stack("act", False),
            "vobs": stack("vobs", False), "ret": stack("ret", False),
            "val_obs": stack("obs", True), "val_act": stack("act", True),
            "val_vobs": stack("vobs", True), "val_ret": stack("ret", True)}

    with warnings.catch_warnings():  # tek ortamlık tampon uyarısı; eğitimde 8 ortam var
        warnings.filterwarnings("ignore", message=".*mini-batch size.*")
        model = PPO("MlpPolicy", _spaces_only_env(), device="cpu", seed=args.seed,
                    verbose=0, **PPO_KWARGS)
    final = fit(model, data, args.epochs, args.lr, args.batch, log)
    with torch.no_grad():
        model.policy.log_std.fill_(math.log(args.std))
    model.save(out / "model")
    log(f"std {args.std}; doğrulama: " + ", ".join(f"{k} {v:.4f}" for k, v in final.items()))

    for vx in (0.05, 0.10, 0.15):
        log(f"--- değerlendirme, komut vx={vx}\n" + format_result(evaluate(model, vx=vx)))
    (out / "ozet.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"kaydedildi -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
