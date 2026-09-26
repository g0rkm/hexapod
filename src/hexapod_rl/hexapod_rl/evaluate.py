"""Eğitilmiş (ya da ara kayıt) bir politikayı değerlendir.

    python -m hexapod_rl.evaluate ~/hexapod_runs/AD/model.zip
    python -m hexapod_rl.evaluate ~/hexapod_runs/AD/checkpoints/ppo_2000000_steps.zip --vx 0.1
    python -m hexapod_rl.evaluate tripod --vx 0.1      # karşılaştırma: Samet'in tripod'u
    python -m hexapod_rl.evaluate tripod --noise 0.1   # eyleme gürültü: dayanıklılık
    python -m hexapod_rl.evaluate tripod --randomize   # alan rastgeleleştirme açık

Politika deterministik koşturulur; alınan yol, ortalama hız, devrilme,
yön sapması, adım başı ödül, ortalama mekanik güç, tripod ritmine uyum ve
ayakların havada kalma oranı yazılır. --noise eyleme N(0, noise) ekler
(eylem birimi; 0.1 = 0.05 rad): servo titremesine ve PPO'nun eğitimde
kendi keşif gürültüsüyle koştuğu koşula dayanıklılığı ölçer. Ayrıntılı
ölçüm aracı Samet'in işi (GOREVLER.md S6); bu yalnızca hızlı bir bakış.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path


def _yaw(q) -> float:
    """(w, x, y, z) -> yukarı eksen etrafındaki yön, rad."""
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def evaluate(model, seconds: float = 10.0, vx: float = 0.1, seed: int = 123,
             noise: float = 0.0, task=None, terrain_sdf: str = "") -> dict:
    import numpy as np

    from .env import HexapodEnv

    env = HexapodEnv(task=task, terrain_sdf=terrain_sdf)
    obs, _ = env.reset(seed=seed, options={"command": (vx, 0.0, 0.0)})
    if hasattr(model, "reset"):   # iç durumu olan denetleyici (baseline.TripodPolicy)
        model.reset()
    rng = np.random.default_rng(seed)
    x0, y0 = env._state.base_pos[0], env._state.base_pos[1]
    yaw0 = _yaw(env._state.base_quat)
    steps = int(round(seconds / env.dt))
    fell, gait, airborne, total, power, n = False, 0.0, 0.0, 0.0, 0.0, 0
    info = {"base_pos": env._state.base_pos}
    for _ in range(steps):
        action, _ = model.predict(obs, deterministic=True)
        if noise > 0:
            action = np.clip(np.asarray(action) + rng.normal(0.0, noise, len(action)), -1, 1)
        obs, r, terminated, _, info = env.step(action)
        n += 1
        total += r
        power += info["reward_terms"]["power"] / env.task.w["power"]
        gait += info["reward_terms"]["gait"] / env.task.w["gait"]
        airborne += sum(not c for c in info["foot_contact"]) / 6
        if terminated:
            fell = True
            break
    dx = info["base_pos"][0] - x0
    dy = info["base_pos"][1] - y0
    yaw1 = _yaw(env._state.base_quat)
    turned = math.degrees(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))
    env.close()
    return {"komut_vx": vx, "gurultu": noise, "sure_s": n * env.dt, "alinan_yol_m": dx, "yana_kayma_m": dy,
            "ortalama_hiz_m_s": dx / max(n * env.dt, 1e-9),
            "toplam_yol_m": math.hypot(dx, dy), "yon_sapmasi_derece": turned, "devrildi": fell,
            "adim_basi_odul": total / max(n, 1), "ortalama_guc_w": power / max(n, 1),
            "ritim_uyumu": gait / max(n, 1), "havadaki_ayak_orani": airborne / max(n, 1)}


def format_result(result: dict) -> str:
    return "\n".join(f"{k}: {v:.3f}" if isinstance(v, float) else f"{k}: {v}"
                     for k, v in result.items())


def main(argv: list[str] | None = None) -> int:
    from stable_baselines3 import PPO

    parser = argparse.ArgumentParser(description="Politikayı değerlendir")
    parser.add_argument("model", help="model.zip yolu ya da 'tripod'")
    parser.add_argument("--vx", type=float, default=0.1)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--noise", type=float, default=0.0, help="eyleme gürültü (std)")
    parser.add_argument("--randomize", action="store_true", help="alan rastgeleleştirme açık")
    parser.add_argument("--seed", type=int, default=123)
    args = parser.parse_args(argv)
    if args.model == "tripod":
        from .baseline import TripodPolicy
        model = TripodPolicy()
    else:
        model = PPO.load(Path(args.model), device="cpu")
    from .task import Randomization, TaskConfig
    task = TaskConfig(randomization=Randomization()) if args.randomize else None
    print(format_result(evaluate(model, args.seconds, args.vx, seed=args.seed,
                                 noise=args.noise, task=task)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
