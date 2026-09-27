"""Eğitilmiş (ya da ara kayıt) bir politikayı değerlendir.

    python -m hexapod_rl.evaluate ~/hexapod_runs/AD/model.zip
    python -m hexapod_rl.evaluate ~/hexapod_runs/AD/checkpoints/ppo_2000000_steps.zip --vx 0.1
    python -m hexapod_rl.evaluate tripod --vx 0.1      # karşılaştırma: Samet'in tripod'u
    python -m hexapod_rl.evaluate tripod --noise 0.1   # eyleme gürültü: dayanıklılık
    python -m hexapod_rl.evaluate tripod --randomize   # alan rastgeleleştirme açık
    python -m hexapod_rl.evaluate AD/model.zip --residual --vx 0 --wz 0.4   # yerinde dönüş

Politika deterministik koşturulur; alınan yol, ortalama hız, devrilme,
yön sapması, adım başı ödül, ortalama mekanik güç, tripod ritmine uyum ve
ayakların havada kalma oranı yazılır. Yana ve dönüş komutları için gövde
çerçevesinde ortalama hız (govde_vx/vy) ve açılmış dönüş hızı da yazılır;
komutla doğrudan karşılaştırılır. --noise eyleme N(0, noise) ekler
(eylem birimi; 0.1 = 0.05 rad): servo titremesine ve PPO'nun eğitimde
kendi keşif gürültüsüyle koştuğu koşula dayanıklılığı ölçer. Ayrıntılı
ölçüm aracı Samet'in işi (GOREVLER.md S6); bu yalnızca hızlı bir bakış.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


#: Ara kayıt seçiminde ve taklit sonunda ölçülen komutlar (vx, vy, wz).
EVAL_FORWARD = ((0.05, 0.0, 0.0), (0.10, 0.0, 0.0), (0.15, 0.0, 0.0))
EVAL_OMNI = ((0.10, 0.0, 0.0), (-0.10, 0.0, 0.0), (0.0, 0.06, 0.0), (0.0, -0.06, 0.0),
             (0.0, 0.0, 0.4), (0.0, 0.0, -0.4), (0.10, 0.04, 0.25))


def eval_commands(task) -> tuple:
    """Görevin eğitildiği komutlara uygun ölçüm seti."""
    return EVAL_OMNI if task is not None and task.vy_range[1] > task.vy_range[0] else EVAL_FORWARD


def evaluate_set(model, commands, seconds: float = 10.0, task=None, env=None) -> list[dict]:
    """Aynı ortamda (tek Gazebo) birkaç komutun deterministik ölçümü."""
    from .env import HexapodEnv

    own = env is None
    if own:
        env = HexapodEnv(task=task)
    try:
        return [evaluate(model, seconds, vx=c[0], vy=c[1], wz=c[2], env=env) for c in commands]
    finally:
        if own:
            env.close()


def _yaw(q) -> float:
    """(w, x, y, z) -> yukarı eksen etrafındaki yön, rad."""
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def evaluate(model, seconds: float = 10.0, vx: float = 0.1, seed: int = 123,
             noise: float = 0.0, task=None, terrain_sdf: str = "",
             terrain_height=None, vy: float = 0.0, wz: float = 0.0, env=None,
             body_mass_scale: float = 1.0, perturbation=None) -> dict:
    """env: yeniden kullanılacak bir HexapodEnv (eğitimde ara kayıt seçimi her
    seferinde Gazebo kurmasın diye); verilirse task/terrain/kütle/bozulma yok
    sayılır ve kapatılmaz. perturbation: task.Perturbation (dayanıklılık
    taraması)."""
    import numpy as np

    from .env import HexapodEnv

    own = env is None
    if own:
        env = HexapodEnv(task=task, terrain_sdf=terrain_sdf, terrain_height=terrain_height,
                         body_mass_scale=body_mass_scale, perturbation=perturbation)
    obs, _ = env.reset(seed=seed, options={"command": (vx, vy, wz)})
    if hasattr(model, "reset"):   # iç durumu olan denetleyici (baseline.TripodPolicy)
        model.reset()
    rng = np.random.default_rng(seed)
    x0, y0 = env._state.base_pos[0], env._state.base_pos[1]
    yaw0 = yaw_prev = _yaw(env._state.base_quat)
    steps = int(round(seconds / env.dt))
    fell, gait, airborne, total, power, n = False, 0.0, 0.0, 0.0, 0.0, 0
    body_v = [0.0, 0.0]   # gövde çerçevesinde ortalama hız
    lifts = []            # taban tripod'un ayak kaldırması (öğrenilmiş kaldırmada değişken)
    turned_rad = 0.0      # açılmış (sarılmamış) toplam dönüş
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
        if info.get("lift_mm") is not None:
            lifts.append(info["lift_mm"])
        v = env._state.lin_vel_in_base()
        body_v[0] += v[0]
        body_v[1] += v[1]
        y = _yaw(env._state.base_quat)
        turned_rad += math.atan2(math.sin(y - yaw_prev), math.cos(y - yaw_prev))
        yaw_prev = y
        if terminated:
            fell = True
            break
    dx = info["base_pos"][0] - x0
    dy = info["base_pos"][1] - y0
    yaw1 = _yaw(env._state.base_quat)
    turned = math.degrees(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))
    t = max(n * env.dt, 1e-9)
    if own:
        env.close()
    return {"komut_vx": vx, "komut_vy": vy, "komut_wz": wz, "gurultu": noise,
            "sure_s": n * env.dt, "alinan_yol_m": dx, "yana_kayma_m": dy,
            "ortalama_hiz_m_s": dx / t,
            "toplam_yol_m": math.hypot(dx, dy), "yon_sapmasi_derece": turned, "devrildi": fell,
            "adim_basi_odul": total / max(n, 1), "ortalama_guc_w": power / max(n, 1),
            "ritim_uyumu": gait / max(n, 1), "havadaki_ayak_orani": airborne / max(n, 1),
            # gövde çerçevesinde (komutla doğrudan karşılaştırılır; dönerken de anlamlı)
            "govde_vx_m_s": body_v[0] / max(n, 1), "govde_vy_m_s": body_v[1] / max(n, 1),
            "donus_hizi_rad_s": turned_rad / t,
            "ayak_kaldirma_mm": sum(lifts) / len(lifts) if lifts else float("nan")}


def format_result(result: dict) -> str:
    return "\n".join(f"{k}: {v:.3f}" if isinstance(v, float) else f"{k}: {v}"
                     for k, v in result.items())


def main(argv: list[str] | None = None) -> int:
    from stable_baselines3 import PPO

    parser = argparse.ArgumentParser(description="Politikayı değerlendir")
    parser.add_argument("model", help="model.zip yolu ya da 'tripod'")
    parser.add_argument("--vx", type=float, default=0.1)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--noise", type=float, default=0.0, help="eyleme gürültü (std)")
    parser.add_argument("--randomize", action="store_true", help="alan rastgeleleştirme açık")
    parser.add_argument("--residual", action="store_true",
                        help="politika artık eylem modunda (tripod + düzeltme)")
    parser.add_argument("--mass", type=float, default=1.0,
                        help="gövde kütlesi çarpanı (eğitim aralığı task.Randomization)")
    parser.add_argument("--seed", type=int, default=123)
    args = parser.parse_args(argv)
    from dataclasses import replace

    from .task import Randomization, find_task, standard_reward, task_from_flags
    task = task_from_flags(args.residual, randomize=args.randomize)
    if args.model == "tripod":
        from .baseline import TripodPolicy
        model = TripodPolicy()
    else:
        model = PPO.load(Path(args.model), device="cpu")
        trained = find_task(args.model)   # eğitimin gorev.json'ı: eylem modu, taban yürüyüş
        if trained is not None:
            task = standard_reward(replace(
                trained, randomization=Randomization() if args.randomize else None))
            lift = (f"öğrenilmiş ayak kaldırma {task.lift_action[0]:g}–{task.lift_action[1]:g} mm"
                    if task.lift_action is not None else f"ayak kaldırma {task.lift_mm:g} mm")
            print(f"# görev ayarı modelin gorev.json'ından (eylem modu {task.action_mode}, "
                  f"{lift})", file=sys.stderr)
    print(format_result(evaluate(model, args.seconds, args.vx, seed=args.seed,
                                 noise=args.noise, task=task, vy=args.vy, wz=args.wz,
                                 body_mass_scale=args.mass)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
