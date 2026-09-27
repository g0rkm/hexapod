"""Eğitilmiş politikayı Pi'de torch'suz çalışacak biçime aktar (GOREVLER.md G8).

    python -m hexapod_rl.export models/ppo_omni_250k/model.zip   # -> models/ppo_omni_250k/policy.npz
    python -m hexapod_rl.export model.zip --out /yol/policy.npz

SB3 PPO'nun aktör ağı (gözlem -> eylem ortalaması) numpy dizilerine yazılır;
hexapod_policy.MlpPolicy okur. Kritik (değer ağı) ve std alınmaz: robotta
politika deterministik koşar. Politikanın eğitildiği sözleşme de dosyaya
girer (eylem ölçeği, varsayılan duruş, adım saati, komut hızı, komut
aralıkları); düğüm bunları buradan alır. Sözleşme şimdiki TaskConfig'ten
okunur: eğitimden sonra TaskConfig'in bu alanları değiştiyse eski modeli
aktarırken dikkat (ödül değişikliği sözleşmeyi etkilemez).

Aktarımdan sonra dosya geri okunur ve rastgele gözlemlerde SB3'ün
deterministik çıktısıyla karşılaştırılır; fark 1e-5'i geçerse hata.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from hexapod_policy.mlp import MlpPolicy, PolicyContract

from .task import ACTION_SIZE, OBS_SIZE, TaskConfig, find_task, task_from_flags

_ACTIVATIONS = {"Tanh": "tanh", "ReLU": "relu"}


def contract_for(task: TaskConfig, config_path=None) -> PolicyContract:
    from hexapod_description.interface import COMMAND_RATE_HZ
    from hexapod_driver.config import RobotConfig
    from hexapod_gazebo.pose import standing_pose
    from hexapod_kinematics import HexapodKinematics

    from .task import tripod_groups

    kin = HexapodKinematics.from_config(RobotConfig.load(config_path))
    pose = standing_pose(kin, task.stand_reach_mm, task.stand_height_mm)
    default = tuple(math.radians(v) for leg in sorted(pose) for v in pose[leg].as_dict().values())
    residual = task.action_mode == "residual"
    base = None
    if residual:
        groups = tripod_groups({leg: m.yaw for leg, m in kin.mounts.items()})
        base = {"groups": [list(g) for g in groups], "reach_mm": task.stand_reach_mm,
                "height_mm": task.stand_height_mm, "lift_mm": task.lift_mm}
    return PolicyContract(
        obs_size=OBS_SIZE, action_size=ACTION_SIZE, action_scale=task.action_scale,
        gait_hz=task.gait_hz, control_hz=COMMAND_RATE_HZ, default_rad=default,
        command_ranges={"vx": tuple(task.vx_range), "vy": tuple(task.vy_range),
                        "wz": tuple(task.wz_range)},
        action_mode=task.action_mode,
        residual_scale=task.residual_scale if residual else 0.0,
        base_gait=base,
        # eğitimde çekilen en küçük komutun yarısı: altında düğüm ayakta bekler
        command_deadband=0.5 * task.min_command_frac,
        lift_range=tuple(task.lift_action) if task.lift_action is not None else None,
    )


def from_sb3(model, contract: PolicyContract, source: str = "") -> MlpPolicy:
    """SB3 PPO (MlpPolicy) -> MlpPolicy. Yalnız aktör yolu: policy_net + action_net."""
    p = model.policy
    activation = _ACTIVATIONS.get(p.activation_fn.__name__)
    if activation is None:
        raise ValueError(f"desteklenmeyen aktivasyon: {p.activation_fn.__name__}")
    linears = [m for m in p.mlp_extractor.policy_net if hasattr(m, "weight")] + [p.action_net]
    layers = [(m.weight.detach().cpu().numpy(), m.bias.detach().cpu().numpy()) for m in linears]
    return MlpPolicy(layers, activation, contract, source)


def export(zip_path: Path, out: Path | None = None, check: int = 256,
           task: TaskConfig | None = None) -> Path:
    from stable_baselines3 import PPO

    zip_path = Path(zip_path)
    out = Path(out) if out else zip_path.with_name("policy.npz")
    model = PPO.load(zip_path, device="cpu")
    policy = from_sb3(model, contract_for(task or TaskConfig()), source=zip_path.name)
    policy.save(out)

    loaded = MlpPolicy.load(out)
    rng = np.random.default_rng(0)
    obs = rng.normal(0.0, 1.0, (check, OBS_SIZE)).astype(np.float32)
    ref, _ = model.predict(obs, deterministic=True)            # SB3 [-1, 1]'e kırpar
    ours = np.clip(np.stack([loaded(o) for o in obs]), -1.0, 1.0)
    if ours.shape != ref.shape:
        raise RuntimeError(f"aktarılan çıkış {ours.shape}, SB3 {ref.shape}: sözleşme uymuyor")
    err = float(np.max(np.abs(ours - ref)))
    if err > 1e-5:
        raise RuntimeError(f"aktarılan politika SB3'ten farklı: en büyük fark {err:g}")
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="SB3 modelini numpy politikasına aktar")
    parser.add_argument("model", type=Path)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--residual", action="store_true",
                        help="politika artık eylem modunda eğitildi (tripod + düzeltme)")
    parser.add_argument("--omni", action="store_true",
                        help="politika her yöne eğitildi (task.OMNI_COMMANDS)")
    args = parser.parse_args(argv)
    # Eğitimin gorev.json'ı varsa sözleşme ondan (bayraklar yok sayılır): eylem
    # modu, komut aralıkları ve taban yürüyüş eğitimdekiyle aynı olmalı.
    task = find_task(args.model) or task_from_flags(args.residual, args.omni)
    out = export(args.model, args.out, task=task)
    print(f"yazıldı -> {out} ({out.stat().st_size / 1024:.0f} KB); SB3 ile aynı (fark < 1e-5)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
