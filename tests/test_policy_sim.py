"""hexapod_policy uçtan uca: SB3 modeli -> export -> numpy politika -> denetleyici -> Gazebo.

Yalnız gz.sim ve stable_baselines3 olan yerde koşar (WSL, ROS 2 Lyrical +
~/hexapod_venv). ROS kullanılmaz: denetleyici IMU'yu doğrudan simülasyon
durumundan alır. Gözlem sözleşmesinde bir kayma olursa (eğitimdeki gözlemle
düğümünki farklılaşırsa) robot burada yürüyemez.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

pytest.importorskip("gz.sim", reason="gz.sim yok (ROS 2 Lyrical kurulu Linux gerekir)")
pytest.importorskip("stable_baselines3", reason="SB3 yok (tools/wsl/rl_kurulum.sh)")

from hexapod_description import RobotModel  # noqa: E402
from hexapod_description.interface import joint_names  # noqa: E402
from hexapod_driver import RobotConfig  # noqa: E402
from hexapod_policy import MlpPolicy, PolicyController  # noqa: E402
from hexapod_rl.export import export  # noqa: E402
from hexapod_rl.sim import HexapodSim  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
MODEL_ZIP = REPO / "models" / "taklit_bc_v4" / "model.zip"


def _yaw(q) -> float:
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def test_aktarilan_politika_denetleyiciyle_simde_duz_yurur(tmp_path):
    policy = MlpPolicy.load(export(MODEL_ZIP, tmp_path / "policy.npz"))
    model = RobotModel.from_config(RobotConfig.load(REPO / "config" / "robot.yaml"))
    sim = HexapodSim(model, workdir=tmp_path / "sim")
    limits = [(model.limits[(int(n[3]), n.split("_")[1])].lower,
               model.limits[(int(n[3]), n.split("_")[1])].upper) for n in joint_names(model.mounts)]
    c = PolicyController(policy, limits)

    def tick(state, command):
        now = state.time
        c.on_imu(state.base_quat, state.ang_vel_in_base(), now)
        if command is not None:
            c.on_command(*command, now)
        return sim.step(c.tick(now))

    state = sim.reset()
    for _ in range(50):                        # 1 s: komut yok -> ayakta bekler
        state = tick(state, None)
    assert c.status == "komut yok"
    x0, yaw0 = state.base_pos[0], _yaw(state.base_quat)
    for _ in range(200):                       # 4 s, 0.1 m/s
        state = tick(state, (0.1, 0.0, 0.0))
    assert c.status == "yürüyor"
    assert state.base_pos[0] - x0 > 0.7 * 0.1 * 4.0
    assert abs(math.degrees(_yaw(state.base_quat) - yaw0)) < 5.0
    assert state.base_pos[2] > 0.08


def test_artik_eylem_sifir_duzeltmeyle_tripod_gibi_yurur(tmp_path):
    """Artık eylem modunda uçtan uca: düzeltmesi hep 0 olan bir SB3 modeli ->
    export (residual) -> denetleyici (kin ile) -> Gazebo. Eylem 0 = tripod,
    robot tripod gibi yürümeli. Eğitim ortamı da aynı tabanı kullanıyor."""
    import torch
    from stable_baselines3 import PPO

    from hexapod_kinematics import HexapodKinematics
    from hexapod_rl.pretrain import _spaces_only_env
    from hexapod_rl.task import TaskConfig
    from hexapod_rl.train import PPO_KWARGS

    ppo = PPO("MlpPolicy", _spaces_only_env(), device="cpu", seed=0, verbose=0, **PPO_KWARGS)
    with torch.no_grad():
        ppo.policy.action_net.weight.zero_()
        ppo.policy.action_net.bias.zero_()
    ppo.save(tmp_path / "sifir")
    npz = export(tmp_path / "sifir.zip", tmp_path / "policy.npz",
                 task=TaskConfig(action_mode="residual"))
    policy = MlpPolicy.load(npz)
    assert policy.contract.action_mode == "residual"

    config = RobotConfig.load(REPO / "config" / "robot.yaml")
    model = RobotModel.from_config(config)
    sim = HexapodSim(model, workdir=tmp_path / "sim")
    limits = [(model.limits[(int(n[3]), n.split("_")[1])].lower,
               model.limits[(int(n[3]), n.split("_")[1])].upper) for n in joint_names(model.mounts)]
    c = PolicyController(policy, limits, kin=HexapodKinematics.from_config(config))
    state = sim.reset()
    for _ in range(50):
        c.on_imu(state.base_quat, state.ang_vel_in_base(), state.time)
        state = sim.step(c.tick(state.time))
    x0, yaw0 = state.base_pos[0], _yaw(state.base_quat)
    for _ in range(200):
        c.on_imu(state.base_quat, state.ang_vel_in_base(), state.time)
        c.on_command(0.1, 0.0, 0.0, state.time)
        state = sim.step(c.tick(state.time))
    assert state.base_pos[0] - x0 > 0.8 * 0.1 * 4.0
    assert abs(math.degrees(_yaw(state.base_quat) - yaw0)) < 3.0
