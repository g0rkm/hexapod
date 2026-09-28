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


OMNI_NPZ = REPO / "models" / "ppo_omni_250k" / "policy.npz"


@pytest.mark.parametrize("command", [(-0.1, 0.0, 0.0), (0.0, 0.06, 0.0), (0.0, 0.0, 0.4),
                                     (0.0, 0.0, 0.0)])
def test_her_yon_politikasi_robottaki_koduyla_yurur(tmp_path, command):
    """Depodaki her yön politikası (models/ppo_omni_250k/policy.npz), robotta
    koşacak denetleyiciyle: geri, yana ve yerinde dönüşte komutun en az %70'i;
    sıfır komutta ölü bölge -> ayakta bekler, yerinden oynamaz."""
    from hexapod_kinematics import HexapodKinematics

    policy = MlpPolicy.load(OMNI_NPZ)
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
    p0, yaw0, seconds = state.base_pos, _yaw(state.base_quat), 4.0
    for _ in range(int(seconds * 50)):
        c.on_imu(state.base_quat, state.ang_vel_in_base(), state.time)
        c.on_command(*command, state.time)
        state = sim.step(c.tick(state.time))
    dx, dy = state.base_pos[0] - p0[0], state.base_pos[1] - p0[1]
    turned = math.atan2(math.sin(_yaw(state.base_quat) - yaw0), math.cos(_yaw(state.base_quat) - yaw0))
    vx, vy, wz = command
    if command == (0.0, 0.0, 0.0):
        assert c.status.startswith("dur (komut ölü bölgede")
        assert math.hypot(dx, dy) < 0.005 and abs(turned) < 0.02
        return
    assert c.status == "yürüyor"
    if vx:
        assert dx / (vx * seconds) > 0.7 and abs(dy) < 0.05
    if vy:
        assert dy / (vy * seconds) > 0.7 and abs(dx) < 0.05
    if wz:
        assert turned / (wz * seconds) > 0.7 and math.hypot(dx, dy) < 0.05
    assert state.base_pos[2] > 0.08


@pytest.mark.parametrize("reference", ["yercekimi", "egim"])
def test_refleksli_denetleyici_45_mm_basamagi_cikar(tmp_path, reference):
    """Robottaki yol, ROS'suz: ppo_kaldirma35_250k/policy.npz + mesafe sensörlü
    refleks (DENEYSEL yerleşim) -> denetleyici -> Gazebo, 45 mm basamak. Mesafe
    rangefinder.read ile simden (robotta sürücüden gelecek). Refleks basamağı
    görüp kaldırmayı yükseltmeli ve robot basamağın üstüne çıkmalı; refleks
    olmadan (yalnız politika, ~35 mm) de ölçülüp karşılaştırılır. "govde"
    kipi burada basamağa çıkarken burun kalkınca kaldırmayı erken indirip
    yarı yolda kalıyordu (0.45 m, ders 50); "egim" onu bu geçişte yakalamalı."""
    from hexapod_kinematics import HexapodKinematics
    from hexapod_policy.lift_reflex import LiftReflex, RangeSensor
    from hexapod_rl import rangefinder
    from hexapod_rl.terrain_probe import step

    npz = REPO / "models" / "ppo_kaldirma35_250k" / "policy.npz"
    config = RobotConfig.load(REPO / "config" / "robot.yaml")
    model = RobotModel.from_config(config)
    limits = [(model.limits[(int(n[3]), n.split("_")[1])].lower,
               model.limits[(int(n[3]), n.split("_")[1])].upper) for n in joint_names(model.mounts)]
    sensors = [RangeSensor(0.10, 0.0, 0.02, 0.0, 20.0, 1.0),
               RangeSensor(0.095, 0.03, 0.02, 25.0, 20.0, 1.0),
               RangeSensor(0.095, -0.03, 0.02, -25.0, 20.0, 1.0)]
    sdf, height = step(0.045, at_x=0.3)
    results = {}
    for use_reflex in (False, True):
        sim = HexapodSim(model, workdir=tmp_path / f"sim{int(use_reflex)}", terrain_sdf=sdf,
                         terrain_height=height)
        kw = (dict(range_sensors=sensors, reflex=LiftReflex(reference=reference))
              if use_reflex else {})
        c = PolicyController(MlpPolicy.load(npz), limits,
                             kin=HexapodKinematics.from_config(config), **kw)
        state = sim.reset()
        x0, lifts = state.base_pos[0], []
        for _ in range(500):                                   # 10 s, 0.1 m/s ileri
            now = state.time
            c.on_imu(state.base_quat, state.ang_vel_in_base(), now)
            c.on_command(0.1, 0.0, 0.0, now)
            if use_reflex:
                c.on_ranges(rangefinder.read(sensors, state.base_pos, state.base_quat, height),
                            now)
            state = sim.step(c.tick(now))
            lifts.append(c.lift_mm)
        results[use_reflex] = (state.base_pos[0] - x0, state.base_pos[2], max(lifts))
        sim.close()
    dist, z, top = results[True]
    assert dist > 0.6 and z > 0.13, results                   # basamağın üstünde (0.1 + 0.045)
    assert top > 55.0, results                                # refleks kaldırmayı yükseltti
    assert results[False][2] < 40.0, results                  # yalnız politika: ~35 mm
