"""hexapod_rl.env — Gymnasium ortamı. Yalnız gz.sim ve gymnasium olan yerde koşar
(WSL: source /opt/ros/lyrical/setup.bash; source ~/hexapod_venv/bin/activate).
"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("gz.sim", reason="gz.sim yok (ROS 2 Lyrical kurulu Linux gerekir)")
pytest.importorskip("gymnasium", reason="gymnasium yok (tools/wsl/rl_kurulum.sh)")

from hexapod_rl.env import HexapodEnv  # noqa: E402
from hexapod_rl.task import OBS_SIZE  # noqa: E402


@pytest.fixture(scope="module")
def env(tmp_path_factory):
    e = HexapodEnv(workdir=tmp_path_factory.mktemp("env"))
    yield e
    e.close()


def test_sb3_denetimi(env):
    sb3 = pytest.importorskip("stable_baselines3")
    from stable_baselines3.common.env_checker import check_env
    check_env(env, warn=True, skip_render_check=True)
    assert sb3.__version__


def test_reset_sonrasi_ayakta_duruyor(env):
    obs, info = env.reset(seed=0)
    assert obs.shape == (OBS_SIZE,) and obs.dtype == np.float32
    assert env._state.base_pos[2] == pytest.approx(env.task.stand_height_mm / 1000, abs=0.003)
    assert all(env._state.foot_contact)
    assert obs[0:3] == pytest.approx([0.0, 0.0, -1.0], abs=0.01)


def test_sifir_eylemle_devrilmeden_durur(env):
    env.reset(seed=1, options={"command": (0.0, 0.0, 0.0)})
    for _ in range(100):  # 2 s
        obs, r, terminated, truncated, info = env.step(np.zeros(18, dtype=np.float32))
        assert not terminated
        assert np.isfinite(r) and np.all(np.isfinite(obs))
    assert info["base_pos"][2] == pytest.approx(0.100, abs=0.003)
    # hız komutu 0, robot duruyor: hız izleme ödülü neredeyse tam
    assert info["reward_terms"]["lin_vel"] == pytest.approx(env.task.w["lin_vel"], abs=0.05)


def test_ayni_tohum_ayni_bolum(env):
    rng = np.random.default_rng(0)
    actions = rng.uniform(-0.3, 0.3, size=(25, 18)).astype(np.float32)
    runs = []
    for _ in range(2):
        obs, info = env.reset(seed=42)
        seq = [obs]
        for a in actions:
            obs, *_ = env.step(a)
            seq.append(obs)
        runs.append((info["command"], np.stack(seq)))
    assert runs[0][0] == runs[1][0]
    np.testing.assert_allclose(runs[0][1], runs[1][1], atol=1e-6)


def test_bolum_suresi_dolunca_kesilir(env):
    env.reset(seed=2)
    env._steps = env.max_steps - 1
    *_, truncated, _ = env.step(np.zeros(18, dtype=np.float32))
    assert truncated


def test_gosterim_duz_yurur(env):
    """Taklit edilen gösterim (demo.py) ortamda gerçekten dümdüz yürümeli;
    ödül v4'te izleme terimlerinin çoğunu almalı."""
    import math

    from hexapod_rl.pretrain import demo_for

    demo = demo_for(env)
    cmd = (0.1, 0.0, 0.0)
    env.reset(seed=2, options={"command": cmd})
    x0 = env._state.base_pos[0]
    q = env._state.base_quat
    yaw0 = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
    yaw_terms, n = 0.0, int(round(4.0 / env.dt))
    for _ in range(n):
        _, _, terminated, _, info = env.step(np.asarray(demo.action(env._phase, cmd)))
        assert not terminated
        yaw_terms += info["reward_terms"]["yaw_rate"]
    q = env._state.base_quat
    yaw1 = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
    assert env._state.base_pos[0] - x0 > 0.7 * cmd[0] * 4.0
    assert abs(math.degrees(yaw1 - yaw0)) < 5.0
    assert yaw_terms / n > 0.9 * env.task.w["yaw_rate"]


def test_alan_rastgelelestirme(tmp_path_factory):
    from hexapod_rl.task import Randomization, TaskConfig

    r = Randomization()
    e = HexapodEnv(task=TaskConfig(randomization=r), workdir=tmp_path_factory.mktemp("dr"))
    try:
        seen = []
        for seed in (3, 4, 3):
            _, info = e.reset(seed=seed)
            d = info["dynamics"]
            assert r.servo_strength[0] <= d["servo_strength"] <= r.servo_strength[1]
            assert r.servo_stiffness[0] <= d["servo_stiffness"] <= r.servo_stiffness[1]
            assert 0.0 <= d["latency_ms"] <= r.latency_ms[1] + 1.0
            seen.append(d)
        assert seen[0] == seen[2] and seen[0] != seen[1]    # tohum aynı -> aynı dinamik
        e.reset(seed=5)
        pushes = 0
        for _ in range(int(round(6.0 / e.dt))):             # 6 s: en az bir itme gelmeli
            before = e._next_push
            e.step(np.zeros(18, dtype=np.float32))
            pushes += e._next_push != before
        assert pushes >= 1
    finally:
        e.close()
