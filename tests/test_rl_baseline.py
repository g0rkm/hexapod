"""hexapod_rl.baseline — Samet'in tripod'unu RL ortamında politika gibi koşturan adaptör.
Saf Python, her yerde koşar."""

from __future__ import annotations

from hexapod_rl.baseline import _COMMAND, TripodPolicy
from hexapod_rl.state import SimState
from hexapod_rl.task import TaskConfig, observation


def test_gozlemden_hiz_komutunu_okur():
    """Adaptör komutu gözlemdeki yerinden okuyor; task.observation düzeni değişirse yakalanır."""
    s = SimState(time=0.0, joint_pos=(0.0,) * 18, joint_vel=(0.0,) * 18,
                 joint_target=(0.0,) * 18, joint_effort=(0.0,) * 18,
                 base_pos=(0.0, 0.0, 0.1), base_quat=(1.0, 0.0, 0.0, 0.0),
                 base_lin_vel=(0.0, 0.0, 0.0), base_ang_vel=(0.0, 0.0, 0.0),
                 foot_pos=((0.0, 0.0, 0.0),) * 6, foot_contact=(True,) * 6)
    cmd = (0.1, 0.02, 0.3)
    obs = observation(s, cmd, 0.25, [0.0] * 18, 0.5)
    assert tuple(obs[_COMMAND]) == cmd


def test_tripod_eylemleri_kirpilmadan_sigar():
    """Eğitimdeki hız aralığında tripod eylem sınırlarını ([-1, 1]) aşmamalı,
    yoksa ortam kırpar ve ölçülen tripod Samet'inkinden farklı olur."""
    policy = TripodPolicy(TaskConfig())
    obs = [0.0] * 29
    for vx in (0.05, 0.15):
        policy.reset()
        obs[_COMMAND] = [vx, 0.0, 0.0]
        for _ in range(100):                 # 2 s, 3 adım döngüsü
            action, _ = policy.predict(obs)
            assert len(action) == 18
            assert max(abs(a) for a in action) <= 1.0


def test_adim_yuksekligi_verilebilir():
    """Zeminde RL ile adil karşılaştırma: tripod da politikanın tabanı kadar
    yüksek adım atabilmeli; verilmezse Samet'in varsayılanı."""
    from hexapod_gait import GaitParams

    assert TripodPolicy().gait.params.step_height_mm == GaitParams().step_height_mm
    assert TripodPolicy(step_height_mm=50.0).gait.params.step_height_mm == 50.0
