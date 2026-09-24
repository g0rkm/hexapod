"""hexapod_rl.task — gözlem, eylem, ödül, devrilme. Saf Python, her yerde koşar."""

from __future__ import annotations

import math

import pytest

from hexapod_rl.state import SimState
from hexapod_rl.task import (
    ACTION_SIZE,
    OBS_SIZE,
    TaskConfig,
    action_to_targets,
    fallen,
    observation,
    reward,
)

LEVEL = (1.0, 0.0, 0.0, 0.0)


def state(z=0.1, quat=LEVEL, lin=(0.0, 0.0, 0.0), ang=(0.0, 0.0, 0.0), targets=None,
          vel=None) -> SimState:
    return SimState(
        time=0.0, joint_pos=tuple([0.0] * 18), joint_vel=tuple(vel or [0.0] * 18),
        joint_target=tuple(targets or [0.0] * 18), base_pos=(0.0, 0.0, z), base_quat=quat,
        base_lin_vel=lin, base_ang_vel=ang, foot_pos=tuple([(0.0, 0.0, 0.0)] * 6),
        foot_contact=tuple([True] * 6),
    )


def test_eylem_hedefe_olcekli_ve_kirpik():
    default = [0.1] * 18
    limits = [(-0.5, 0.5)] * 18
    t = action_to_targets([1.0] * 18, default, 0.3, limits)
    assert t[0] == pytest.approx(0.4)
    t = action_to_targets([5.0] * 18, default, 0.3, limits)   # eylem [-1,1]'e kırpılır
    assert t[0] == pytest.approx(0.4)
    t = action_to_targets([1.0] * 18, default, 1.0, limits)   # hedef limite kırpılır
    assert t[0] == pytest.approx(0.5)
    with pytest.raises(ValueError):
        action_to_targets([0.0] * 17, default, 0.3, limits)


def test_gozlem_boyutu_ve_duz_govde():
    default = [0.2] * 18
    obs = observation(state(targets=[0.2] * 18), (0.1, 0.0, 0.0), 0.0, default, 0.5)
    assert len(obs) == OBS_SIZE == 29
    assert obs[0:3] == pytest.approx([0.0, 0.0, -1.0])     # yerçekimi aşağı
    assert obs[6:24] == pytest.approx([0.0] * 18)          # hedef = varsayılan
    assert obs[24:27] == pytest.approx([0.1, 0.0, 0.0])    # komut
    assert obs[27:29] == pytest.approx([0.0, 1.0])         # saat sin, cos


def test_gozlemde_olculen_aci_ve_temas_yok():
    """Gerçek robotta ölçülen eklem açısı ve ayak teması yok (docs/ARAYUZ.md)."""
    a = state()
    b = SimState(**{**a.__dict__, "joint_pos": tuple([1.0] * 18),
                    "foot_contact": tuple([False] * 6)})
    args = ((0.1, 0.0, 0.0), 0.3, [0.0] * 18, 0.5)
    assert observation(a, *args) == observation(b, *args)


def test_tam_izleme_en_yuksek_hiz_odulu():
    cfg = TaskConfig()
    _, perfect = reward(state(lin=(0.1, 0.0, 0.0)), [0.0] * 18, [0.0] * 18,
                        (0.1, 0.0, 0.0), cfg, False)
    _, still = reward(state(), [0.0] * 18, [0.0] * 18, (0.1, 0.0, 0.0), cfg, False)
    assert perfect["lin_vel"] == pytest.approx(cfg.w["lin_vel"])
    assert still["lin_vel"] < perfect["lin_vel"]


def test_cezalar_isaretli():
    cfg = TaskConfig()
    _, terms = reward(state(z=0.08, vel=[1.0] * 18), [1.0] * 18, [0.0] * 18,
                      (0.0, 0.0, 0.0), cfg, True)
    for k in ("height", "joint_vel", "action_rate", "fall"):
        assert terms[k] < 0, k


def test_devrilme():
    cfg = TaskConfig()
    assert not fallen(state(z=0.1), cfg)
    assert fallen(state(z=0.03), cfg)                      # gövde yerde
    a = math.radians(50)                                   # 50° yatık
    assert fallen(state(quat=(math.cos(a / 2), math.sin(a / 2), 0.0, 0.0)), cfg)
    a = math.radians(30)
    assert not fallen(state(quat=(math.cos(a / 2), math.sin(a / 2), 0.0, 0.0)), cfg)


def test_eylem_boyutu():
    assert ACTION_SIZE == 18
