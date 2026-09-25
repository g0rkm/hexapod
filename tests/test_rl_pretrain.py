"""hexapod_rl.pretrain — taklit ile başlatmanın saf kısımları."""

from __future__ import annotations

import pytest

pytest.importorskip("numpy")

from hexapod_rl.pretrain import discounted_returns  # noqa: E402


def test_indirimli_getiri():
    assert discounted_returns([1.0, 1.0, 1.0], 0.5) == pytest.approx([1.75, 1.5, 1.0])


def test_bos_bolum():
    assert discounted_returns([], 0.99) == []


@pytest.mark.parametrize("sde", [False, True])
def test_kesif_std_istenen_duzeyde(sde):
    """Normal ve gSDE politikada keşif gürültüsü istenen std'ye kurulabilmeli
    (gSDE'de gürültü gizli katmana bağlı olduğu için ölçekleme gerekiyor)."""
    pytest.importorskip("stable_baselines3")
    pytest.importorskip("gymnasium")
    import numpy as np
    from stable_baselines3 import PPO

    from hexapod_rl.pretrain import SDE_SAMPLE_FREQ, _spaces_only_env, set_exploration_std
    from hexapod_rl.train import PPO_KWARGS

    kwargs = dict(PPO_KWARGS)
    if sde:
        kwargs.update(use_sde=True, sde_sample_freq=SDE_SAMPLE_FREQ,
                      policy_kwargs={**PPO_KWARGS["policy_kwargs"], "full_std": True})
    model = PPO("MlpPolicy", _spaces_only_env(), device="cpu", seed=0, verbose=0, **kwargs)
    obs = np.random.default_rng(0).normal(0, 0.5, (256, 29)).astype(np.float32)
    assert set_exploration_std(model, obs, 0.1) == pytest.approx(0.1, rel=0.3)
