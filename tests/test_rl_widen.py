"""hexapod_rl.widen — modele öğrenilmiş ayak kaldırma çıkışı ekleme (sıcak başlangıç)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("stable_baselines3", reason="SB3 yok (tools/wsl/rl_kurulum.sh)")

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "models" / "ppo_lift50_3750k" / "model.zip"


def test_genisletilen_model_eskisiyle_ayni_davranir(tmp_path):
    from stable_baselines3 import PPO

    from hexapod_rl.task import action_for_lift, find_task
    from hexapod_rl.widen import widen

    out = widen(SRC, tmp_path / "w" / "model.zip", (20.0, 60.0), lift_std=0.3)
    old, new = PPO.load(SRC, device="cpu"), PPO.load(out, device="cpu")
    task = find_task(out)
    assert task.lift_action == (20.0, 60.0) and task.lift_mm == 50.0
    obs = np.random.default_rng(0).normal(0, 0.5, (64, 29)).astype(np.float32)
    a_old, _ = old.predict(obs, deterministic=True)
    a_new, _ = new.predict(obs, deterministic=True)
    assert a_new.shape == (64, 19)
    np.testing.assert_allclose(a_new[:, :18], a_old, atol=1e-6)       # eklemler aynı
    np.testing.assert_allclose(a_new[:, 18], action_for_lift(50.0, (20.0, 60.0)), atol=1e-6)
    std = new.policy.log_std.exp().detach().numpy()
    assert std[18] == pytest.approx(0.3, rel=1e-5)
    np.testing.assert_allclose(std[:18], old.policy.log_std.exp().detach().numpy(), rtol=1e-6)
    with pytest.raises(ValueError):                                       # aralık dışı taban
        widen(SRC, tmp_path / "x" / "model.zip", (20.0, 40.0))
    out = widen(SRC, tmp_path / "s" / "model.zip", (20.0, 60.0), lift_start=35.0)
    a_start, _ = PPO.load(out, device="cpu").predict(obs, deterministic=True)
    np.testing.assert_allclose(a_start[:, :18], a_old, atol=1e-6)     # eklemler yine aynı
    np.testing.assert_allclose(a_start[:, 18], action_for_lift(35.0, (20.0, 60.0)), atol=1e-6)
    with pytest.raises(ValueError):                                       # aralık dışı başlangıç
        widen(SRC, tmp_path / "y" / "model.zip", (20.0, 60.0), lift_start=65.0)
