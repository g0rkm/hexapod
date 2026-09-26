"""hexapod_policy.mlp — numpy ile MLP çıkarımı ve .npz biçimi. Her yerde koşar."""

from __future__ import annotations

import json

import numpy as np
import pytest

from hexapod_policy.mlp import MlpPolicy, PolicyContract


def contract(obs=4, act=2) -> PolicyContract:
    return PolicyContract(obs_size=obs, action_size=act, action_scale=0.5, gait_hz=1.5,
                          control_hz=50.0, default_rad=tuple([0.1] * act),
                          command_ranges={"vx": (0.05, 0.15), "vy": (0.0, 0.0), "wz": (0.0, 0.0)})


def random_policy(seed=0, obs=4, hidden=3, act=2) -> MlpPolicy:
    rng = np.random.default_rng(seed)
    layers = [(rng.normal(size=(hidden, obs)), rng.normal(size=hidden)),
              (rng.normal(size=(hidden, hidden)), rng.normal(size=hidden)),
              (rng.normal(size=(act, hidden)), rng.normal(size=act))]
    return MlpPolicy(layers, "tanh", contract(obs, act), "deneme")


def test_ileri_gecis_elle_hesapla_ayni():
    p = random_policy()
    x = np.array([0.3, -0.2, 1.0, 0.5])
    (w0, b0), (w1, b1), (w2, b2) = p.layers
    expected = w2 @ np.tanh(w1 @ np.tanh(w0 @ x + b0) + b1) + b2
    assert p(x) == pytest.approx(expected)


def test_kaydet_yukle_ayni_cikti_ve_sozlesme(tmp_path):
    p = random_policy()
    path = p.save(tmp_path / "policy.npz")
    q = MlpPolicy.load(path)
    x = np.linspace(-1, 1, 4)
    assert q(x) == pytest.approx(p(x))
    assert q.contract == p.contract
    assert q.source == "deneme" and q.activation == "tanh"


def test_bicim_surumu_uyusmazsa_reddeder(tmp_path):
    p = random_policy()
    path = p.save(tmp_path / "policy.npz")
    with np.load(path) as data:
        arrays = {k: data[k] for k in data.files}
    meta = json.loads(str(arrays["meta"]))
    meta["format_version"] = 99
    arrays["meta"] = np.array(json.dumps(meta))
    np.savez(tmp_path / "eski.npz", **arrays)
    with pytest.raises(ValueError, match="sürüm"):
        MlpPolicy.load(tmp_path / "eski.npz")


def test_boyut_hatalari_yakalanir():
    good = random_policy()
    with pytest.raises(ValueError):                       # giriş boyutu sözleşmeyle uyuşmuyor
        MlpPolicy(good.layers, "tanh", contract(obs=5))
    with pytest.raises(ValueError):                       # çıkış boyutu
        MlpPolicy(good.layers, "tanh", contract(act=3))
    with pytest.raises(ValueError):
        MlpPolicy(good.layers, "sigmoid", contract())
    with pytest.raises(ValueError):                       # yanlış gözlem boyutu
        good(np.zeros(3))
    with pytest.raises(ValueError):                       # varsayılan duruş boyu
        PolicyContract(obs_size=4, action_size=2, action_scale=0.5, gait_hz=1.5,
                       control_hz=50.0, default_rad=(0.0,),
                       command_ranges={"vx": (0, 1), "vy": (0, 0), "wz": (0, 0)})


# --- artık eylem sözleşmesi ------------------------------------------------------

BASE = {"groups": [[0, 2, 4], [1, 3, 5]], "reach_mm": 130.0, "height_mm": 100.0, "lift_mm": 25.0}


def residual_contract(**kw) -> PolicyContract:
    args = dict(obs_size=4, action_size=2, action_scale=0.5, gait_hz=1.5, control_hz=50.0,
                default_rad=(0.1, 0.1),
                command_ranges={"vx": (0.05, 0.15), "vy": (0.0, 0.0), "wz": (0.0, 0.0)},
                action_mode="residual", residual_scale=0.2, base_gait=BASE)
    args.update(kw)
    return PolicyContract(**args)


def test_artik_eylem_sozlesmesi_dogrulanir():
    residual_contract()
    with pytest.raises(ValueError):
        residual_contract(residual_scale=0.0)
    with pytest.raises(ValueError):
        residual_contract(base_gait={"groups": [[0], [1]]})
    with pytest.raises(ValueError):
        residual_contract(action_mode="hayali")


def test_artik_eylem_kaydet_yukle(tmp_path):
    p = random_policy()
    q = MlpPolicy(p.layers, "tanh", residual_contract())
    r = MlpPolicy.load(q.save(tmp_path / "r.npz"))
    assert r.contract.action_mode == "residual"
    assert r.contract.residual_scale == pytest.approx(0.2)
    assert r.contract.base_gait == BASE


def test_eski_dosyalar_mutlak_modda_yuklenir(tmp_path):
    """Mod alanları eklenmeden önce yazılmış dosyalar (sözleşmede anahtar yok) mutlak modda."""
    p = random_policy()
    path = p.save(tmp_path / "eski.npz")
    with np.load(path) as data:
        arrays = {k: data[k] for k in data.files}
    meta = json.loads(str(arrays["meta"]))
    for k in ("action_mode", "residual_scale", "base_gait"):
        meta["contract"].pop(k)
    arrays["meta"] = np.array(json.dumps(meta))
    np.savez(tmp_path / "eski2.npz", **arrays)
    assert MlpPolicy.load(tmp_path / "eski2.npz").contract.action_mode == "absolute"


def test_olu_bolge_kaydet_yukle_ve_eski_dosyada_sifir(tmp_path):
    from dataclasses import replace

    p = random_policy()
    c = replace(p.contract, command_ranges={"vx": (-0.15, 0.15), "vy": (-0.08, 0.08),
                                            "wz": (-0.5, 0.5)}, command_deadband=1 / 6)
    q = MlpPolicy.load(MlpPolicy(p.layers, "tanh", c).save(tmp_path / "omni.npz"))
    assert q.contract.command_deadband == pytest.approx(1 / 6)
    assert random_policy().contract.command_deadband == 0.0     # eski dosyalar: ölü bölge yok
    with pytest.raises(ValueError):
        replace(c, command_deadband=1.0)
