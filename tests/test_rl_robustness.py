"""hexapod_rl.robustness — dayanıklılık taramasının bozulma üretimi (gz'siz)."""

from __future__ import annotations

import math

import pytest

from hexapod_rl.robustness import CASES, perturbation, seeds_for
from hexapod_rl.task import Perturbation


def test_bozulmasiz_durum_varsayilan():
    assert perturbation("yok", 0.0, 1) == Perturbation()


def test_ofset_tohumdan_belirlenimci_ve_olcekli():
    a, b = perturbation("ofset", 2.0, 1), perturbation("ofset", 2.0, 1)
    assert a == b and len(a.joint_offset_deg) == 18
    assert a != perturbation("ofset", 2.0, 2)
    big = perturbation("ofset", 4.0, 1)                       # aynı tohum, iki kat σ
    assert big.joint_offset_deg == pytest.approx([2 * v for v in a.joint_offset_deg])


def test_imu_egikligi_buyuklugu_korunur():
    for seed in (1, 2, 3):
        roll, pitch = perturbation("imu", 6.0, seed).imu_tilt_deg
        assert math.hypot(roll, pitch) == pytest.approx(6.0)


def test_gecikme_ve_servo():
    assert perturbation("gecikme", 40.0, 1) == Perturbation(delay_ms=40.0)
    assert perturbation("servo", 0.6, 1) == Perturbation(servo_strength=0.6)
    with pytest.raises(ValueError):
        perturbation("bilinmeyen", 1.0, 1)


def test_rastgele_yonlu_olmayanlar_tek_kosu():
    assert seeds_for("ofset", 3) == (1, 2, 3) and seeds_for("imu", 3) == (1, 2, 3)
    assert seeds_for("gecikme", 3) == (1,) and seeds_for("servo", 3) == (1,)
    assert CASES[0][1] == "yok"                               # tablonun ilk sütunu referans
