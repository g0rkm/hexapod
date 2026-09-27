"""hexapod_policy.lift_reflex — mesafe sensörlü ayak kaldırma refleksi (saf Python)."""

from __future__ import annotations

import pytest

from hexapod_policy.lift_reflex import LiftReflex, RangeSensor, obstacle_height


def test_engel_yokken_dusuk_kaldirma():
    r = LiftReflex()
    for k in range(100):
        assert r.update(0.02 * k, [0.0, None, 0.005]) == r.low_mm


def test_engel_art_arda_gorulunce_yukselir_ve_suresi_dolunca_iner():
    r = LiftReflex(persist=2, hold_s=3.0, margin_m=0.015)
    assert r.update(0.00, [0.045]) == r.low_mm            # tek okuma yetmez
    assert r.update(0.02, [0.045]) == pytest.approx(60.0)  # 45 + 15 mm
    assert r.update(1.00, [0.0]) == pytest.approx(60.0)   # tutuluyor
    assert r.update(3.03, [0.0]) == r.low_mm               # 3 s geçti


def test_tek_okumalik_gurultu_sayilmaz_ve_kaldirma_sinirlanir():
    r = LiftReflex(persist=2)
    for k in range(50):                                     # arada bir yanlış okuma
        assert r.update(0.02 * k, [0.03 if k % 2 else 0.0]) == r.low_mm
    r = LiftReflex(persist=1, margin_m=0.015)
    assert r.update(0.0, [0.030]) == pytest.approx(45.0)
    assert r.update(0.1, [0.200]) == pytest.approx(r.high_mm)  # 200 mm engel: en çok high
    r.reset()
    assert r.update(0.2, [0.0]) == r.low_mm


def test_engel_yuksekligi_sensorun_yerine_ve_imu_ya_gore():
    s = RangeSensor(0.10, 0.0, 0.02, 0.0, 30.0, 1.0)
    assert obstacle_height(s, 0.24, (0.0, 0.0, -1.0), 0.1) == pytest.approx(0.0, abs=1e-9)
    assert obstacle_height(s, 0.15, (0.0, 0.0, -1.0), 0.1) == pytest.approx(0.045)
    assert obstacle_height(s, 1.0, (0.0, 0.0, -1.0), 0.1) is None


def test_sensorun_gordugu_yurume_yonleri():
    from hexapod_policy.lift_reflex import covers

    front = [RangeSensor(0.1, 0.0, 0.02, y, 20.0, 1.0) for y in (0.0, 25.0, -25.0)]
    assert covers(front, 0.1, 0.0) and covers(front, 0.07, 0.04)      # ileri, çapraz ~30°
    assert not covers(front, 0.0, 0.06) and not covers(front, -0.1, 0.0)   # sola, geri
    assert covers(front, 0.0, 0.0)                                     # yerinde dönüş
    back = [RangeSensor(-0.1, 0.0, 0.02, 180.0, 20.0, 1.0)]
    assert covers(back, -0.1, 0.0) and covers(back, -0.1, -0.05)       # -180/+180 sarması
