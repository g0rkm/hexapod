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


SENSOR = RangeSensor(0.10, 0.0, 0.02, 0.0, 20.0, 1.0)
LEVEL_G = (0.0, 0.0, -1.0)


def _nose_up(deg):
    """Burnu deg kadar kalkmış gövdede yerçekimi yönü (gövde çerçevesi)."""
    import math
    t = math.radians(deg)
    return (-math.sin(t), 0.0, -math.cos(t))


def test_duzgun_yokus_yercekimine_gore_engel_govdeye_gore_degil():
    """Gövde 10° yokuşta zemine paralel (burnu yukarı): ışın gövde çerçevesinde
    düz zemindeki kadar ileride zemine çarpar. Yerçekimine göre bu nokta ~76 mm
    "yüksek" görünüyor ve refleks ayağı boşuna kaldırıyordu; gövdeye göre 0."""
    import math

    d = 0.12 / math.sin(math.radians(20.0))            # gövde düzleminde zemine çarpma
    g = _nose_up(10.0)
    assert LiftReflex(reference="govde").heights([SENSOR], [d], g, 0.1)[0] == \
        pytest.approx(0.0, abs=1e-9)
    assert LiftReflex(reference="yercekimi").heights([SENSOR], [d], g, 0.1)[0] > 0.07
    step = 0.6 * d                                     # 45 mm'lik basamağa daha yakında çarpar
    assert LiftReflex().reference == "egim"            # varsayılan (ders 50)
    for ref in ("yercekimi", "govde", "egim"):
        r = LiftReflex(reference=ref)
        assert r.heights([SENSOR], [step], LEVEL_G, 0.1)[0] > r.threshold_m, ref
        assert r.heights([SENSOR], [1.0], LEVEL_G, 0.1) == [None], ref   # görmüyor
    with pytest.raises(ValueError):
        LiftReflex(reference="imu")


def test_egim_kalici_yokusu_duz_kisa_egilmeyi_yercekimi_gibi_gorur():
    """"egim": yerçekimi yönünün yavaş ortalaması zeminin eğimi. Uzun süre 10°
    yokuşta: gövde kipi gibi (engel yok). Düz zeminde yürürken burun birden 10°
    kalkarsa (basamağa çıkarken): yerçekimi kipi gibi (gövde kipi burada basamağın
    üstünü alçak görüp kaldırmayı erken indiriyordu)."""
    import math

    d = 0.12 / math.sin(math.radians(20.0))
    g = _nose_up(10.0)
    grav = LiftReflex(reference="yercekimi").heights([SENSOR], [d], g, 0.1)[0]

    r = LiftReflex(reference="egim", slope_tau_s=5.0)
    for k in range(1500):                               # 30 s yokuşta
        h = r.heights([SENSOR], [d], g, 0.1, 0.02 * k)[0]
    assert h == pytest.approx(0.0, abs=1e-3)

    r = LiftReflex(reference="egim", slope_tau_s=5.0)
    for k in range(500):                                # 10 s düzde
        r.heights([SENSOR], [0.3], LEVEL_G, 0.1, 0.02 * k)
    h = r.heights([SENSOR], [d], g, 0.1, 10.02)[0]      # burun birden kalktı
    assert h == pytest.approx(grav, abs=2e-3)
    r.reset()                                           # sıfırlanınca durduğu zemin eğim sayılır
    assert r.heights([SENSOR], [d], g, 0.1, 20.0)[0] == pytest.approx(0.0, abs=1e-9)


def test_donus_a_yi_b_ye_cevirir():
    from hexapod_policy.lift_reflex import rotate_between

    a, b = LEVEL_G, _nose_up(25.0)
    assert rotate_between(a, b, a) == pytest.approx(b)
    assert rotate_between(a, a, (0.3, -0.2, 0.9)) == pytest.approx((0.3, -0.2, 0.9))
    v = rotate_between(a, b, (0.0, 1.0, 0.0))           # dönüş ekseni üzerinde: değişmez
    assert v == pytest.approx((0.0, 1.0, 0.0))


def test_sensorun_gordugu_yurume_yonleri():
    from hexapod_policy.lift_reflex import covers

    front = [RangeSensor(0.1, 0.0, 0.02, y, 20.0, 1.0) for y in (0.0, 25.0, -25.0)]
    assert covers(front, 0.1, 0.0) and covers(front, 0.07, 0.04)      # ileri, çapraz ~30°
    assert not covers(front, 0.0, 0.06) and not covers(front, -0.1, 0.0)   # sola, geri
    assert covers(front, 0.0, 0.0)                                     # yerinde dönüş
    back = [RangeSensor(-0.1, 0.0, 0.02, 180.0, 20.0, 1.0)]
    assert covers(back, -0.1, 0.0) and covers(back, -0.1, -0.05)       # -180/+180 sarması
