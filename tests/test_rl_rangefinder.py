"""hexapod_rl.rangefinder — tek ışınlı mesafe sensörü simülasyonu (gz'siz)."""

from __future__ import annotations

import math

import pytest

from hexapod_rl.math3d import rotate, rotate_inverse
from hexapod_rl.rangefinder import RangeSensor, obstacle_height, ray_distance, read
from hexapod_rl.terrain_probe import step

FLAT = (lambda x, y: 0.0)
LEVEL = (1.0, 0.0, 0.0, 0.0)
FRONT = RangeSensor(x=0.10, y=0.0, z=0.02, yaw_deg=0.0, pitch_deg=30.0, max_m=1.0)


def test_rotate_ve_tersi():
    s = math.sqrt(0.5)
    yaw90 = (s, 0.0, 0.0, s)                        # z etrafında +90°
    assert rotate(yaw90, (1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))
    v = (0.3, -0.2, 0.7)
    q = (0.9, 0.1, -0.3, 0.2)
    n = math.sqrt(sum(c * c for c in q))
    q = tuple(c / n for c in q)
    assert rotate(q, rotate_inverse(q, v)) == pytest.approx(v)


def test_duz_zeminde_mesafe_ve_engel_yok():
    """Gövde 0.1 m'de, sensör 2 cm yukarıda, 30° aşağı: 0.12 / sin 30° = 0.24 m."""
    (d,) = read([FRONT], (0.0, 0.0, 0.1), LEVEL, FLAT)
    assert d == pytest.approx(0.24, abs=1e-4)
    assert obstacle_height(FRONT, d, (0.0, 0.0, -1.0), 0.1) == pytest.approx(0.0, abs=1e-4)


def test_basamak_once_on_yuzde_sonra_ustte():
    """45 mm basamak x=0.3'te. Uzaktayken ışın ön yüzün alt kısmına çarpar
    (engel yüksekliği küçük ama > 0); yaklaşınca üst yüzeye (45 mm)."""
    _, h = step(0.045, at_x=0.3)
    far = read([FRONT], (0.0, 0.0, 0.1), LEVEL, h)[0]       # düzde 0.24 m olurdu
    assert far == pytest.approx(0.2 / math.cos(math.radians(30.0)), abs=1e-4)   # ön yüz
    assert 0.0 < obstacle_height(FRONT, far, (0.0, 0.0, -1.0), 0.1) < 0.01
    near = read([FRONT], (0.15, 0.0, 0.1), LEVEL, h)[0]     # ışın üst yüzeye düşer
    assert obstacle_height(FRONT, near, (0.0, 0.0, -1.0), 0.1) == pytest.approx(0.045, abs=1e-3)


def test_gormeyen_sensor_ve_egik_govde():
    up = RangeSensor(0.1, 0.0, 0.02, 0.0, -10.0, max_m=0.5)   # yukarı bakıyor
    (d,) = read([up], (0.0, 0.0, 0.1), LEVEL, FLAT)
    assert d == 0.5 and obstacle_height(up, d, (0.0, 0.0, -1.0), 0.1) is None
    # gövde 10° öne eğik (burun aşağı): ışın daha dik, daha yakına düşer;
    # IMU'nun yerçekimi yönüyle hesaplanan engel yüksekliği yine ~0
    p = math.radians(10.0)
    pitched = (math.cos(p / 2), 0.0, math.sin(p / 2), 0.0)   # y etrafında +10°: burun aşağı
    (d,) = read([FRONT], (0.0, 0.0, 0.1), pitched, FLAT)
    assert d < 0.24
    g = rotate_inverse(pitched, (0.0, 0.0, -1.0))
    assert obstacle_height(FRONT, d, g, 0.1) == pytest.approx(0.0, abs=0.02)


def test_isin_zeminin_altindan_baslarsa_sifir():
    assert ray_distance((0.0, 0.0, -0.01), (1.0, 0.0, 0.0), FLAT, 1.0) == 0.0
