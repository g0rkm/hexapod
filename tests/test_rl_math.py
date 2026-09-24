"""hexapod_rl.math3d — saf Python, her yerde koşar."""

from __future__ import annotations

import math

import pytest

from hexapod_rl.math3d import rotate_inverse



def test_birim_dondurme_degistirmez():
    assert rotate_inverse((1.0, 0.0, 0.0, 0.0), (1.0, 2.0, 3.0)) == pytest.approx((1.0, 2.0, 3.0))


def test_yaw_90_dunyadaki_x_govdede_eksi_y():
    # gövde +z etrafında +90° dönmüş: dünya +x, gövdeden bakınca -y'de
    h = math.sqrt(0.5)
    assert rotate_inverse((h, 0.0, 0.0, h), (1.0, 0.0, 0.0)) == pytest.approx((0.0, -1.0, 0.0))


def test_roll_yercekimi_govdede_yana_kayar():
    # +x etrafında +30° yatık gövde: yerçekimi gövdede -y ve -z bileşenli
    a = math.radians(30)
    q = (math.cos(a / 2), math.sin(a / 2), 0.0, 0.0)
    g = rotate_inverse(q, (0.0, 0.0, -1.0))
    assert g == pytest.approx((0.0, -math.sin(a), -math.cos(a)))
