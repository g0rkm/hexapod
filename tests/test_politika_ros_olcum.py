"""tools/politika_ros_olcum.py — gövde çerçevesinde hız hesabı (saf Python)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

from politika_ros_olcum import body_velocity  # noqa: E402


def test_duz_giden_govde_yonunde_hiz():
    """Gövde dünyada +y'ye bakıyor (yaw 90°) ve +y'ye gidiyor: gövde çerçevesinde ileri."""
    samples = [(0.1 * i, 0.0, 0.01 * i, 0.1, math.pi / 2) for i in range(11)]
    assert body_velocity(samples) == pytest.approx((0.1, 0.0, 0.0), abs=1e-9)


def test_donerek_ilerlerken_hiz_dogru():
    """Çember üzerinde, hep gövdenin önüne doğru giden robot: v = w r, gövdede yana hız yok.
    Başlangıç yönüne göre çevirmek (eski hata) burada ileri hızı küçük gösterirdi."""
    w, r, n, dt = 0.25, 0.4, 2000, 0.005
    samples = [(k * dt, r * math.sin(w * k * dt), r - r * math.cos(w * k * dt), 0.1, w * k * dt)
               for k in range(n + 1)]
    vx, vy, wz = body_velocity(samples)
    assert vx == pytest.approx(w * r, rel=1e-3)
    assert vy == pytest.approx(0.0, abs=1e-3)
    assert wz == pytest.approx(w, rel=1e-6)
