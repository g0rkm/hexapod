"""hexapod_rl.pretrain — taklit ile başlatmanın saf kısımları."""

from __future__ import annotations

import pytest

pytest.importorskip("numpy")

from hexapod_rl.pretrain import discounted_returns  # noqa: E402


def test_indirimli_getiri():
    assert discounted_returns([1.0, 1.0, 1.0], 0.5) == pytest.approx([1.75, 1.5, 1.0])


def test_bos_bolum():
    assert discounted_returns([], 0.99) == []
