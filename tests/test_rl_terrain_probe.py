"""hexapod_rl.terrain_probe — deneme zeminlerinin SDF'i ile yükseklik fonksiyonu tutarlı mı."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import pytest

from hexapod_rl.terrain_probe import TERRAINS, slope, step


def test_egim_yonu_ve_yuksekligi():
    _, up = slope(-10.0)                       # +x yönünde yükselir: yokuş yukarı
    assert up(1.0, 0.3) == pytest.approx(math.tan(math.radians(10.0)))
    assert up(0.0, 5.0) == 0.0                 # üst yüz orijinden geçer
    _, side = slope(10.0, "y")                 # +y yönünde alçalır
    assert side(0.7, 1.0) == pytest.approx(-math.tan(math.radians(10.0)))
    with pytest.raises(ValueError):
        slope(5.0, "z")


def test_egim_kutusunun_ust_yuzu_yukseklikle_ayni():
    """SDF'teki kutunun üst yüzünün merkezi (0, 0, 0) ve normali eğimin normali."""
    deg = 15.0
    sdf, height = slope(deg)
    pose = [float(v) for v in ET.fromstring(sdf).find(".//collision/pose").text.split()]
    thick = float(ET.fromstring(sdf).find(".//collision/geometry/box/size").text.split()[2])
    t = pose[4]                                               # pitch
    top = [pose[0] + math.sin(t) * thick / 2, pose[2] + math.cos(t) * thick / 2]
    assert top == pytest.approx([0.0, 0.0], abs=1e-12)
    x = 0.4                                                   # yüzeyde bir nokta
    assert height(x, 0.0) == pytest.approx(-math.tan(math.radians(deg)) * x)


def test_basamak():
    sdf, height = step(0.03, at_x=0.3)
    assert height(0.29, 0.0) == 0.0 and height(0.31, -1.0) == 0.03
    box = ET.fromstring(sdf).find(".//collision[@name='step']")
    x, _, z = (float(v) for v in box.find("pose").text.split()[:3])
    sx, _, sz = (float(v) for v in box.find("geometry/box/size").text.split())
    assert x - sx / 2 == pytest.approx(0.3) and z + sz / 2 == pytest.approx(0.03)


def test_zemin_listesi_duz_zeminle_baslar():
    assert TERRAINS[0] == ("düz", None)
    for name, make in TERRAINS[1:]:
        sdf, height = make()
        assert sdf.startswith("<model") and callable(height), name
