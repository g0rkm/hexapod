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


def test_cukur_her_yonde_basamak():
    from hexapod_rl.terrain_probe import pit

    sdf, height = pit(0.045, half=0.35)
    assert height(0.0, 0.0) == 0.0 and height(0.34, -0.34) == 0.0
    for x, y in ((0.36, 0.0), (-0.36, 0.1), (0.0, 0.36), (0.2, -0.4)):
        assert height(x, y) == 0.045
    boxes = [c for c in ET.fromstring(sdf).iter("collision") if c.find("geometry/box") is not None]
    assert len(boxes) == 4
    for box in boxes:                                     # hepsinin üstü 45 mm'de, iç kenarı 0.35'te
        cx, cy, cz = (float(v) for v in box.find("pose").text.split()[:3])
        sx, sy, sz = (float(v) for v in box.find("geometry/box/size").text.split())
        assert cz + sz / 2 == pytest.approx(0.045)
        inner = min(abs(cx) - sx / 2 if abs(cx) > 1e-9 else math.inf,
                    abs(cy) - sy / 2 if abs(cy) > 1e-9 else math.inf)
        assert inner == pytest.approx(0.35)


def test_yayla_ve_egitim_seti():
    from hexapod_rl.terrain_probe import TRAIN_SETS, plateau, training_terrains

    _, height = plateau(0.05, half=0.5)
    assert height(0.0, 0.0) == 0.05 and height(0.49, 0.49) == 0.05 and height(0.51, 0.0) == 0.0
    items = training_terrains("deneme", 20)                   # 16'lık liste başa sarar
    assert len(items) == 20 and items[16][0] == TRAIN_SETS["deneme"][0][0]
    for label, sdf, h in items:
        assert (sdf == "") == (h is None), label              # düz zemin: ikisi de boş
    with pytest.raises(ValueError):
        training_terrains("yok", 4)


def test_ara_kayit_zemin_durumlari():
    from hexapod_rl.terrain_probe import EVAL_CASES, TRAIN_SETS, eval_cases

    assert set(EVAL_CASES) <= set(TRAIN_SETS)       # her ölçüm seti bir eğitim setine ait
    cases = eval_cases("deneme")
    assert len(cases) == len(EVAL_CASES["deneme"])
    for label, sdf, height, cmd in cases:
        assert sdf.startswith("<model") and callable(height) and len(cmd) == 3, label
    with pytest.raises(ValueError):
        eval_cases("yok")


def test_surtunme_ve_engebe():
    from hexapod_rl.terrain_probe import TRAIN_SETS, flat, rough, slope

    sdf, h = slope(15.0, mu=0.3)
    mus = [float(m.text) for m in ET.fromstring(sdf).iter("mu")]
    assert mus == [0.3] and h(0.0, 0.0) == 0.0
    assert "<mu>" not in slope(15.0)[0]                        # varsayılan: Gazebo'nunki
    with pytest.raises(ValueError):
        flat(0.0)
    assert [float(m.text) for m in ET.fromstring(flat(0.2)[0]).iter("mu")] == [0.2]
    # engebe: aynı tohum aynı zemin; yükseklik fonksiyonu bloklarla birebir
    a, ha = rough(0.04, seed=3)
    b, hb = rough(0.04, seed=3)
    assert a == b and ha(0.31, -0.2) == hb(0.31, -0.2)
    assert rough(0.04, seed=4)[0] != a
    boxes = [c for c in ET.fromstring(a).iter("collision") if c.find("geometry/box") is not None]
    assert len(boxes) == 25 * 25
    for box in boxes[::37]:
        cx, cy, cz = (float(v) for v in box.find("pose").text.split()[:3])
        sz = float(box.find("geometry/box/size").text.split()[2])
        assert cz + sz / 2 == pytest.approx(ha(cx, cy), abs=1e-5)
        assert 0.0 <= ha(cx, cy) <= 0.04
    assert ha(2.0, 0.0) == 0.0                                  # engebe alanının dışı düz
    assert len(TRAIN_SETS["deneme2"]) == 16
