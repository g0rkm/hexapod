"""URDF üreticisi testleri. ROS gerektirmez.

En önemlisi: URDF'teki eklem zincirinden hesaplanan ayak konumları
hexapod_kinematics.forward ile aynı olmalı. Simülasyon ile IK ayrışırsa
simülasyonda öğrenilen her şey gerçek robotta kayık basar.

URDF burada üreticiden bağımsız, sıfırdan yazılmış küçük bir zincir
hesaplayıcıyla okunur; üreticinin kendi kodunu tekrar kullanmak testi
anlamsız kılardı.
"""

from __future__ import annotations

import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from hexapod_description import RobotModel
from hexapod_description.urdf import MeshSet, build_urdf, joint_name, link_name
from hexapod_driver import RobotConfig
from hexapod_kinematics import HexapodKinematics, JointAngles

REAL_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"


# ---------------------------------------------------------------------------
# Bağımsız URDF zincir hesaplayıcısı (4x4 homojen matrisler, saf Python)
# ---------------------------------------------------------------------------


def _mat(rot, t):
    return [list(rot[0]) + [t[0]], list(rot[1]) + [t[1]], list(rot[2]) + [t[2]], [0, 0, 0, 1]]


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _rpy(r, p, y):
    """URDF: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _axis_angle(axis, q):
    """Rodrigues: birim eksen etrafında q radyan."""
    x, y, z = axis
    c, s, v = math.cos(q), math.sin(q), 1 - math.cos(q)
    return [
        [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
        [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
        [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
    ]


def _floats(text):
    return [float(v) for v in text.split()]


class Chain:
    def __init__(self, urdf: str) -> None:
        self.root = ET.fromstring(urdf)
        self.joints = {j.get("name"): j for j in self.root.findall("joint")}
        self.by_child = {j.find("child").get("link"): j for j in self.joints.values()}

    def pose(self, link: str, q: dict[str, float]):
        """base_link -> link dönüşümü; q eklem adı -> radyan."""
        chain = []
        while link in self.by_child:
            joint = self.by_child[link]
            chain.append(joint)
            link = joint.find("parent").get("link")
        T = _mat([[1, 0, 0], [0, 1, 0], [0, 0, 1]], (0, 0, 0))
        for joint in reversed(chain):
            origin = joint.find("origin")
            T = _mul(T, _mat(_rpy(*_floats(origin.get("rpy"))), _floats(origin.get("xyz"))))
            if joint.get("type") == "revolute":
                axis = _floats(joint.find("axis").get("xyz"))
                T = _mul(T, _mat(_axis_angle(axis, q.get(joint.get("name"), 0.0)), (0, 0, 0)))
        return T

    def point(self, link: str, q: dict[str, float]):
        T = self.pose(link, q)
        return (T[0][3], T[1][3], T[2][3])


# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def config() -> RobotConfig:
    return RobotConfig.load(REAL_CONFIG)


@pytest.fixture(scope="module")
def model(config) -> RobotModel:
    return RobotModel.from_config(config)


@pytest.fixture(scope="module")
def urdf(model) -> str:
    return build_urdf(model)


def test_ayak_konumlari_ik_ile_ayni(config, urdf):
    """URDF zinciri == hexapod_kinematics.forward, rastgele 300 pozda, 1e-6 mm."""
    kin = HexapodKinematics.from_config(config)
    chain = Chain(urdf)
    rng = random.Random(7)
    for _ in range(50):
        angles = {leg: JointAngles(rng.uniform(-80, 80), rng.uniform(-80, 80),
                                   rng.uniform(-80, 80)) for leg in range(6)}
        expected = kin.forward(angles)
        for leg, a in angles.items():
            q = {joint_name(leg, part): math.radians(v) for part, v in a.as_dict().items()}
            got = chain.point(link_name(leg, "foot"), q)
            want = tuple(v / 1000 for v in expected[leg])
            assert got == pytest.approx(want, abs=1e-9), f"bacak {leg}, açılar {a}"


def test_sifir_durusunda_ayak_dogru_yerde(config, urdf):
    kin = HexapodKinematics.from_config(config)
    chain = Chain(urdf)
    for leg, p in kin.neutral_stance().items():
        assert chain.point(link_name(leg, "foot"), {}) == pytest.approx(
            tuple(v / 1000 for v in p), abs=1e-9)


def test_pozitif_yonler_sozlesmeyle_uyumlu(urdf):
    """Kalibrasyon sözleşmesi: femur + ayağı kaldırır, tibia + ayağı dışarı götürür,
    coxa + yukarıdan bakınca saat yönünün tersine."""
    chain = Chain(urdf)
    for leg in range(6):
        foot = link_name(leg, "foot")
        base = chain.point(foot, {})
        radial = math.hypot(base[0], base[1])
        up = chain.point(foot, {joint_name(leg, "femur"): 0.1})
        assert up[2] > base[2]
        out = chain.point(foot, {joint_name(leg, "tibia"): 0.1})
        assert math.hypot(out[0], out[1]) > radial
        turned = chain.point(foot, {joint_name(leg, "coxa"): 0.1})
        cross_z = base[0] * turned[1] - base[1] * turned[0]
        assert cross_z > 0


def test_agac_gecerli(urdf):
    root = ET.fromstring(urdf)
    links = [el.get("name") for el in root.findall("link")]
    joints = root.findall("joint")
    assert len(links) == len(set(links)), "tekrarlanan link adı"
    children = [j.find("child").get("link") for j in joints]
    assert len(children) == len(set(children)), "bir link iki kez çocuk"
    assert set(links) - set(children) == {"base_link"}, "tek kök base_link olmalı"
    for j in joints:
        assert j.find("parent").get("link") in links
        assert j.find("child").get("link") in links
    assert sum(j.get("type") == "revolute" for j in joints) == 18
    assert sum(j.get("type") == "fixed" for j in joints) == 8  # body + imu + 6 ayak


def test_kok_link_ataletsiz_diger_fiziksel_linkler_ataletli(urdf):
    root = ET.fromstring(urdf)
    for link in root.findall("link"):
        name = link.get("name")
        frame_only = name in ("base_link", "imu_link") or name.endswith("_foot")
        assert (link.find("inertial") is None) == frame_only, name


def test_kutle_toplami_modelle_ayni(model, urdf):
    root = ET.fromstring(urdf)
    total = sum(float(m.get("value")) for m in root.iter("mass"))
    assert total == pytest.approx(model.total_mass())


def test_limitler_ve_servo_degerleri(model, urdf):
    root = ET.fromstring(urdf)
    for (leg, part), lim in model.limits.items():
        el = root.find(f"joint[@name='{joint_name(leg, part)}']/limit")
        assert float(el.get("lower")) == pytest.approx(lim.lower)
        assert float(el.get("upper")) == pytest.approx(lim.upper)
        assert float(el.get("effort")) == pytest.approx(model.effort)
        assert float(el.get("velocity")) == pytest.approx(model.velocity)


def test_ayak_kuresinin_alt_ucu_ayak_noktasinda(model, urdf):
    root = ET.fromstring(urdf)
    for leg in range(6):
        tibia = root.find(f"link[@name='{link_name(leg, 'tibia')}']")
        sphere = [c for c in tibia.findall("collision") if c.find("geometry/sphere") is not None]
        assert len(sphere) == 1
        r = float(sphere[0].find("geometry/sphere").get("radius"))
        z = _floats(sphere[0].find("origin").get("xyz"))[2]
        assert z - r == pytest.approx(-model.tibia)


def test_mesh_modu_meshes_yaml_dosyalarini_kullanir(model):
    meshes = MeshSet.load("package://hexapod_description/meshes/")
    root = ET.fromstring(build_urdf(model, meshes))
    used = [m.get("filename") for m in root.iter("mesh")]
    assert used, "mesh modunda hiç mesh yok"
    assert all(u.startswith("package://hexapod_description/meshes/") for u in used)
    assert {u.rsplit("/", 1)[1] for u in used} == meshes.files()
    # görseller mesh, çarpışmalar hâlâ kutu/küre
    assert all(c.find("geometry/mesh") is None for c in root.iter("collision"))


def test_mesh_yoksa_gorseller_kutu(urdf):
    root = ET.fromstring(urdf)
    assert next(root.iter("mesh"), None) is None
    assert all(v.find("geometry/box") is not None for v in root.iter("visual"))
