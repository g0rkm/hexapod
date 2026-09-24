"""RobotModel -> URDF.

URDF elle yazılmaz, her seferinde robot.yaml'dan üretilir: geometri tek
kaynaktan gelsin, IK ile simülasyon ayrışmasın.

Ağaç
----
    base_link                  gövde çerçevesi (REP-103), ataletsiz
    └─ body (sabit)            gövdenin kütlesi, çarpışması, görseli
    └─ leg{i}_coxa_joint       revolute, eksen +z, montaj (x, y, z, yaw)
       └─ leg{i}_coxa
          └─ leg{i}_femur_joint   revolute, eksen -y, +x yönünde coxa ötede
             └─ leg{i}_femur
                └─ leg{i}_tibia_joint   revolute, eksen -y, +x yönünde femur ötede
                   └─ leg{i}_tibia       ayak küresi burada
                      └─ leg{i}_foot (sabit)   ayak ucu, (0, 0, -tibia); yalnız çerçeve

Eksenler IK ↔ kalibrasyon sözleşmesinden (CLAUDE.md): femur + bacağı
kaldırır, tibia + ayağı dışarı götürür; ikisi de -y etrafında dönme.
Sıfır açıda femur yatay, tibia dümdüz aşağı.

Kök link neden ataletsiz: KDL (robot_state_publisher) ataletli kök linki
desteklemiyor ve uyarı veriyor. Gövde kütlesi sabit eklemle bağlı "body"
linkinde; Gazebo sabit eklemleri zaten birleştirir.

Ayak küresi neden tibia'da: ataletsiz bir link sabit eklemle
birleştirilirken bazı dönüştürücüler çarpışmasını düşürebiliyor. leg{i}_foot
yalnızca çerçeve (IK hedefi, TF).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import yaml

from .model import CollisionBox, Inertial, LinkModel, RobotModel

JOINT_NAMES = ("coxa", "femur", "tibia")

MESHES_YAML = Path(__file__).resolve().parent / "meshes.yaml"

# Görünüş için; fiziksel bir anlamı yok. Parçalar siyah ve beyaz PETG basıldı.
_MATERIALS = {
    "govde": "0.15 0.15 0.15 1",
    "bacak": "0.92 0.92 0.92 1",
}


def link_name(leg: int, part: str) -> str:
    """ör. leg0_coxa — calibration.yaml anahtarıyla aynı biçim."""
    return f"leg{leg}_{part}"


def joint_name(leg: int, part: str) -> str:
    return f"leg{leg}_{part}_joint"


@dataclass(frozen=True)
class MeshPlacement:
    file: str
    xyz: tuple[float, float, float]  # m, link çerçevesinde
    rpy: tuple[float, float, float]  # rad


class MeshSet:
    """Görsel mesh'ler: meshes.yaml'daki yerleşim + dosyaların URI öneki.

    uri_prefix örnekleri:
      "package://hexapod_description/meshes/"   ROS kurulumunda
      "file:///home/.../meshes/"                   ROS'suz, yerel dosyadan
    """

    def __init__(self, placements: dict[str, list[MeshPlacement]], uri_prefix: str) -> None:
        self.placements = placements
        self.uri_prefix = uri_prefix if uri_prefix.endswith("/") else uri_prefix + "/"

    @classmethod
    def load(cls, uri_prefix: str, path: Path = MESHES_YAML) -> "MeshSet":
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        placements = {
            link: [MeshPlacement(str(m["file"]), tuple(m["xyz"]), tuple(m["rpy"])) for m in items]
            for link, items in raw.items()
        }
        return cls(placements, uri_prefix)

    def files(self) -> set[str]:
        return {m.file for items in self.placements.values() for m in items}


def build_urdf(model: RobotModel, meshes: MeshSet | None = None) -> str:
    """URDF metni. meshes verilmezse görseller çarpışma kutularından çizilir."""
    robot = ET.Element("robot", name=model.name)
    robot.append(ET.Comment(
        " ÜRETİLDİ: hexapod_description, config/robot.yaml'dan. Elle düzenlemeyin. "
        "Kütle/atalet TAHMİN (robot.yaml -> simulation). "
    ))
    for name, rgba in _MATERIALS.items():
        ET.SubElement(ET.SubElement(robot, "material", name=name), "color", rgba=rgba)

    ET.SubElement(robot, "link", name="base_link")
    body = _link(robot, "body", model.links["body"], "govde", meshes, "body")
    _fixed(robot, "body_joint", "base_link", body, (0.0, 0.0, 0.0))

    for leg_id in sorted(model.mounts):
        _leg(robot, model, leg_id, meshes)

    ET.indent(robot, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding="unicode") + "\n"


# ---------------------------------------------------------------------------
# Parçalar
# ---------------------------------------------------------------------------


def _leg(robot: ET.Element, model: RobotModel, leg_id: int, meshes: MeshSet | None) -> None:
    mount = model.mounts[leg_id]
    parent = "base_link"
    origins = {
        "coxa": ((mount.x, mount.y, mount.z), (0.0, 0.0, mount.yaw)),
        "femur": ((model.coxa, 0.0, 0.0), (0.0, 0.0, 0.0)),
        "tibia": ((model.femur, 0.0, 0.0), (0.0, 0.0, 0.0)),
    }
    axes = {"coxa": (0, 0, 1), "femur": (0, -1, 0), "tibia": (0, -1, 0)}

    for part in JOINT_NAMES:
        child = _link(robot, link_name(leg_id, part), model.leg_link(leg_id, part), "bacak",
                      meshes, part)
        joint = ET.SubElement(robot, "joint", name=joint_name(leg_id, part), type="revolute")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        xyz, rpy = origins[part]
        _origin(joint, xyz, rpy)
        ET.SubElement(joint, "axis", xyz=_vec(axes[part]))
        limit = model.limits[(leg_id, part)]
        ET.SubElement(joint, "limit", lower=_num(limit.lower), upper=_num(limit.upper),
                      effort=_num(model.effort), velocity=_num(model.velocity))
        parent = child

    # Ayak küresi tibia'da; kürenin alt ucu tibia uzunluğunda.
    tibia = robot.find(f"link[@name='{parent}']")
    r = model.foot_radius
    collision = ET.SubElement(tibia, "collision", name=f"{link_name(leg_id, 'foot')}_collision")
    _origin(collision, (0.0, 0.0, -(model.tibia - r)))
    ET.SubElement(ET.SubElement(collision, "geometry"), "sphere", radius=_num(r))

    foot = link_name(leg_id, "foot")
    ET.SubElement(robot, "link", name=foot)
    _fixed(robot, joint_name(leg_id, "foot"), parent, foot, (0.0, 0.0, -model.tibia))


def _link(robot: ET.Element, name: str, link: LinkModel, material: str,
          meshes: MeshSet | None, mesh_key: str) -> str:
    el = ET.SubElement(robot, "link", name=name)
    _inertial(el, link.inertial)

    if meshes is not None:
        for m in meshes.placements.get(mesh_key, []):
            visual = ET.SubElement(el, "visual")
            _origin(visual, m.xyz, m.rpy)
            ET.SubElement(ET.SubElement(visual, "geometry"), "mesh",
                          filename=meshes.uri_prefix + m.file, scale="0.001 0.001 0.001")
            ET.SubElement(visual, "material", name=material)
    else:
        for box in link.boxes:
            visual = ET.SubElement(el, "visual")
            _box(visual, box)
            ET.SubElement(visual, "material", name=material)

    for i, box in enumerate(link.boxes):
        _box(ET.SubElement(el, "collision", name=f"{name}_collision_{i}"), box)
    return name


def _inertial(el: ET.Element, inertial: Inertial) -> None:
    node = ET.SubElement(el, "inertial")
    _origin(node, inertial.com)
    ET.SubElement(node, "mass", value=_num(inertial.mass))
    ixx, iyy, izz, ixy, ixz, iyz = inertial.inertia
    ET.SubElement(node, "inertia", ixx=_num(ixx), ixy=_num(ixy), ixz=_num(ixz),
                  iyy=_num(iyy), iyz=_num(iyz), izz=_num(izz))


def _box(parent: ET.Element, box: CollisionBox) -> None:
    _origin(parent, box.center)
    ET.SubElement(ET.SubElement(parent, "geometry"), "box", size=_vec(box.size))


def _fixed(robot: ET.Element, name: str, parent: str, child: str, xyz) -> None:
    joint = ET.SubElement(robot, "joint", name=name, type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    _origin(joint, xyz)


def _origin(parent: ET.Element, xyz, rpy=(0.0, 0.0, 0.0)) -> None:
    ET.SubElement(parent, "origin", xyz=_vec(xyz), rpy=_vec(rpy))


def _vec(values) -> str:
    return " ".join(_num(v) for v in values)


def _num(v: float) -> str:
    """Kısa ama kayıpsıza yakın; -0 yazma."""
    v = float(v)
    if abs(v) < 1e-12:
        return "0"
    if not math.isfinite(v):
        raise ValueError(f"URDF'e sonlu olmayan sayı yazılamaz: {v}")
    return format(v, ".10g")
