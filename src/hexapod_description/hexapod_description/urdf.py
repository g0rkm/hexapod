"""RobotModel -> URDF.

URDF elle yazılmaz, her seferinde robot.yaml'dan üretilir: geometri tek
kaynaktan gelsin, IK ile simülasyon ayrışmasın.

Ağaç
----
    base_link                  gövde çerçevesi (REP-103), ataletsiz
    ├─ body (sabit)            gövdenin kütlesi, çarpışması, görseli
    ├─ imu_link (sabit)        IMU çerçevesi; base_link ile aynı (bkz. interface.py)
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

Çarpışma adları: sdformat URDF'i SDF'e çevirirken çarpışmayı
"<ad>_collision" ve linkteki sırası 0 değilse "_<sıra>" ekiyle adlandırır
(sdformat parser_urdf.cc, CreateCollisions). Ayak temas sensörü çarpışmayı
bu adla bulur; o yüzden ayak küresi tibia linkinin İLK çarpışması ve adı
sabit: SDF'te "leg{i}_tibia_foot_collision". Adlar link adını içerir
(sdformat bunu bekliyor).

Gazebo (gazebo=GazeboOptions): ros2_control bloğu (18 eklem), gz_ros2_control
eklentisi, IMU ve ayak temas sensörleri eklenir. Bunlar ROS/Gazebo dışında
hiçbir şeyi etkilemez; RViz için gerekmez.

Servo modeli (GazeboOptions.servo):
  "torque"    eklem EFOR komutu alır; tork control.py'deki pid_controller'da
              hesaplanır (P, durma torkuyla sınırlı) ve eklem sönümü URDF'te
              (<dynamics damping>). hexapod_rl.sim'in tork modeliyle aynı:
              tork = sertlik x hata - sönüm x hız, |tork| <= durma torku.
              Fark: tork-hız doğrusu yok (tork sınırı hızla azalmıyor).
  "velocity"  eski model: konum komutu gz_ros2_control'de hız kontrolüyle
              uygulanır (zaman sabiti T). Ayaklar kayar (PROJE_DEVIR §12.18);
              yalnız karşılaştırma için tutuluyor.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import yaml

from .interface import IMU_FRAME, IMU_TOPIC, JOINT_PARTS, foot_contact_topic
from .model import CollisionBox, Inertial, LinkModel, RobotModel

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
    """ör. leg0_coxa_joint; ayak için leg0_foot_joint (sabit)."""
    return f"leg{leg}_{part}_joint"


def foot_collision_name(leg: int) -> str:
    """Ayak küresinin URDF adı. SDF'teki adı bunun sonuna "_collision" eklenmiş hâli."""
    return f"{link_name(leg, 'tibia')}_foot"


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


SERVO_MODELS = ("torque", "velocity")


@dataclass(frozen=True)
class GazeboOptions:
    """Gazebo'ya özgü eklerin girdileri.

    controllers_yaml: gz_ros2_control'ün okuyacağı kontrolcü dosyasının
    MUTLAK yolu (hexapod_description.control.write_controllers_yaml üretir;
    aynı servo modeliyle).
    servo: "torque" ya da "velocity" (modül açıklaması).
    """

    controllers_yaml: str
    servo: str = "torque"
    imu_rate_hz: float = 100.0
    contact_rate_hz: float = 100.0

    def __post_init__(self) -> None:
        if self.servo not in SERVO_MODELS:
            raise ValueError(f"servo modeli {SERVO_MODELS} olmalı, {self.servo!r} verildi")


def build_urdf(model: RobotModel, meshes: MeshSet | None = None,
               gazebo: GazeboOptions | None = None) -> str:
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
    ET.SubElement(robot, "link", name=IMU_FRAME)
    _fixed(robot, f"{IMU_FRAME}_joint", "base_link", IMU_FRAME, (0.0, 0.0, 0.0))

    for leg_id in sorted(model.mounts):
        _leg(robot, model, leg_id, meshes)

    if gazebo is not None:
        _gazebo(robot, model, gazebo)

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

    for part in JOINT_PARTS:
        name = link_name(leg_id, part)
        # Ayak küresi tibia'nın ilk çarpışması (bkz. modül açıklaması).
        first = [_foot_sphere(model, leg_id)] if part == "tibia" else []
        child = _link(robot, name, model.leg_link(leg_id, part), "bacak", meshes, part, first)
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

    foot = link_name(leg_id, "foot")
    ET.SubElement(robot, "link", name=foot)
    _fixed(robot, joint_name(leg_id, "foot"), parent, foot, (0.0, 0.0, -model.tibia))


def _foot_sphere(model: RobotModel, leg_id: int) -> ET.Element:
    """Kürenin alt ucu tibia uzunluğunda (ayak noktasında)."""
    r = model.foot_radius
    collision = ET.Element("collision", name=foot_collision_name(leg_id))
    _origin(collision, (0.0, 0.0, -(model.tibia - r)))
    ET.SubElement(ET.SubElement(collision, "geometry"), "sphere", radius=_num(r))
    return collision


def _link(robot: ET.Element, name: str, link: LinkModel, material: str,
          meshes: MeshSet | None, mesh_key: str, first_collisions=()) -> str:
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

    for collision in first_collisions:
        el.append(collision)
    for i, box in enumerate(link.boxes):
        _box(ET.SubElement(el, "collision", name=f"{name}_box{i}"), box)
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


# ---------------------------------------------------------------------------
# Gazebo
# ---------------------------------------------------------------------------


def _gazebo(robot: ET.Element, model: RobotModel, opts: GazeboOptions) -> None:
    # ros2_control: 18 eklem; durum konum + hız (+ tork modelinde efor).
    torque = opts.servo == "torque"
    rc = ET.SubElement(robot, "ros2_control", name="GazeboSimSystem", type="system")
    ET.SubElement(ET.SubElement(rc, "hardware"), "plugin").text = "gz_ros2_control/GazeboSimSystem"
    # Sıra interface.joint_names() ile aynı (bacak bacak, coxa-femur-tibia).
    for leg in sorted(model.mounts):
        for part in JOINT_PARTS:
            name = joint_name(leg, part)
            if torque:
                _rc_joint_effort(rc, name, model.effort)
                # tork modelinin "- sönüm x hız" terimi: fizik motoru her adımda uygular
                ET.SubElement(robot.find(f"joint[@name='{name}']"), "dynamics",
                              damping=_num(model.servo_damping))
            else:
                _rc_joint(rc, name, model.limits[(leg, part)])

    plugin = ET.SubElement(ET.SubElement(robot, "gazebo"), "plugin",
                           filename="gz_ros2_control-system",
                           name="gz_ros2_control::GazeboSimROS2ControlPlugin")
    ET.SubElement(plugin, "parameters").text = opts.controllers_yaml
    if not torque:
        # Konum komutu hız kontrolüyle uygulanır:
        # hız = kazanç x hata x kontrol hızı -> zaman sabiti T (robot.yaml).
        ET.SubElement(plugin, "position_proportional_gain").text = _num(model.position_gain())

    # IMU: imu_link base_link'e sabit, sdformat onu base_link'e birleştirir.
    ref = ET.SubElement(robot, "gazebo", reference=IMU_FRAME)
    imu = ET.SubElement(ref, "sensor", name="imu", type="imu")
    ET.SubElement(imu, "always_on").text = "true"
    ET.SubElement(imu, "update_rate").text = _num(opts.imu_rate_hz)
    ET.SubElement(imu, "topic").text = IMU_TOPIC
    ET.SubElement(imu, "gz_frame_id").text = IMU_FRAME

    # Ayak temasları (YALNIZCA simülasyon; gerçek robotta bu sensör yok).
    for leg in sorted(model.mounts):
        ref = ET.SubElement(robot, "gazebo", reference=link_name(leg, "tibia"))
        sensor = ET.SubElement(ref, "sensor", name=f"leg{leg}_foot_contact", type="contact")
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = _num(opts.contact_rate_hz)
        # gz-sim Contact sistemi konuyu <contact> içinde okur (gz-sim10 örneği).
        contact = ET.SubElement(sensor, "contact")
        ET.SubElement(contact, "collision").text = foot_collision_name(leg) + "_collision"
        ET.SubElement(contact, "topic").text = foot_contact_topic(leg)


def _rc_joint_effort(rc: ET.Element, name: str, effort: float) -> None:
    joint = ET.SubElement(rc, "joint", name=name)
    cmd = ET.SubElement(joint, "command_interface", name="effort")
    ET.SubElement(cmd, "param", name="min").text = _num(-effort)
    ET.SubElement(cmd, "param", name="max").text = _num(effort)
    state = ET.SubElement(joint, "state_interface", name="position")
    ET.SubElement(state, "param", name="initial_value").text = "0"
    ET.SubElement(joint, "state_interface", name="velocity")
    ET.SubElement(joint, "state_interface", name="effort")


def _rc_joint(rc: ET.Element, name: str, limit) -> None:
    joint = ET.SubElement(rc, "joint", name=name)
    cmd = ET.SubElement(joint, "command_interface", name="position")
    ET.SubElement(cmd, "param", name="min").text = _num(limit.lower)
    ET.SubElement(cmd, "param", name="max").text = _num(limit.upper)
    state = ET.SubElement(joint, "state_interface", name="position")
    ET.SubElement(state, "param", name="initial_value").text = "0"
    ET.SubElement(joint, "state_interface", name="velocity")


# ---------------------------------------------------------------------------


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
