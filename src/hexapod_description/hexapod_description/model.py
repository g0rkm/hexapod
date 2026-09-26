"""Simülasyon modelinin verisi, robot.yaml'dan; birimler SI (m, kg, rad).

URDF üreticisi (ve ileride Gazebo/RL ortamı) sayıları buradan alır, böylece
her katman aynı kaynağı görür. Geometri ve bacak yerleşimi doğrudan
hexapod_kinematics'ten gelir: simülasyondaki eklem zinciri IK'nın
varsaydığıyla birebir aynı olmak ZORUNDA, yoksa simülasyonda öğrenilen her
şey gerçek robotta kayık basar.

Kütle, atalet ve çarpışma kutuları robot.yaml -> simulation.links altında,
tools/cad_sim_model.py'nin CAD'den hesapladığı TAHMİNLER. Normal bacak için
tutulurlar; aynalı bacaklarda y ekseni burada ters çevrilir.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from hexapod_driver.config import JOINT_NAMES, RobotConfig, Value
from hexapod_driver.errors import ConfigError
from hexapod_kinematics import HexapodKinematics, LegMount

Vec3 = tuple[float, float, float]

LINK_NAMES = ("body", "coxa", "femur", "tibia")

_MM = 1e-3
_KG_MM2 = 1e-6  # kg·mm² -> kg·m²


@dataclass(frozen=True)
class Inertial:
    mass: float  # kg
    com: Vec3  # m, link çerçevesinde
    inertia: tuple[float, float, float, float, float, float]  # ixx iyy izz ixy ixz iyz, kg·m²

    def mirrored_y(self) -> "Inertial":
        """Link çerçevesinin xz düzlemine göre yansıması (y -> -y)."""
        ixx, iyy, izz, ixy, ixz, iyz = self.inertia
        return Inertial(self.mass, (self.com[0], -self.com[1], self.com[2]),
                        (ixx, iyy, izz, -ixy, ixz, -iyz))


@dataclass(frozen=True)
class CollisionBox:
    center: Vec3  # m
    size: Vec3  # m

    def mirrored_y(self) -> "CollisionBox":
        return CollisionBox((self.center[0], -self.center[1], self.center[2]), self.size)


@dataclass(frozen=True)
class LinkModel:
    inertial: Inertial
    boxes: tuple[CollisionBox, ...]

    def mirrored_y(self) -> "LinkModel":
        return LinkModel(self.inertial.mirrored_y(), tuple(b.mirrored_y() for b in self.boxes))


@dataclass(frozen=True)
class JointLimit:
    lower: float  # rad
    upper: float  # rad
    # True: kalibrasyon limitleri (joints[*].limits_deg) yok, geçici
    # simulation.provisional_joint_limits_deg kullanıldı.
    provisional: bool


@dataclass
class RobotModel:
    name: str
    coxa: float  # m
    femur: float  # m
    tibia: float  # m
    foot_radius: float  # m
    mounts: dict[int, LegMount]  # x, y, z metre; yaw radyan
    mirrored: dict[int, bool]
    links: dict[str, LinkModel]  # normal bacak için
    limits: dict[tuple[int, str], JointLimit]
    effort: float  # N·m
    velocity: float  # rad/s
    servo_time_constant: float  # s (ROS simi: gz_ros2_control konum kazancı)
    control_rate: float  # Hz
    servo_stiffness: float  # N·m/rad (RL simi: tork tabanlı servo)
    servo_damping: float  # N·m·s/rad

    @classmethod
    def from_config(cls, config: RobotConfig) -> "RobotModel":
        """robot.yaml'dan kur. Eksik bir değer varsa MissingValue fırlar."""
        kin = HexapodKinematics.from_config(config)
        raw_leg = config.raw.get("leg") or {}
        sim = config.raw.get("simulation") or {}

        foot = Value.parse(raw_leg.get("foot_tip_radius"), "leg.foot_tip_radius")
        servo = sim.get("servo") or {}
        effort = Value.parse(servo.get("effort_nm"), "simulation.servo.effort_nm")
        velocity = Value.parse(servo.get("velocity_rad_s"), "simulation.servo.velocity_rad_s")
        time_constant = Value.parse(servo.get("time_constant_s"),
                                    "simulation.servo.time_constant_s")
        rate = Value.parse((sim.get("control") or {}).get("update_rate_hz"),
                           "simulation.control.update_rate_hz")
        stiffness = Value.parse(servo.get("stiffness_nm_per_rad"),
                                "simulation.servo.stiffness_nm_per_rad")
        damping = Value.parse(servo.get("damping_nm_s_per_rad"),
                              "simulation.servo.damping_nm_s_per_rad")

        mounts = {
            i: LegMount(m.x * _MM, m.y * _MM, m.z * _MM, m.yaw) for i, m in kin.mounts.items()
        }
        return cls(
            name=str((config.raw.get("meta") or {}).get("name", "hexapod")),
            coxa=kin.leg.coxa * _MM,
            femur=kin.leg.femur * _MM,
            tibia=kin.leg.tibia * _MM,
            foot_radius=float(foot.require()) * _MM,
            mounts=mounts,
            mirrored={leg.id: leg.mirrored for leg in config.legs},
            links=_links(sim.get("links") or {}),
            limits=_limits(config, sim),
            effort=float(effort.require()),
            velocity=float(velocity.require()),
            servo_time_constant=float(time_constant.require()),
            control_rate=float(rate.require()),
            servo_stiffness=float(stiffness.require()),
            servo_damping=float(damping.require()),
        )

    def leg_link(self, leg_id: int, name: str) -> LinkModel:
        link = self.links[name]
        return link.mirrored_y() if self.mirrored[leg_id] else link

    def provisional_joints(self) -> list[tuple[int, str]]:
        return sorted(k for k, v in self.limits.items() if v.provisional)

    def position_gain(self) -> float:
        """gz_ros2_control position_proportional_gain.

        Konum komutu: hız = kazanç x hata x kontrol hızı, yani zaman sabiti
        T = 1 / (kazanç x hız). Kazanç > 1 salınım, > 2 kararsızlık demek
        (gz_ros2_control belgesi); öyleyse config tutarsızdır, sessizce
        kırpılmaz.
        """
        gain = 1.0 / (self.servo_time_constant * self.control_rate)
        if not 0.0 < gain <= 1.0:
            raise ConfigError(
                f"position_proportional_gain = {gain:.3f}; (0, 1] olmalı. "
                "simulation.servo.time_constant_s ya da simulation.control.update_rate_hz "
                "tutarsız (T x hız >= 1 olmalı)."
            )
        return gain

    def total_mass(self) -> float:
        legs = sum(self.links[n].inertial.mass for n in ("coxa", "femur", "tibia"))
        return self.links["body"].inertial.mass + len(self.mounts) * legs

    def with_body_mass_scale(self, scale: float) -> "RobotModel":
        """Gövde bağlantısının kütlesi ve ataleti x scale olan kopya (ağırlık
        merkezi ve çarpışma aynı: fark gövdeye yayılmış kabul edilir). RL alan
        rastgeleleştirmesi için (hexapod_rl.task.Randomization.body_mass_scale);
        robot.yaml değişmez."""
        if scale == 1.0:
            return self
        if not scale > 0:
            raise ValueError(f"kütle çarpanı pozitif olmalı: {scale}")
        body = self.links["body"]
        inertial = replace(body.inertial, mass=body.inertial.mass * scale,
                           inertia=tuple(v * scale for v in body.inertial.inertia))
        return replace(self, links={**self.links, "body": replace(body, inertial=inertial)})


def _links(raw: dict) -> dict[str, LinkModel]:
    out = {}
    for name in LINK_NAMES:
        node = raw.get(name) or {}
        base = f"simulation.links.{name}"

        def need(key: str):
            return Value.parse(node.get(key), f"{base}.{key}").require()

        mass = float(need("mass_kg"))
        com = [float(v) * _MM for v in need("com_mm")]
        inertia = [float(v) * _KG_MM2 for v in need("inertia_kg_mm2")]
        boxes = [[float(v) * _MM for v in b] for b in need("collision_boxes_mm")]
        if mass <= 0 or len(com) != 3 or len(inertia) != 6:
            raise ConfigError(f"{base}: kütle > 0, com 3 ve inertia 6 elemanlı olmalı.")
        if not boxes or any(len(b) != 6 for b in boxes):
            raise ConfigError(f"{base}.collision_boxes_mm: [merkez xyz, boy xyz] listesi olmalı.")
        out[name] = LinkModel(
            Inertial(mass, tuple(com), tuple(inertia)),
            tuple(CollisionBox(tuple(b[:3]), tuple(b[3:])) for b in boxes),
        )
    return out


def _limits(config: RobotConfig, sim: dict) -> dict[tuple[int, str], JointLimit]:
    """Kalibrasyon limitleri varsa onlar; yoksa geçici simülasyon limitleri.

    Geçici limitler de robot.yaml'daki kaynaklı bir değer; kod burada kendi
    varsayılanını uydurmaz.
    """
    provisional = Value.parse(sim.get("provisional_joint_limits_deg"),
                              "simulation.provisional_joint_limits_deg")
    out = {}
    for spec in config.joints:
        if spec.limit_min.known and spec.limit_max.known:
            lo, hi, temp = float(spec.limit_min.value), float(spec.limit_max.value), False
        else:
            table = provisional.require()
            if spec.joint not in table:
                raise ConfigError(f"{provisional.path}: '{spec.joint}' için limit yok.")
            lo, hi = (float(v) for v in table[spec.joint])
            temp = True
        if lo >= hi:
            raise ConfigError(f"{spec}: limit min ({lo}) max'tan ({hi}) küçük olmalı.")
        out[(spec.leg, spec.joint)] = JointLimit(math.radians(lo), math.radians(hi), temp)
    missing = {(leg.id, j) for leg in config.legs for j in JOINT_NAMES} - set(out)
    if missing:
        raise ConfigError(f"Limit tanımı eksik eklemler: {sorted(missing)}")
    return out
