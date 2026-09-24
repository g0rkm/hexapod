"""Altı bacak + gövde pozu.

Gövde çerçevesi (ROS REP-103)
-----------------------------
Orijin coxa eksenlerinin geçtiği çemberin merkezi, bacak montaj referans
düzleminin yüksekliğinde.
  +x ileri, +y sol, +z yukarı.

Gövde pozu
----------
Ayaklar yere sabit kabul edilir; gövde BodyPose kadar kaydırılıp
döndürülür. Ayak hedefleri "dünya" çerçevesinde verilir; bu çerçeve
nötr pozda gövde çerçevesiyle çakışır. Dönme sırası ZYX (önce yaw, sonra
pitch, sonra roll), REP-103 ile aynı.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from . import leg as leg_kin
from .errors import ReachError
from .leg import JointAngles, LegGeometry, Vec3


@dataclass(frozen=True)
class LegMount:
    """Bir bacağın femur-eklemi yüksekliğindeki coxa ekseni, gövde çerçevesinde."""

    x: float
    y: float
    z: float
    yaw: float  # radyan; bacağın dışarı baktığı yön


@dataclass(frozen=True)
class BodyPose:
    """Gövdenin nötr konumuna göre kayması (mm) ve dönmesi (derece)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


NEUTRAL = BodyPose()


@dataclass
class HexapodKinematics:
    leg: LegGeometry
    mounts: dict[int, LegMount] = field(default_factory=dict)

    # -- kurulum ----------------------------------------------------------

    @classmethod
    def from_config(cls, config) -> "HexapodKinematics":
        """robot.yaml'dan kur. Eksik bir değer varsa MissingValue fırlar."""
        seg = config.segments
        geometry = LegGeometry(
            coxa=float(seg["coxa"].require()),
            femur=float(seg["femur"].require()),
            tibia=float(seg["tibia"].require()),
        )
        radius = float(config.coxa_axis_radius.require())
        height = float(config.femur_joint_z_offset.require())
        forward = float(config.forward_offset_deg.require())

        mounts = {}
        for spec in config.legs:
            yaw = math.radians(wrap_deg(forward - spec.azimuth_deg))
            mounts[spec.id] = LegMount(
                x=radius * math.cos(yaw),
                y=radius * math.sin(yaw),
                z=height,
                yaw=yaw,
            )
        return cls(leg=geometry, mounts=mounts)

    # -- çerçeve dönüşümleri ----------------------------------------------

    def to_leg_frame(self, leg_id: int, p: Vec3) -> Vec3:
        m = self.mounts[leg_id]
        dx, dy, dz = p[0] - m.x, p[1] - m.y, p[2] - m.z
        c, s = math.cos(m.yaw), math.sin(m.yaw)
        return (c * dx + s * dy, -s * dx + c * dy, dz)

    def to_body_frame(self, leg_id: int, p: Vec3) -> Vec3:
        m = self.mounts[leg_id]
        c, s = math.cos(m.yaw), math.sin(m.yaw)
        return (m.x + c * p[0] - s * p[1], m.y + s * p[0] + c * p[1], m.z + p[2])

    # -- kinematik --------------------------------------------------------

    def neutral_stance(self) -> dict[int, Vec3]:
        """Bütün eklemler sıfırdayken ayakların gövde çerçevesindeki yeri.

        Bu bir yürüyüş duruşu değil, sadece kalibrasyon sıfırının karşılığı.
        Gerçek duruş (ayak açıklığı, gövde yüksekliği) gait katmanında seçilir.
        """
        tip = leg_kin.forward(self.leg, leg_kin.ZERO)
        return {i: self.to_body_frame(i, tip) for i in self.mounts}

    def forward(self, angles: dict[int, JointAngles]) -> dict[int, Vec3]:
        """Eklem açılarından ayakların gövde çerçevesindeki yeri."""
        return {
            i: self.to_body_frame(i, leg_kin.forward(self.leg, a))
            for i, a in angles.items()
        }

    def inverse(self, feet: dict[int, Vec3],
                pose: BodyPose = NEUTRAL) -> dict[int, JointAngles]:
        """Dünya çerçevesindeki ayak hedefleri + gövde pozu -> eklem açıları.

        Erişilemeyen bir ayak varsa hangi bacak olduğunu söyleyen ReachError
        fırlar; hiçbir bacak için kısmi sonuç dönmez.
        """
        rot = _rotation(pose)
        out: dict[int, JointAngles] = {}
        for i, p in feet.items():
            # Dünya -> gövde:  p_gövde = Rᵀ (p_dünya - t)
            d = (p[0] - pose.x, p[1] - pose.y, p[2] - pose.z)
            body = tuple(rot[0][k] * d[0] + rot[1][k] * d[1] + rot[2][k] * d[2]
                         for k in range(3))
            try:
                out[i] = leg_kin.inverse(self.leg, *self.to_leg_frame(i, body))
            except ReachError as exc:
                raise ReachError(f"bacak {i}: {exc}") from exc
        return out


def wrap_deg(angle: float) -> float:
    """Açıyı (-180, 180] aralığına getir."""
    a = math.fmod(angle, 360.0)
    if a <= -180.0:
        a += 360.0
    elif a > 180.0:
        a -= 360.0
    return a


def _rotation(pose: BodyPose) -> tuple[tuple[float, float, float], ...]:
    """Gövde -> dünya dönme matrisi, R = Rz(yaw) · Ry(pitch) · Rx(roll)."""
    r, p, y = (math.radians(v) for v in (pose.roll, pose.pitch, pose.yaw))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
