"""robot.yaml ve calibration.yaml yükleyicisi.

Tasarım kuralı: eksik (null) bir değer asla varsayılanla doldurulmaz.
Değer okunmaya çalışıldığında MissingValue fırlar. Böylece "robot garip
yürüyor" yerine "tibia uzunluğu girilmemiş" hatası alınır.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator

import yaml

from .errors import ConfigError, MissingValue

JOINT_NAMES = ("coxa", "femur", "tibia")
LEG_COUNT = 6


# ---------------------------------------------------------------------------
# Kaynak bilgisi taşıyan değer
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Value:
    """Bir yapılandırma değeri ve nereden geldiği.

    robot.yaml'daki {value, source, measured} üçlüsünün karşılığı.
    """

    path: str
    value: Any
    source: str = ""
    measured: bool = False

    @property
    def known(self) -> bool:
        return self.value is not None

    def require(self) -> Any:
        """Değeri döndür; yoksa nereden geleceğini söyleyerek hata ver."""
        if self.value is None:
            raise MissingValue(self.path, self.source)
        return self.value

    @classmethod
    def parse(cls, node: Any, path: str) -> "Value":
        """Hem {value,source,measured} sözlüğünü hem de çıplak sayıyı kabul et."""
        if isinstance(node, dict) and "value" in node:
            return cls(
                path=path,
                value=node.get("value"),
                source=str(node.get("source", "")),
                measured=bool(node.get("measured", False)),
            )
        return cls(path=path, value=node, source="", measured=False)


# ---------------------------------------------------------------------------
# Alt yapılar
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class DriverSpec:
    id: int
    type: str
    i2c_bus: int
    address: Value


@dataclass(frozen=True)
class JointSpec:
    leg: int
    joint: str
    driver: Value
    channel: Value
    limit_min: Value
    limit_max: Value

    @property
    def key(self) -> str:
        """calibration.yaml içinde kullanılan anahtar, ör. 'leg0_coxa'."""
        return f"leg{self.leg}_{self.joint}"

    def __str__(self) -> str:
        return f"bacak {self.leg} {self.joint}"


@dataclass(frozen=True)
class LegSpec:
    id: int
    azimuth_deg: float
    mirrored: bool
    label: str | None


# ---------------------------------------------------------------------------
# Ana yapılandırma
# ---------------------------------------------------------------------------


@dataclass
class RobotConfig:
    raw: dict
    path: Path

    segments: dict[str, Value] = field(default_factory=dict)
    femur_joint_z_offset: Value = None  # type: ignore[assignment]
    coxa_axis_radius: Value = None  # type: ignore[assignment]
    standing_height: Value = None  # type: ignore[assignment]
    total_mass_kg: Value = None  # type: ignore[assignment]
    forward_offset_deg: Value = None  # type: ignore[assignment]

    legs: list[LegSpec] = field(default_factory=list)
    drivers: dict[int, DriverSpec] = field(default_factory=dict)
    joints: list[JointSpec] = field(default_factory=list)

    pwm_frequency_hz: float = 50.0
    nominal_range_deg: float = 180.0
    pulse_us_min: int = 500
    pulse_us_max: int = 2500
    servo_model: str = ""

    # -- yükleme ----------------------------------------------------------

    @classmethod
    def load(cls, path: str | os.PathLike | None = None) -> "RobotConfig":
        p = Path(path) if path else default_config_dir() / "robot.yaml"
        if not p.is_file():
            raise ConfigError(f"robot.yaml bulunamadı: {p}")
        try:
            raw = yaml.safe_load(p.read_text(encoding="utf-8"))
        except yaml.YAMLError as exc:
            raise ConfigError(f"{p} ayrıştırılamadı: {exc}") from exc
        if not isinstance(raw, dict):
            raise ConfigError(f"{p} bir eşleme (mapping) değil.")
        return cls._build(raw, p)

    @classmethod
    def _build(cls, raw: dict, p: Path) -> "RobotConfig":
        cfg = cls(raw=raw, path=p)

        seg = (raw.get("leg") or {}).get("segments") or {}
        for name in JOINT_NAMES:
            cfg.segments[name] = Value.parse(seg.get(name), f"leg.segments.{name}")

        cfg.femur_joint_z_offset = Value.parse(
            (raw.get("leg") or {}).get("femur_joint_z_offset"), "leg.femur_joint_z_offset"
        )

        body = raw.get("body") or {}
        cfg.coxa_axis_radius = Value.parse(body.get("coxa_axis_radius"), "body.coxa_axis_radius")
        cfg.standing_height = Value.parse(body.get("standing_height"), "body.standing_height")
        cfg.total_mass_kg = Value.parse(body.get("total_mass_kg"), "body.total_mass_kg")

        cfg.forward_offset_deg = Value.parse(
            ((raw.get("frames") or {}).get("body") or {}).get("forward_offset_deg"),
            "frames.body.forward_offset_deg",
        )

        for entry in raw.get("legs") or []:
            cfg.legs.append(
                LegSpec(
                    id=int(entry["id"]),
                    azimuth_deg=float(entry["azimuth_deg"]),
                    mirrored=bool(entry["mirrored"]),
                    label=entry.get("label"),
                )
            )

        servo = raw.get("servo") or {}
        cfg.servo_model = str(servo.get("model", ""))
        cfg.pwm_frequency_hz = float(servo.get("pwm_frequency_hz", 50.0))
        cfg.nominal_range_deg = float(servo.get("nominal_range_deg", 180.0))
        hard = servo.get("pulse_us_hard_limits") or {}
        cfg.pulse_us_min = int(hard.get("min", 500))
        cfg.pulse_us_max = int(hard.get("max", 2500))
        if cfg.pulse_us_min >= cfg.pulse_us_max:
            raise ConfigError("servo.pulse_us_hard_limits: min >= max")

        for entry in raw.get("drivers") or []:
            did = int(entry["id"])
            cfg.drivers[did] = DriverSpec(
                id=did,
                type=str(entry.get("type", "PCA9685")),
                i2c_bus=int(entry.get("i2c_bus", 1)),
                address=Value.parse(entry.get("address"), f"drivers[{did}].address"),
            )

        for entry in raw.get("joints") or []:
            leg = int(entry["leg"])
            joint = str(entry["joint"])
            if joint not in JOINT_NAMES:
                raise ConfigError(f"Bilinmeyen eklem adı: {joint!r}")
            base = f"joints[leg{leg}_{joint}]"
            limits = entry.get("limits_deg") or {}
            cfg.joints.append(
                JointSpec(
                    leg=leg,
                    joint=joint,
                    driver=Value.parse(entry.get("driver"), f"{base}.driver"),
                    channel=Value.parse(entry.get("channel"), f"{base}.channel"),
                    limit_min=Value.parse(limits.get("min"), f"{base}.limits_deg.min"),
                    limit_max=Value.parse(limits.get("max"), f"{base}.limits_deg.max"),
                )
            )

        cfg._check_shape()
        return cfg

    def _check_shape(self) -> None:
        if len(self.legs) != LEG_COUNT:
            raise ConfigError(f"{LEG_COUNT} bacak bekleniyordu, {len(self.legs)} tanımlı.")
        expected = {(leg.id, j) for leg in self.legs for j in JOINT_NAMES}
        actual = {(j.leg, j.joint) for j in self.joints}
        if expected != actual:
            eksik = sorted(expected - actual)
            fazla = sorted(actual - expected)
            raise ConfigError(f"joints listesi eksik/fazla. eksik={eksik} fazla={fazla}")

    # -- sorgular ---------------------------------------------------------

    def joint(self, leg: int, joint: str) -> JointSpec:
        for spec in self.joints:
            if spec.leg == leg and spec.joint == joint:
                return spec
        raise ConfigError(f"Böyle bir eklem yok: bacak {leg} {joint}")

    def unknowns(self) -> list[Value]:
        """Henüz girilmemiş (null) tüm alanlar, yol sırasına göre."""
        out: list[Value] = [v for v in self._all_values() if not v.known]
        return sorted(out, key=lambda v: v.path)

    def unverified(self) -> list[Value]:
        """Değeri olan ama monte robottan ölçülmemiş alanlar."""
        return sorted(
            (v for v in self._all_values() if v.known and not v.measured and v.source),
            key=lambda v: v.path,
        )

    def _all_values(self) -> Iterator[Value]:
        yield from self.segments.values()
        yield self.femur_joint_z_offset
        yield self.coxa_axis_radius
        yield self.standing_height
        yield self.total_mass_kg
        yield self.forward_offset_deg
        for d in self.drivers.values():
            yield d.address
        for j in self.joints:
            yield j.driver
            yield j.channel
            yield j.limit_min
            yield j.limit_max

    def wiring_is_complete(self) -> bool:
        """Servoları sürebilmek için gereken minimum bilgi girilmiş mi?

        Kinematik parametreleri (segment uzunlukları vb.) burada aranmaz;
        servo sürmek için onlara ihtiyaç yok.
        """
        if any(not d.address.known for d in self.drivers.values()):
            return False
        return all(j.driver.known and j.channel.known for j in self.joints)

    def wiring_gaps(self) -> list[Value]:
        gaps = [d.address for d in self.drivers.values() if not d.address.known]
        for j in self.joints:
            if not j.driver.known:
                gaps.append(j.driver)
            if not j.channel.known:
                gaps.append(j.channel)
        return gaps


def default_config_dir() -> Path:
    """config/ klasörünü bul.

    Sıra: HEXAPOD_CONFIG_DIR ortam değişkeni -> bu dosyadan yukarı doğru
    arama -> çalışma dizininden yukarı doğru arama.
    """
    env = os.environ.get("HEXAPOD_CONFIG_DIR")
    if env:
        return Path(env)
    for start in (Path(__file__).resolve(), Path.cwd().resolve() / "_"):
        for parent in start.parents:
            candidate = parent / "config"
            if (candidate / "robot.yaml").is_file():
                return candidate
    raise ConfigError(
        "config/robot.yaml bulunamadı. HEXAPOD_CONFIG_DIR ortam değişkenini ayarlayın."
    )
