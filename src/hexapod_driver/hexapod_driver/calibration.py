"""calibration.yaml okuma/yazma.

Bu dosya ELLE AYARLANAN değerleri tutar: her eklemin merkez darbesi, dönüş
yönü ve derece başına mikrosaniye katsayısı. tools/calibrate.py üretir.

Brifteki risk maddesi: "Kalibrasyonun kayıt altına alınmaması" — bu yüzden
yazma işlemi atomiktir ve mevcut kayıtları asla sessizce silmez.
"""

from __future__ import annotations

import os
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from .config import JOINT_NAMES, default_config_dir
from .errors import ConfigError, MissingValue

SCHEMA_VERSION = 1

HEADER = """\
# =============================================================================
# hexapod / config/calibration.yaml
# =============================================================================
# tools/calibrate.py tarafından üretilir. ELLE de düzenlenebilir.
#
# Her eklem için:
#   center_us  : eklemin 0 derece kabul edilen konumundaki darbe genişliği.
#   direction  : +1 veya -1. Darbe artarken eklemin pozitif yönde dönüp
#                dönmediği. Aynalı bacaklarda ters olabilir; varsayılmaz.
#   us_per_deg : derece başına mikrosaniye. Açı komutu verebilmek için gerek.
#                Sadece merkezleme yapıldıysa null kalır.
#   limit_*_us : eklemin mekanik olarak durduğu uçlar, darbe cinsinden.
#                Dereceye çevrilmiş hâli config/robot.yaml -> limits_deg.
#
# null olan bir alan, o özelliği kullanan çağrıda hata verir.
# =============================================================================
"""


@dataclass
class JointCalibration:
    center_us: int | None = None
    direction: int | None = None
    us_per_deg: float | None = None
    limit_min_us: int | None = None
    limit_max_us: int | None = None
    calibrated_at: str | None = None

    def limits_deg(self) -> tuple[float, float] | None:
        """Ölçülen mekanik uçları dereceye çevir.

        center_us, direction ve us_per_deg bilinmeden çevrilemez; o durumda
        None döner (uydurma yapılmaz).
        """
        if None in (self.center_us, self.direction, self.us_per_deg):
            return None
        if self.limit_min_us is None or self.limit_max_us is None:
            return None
        lo = (self.limit_min_us - self.center_us) / (self.direction * self.us_per_deg)
        hi = (self.limit_max_us - self.center_us) / (self.direction * self.us_per_deg)
        return (min(lo, hi), max(lo, hi))

    def require_center(self, key: str) -> int:
        if self.center_us is None:
            raise MissingValue(f"calibration.joints.{key}.center_us",
                               "tools/calibrate.py ile merkezleyin")
        return self.center_us

    def require_angle_terms(self, key: str) -> tuple[int, int, float]:
        center = self.require_center(key)
        if self.direction is None:
            raise MissingValue(f"calibration.joints.{key}.direction",
                               "tools/calibrate.py içinde 'dir' komutu")
        if self.us_per_deg is None:
            raise MissingValue(f"calibration.joints.{key}.us_per_deg",
                               "açı komutu için derece/us katsayısı gerekli")
        return center, self.direction, self.us_per_deg


@dataclass
class Calibration:
    joints: dict[str, JointCalibration]
    path: Path | None = None

    @classmethod
    def empty(cls) -> "Calibration":
        return cls(joints={})

    @classmethod
    def load(cls, path: str | os.PathLike | None = None,
             missing_ok: bool = True) -> "Calibration":
        p = Path(path) if path else default_config_dir() / "calibration.yaml"
        if not p.is_file():
            if missing_ok:
                return cls(joints={}, path=p)
            raise ConfigError(f"calibration.yaml bulunamadı: {p}")
        try:
            raw = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
        except yaml.YAMLError as exc:
            raise ConfigError(f"{p} ayrıştırılamadı: {exc}") from exc

        joints: dict[str, JointCalibration] = {}
        for key, node in (raw.get("joints") or {}).items():
            node = node or {}
            joints[key] = JointCalibration(
                center_us=_opt_int(node.get("center_us")),
                direction=_opt_int(node.get("direction")),
                us_per_deg=_opt_float(node.get("us_per_deg")),
                limit_min_us=_opt_int(node.get("limit_min_us")),
                limit_max_us=_opt_int(node.get("limit_max_us")),
                calibrated_at=node.get("calibrated_at"),
            )
        return cls(joints=joints, path=p)

    def get(self, key: str) -> JointCalibration:
        return self.joints.setdefault(key, JointCalibration())

    def set_center(self, key: str, center_us: int) -> None:
        entry = self.get(key)
        entry.center_us = int(center_us)
        entry.calibrated_at = _now()

    def set_direction(self, key: str, direction: int) -> None:
        if direction not in (1, -1):
            raise ValueError("direction yalnızca +1 veya -1 olabilir")
        entry = self.get(key)
        entry.direction = direction
        entry.calibrated_at = _now()

    def set_limit(self, key: str, which: str, pulse_us: int) -> None:
        if which not in ("min", "max"):
            raise ValueError("which yalnızca 'min' veya 'max' olabilir")
        entry = self.get(key)
        setattr(entry, f"limit_{which}_us", int(pulse_us))
        entry.calibrated_at = _now()

    def set_span(self, key: str, pulse_us: int, degrees: float) -> float:
        """Bilinen bir açıdaki darbeden us_per_deg hesapla.

        degrees, merkeze göre ölçülen gerçek eklem açısı (açıölçerle).
        """
        entry = self.get(key)
        center = entry.require_center(key)
        if entry.direction is None:
            raise MissingValue(f"calibration.joints.{key}.direction",
                               "önce 'dir +' veya 'dir -' ile yönü belirleyin")
        if abs(degrees) < 1.0:
            raise ValueError("Açı sıfıra çok yakın; en az 1 derece uzaklıkta ölçün")
        entry.us_per_deg = abs((pulse_us - center) / (entry.direction * degrees))
        entry.calibrated_at = _now()
        return entry.us_per_deg

    def save(self, path: str | os.PathLike | None = None) -> Path:
        """Atomik yazma: önce .tmp, sonra yerine taşı."""
        p = Path(path) if path else (self.path or default_config_dir() / "calibration.yaml")
        body: dict[str, Any] = {
            "schema_version": SCHEMA_VERSION,
            "generated_by": "tools/calibrate.py",
            "generated_at": _now(),
            "joints": {
                key: {k: v for k, v in asdict(cal).items()}
                for key, cal in sorted(self.joints.items(), key=_joint_sort_key)
            },
        }
        text = HEADER + yaml.safe_dump(body, allow_unicode=True, sort_keys=False)
        tmp = p.with_suffix(p.suffix + ".tmp")
        p.parent.mkdir(parents=True, exist_ok=True)
        tmp.write_text(text, encoding="utf-8")
        os.replace(tmp, p)
        self.path = p
        return p

    def summary(self) -> tuple[int, int]:
        """(merkezlenmiş eklem sayısı, yönü belirlenmiş eklem sayısı)."""
        centered = sum(1 for c in self.joints.values() if c.center_us is not None)
        directed = sum(1 for c in self.joints.values() if c.direction is not None)
        return centered, directed


def _joint_sort_key(item: tuple[str, JointCalibration]) -> tuple[int, int]:
    key = item[0]
    try:
        leg_part, joint_part = key.split("_", 1)
        return (int(leg_part.removeprefix("leg")), JOINT_NAMES.index(joint_part))
    except (ValueError, IndexError):
        return (99, 99)


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _opt_int(v: Any) -> int | None:
    return None if v is None else int(v)


def _opt_float(v: Any) -> float | None:
    return None if v is None else float(v)
