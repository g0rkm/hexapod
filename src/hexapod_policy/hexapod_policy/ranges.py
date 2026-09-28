"""Mesafe sensörleri: yerleşimi robot.yaml'dan oku, ayrı konulardan gelen
ölçümleri bir sete topla (saf Python; düğüm bunları kullanır, testleri ROS'suz).

Kaldırma refleksi (lift_reflex) her sensörün gövdedeki yerini ve bakışını
ister. Bunlar robot.yaml -> sensors.range_finders'tan gelir (D8 doldurur):
position_m [x, y, z], direction_deg, pitch_deg ve ortak max_range_m. Biri
bilinmiyorsa MissingValue: yerleşim uydurulmaz (CLAUDE.md temel kuralı; simde
denenen yerleşim DENEYSEL, robotta doğru olduğu bilinmiyor).

Sürücü düğümü (hexapod_sensors) her sensörü ayrı bir konuya yayınlar
(/range0, /range1, ...; sensor_msgs/Range). Refleks ise bütün sensörlerin
aynı turdaki ölçümünü ister (PolicyController.on_ranges): RangeCollector her
sensörden birer ölçüm gelince tam seti verir ve yeniden başlar. Bir sensör
susarsa set hiç tamamlanmaz, denetleyicideki ölçüm bayatlar ve refleks kör
davranışa döner (range_timeout_s); eksik seti eski değerle doldurmak yerine
bu seçildi.
"""

from __future__ import annotations

import math
from typing import Sequence

from hexapod_driver.config import RobotConfig, Value
from hexapod_driver.errors import MissingValue

from .lift_reflex import RangeSensor

_SOURCES = {
    "position_m": "MONTAJ - sensörün gövdedeki yeri [x, y, z], m, gövde çerçevesinde (D8)",
    "direction_deg": "MONTAJ - sensör nereye bakıyor, yukarıdan: 0 ileri, + sola (D8)",
    "pitch_deg": "MONTAJ - sensör kaç derece aşağı bakıyor (D8; simde 20-25° çalışıyor)",
}


def range_sensors_from_config(config: RobotConfig) -> list[tuple[int, RangeSensor]]:
    """robot.yaml -> [(sensör kimliği, RangeSensor)], kimlik sırasıyla. Kimlik,
    sürücünün yayın konusudur (/range<kimlik>). Eksik alan: MissingValue."""
    raw = ((config.raw.get("sensors") or {}).get("range_finders") or {})
    devices = raw.get("devices") or []
    if not devices:
        raise MissingValue("sensors.range_finders.devices",
                           "robot.yaml'da mesafe sensörü tanımlı değil")
    max_m = Value.parse(raw.get("max_range_m"), "sensors.range_finders.max_range_m")
    max_m = float(max_m.require())
    if not max_m > 0:
        raise ValueError(f"sensors.range_finders.max_range_m pozitif olmalı: {max_m}")
    out = []
    for n, entry in enumerate(devices):
        i = int(entry.get("id", n))
        base = f"sensors.range_finders.devices[{i}]"
        values = {key: Value.parse(entry.get(key), f"{base}.{key}") for key in _SOURCES}
        for key, value in values.items():
            if not value.known:
                raise MissingValue(value.path, value.source or _SOURCES[key])
        pos = values["position_m"].value
        if not (isinstance(pos, (list, tuple)) and len(pos) == 3):
            raise ValueError(f"{base}.position_m [x, y, z] olmalı: {pos!r}")
        x, y, z = (float(v) for v in pos)
        pitch = float(values["pitch_deg"].value)
        if not 0.0 < pitch < 90.0:
            raise ValueError(f"{base}.pitch_deg 0-90 arasında olmalı (aşağı bakış): {pitch}")
        out.append((i, RangeSensor(x, y, z, float(values["direction_deg"].value), pitch, max_m)))
    ids = [i for i, _ in out]
    if len(set(ids)) != len(ids):
        raise ValueError(f"sensör kimlikleri tekrar ediyor: {ids}")
    return sorted(out, key=lambda item: item[0])


class RangeCollector:
    """Sensör başına gelen ölçümleri tam bir sete toplar.

        c = RangeCollector(sensors)
        s = c.add(0, 0.41, max_range=1.2)   # None: set eksik
        ...
        s = c.add(2, 1.2, max_range=1.2)    # (d0, d1, d2): tam set, toplama yeniden başlar
    """

    def __init__(self, sensors: Sequence[RangeSensor]) -> None:
        self.sensors = tuple(sensors)
        self._slots: list[float | None] = [None] * len(self.sensors)

    def add(self, index: int, distance: float,
            max_range: float | None = None) -> tuple[float, ...] | None:
        """index. sensörün ölçümü, m. sensor_msgs/Range sözleşmesi (REP-117):
        +inf ya da >= max_range "görmüyor" (sensörün max_m'sine çevrilir),
        -inf "çok yakın" (0), NaN geçersiz (yok sayılır). Tam set olunca döner."""
        if not 0 <= index < len(self.sensors):
            return None
        d = float(distance)
        if math.isnan(d):
            return None
        s = self.sensors[index]
        if d == -math.inf or d < 0:
            d = 0.0
        elif d >= s.max_m or (max_range is not None and d >= max_range):
            d = s.max_m
        self._slots[index] = d
        if any(v is None for v in self._slots):
            return None
        out = tuple(self._slots)
        self._slots = [None] * len(self.sensors)
        return out


__all__ = ["RangeCollector", "range_sensors_from_config"]
