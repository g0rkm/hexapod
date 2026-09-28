"""DENEYSEL mesafe sensörü yerleşimli robot.yaml kopyası (simde ve testte refleks için).

    python -m hexapod_rl.deneysel_yerlesim /tmp/robot_deneysel.yaml [--pitch 20]

Politika düğümü refleksi açınca (-p reflex:=true) sensör yerleşimini
robot.yaml'dan okur; depodaki robot.yaml'da bu değerler D8'e kadar null ve
öyle kalmalı (değer uydurulmaz). ROS'lu simde uçtan uca denemek için bu araç
robot.yaml'ın bir KOPYASINA eğitimdeki DENEYSEL yerleşimi yazar
(train.py --reflex, olcum +refleks: gövde kenarında 0.10 m yarıçapta ileri ve
±90°, 0.02 m yukarıda, pitch derece aşağı; reflex_probe.ring_sensors) ve
kopyanın başına DENEYSEL notu düşer. Kopya robotta kullanılmaz.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

#: Eğitimdeki yerleşimin bakış yönleri (train.py --reflex).
YAWS_DEG = (0.0, 90.0, -90.0)

_HEADER = ("# DENEYSEL - hexapod_rl.deneysel_yerlesim üretti. Mesafe sensörü yerleşimi\n"
           "# eğitimdeki simülasyon önerisi, robotta ölçülmedi (D8). Robotta KULLANMAYIN.\n")


def experimental_config(out: str | Path, pitch_deg: float = 20.0,
                        config_path: str | Path | None = None) -> Path:
    """robot.yaml'ı (verilmezse depodakini) okuyup DENEYSEL yerleşimle out'a yazar."""
    from hexapod_driver.config import RobotConfig

    from .reflex_probe import ring_sensors

    source = RobotConfig.load(config_path).path
    raw = yaml.safe_load(Path(source).read_text(encoding="utf-8"))
    devices = raw["sensors"]["range_finders"]["devices"]
    sensors = ring_sensors(YAWS_DEG, pitch_deg)
    if len(devices) != len(sensors):
        raise ValueError(f"robot.yaml'da {len(devices)} mesafe sensörü var, "
                         f"deneysel yerleşim {len(sensors)} sensörlük")
    for device, s in zip(devices, sensors):
        device["position_m"] = [round(s.x, 6), round(s.y, 6), round(s.z, 6)]
        device["direction_deg"] = s.yaw_deg
        device["pitch_deg"] = s.pitch_deg
    path = Path(out)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(_HEADER + yaml.safe_dump(raw, allow_unicode=True, sort_keys=False),
                    encoding="utf-8")
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="DENEYSEL mesafe sensörü yerleşimli config kopyası")
    parser.add_argument("out", type=Path)
    parser.add_argument("--pitch", type=float, default=20.0, help="aşağı bakış, derece")
    parser.add_argument("--config", type=Path, default=None, help="kaynak robot.yaml")
    args = parser.parse_args(argv)
    print(experimental_config(args.out, args.pitch, args.config))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
