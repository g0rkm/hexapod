"""hexapod_policy.ranges — refleksin sensör yerleşimi (robot.yaml) ve ölçüm toplama.

Saf Python, her yerde koşar. Düğümün kendisi test_policy_node.py'de (ROS'lu).
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from hexapod_driver.config import RobotConfig
from hexapod_driver.errors import MissingValue
from hexapod_policy.lift_reflex import RangeSensor
from hexapod_policy.ranges import RangeCollector, range_sensors_from_config

REPO_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"


def config_with(tmp_path: Path, devices, max_range=1.2) -> RobotConfig:
    """Depodaki robot.yaml'ın kopyası, mesafe sensörleri verilenle değiştirilmiş."""
    raw = yaml.safe_load(REPO_CONFIG.read_text(encoding="utf-8"))
    rf = raw["sensors"]["range_finders"]
    rf["devices"] = devices
    rf["max_range_m"] = {"value": max_range, "source": "test", "measured": False}
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(raw, allow_unicode=True), encoding="utf-8")
    return RobotConfig.load(path)


def device(i, pos=(0.1, 0.0, 0.02), yaw=0.0, pitch=20.0):
    return {"id": i, "xshut_gpio": None, "address": None, "direction_deg": yaw,
            "position_m": list(pos) if pos is not None else None, "pitch_deg": pitch}


def test_yerlesim_robot_yamldan_okunur_kimlik_sirasiyla(tmp_path):
    cfg = config_with(tmp_path, [device(2, (0.0, -0.1, 0.02), -90.0),
                                 device(0), device(1, (0.0, 0.1, 0.02), 90.0, 25.0)])
    placed = range_sensors_from_config(cfg)
    assert [i for i, _ in placed] == [0, 1, 2]                 # konu: /range<kimlik>
    assert placed[1][1] == RangeSensor(0.0, 0.1, 0.02, 90.0, 25.0, 1.2)
    assert all(s.max_m == 1.2 for _, s in placed)              # ortak "görmüyor" eşiği


def test_yerlesim_bilinmiyorsa_eksik_alani_soyler(tmp_path):
    """Değer uydurulmaz: D8 doldurana kadar refleks açılamaz ve neyin eksik
    olduğu söylenir (depodaki robot.yaml'da bugün hepsi null)."""
    with pytest.raises(MissingValue) as exc:
        range_sensors_from_config(config_with(tmp_path, [device(0), device(1, pos=None)]))
    assert "sensors.range_finders.devices[1].position_m" in str(exc.value)
    with pytest.raises(MissingValue):
        range_sensors_from_config(config_with(tmp_path, [device(0, pitch=None)]))
    with pytest.raises(MissingValue):
        range_sensors_from_config(config_with(tmp_path, [device(0)], max_range=None))


def test_gecersiz_yerlesim_reddedilir(tmp_path):
    for bad in ([device(0, pitch=0.0)], [device(0, pitch=90.0)],
                [device(0, pos=(0.1, 0.0))], [device(0), device(0)]):
        with pytest.raises(ValueError):
            range_sensors_from_config(config_with(tmp_path, bad))


SENSORS = [RangeSensor(0.1, 0.0, 0.02, 0.0, 20.0, 1.2),
           RangeSensor(0.0, 0.1, 0.02, 90.0, 20.0, 1.2),
           RangeSensor(0.0, -0.1, 0.02, -90.0, 20.0, 1.2)]


def test_her_sensorden_birer_olcum_gelince_tam_set():
    c = RangeCollector(SENSORS)
    assert c.add(0, 0.40) is None
    assert c.add(2, 0.35) is None
    assert c.add(0, 0.30) is None                              # aynı sensör: son ölçüm geçerli
    assert c.add(1, 0.33) == (0.30, 0.33, 0.35)
    assert c.add(1, 0.33) is None                              # yeni tur baştan
    assert c.add(7, 0.3) is None                               # bilinmeyen sensör yok sayılır


def test_range_mesaji_sozlesmesi():
    """REP-117: +inf ya da >= max_range görmüyor (sensörün max_m'si), -inf çok
    yakın (0), NaN geçersiz. Sürücü görmüyorken max_range yayınlıyor (S7)."""
    c = RangeCollector(SENSORS)
    assert c.add(0, math.nan) is None
    assert c.add(0, math.inf) is None
    assert c.add(1, -math.inf) is None
    assert c.add(2, 1.2, max_range=1.2) == (1.2, 0.0, 1.2)
    c.add(0, 0.9, max_range=0.8)                               # sürücünün menzili daha kısa
    c.add(1, 0.2)
    assert c.add(2, 0.3)[0] == 1.2


def test_deneysel_yerlesim_kopyaya_yazilir_kaynak_degismez(tmp_path):
    """Simde uçtan uca deneme için: eğitimdeki DENEYSEL yerleşim bir KOPYAYA;
    depodaki robot.yaml'a dokunulmaz (D8)."""
    from hexapod_rl.deneysel_yerlesim import experimental_config
    from hexapod_rl.reflex_probe import ring_sensors

    before = REPO_CONFIG.read_bytes()
    out = experimental_config(tmp_path / "deneysel.yaml", pitch_deg=20.0,
                              config_path=REPO_CONFIG)
    assert REPO_CONFIG.read_bytes() == before
    assert out.read_text(encoding="utf-8").startswith("# DENEYSEL")
    placed = [s for _, s in range_sensors_from_config(RobotConfig.load(out))]
    ring = ring_sensors((0.0, 90.0, -90.0), 20.0)
    for got, want in zip(placed, ring):                        # eğitimle aynı yerleşim
        assert (got.x, got.y, got.z, got.yaw_deg, got.pitch_deg) == pytest.approx(
            (want.x, want.y, want.z, want.yaw_deg, want.pitch_deg), abs=1e-6)
    assert placed[0].max_m == 1.2                              # eşik robot.yaml'dan
