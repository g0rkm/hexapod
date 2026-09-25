"""hexapod_hardware.DriverController testleri (saf Python, DryRunBackend) — GOREVLER.md S4.

"Bitti sayılır": dry-run'da 18 eklem doğru kanala doğru darbeyi yazıyor;
simülasyonla aynı komut arayüzü (S3'ün ürettiği komut sürücüyü de sürüyor).

Düğümün (node.py) kendisi ROS'lu ortamda elle doğrulanır; burada yalnızca
ROS'suz çekirdek test edilir.
"""

from __future__ import annotations

import copy
import math
from pathlib import Path

import pytest
import yaml

from hexapod_driver import Calibration, DryRunBackend, RobotConfig, ServoBus
from hexapod_hardware import DriverController
from hexapod_description.interface import joint_names, to_command
from hexapod_kinematics import HexapodKinematics
from hexapod_teleop import TeleopController

REAL_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"
DT = 1.0 / 50.0
ALL_OFF = [0x00, 0x00, 0x00, 0x10]  # PCA9685'te "kanal tamamen kapalı" deseni
LIMIT_DEG = 90.0


def wired_config(tmp_path: Path) -> RobotConfig:
    """Gerçek robot.yaml + UYDURMA kablolama (yalnızca testte; gerçek değerler
    kablolama yapılınca elle girilecek). 18 eklem iki karta sırayla dağılır."""
    raw = copy.deepcopy(yaml.safe_load(REAL_CONFIG.read_text(encoding="utf-8")))
    raw["drivers"][0]["address"]["value"] = 0x40
    raw["drivers"][1]["address"]["value"] = 0x41
    for index, joint in enumerate(raw["joints"]):
        joint["driver"] = 0 if index < 16 else 1
        joint["channel"] = index % 16
        joint["limits_deg"] = {"min": -LIMIT_DEG, "max": LIMIT_DEG}
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(raw, allow_unicode=True), encoding="utf-8")
    return RobotConfig.load(path)


def joint_calibration(index: int) -> tuple[int, int, float]:
    """UYDURMA kalibrasyon: her eklem için farklı merkez, karışık yön, farklı katsayı."""
    return 1400 + 10 * index, (1 if index % 2 == 0 else -1), 8.0 + index % 3


def make_calibration(config: RobotConfig) -> Calibration:
    cal = Calibration.empty()
    for index, spec in enumerate(config.joints):
        center, direction, per_deg = joint_calibration(index)
        cal.set_center(spec.key, center)
        cal.set_direction(spec.key, direction)
        cal.get(spec.key).us_per_deg = per_deg
    return cal


def make_controller(tmp_path: Path, calibrated: bool = True):
    config = wired_config(tmp_path)
    backend = DryRunBackend()
    cal = make_calibration(config) if calibrated else Calibration.empty()
    controller = DriverController(ServoBus(config, cal, backend=backend))
    controller.start()
    backend.writes.clear()
    return controller, backend


def last_write(backend: DryRunBackend, address: int, register: int) -> list[int]:
    return [data for addr, reg, data in backend.writes if (addr, reg) == (address, register)][-1]


# ---------------------------------------------------------------------------
# 18 eklem, doğru kanala, doğru darbe
# ---------------------------------------------------------------------------


def test_18_eklem_dogru_kanala_dogru_darbeyi_yaziyor(tmp_path):
    controller, backend = make_controller(tmp_path)
    degrees = [((index * 7) % 41) - 20 for index in range(18)]  # her eklem farklı, ±20° içinde
    result = controller.on_command([math.radians(d) for d in degrees])
    assert result.accepted, result.reason

    for index in range(18):
        center, direction, per_deg = joint_calibration(index)
        expected_us = round(center + direction * degrees[index] * per_deg)
        expected_counts = round(expected_us * 4096 / 20000)

        address = 0x40 if index < 16 else 0x41
        register = 0x06 + 4 * (index % 16)
        on_lo, on_hi, off_lo, off_hi = last_write(backend, address, register)
        assert (on_hi << 8 | on_lo) == 0
        assert (off_hi << 8 | off_lo) == expected_counts, (
            f"eklem {index}: kart 0x{address:02x} kanal {index % 16}, "
            f"beklenen {expected_us} us ({expected_counts} adım)")
    controller.stop()


# ---------------------------------------------------------------------------
# Bozuk / reddedilen komut servoya gitmiyor
# ---------------------------------------------------------------------------


def test_yanlis_uzunluktaki_komut_reddediliyor(tmp_path):
    controller, backend = make_controller(tmp_path)
    result = controller.on_command([0.0] * 17)
    assert not result.accepted
    assert "18" in result.reason
    assert backend.writes == []
    controller.stop()


def test_sonlu_olmayan_deger_reddediliyor(tmp_path):
    controller, backend = make_controller(tmp_path)
    data = [0.0] * 18
    data[5] = float("nan")
    assert not controller.on_command(data).accepted
    assert backend.writes == []
    controller.stop()


def test_bir_eklem_limit_disiysa_komutun_hicbir_parcasi_gitmiyor(tmp_path):
    controller, backend = make_controller(tmp_path)
    data = [0.0] * 18
    data[7] = math.radians(LIMIT_DEG + 15.0)  # tek eklem limit dışı
    result = controller.on_command(data)
    assert not result.accepted
    assert "LimitError" in result.reason
    assert backend.writes == [], "17 sağlam eklem de gitmemeli (yarım komut sıçratır)"
    controller.stop()


def test_kalibrasyonsuz_komut_reddediliyor(tmp_path):
    controller, backend = make_controller(tmp_path, calibrated=False)
    result = controller.on_command([0.0] * 18)
    assert not result.accepted
    assert "MissingValue" in result.reason
    assert backend.writes == []
    controller.stop()


# ---------------------------------------------------------------------------
# /joint_states = son gönderilen komut
# ---------------------------------------------------------------------------


def test_ilk_komuttan_once_konum_uydurulmuyor(tmp_path):
    controller, _ = make_controller(tmp_path)
    assert controller.joint_state() is None
    controller.stop()


def test_joint_state_son_kabul_edilen_komut(tmp_path):
    controller, _ = make_controller(tmp_path)
    good = [math.radians(5.0 * (i % 5)) for i in range(18)]
    assert controller.on_command(good).accepted

    names, positions = controller.joint_state()
    assert names == joint_names(range(6))
    assert positions == pytest.approx(good)

    bad = [0.0] * 17
    assert not controller.on_command(bad).accepted
    assert controller.joint_state()[1] == pytest.approx(good), "reddedilen komut durumu bozmamalı"
    controller.stop()


# ---------------------------------------------------------------------------
# Kapanış: servolar serbest
# ---------------------------------------------------------------------------


def test_stop_butun_servolari_serbest_birakiyor(tmp_path):
    controller, backend = make_controller(tmp_path)
    controller.on_command([0.0] * 18)
    backend.writes.clear()
    controller.stop()
    for address in (0x40, 0x41):
        assert last_write(backend, address, 0xFA) == ALL_OFF  # ALL_LED: tümü kapalı


# ---------------------------------------------------------------------------
# Simülasyonla aynı komut arayüzü
# ---------------------------------------------------------------------------


def test_tripod_komutu_surucuyu_de_suruyor(tmp_path):
    """S3'ün Gazebo'ya yayınladığı komutun aynısı gerçek sürücüde kabul ediliyor."""
    controller, backend = make_controller(tmp_path)
    kin = HexapodKinematics.from_config(RobotConfig.load(REAL_CONFIG))
    teleop = TeleopController(kin)
    teleop.on_command(0.08, 0.03, 0.2, now=0.0)

    for tick in range(1, 200):  # ~4 sn, birkaç adım döngüsü
        command = teleop.tick(now=tick * DT, dt=DT)
        result = controller.on_command(command)
        assert result.accepted, f"tick {tick}: {result.reason}"
    assert len(backend.writes) >= 199 * 18
    controller.stop()
