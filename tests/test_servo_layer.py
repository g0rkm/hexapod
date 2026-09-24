"""Servo sürücü katmanı testleri.

Vurgu iki noktada:
  1. Eksik yapılandırma sessizce varsayılana düşmüyor, hata veriyor.
  2. Açı -> darbe dönüşümü ve güvenlik kırpması doğru.
"""

from __future__ import annotations

import copy
from pathlib import Path

import pytest
import yaml

from hexapod_driver import (
    Calibration,
    DryRunBackend,
    LimitError,
    MissingValue,
    PCA9685,
    RobotConfig,
    ServoBus,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
REAL_CONFIG = REPO_ROOT / "config" / "robot.yaml"


# ---------------------------------------------------------------------------
# Yardımcılar
# ---------------------------------------------------------------------------


def wired_config(tmp_path: Path) -> RobotConfig:
    """Gerçek robot.yaml'ı alıp kablolama alanlarını doldurulmuş hâle getir.

    Bu değerler UYDURMA'dır ve yalnızca testte kullanılır; gerçek kablolama
    yapıldığında config/robot.yaml elle doldurulacak.
    """
    raw = yaml.safe_load(REAL_CONFIG.read_text(encoding="utf-8"))
    raw = copy.deepcopy(raw)
    raw["drivers"][0]["address"]["value"] = 0x40
    raw["drivers"][1]["address"]["value"] = 0x41
    for index, joint in enumerate(raw["joints"]):
        joint["driver"] = 0 if index < 16 else 1
        joint["channel"] = index % 16
        joint["limits_deg"] = {"min": -60.0, "max": 60.0}
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(raw, allow_unicode=True), encoding="utf-8")
    return RobotConfig.load(path)


def make_bus(config: RobotConfig, calibration: Calibration | None = None):
    backend = DryRunBackend()
    bus = ServoBus(config, calibration or Calibration.empty(), backend=backend)
    bus.start()
    return bus, backend


# ---------------------------------------------------------------------------
# robot.yaml'ın kendisi
# ---------------------------------------------------------------------------


def test_gercek_config_yuklenebiliyor():
    config = RobotConfig.load(REAL_CONFIG)
    assert len(config.joints) == 18
    assert len(config.legs) == 6
    assert len(config.drivers) == 2


def test_cad_degerleri_olculmemis_olarak_isaretli():
    config = RobotConfig.load(REAL_CONFIG)
    coxa = config.segments["coxa"]
    assert coxa.value == 50.0
    assert coxa.measured is False, "CAD değeri ölçülmüş gibi işaretlenmemeli"
    assert "CAD" in coxa.source


def test_bilinmeyen_deger_okunmaya_calisilinca_hata_veriyor():
    config = RobotConfig.load(REAL_CONFIG)
    height = config.standing_height
    assert height.known is False
    with pytest.raises(MissingValue) as exc:
        height.require()
    assert "body.standing_height" in str(exc.value)


def test_segment_uzunluklari_cadden_dolu_ama_olculmemis():
    config = RobotConfig.load(REAL_CONFIG)
    for name, expected in (("coxa", 50.0), ("femur", 80.0), ("tibia", 126.6)):
        segment = config.segments[name]
        assert segment.require() == expected
        assert segment.measured is False, f"{name} ölçülmüş gibi işaretlenmemeli"


def test_kablolama_eksik_oldugu_bildiriliyor():
    config = RobotConfig.load(REAL_CONFIG)
    assert config.wiring_is_complete() is False
    gaps = {gap.path for gap in config.wiring_gaps()}
    assert "drivers[0].address" in gaps
    assert "joints[leg0_coxa].channel" in gaps
    # 2 kart adresi + 18 eklem x (driver, channel)
    assert len(gaps) == 2 + 36


def test_bacak_yerlesimi_altigen():
    config = RobotConfig.load(REAL_CONFIG)
    azimuths = sorted(leg.azimuth_deg for leg in config.legs)
    assert azimuths == [-120.0, -60.0, 0.0, 60.0, 120.0, 180.0]
    assert sum(1 for leg in config.legs if leg.mirrored) == 3


# ---------------------------------------------------------------------------
# PCA9685
# ---------------------------------------------------------------------------


def test_prescale_50hz_icin_dogru():
    backend = DryRunBackend()
    board = PCA9685(backend, 0x40, 50.0)
    board.begin()
    # prescale = round(25e6 / (4096 * 50)) - 1 = 122 - 1 = 121
    prescale_writes = [w for w in backend.writes if w[1] == 0xFE]
    assert prescale_writes, "PRESCALE yazmacına hiç yazılmadı"
    assert prescale_writes[-1][2] == [121]


def test_darbe_sayaca_dogru_cevriliyor():
    backend = DryRunBackend()
    board = PCA9685(backend, 0x40, 50.0)
    board.begin()
    backend.writes.clear()
    board.set_pulse_us(3, 1500)
    # 50 Hz -> 20000 us periyot. 1500/20000 * 4096 = 307.2 -> 307
    addr, reg, data = backend.writes[-1]
    assert reg == 0x06 + 4 * 3
    off = data[2] | (data[3] << 8)
    assert off == 307


def test_gecersiz_kanal_reddediliyor():
    board = PCA9685(DryRunBackend(), 0x40, 50.0)
    with pytest.raises(Exception):
        board.set_pulse_us(16, 1500)


# ---------------------------------------------------------------------------
# ServoBus
# ---------------------------------------------------------------------------


def test_ham_darbe_gonderilebiliyor(tmp_path):
    config = wired_config(tmp_path)
    bus, backend = make_bus(config)
    sent = bus.set_pulse_us(0, "coxa", 1500)
    assert sent == 1500
    assert bus.state(0, "coxa").pulse_us == 1500
    bus.stop()


def test_guvenlik_sinirlari_disi_reddediliyor(tmp_path):
    config = wired_config(tmp_path)
    bus, _ = make_bus(config)
    with pytest.raises(LimitError):
        bus.set_pulse_us(0, "coxa", 3000)
    with pytest.raises(LimitError):
        bus.set_pulse_us(0, "coxa", 100)
    bus.stop()


def test_kalibrasyon_yoksa_aci_komutu_hata_veriyor(tmp_path):
    config = wired_config(tmp_path)
    bus, _ = make_bus(config)
    with pytest.raises(MissingValue) as exc:
        bus.set_angle(0, "coxa", 10.0)
    assert "center_us" in str(exc.value)
    bus.stop()


def test_aci_darbeye_dogru_cevriliyor(tmp_path):
    config = wired_config(tmp_path)
    cal = Calibration.empty()
    cal.set_center("leg0_coxa", 1500)
    cal.set_direction("leg0_coxa", 1)
    cal.get("leg0_coxa").us_per_deg = 10.0

    bus, _ = make_bus(config, cal)
    assert bus.set_pulse_us(0, "coxa", 1500) == 1500
    assert bus.set_angle(0, "coxa", 30.0) == 1800
    assert bus.set_angle(0, "coxa", -30.0) == 1200
    bus.stop()


def test_ters_yon_isareti_uygulaniyor(tmp_path):
    config = wired_config(tmp_path)
    cal = Calibration.empty()
    cal.set_center("leg2_femur", 1500)
    cal.set_direction("leg2_femur", -1)
    cal.get("leg2_femur").us_per_deg = 10.0

    bus, _ = make_bus(config, cal)
    assert bus.set_angle(2, "femur", 30.0) == 1200
    bus.stop()


def test_eklem_limiti_disi_aci_reddediliyor(tmp_path):
    config = wired_config(tmp_path)
    cal = Calibration.empty()
    cal.set_center("leg0_coxa", 1500)
    cal.set_direction("leg0_coxa", 1)
    cal.get("leg0_coxa").us_per_deg = 10.0

    bus, _ = make_bus(config, cal)
    with pytest.raises(LimitError) as exc:
        bus.set_angle(0, "coxa", 75.0)
    assert "limit" in str(exc.value).lower()
    bus.stop()


def test_release_servoyu_serbest_birakiyor(tmp_path):
    config = wired_config(tmp_path)
    bus, _ = make_bus(config)
    bus.set_pulse_us(0, "coxa", 1500)
    assert bus.active_joints() == ["leg0_coxa"]
    bus.release(0, "coxa")
    assert bus.active_joints() == []
    bus.stop()


def test_release_all_hepsini_kapatiyor(tmp_path):
    config = wired_config(tmp_path)
    bus, _ = make_bus(config)
    bus.set_pulse_us(0, "coxa", 1500)
    bus.set_pulse_us(1, "femur", 1600)
    bus.release_all()
    assert bus.active_joints() == []
    bus.stop()


def test_kart_adresi_yoksa_start_hata_veriyor():
    config = RobotConfig.load(REAL_CONFIG)
    bus = ServoBus(config, Calibration.empty(), backend=DryRunBackend())
    with pytest.raises(MissingValue) as exc:
        bus.start()
    assert "address" in str(exc.value)


# ---------------------------------------------------------------------------
# calibration.yaml gidiş-dönüş
# ---------------------------------------------------------------------------


def test_kalibrasyon_yazilip_geri_okunabiliyor(tmp_path):
    path = tmp_path / "calibration.yaml"
    cal = Calibration.empty()
    cal.set_center("leg0_coxa", 1487)
    cal.set_direction("leg0_coxa", -1)
    cal.save(path)

    again = Calibration.load(path)
    entry = again.joints["leg0_coxa"]
    assert entry.center_us == 1487
    assert entry.direction == -1
    assert entry.us_per_deg is None
    assert entry.calibrated_at is not None


def test_kalibrasyon_dosyasi_yoksa_bos_baslar(tmp_path):
    cal = Calibration.load(tmp_path / "yok.yaml")
    assert cal.joints == {}
    assert cal.summary() == (0, 0)


def test_gecersiz_yon_reddediliyor():
    cal = Calibration.empty()
    with pytest.raises(ValueError):
        cal.set_direction("leg0_coxa", 0)


def test_iki_kart_arasinda_dogru_yonlendirme(tmp_path):
    """18 servo 2 karta bolunuyor: dogru adres ve dogru kanala gitmeli."""
    config = wired_config(tmp_path)
    bus, backend = make_bus(config)

    # leg5_femur -> kart 1 (0x41), kanal 0   (17. eklem, indeks 16)
    backend.writes.clear()
    bus.set_pulse_us(5, "femur", 1500)
    addr, reg, _ = backend.writes[-1]
    assert addr == 0x41, "ikinci karta gitmeliydi"
    assert reg == 0x06, "kanal 0 olmaliydi"

    # leg0_coxa -> kart 0 (0x40), kanal 0
    backend.writes.clear()
    bus.set_pulse_us(0, "coxa", 1500)
    addr, reg, _ = backend.writes[-1]
    assert addr == 0x40
    assert reg == 0x06

    # leg5_coxa -> kart 0 (0x40), kanal 15  (16. eklem, indeks 15)
    backend.writes.clear()
    bus.set_pulse_us(5, "coxa", 1500)
    addr, reg, _ = backend.writes[-1]
    assert addr == 0x40
    assert reg == 0x06 + 4 * 15
    bus.stop()


def test_her_kart_kendi_frekansiyla_baslatiliyor(tmp_path):
    config = wired_config(tmp_path)
    bus, backend = make_bus(config)
    for address in (0x40, 0x41):
        prescale = [w for w in backend.writes if w[0] == address and w[1] == 0xFE]
        assert prescale, f"0x{address:02x} kartinda PRESCALE ayarlanmadi"
        assert prescale[-1][2] == [121]
    bus.stop()


# ---------------------------------------------------------------------------
# Limit ve span kalibrasyonu
# ---------------------------------------------------------------------------


def test_span_us_per_deg_hesapliyor():
    cal = Calibration.empty()
    cal.set_center("leg0_coxa", 1500)
    cal.set_direction("leg0_coxa", 1)
    assert cal.set_span("leg0_coxa", 1950, 45.0) == pytest.approx(10.0)


def test_span_ters_yonde_de_pozitif_katsayi_veriyor():
    cal = Calibration.empty()
    cal.set_center("leg0_coxa", 1500)
    cal.set_direction("leg0_coxa", -1)
    assert cal.set_span("leg0_coxa", 1050, 45.0) == pytest.approx(10.0)


def test_span_merkez_yoksa_hata_veriyor():
    cal = Calibration.empty()
    with pytest.raises(MissingValue):
        cal.set_span("leg0_coxa", 1950, 45.0)


def test_span_sifira_yakin_aci_reddediliyor():
    cal = Calibration.empty()
    cal.set_center("leg0_coxa", 1500)
    cal.set_direction("leg0_coxa", 1)
    with pytest.raises(ValueError):
        cal.set_span("leg0_coxa", 1505, 0.2)


def test_limitler_dereceye_cevriliyor():
    cal = Calibration.empty()
    cal.set_center("leg0_coxa", 1500)
    cal.set_direction("leg0_coxa", 1)
    cal.set_span("leg0_coxa", 1950, 45.0)
    cal.set_limit("leg0_coxa", "min", 900)
    cal.set_limit("leg0_coxa", "max", 2100)
    lo, hi = cal.get("leg0_coxa").limits_deg()
    assert lo == pytest.approx(-60.0)
    assert hi == pytest.approx(60.0)


def test_eksik_bilgiyle_limit_derecesi_uydurulmuyor():
    cal = Calibration.empty()
    cal.set_center("leg0_coxa", 1500)
    cal.set_limit("leg0_coxa", "min", 900)
    # yön ve us_per_deg yok -> cevrilemez
    assert cal.get("leg0_coxa").limits_deg() is None


def test_limit_alanlari_kaydedilip_geri_okunuyor(tmp_path):
    path = tmp_path / "calibration.yaml"
    cal = Calibration.empty()
    cal.set_center("leg1_tibia", 1520)
    cal.set_direction("leg1_tibia", -1)
    cal.set_span("leg1_tibia", 1070, 45.0)
    cal.set_limit("leg1_tibia", "min", 1000)
    cal.set_limit("leg1_tibia", "max", 2000)
    cal.save(path)

    again = Calibration.load(path).joints["leg1_tibia"]
    assert again.limit_min_us == 1000
    assert again.limit_max_us == 2000
    assert again.us_per_deg == pytest.approx(10.0)
