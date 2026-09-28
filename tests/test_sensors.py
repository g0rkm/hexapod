"""hexapod_sensors testleri (donanımsız, veri sayfası davranışına göre) — S7.

"Bitti sayılır": veri sayfalarına göre yazmaç düzeyinde testler robotsuz
geçiyor. Bunun için sahte cihazlar (hexapod_sensors.fake) kimlik yazmacını,
adres değişimini, XSHUT davranışını ve ölçüm durum makinesini gerçek çipler
gibi taklit ediyor; DryRunBackend yetmezdi (her okumaya 0 döner, testler
kendi kendini onaylardı).
"""

from __future__ import annotations

import math

import pytest

from hexapod_driver.errors import BackendError, MissingValue
from hexapod_sensors import bno055 as B
from hexapod_sensors import mount, rangefinders, vl53l0x
from hexapod_sensors.fake import FakeBno055, FakeI2CBus, FakeVl53l0x, FakeXshutGpio


def nosleep(_seconds: float) -> None:
    """Testler gerçek zaman beklemesin."""


# ---------------------------------------------------------------------------
# VL53L0X: kimlik, adres, ölçüm
# ---------------------------------------------------------------------------


def test_kimlik_dogrulaniyor():
    bus = FakeI2CBus({0x29: FakeVl53l0x()})
    sensor = vl53l0x.Vl53l0x(bus, 0x29)
    assert sensor.identify() == vl53l0x.MODEL_ID
    sensor.check()  # patlamamalı


def test_baska_cip_varsa_kimlik_reddediliyor():
    """0x29'da BNO055 olabilir (ADR yüksek). Sürücü bunu fark etmeli.

    BNO055'te VL53L0X'in kimlik yazmacı (0xC0) yok; okunan değer 0xEE
    olmadığı sürece sürücü durmalı. Mesaj hem bulunan değeri hem olası
    sebebi söylüyor ki donanımda "sensör bozuk" diye aranmasın.
    """
    bus = FakeI2CBus({0x29: FakeBno055(address=0x29)})
    sensor = vl53l0x.Vl53l0x(bus, 0x29)
    with pytest.raises(BackendError) as exc:
        sensor.check()
    mesaj = str(exc.value)
    assert "VL53L0X yok" in mesaj
    assert "0xee" in mesaj.lower(), "beklenen kimlik söylenmeli"
    assert "BNO055" in mesaj, "olası adres çakışması söylenmeli"


def test_adres_degistirme_yazmaca_yaziliyor_ve_sensor_tasiniyor():
    device = FakeVl53l0x()
    bus = FakeI2CBus({0x29: device})
    sensor = vl53l0x.Vl53l0x(bus, 0x29)
    sensor.set_address(0x30)

    assert (0x29, vl53l0x.I2C_SLAVE_DEVICE_ADDRESS, [0x30]) in bus.writes
    assert sensor.address == 0x30
    assert device.address == 0x30
    assert sensor.identify() == vl53l0x.MODEL_ID, "yeni adreste cevap vermeli"
    with pytest.raises(BackendError):
        bus.read_byte_data(0x29, vl53l0x.IDENTIFICATION_MODEL_ID)  # eski adres boş


def test_gecersiz_adres_reddediliyor():
    bus = FakeI2CBus({0x29: FakeVl53l0x()})
    sensor = vl53l0x.Vl53l0x(bus, 0x29)
    for bad in (0x00, 0x80):
        with pytest.raises(BackendError):
            sensor.set_address(bad)


def test_mesafe_metre_olarak_okunuyor():
    bus = FakeI2CBus({0x29: FakeVl53l0x(distance_mm=347)})
    sensor = vl53l0x.Vl53l0x(bus, 0x29)
    sensor.start_continuous()
    assert sensor.read_distance() == pytest.approx(0.347)


def test_gormuyorsa_none_donuyor():
    """Menzil dışında sensör 8190 gibi bir değer döner; bu mesafe DEĞİL.

    Refleks "görmüyor"u ayırt edebilmeli (Görkem'in notu): 0.0 ile
    karışırsa engel yokken varmış gibi tetiklenir.
    """
    bus = FakeI2CBus({0x29: FakeVl53l0x(distance_mm=None)})
    sensor = vl53l0x.Vl53l0x(bus, 0x29)
    sensor.start_continuous()
    assert sensor.read_distance() is None


def test_sensorun_gormuyor_isareti_mesafe_sayilmiyor():
    """8190 mm bir MESAFE değil, "menzilde bir şey yok" işareti (veri sayfası).

    Menzil geniş tutulursa (burada 10 m) bu değer geçerli bir ölçüm gibi
    görünürdü; refleks 8 m ötede engel var sanardı.
    """
    bus = FakeI2CBus({0x29: FakeVl53l0x(distance_mm=None)})
    sensor = vl53l0x.Vl53l0x(bus, 0x29, max_range_m=10.0)
    assert sensor.read_distance() is None


def test_menzil_disindaki_olcum_none():
    bus = FakeI2CBus({0x29: FakeVl53l0x(distance_mm=2000)})
    sensor = vl53l0x.Vl53l0x(bus, 0x29, max_range_m=1.2)
    assert sensor.read_distance() is None, "1.2 m menzilde 2 m ölçüm geçersiz"


def test_olcum_sonrasi_kesme_temizleniyor():
    """Temizlenmezse sensör yeni ölçüm vermez (veri sayfası)."""
    device = FakeVl53l0x()
    sensor = vl53l0x.Vl53l0x(FakeI2CBus({0x29: device}), 0x29)
    sensor.read_distance()
    assert device.interrupt_cleared == 1


def test_olcum_baslamadan_veri_hazir_degil():
    device = FakeVl53l0x()
    sensor = vl53l0x.Vl53l0x(FakeI2CBus({0x29: device}), 0x29)
    assert sensor.data_ready() is False
    sensor.start_continuous()
    assert sensor.data_ready() is True


def test_measure_zaman_asiminda_none():
    device = FakeVl53l0x()
    sensor = vl53l0x.Vl53l0x(FakeI2CBus({0x29: device}), 0x29)
    assert sensor.measure(timeout_s=0.0, sleep=nosleep) is None, "ölçüm başlamadı"


def test_begin_2v8_modunu_aciyor():
    device = FakeVl53l0x()
    sensor = vl53l0x.Vl53l0x(FakeI2CBus({0x29: device}), 0x29)
    sensor.begin()
    assert device.registers[vl53l0x.VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV] & 0x01


def test_ince_ayar_dizisi_veri_olarak_yaziliyor():
    """Tuning sürücüye gömülü değil; D8'de donanımda düzeltilebilsin diye."""
    device = FakeVl53l0x()
    sensor = vl53l0x.Vl53l0x(FakeI2CBus({0x29: device}), 0x29,
                             tuning=((0x44, 0x11), (0x45, 0x22)))
    sensor.begin()
    assert device.registers[0x44] == 0x11 and device.registers[0x45] == 0x22


# ---------------------------------------------------------------------------
# Üç sensörün XSHUT ile ayrılması — S7'nin asıl işi
# ---------------------------------------------------------------------------


def uc_sensor():
    devices = {0: FakeVl53l0x(distance_mm=100), 1: FakeVl53l0x(distance_mm=200),
               2: FakeVl53l0x(distance_mm=None)}
    bus = FakeI2CBus()
    gpio = FakeXshutGpio(bus, {5: devices[0], 6: devices[1], 7: devices[2]})
    specs = [rangefinders.RangeFinderSpec(0, 5, 0x30, 0.0),
             rangefinders.RangeFinderSpec(1, 6, 0x31, 90.0),
             rangefinders.RangeFinderSpec(2, 7, 0x32, -90.0)]
    return specs, bus, gpio, devices


def test_uc_sensor_ayri_adreslere_yerlesiyor():
    specs, bus, gpio, devices = uc_sensor()
    finders = rangefinders.RangeFinders(specs, bus, gpio, sleep=nosleep)
    finders.begin()

    assert [d.address for d in devices.values()] == [0x30, 0x31, 0x32]
    assert sorted(bus.devices) == [0x30, 0x31, 0x32]
    assert 0x29 not in bus.devices, "fabrika adresi boşalmalı"


def test_adresleme_once_hepsini_kapatiyor_sonra_teker_teker_aciyor():
    """Sıra yanlışsa iki sensör aynı anda 0x29'da olur ve adresleme bozulur."""
    specs, bus, gpio, _ = uc_sensor()
    rangefinders.RangeFinders(specs, bus, gpio, sleep=nosleep).begin()

    ilk_uc = gpio.history[:3]
    assert ilk_uc == [(5, False), (6, False), (7, False)], "önce hepsi kapalı"
    acilma_sirasi = [pin for pin, value in gpio.history[3:] if value]
    assert acilma_sirasi == [5, 6, 7], "teker teker ve sırayla açılmalı"


def test_her_sensor_okunuyor_gormeyen_menzil_donuyor():
    specs, bus, gpio, _ = uc_sensor()
    finders = rangefinders.RangeFinders(specs, bus, gpio, max_range_m=1.2, sleep=nosleep)
    finders.begin()
    assert finders.read() == pytest.approx([0.1, 0.2, 1.2])


def test_uyanmayan_sensor_hangisi_oldugu_soyleniyor():
    specs, bus, gpio, devices = uc_sensor()
    gpio.pins.pop(6)   # 1 numaralı sensörün XSHUT'ı bağlı değil: hiç uyanmıyor
    finders = rangefinders.RangeFinders(specs, bus, gpio, sleep=nosleep)
    with pytest.raises(BackendError) as exc:
        finders.begin()
    assert "sensör 1" in str(exc.value) and "GPIO 6" in str(exc.value)


def test_kapatinca_olcum_duruyor_ve_xshut_dusuyor():
    specs, bus, gpio, devices = uc_sensor()
    finders = rangefinders.RangeFinders(specs, bus, gpio, sleep=nosleep)
    finders.begin()
    finders.close()
    assert all(not v for v in (gpio.values[5], gpio.values[6], gpio.values[7]))
    assert all(not d.ranging for d in devices.values())


# ---------------------------------------------------------------------------
# Adres çakışmaları: donanımda "sensör bozuk" gibi görünen hatalar
# ---------------------------------------------------------------------------


def test_imu_ile_ayni_adres_reddediliyor():
    specs = [rangefinders.RangeFinderSpec(0, 5, 0x29 - 1, 0.0)]
    with pytest.raises(BackendError) as exc:
        rangefinders.check_addresses(specs, imu_address=0x28)
    assert "IMU" in str(exc.value)


def test_iki_sensore_ayni_adres_reddediliyor():
    specs = [rangefinders.RangeFinderSpec(0, 5, 0x30, 0.0),
             rangefinders.RangeFinderSpec(1, 6, 0x30, 0.0)]
    with pytest.raises(BackendError):
        rangefinders.check_addresses(specs)


def test_fabrika_adresi_hedef_olarak_reddediliyor():
    """0x30 yerine 0x29 verilirse sıradaki sensör orada doğar ve çakışır."""
    specs = [rangefinders.RangeFinderSpec(0, 5, 0x29, 0.0)]
    with pytest.raises(BackendError) as exc:
        rangefinders.check_addresses(specs)
    assert "fabrika" in str(exc.value)


def test_cakisma_denetimi_kurulusta_calisiyor():
    """Denetim yalnız ayrı bir fonksiyonda değil, RangeFinders kurulurken de
    çalışmalı; yoksa çakışan yapılandırma donanıma kadar gider."""
    specs = [rangefinders.RangeFinderSpec(0, 5, 0x30, 0.0),
             rangefinders.RangeFinderSpec(1, 6, 0x30, 0.0)]
    with pytest.raises(BackendError):
        rangefinders.RangeFinders(specs, FakeI2CBus(), FakeXshutGpio(FakeI2CBus(), {}),
                                  sleep=nosleep)


def test_iki_sensor_ayni_pinde_reddediliyor():
    specs = [rangefinders.RangeFinderSpec(0, 5, 0x30, 0.0),
             rangefinders.RangeFinderSpec(1, 5, 0x31, 0.0)]
    with pytest.raises(BackendError):
        rangefinders.check_addresses(specs)


# ---------------------------------------------------------------------------
# robot.yaml: eksik değer uydurulmuyor
# ---------------------------------------------------------------------------


def test_kablolama_girilmemisse_hata_ve_hangi_alan_soyleniyor():
    from hexapod_driver import RobotConfig

    config = RobotConfig.load()   # gerçek robot.yaml: alanlar null (D8)
    with pytest.raises(MissingValue) as exc:
        rangefinders.specs_from_config(config)
    assert "sensors.range_finders.devices[0]" in str(exc.value)


def test_imu_montaji_girilmemisse_hata():
    from hexapod_driver import RobotConfig

    with pytest.raises(MissingValue):
        mount.mount_from_config(RobotConfig.load())


# ---------------------------------------------------------------------------
# BNO055
# ---------------------------------------------------------------------------


def test_imu_kimligi_dogrulaniyor():
    bus = FakeI2CBus({0x28: FakeBno055()})
    imu = B.Bno055(bus, 0x28, sleep=nosleep)
    assert imu.identify() == B.CHIP_ID_VALUE
    imu.check()


def test_imu_yerine_mesafe_sensoru_varsa_cakisma_soyleniyor():
    """0x29'da VL53L0X olabilir; hata mesajı bunu söylemeli."""
    bus = FakeI2CBus({0x29: FakeVl53l0x()})
    imu = B.Bno055(bus, 0x29, sleep=nosleep)
    with pytest.raises(BackendError) as exc:
        imu.check()
    assert "VL53L0X" in str(exc.value)


def test_begin_ndof_moduna_aliyor():
    device = FakeBno055()
    imu = B.Bno055(FakeI2CBus({0x28: device}), 0x28, sleep=nosleep)
    imu.begin()
    assert device.mode == B.MODE_NDOF


def test_gecersiz_imu_adresi_reddediliyor():
    with pytest.raises(BackendError):
        B.Bno055(FakeI2CBus(), 0x40, sleep=nosleep)


def test_yercekimi_yonu_birim_vektor():
    bus = FakeI2CBus({0x28: FakeBno055(gravity=(0.0, 0.0, -9.81))})
    imu = B.Bno055(bus, 0x28, sleep=nosleep)
    assert imu.gravity() == pytest.approx((0.0, 0.0, -9.81), abs=0.01)
    assert imu.gravity_direction() == pytest.approx((0.0, 0.0, -1.0), abs=1e-3)


def test_egik_govdede_yercekimi_yonu_dogru():
    """30° yana yatmış robot: yerçekimi yönü de o kadar yatık görünmeli."""
    a = math.radians(30.0)
    bus = FakeI2CBus({0x28: FakeBno055(gravity=(0.0, 9.81 * math.sin(a), -9.81 * math.cos(a)))})
    imu = B.Bno055(bus, 0x28, sleep=nosleep)
    x, y, z = imu.gravity_direction()
    assert math.degrees(math.atan2(y, -z)) == pytest.approx(30.0, abs=0.5)


def test_fuzyon_hazir_degilse_yercekimi_yonu_hata_veriyor():
    """Sıfır vektör "yukarısı bilinmiyor" demek; sessizce geçerse politika
    yanlış poz görür."""
    bus = FakeI2CBus({0x28: FakeBno055(gravity=(0.0, 0.0, 0.0))})
    with pytest.raises(BackendError):
        B.Bno055(bus, 0x28, sleep=nosleep).gravity_direction()


def test_acisal_hiz_radyan_bolu_saniye():
    bus = FakeI2CBus({0x28: FakeBno055(gyro_dps=(0.0, 0.0, 90.0))})
    imu = B.Bno055(bus, 0x28, sleep=nosleep)
    assert imu.angular_velocity()[2] == pytest.approx(math.pi / 2, abs=1e-3)


def test_negatif_deger_isaretli_cozuluyor():
    bus = FakeI2CBus({0x28: FakeBno055(gyro_dps=(-45.0, 0.0, 0.0))})
    imu = B.Bno055(bus, 0x28, sleep=nosleep)
    assert imu.angular_velocity()[0] == pytest.approx(math.radians(-45.0), abs=1e-3)


def test_kalibrasyon_durumu_cozuluyor():
    bus = FakeI2CBus({0x28: FakeBno055(calibration=0b11_11_11_00)})
    cal = B.Bno055(bus, 0x28, sleep=nosleep).calibration()
    assert (cal.system, cal.gyroscope, cal.accelerometer, cal.magnetometer) == (3, 3, 3, 0)
    assert cal.ready is True, "pusula yürüyüş için gerekmiyor"


def test_jiroskop_kalibresizse_hazir_degil():
    cal = B.Calibration(system=3, gyroscope=1, accelerometer=3, magnetometer=3)
    assert cal.ready is False


# ---------------------------------------------------------------------------
# Montaj dönüşü
# ---------------------------------------------------------------------------


def test_donussuz_montajda_vektor_degismiyor():
    assert mount.rotate_to_base(mount.IDENTITY, (1.0, 2.0, 3.0)) == pytest.approx((1, 2, 3))


def test_90_derece_donuk_takilmis_imu():
    """IMU 90° döndürülmüş takılıysa sensörün +x'i gövdenin +y'si olur."""
    matrix = mount.parse_mount(90.0)
    assert mount.rotate_to_base(matrix, (1.0, 0.0, 0.0)) == pytest.approx((0, 1, 0), abs=1e-9)


def test_ters_takilmis_imu_yercekimini_dogru_ceviriyor():
    """Kart ters (roll 180°): sensörün gördüğü -z, gövdede +z olmalı."""
    matrix = mount.parse_mount((180.0, 0.0, 0.0))
    assert mount.rotate_to_base(matrix, (0.0, 0.0, -1.0)) == pytest.approx((0, 0, 1), abs=1e-9)


def test_montaj_bicimleri():
    assert mount.parse_mount(0.0) == mount.IDENTITY
    assert mount.parse_mount([0.0, 0.0, 0.0]) == mount.IDENTITY
    with pytest.raises(Exception):
        mount.parse_mount("çok")


# ---------------------------------------------------------------------------
# Quaternion da gövde çerçevesine taşınmalı (politika yönelimi BUNDAN okuyor)
# ---------------------------------------------------------------------------


def gravity_in_base(quat):
    """Politikanın yaptığı hesap: dünya <- gövde quaternion'undan gövdedeki
    yerçekimi yönü (hexapod_rl.state.gravity_in_base ile aynı)."""
    m = mount.quaternion_to_matrix(quat)
    return tuple(sum(m[j][i] * v for j, v in enumerate((0.0, 0.0, -1.0))) for i in range(3))


@pytest.mark.parametrize("montaj", [(90.0, 0.0, 0.0), (0.0, 45.0, 0.0),
                                    (180.0, 0.0, 0.0), (20.0, -30.0, 60.0)])
def test_yan_takilmis_imuda_govde_duz_gorunuyor(montaj):
    """IMU yan/ters takılıyken gövde DÜZ ise politika düz görmeli.

    Bu testin montajında roll/pitch var: yalnız yaw'lı bir montaj yerçekimi
    yönünü hiç değiştirmediği için hatayı gizler (ilk yazdığım test bu yüzden
    montaj dönüşü quaternion'a hiç uygulanmasa da geçiyordu).
    """
    matrix = mount.parse_mount(montaj)
    # Gövde düz: gövde->dünya birim. Sensör gövdeye göre M kadar dönük
    # olduğundan sensör->dünya = M; IMU bunu bildirir.
    sensor_quat = mount.matrix_to_quaternion(matrix)

    govde_quat = mount.quaternion_to_base(matrix, sensor_quat)
    assert gravity_in_base(govde_quat) == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)


def test_quaternion_ve_vektor_ayni_egikligi_veriyor():
    """Quaternion yolu ile vektör yolu aynı eğikliği vermeli.

    Politika eğikliği quaternion'dan okuyor (controller.on_imu), sürücü
    yerçekimi vektörünü ayrıca döndürüyor; ikisi ayrışırsa robot eğikliğini
    yanlış okur ve hata "politika kötü" gibi görünür.
    """
    matrix = mount.parse_mount((90.0, 0.0, 0.0))   # IMU yan yatık takılı
    yatis = math.radians(15.0)
    # Gövde +15° roll döndüyse (R_b), gövde çerçevesindeki yerçekimi
    # R_bᵀ·(0,0,-1) = (0, -sin, -cos); işareti ters yazmak testi kendi içinde
    # tutarsız yapar (ilk yazışta öyle oldu).
    govde_yercekimi = (0.0, -math.sin(yatis), -math.cos(yatis))
    # Sensörün gördüğü yerçekimi ve sensörün yönelimi (gövde bu kadar yatıkken)
    sensor_gravity = tuple(sum(matrix[j][i] * govde_yercekimi[j] for j in range(3))
                           for i in range(3))
    govde_donmesi = mount.rotation(15.0, 0.0, 0.0)
    sensor_quat = mount.matrix_to_quaternion(
        tuple(tuple(sum(govde_donmesi[i][k] * matrix[k][j] for k in range(3))
                    for j in range(3)) for i in range(3)))

    quat_yolu = gravity_in_base(mount.quaternion_to_base(matrix, sensor_quat))
    vektor_yolu = mount.rotate_to_base(matrix, sensor_gravity)
    assert quat_yolu == pytest.approx(vektor_yolu, abs=1e-9)
    assert quat_yolu == pytest.approx(govde_yercekimi, abs=1e-9)


@pytest.mark.parametrize("montaj", [0.0, 90.0, -90.0, (0.0, 0.0, 180.0),
                                    (180.0, 0.0, 0.0), (0.0, 30.0, 45.0)])
def test_egik_govde_her_montajda_dogru_okunuyor(montaj):
    """Gövde 20° yana yatıkken, IMU nasıl takılırsa takılsın politikanın
    gördüğü eğiklik 20° olmalı."""
    matrix = mount.parse_mount(montaj)
    yatis = math.radians(20.0)
    # Gövde çerçevesinde beklenen yerçekimi yönü
    beklenen = (0.0, math.sin(yatis), -math.cos(yatis))
    # Sensörün gördüğü: gövdedekinin ters dönüşümü (v_sensör = Mᵀ v_gövde)
    sensor_gravity = tuple(sum(matrix[j][i] * beklenen[j] for j in range(3))
                           for i in range(3))
    assert mount.rotate_to_base(matrix, sensor_gravity) == pytest.approx(beklenen, abs=1e-9)


def test_quaternion_matris_gidis_donus():
    for montaj in (0.0, 37.0, (10.0, -20.0, 30.0)):
        m = mount.parse_mount(montaj)
        q = mount.matrix_to_quaternion(m)
        geri = mount.quaternion_to_matrix(q)
        for satir_beklenen, satir_geri in zip(m, geri):
            assert satir_geri == pytest.approx(satir_beklenen, abs=1e-9)


def test_donussuz_montajda_quaternion_degismiyor():
    q = (0.9238795, 0.0, 0.0, 0.3826834)   # 45° yaw
    assert mount.quaternion_to_base(mount.IDENTITY, q) == pytest.approx(q, abs=1e-6)
