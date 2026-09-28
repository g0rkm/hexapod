"""Veri sayfasına göre davranan sahte cihazlar (GOREVLER.md S7).

Donanım olmadan sürücüyü gerçekten sınamanın tek yolu: doğru yazmaçlara doğru
cevapları veren, durumu olan bir taklit. hexapod_driver'ın DryRunBackend'i
bunu yapamaz — her okumaya 0 döner, o yüzden "sürücü kimliği doğruladı" gibi
testler kendi kendini onaylar.

Buradaki taklitler:
  - kimlik yazmaçlarını doğru döndürür (VL53L0X 0xEE, BNO055 0xA0)
  - adres değişimini gerçekten uygular (0x8A'ya yazınca yeni adrese taşınır)
  - XSHUT kapalıyken I2C'de CEVAP VERMEZ (gerçek davranış: sensör kapalı)
  - VL53L0X ölçüme başlamadan "veri hazır" demez
  - BNO055 mod değişimini ve kalibrasyon durumunu tutar

Bunlar test altyapısıdır ama pakette durur: robotsuz geliştirmenin yolu bu
(CLAUDE.md "Donanımsız geliştirme"), donanım vardiyasında da bir davranış
tartışılırken "taklit ne diyor" diye bakılabilsin.
"""

from __future__ import annotations

from hexapod_driver.errors import BackendError

from . import bno055 as B
from . import vl53l0x as V


class FakeVl53l0x:
    """Tek bir VL53L0X taklidi.

    distance_mm: sensörün ölçtüğü mesafe. None = menzilde bir şey yok
    (veri sayfası: menzil dışında 8190 gibi büyük değer döner).
    """

    def __init__(self, distance_mm: int | None = 300, powered: bool = True) -> None:
        self.address = V.DEFAULT_ADDRESS
        self.distance_mm = distance_mm
        self.powered = powered
        self.ranging = False
        self.registers: dict[int, int] = {V.VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV: 0x00}
        self.interrupt_cleared = 0

    def read(self, reg: int) -> int:
        if reg == V.IDENTIFICATION_MODEL_ID:
            return V.MODEL_ID
        if reg == V.IDENTIFICATION_REVISION_ID:
            return 0x10
        if reg == V.RESULT_INTERRUPT_STATUS:
            return 0x04 if self.ranging else 0x00
        return self.registers.get(reg, 0)

    def read_block(self, reg: int, length: int) -> list[int]:
        if reg == V.RESULT_RANGE_STATUS:
            out = [0] * length
            mm = 8190 if self.distance_mm is None else self.distance_mm
            if length > 11:
                out[10], out[11] = (mm >> 8) & 0xFF, mm & 0xFF
            return out
        return [self.read(reg + i) for i in range(length)]

    def write(self, reg: int, value: int) -> None:
        if reg == V.I2C_SLAVE_DEVICE_ADDRESS:
            self.address = value & 0x7F
            return
        if reg == V.SYSRANGE_START:
            self.ranging = value in (0x02, 0x04)
            return
        if reg == V.SYSTEM_INTERRUPT_CLEAR:
            self.interrupt_cleared += 1
            return
        self.registers[reg] = value & 0xFF


class FakeBno055:
    """BNO055 taklidi: mod, kalibrasyon ve üç veri bloğu."""

    def __init__(self, address: int = B.ADDRESS_LOW, gravity=(0.0, 0.0, -9.81),
                 gyro_dps=(0.0, 0.0, 0.0), calibration: int = 0xFF) -> None:
        self.address = address
        self.gravity = gravity
        self.gyro_dps = gyro_dps
        self.calibration = calibration
        self.mode = B.MODE_CONFIG
        self.registers: dict[int, int] = {}

    @staticmethod
    def _le16(values, scale) -> list[int]:
        out = []
        for v in values:
            raw = int(round(v * scale))
            raw = raw + 0x10000 if raw < 0 else raw
            out += [raw & 0xFF, (raw >> 8) & 0xFF]
        return out

    def read(self, reg: int) -> int:
        if reg == B.CHIP_ID:
            return B.CHIP_ID_VALUE
        if reg == B.OPR_MODE:
            return self.mode
        if reg == B.CALIB_STAT:
            return self.calibration
        return self.registers.get(reg, 0)

    def read_block(self, reg: int, length: int) -> list[int]:
        if reg == B.GRAVITY_DATA:
            return self._le16(self.gravity, 100.0)[:length]
        if reg == B.GYRO_DATA:
            return self._le16(self.gyro_dps, 16.0)[:length]
        if reg == B.QUATERNION_DATA:
            return self._le16((1.0, 0.0, 0.0, 0.0), 1 << 14)[:length]
        return [self.read(reg + i) for i in range(length)]

    def write(self, reg: int, value: int) -> None:
        if reg == B.OPR_MODE:
            self.mode = value
            return
        self.registers[reg] = value & 0xFF


class FakeI2CBus:
    """Sahte cihazları bir I2C veri yolunda toplayan arka uç.

    hexapod_driver.I2CBackend protokolünü karşılar. Adreste cihaz yoksa
    BackendError verir — gerçek I2C'de de olmayan adres hata verir; sessizce
    0 dönmek, olmayan bir sensörü var gibi gösterirdi.
    """

    def __init__(self, devices: dict[int, object] | None = None) -> None:
        self.devices: dict[int, object] = dict(devices or {})
        self.writes: list[tuple[int, int, list[int]]] = []

    # -- cihaz yönetimi ------------------------------------------------------

    def attach(self, device) -> None:
        self.devices[device.address] = device

    def detach(self, address: int) -> None:
        self.devices.pop(address, None)

    def _device(self, addr: int):
        device = self.devices.get(addr)
        if device is None:
            raise BackendError(f"0x{addr:02x} adresinde cihaz yok (cevap vermedi)")
        return device

    def _sync_address(self, device, previous: int) -> None:
        """Cihaz adres değiştirdiyse veri yolundaki yerini güncelle."""
        if device.address != previous:
            self.devices.pop(previous, None)
            self.devices[device.address] = device

    # -- I2CBackend ----------------------------------------------------------

    def write_byte_data(self, addr: int, reg: int, value: int) -> None:
        device = self._device(addr)
        self.writes.append((addr, reg, [value & 0xFF]))
        device.write(reg, value & 0xFF)
        self._sync_address(device, addr)

    def write_block_data(self, addr: int, reg: int, data: list[int]) -> None:
        device = self._device(addr)
        self.writes.append((addr, reg, [d & 0xFF for d in data]))
        for i, value in enumerate(data):
            device.write(reg + i, value & 0xFF)
        self._sync_address(device, addr)

    def read_byte_data(self, addr: int, reg: int) -> int:
        return self._device(addr).read(reg)

    def read_block_data(self, addr: int, reg: int, length: int) -> list[int]:
        return self._device(addr).read_block(reg, length)

    def close(self) -> None:
        pass


class FakeXshutGpio:
    """XSHUT pinleri: pin düşerse sensör veri yolundan kaybolur, yükselirse
    fabrika adresinde (0x29) geri gelir — gerçek davranış."""

    def __init__(self, bus: FakeI2CBus, pins: dict[int, FakeVl53l0x]) -> None:
        self.bus = bus
        self.pins = pins
        self.values: dict[int, bool] = {}
        self.history: list[tuple[int, bool]] = []

    def setup_output(self, pin: int, value: bool) -> None:
        self.write(pin, value)

    def write(self, pin: int, value: bool) -> None:
        self.values[pin] = bool(value)
        self.history.append((pin, bool(value)))
        sensor = self.pins.get(pin)
        if sensor is None:
            return
        if value:
            sensor.address = V.DEFAULT_ADDRESS   # açılışta fabrika adresinde doğar
            sensor.ranging = False
            self.bus.attach(sensor)
        else:
            self.bus.detach(sensor.address)

    def close(self) -> None:
        pass


__all__ = ["FakeBno055", "FakeI2CBus", "FakeVl53l0x", "FakeXshutGpio"]
