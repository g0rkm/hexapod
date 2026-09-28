"""BNO055 IMU sürücüsü (GOREVLER.md S7).

Saf Python, ROS'suz; I2C erişimi hexapod_driver'ın arka uçlarından.

Ne için: politika ve refleks gövdenin yönelimini kullanıyor. Eğitimde gözlem
YERÇEKİMİ YÖNÜ (gövde çerçevesinde birim vektör) ve açısal hız
(hexapod_rl.task.observation); robotta da aynısı verilmeli, yoksa politika
simde öğrendiğini robotta bulamaz.

BNO055 füzyonu kendi içinde yapıyor: NDOF modunda ivmeölçer + jiroskop +
manyetometre birleşip yerçekimi vektörünü ayrı bir yazmaçtan veriyor. Bu,
ham ivmeden yerçekimini ayıklamaya göre çok daha sağlam (yürürken gövde
ivmeleniyor).

Yazmaçlar (Bosch BNO055 veri sayfası, BST-BNO055-DS000)
-------------------------------------------------------
  0x00  CHIP_ID            0xA0 olmalı
  0x3D  OPR_MODE           0x00 CONFIG, 0x0C NDOF (füzyon)
  0x3E  PWR_MODE           0x00 normal
  0x3F  SYS_TRIGGER        bit5: harici kristal (kartta var)
  0x35  CALIB_STAT         sys/gyro/acc/mag, her biri 2 bit; 3 = kalibre
  0x14  GYRO_DATA_X_LSB    6 bayt, LSB önce; ölçek 16 LSB/(°/s)
  0x2E  GRAVITY_DATA_X_LSB 6 bayt, LSB önce; ölçek 100 LSB/(m/s²)
  0x20  QUATERNION_DATA_W_LSB  8 bayt; ölçek 2^14

Adres: ADR pini düşükse 0x28, yüksekse 0x29. **0x29 VL53L0X'in fabrika
adresiyle çakışır** (rangefinders.check_addresses bunu reddeder).

Montaj yönelimi
---------------
IMU gövdeye nasıl takıldığı bilinmiyor (robot.yaml
`sensors.imu.mount_rotation_deg: null`, D8). Sürücü bunu UYDURMAZ; verilmezse
ham eksenler döner ve `rotate_to_base` çağrılamaz. Politika düğümü gövde
çerçevesinde veri bekliyor (docs/ARAYUZ.md madde 3).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from hexapod_driver.backends import I2CBackend
from hexapod_driver.errors import BackendError

# -- yazmaçlar (BST-BNO055-DS000) --------------------------------------------
CHIP_ID = 0x00
OPR_MODE = 0x3D
PWR_MODE = 0x3E
SYS_TRIGGER = 0x3F
CALIB_STAT = 0x35
GYRO_DATA = 0x14
GRAVITY_DATA = 0x2E
QUATERNION_DATA = 0x20

#: CHIP_ID bu değeri vermeli.
CHIP_ID_VALUE = 0xA0

#: Çalışma modları.
MODE_CONFIG = 0x00
MODE_NDOF = 0x0C

#: Adresler: ADR pini düşük / yüksek. 0x29 VL53L0X ile çakışır.
ADDRESS_LOW = 0x28
ADDRESS_HIGH = 0x29

#: Ölçekler (veri sayfası).
_GYRO_LSB_PER_DPS = 16.0
_GRAVITY_LSB_PER_MS2 = 100.0
_QUAT_LSB = 1 << 14

#: Mod değişimi sonrası bekleme (veri sayfası: CONFIG->çalışma 7 ms,
#: çalışma->CONFIG 19 ms). Bol pay.
MODE_SWITCH_S = 0.03

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class Calibration:
    """BNO055'in kendi kalibrasyon durumu; her biri 0-3 (3 = tam)."""

    system: int
    gyroscope: int
    accelerometer: int
    magnetometer: int

    @property
    def ready(self) -> bool:
        """Yerçekimi yönü için jiroskop ve ivmeölçer yeterli.

        Manyetometre (pusula) yürüyüş için gerekmiyor: politika yön açısını
        değil yerçekimi yönünü ve açısal hızı kullanıyor. Robotun yanındaki
        servolar ve akım kabloları manyetometreyi zaten bozar.
        """
        return self.gyroscope >= 3 and self.accelerometer >= 3


def _signed16(low: int, high: int) -> int:
    value = (high << 8) | low
    return value - 0x10000 if value & 0x8000 else value


class Bno055:
    """BNO055, NDOF (füzyon) modunda.

        imu = Bno055(backend, address=0x28)
        imu.begin()
        g = imu.gravity()            # m/s^2, sensör çerçevesinde
        w = imu.angular_velocity()   # rad/s
    """

    def __init__(self, backend: I2CBackend, address: int = ADDRESS_LOW,
                 sleep=time.sleep) -> None:
        if address not in (ADDRESS_LOW, ADDRESS_HIGH):
            raise BackendError(
                f"BNO055 adresi 0x{ADDRESS_LOW:02x} ya da 0x{ADDRESS_HIGH:02x} olabilir, "
                f"0x{address:02x} verildi")
        self.backend = backend
        self.address = address
        self._sleep = sleep

    # -- kurulum -------------------------------------------------------------

    def identify(self) -> int:
        return self.backend.read_byte_data(self.address, CHIP_ID)

    def check(self) -> None:
        chip = self.identify()
        if chip != CHIP_ID_VALUE:
            raise BackendError(
                f"0x{self.address:02x} adresinde BNO055 yok: kimlik 0x{chip:02x}, "
                f"beklenen 0x{CHIP_ID_VALUE:02x}. "
                + ("0x29 VL53L0X'in de fabrika adresi; çakışıyor olabilir."
                   if self.address == ADDRESS_HIGH else "Kart beslenmiyor olabilir.")
            )

    def begin(self, external_crystal: bool = True) -> None:
        """Kimliği doğrula, NDOF moduna al.

        external_crystal: Gravity 10DOF kartında harici kristal var; veri
        sayfası onu kullanmayı öneriyor (daha kararlı zamanlama).
        """
        self.check()
        self.set_mode(MODE_CONFIG)
        self.backend.write_byte_data(self.address, PWR_MODE, 0x00)   # normal güç
        if external_crystal:
            self.backend.write_byte_data(self.address, SYS_TRIGGER, 0x80)
            self._sleep(MODE_SWITCH_S)
        self.set_mode(MODE_NDOF)

    def set_mode(self, mode: int) -> None:
        self.backend.write_byte_data(self.address, OPR_MODE, mode)
        self._sleep(MODE_SWITCH_S)

    def calibration(self) -> Calibration:
        raw = self.backend.read_byte_data(self.address, CALIB_STAT)
        return Calibration(system=(raw >> 6) & 0x03, gyroscope=(raw >> 4) & 0x03,
                           accelerometer=(raw >> 2) & 0x03, magnetometer=raw & 0x03)

    # -- okuma ---------------------------------------------------------------

    def gravity(self) -> Vec3:
        """Yerçekimi vektörü (m/s²), SENSÖR çerçevesinde.

        Füzyondan gelir, ham ivme değil: robot yürürken gövde ivmelense de
        bu vektör yerçekimini gösterir.
        """
        raw = self.backend.read_block_data(self.address, GRAVITY_DATA, 6)
        return tuple(_signed16(raw[i], raw[i + 1]) / _GRAVITY_LSB_PER_MS2
                     for i in (0, 2, 4))  # type: ignore[return-value]

    def gravity_direction(self) -> Vec3:
        """Yerçekimi YÖNÜ (birim vektör), sensör çerçevesinde.

        Politikanın gözlemindeki büyüklük bu (hexapod_rl.task.observation:
        state.gravity_in_base). Vektör sıfırsa (sensör daha veri vermediyse)
        hata verir; sıfır vektör "yukarısı neresi bilinmiyor" demektir ve
        sessizce geçerse politika yanlış poz görür.
        """
        x, y, z = self.gravity()
        norm = math.sqrt(x * x + y * y + z * z)
        if norm < 1e-6:
            raise BackendError("BNO055 yerçekimi vektörü sıfır: füzyon henüz hazır değil")
        return (x / norm, y / norm, z / norm)

    def angular_velocity(self) -> Vec3:
        """Açısal hız (rad/s), sensör çerçevesinde."""
        raw = self.backend.read_block_data(self.address, GYRO_DATA, 6)
        return tuple(math.radians(_signed16(raw[i], raw[i + 1]) / _GYRO_LSB_PER_DPS)
                     for i in (0, 2, 4))  # type: ignore[return-value]

    def quaternion(self) -> tuple[float, float, float, float]:
        """Yönelim (w, x, y, z), birim quaternion."""
        raw = self.backend.read_block_data(self.address, QUATERNION_DATA, 8)
        w, x, y, z = (_signed16(raw[i], raw[i + 1]) / _QUAT_LSB for i in (0, 2, 4, 6))
        return (w, x, y, z)


__all__ = ["ADDRESS_HIGH", "ADDRESS_LOW", "Bno055", "CHIP_ID_VALUE", "Calibration",
           "MODE_CONFIG", "MODE_NDOF"]
