"""VL53L0X kızılötesi mesafe sensörü sürücüsü (GOREVLER.md S7).

Saf Python, ROS'a bağımlı değil; I2C erişimi hexapod_driver'ın arka uçlarından
(DryRunBackend ile robotsuz test edilir). hexapod_driver'daki desenin aynısı.

Ne için: kör politikanın ayak kaldırmayı zemine göre seçebilmesi
(hexapod_policy.lift_reflex). Refleksin istediği: her sensörden mesafe (m),
görmüyorsa menzil değeri, en az ~25 Hz.

Yazmaçlar ve dizi
-----------------
Kaynak: ST VL53L0X veri sayfası (DS33030) ve ST'nin API'sindeki yazmaç adları
(Api/core/inc/vl53l0x_device.h). Aşağıdakilerin hepsi belgelenmiş yazmaçlardır;
ST'nin uzun "ince ayar" (tuning) dizisi BURADA YOK — bkz. TUNING notu.

  0x00  SYSRANGE_START            1: tek atış, 2: sürekli (0x04: zamanlanmış)
  0x0B  SYSTEM_INTERRUPT_CLEAR    ölçüm alındıktan sonra 1 yazılır
  0x14  RESULT_RANGE_STATUS       bit0: veri hazır; +10/+11: mesafe (mm, big endian)
  0x13  RESULT_INTERRUPT_STATUS   bit0-2: kesme durumu
  0x8A  I2C_SLAVE_DEVICE_ADDRESS  yeni 7 bit adres (adresleme burada değişir)
  0xC0  IDENTIFICATION_MODEL_ID   0xEE olmalı (kimlik doğrulaması)
  0xC2  IDENTIFICATION_REVISION_ID
  0x89  VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV   bit0: 2V8 modu

TUNING (dikkat)
---------------
ST'nin resmi başlatma dizisi ("static init") ~80 sihirli yazmaç yazımı içerir
ve ölçüm doğruluğunu/menzilini etkiler. Bu sürücü onu İÇERMEZ: donanım
olmadan doğrulanamayacağı için ezberden yazılmış bir sihirli sayı listesi
koymak, "çalışıyor" görünüp sessizce yanlış ölçen bir sürücü demek olurdu.
Sensör fabrika varsayılanlarıyla da ölçer (varsayılan zamanlama bütçesi
~33 ms, menzil ~1.2 m); refleksin ihtiyacı olan budur.

`tuning` parametresi bu yüzden VERİ olarak dışarıdan verilir: D8'de gerçek
donanımda gerekirse ST'nin dizisi eklenir, sürücünün mantığı değişmez.
Donanımda ölçüm menzili/doğruluğu yetmezse bakılacak ilk yer burası.
"""

from __future__ import annotations

import time
from typing import Sequence

from hexapod_driver.backends import I2CBackend
from hexapod_driver.errors import BackendError

# -- yazmaçlar (ST DS33030) --------------------------------------------------
SYSRANGE_START = 0x00
SYSTEM_INTERRUPT_CLEAR = 0x0B
RESULT_INTERRUPT_STATUS = 0x13
RESULT_RANGE_STATUS = 0x14
I2C_SLAVE_DEVICE_ADDRESS = 0x8A
VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV = 0x89
IDENTIFICATION_MODEL_ID = 0xC0
IDENTIFICATION_REVISION_ID = 0xC2

#: Fabrika çıkışı 7 bit adres. ÜÇ SENSÖR DE burada doğar (bkz. rangefinders).
DEFAULT_ADDRESS = 0x29

#: IDENTIFICATION_MODEL_ID bu değeri vermeli; vermezse oradaki çip VL53L0X değil.
MODEL_ID = 0xEE

#: RESULT_RANGE_STATUS + bu ofsette mesafe (mm, big endian, 2 bayt).
_RANGE_OFFSET = 10

#: Sensörün "hiçbir şey görmüyorum" dediği ölçüm. Veri sayfası: menzil dışında
#: 8190/8191 mm gibi değerler döner; bunlar mesafe değil, "yok" demektir.
_OUT_OF_RANGE_MM = 8000


class Vl53l0x:
    """Tek bir VL53L0X.

    Kullanım (tek sensör; üçü birden için rangefinders.RangeFinders):
        sensor = Vl53l0x(backend, address=0x29)
        sensor.begin()              # kimlik doğrular, 2V8 modunu açar
        sensor.start_continuous()
        m = sensor.read_distance()  # metre, görmüyorsa None
    """

    def __init__(self, backend: I2CBackend, address: int = DEFAULT_ADDRESS,
                 max_range_m: float = 1.2, tuning: Sequence[tuple[int, int]] = ()) -> None:
        if not 0x08 <= address <= 0x77:
            raise BackendError(f"Geçersiz I2C adresi: 0x{address:02x}")
        self.backend = backend
        self.address = address
        self.max_range_m = max_range_m
        self.tuning = tuple(tuning)
        self._continuous = False

    # -- kurulum -------------------------------------------------------------

    def identify(self) -> int:
        """Kimlik yazmacını oku. VL53L0X ise MODEL_ID döner.

        Adresleme yanlış giderse (iki sensör aynı adreste, ya da orada IMU var)
        bunu erken yakalamanın tek yolu bu; adres taraması "bir şey cevap
        verdi" der, "bu VL53L0X" demez.
        """
        return self.backend.read_byte_data(self.address, IDENTIFICATION_MODEL_ID)

    def check(self) -> None:
        """Kimlik doğru değilse hata ver."""
        model = self.identify()
        if model != MODEL_ID:
            raise BackendError(
                f"0x{self.address:02x} adresinde VL53L0X yok: kimlik 0x{model:02x}, "
                f"beklenen 0x{MODEL_ID:02x}. Adres çakışması olabilir "
                f"(BNO055 de 0x29'da olabilir) ya da sensör beslenmiyor."
            )

    def begin(self) -> None:
        """Kimliği doğrula, 2V8 modunu aç, varsa ince ayar dizisini yaz."""
        self.check()
        # 2V8 modu: kart 2.8 V I/O ile çalışıyor (Pi'nin 3.3 V'una uyumlu
        # seviyeleyicili kartlarda da üreticinin önerdiği ayar).
        current = self.backend.read_byte_data(self.address, VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV)
        self.backend.write_byte_data(self.address, VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
                                     current | 0x01)
        for reg, value in self.tuning:
            self.backend.write_byte_data(self.address, reg, value)

    def set_address(self, new_address: int) -> None:
        """Sensöre yeni 7 bit adres ver (yalnız o an uyanık olana).

        Adres güç kesilene kadar kalıcıdır; her açılışta yeniden verilmelidir.
        """
        if not 0x08 <= new_address <= 0x77:
            raise BackendError(f"Geçersiz I2C adresi: 0x{new_address:02x}")
        if new_address == self.address:
            return
        self.backend.write_byte_data(self.address, I2C_SLAVE_DEVICE_ADDRESS,
                                     new_address & 0x7F)
        self.address = new_address

    # -- ölçüm ---------------------------------------------------------------

    def start_continuous(self) -> None:
        """Sürekli ölçüm. Refleks ~25 Hz istiyor; tek atış her ölçümde
        başlatma gecikmesi ekler, sürekli modda sensör kendi temposunda ölçer."""
        self.backend.write_byte_data(self.address, SYSRANGE_START, 0x02)
        self._continuous = True

    def stop_continuous(self) -> None:
        self.backend.write_byte_data(self.address, SYSRANGE_START, 0x01)
        self._continuous = False

    def data_ready(self) -> bool:
        """Yeni ölçüm hazır mı (RESULT_INTERRUPT_STATUS'un alt 3 biti)."""
        return bool(self.backend.read_byte_data(self.address, RESULT_INTERRUPT_STATUS) & 0x07)

    def read_distance(self) -> float | None:
        """Son ölçüm, METRE. Sensör bir şey görmüyorsa None.

        None ile 0.0 karışmamalı: refleks "görmüyor"u ayırt edebilmeli
        (Görkem'in notu). Menzil dışı okuma (8190 mm gibi) da None sayılır.
        """
        raw = self.backend.read_block_data(self.address, RESULT_RANGE_STATUS,
                                           _RANGE_OFFSET + 2)
        millimetres = (raw[_RANGE_OFFSET] << 8) | raw[_RANGE_OFFSET + 1]
        self.backend.write_byte_data(self.address, SYSTEM_INTERRUPT_CLEAR, 0x01)
        if millimetres == 0 or millimetres >= _OUT_OF_RANGE_MM:
            return None
        metres = millimetres / 1000.0
        return None if metres > self.max_range_m else metres

    def measure(self, timeout_s: float = 0.1, sleep=time.sleep) -> float | None:
        """Hazır olana kadar bekleyip oku. Süre dolarsa None ("görmüyor" değil,
        "okunamadı" — çağıran ikisini ayırmak isterse data_ready'yi kendi
        yoklar). sleep dışarıdan verilebilir (testte gerçek zaman beklenmesin).
        """
        deadline = time.monotonic() + timeout_s
        while not self.data_ready():
            if time.monotonic() >= deadline:
                return None
            sleep(0.001)
        return self.read_distance()


__all__ = ["DEFAULT_ADDRESS", "MODEL_ID", "Vl53l0x"]
