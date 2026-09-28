"""Üç VL53L0X'i XSHUT ile ayırıp adresleme (GOREVLER.md S7).

Sorun: üç sensör de fabrika çıkışında 0x29'da doğar. I2C'de aynı adreste iki
cihaz varsa ikisi birden cevap verir; okunan değer çöptür ve bu, "sensör bozuk"
gibi görünen ama aslında adresleme olan bir hatadır.

Çözüm (tek yol): her sensörün XSHUT pini ayrı bir GPIO'ya bağlanır.
  1. Hepsini XSHUT=0 ile kapat (I2C'de kimse yok).
  2. Birini XSHUT=1 yap, açılmasını bekle, 0x29'da kimliğini doğrula.
  3. Ona yeni adresini ver; artık 0x29'u boşalttı.
  4. Sıradakine geç.
Adresler GÜÇ KESİLENE KADAR kalıcıdır: her açılışta bu dizi yeniden koşar.

Yerleşim ve pinler robot.yaml'dan gelir (D8'in kararı); şu an null oldukları
için bu modül MissingValue fırlatır ve neyin eksik olduğunu söyler. Varsayılan
KOYMAZ (CLAUDE.md temel kuralı): yanlış bir pin numarası, kodun başka bir
donanımı sürmesi demek.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from hexapod_driver.backends import GpioBackend, I2CBackend
from hexapod_driver.config import RobotConfig, Value
from hexapod_driver.errors import BackendError, MissingValue

from .vl53l0x import DEFAULT_ADDRESS, Vl53l0x

#: XSHUT yükseldikten sonra sensörün I2C'ye cevap vermeye hazır olması
#: (veri sayfası: açılış ~1.2 ms). Bol pay bırakıldı; açılışta bir kez olur.
BOOT_S = 0.01


@dataclass(frozen=True)
class RangeFinderSpec:
    """Bir mesafe sensörünün kablolaması ve yerleşimi (robot.yaml'dan)."""

    id: int
    xshut_gpio: int
    address: int
    direction_deg: float


def specs_from_config(config: RobotConfig) -> list[RangeFinderSpec]:
    """robot.yaml -> sensör tanımları. Eksik alan varsa MissingValue.

    direction_deg (sensör nereye bakıyor) D8'in kararı; kod uydurmaz.
    """
    raw = ((config.raw.get("sensors") or {}).get("range_finders") or {})
    devices = raw.get("devices") or []
    if not devices:
        raise MissingValue("sensors.range_finders.devices",
                           "robot.yaml'da mesafe sensörü tanımlı değil")
    out = []
    for entry in devices:
        i = int(entry.get("id", len(out)))
        base = f"sensors.range_finders.devices[{i}]"
        values = {
            key: Value.parse(entry.get(key), f"{base}.{key}")
            for key in ("xshut_gpio", "address", "direction_deg")
        }
        for key, value in values.items():
            if not value.known:
                raise MissingValue(
                    value.path,
                    value.source or {
                        "xshut_gpio": "KABLOLAMA - XSHUT hangi GPIO'ya bağlı (D8)",
                        "address": "KARAR - sensöre verilecek yeni I2C adresi (D8)",
                        "direction_deg": "MONTAJ - sensör nereye bakıyor (D8)",
                    }[key])
        out.append(RangeFinderSpec(
            id=i,
            xshut_gpio=int(values["xshut_gpio"].value),
            address=int(values["address"].value),
            direction_deg=float(values["direction_deg"].value),
        ))
    return out


def check_addresses(specs: list[RangeFinderSpec], imu_address: int | None = None) -> None:
    """Adres/pin çakışmalarını BAŞLAMADAN önce yakala.

    Çakışma donanımda "sensör bozuk" gibi görünür: iki cihaz aynı adreste
    cevap verir, okunan değer anlamsız olur. Ucuz bir kontrolle önlenir.
    """
    seen: dict[int, int] = {}
    pins: dict[int, int] = {}
    for spec in specs:
        if spec.address == DEFAULT_ADDRESS:
            raise BackendError(
                f"sensör {spec.id}: hedef adres 0x{DEFAULT_ADDRESS:02x} olamaz "
                "(fabrika adresi; sıradaki sensör orada doğacak)")
        if spec.address in seen:
            raise BackendError(
                f"sensör {spec.id} ile {seen[spec.address]} aynı adreste "
                f"(0x{spec.address:02x}); her sensöre ayrı adres gerekir")
        if imu_address is not None and spec.address == imu_address:
            raise BackendError(
                f"sensör {spec.id} adresi IMU ile çakışıyor (0x{spec.address:02x}). "
                "BNO055 0x28/0x29'da olabilir; sensörlere başka adres verin")
        if spec.xshut_gpio in pins:
            raise BackendError(
                f"sensör {spec.id} ile {pins[spec.xshut_gpio]} aynı XSHUT pininde "
                f"(GPIO {spec.xshut_gpio}); her sensör ayrı pine bağlanmalı")
        seen[spec.address] = spec.id
        pins[spec.xshut_gpio] = spec.id


class RangeFinders:
    """Üç VL53L0X: adresleme, başlatma ve topluca okuma.

        finders = RangeFinders(specs, i2c, gpio)
        finders.begin()                       # XSHUT dizisi + adresleme
        mesafeler = finders.read()            # metre listesi; görmeyen için max
    """

    def __init__(self, specs: list[RangeFinderSpec], backend: I2CBackend,
                 gpio: GpioBackend, imu_address: int | None = None,
                 max_range_m: float = 1.2, sleep=time.sleep) -> None:
        check_addresses(specs, imu_address)
        self.specs = list(specs)
        self.backend = backend
        self.gpio = gpio
        self.max_range_m = max_range_m
        self._sleep = sleep
        self.sensors: list[Vl53l0x] = []

    def begin(self) -> None:
        """Hepsini kapat, teker teker uyandır, adres ver, ölçüme başlat."""
        for spec in self.specs:                       # 1. hepsi kapalı
            self.gpio.setup_output(spec.xshut_gpio, False)
        self._sleep(BOOT_S)

        self.sensors = []
        for spec in self.specs:
            self.gpio.write(spec.xshut_gpio, True)    # 2. yalnız bunu uyandır
            self._sleep(BOOT_S)
            sensor = Vl53l0x(self.backend, DEFAULT_ADDRESS, self.max_range_m)
            try:
                sensor.check()                        # 3. gerçekten VL53L0X mi
            except BackendError as exc:
                raise BackendError(
                    f"sensör {spec.id} (XSHUT GPIO {spec.xshut_gpio}) uyandırılamadı: {exc}"
                ) from exc
            sensor.set_address(spec.address)          # 4. yeni adres
            sensor.check()                            # 5. yeni adreste de cevap veriyor mu
            sensor.begin()
            sensor.start_continuous()
            self.sensors.append(sensor)

    def read(self) -> list[float]:
        """Bütün sensörlerden mesafe (m). Görmeyen sensör için max_range_m.

        Refleksin sözleşmesi bu: "bu mesafede bir şey yoksa ölçüm max_m"
        (hexapod_policy.lift_reflex.RangeSensor). Okunamayan ölçüm de
        "görmüyor" sayılır; refleks zaten bayat veriyi kendi zaman aşımıyla
        eliyor (controller.range_timeout_s).
        """
        out = []
        for sensor in self.sensors:
            distance = sensor.read_distance()
            out.append(self.max_range_m if distance is None else distance)
        return out

    def close(self) -> None:
        """Ölçümü durdur, sensörleri kapat (XSHUT=0), GPIO'yu bırak."""
        for sensor in self.sensors:
            try:
                sensor.stop_continuous()
            except BackendError:
                pass
        for spec in self.specs:
            try:
                self.gpio.write(spec.xshut_gpio, False)
            except BackendError:
                pass
        self.gpio.close()


__all__ = ["BOOT_S", "RangeFinderSpec", "RangeFinders", "check_addresses", "specs_from_config"]
