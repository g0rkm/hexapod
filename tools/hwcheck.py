#!/usr/bin/env python3
"""I2C veri yolunu tara ve config/robot.yaml ile karşılaştır.

Ne işe yarar
------------
Robotta hangi I2C cihazının hangi adreste olduğunu bulur ve robot.yaml'da
yazan beklentiyle karşılaştırır. Yapılandırmadaki adres alanları hâlâ boşsa,
yapıştırılabilir bir öneri üretir.

Adres tahminleri HEURISTIK'tir: I2C'de cihazlar kendilerini tanıtmaz, sadece
bir adreste cevap verirler. Araç "bu adreste bir şey var" der; "bu kesinlikle
PCA9685'tir" demez. Belirsiz durumları açıkça belirtir.

Kullanım
--------
    python tools/hwcheck.py                 # varsayılan veri yolu (robot.yaml)
    python tools/hwcheck.py --bus 1
    python tools/hwcheck.py --dry-run       # donanımsız, akışı görmek için
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "hexapod_driver"))

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[union-attr]
    except (AttributeError, OSError, ValueError):
        pass

from hexapod_driver import ConfigError, RobotConfig  # noqa: E402
from hexapod_driver.errors import BackendError  # noqa: E402

# Bilinen cihazların fabrika adresleri. Kesinlik iddiası yok, sadece ipucu.
KNOWN = {
    0x28: ["BNO055 (fabrika, ADR=düşük)"],
    0x29: ["BNO055 (ADR=yüksek)", "VL53L0X (fabrika — üçü de burada doğar)"],
    0x76: ["BMP280 (fabrika)"],
    0x77: ["BMP280 (SDO=yüksek)"],
}
# PCA9685 0x40-0x7F aralığında yapılandırılabilir; fabrika çıkışı 0x40.
PCA_RANGE = range(0x40, 0x80)
# Bütün PCA9685'ler fabrika ayarında bu adreste de cevap verir (ALLCALL).
# Bir kartın kendi adresi DEĞİL; kart adayı olarak önerilmemeli.
PCA_ALLCALL = 0x70


def scan_bus(bus_number: int) -> list[int]:
    """Veri yolundaki adresleri tara. smbus2 gerektirir."""
    try:
        from smbus2 import SMBus, i2c_msg  # type: ignore[import-not-found]
    except ImportError as exc:
        raise BackendError(
            "smbus2 kurulu değil. Raspberry Pi üzerinde: pip install smbus2"
        ) from exc

    found: list[int] = []
    try:
        with SMBus(bus_number) as bus:
            for address in range(0x03, 0x78):
                try:
                    bus.i2c_rdwr(i2c_msg.write(address, []))
                    found.append(address)
                except OSError:
                    continue
    except OSError as exc:
        raise BackendError(
            f"I2C veri yolu {bus_number} açılamadı: {exc}\n"
            "raspi-config -> Interface Options -> I2C etkin mi?"
        ) from exc
    return found


def guesses(address: int) -> list[str]:
    if address == PCA_ALLCALL:
        return ["PCA9685 ortak çağrı adresi (ALLCALL) — kart adresi değil, normal"]
    out = list(KNOWN.get(address, []))
    if address in PCA_RANGE:
        label = "PCA9685" + (" (fabrika çıkışı)" if address == 0x40 else "")
        out.append(label)
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Hexapod I2C donanım kontrolü")
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--bus", type=int, default=None, help="I2C veri yolu numarası")
    parser.add_argument("--dry-run", action="store_true", help="Tarama yapma, akışı göster")
    args = parser.parse_args(argv)

    try:
        config = RobotConfig.load(args.config)
    except ConfigError as exc:
        print(f"HATA: {exc}", file=sys.stderr)
        return 2

    bus_number = args.bus
    if bus_number is None:
        buses = {d.i2c_bus for d in config.drivers.values()}
        bus_number = buses.pop() if len(buses) == 1 else 1

    print(f"\nI2C veri yolu {bus_number} taranıyor...\n")

    if args.dry_run:
        found: list[int] = []
        print("(--dry-run: gerçek tarama yapılmadı)\n")
    else:
        try:
            found = scan_bus(bus_number)
        except BackendError as exc:
            print(f"HATA: {exc}", file=sys.stderr)
            return 2

    # -- bulunanlar -------------------------------------------------------
    if found:
        print(f"Bulunan cihazlar ({len(found)}):")
        for address in found:
            hints = guesses(address)
            hint = "  muhtemelen: " + " veya ".join(hints) if hints else "  (bilinmeyen)"
            print(f"  0x{address:02x}{hint}")
    else:
        print("Hiçbir cihaz cevap vermedi.")
        if not args.dry_run:
            print("  - Kartlar besleniyor mu?")
            print("  - SDA/SCL ve ortak toprak bağlı mı?")
    print()

    # -- robot.yaml ile karşılaştırma -------------------------------------
    expected: dict[int, str] = {}
    for driver in config.drivers.values():
        if driver.address.known:
            expected[int(driver.address.value)] = f"sürücü kartı #{driver.id}"
    imu = ((config.raw.get("sensors") or {}).get("imu") or {}).get("address") or {}
    if isinstance(imu, dict) and imu.get("value") is not None:
        expected[int(imu["value"])] = "IMU (BNO055)"

    if expected:
        print("robot.yaml beklentisiyle karşılaştırma:")
        for address, label in sorted(expected.items()):
            mark = "VAR" if address in found else "YOK"
            print(f"  0x{address:02x}  {label:<22} {mark}")
        extra = [a for a in found if a not in expected]
        if extra:
            print("  Yapılandırmada yazmayan adresler: "
                  + ", ".join(f"0x{a:02x}" for a in extra))
        print()
    else:
        print("robot.yaml'da hiç adres girilmemiş — karşılaştırma yapılamıyor.\n")

    # -- öneri ------------------------------------------------------------
    missing = [d for d in config.drivers.values() if not d.address.known]
    if missing and found:
        candidates = [a for a in found if a in PCA_RANGE and a != PCA_ALLCALL]
        print("ÖNERİ — config/robot.yaml -> drivers:")
        if len(candidates) >= len(missing):
            for driver, address in zip(sorted(missing, key=lambda d: d.id), candidates):
                print(f"  drivers[{driver.id}].address.value: 0x{address:02x}")
            print("\n  Hangi kartın hangi adreste olduğunu doğrulamak için: bir kartın")
            print("  beslemesini kesip tekrar tarayın, kaybolan adres o karta aittir.")
        else:
            print(f"  {len(missing)} kart bekleniyordu, PCA9685 aralığında "
                  f"{len(candidates)} adres bulundu.")
            if len(candidates) == 1 and PCA_ALLCALL in found:
                print("  Büyük ihtimalle iki kart da aynı adreste. Kartlardan birinin")
                print("  A0 pedini lehimle birleştirin; o kart 0x41'e geçer.")
            else:
                print("  Adres jumper'larını ve besleme bağlantılarını kontrol edin.")
        print()

    vl53 = [a for a in found if a == 0x29]
    if vl53:
        print("NOT: 0x29'da cihaz var. Üç VL53L0X de fabrika çıkışında bu adrestedir;")
        print("     aynı anda yalnızca biri görünür. Ayrı XSHUT pinleriyle sırayla")
        print("     uyandırılıp yeniden adreslenmeleri gerekir. BNO055 de ADR=yüksek")
        print("     iken 0x29'dadır — çakışmaya dikkat.\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
