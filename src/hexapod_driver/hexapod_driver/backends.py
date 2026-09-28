"""I2C veri yolu arka uçları.

İki arka uç var:
  SMBusBackend  -> gerçek donanım (Raspberry Pi üzerinde smbus2)
  DryRunBackend -> donanım yokken geliştirme/test. Hiçbir şey yazmaz,
                   yazma isteklerini kaydeder.

Böylece sürücü katmanı robot olmadan da test edilebilir.
"""

from __future__ import annotations

from typing import Protocol

from .errors import BackendError


class I2CBackend(Protocol):
    def write_byte_data(self, addr: int, reg: int, value: int) -> None: ...

    def write_block_data(self, addr: int, reg: int, data: list[int]) -> None: ...

    def read_byte_data(self, addr: int, reg: int) -> int: ...

    def read_block_data(self, addr: int, reg: int, length: int) -> list[int]: ...

    def close(self) -> None: ...


class SMBusBackend:
    """Gerçek I2C. smbus2 gerektirir (pip install smbus2)."""

    def __init__(self, bus: int = 1):
        try:
            from smbus2 import SMBus  # type: ignore[import-not-found]
        except ImportError as exc:  # pragma: no cover - donanıma özel
            raise BackendError(
                "smbus2 kurulu değil. Raspberry Pi üzerinde: pip install smbus2\n"
                "Donanımsız geliştirme için DryRunBackend kullanın."
            ) from exc
        try:
            self._bus = SMBus(bus)
        except OSError as exc:  # pragma: no cover - donanıma özel
            raise BackendError(
                f"I2C veri yolu {bus} açılamadı: {exc}. "
                "raspi-config'den I2C etkin mi?"
            ) from exc
        self.bus_number = bus

    def write_byte_data(self, addr: int, reg: int, value: int) -> None:
        try:
            self._bus.write_byte_data(addr, reg, value & 0xFF)
        except OSError as exc:  # pragma: no cover - donanıma özel
            raise BackendError(f"I2C yazma hatası (0x{addr:02x} reg 0x{reg:02x}): {exc}") from exc

    def write_block_data(self, addr: int, reg: int, data: list[int]) -> None:
        try:
            self._bus.write_i2c_block_data(addr, reg, [d & 0xFF for d in data])
        except OSError as exc:  # pragma: no cover - donanıma özel
            raise BackendError(f"I2C blok yazma hatası (0x{addr:02x}): {exc}") from exc

    def read_byte_data(self, addr: int, reg: int) -> int:
        try:
            return int(self._bus.read_byte_data(addr, reg))
        except OSError as exc:  # pragma: no cover - donanıma özel
            raise BackendError(f"I2C okuma hatası (0x{addr:02x} reg 0x{reg:02x}): {exc}") from exc

    def read_block_data(self, addr: int, reg: int, length: int) -> list[int]:
        """reg'den başlayarak length bayt oku (tek işlemde).

        Bayt bayt okumak YETMEZ: çok baytlı bir ölçüm (VL53L0X mesafesi 16 bit,
        BNO055 quaternion'u 8 bayt) okumalar arasında güncellenirse yarısı eski
        yarısı yeni bir değer çıkar ("yırtılma"). Bu yüzden tek I2C işlemi.
        """
        if not 1 <= length <= 32:
            raise BackendError(f"blok okuma uzunluğu 1-32 olmalı, {length} verildi")
        try:
            return [int(v) for v in self._bus.read_i2c_block_data(addr, reg, length)]
        except OSError as exc:  # pragma: no cover - donanıma özel
            raise BackendError(
                f"I2C blok okuma hatası (0x{addr:02x} reg 0x{reg:02x}): {exc}") from exc

    def close(self) -> None:
        try:
            self._bus.close()
        except Exception:  # pragma: no cover
            pass


class DryRunBackend:
    """Donanımsız arka uç. Yazmaları kaydeder, okumalarda 0 döndürür."""

    def __init__(self, verbose: bool = False):
        self.writes: list[tuple[int, int, list[int]]] = []
        self.registers: dict[tuple[int, int], int] = {}
        self.verbose = verbose

    def write_byte_data(self, addr: int, reg: int, value: int) -> None:
        self.registers[(addr, reg)] = value & 0xFF
        self.writes.append((addr, reg, [value & 0xFF]))
        if self.verbose:
            print(f"[dry-run] 0x{addr:02x} reg 0x{reg:02x} <- 0x{value & 0xFF:02x}")

    def write_block_data(self, addr: int, reg: int, data: list[int]) -> None:
        self.writes.append((addr, reg, [d & 0xFF for d in data]))
        if self.verbose:
            payload = " ".join(f"{d & 0xFF:02x}" for d in data)
            print(f"[dry-run] 0x{addr:02x} reg 0x{reg:02x} <- {payload}")

    def read_byte_data(self, addr: int, reg: int) -> int:
        return self.registers.get((addr, reg), 0)

    def read_block_data(self, addr: int, reg: int, length: int) -> list[int]:
        return [self.registers.get((addr, reg + i), 0) for i in range(length)]

    def close(self) -> None:
        pass

