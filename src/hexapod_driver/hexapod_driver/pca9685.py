"""PCA9685 16 kanallı I2C PWM sürücüsü.

Veri sayfası: NXP PCA9685, 25 MHz dahili osilatör, 12 bit (4096 adım) PWM.
Bu robotta 2 kart var (18 servo, kart başına 16 kanal).
"""

from __future__ import annotations

import time

from .backends import I2CBackend
from .errors import BackendError

# Yazmaç adresleri
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06
ALL_LED_ON_L = 0xFA

# MODE1 bitleri
MODE1_RESTART = 0x80
MODE1_AI = 0x20      # otomatik artan adresleme
MODE1_SLEEP = 0x10
MODE1_ALLCALL = 0x01

# MODE2 bitleri
MODE2_OUTDRV = 0x04  # totem-pole çıkış

OSC_HZ = 25_000_000
STEPS = 4096
CHANNELS = 16


class PCA9685:
    """Tek bir PCA9685 kartı."""

    def __init__(self, backend: I2CBackend, address: int, frequency_hz: float = 50.0):
        if not 0x03 <= address <= 0x77:
            raise BackendError(f"Geçersiz I2C adresi: 0x{address:02x}")
        self.backend = backend
        self.address = address
        self._frequency_hz = float(frequency_hz)
        self._initialised = False

    # -- kurulum ----------------------------------------------------------

    def begin(self) -> None:
        """Kartı uyandır, çıkışları kapat, frekansı ayarla."""
        self.backend.write_byte_data(self.address, MODE2, MODE2_OUTDRV)
        self.backend.write_byte_data(self.address, MODE1, MODE1_ALLCALL)
        time.sleep(0.005)
        mode1 = self.backend.read_byte_data(self.address, MODE1) & ~MODE1_SLEEP
        self.backend.write_byte_data(self.address, MODE1, mode1)
        time.sleep(0.005)
        self.all_off()
        self.set_frequency(self._frequency_hz)
        self._initialised = True

    def set_frequency(self, hz: float) -> None:
        """PWM taşıyıcı frekansı. Analog servolar için 50 Hz.

        Frekans değişimi kartı uyku moduna almayı gerektirir; bu sırada
        çıkışlar durur, bu yüzden önce hepsini kapatıyoruz.
        """
        if not 24.0 <= hz <= 1526.0:
            raise BackendError(f"PCA9685 frekansı aralık dışı: {hz} Hz")
        prescale = int(round(OSC_HZ / (STEPS * float(hz)))) - 1
        prescale = max(3, min(255, prescale))

        self.all_off()
        old = self.backend.read_byte_data(self.address, MODE1)
        self.backend.write_byte_data(self.address, MODE1, (old & 0x7F) | MODE1_SLEEP)
        self.backend.write_byte_data(self.address, PRESCALE, prescale)
        self.backend.write_byte_data(self.address, MODE1, old)
        time.sleep(0.005)
        self.backend.write_byte_data(self.address, MODE1, old | MODE1_RESTART | MODE1_AI)
        self._frequency_hz = float(hz)

    @property
    def frequency_hz(self) -> float:
        return self._frequency_hz

    # -- çıkış ------------------------------------------------------------

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        _check_channel(channel)
        on &= 0x0FFF
        off &= 0x0FFF
        self.backend.write_block_data(
            self.address,
            LED0_ON_L + 4 * channel,
            [on & 0xFF, on >> 8, off & 0xFF, off >> 8],
        )

    def set_pulse_us(self, channel: int, microseconds: float) -> None:
        """Kanala verilen darbe genişliğini mikrosaniye olarak ayarla."""
        period_us = 1_000_000.0 / self._frequency_hz
        if microseconds <= 0:
            self.set_off(channel)
            return
        if microseconds >= period_us:
            raise BackendError(
                f"{microseconds:.0f} us, {self._frequency_hz:.0f} Hz periyodundan "
                f"({period_us:.0f} us) uzun."
            )
        counts = int(round(microseconds * STEPS / period_us))
        counts = max(0, min(STEPS - 1, counts))
        self.set_pwm(channel, 0, counts)

    def set_off(self, channel: int) -> None:
        """Kanalı tamamen kapat (servo serbest kalır, tork uygulamaz)."""
        _check_channel(channel)
        # LEDn_OFF_H bit 4 = tam kapalı
        self.backend.write_block_data(
            self.address, LED0_ON_L + 4 * channel, [0x00, 0x00, 0x00, 0x10]
        )

    def all_off(self) -> None:
        """Karttaki bütün kanalları kapat."""
        self.backend.write_block_data(self.address, ALL_LED_ON_L, [0x00, 0x00, 0x00, 0x10])

    def __repr__(self) -> str:
        return f"PCA9685(0x{self.address:02x}, {self._frequency_hz:.0f} Hz)"


def _check_channel(channel: int) -> None:
    if not 0 <= channel < CHANNELS:
        raise BackendError(f"PCA9685 kanalı 0-{CHANNELS - 1} arasında olmalı, {channel} verildi")
