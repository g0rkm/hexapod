"""Servo sürücü katmanı: eklem adı -> kart/kanal -> darbe.

Bu katmanın üstündeki hiçbir katman I2C, kanal numarası veya mikrosaniye
görmez; sadece eklem adı ve açı bilir.

Katmanın iki giriş kapısı var:
  set_pulse_us() : ham darbe. Kalibrasyon aracı bunu kullanır.
  set_angle()    : derece cinsinden eklem açısı. IK ve gait bunu kullanır,
                   kalibrasyon tamamlanmadan çalışmaz.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .backends import DryRunBackend, I2CBackend, SMBusBackend
from .calibration import Calibration
from .config import JointSpec, RobotConfig
from .errors import LimitError, MissingValue
from .pca9685 import PCA9685


@dataclass(frozen=True)
class JointState:
    """Bir eklemin en son gönderilen komutu."""

    pulse_us: int | None  # None = kanal kapalı (servo serbest)


class ServoBus:
    """18 eklemi iki PCA9685 kartı üzerinden süren katman."""

    def __init__(
        self,
        config: RobotConfig,
        calibration: Calibration | None = None,
        backend: I2CBackend | None = None,
        dry_run: bool = False,
    ):
        self.config = config
        self.calibration = calibration or Calibration.empty()
        self.dry_run = dry_run
        self._state: dict[str, JointState] = {}

        if backend is not None:
            self._backend = backend
            self._owns_backend = False
        elif dry_run:
            self._backend = DryRunBackend()
            self._owns_backend = True
        else:
            buses = {d.i2c_bus for d in config.drivers.values()}
            if len(buses) != 1:
                raise MissingValue(
                    "drivers[*].i2c_bus",
                    "Tüm kartlar aynı I2C veri yolunda olmalı ya da arka uç elle verilmeli",
                )
            self._backend = SMBusBackend(buses.pop())
            self._owns_backend = True

        self._boards: dict[int, PCA9685] = {}
        self._started = False

    # -- yaşam döngüsü ----------------------------------------------------

    def start(self) -> None:
        """Kartları başlat. Adresler girilmemişse burada hata verir."""
        for driver_id, spec in sorted(self.config.drivers.items()):
            address = spec.address.require()
            board = PCA9685(self._backend, int(address), self.config.pwm_frequency_hz)
            board.begin()
            self._boards[driver_id] = board
        self._started = True

    def stop(self) -> None:
        """Tüm çıkışları kapat ve veri yolunu bırak."""
        for board in self._boards.values():
            try:
                board.all_off()
            except Exception:
                pass
        self._state.clear()
        if self._owns_backend:
            self._backend.close()
        self._started = False

    def __enter__(self) -> "ServoBus":
        self.start()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.stop()

    # -- çıkış ------------------------------------------------------------

    def set_pulse_us(self, leg: int, joint: str, microseconds: float) -> int:
        """Ham darbe gönder. Mutlak güvenlik sınırları dışına çıkamaz.

        Dönen değer, gerçekten gönderilen (kırpılmamış) darbedir.
        """
        spec = self.config.joint(leg, joint)
        us = int(round(microseconds))
        lo, hi = self.config.pulse_us_min, self.config.pulse_us_max
        if not lo <= us <= hi:
            raise LimitError(
                f"{spec}: {us} us, güvenlik aralığının ({lo}-{hi} us) dışında. "
                "Bu sınırlar config/robot.yaml -> servo.pulse_us_hard_limits."
            )
        board, channel = self._resolve(spec)
        board.set_pulse_us(channel, us)
        self._state[spec.key] = JointState(pulse_us=us)
        return us

    def set_angle(self, leg: int, joint: str, degrees: float) -> int:
        """Eklem açısı gönder. Kalibrasyon tamamlanmamışsa hata verir.

        0 derece = calibration.yaml içindeki center_us konumu.
        """
        spec = self.config.joint(leg, joint)
        cal = self.calibration.get(spec.key)
        center, direction, us_per_deg = cal.require_angle_terms(spec.key)

        lo = spec.limit_min.require()
        hi = spec.limit_max.require()
        if not lo <= degrees <= hi:
            raise LimitError(
                f"{spec}: {degrees:.1f} derece, eklem limitlerinin "
                f"({lo:.1f} .. {hi:.1f}) dışında."
            )
        return self.set_pulse_us(leg, joint, center + direction * degrees * us_per_deg)

    def release(self, leg: int, joint: str) -> None:
        """Tek eklemi serbest bırak (darbe kes, servo tork uygulamaz)."""
        spec = self.config.joint(leg, joint)
        board, channel = self._resolve(spec)
        board.set_off(channel)
        self._state[spec.key] = JointState(pulse_us=None)

    def release_all(self) -> None:
        """Bütün servoları serbest bırak.

        Akım çekişini sıfırlar. Brifteki brownout riski için ilk savunma.
        """
        for board in self._boards.values():
            board.all_off()
        for spec in self.config.joints:
            self._state[spec.key] = JointState(pulse_us=None)

    # -- durum ------------------------------------------------------------

    def state(self, leg: int, joint: str) -> JointState:
        spec = self.config.joint(leg, joint)
        return self._state.get(spec.key, JointState(pulse_us=None))

    def active_joints(self) -> list[str]:
        return [k for k, v in self._state.items() if v.pulse_us is not None]

    # -- iç işler ---------------------------------------------------------

    def _resolve(self, spec: JointSpec) -> tuple[PCA9685, int]:
        if not self._started:
            raise RuntimeError("ServoBus.start() çağrılmadı.")
        driver_id = int(spec.driver.require())
        channel = int(spec.channel.require())
        board = self._boards.get(driver_id)
        if board is None:
            raise MissingValue(
                spec.driver.path,
                f"{driver_id} numaralı sürücü kartı config/robot.yaml'da tanımlı değil",
            )
        return board, channel


def iter_joint_keys(config: RobotConfig) -> Iterable[str]:
    for spec in config.joints:
        yield spec.key
