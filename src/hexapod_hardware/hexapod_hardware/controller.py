"""Gerçek robot sürücüsünün ROS'suz çekirdeği: komut dizisi -> servo darbesi (GOREVLER.md S4).

node.py bu sınıfı ince bir rclpy kabuğuyla sarar; asıl mantık burada, robotsuz
(DryRunBackend ile) test edilir. hexapod_driver'daki "donanıma dokunan katman
saf Python kalsın, ROS sarmalayıcısı ayrı olsun" ayrımının aynısı.

Sözleşme docs/ARAYUZ.md'de: komut 18 değer, RADYAN, interface.joint_names()
sırasıyla. Bozuk komut (yanlış uzunluk, sonlu olmayan değer) servoya gitmemeli;
limit dışı komut gerçek robotta LimitError verir, yine servoya gitmez.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from hexapod_description.interface import from_command, joint_names
from hexapod_driver import HexapodError, ServoBus


@dataclass(frozen=True)
class Result:
    """Bir komutun sonucu. accepted=False ise servolara HİÇBİR ŞEY gitmedi."""

    accepted: bool
    reason: str = ""


class DriverController:
    """Eklem komutunu ServoBus'a taşır.

    Kullanım:
        bus = ServoBus(config, calibration, dry_run=True)
        controller = DriverController(bus)
        controller.start()
        result = controller.on_command(msg.data)        # 18 değer, radyan
        state = controller.joint_state()                # (adlar, radyan) ya da None
        controller.stop()

    Komut ya hep birlikte uygulanır ya hiç: 18 eklemin hepsi önce doğrulanır
    (ServoBus.set_angles), biri reddedilirse hiçbir servo kıpırdamaz.
    """

    def __init__(self, bus: ServoBus) -> None:
        self.bus = bus
        self._names = joint_names(range(6))
        self._last: list[float] | None = None

    def start(self) -> None:
        """Kartları başlat. Adresler girilmemişse MissingValue fırlatır."""
        self.bus.start()

    def stop(self) -> None:
        """Bütün servoları serbest bırak (tork kesilir) ve veri yolunu kapat."""
        self.bus.stop()

    def on_command(self, data: Sequence[float]) -> Result:
        try:
            angles = from_command(data)  # derece; uzunluk ve sonlu sayı denetimi burada
        except ValueError as exc:
            return Result(False, f"bozuk komut: {exc}")

        targets = {
            (leg, part): degrees
            for leg, a in angles.items()
            for part, degrees in a.as_dict().items()
        }
        try:
            self.bus.set_angles(targets)
        except HexapodError as exc:  # LimitError, MissingValue, BackendError
            return Result(False, f"{type(exc).__name__}: {exc}")

        self._last = [float(v) for v in data]
        return Result(True)

    def joint_state(self) -> tuple[list[str], list[float]] | None:
        """(eklem adları, son gönderilen komut, radyan). Henüz komut yoksa None.

        MG996R konum geri bildirimi vermez: bu ÖLÇÜM değil, son gönderilen
        komuttur (docs/ARAYUZ.md). İlk komuttan önce hiçbir konum bilinmez;
        uydurulmaz, None döner.
        """
        if self._last is None:
            return None
        return list(self._names), list(self._last)


__all__ = ["DriverController", "Result"]
