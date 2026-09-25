"""hexapod_teleop çekirdeği: hız komutundan eklem hedefine, ROS'suz (GOREVLER.md S3).

node.py bu sınıfı ince bir rclpy kabuğuyla sarar (subscribe/publish/timer);
asıl mantık burada, robotsuz test edilebilir (hexapod_gait/hexapod_rl'deki
"çekirdek saf Python, ROS ayrı" ayrımıyla aynı desen).
"""

from __future__ import annotations

from dataclasses import dataclass

from hexapod_description.interface import to_command
from hexapod_gait import GaitParams, TripodGait
from hexapod_kinematics import HexapodKinematics, ReachError

Command = tuple[float, float, float]  # vx m/s, vy m/s, wz rad/s


@dataclass(frozen=True)
class TeleopLimits:
    """Bu düğümün kendi güvenlik payı — robot parametresi DEĞİL.

    S2'nin test ettiği aralık (tests/test_tripod_gait.py: düz/yana/dönerek
    0.15 m/s'e kadar ReachError'sız). Bunun dışı hiç denenmedi.
    """

    vx_max: float = 0.15
    vy_max: float = 0.08
    wz_max: float = 0.5


class TeleopController:
    """Hız komutunu TripodGait'e taşıyan, ROS'a bağımlı olmayan çekirdek.

    Kullanım:
        controller = TeleopController(kin)
        controller.on_command(vx, vy, wz, now)   # her Twist geldiğinde
        command = controller.tick(now, dt)        # kontrol hızında (50 Hz) bir
        if command is not None:
            publisher.publish(Float64MultiArray(data=command))  # 18 değer, radyan

    now, dt: saniye (tutarlı bir saat kullanılsın yeter — gerçek zaman ya da
    test saatinde farketmez).
    """

    def __init__(self, kin: HexapodKinematics, params: GaitParams | None = None,
                 limits: TeleopLimits | None = None, timeout_s: float = 0.5) -> None:
        self.gait = TripodGait(kin, params)
        self.gait.reset()
        self.limits = limits or TeleopLimits()
        self.timeout_s = timeout_s
        self._command: Command = (0.0, 0.0, 0.0)
        self._last_command_at: float | None = None
        self._last_good: list[float] | None = None

    def on_command(self, vx: float, vy: float, wz: float, now: float) -> None:
        """Yeni bir hız komutu geldi (ör. /cmd_vel). Güvenlik sınırına kırpılır."""
        lim = self.limits
        self._command = (
            _clamp(vx, lim.vx_max),
            _clamp(vy, lim.vy_max),
            _clamp(wz, lim.wz_max),
        )
        self._last_command_at = now

    def tick(self, now: float, dt: float) -> list[float] | None:
        """Bir kontrol adımı ilerle; yayınlanacak 18 değeri (radyan) döndür.

        Komut zaman aşımına uğradıysa (timeout_s içinde on_command çağrılmadıysa,
        ya da hiç çağrılmadıysa) SIFIR hızla ilerler (deadman) — komut veren
        taraf çökerse/bağlantı keserse robot sonsuza kadar yürümeye devam
        etmesin diye. Sıfır hızda gövde ilerlemez, yalnızca adım saati
        dönmeye devam eder (bacaklar yerinde hafifçe iner kalkar); bu bir
        tuzak değil, TripodGait'in tasarımı gereği böyle (faz komuttan
        bağımsız akar).

        Hedef erişilemezse (ReachError) son geçerli komut tekrar döner;
        döngü çökmez, robot son iyi durumda kalır.
        """
        stale = self._last_command_at is None or (now - self._last_command_at) > self.timeout_s
        vx, vy, wz = (0.0, 0.0, 0.0) if stale else self._command
        try:
            angles = self.gait.step(vx, vy, wz, dt)
        except ReachError:
            return self._last_good
        self._last_good = to_command(angles)
        return self._last_good


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


__all__ = ["Command", "TeleopController", "TeleopLimits"]
