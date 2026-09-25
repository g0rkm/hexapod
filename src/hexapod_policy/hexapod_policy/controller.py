"""hexapod_policy çekirdeği: IMU + hız komutu -> politika -> eklem hedefi, ROS'suz (G8).

node.py bunu ince bir rclpy kabuğuyla sarar; asıl mantık burada, robotsuz ve
ROS'suz test edilir (hexapod_teleop ile aynı desen).

Gözlem eğitimdekiyle BİREBİR aynı kurulur (hexapod_rl.task.observation;
tests/test_policy_controller.py ikisini karşılaştırır):

    yerçekimi yönü, gövde çerçevesinde (3)   IMU yöneliminden
    açısal hız, gövde çerçevesinde (3)      IMU jiroskobu
    son eklem hedefleri, normalize (18)     bu denetleyicinin son komutu
    hız komutu vx, vy, wz (3)
    adım saati sin, cos (2)                 politikanın eğitildiği frekansta

Güvenlik: aşağıdaki durumlarda politika koşmaz, robot ayakta duruş pozunda
bekler. Adım saati de sıfırlanır; yürüyüş yeniden başlarken politika,
eğitimdeki bölüm başıyla aynı durumu görür.
  - hız komutu yok ya da zaman aşımı (deadman; komut veren taraf çökerse
    robot yürümeye devam etmesin),
  - ileri hız eğitimde görülen aralığın yarısının altında (dur komutu; geri,
    yana ya da yerinde dönüş gibi eğitilmemiş komutlar da buraya düşer),
  - IMU verisi yok ya da bayat (politika kör koşmasın),
  - gövde max_tilt_deg'den fazla yatmış (eğitimde bölüm burada biterdi).
Eğitim aralığının dışındaki komut aralığa kırpılır (clipped_command).
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from .mlp import MlpPolicy

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]   # (w, x, y, z), dünya <- gövde


def gravity_in_base(q: Quat) -> Vec3:
    """Yerçekimi yönünün (dünyada -z) gövde çerçevesindeki birim vektörü."""
    w, x, y, z = q
    # R^T (0, 0, -1): R'nin üçüncü satırının eksisi
    return (-2.0 * (x * z - w * y), -2.0 * (y * z + w * x), -(1.0 - 2.0 * (x * x + y * y)))


class PolicyController:
    """Kullanım:
        c = PolicyController(MlpPolicy.load("policy.npz"), limits)
        c.on_imu((w, x, y, z), (gx, gy, gz), now)    # her IMU mesajında
        c.on_command(vx, vy, wz, now)                # her /cmd_vel'de
        targets = c.tick(now)                        # 50 Hz; 18 değer, radyan
    """

    def __init__(self, policy: MlpPolicy, limits: Sequence[tuple[float, float]],
                 cmd_timeout_s: float = 0.5, imu_timeout_s: float = 0.2,
                 max_tilt_deg: float = 45.0) -> None:
        c = policy.contract
        if len(limits) != c.action_size:
            raise ValueError(f"{c.action_size} eklem limiti bekleniyordu, {len(limits)} geldi")
        self.policy = policy
        self.limits = [(float(lo), float(hi)) for lo, hi in limits]
        self.stand = [_clip(d, lim) for d, lim in zip(c.default_rad, self.limits)]
        self.cmd_timeout_s = cmd_timeout_s
        self.imu_timeout_s = imu_timeout_s
        self.max_tilt_deg = max_tilt_deg
        self._command: tuple[float, float, float] | None = None
        self._command_at = -math.inf
        self._imu: tuple[Quat, Vec3] | None = None
        self._imu_at = -math.inf
        self._phase = 0.0
        self._targets = list(self.stand)
        self.status = "başlıyor"
        self.clipped_command = False

    # -- girdiler -----------------------------------------------------------------

    def on_command(self, vx: float, vy: float, wz: float, now: float) -> None:
        self._command = (float(vx), float(vy), float(wz))
        self._command_at = now

    def on_imu(self, quat_wxyz: Sequence[float], gyro: Sequence[float], now: float) -> None:
        q = tuple(float(v) for v in quat_wxyz)
        n = math.sqrt(sum(v * v for v in q))
        if not n > 0.5 or not all(math.isfinite(v) for v in (*q, *gyro)):
            return   # bozuk ölçüm; bayatlayınca politika durur
        self._imu = (tuple(v / n for v in q), tuple(float(v) for v in gyro))
        self._imu_at = now

    # -- kontrol adımı ------------------------------------------------------------

    def tick(self, now: float) -> list[float]:
        """Bir kontrol adımı: yayınlanacak 18 eklem hedefi (radyan, interface sırası)."""
        reason = self._hold_reason(now)
        if reason is not None:
            self._phase = 0.0
            self._targets = list(self.stand)
            self.status = reason
            return list(self._targets)
        c = self.policy.contract
        action = np.clip(self.policy(self.observation(self._effective_command())), -1.0, 1.0)
        self._targets = [_clip(d + c.action_scale * float(a), lim)
                         for d, a, lim in zip(c.default_rad, action, self.limits)]
        self._phase = (self._phase + c.gait_hz / c.control_hz) % 1.0
        self.status = "yürüyor"
        return list(self._targets)

    def observation(self, command: tuple[float, float, float]) -> np.ndarray:
        c = self.policy.contract
        q, gyro = self._imu
        targets = [(t - d) / c.action_scale for t, d in zip(self._targets, c.default_rad)]
        clock = (math.sin(2 * math.pi * self._phase), math.cos(2 * math.pi * self._phase))
        obs = np.array([*gravity_in_base(q), *gyro, *targets, *command, *clock])
        assert obs.shape == (c.obs_size,)
        return obs

    # -- iç -----------------------------------------------------------------------

    def _hold_reason(self, now: float) -> str | None:
        if self._command is None:
            return "komut yok"
        if now - self._command_at > self.cmd_timeout_s:
            return "komut zaman aşımı"
        vx_lo = self.policy.contract.command_ranges["vx"][0]
        if self._command[0] < 0.5 * vx_lo:
            return f"dur (vx < {0.5 * vx_lo:g} m/s)"
        if self._imu is None:
            return "IMU yok"
        if now - self._imu_at > self.imu_timeout_s:
            return "IMU bayat"
        g = gravity_in_base(self._imu[0])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -g[2]))))
        if tilt > self.max_tilt_deg:
            return f"devrildi ({tilt:.0f}°)"
        return None

    def _effective_command(self) -> tuple[float, float, float]:
        ranges = self.policy.contract.command_ranges
        out = tuple(_clip(v, ranges[k]) for v, k in zip(self._command, ("vx", "vy", "wz")))
        self.clipped_command = any(abs(a - b) > 1e-6 for a, b in zip(out, self._command))
        return out


def _clip(value: float, lim: tuple[float, float]) -> float:
    return max(lim[0], min(lim[1], value))


__all__ = ["PolicyController", "gravity_in_base"]
