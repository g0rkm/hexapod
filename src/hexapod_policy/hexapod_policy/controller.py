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
  - dur komutu: yalnız ileri eğitilmiş politikada ileri hız eğitim
    aralığının yarısının altında (geri, yana ya da yerinde dönüş gibi
    eğitilmemiş komutlar da buraya düşer); her yöne eğitilmiş politikada
    komut, sözleşmenin ölü bölgesinde (command_deadband; _stop_reason),
  - IMU verisi yok ya da bayat (politika kör koşmasın),
  - gövde max_tilt_deg'den fazla yatmış (eğitimde bölüm burada biterdi).
Eğitim aralığının dışındaki komut aralığa kırpılır (clipped_command).

Artık eylem modunda (sözleşme action_mode = "residual") hedef, adım
saatinin tripod'u (tripod.PhaseTripod; eğitimdekiyle aynı kod) + politikanın
düzeltmesidir; bunun için robotun kinematiği (kin) verilmelidir.

Mesafe sensörlü kaldırma refleksi (isteğe bağlı; lift_reflex, 2026-09-27):
range_sensors (yerleşim) + reflex verilirse her salınımın başında taban
tripod'un ayak kaldırmasını refleks seçer (on_ranges ile gelen mesafeler;
engel yüksekliği LiftReflex.reference'a göre, varsayılan gövde düzlemi).
Mesafe gelmiyor ya da bayatsa (range_timeout_s) ya da yürüyüş yönüne
hiçbir sensör bakmıyorsa (lift_reflex.covers) refleks devre dışı, politika
kör davranışına döner: kaldırma çıkışı varsa onun seçtiği, yoksa
tabanın sabit kaldırması (reflex_status). Yerleşim robot.yaml'dan (D8)
gelmeli; bu katman varsayılan koymaz.
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from .lift_reflex import LiftReflex, RangeSensor, covers
from .mlp import MlpPolicy
from .tripod import PhaseTripod

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
                 max_tilt_deg: float = 45.0, kin=None,
                 range_sensors: Sequence[RangeSensor] | None = None,
                 reflex: LiftReflex | None = None, range_timeout_s: float = 0.2) -> None:
        c = policy.contract
        self.base = None
        if c.action_mode == "residual":
            if kin is None:
                raise ValueError("artık eylem politikası robotun kinematiğini (kin) ister")
            g = c.base_gait
            self.base = PhaseTripod(kin, tuple(tuple(x) for x in g["groups"]), c.gait_hz,
                                    g["reach_mm"], g["height_mm"], g["lift_mm"])
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
        self.lift_mm: float | None = None   # öğrenilmiş ayak kaldırmada son seçilen, mm
        self._lift_group: int | None = None
        # mesafe sensörlü kaldırma refleksi
        self.range_sensors = tuple(range_sensors) if range_sensors else None
        self.reflex = reflex
        if (self.reflex is None) != (self.range_sensors is None):
            raise ValueError("refleks için mesafe sensörü yerleşimi ve LiftReflex birlikte verilir")
        if self.reflex is not None and self.base is None:
            raise ValueError("refleks taban tripod'un kaldırmasını seçer: yalnız artık eylem modunda")
        self.range_timeout_s = range_timeout_s
        self._ranges: tuple[float, ...] | None = None
        self._ranges_at = -math.inf
        self._ranges_new = False
        self._reflex_lift: float | None = None
        self.reflex_status: str | None = None   # "görüyor" / "mesafe yok" / "mesafe bayat"

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

    def on_ranges(self, distances: Sequence[float], now: float) -> None:
        """Mesafe sensörlerinin yeni ölçümü, m, range_sensors sırasıyla (menzil
        dışı: sensörün max_m'si). Bozuk ölçüm yok sayılır; bayatlayınca refleks
        devre dışı kalır."""
        if self.range_sensors is None:
            return
        d = tuple(float(v) for v in distances)
        if len(d) != len(self.range_sensors) or not all(math.isfinite(v) and v >= 0 for v in d):
            return
        self._ranges, self._ranges_at, self._ranges_new = d, now, True

    # -- kontrol adımı ------------------------------------------------------------

    def tick(self, now: float) -> list[float]:
        """Bir kontrol adımı: yayınlanacak 18 eklem hedefi (radyan, interface sırası)."""
        reason = self._hold_reason(now)
        if reason is not None:
            self._phase = 0.0
            self.lift_mm, self._lift_group = None, None
            if self.reflex is not None:
                self.reflex.reset()
                self._reflex_lift = None
            self._targets = list(self.stand)
            self.status = reason
            return list(self._targets)
        c = self.policy.contract
        command = self._effective_command()
        out = np.clip(self.policy(self.observation(command)), -1.0, 1.0)
        action = out[:c.action_size]
        reflex_lift = self._update_reflex(now, command)
        if self.base is not None and (c.lift_range is not None or self.reflex is not None):
            # ayak kaldırma yalnız salınımın ilk adımında seçilir, salınım boyunca
            # sabit (hexapod_rl.env ile aynı): refleks görüyorsa onun seçtiği, yoksa
            # politikanın son çıkışı (eğitimdeki eşlemeyle), o da yoksa tabanınki
            group = self.base.swing_group(self._phase)
            if group != self._lift_group:
                if reflex_lift is not None:
                    self.lift_mm = reflex_lift
                elif c.lift_range is not None:
                    lo, hi = c.lift_range
                    self.lift_mm = lo + (float(out[c.action_size]) + 1.0) * 0.5 * (hi - lo)
                else:
                    self.lift_mm = None
                self._lift_group = group
        if self.base is None:
            base, scale = c.default_rad, c.action_scale
        else:
            base = self.base.targets(self._phase, command, self.lift_mm)
            scale = c.residual_scale
        self._targets = [_clip(b + scale * float(a), lim)
                         for b, a, lim in zip(base, action, self.limits)]
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

    def _update_reflex(self, now: float, command) -> float | None:
        """Refleksin kaldırması, mm; refleks yoksa, mesafe gelmiyorsa ya da yürüyüş
        yönüne hiçbir sensör bakmıyorsa (lift_reflex.covers) None: görmediği yönde
        refleks 25 mm'de kalıp takılırdı, kör davranış daha iyi (models/README).
        Refleks yalnız YENİ bir ölçümle güncellenir (sensör kontrol hızından
        yavaşsa aynı ölçüm art arda sayılmasın)."""
        if self.reflex is None:
            return None
        if not covers(self.range_sensors, command[0], command[1]):
            self.reflex_status = "yön görülmüyor"
            return None
        if self._ranges is None or now - self._ranges_at > self.range_timeout_s:
            self.reflex_status = "mesafe yok" if self._ranges is None else "mesafe bayat"
            self.reflex.reset()
            self._reflex_lift = None
            return None
        if self._ranges_new or self._reflex_lift is None:
            stand = self.policy.contract.base_gait["height_mm"] / 1000.0
            heights = self.reflex.heights(self.range_sensors, self._ranges,
                                          gravity_in_base(self._imu[0]), stand, now)
            self._reflex_lift = self.reflex.update(now, heights)
            self._ranges_new = False
        self.reflex_status = "görüyor"
        return self._reflex_lift

    def _hold_reason(self, now: float) -> str | None:
        if self._command is None:
            return "komut yok"
        if now - self._command_at > self.cmd_timeout_s:
            return "komut zaman aşımı"
        stop = self._stop_reason(self._command)
        if stop is not None:
            return stop
        if self._imu is None:
            return "IMU yok"
        if now - self._imu_at > self.imu_timeout_s:
            return "IMU bayat"
        g = gravity_in_base(self._imu[0])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -g[2]))))
        if tilt > self.max_tilt_deg:
            return f"devrildi ({tilt:.0f}°)"
        return None

    def _stop_reason(self, command) -> str | None:
        """Eğitilmemiş küçük ya da ters komut: politika koşmaz, robot ayakta bekler.

        Aralığı 0'ı içermeyen eksende (eski dosyalarda ileri hız 0.05-0.15)
        aralığın alt ucunun yarısının altı; aralığı 0'ı içeren eksenlerde ise
        komutun büyüklüğü sözleşmenin ölü bölgesinin altıysa (her yöne
        eğitilmiş dosyalar; eğitimde bu kadar küçük komut yoktu)."""
        c = self.policy.contract
        size = 0.0
        for axis, v in zip(("vx", "vy", "wz"), command):
            lo, hi = c.command_ranges[axis]
            if lo > 0 and v < 0.5 * lo:
                return f"dur ({axis} < {0.5 * lo:g})"
            if hi < 0 and v > 0.5 * hi:
                return f"dur ({axis} > {0.5 * hi:g})"
            if hi > lo:
                size = max(size, abs(v) / max(abs(lo), abs(hi)))
        if size < c.command_deadband:
            return f"dur (komut ölü bölgede, {size:.2f} < {c.command_deadband:.2f})"
        return None

    def _effective_command(self) -> tuple[float, float, float]:
        ranges = self.policy.contract.command_ranges
        out = tuple(_clip(v, ranges[k]) for v, k in zip(self._command, ("vx", "vy", "wz")))
        self.clipped_command = any(abs(a - b) > 1e-6 for a, b in zip(out, self._command))
        return out


def _clip(value: float, lim: tuple[float, float]) -> float:
    return max(lim[0], min(lim[1], value))


__all__ = ["PolicyController", "gravity_in_base"]
