"""Gymnasium ortamı: HexapodEnv (GOREVLER.md G6).

    from hexapod_rl.env import HexapodEnv
    env = HexapodEnv()
    obs, info = env.reset(seed=0)
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())

Mantık task.py'de (gözlem, ödül, devrilme), simülasyon sim.py'de; bu dosya
ikisini Gymnasium arayüzüne bağlar. Bir adım = bir kontrol adımı (1/50 s),
gerçek robottaki komut hızıyla aynı (interface.COMMAND_RATE_HZ).

Paralel ortamlar için make_env() kullanın (her süreç kendi Gazebo'sunu
kurar): SubprocVecEnv([make_env(i) for i in range(8)]).

Alan rastgeleleştirme (TaskConfig.randomization, varsayılan kapalı): her
bölüm başında servo gücü/sertliği ve komut gecikmesi çekilir; bölüm boyunca
rastgele aralıklarla gövde yandan itilir; gözlemdeki IMU değerlerine gürültü
eklenir. Çekilen değerler reset()'in info'sunda ("dynamics"). Gövde kütlesi
ortam başına sabit (body_mass_scale; train.py ortamlara task.body_mass_scales
ile dağıtır).

Müfredat (terrain_levels, varsayılan yok): kolaydan zora zemin üreteçleri
listesi. Ortam en kolayından başlar; her bölüm sonunda robot doğduğu yerden
CURRICULUM_PROMOTE_M kadar uzaklaştıysa bir zorlaşır, devrildiyse ya da
komutun istediği yolun yarısını gidemediyse bir kolaylaşır; en zoru geçince
rastgele bir seviyeye döner (unutmasın diye). Zemin değişince Gazebo dünyası
yeniden kurulur (~0.1 s, bir reset'ten biraz fazla; ölçüldü 2026-09-27).

Sabit bozulmalar (Perturbation, varsayılan yok): eğitimde rastgeleleştirilmeyen
ama robota geçişte beklenen hatalar (kalibrasyon ofseti, eğik takılmış IMU,
kontrol adımından uzun gecikme, zayıf servo). Dayanıklılık taraması
(hexapod_rl.robustness) bunlarla modelin nerede bozulduğunu ölçer.
"""

from __future__ import annotations

import copy
import math
from collections import deque
from dataclasses import replace
from pathlib import Path
from typing import Callable, Sequence

import gymnasium as gym
import numpy as np

from hexapod_description.model import RobotModel
from hexapod_driver.config import RobotConfig
from hexapod_gazebo.pose import standing_pose
from hexapod_kinematics import HexapodKinematics
from hexapod_policy.tripod import PhaseTripod

from hexapod_policy.lift_reflex import LiftReflex, covers, obstacle_height

from . import rangefinder
from .rangefinder import RangeSensor
from .sim import HexapodSim, TerrainHeight
from .task import (
    ACTION_SIZE,
    OBS_SIZE,
    Perturbation,
    TaskConfig,
    VelocityFilter,
    action_dim,
    action_to_targets,
    fallen,
    lift_from_action,
    observation,
    reward,
    sample_command,
    tripod_groups,
)


#: Müfredat: bölümde doğduğu yerden bu kadar (m, yatay) uzaklaşan zorlaşır.
#: Deneme zeminlerinde çukurdan çıkış 0.35 m, yayladan iniş 0.5 m.
CURRICULUM_PROMOTE_M = 0.5
#: ... komutun istediği yolun (en çok CURRICULUM_PROMOTE_M) bu kesrine
#: varamayan kolaylaşır. Yerinde dönüş gibi yavaş komutlarda karar verilmez.
CURRICULUM_DEMOTE_FRAC = 0.5
CURRICULUM_MIN_SPEED = 0.03   # m/s

TerrainMaker = Callable[[], tuple]   # () -> (terrain_sdf, terrain_height)


def _tilt_matrix(roll_deg: float, pitch_deg: float) -> np.ndarray:
    """IMU çerçevesinden gövdeye dönme (önce roll x ekseni, sonra pitch y ekseni)."""
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    return ry @ rx


class HexapodEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, task: TaskConfig | None = None, physics_step: float = 0.002,
                 config_path: str | Path | None = None, workdir: Path | None = None,
                 terrain_sdf: str = "", terrain_height: TerrainHeight | None = None,
                 body_mass_scale: float = 1.0, perturbation: Perturbation | None = None,
                 terrain_levels: Sequence[TerrainMaker] | None = None,
                 range_sensors: Sequence[RangeSensor] | None = None,
                 lift_reflex: LiftReflex | None = None, range_noise: float = 0.0,
                 range_drop: float = 0.0) -> None:
        """terrain_sdf: düz zeminin yerine geçen statik <model> SDF parçası (S5'in
        zemin üreteci; boşsa düz zemin); terrain_height(x, y): aynı zeminin üst
        yüzeyinin z'si, m (ikisi birlikte). Kısıtlar HexapodSim açıklamasında.
        body_mass_scale: gövde kütlesi (ve ataleti) bu çarpanla; bu ortamın
        ömrü boyunca sabit (task.body_mass_scales). perturbation: sabit
        bozulmalar (Perturbation; dayanıklılık taraması). terrain_levels:
        müfredat, kolaydan zora zemin üreteçleri (terrain_probe.CURRICULA);
        verilirse terrain_sdf/terrain_height verilmez, seviye 0'dan başlar.
        range_sensors: DENEYSEL mesafe sensörü yerleşimi (rangefinder); her
        adımda ölçümler info["ranges_m"]'de, gözlem değişmez. range_noise /
        range_drop: ölçüme bağıl gürültü (std) ve okumanın gelmeme olasılığı.
        lift_reflex: taban tripod'un kaldırmasını refleks seçer (robottaki
        PolicyController ile aynı kural: salınım başında; yürüyüş yönüne sensör
        bakmıyorsa politikanın kaldırma çıkışı ya da tabanınki). Kopyalanır:
        her ortamın kendi durumu olur."""
        super().__init__()
        self.task = task or TaskConfig()
        config = RobotConfig.load(config_path)
        model = RobotModel.from_config(config).with_body_mass_scale(body_mass_scale)
        self.body_mass_scale = body_mass_scale
        self._model, self._physics_step = model, physics_step
        self.terrain_levels = list(terrain_levels) if terrain_levels else None
        self.range_sensors = tuple(range_sensors) if range_sensors else None
        self.ranges: list[float] | None = None
        self.lift_reflex = copy.deepcopy(lift_reflex)
        self.range_noise, self.range_drop = float(range_noise), float(range_drop)
        self._reflex_lift: float | None = None
        if self.lift_reflex is not None and self.range_sensors is None:
            raise ValueError("refleks için mesafe sensörü yerleşimi (range_sensors) gerekir")
        self.level = 0
        if self.terrain_levels is not None:
            if terrain_sdf or terrain_height is not None:
                raise ValueError("müfredatta zemin seviyelerden gelir; terrain_sdf verilmez")
            terrain_sdf, terrain_height = self.terrain_levels[0]()
        self.sim = HexapodSim(model, physics_step=physics_step, workdir=workdir,
                              terrain_sdf=terrain_sdf, terrain_height=terrain_height)
        self.dt = self.sim.dt
        self.perturbation = perturbation or Perturbation()
        pert = self.perturbation
        if pert.joint_offset_deg:
            if len(pert.joint_offset_deg) != len(self.sim.names):
                raise ValueError(f"{len(self.sim.names)} eklem ofseti bekleniyordu, "
                                 f"{len(pert.joint_offset_deg)} geldi")
            self.sim.joint_offset = [math.radians(v) for v in pert.joint_offset_deg]
        if pert.delay_ms < 0 or pert.servo_strength <= 0:
            raise ValueError(f"geçersiz bozulma: {pert}")
        self._imu_tilt = (None if pert.imu_tilt_deg == (0.0, 0.0)
                          else _tilt_matrix(*pert.imu_tilt_deg).T.astype(np.float32))
        self._delay_steps = 0                 # kontrol adımı; _randomize_dynamics kurar
        self._delayed: deque = deque()        # yolda olan komutlar
        self._commanded: list[float] = []     # son komut (gözlem bunu görür)

        pose = standing_pose(HexapodKinematics.from_config(config),
                             self.task.stand_reach_mm, self.task.stand_height_mm)
        self.default = [math.radians(v) for leg in sorted(pose)
                        for v in pose[leg].as_dict().values()]
        self.groups = tripod_groups({leg: m.yaw for leg, m in model.mounts.items()})
        if self.task.action_mode not in ("absolute", "residual"):
            raise ValueError(f"bilinmeyen eylem modu: {self.task.action_mode!r}")
        self.base = None
        if self.task.action_mode == "residual":
            t = self.task
            self.base = PhaseTripod(HexapodKinematics.from_config(config), self.groups,
                                    t.gait_hz, t.stand_reach_mm, t.stand_height_mm, t.lift_mm)
        self.limits = [(model.limits[(int(n[3]), n.split("_")[1])].lower,
                        model.limits[(int(n[3]), n.split("_")[1])].upper)
                       for n in self.sim.names]

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (OBS_SIZE,), np.float32)
        if self.task.lift_action is not None and self.base is None:
            raise ValueError("öğrenilmiş ayak kaldırma yalnız artık eylem modunda")
        if self.lift_reflex is not None and self.base is None:
            raise ValueError("refleks taban tripod'un kaldırmasını seçer: yalnız artık eylem modunda")
        self.n_actions = action_dim(self.task)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (self.n_actions,), np.float32)
        self.max_steps = int(round(self.task.episode_s / self.dt))

        self._state = None
        self._command = (0.0, 0.0, 0.0)
        self._phase = 0.0
        self._lift, self._lift_group = None, None   # öğrenilmiş kaldırma: salınım başında seçilen
        self._steps = 0
        self._prev_action = [0.0] * self.n_actions
        self._vel = VelocityFilter(self.dt, self.task.vel_filter_s)
        self._next_push = math.inf
        self.dynamics: dict[str, float] = {}

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        if self.terrain_levels is not None and self._steps > 0:
            self._update_level()   # biten bölümün sonucuna göre
        self._randomize_dynamics()
        state = self.sim.reset()
        for _ in range(int(round(self.task.settle_s / self.dt))):  # duruşa yerleş
            state = self.sim.step(self.default)
        self._commanded = list(state.joint_target)
        self._delayed = deque([self._commanded] * self._delay_steps)
        self._command = sample_command(self.task, self.np_random)
        if options and "command" in options:
            self._command = tuple(float(v) for v in options["command"])
        self._phase = 0.0
        self._lift, self._lift_group = None, None
        self._steps = 0
        self._prev_action = [0.0] * self.n_actions
        self._vel.reset()
        self._next_push = self._push_gap()
        self._state = state
        self._spawn_xy = (state.base_pos[0], state.base_pos[1])
        self._max_disp, self._fell = 0.0, False
        info = {"command": self._command, "dynamics": dict(self.dynamics)}
        if self.terrain_levels is not None:
            info["terrain_level"] = self.level
        if self.lift_reflex is not None:
            self.lift_reflex.reset()
        if self.range_sensors is not None:
            info["ranges_m"] = self._read_ranges()
        return self._obs(), info

    def step(self, action):
        action = [float(a) for a in np.asarray(action, dtype=np.float64).reshape(-1)]
        joints, lift = action[:ACTION_SIZE], None
        if len(action) != self.n_actions:
            raise ValueError(f"{self.n_actions} eylem bekleniyordu, {len(action)} geldi")
        if self.task.lift_action is not None or self.lift_reflex is not None:
            # Kaldırma yalnız salınımın ilk adımında seçilir, salınım boyunca sabit:
            # her adım seçilse keşif gürültüsü bir salınım içinde ortalanır ve yüksek
            # bir salınım hiç denenmez (v18, v19); yörünge de titrer. Refleks
            # görüyorsa onun, yoksa son eylemin (öğrenilmiş kaldırma), o da yoksa
            # tabanın kaldırması (hexapod_policy.PolicyController ile aynı).
            group = self.base.swing_group(self._phase)
            if group != self._lift_group:
                if (self.lift_reflex is not None and self._reflex_lift is not None
                        and covers(self.range_sensors, self._command[0], self._command[1])):
                    self._lift = self._reflex_lift
                elif self.task.lift_action is not None:
                    self._lift = lift_from_action(action[ACTION_SIZE], self.task.lift_action)
                else:
                    self._lift = None
                self._lift_group = group
            lift = self._lift
        if self.base is None:
            targets = action_to_targets(action, self.default, self.task.action_scale, self.limits)
        else:   # artık eylem: tripod(saat, komut[, kaldırma]) + düzeltme
            targets = action_to_targets(joints,
                                        self.base.targets(self._phase, self._command, lift),
                                        self.task.residual_scale, self.limits)
        if self._steps >= self._next_push:
            self._push()
        self._commanded = list(targets)
        if self._delay_steps:   # kontrol adımından uzun gecikme: d adım önceki komut
            self._delayed.append(self._commanded)
            targets = self._delayed.popleft()
        state = self.sim.step(targets)
        self._steps += 1
        self._phase = (self._phase + self.task.gait_hz * self.dt) % 1.0
        fell = fallen(state, self.task)
        self._fell = self._fell or fell
        self._max_disp = max(self._max_disp, math.hypot(state.base_pos[0] - self._spawn_xy[0],
                                                        state.base_pos[1] - self._spawn_xy[1]))
        r, terms = reward(state, action, self._prev_action, self._command, self.task, fell,
                          phase=self._phase, groups=self.groups,
                          tracked=self._vel.update(state))
        self._prev_action = action
        self._state = state
        info = {"reward_terms": terms, "command": self._command,
                "base_pos": state.base_pos, "foot_contact": state.foot_contact,
                "lift_mm": lift if lift is not None else
                (self.task.lift_mm if self.base is not None else None)}
        if self.range_sensors is not None:
            info["ranges_m"] = self._read_ranges()
        return self._obs(), float(r), fell, self._steps >= self.max_steps, info

    def close(self) -> None:
        """Gazebo dünyasını bırak (HexapodSim.close)."""
        self.sim.close()
        super().close()

    def _read_ranges(self) -> list[float]:
        """Mesafe sensörlerinin ölçümü (ideal ışın, rangefinder.read; istenirse
        gürültü ve düşen okuma); refleks varsa onu da günceller."""
        s = self._state
        ranges = rangefinder.read(self.range_sensors, s.base_pos, s.base_quat, self.sim.height)
        if self.range_noise > 0 or self.range_drop > 0:
            u = self.np_random
            ranges = [sensor.max_m if u.uniform() < self.range_drop
                      else (d * (1.0 + u.normal(0.0, self.range_noise)) if d < sensor.max_m else d)
                      for sensor, d in zip(self.range_sensors, ranges)]
        self.ranges = ranges
        if self.lift_reflex is not None:
            g = s.gravity_in_base()
            stand = self.task.stand_height_mm / 1000.0
            self._reflex_lift = self.lift_reflex.update(
                self._steps * self.dt,
                [obstacle_height(sensor, d, g, stand) for sensor, d in zip(self.range_sensors, ranges)])
        return list(self.ranges)

    # -- müfredat ----------------------------------------------------------------

    def _update_level(self) -> None:
        """Biten bölüme göre zemin seviyesi: geçtiyse zorlaş, takıldıysa kolaylaş."""
        n = len(self.terrain_levels)
        speed = math.hypot(self._command[0], self._command[1])
        wanted = min(speed * self._steps * self.dt, CURRICULUM_PROMOTE_M)
        level = self.level
        if not self._fell and self._max_disp >= CURRICULUM_PROMOTE_M:
            level = level + 1 if level + 1 < n else int(self.np_random.integers(0, n))
        elif self._fell or (speed >= CURRICULUM_MIN_SPEED
                            and self._max_disp < CURRICULUM_DEMOTE_FRAC * wanted):
            level = max(0, level - 1)
        if level != self.level:
            self.set_level(level)

    def set_level(self, level: int) -> None:
        """Zemini müfredatın bu seviyesine çevir: Gazebo dünyası yeniden kurulur
        (aynı model, kütle ve kalibrasyon ofseti; dinamik reset'te yeniden çekilir)."""
        if self.terrain_levels is None or not 0 <= level < len(self.terrain_levels):
            raise ValueError(f"geçersiz müfredat seviyesi: {level}")
        sdf, height = self.terrain_levels[level]()
        workdir, offset = self.sim.workdir, list(self.sim.joint_offset)
        self.sim.close()
        self.sim = HexapodSim(self._model, physics_step=self._physics_step, workdir=workdir,
                              terrain_sdf=sdf, terrain_height=height)
        self.sim.joint_offset = offset
        self.level = level

    def _obs(self) -> np.ndarray:
        state = self._state
        if self._delay_steps:   # denetleyici servoya ulaşanı değil kendi son komutunu bilir
            state = replace(state, joint_target=tuple(self._commanded))
        obs = np.asarray(observation(state, self._command, self._phase, self.default,
                                     self.task.action_scale), dtype=np.float32)
        if self._imu_tilt is not None:   # eğik IMU: gövde vektörleri IMU çerçevesinde
            obs[0:3] = self._imu_tilt @ obs[0:3]
            obs[3:6] = self._imu_tilt @ obs[3:6]
        r = self.task.randomization
        if r is not None:   # IMU gürültüsü: yerçekimi yönü [0:3], jiroskop [3:6]
            obs[0:3] += self.np_random.normal(0.0, r.gravity_noise, 3).astype(np.float32)
            obs[3:6] += self.np_random.normal(0.0, r.gyro_noise, 3).astype(np.float32)
        return obs

    # -- alan rastgeleleştirme -------------------------------------------------

    def _randomize_dynamics(self) -> None:
        r, pert = self.task.randomization, self.perturbation
        if r is None:
            strength, stiffness, latency_ms = 1.0, 1.0, 0.0
        else:
            u = self.np_random.uniform
            strength, stiffness = float(u(*r.servo_strength)), float(u(*r.servo_stiffness))
            latency_ms = float(u(*r.latency_ms))
        strength *= pert.servo_strength
        self.sim.set_servo(strength, stiffness)
        # gecikme fizik adımına yuvarlanır; kontrol adımından uzunsa tam adımları
        # env kuyruğu, kalanı sim.latency_steps taşır
        n = int(round((latency_ms + pert.delay_ms) / 1000.0 / self.sim.physics_step))
        self._delay_steps, self.sim.latency_steps = divmod(n, self.sim.steps_per_action)
        self.dynamics = {"body_mass_scale": self.body_mass_scale}
        if r is not None or pert != Perturbation():
            self.dynamics.update({"servo_strength": strength, "servo_stiffness": stiffness,
                                  "latency_ms": n * self.sim.physics_step * 1000.0})

    def _push_gap(self) -> float:
        """Bir sonraki itmeye kadar kontrol adımı (rastgeleleştirme kapalıysa hiç)."""
        r = self.task.randomization
        if r is None or r.push_force_n[1] <= 0:
            return math.inf
        return self._steps + round(float(self.np_random.uniform(*r.push_every_s)) / self.dt)

    def _push(self) -> None:
        r = self.task.randomization
        force = float(self.np_random.uniform(*r.push_force_n))
        angle = float(self.np_random.uniform(0.0, 2.0 * math.pi))
        self.sim.push((force * math.cos(angle), force * math.sin(angle), 0.0), r.push_s)
        self._next_push = self._push_gap()


def make_env(rank: int, task: TaskConfig | None = None, physics_step: float = 0.002,
             terrain_sdf: str = "", terrain_height: TerrainHeight | None = None,
             body_mass_scale: float = 1.0, terrain_levels: Sequence[TerrainMaker] | None = None,
             **sensor_kwargs):
    """SubprocVecEnv için fabrika; her süreç kendi Gazebo dünyasını kurar.
    sensor_kwargs: range_sensors, lift_reflex, range_noise, range_drop."""
    def _init():
        env = HexapodEnv(task=task, physics_step=physics_step, terrain_sdf=terrain_sdf,
                         terrain_height=terrain_height, body_mass_scale=body_mass_scale,
                         terrain_levels=terrain_levels, **sensor_kwargs)
        env.reset(seed=rank)
        return env
    return _init
