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
"""

from __future__ import annotations

import math
from pathlib import Path

import gymnasium as gym
import numpy as np

from hexapod_description.model import RobotModel
from hexapod_driver.config import RobotConfig
from hexapod_gazebo.pose import standing_pose
from hexapod_kinematics import HexapodKinematics
from hexapod_policy.tripod import PhaseTripod

from .sim import HexapodSim, TerrainHeight
from .task import (
    ACTION_SIZE,
    OBS_SIZE,
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


class HexapodEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, task: TaskConfig | None = None, physics_step: float = 0.002,
                 config_path: str | Path | None = None, workdir: Path | None = None,
                 terrain_sdf: str = "", terrain_height: TerrainHeight | None = None,
                 body_mass_scale: float = 1.0) -> None:
        """terrain_sdf: düz zeminin yerine geçen statik <model> SDF parçası (S5'in
        zemin üreteci; boşsa düz zemin); terrain_height(x, y): aynı zeminin üst
        yüzeyinin z'si, m (ikisi birlikte). Kısıtlar HexapodSim açıklamasında.
        body_mass_scale: gövde kütlesi (ve ataleti) bu çarpanla; bu ortamın
        ömrü boyunca sabit (task.body_mass_scales)."""
        super().__init__()
        self.task = task or TaskConfig()
        config = RobotConfig.load(config_path)
        model = RobotModel.from_config(config).with_body_mass_scale(body_mass_scale)
        self.body_mass_scale = body_mass_scale
        self.sim = HexapodSim(model, physics_step=physics_step, workdir=workdir,
                              terrain_sdf=terrain_sdf, terrain_height=terrain_height)
        self.dt = self.sim.dt

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
        self._randomize_dynamics()
        state = self.sim.reset()
        for _ in range(int(round(self.task.settle_s / self.dt))):  # duruşa yerleş
            state = self.sim.step(self.default)
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
        return self._obs(), {"command": self._command, "dynamics": dict(self.dynamics)}

    def step(self, action):
        action = [float(a) for a in np.asarray(action, dtype=np.float64).reshape(-1)]
        joints, lift = action[:ACTION_SIZE], None
        if self.task.lift_action is not None:   # son eylem: ayak kaldırma
            if len(action) != self.n_actions:
                raise ValueError(f"{self.n_actions} eylem bekleniyordu, {len(action)} geldi")
            # Yalnız salınımın ilk adımında seçilir, salınım boyunca sabit: her adım
            # seçilse keşif gürültüsü bir salınım içinde ortalanır ve yüksek bir
            # salınım hiç denenmez (v18, v19); yörünge de titrer.
            group = self.base.swing_group(self._phase)
            if group != self._lift_group:
                self._lift = lift_from_action(action[ACTION_SIZE], self.task.lift_action)
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
        state = self.sim.step(targets)
        self._steps += 1
        self._phase = (self._phase + self.task.gait_hz * self.dt) % 1.0
        fell = fallen(state, self.task)
        r, terms = reward(state, action, self._prev_action, self._command, self.task, fell,
                          phase=self._phase, groups=self.groups,
                          tracked=self._vel.update(state))
        self._prev_action = action
        self._state = state
        info = {"reward_terms": terms, "command": self._command,
                "base_pos": state.base_pos, "foot_contact": state.foot_contact,
                "lift_mm": lift if lift is not None else
                (self.task.lift_mm if self.base is not None else None)}
        return self._obs(), float(r), fell, self._steps >= self.max_steps, info

    def close(self) -> None:
        """Gazebo dünyasını bırak (HexapodSim.close)."""
        self.sim.close()
        super().close()

    def _obs(self) -> np.ndarray:
        obs = np.asarray(observation(self._state, self._command, self._phase, self.default,
                                     self.task.action_scale), dtype=np.float32)
        r = self.task.randomization
        if r is not None:   # IMU gürültüsü: yerçekimi yönü [0:3], jiroskop [3:6]
            obs[0:3] += self.np_random.normal(0.0, r.gravity_noise, 3).astype(np.float32)
            obs[3:6] += self.np_random.normal(0.0, r.gyro_noise, 3).astype(np.float32)
        return obs

    # -- alan rastgeleleştirme -------------------------------------------------

    def _randomize_dynamics(self) -> None:
        r = self.task.randomization
        if r is None:
            self.sim.set_servo(1.0, 1.0)
            self.sim.latency_steps = 0
            self.dynamics = {"body_mass_scale": self.body_mass_scale}
            return
        u = self.np_random.uniform
        strength, stiffness = float(u(*r.servo_strength)), float(u(*r.servo_stiffness))
        self.sim.set_servo(strength, stiffness)
        latency = int(round(u(*r.latency_ms) / 1000.0 / self.sim.physics_step))
        self.sim.latency_steps = min(latency, self.sim.steps_per_action - 1)
        self.dynamics = {"servo_strength": strength, "servo_stiffness": stiffness,
                         "latency_ms": self.sim.latency_steps * self.sim.physics_step * 1000.0,
                         "body_mass_scale": self.body_mass_scale}

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
             body_mass_scale: float = 1.0):
    """SubprocVecEnv için fabrika; her süreç kendi Gazebo dünyasını kurar."""
    def _init():
        env = HexapodEnv(task=task, physics_step=physics_step, terrain_sdf=terrain_sdf,
                         terrain_height=terrain_height, body_mass_scale=body_mass_scale)
        env.reset(seed=rank)
        return env
    return _init
