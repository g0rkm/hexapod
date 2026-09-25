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

from .sim import HexapodSim
from .task import (
    ACTION_SIZE,
    OBS_SIZE,
    TaskConfig,
    VelocityFilter,
    action_to_targets,
    fallen,
    observation,
    reward,
    tripod_groups,
)


class HexapodEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, task: TaskConfig | None = None, physics_step: float = 0.002,
                 config_path: str | Path | None = None, workdir: Path | None = None) -> None:
        super().__init__()
        self.task = task or TaskConfig()
        config = RobotConfig.load(config_path)
        model = RobotModel.from_config(config)
        self.sim = HexapodSim(model, physics_step=physics_step, workdir=workdir)
        self.dt = self.sim.dt

        pose = standing_pose(HexapodKinematics.from_config(config),
                             self.task.stand_reach_mm, self.task.stand_height_mm)
        self.default = [math.radians(v) for leg in sorted(pose)
                        for v in pose[leg].as_dict().values()]
        self.groups = tripod_groups({leg: m.yaw for leg, m in model.mounts.items()})
        self.limits = [(model.limits[(int(n[3]), n.split("_")[1])].lower,
                        model.limits[(int(n[3]), n.split("_")[1])].upper)
                       for n in self.sim.names]

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (OBS_SIZE,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (ACTION_SIZE,), np.float32)
        self.max_steps = int(round(self.task.episode_s / self.dt))

        self._state = None
        self._command = (0.0, 0.0, 0.0)
        self._phase = 0.0
        self._steps = 0
        self._prev_action = [0.0] * ACTION_SIZE
        self._vel = VelocityFilter(self.dt, self.task.vel_filter_s)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        state = self.sim.reset()
        for _ in range(int(round(self.task.settle_s / self.dt))):  # duruşa yerleş
            state = self.sim.step(self.default)
        t = self.task
        self._command = tuple(float(self.np_random.uniform(*r))
                              for r in (t.vx_range, t.vy_range, t.wz_range))
        if options and "command" in options:
            self._command = tuple(float(v) for v in options["command"])
        self._phase = 0.0
        self._steps = 0
        self._prev_action = [0.0] * ACTION_SIZE
        self._vel.reset()
        self._state = state
        return self._obs(), {"command": self._command}

    def step(self, action):
        action = [float(a) for a in np.asarray(action, dtype=np.float64).reshape(-1)]
        targets = action_to_targets(action, self.default, self.task.action_scale, self.limits)
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
                "base_pos": state.base_pos, "foot_contact": state.foot_contact}
        return self._obs(), float(r), fell, self._steps >= self.max_steps, info

    def _obs(self) -> np.ndarray:
        return np.asarray(observation(self._state, self._command, self._phase, self.default,
                                      self.task.action_scale), dtype=np.float32)


def make_env(rank: int, task: TaskConfig | None = None, physics_step: float = 0.002):
    """SubprocVecEnv için fabrika; her süreç kendi Gazebo dünyasını kurar."""
    def _init():
        env = HexapodEnv(task=task, physics_step=physics_step)
        env.reset(seed=rank)
        return env
    return _init
