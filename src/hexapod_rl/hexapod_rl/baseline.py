"""Klasik tripod'u (Samet, hexapod_gait — GOREVLER.md S2) RL ortamında politika gibi koşturur.

G6'nın "bitti" şartı tripod'un ödülünün aynı ortamda ölçülmesi; G7'ninki
politikanın onu geçmesi. evaluate.py modeli predict() ile çağırdığı için bu
sınıf aynı arayüzü verir:

    python -m hexapod_rl.evaluate tripod --vx 0.1

Gözlemden yalnızca hız komutunu okur; yürüyüş açık döngü (TripodGait geri
besleme kullanmaz). Adım saati ortamınkiyle aynı frekansta ve aynı
sözleşmede (ilk yarıda groups[0] havada), ikisi de bölüm başında sıfırdan
başlar; bu yüzden bölüm başında reset() çağrılmalı (evaluate çağırır).
"""

from __future__ import annotations

import math

from hexapod_description.interface import COMMAND_RATE_HZ
from hexapod_driver.config import RobotConfig
from hexapod_gait import GaitParams, TripodGait
from hexapod_gazebo.pose import standing_pose
from hexapod_kinematics import HexapodKinematics

from .task import OBS_SIZE, TaskConfig

_COMMAND = slice(OBS_SIZE - 5, OBS_SIZE - 2)   # gözlemde vx, vy, wz (task.observation)


class TripodPolicy:
    def __init__(self, task: TaskConfig | None = None, config_path=None) -> None:
        self.task = task or TaskConfig()
        kin = HexapodKinematics.from_config(RobotConfig.load(config_path))
        pose = standing_pose(kin, self.task.stand_reach_mm, self.task.stand_height_mm)
        self.default = [math.radians(v) for leg in sorted(pose)
                        for v in pose[leg].as_dict().values()]
        self.gait = TripodGait(kin, GaitParams(cycle_hz=self.task.gait_hz,
                                               stance_reach_mm=self.task.stand_reach_mm,
                                               stance_height_mm=self.task.stand_height_mm))
        self.dt = 1.0 / COMMAND_RATE_HZ

    def reset(self) -> None:
        self.gait.reset()

    def predict(self, obs, deterministic: bool = True):
        """SB3 modeliyle aynı imza: (eylem, durum). Eylem kırpılmaz; ortam kırpar."""
        vx, vy, wz = (float(v) for v in obs[_COMMAND])
        angles = self.gait.step(vx, vy, wz, self.dt)
        targets = [math.radians(v) for leg in sorted(angles)
                   for v in angles[leg].as_dict().values()]
        action = [(t - d) / self.task.action_scale for t, d in zip(targets, self.default)]
        return action, None
