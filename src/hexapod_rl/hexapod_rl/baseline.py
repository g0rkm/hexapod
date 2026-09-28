"""Klasik tripod'u (Samet, hexapod_gait — GOREVLER.md S2) RL ortamında politika gibi koşturur.

G6'nın "bitti" şartı tripod'un ödülünün aynı ortamda ölçülmesi; G7'ninki
politikanın onu geçmesi. evaluate.py modeli predict() ile çağırdığı için bu
sınıf aynı arayüzü verir:

    python -m hexapod_rl.evaluate tripod --vx 0.1

Gözlemden yalnızca hız komutunu okur; yürüyüş açık döngü (TripodGait geri
besleme kullanmaz). Adım saati ortamınkiyle aynı frekansta ve aynı
sözleşmede (ilk yarıda groups[0] havada), ikisi de bölüm başında sıfırdan
başlar; bu yüzden bölüm başında reset() çağrılmalı (evaluate çağırır).

Tripod ortamda kendi görev ayarıyla koşmalı (`policy.task`): eylem ölçeği
ACTION_SCALE. Eğitimdeki 0.5 rad'lık ölçek 30 mm'den yüksek adımda femura
yetmiyor; ortam eylemi [-1, 1]'e kırptığı için "tripod:50" 2026-09-28'e kadar
aslında ~30 mm kaldırıyordu (50 mm femurda 49° ister, sınır 28.6°; ders 49).
"""

from __future__ import annotations

import math
from dataclasses import replace

from hexapod_description.interface import COMMAND_RATE_HZ
from hexapod_driver.config import RobotConfig
from hexapod_gait import GaitParams, TripodGait
from hexapod_gazebo.pose import standing_pose
from hexapod_kinematics import HexapodKinematics

from .task import OBS_SIZE, TaskConfig

_COMMAND = slice(OBS_SIZE - 5, OBS_SIZE - 2)   # gözlemde vx, vy, wz (task.observation)

#: Tripod'un eylem ölçeği, rad: eylem ±1 = duruştan ±90°, yani eklem limitlerinin
#: (geçici ±90°) tamamı; tripod'un hedefini ortam değil yalnız eklem limitleri
#: kırpar (robotta da öyle). Tripod gözlemden yalnız komutu okuduğu için ölçek
#: başka hiçbir şeyi değiştirmez (ödülün action_rate terimi ölçekle küçülür,
#: tripod'da adım başı 0.001'in altında).
ACTION_SCALE = math.pi / 2


class TripodPolicy:
    def __init__(self, task: TaskConfig | None = None, config_path=None,
                 step_height_mm: float | None = None) -> None:
        """step_height_mm: salınımda ayak kaldırma (GaitParams.step_height_mm;
        verilmezse Samet'in varsayılanı). Zeminde RL ile adil karşılaştırma
        için: politika tabanı 50 mm kaldırıyorsa tripod da öyle ölçülür.
        task: gövde duruşu, adım saati (eylem ölçeği ACTION_SCALE'e çevrilir;
        ortamda self.task kullanılmalı)."""
        self.task = replace(task or TaskConfig(), action_scale=ACTION_SCALE)
        kin = HexapodKinematics.from_config(RobotConfig.load(config_path))
        pose = standing_pose(kin, self.task.stand_reach_mm, self.task.stand_height_mm)
        self.default = [math.radians(v) for leg in sorted(pose)
                        for v in pose[leg].as_dict().values()]
        extra = {} if step_height_mm is None else {"step_height_mm": float(step_height_mm)}
        self.gait = TripodGait(kin, GaitParams(cycle_hz=self.task.gait_hz,
                                               stance_reach_mm=self.task.stand_reach_mm,
                                               stance_height_mm=self.task.stand_height_mm,
                                               **extra))
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
