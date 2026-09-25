"""hexapod_gait — tripod yürüyüş çekirdeği (GOREVLER.md S2).

Saf Python, ROS'a bağımlı değil. Girdi gövde hız komutu (vx, vy m/s; wz
rad/s), çıktı eklem açıları (hexapod_kinematics.JointAngles, derece).
ROS'a bağlama işi ayrı bir katmanda (GOREVLER.md S3): bu paket yalnızca
step() çağrıldıkça açı üretir, konu/mesaj bilmez.

Belge: docs/ARAYUZ.md — S3, buradan alınan açıları
hexapod_description.interface.to_command() ile yayınlayacak.
"""

from .tripod import GaitParams, TripodGait, tripod_groups

__all__ = ["GaitParams", "TripodGait", "tripod_groups"]

__version__ = "0.1.0"
