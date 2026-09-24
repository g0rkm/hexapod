"""hexapod_kinematics — ters/düz kinematik ve gövde pozu.

Katman sırası (proje brifi §4):
    Gait motoru
    Ters kinematik (IK) + gövde pozu   <- burası
    Servo sürücü katmanı (hexapod_driver)

Donanıma dokunmaz. Çıktısı eklem açılarıdır (derece); bunları servoya
ServoBus.set_angle() taşır.
"""

from .body import NEUTRAL, BodyPose, HexapodKinematics, LegMount, wrap_deg
from .errors import ReachError
from .leg import ZERO, JointAngles, LegGeometry, forward, inverse

__all__ = [
    "BodyPose",
    "HexapodKinematics",
    "JointAngles",
    "LegGeometry",
    "LegMount",
    "NEUTRAL",
    "ReachError",
    "ZERO",
    "forward",
    "inverse",
    "wrap_deg",
]

__version__ = "0.1.0"
