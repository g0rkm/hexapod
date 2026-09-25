"""pytest'in paketleri ROS 2 kurulumu olmadan bulabilmesi için."""
import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent / "src"
for _pkg in ("hexapod_driver", "hexapod_kinematics", "hexapod_gait", "hexapod_description",
             "hexapod_gazebo", "hexapod_rl", "hexapod_teleop", "hexapod_hardware"):
    sys.path.insert(0, str(_SRC / _pkg))
