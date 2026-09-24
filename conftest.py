"""pytest'in paketleri ROS 2 kurulumu olmadan bulabilmesi için."""
import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent / "src"
for _pkg in ("hexapod_driver", "hexapod_kinematics"):
    sys.path.insert(0, str(_SRC / _pkg))
