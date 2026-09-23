"""pytest'in hexapod_driver'i ROS kurulumu olmadan bulabilmesi icin."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "src" / "hexapod_driver"))
