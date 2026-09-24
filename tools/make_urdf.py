#!/usr/bin/env python3
"""robot.yaml -> URDF, ROS kurulu olmadan, depo kökünden.

    python tools/make_urdf.py -o hexapod.urdf
    python tools/make_urdf.py -o hexapod.urdf --meshes file

Asıl kod src/hexapod_description/hexapod_description/__main__.py'de; bu
dosya yalnızca paketleri Python yoluna ekler (ROS kurulumunda aynı iş
`ros2 run hexapod_description make_urdf` ile yapılır).
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
for _pkg in ("hexapod_driver", "hexapod_kinematics", "hexapod_description"):
    sys.path.insert(0, str(REPO_ROOT / "src" / _pkg))

from hexapod_description.__main__ import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
