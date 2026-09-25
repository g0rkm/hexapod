"""hexapod_hardware — gerçek robot sürücü düğümü (GOREVLER.md S4).

controller.py  ROS'suz çekirdek: komut dizisi -> ServoBus.set_angles (hep ya da hiç),
               son komutu joint_state olarak tutar. DryRunBackend ile robotsuz test edilir.
node.py        ince rclpy kabuğu: COMMAND_TOPIC'i dinler, /joint_states yayınlar.

Kullanım: ros2 run hexapod_hardware driver --ros-args -p dry_run:=true
"""

from .controller import DriverController, Result

__all__ = ["DriverController", "Result"]

__version__ = "0.1.0"
