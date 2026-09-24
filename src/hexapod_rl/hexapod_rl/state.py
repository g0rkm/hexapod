"""Simülasyon durumu (saf Python; gz.sim gerektirmez)."""

from __future__ import annotations

from dataclasses import dataclass

from .math3d import Quat, Vec3, rotate_inverse


@dataclass(frozen=True)
class SimState:
    """Bir kontrol adımının sonundaki durum. Dizi sırası interface.joint_names()."""

    time: float                 # s, simülasyon zamanı
    joint_pos: tuple[float, ...]     # rad, ÖLÇÜLEN (yalnız simülasyonda var)
    joint_vel: tuple[float, ...]     # rad/s
    joint_target: tuple[float, ...]  # rad, son komut (gerçek robotta /joint_states budur)
    joint_effort: tuple[float, ...]  # N·m, servonun uyguladığı tork (yalnız sim)
    base_pos: Vec3              # m, dünya
    base_quat: Quat             # dünya <- gövde
    base_lin_vel: Vec3          # m/s, dünya
    base_ang_vel: Vec3          # rad/s, dünya
    foot_pos: tuple[Vec3, ...]  # m, dünya, ayak küresinin alt ucu (6)
    foot_contact: tuple[bool, ...]  # yalnız simülasyon; politika gözlemine girmemeli

    def gravity_in_base(self) -> Vec3:
        """Yerçekimi yönünün gövde çerçevesindeki birim vektörü (IMU'dan çıkarılabilir)."""
        return rotate_inverse(self.base_quat, (0.0, 0.0, -1.0))

    def ang_vel_in_base(self) -> Vec3:
        """Gövde çerçevesinde açısal hız (IMU jiroskobunun ölçtüğü)."""
        return rotate_inverse(self.base_quat, self.base_ang_vel)

    def lin_vel_in_base(self) -> Vec3:
        return rotate_inverse(self.base_quat, self.base_lin_vel)
