"""Süreç içi Gazebo: RL eğitimi için ROS'suz, hızlı simülasyon (GOREVLER.md G6).

Neden ROS'suz
-------------
ROS'lu simülasyon (hexapod_gazebo/sim.launch.py) sınırsız modda bile gerçek
zamanın ~1.3 katı koşuyor; PPO'nun milyonlarca adımı için yavaş. Aynı Gazebo'yu
Python'dan süreç içinde adımlayınca (gz.sim TestFixture) ölçülenler:

    fizik adımı   tek süreç   8 paralel süreç (toplam)
    2 ms          4.5x        ~21x
    4 ms          7.7x        -

(Ubuntu 26.04 / WSL2, i5-10300H 8 iş parçacığı, Gazebo 10.5; ayrıntı
PROJE_DEVIR.md.) Fizik motoru, robot modeli (aynı URDF) ve servo modeli
ROS'lu simülasyonla AYNI; yalnızca ROS katmanı yok. Eğitilen politika sonra
ROS arayüzü (docs/ARAYUZ.md) üzerinden çalışır (G8).

Servo modeli
------------
gz_ros2_control'ün konum komutu ile birebir aynı: kontrol döngüsü hızında
(robot.yaml simulation.control.update_rate_hz)

    hız = kazanç x (hedef - konum) x döngü hızı,   |hız| <= hız limiti

hesaplanıp ekleme hız komutu olarak verilir; kazanç = RobotModel.position_gain().
Hedefler ros2_control'de olduğu gibi eklem limitlerine kırpılır. Hız komutu
Gazebo'da bir sonraki komuta kadar kalıcı (ölçüldü: yalnız döngüde verildiğinde
robot yine duruşunu koruyor).

Bu modül ROS'a bağımlı değil ama gz.sim Python bağlarına bağımlı; onlar ROS 2
Lyrical kurulumuyla geliyor (source /opt/ros/lyrical/setup.bash).
"""

from __future__ import annotations

import math
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import gz.math  # noqa: F401  Pose3d/Vector3d tiplerini pybind11 tanısın diye (yoksa çöker)
from gz.sim import Joint, Link, Model, TestFixture, World, world_entity

from hexapod_description.interface import COMMAND_RATE_HZ, joint_names
from hexapod_description.model import RobotModel
from hexapod_description.urdf import build_urdf, link_name

from .math3d import Quat, Vec3, rotate_inverse

#: Doğarken sıfır duruşunda ayakların yerden yüksekliği.
SPAWN_CLEARANCE_M = 0.01
#: Ayak küresinin alt ucu yere bu kadar yakınsa "temas" sayılır.
CONTACT_TOLERANCE_M = 0.002


@dataclass(frozen=True)
class SimState:
    """Bir kontrol adımının sonundaki durum. Dizi sırası interface.joint_names()."""

    time: float                 # s, simülasyon zamanı
    joint_pos: tuple[float, ...]     # rad, ÖLÇÜLEN (yalnız simülasyonda var)
    joint_vel: tuple[float, ...]     # rad/s
    joint_target: tuple[float, ...]  # rad, son komut (gerçek robotta /joint_states budur)
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


class HexapodSim:
    """Tek bir süreç içi Gazebo dünyası ve içinde robot.

    Kullanım:
        sim = HexapodSim(RobotModel.from_config(RobotConfig.load()))
        state = sim.reset()
        state = sim.step(targets)   # 18 hedef, rad; bir kontrol adımı (1/50 s)
    """

    def __init__(self, model: RobotModel, physics_step: float = 0.002,
                 action_rate: float = COMMAND_RATE_HZ, terrain_sdf: str = "",
                 workdir: Path | None = None) -> None:
        self.model = model
        self.physics_step = physics_step
        physics_rate = 1.0 / physics_step
        self.servo_every = _whole(physics_rate / model.control_rate, "fizik hızı / servo hızı")
        self.steps_per_action = _whole(physics_rate / action_rate, "fizik hızı / eylem hızı")
        self.dt = self.steps_per_action * physics_step

        self.names = joint_names(model.mounts)
        self._limits = [model.limits[(int(n[3]), n.split("_")[1])] for n in self.names]
        self._gain = model.position_gain()
        self._rate = model.control_rate
        self._vmax = model.velocity
        self.initial_targets = tuple(0.0 for _ in self.names)  # sıfır duruşu
        self._targets = list(self.initial_targets)
        self._cmd = [0.0] * len(self.names)

        self._dir = Path(workdir) if workdir else Path(tempfile.mkdtemp(prefix="hexapod_rl_"))
        self._dir.mkdir(parents=True, exist_ok=True)
        world = self._write_world(terrain_sdf)

        self._joints: list[Joint] | None = None
        self._base: Link | None = None
        self._tibias: list[Link] = []
        self._iter = 0
        self._left = 0
        self._awaiting_reset = False
        self._state: SimState | None = None

        self._fixture = TestFixture(str(world))
        self._fixture.on_pre_update(self._pre)
        self._fixture.on_post_update(self._post)
        self._fixture.on_reset(self._on_reset)
        self._fixture.finalize()
        self._server = self._fixture.server()

    # -- genel -----------------------------------------------------------------

    def reset(self) -> SimState:
        """Dünyayı ilk hâline döndür (robot sıfır duruşunda, yerin 1 cm üstünde).

        gz.sim'de sıfırlama isteği bir adım gecikmeyle işlenir ve sıfırlamanın
        kendisi Server.run()'ın adım sayısından birini yer (ölçüldü: reset_all()
        + run(3) = eski durumda 1 adım, sıfırlama, yeni durumda yalnız 1 adım).
        Bu yüzden sıfırlama gerçekleşene kadar tek adımlık çağrılar yapılır ve
        o sırada geri çağrılar iş yapmaz; sonra temiz bir kontrol adımı koşar.
        Böylece reset() sonrası zaman her zaman tam bir kontrol adımıdır (dt).
        """
        self._server.reset_all()
        self._awaiting_reset = True
        for _ in range(5):
            self._server.run(True, 1, False)
            if not self._awaiting_reset:
                break
        else:
            raise RuntimeError("Gazebo sıfırlaması 5 adımda gerçekleşmedi")
        self._targets = list(self.initial_targets)
        self._cmd = [0.0] * len(self.names)
        return self._advance(self.steps_per_action)

    def step(self, targets: Sequence[float]) -> SimState:
        """18 eklem hedefi (rad, interface sırası) ver, bir kontrol adımı ilerle."""
        if len(targets) != len(self.names):
            raise ValueError(f"{len(self.names)} hedef bekleniyordu, {len(targets)} geldi")
        self._targets = [min(max(t, lim.lower), lim.upper)
                         for t, lim in zip(targets, self._limits)]
        return self._advance(self.steps_per_action)

    @property
    def workdir(self) -> Path:
        return self._dir

    # -- Gazebo geri çağrıları ---------------------------------------------------

    def _advance(self, iterations: int) -> SimState:
        self._state = None
        self._left = iterations
        self._server.run(True, iterations, False)
        if self._state is None:
            raise RuntimeError("simülasyon adımı durum üretmedi")
        return self._state

    def _pre(self, info, ecm) -> None:
        if info.paused or self._awaiting_reset:
            return
        if self._joints is None:
            self._bind(ecm)
        if self._iter % self.servo_every == 0:
            for i, joint in enumerate(self._joints):
                pos = joint.position(ecm)
                if pos:
                    v = self._gain * (self._targets[i] - pos[0]) * self._rate
                    self._cmd[i] = max(-self._vmax, min(self._vmax, v))
                joint.set_velocity(ecm, [self._cmd[i]])
        self._iter += 1

    def _post(self, info, ecm) -> None:
        if info.paused or self._awaiting_reset or self._joints is None:
            return
        self._left -= 1
        if self._left == 0:
            self._state = self._read_state(info, ecm)

    def _on_reset(self, info, ecm) -> None:
        # Sıfırlama bileşenleri silebilir; varlıkları ve kontrolleri yeniden kur.
        self._joints = None
        self._iter = 0
        self._awaiting_reset = False

    def _bind(self, ecm) -> None:
        robot = Model(World(world_entity(ecm)).model_by_name(ecm, self.model.name))
        self._joints = [Joint(robot.joint_by_name(ecm, n)) for n in self.names]
        for joint in self._joints:
            joint.enable_position_check(ecm)
            joint.enable_velocity_check(ecm)
        self._base = Link(robot.link_by_name(ecm, "base_link"))
        self._base.enable_velocity_checks(ecm)
        self._tibias = [Link(robot.link_by_name(ecm, link_name(leg, "tibia")))
                        for leg in sorted(self.model.mounts)]

    def _read_state(self, info, ecm) -> SimState:
        pos = [(j.position(ecm) or [math.nan])[0] for j in self._joints]
        vel = [(j.velocity(ecm) or [math.nan])[0] for j in self._joints]
        pose = self._base.world_pose(ecm)
        lin = self._base.world_linear_velocity(ecm)
        ang = self._base.world_angular_velocity(ecm)
        r = self.model.foot_radius
        feet, contact = [], []
        for tibia in self._tibias:
            tp = tibia.world_pose(ecm)
            c = tp.rot().rotate_vector(gz.math.Vector3d(0.0, 0.0, -(self.model.tibia - r)))
            bottom = (tp.pos().x() + c.x(), tp.pos().y() + c.y(), tp.pos().z() + c.z() - r)
            feet.append(bottom)
            contact.append(bottom[2] <= CONTACT_TOLERANCE_M)
        q = pose.rot()
        return SimState(
            time=_seconds(info.sim_time),
            joint_pos=tuple(pos), joint_vel=tuple(vel), joint_target=tuple(self._targets),
            base_pos=(pose.pos().x(), pose.pos().y(), pose.pos().z()),
            base_quat=(q.w(), q.x(), q.y(), q.z()),
            base_lin_vel=_vec(lin), base_ang_vel=_vec(ang),
            foot_pos=tuple(feet), foot_contact=tuple(contact),
        )

    # -- dünya ------------------------------------------------------------------

    def _write_world(self, terrain_sdf: str) -> Path:
        urdf = self._dir / "robot.urdf"
        urdf.write_text(build_urdf(self.model), encoding="utf-8")
        feet_z = min(m.z for m in self.model.mounts.values()) - self.model.tibia
        spawn_z = -feet_z + SPAWN_CLEARANCE_M
        ground = terrain_sdf or _FLAT_GROUND
        world = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="rl">
    <physics name="rl" type="ignored">
      <max_step_size>{self.physics_step}</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    {ground}
    <include>
      <uri>{urdf.resolve().as_uri()}</uri>
      <name>{self.model.name}</name>
      <pose>0 0 {spawn_z:.4f} 0 0 0</pose>
    </include>
  </world>
</sdf>
"""
        path = self._dir / "world.sdf"
        path.write_text(world, encoding="utf-8")
        return path


_FLAT_GROUND = """<model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
      </link>
    </model>"""


# ---------------------------------------------------------------------------


def _vec(v) -> Vec3:
    if v is None:
        return (math.nan, math.nan, math.nan)
    return (v.x(), v.y(), v.z())


def _seconds(sim_time) -> float:
    """UpdateInfo.sim_time: datetime.timedelta ya da saniye."""
    return sim_time.total_seconds() if hasattr(sim_time, "total_seconds") else float(sim_time)


def _whole(ratio: float, what: str) -> int:
    n = round(ratio)
    if n < 1 or abs(ratio - n) > 1e-9:
        raise ValueError(f"{what} tam sayı olmalı, {ratio:g} çıktı")
    return n


__all__ = ["HexapodSim", "SimState"]
