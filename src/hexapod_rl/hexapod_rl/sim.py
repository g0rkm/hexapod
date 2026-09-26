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
PROJE_DEVIR.md; tork modeliyle hızlar yeniden ölçüldü, aşağıda.) Fizik
motoru ve robot modeli (aynı URDF) ROS'lu simülasyonla aynı; yalnızca ROS
katmanı yok. Eğitilen politika sonra ROS arayüzü (docs/ARAYUZ.md) üzerinden
çalışır (G8).

Servo modeli: tork tabanlı
--------------------------
Her fizik adımında, hobi servosu gibi bir P denetleyici ve DC motorun
doğrusal tork-hız eğrisi:

    tork = sertlik x (hedef - konum) - sönüm x hız
    hareket yönünde en fazla  durma torku x (1 - |hız| / yüksüz hız)
    frenlerken en fazla       durma torku

(robot.yaml simulation.servo: effort_nm, velocity_rad_s,
stiffness_nm_per_rad, damping_nm_s_per_rad). Hedefler eklem limitlerine
kırpılır.

Neden hız komutlu model (gz_ros2_control'ün konum komutu) değil: o model
eklemi her adımda istenen hıza ANINDA zorluyor; bacak ivmelenirken bu, tork
sınırını boşa harcayan anlık zorlamalar demek. Ölçüldü (2026-09-25): elle
yazılmış açık döngü tripod, aynı katalog torkunda (1.08 N·m) hız modeliyle
beklenen hızın %12'siyle (ayaklar kayıyor), bu modelle %94-97'siyle yürüyor.
İlk iki PPO eğitimi (robot yerinde saydı) bu yüzden geçersiz. Hareketsiz
tripod duruşu ikisinde de 0.6 N·m'de bile çökmüyor; sorun dinamikti.
ROS'lu simülasyon (sim.launch.py) hâlâ hız modelini kullanıyor; açık iş
(PROJE_DEVIR §13).

Zemin (terrain_sdf; S5): düz zeminin yerine geçen statik bir <model> SDF
parçası. Kısıtlar (2026-09-26):
  - robot orijinde, DÜZ zemine göre doğar (ayaklar z=0'ın 1 cm üstünde):
    orijin çevresinin üst yüzü z=0'da olmalı, yoksa ayaklar zeminin içinde
    doğar;
  - ayak teması (SimState.foot_contact; ödülün ritim terimi ve
    değerlendirme bunu kullanır) geometrik: ayak küresinin alt ucu dünya
    z=0'ın 2 mm yakınında mı. Engebeli/eğimli zeminde YANLIŞ olur; S5 ile
    birlikte gerçek temasa (gz Contact) ya da zemin yüksekliğine geçilmeli.
  - sürtünme zeminin <surface><friction>'ında. Ölçüldü (gösterim tripod'u,
    0.12 m/s): düz zeminde mu 1.0 ile 0.15 arası fark yok, 0.05'te %8
    yavaşlıyor. Yani sürtünme ancak eğimle birlikte (S5) anlam kazanır.

Alan rastgeleleştirme düğmeleri (G7; bölüm başında env.py çeker):
set_servo() durma torkunu ve sertliği ölçekler (akü gerilimi, servo farkı,
TAHMİN olan Kp), latency_steps yeni hedefin servoya kaç fizik adımı sonra
ulaştığı (I2C + PCA9685 + servo tepkisi), push() gövdeye bir süre yatay
kuvvet uygular (itme, darbe).

Bu modül ROS'a bağımlı değil ama gz.sim Python bağlarına bağımlı; onlar ROS 2
Lyrical kurulumuyla geliyor (source /opt/ros/lyrical/setup.bash).
"""

from __future__ import annotations

import math
import os
import tempfile
from pathlib import Path
from typing import Sequence

import gz.math  # noqa: F401  Pose3d/Vector3d tiplerini pybind11 tanısın diye (yoksa çöker)
from gz.sim import Joint, Link, Model, TestFixture, World, world_entity

from hexapod_description.interface import COMMAND_RATE_HZ, joint_names
from hexapod_description.model import RobotModel
from hexapod_description.urdf import build_urdf, link_name

from .math3d import Vec3
from .state import SimState

#: Doğarken sıfır duruşunda ayakların yerden yüksekliği.
SPAWN_CLEARANCE_M = 0.01
#: Ayak küresinin alt ucu yere bu kadar yakınsa "temas" sayılır.
CONTACT_TOLERANCE_M = 0.002
#: Her süreç kendi gz-transport bölümünde (GZ_PARTITION + süreç kimliği).
PARTITION_PREFIX = "hexapod_rl_"


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
        self.steps_per_action = _whole(physics_rate / action_rate, "fizik hızı / eylem hızı")
        self.dt = self.steps_per_action * physics_step

        self.names = joint_names(model.mounts)
        self._limits = [model.limits[(int(n[3]), n.split("_")[1])] for n in self.names]
        self._kp = model.servo_stiffness
        self._kd = model.servo_damping
        self._tau = model.effort
        self._vmax = model.velocity
        self.initial_targets = tuple(0.0 for _ in self.names)  # sıfır duruşu
        self._targets = list(self.initial_targets)
        self._pending: list[float] | None = None   # gecikmedeki yeni hedefler
        self._effort = [0.0] * len(self.names)
        self._strength = 1.0                        # durma torku çarpanı
        self._stiffness = 1.0                       # sertlik (Kp) çarpanı
        self.latency_steps = 0                      # fizik adımı; < steps_per_action
        self._push: tuple[float, float, float] | None = None
        self._push_left = 0
        self._k = 0                                 # kontrol adımı içindeki fizik adımı

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

        # Süreç içi dünya gz-transport'ta görünmesin. Görünürse (2026-09-26'da
        # oldu): eğitim sürerken açılan ROS'lu simülasyonun robot oluşturma
        # isteği ("ros_gz_sim create") dünya listesini sorunca eğitimin "rl"
        # dünyasını buldu ve isteği oraya gönderdi; ROS'lu simde robot hiç
        # doğmadı. Ayrı bölümde ne dışarıdan istek gelir ne dışarı yayın gider.
        os.environ["GZ_PARTITION"] = f"{PARTITION_PREFIX}{os.getpid()}"
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
        self._pending = None
        self._effort = [0.0] * len(self.names)
        self._push, self._push_left = None, 0
        return self._advance(self.steps_per_action)

    def set_servo(self, strength: float = 1.0, stiffness: float = 1.0) -> None:
        """Servo modelini ölçekle: durma torku x strength, sertlik x stiffness."""
        if strength <= 0 or stiffness <= 0:
            raise ValueError(f"çarpanlar pozitif olmalı: {strength}, {stiffness}")
        self._strength, self._stiffness = strength, stiffness

    def push(self, force: Sequence[float], seconds: float) -> None:
        """Gövdeye dünya çerçevesinde force (N) kuvvetini seconds boyunca uygula."""
        if len(force) != 3:
            raise ValueError("kuvvet 3 bileşenli olmalı")
        self._push = (float(force[0]), float(force[1]), float(force[2]))
        self._push_left = max(1, round(seconds / self.physics_step))

    def step(self, targets: Sequence[float]) -> SimState:
        """18 eklem hedefi (rad, interface sırası) ver, bir kontrol adımı ilerle."""
        if len(targets) != len(self.names):
            raise ValueError(f"{len(self.names)} hedef bekleniyordu, {len(targets)} geldi")
        if not 0 <= self.latency_steps < self.steps_per_action:
            raise ValueError(f"gecikme 0..{self.steps_per_action - 1} fizik adımı olmalı, "
                             f"{self.latency_steps} verildi")
        self._pending = [min(max(t, lim.lower), lim.upper)
                         for t, lim in zip(targets, self._limits)]
        return self._advance(self.steps_per_action)

    @property
    def workdir(self) -> Path:
        return self._dir

    # -- Gazebo geri çağrıları ---------------------------------------------------

    def _advance(self, iterations: int) -> SimState:
        self._state = None
        self._left = iterations
        self._k = 0
        self._server.run(True, iterations, False)
        if self._state is None:
            raise RuntimeError("simülasyon adımı durum üretmedi")
        return self._state

    def _pre(self, info, ecm) -> None:
        if info.paused or self._awaiting_reset:
            return
        if self._joints is None:
            self._bind(ecm)
        if self._pending is not None and self._k >= self.latency_steps:
            self._targets, self._pending = self._pending, None
        self._k += 1
        if self._push_left > 0:
            self._base.add_world_force(ecm, gz.math.Vector3d(*self._push))
            self._push_left -= 1
        kp, kd = self._kp * self._stiffness, self._kd
        tau_s, w0 = self._tau * self._strength, self._vmax
        for i, joint in enumerate(self._joints):
            q = joint.position(ecm)
            w = joint.velocity(ecm)
            if not q or not w:
                continue  # bağlandığı ilk adım: bileşenler henüz yok
            tau = kp * (self._targets[i] - q[0]) - kd * w[0]
            # motor hareket yönünde itiyorsa tork-hız doğrusu, frenliyorsa durma torku
            limit = tau_s * max(0.0, 1.0 - abs(w[0]) / w0) if tau * w[0] > 0 else tau_s
            tau = max(-limit, min(limit, tau))
            self._effort[i] = tau
            joint.set_force(ecm, [tau])
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
            joint_effort=tuple(self._effort),
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
