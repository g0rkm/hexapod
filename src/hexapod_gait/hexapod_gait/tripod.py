"""Tripod yürüyüş çekirdeği: gövde hız komutundan eklem açısına (GOREVLER.md S2).

Girdi: vx, vy (m/s, gövde çerçevesi REP-103: +x ileri, +y sol), wz (rad/s,
+ yukarıdan bakınca saat yönünün tersi). Çıktı: bacak başına JointAngles
(derece), hexapod_kinematics ile aynı birim ve işaret sözleşmesi.

Yöntem
------
Altı bacak gövde etrafındaki açılarına göre ikiye bölünür (tripod_groups);
komşu iki bacak asla aynı grupta olmaz. Bir grup destekte (ayak yerde)
iken öteki salınımda (havada); yarım turda yer değiştirirler. Böylece her
an tam üç ayak yerdedir.

Bu çekirdek GERİ BESLEME KULLANMAZ: gövdenin komut edilen hızı tuttuğunu
varsayar ve buna göre "dünya" çerçevesinde nominal bir gövde pozunu kendi
içinde entegre eder (self._pose). Gerçek robotta/simülasyonda gövde bu
hızı tam tutmayabilir; S3 (ROS düğümü) bunu düzeltmez, yalnızca komutu
iletir. Kapanmamış fark RL katmanının (G6) düzeltmesi.

Ayakların destek fazında dünyada sabit kalması
-----------------------------------------------
Bir ayak yere bastığı anda dünya konumu "çapa" (anchor) olarak kilitlenir
ve DESTEK BOYUNCA HİÇ DEĞİŞMEZ; gövde onun üstünden ilerlermiş gibi
görünür (leg IK buna göre bacağı geriye "çeker"). Ayak havalanırken,
kalktığı çapadan, salınım bitince gövdenin nerede olacağı tahmin edilerek
hesaplanan bir sonraki basış noktasına dümdüz gider; yükseklik
step_height_mm ile sinüs kavisi çizer.

Bu olay tabanlı (event-based) tasarım, komut hızı bir bölüm içinde
değişse bile (RL'nin üstüne bindiği senaryo, G6) her destek fazının kendi
içinde tutarlı kalmasını sağlar; saf "faz -> ofset" formülü (hıza göre
sabit genlik varsayardı) değişen hızda bunu garanti edemezdi.

ROS'a bağlama işi ayrı (GOREVLER.md S3): bu modül konu/mesaj bilmez,
yalnızca step() çağrıldıkça açı üretir.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from hexapod_kinematics import BodyPose, HexapodKinematics, JointAngles, LegMount

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class GaitParams:
    """Yürüyüş parametreleri (robot parametresi DEĞİL, yürüyüş ayarı).

    Varsayılanlar Görkem'in RL tarafındaki nötr duruşuyla aynı
    (hexapod_rl.task.TaskConfig: reach 130 mm, yükseklik 100 mm) — iki
    katman da aynı doğal duruşu konuşsun diye.
    """

    cycle_hz: float = 1.5             # tam adım döngüsü frekansı (Hz)
    step_height_mm: float = 25.0      # salınımda ayağın en fazla ne kadar kalkacağı
    stance_reach_mm: float = 130.0    # ayağın coxa ekseninden yatay uzaklığı (nötr)
    stance_height_mm: float = 100.0   # gövdenin yerden yüksekliği (nötr)


@dataclass(frozen=True)
class _Pose2D:
    """Düzlemsel nominal gövde pozu: x, y mm; yaw derece. Yalnızca bu modül içinde."""

    x: float = 0.0
    y: float = 0.0
    yaw_deg: float = 0.0


def tripod_groups(mounts: dict[int, LegMount]) -> tuple[tuple[int, ...], tuple[int, ...]]:
    """Bacakları gövde etrafındaki açıya göre sırala, sırayla ikiye ayır.

    Komşu iki bacak hiçbir zaman aynı grupta olmaz (klasik tripod deseni).
    hexapod_rl.task.tripod_groups ile aynı mantık; hexapod_gait'in ağır
    RL/gz.sim bağımlılığı olmasın diye burada ayrıca (küçük olduğu için)
    tutuluyor.
    """
    order = sorted(mounts, key=lambda leg: mounts[leg].yaw)
    return tuple(sorted(order[0::2])), tuple(sorted(order[1::2]))


class TripodGait:
    """Tripod yürüyüş üreticisi: step() çağrıldıkça eklem açısı verir.

    Kullanım:
        kin = HexapodKinematics.from_config(RobotConfig.load())
        gait = TripodGait(kin)
        gait.reset()
        angles = gait.step(vx=0.08, vy=0.0, wz=0.0, dt=1 / 50)  # {bacak: JointAngles}
    """

    def __init__(self, kin: HexapodKinematics, params: GaitParams | None = None) -> None:
        self.kin = kin
        self.params = params or GaitParams()
        group_a, group_b = tripod_groups(kin.mounts)
        self._offset: dict[int, float] = {leg: 0.0 for leg in group_a}
        self._offset.update({leg: 0.5 for leg in group_b})
        # Nötr duruş noktası, bacak başına, GÖVDE çerçevesinde (dünya çerçevesi
        # nötr pozda gövde çerçevesiyle çakışır). hexapod_gazebo.pose.standing_pose
        # ile aynı hesap: mount.z kasıtlı yok sayılıyor, kin.inverse'in to_leg_frame'i
        # zaten mount.z'yi kendi içinde uyguluyor.
        self._home: dict[int, Vec3] = {
            leg: (
                mount.x + self.params.stance_reach_mm * math.cos(mount.yaw),
                mount.y + self.params.stance_reach_mm * math.sin(mount.yaw),
                -self.params.stance_height_mm,
            )
            for leg, mount in kin.mounts.items()
        }
        self.reset()

    def reset(self) -> None:
        """Nötr duruşa dön: altı ayak da yerde, faz sıfır, gövde pozu sıfır."""
        self._phase = 0.0
        self._pose = _Pose2D()
        self._started = False
        self._anchor: dict[int, Vec3] = dict(self._home)
        self._target: dict[int, Vec3] = dict(self._home)
        self._feet: dict[int, Vec3] = dict(self._home)

    @property
    def phase(self) -> float:
        """Adım saati, [0, 1)."""
        return self._phase

    def foot_targets(self) -> dict[int, Vec3]:
        """Son step()/reset() sonrası ayakların dünya çerçevesindeki hedefi."""
        return dict(self._feet)

    def step(self, vx: float, vy: float, wz: float, dt: float) -> dict[int, JointAngles]:
        """Bir kontrol adımı ilerle, altı bacağın eklem açılarını döndür.

        vx, vy: m/s (gövde çerçevesi). wz: rad/s. dt: s.
        Erişilemeyen bir hedef çıkarsa hexapod_kinematics.ReachError fırlar
        (kırpılmaz); bu genelde step_height/reach/cycle_hz parametrelerinin
        istenen hızla tutarsız olduğu anlamına gelir.
        """
        new_phase = (self._phase + self.params.cycle_hz * dt) % 1.0
        new_pose = _integrate(self._pose, vx, vy, wz, dt)

        for leg, offset in self._offset.items():
            old_leg_phase = (self._phase + offset) % 1.0
            new_leg_phase = (new_phase + offset) % 1.0
            # İlk çağrıda old_swing bilerek False zorlanır: destekte
            # başlayan bacak zaten çapada (home), salınımda başlayan bacak
            # için de "az önce kalktı" dalı tetiklenip gerçek hedef hesaplanır.
            old_swing = self._started and old_leg_phase < 0.5
            new_swing = new_leg_phase < 0.5

            if new_swing and not old_swing:
                # Az önce kalktı: kalan salınım süresi kadar ileri bakıp
                # gövdenin nerede olacağını tahmin et, oraya bas.
                remaining = (0.5 - new_leg_phase) / self.params.cycle_hz
                predicted = _integrate(new_pose, vx, vy, wz, remaining)
                self._target[leg] = _home_world(self._home[leg], predicted)
            elif old_swing and not new_swing:
                # Az önce bastı: hedef, dünyada sabit çapa oldu.
                self._anchor[leg] = self._target[leg]

            if new_swing:
                u = new_leg_phase * 2.0
                p0, p1 = self._anchor[leg], self._target[leg]
                self._feet[leg] = (
                    p0[0] + (p1[0] - p0[0]) * u,
                    p0[1] + (p1[1] - p0[1]) * u,
                    p0[2] + (p1[2] - p0[2]) * u
                    + self.params.step_height_mm * math.sin(math.pi * u),
                )
            else:
                self._feet[leg] = self._anchor[leg]

        self._phase, self._pose, self._started = new_phase, new_pose, True
        pose = BodyPose(x=new_pose.x, y=new_pose.y, z=0.0,
                        roll=0.0, pitch=0.0, yaw=new_pose.yaw_deg)
        return self.kin.inverse(self._feet, pose)


def _integrate(pose: _Pose2D, vx: float, vy: float, wz: float, dt: float) -> _Pose2D:
    """Komut edilen hızla nominal gövde pozunu dt kadar ileri götür.

    Açık döngü tahmin (gövdenin bu hızı tam tuttuğu varsayımıyla). m/s ->
    mm/s için 1000 çarpanı; hexapod_kinematics mm cinsinden çalışıyor.
    """
    yaw = math.radians(pose.yaw_deg)
    dx = (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt * 1000.0
    dy = (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt * 1000.0
    return _Pose2D(pose.x + dx, pose.y + dy, pose.yaw_deg + math.degrees(wz * dt))


def _home_world(home_body: Vec3, pose: _Pose2D) -> Vec3:
    """Bacağın nötr (gövde çerçevesindeki) duruş noktasını dünya çerçevesine taşı."""
    yaw = math.radians(pose.yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = home_body
    return (pose.x + c * x - s * y, pose.y + s * x + c * y, z)


__all__ = ["GaitParams", "TripodGait", "tripod_groups"]
