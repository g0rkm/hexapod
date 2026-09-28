"""hexapod_rl.sim: süreç içi Gazebo simülasyonu.

Yalnızca gz.sim Python bağlarının olduğu yerde (ROS 2 Lyrical kurulu
WSL/Linux) koşar; Windows'ta bütün dosya atlanır.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from hexapod_description import RobotModel
from hexapod_driver import RobotConfig

REAL_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"


# --- Gazebo (yalnız gz.sim varsa) ---------------------------------------------------

gz_sim = pytest.importorskip("gz.sim", reason="gz.sim yok (ROS 2 Lyrical kurulu Linux gerekir)")

from hexapod_gazebo.pose import standing_pose  # noqa: E402
from hexapod_kinematics import HexapodKinematics  # noqa: E402
from hexapod_rl.sim import HexapodSim  # noqa: E402


@pytest.fixture(scope="module")
def model() -> RobotModel:
    return RobotModel.from_config(RobotConfig.load(REAL_CONFIG))


@pytest.fixture(scope="module")
def sim(model, tmp_path_factory) -> HexapodSim:
    return HexapodSim(model, workdir=tmp_path_factory.mktemp("rl"))


def run(sim, targets, seconds):
    state = None
    for _ in range(int(round(seconds / sim.dt))):
        state = sim.step(targets)
    return state


def stand_targets(sim):
    angles = standing_pose(HexapodKinematics.from_config(RobotConfig.load(REAL_CONFIG)),
                           130.0, 100.0)
    return [math.radians(v) for leg in range(6) for v in angles[leg].as_dict().values()]


def test_sifir_durusunda_ayakta_ve_alti_ayak_yerde(sim, model):
    sim.reset()
    s = run(sim, [0.0] * 18, 1.0)
    # gövde = tibia + femur ekleminin montaj düzlemine göre ofseti
    expected = model.tibia - min(m.z for m in model.mounts.values())
    assert s.base_pos[2] == pytest.approx(expected, abs=0.002)
    assert all(s.foot_contact)
    assert s.gravity_in_base() == pytest.approx((0.0, 0.0, -1.0), abs=0.01)


def test_ayaga_kalkma_istenen_yukseklikte(sim):
    sim.reset()
    s = run(sim, stand_targets(sim), 3.0)
    assert s.base_pos[2] == pytest.approx(0.100, abs=0.002)
    assert all(s.foot_contact)
    # ölçülen eklem açıları hedefe oturmuş
    for measured, target in zip(s.joint_pos, s.joint_target):
        assert measured == pytest.approx(target, abs=math.radians(1.0))


def test_sifirlama_ilk_hale_dondurur(sim, model):
    sim.reset()
    run(sim, stand_targets(sim), 1.0)
    s = sim.reset()
    assert s.time == pytest.approx(sim.dt, abs=1e-9)
    assert s.joint_target == tuple([0.0] * 18)
    assert s.base_pos[2] > 0.13  # yeniden doğdu, henüz yerleşiyor


def test_ayni_komutlar_ayni_sonuc(sim):
    """RL tekrarlanabilirliği: aynı başlangıç + aynı komutlar = aynı durum."""
    targets = stand_targets(sim)
    results = []
    for _ in range(2):
        sim.reset()
        results.append(run(sim, targets, 1.0))
    a, b = results
    assert a.base_pos == pytest.approx(b.base_pos, abs=1e-9)
    assert a.joint_pos == pytest.approx(b.joint_pos, abs=1e-9)


def test_hedef_limitlere_kirpilir(sim, model):
    sim.reset()
    s = sim.step([2.0] * 18)
    lim = model.limits[(0, "femur")]
    assert s.joint_target[1] == pytest.approx(lim.upper)


def test_yanlis_hedef_sayisi_reddedilir(sim):
    with pytest.raises(ValueError):
        sim.step([0.0] * 17)


STEP_MM, LIFT_MM, GAIT_HZ, REACH_MM, HEIGHT_MM = 30.0, 25.0, 1.5, 130.0, 100.0


def open_loop_tripod():
    """Açık döngü tripod: adım saati (0..1) -> 18 eklem hedefi (IK ile ayak yörüngesi).
    Yürüyüş Samet'in işi (S2); bu yalnızca fiziği doğrulayan en basit yörünge."""
    from hexapod_rl.task import tripod_groups

    step_mm, lift_mm, reach, height = STEP_MM, LIFT_MM, REACH_MM, HEIGHT_MM
    kin = HexapodKinematics.from_config(RobotConfig.load(REAL_CONFIG))
    group_a, _ = tripod_groups({leg: m.yaw for leg, m in kin.mounts.items()})
    home = {leg: (m.x + reach * math.cos(m.yaw), m.y + reach * math.sin(m.yaw), -height)
            for leg, m in kin.mounts.items()}

    def targets(phase):
        feet = {}
        for leg, (x, y, z) in home.items():
            p = phase if leg in group_a else (phase + 0.5) % 1.0
            if p < 0.5:                      # destek: ayak geriye kayar
                s, dz = 0.5 - 2 * p, 0.0
            else:                            # salınım: kalkıp öne gelir
                q = (p - 0.5) * 2
                s, dz = -0.5 + q, lift_mm * math.sin(math.pi * q)
            feet[leg] = (x + s * step_mm, y, z + dz)
        ang = kin.inverse(feet)
        return [math.radians(v) for leg in range(6) for v in ang[leg].as_dict().values()]

    return targets


def test_simulasyon_yurumeye_izin_veriyor(sim):
    """Açık döngü tripod beklenen hızın en az %70'ine ulaşmalı.

    Bu test, hız komutlu servo modelinde ayakların kaydığını (beklenenin
    %12'si) yakalardı; o model yüzünden iki PPO eğitimi boşa gitti.
    """
    targets = open_loop_tripod()
    sim.reset()
    s = run(sim, targets(0.0), 1.0)
    x0, phase = s.base_pos[0], 0.0
    for _ in range(int(round(4.0 / sim.dt))):
        phase = (phase + GAIT_HZ * sim.dt) % 1.0
        s = sim.step(targets(phase))
    speed = (s.base_pos[0] - x0) / 4.0
    expected = STEP_MM / 1000 / (0.5 / GAIT_HZ)
    assert speed > 0.7 * expected, f"{speed:.3f} m/s, beklenen {expected:.3f}"
    assert abs(s.base_pos[2] - HEIGHT_MM / 1000) < 0.005


def test_guc_kontrol_adimi_ortalamasi_gecikmeden_bagimsiz(sim):
    """Açık döngü yürüyüşte komut gecikmesi yalnız bir faz kaymasıdır, harcanan
    gücü değiştirmemeli. Ödülün güç terimi 2026-09-28'e kadar kontrol adımının
    SON fizik adımındaki tork x hızdan hesaplanıyordu: örnek, hedefin değiştiği
    ana göre kayınca aynı yürüyüş 16 ms gecikmede 4 kat "güç" gösteriyordu
    (ders 48). mean_power bütün fizik adımlarının ortalaması."""
    targets = open_loop_tripod()
    mean, instant = [], []
    for latency in (0, 8):                                # 0 ve 16 ms
        sim.reset()
        sim.latency_steps = latency
        run(sim, targets(0.0), 1.0)
        phase, m, i, n = 0.0, 0.0, 0.0, int(round(2.0 / sim.dt))
        for _ in range(n):
            phase = (phase + GAIT_HZ * sim.dt) % 1.0
            s = sim.step(targets(phase))
            m += s.mean_power
            i += sum(abs(t * v) for t, v in zip(s.joint_effort, s.joint_vel))
        mean.append(m / n)
        instant.append(i / n)
    sim.latency_steps = 0
    assert mean[1] == pytest.approx(mean[0], rel=0.05), mean
    assert max(instant) > 1.5 * min(instant), instant     # eski ölçüm: örnekleme kayması
    assert mean[0] > 0.5                                  # yürüyüş iş yapıyor (W)


# --- alan rastgeleleştirme düğmeleri (G7) ----------------------------------------


@pytest.fixture
def ayakta(sim):
    """Ayakta duran robot; test sonunda düğmeler varsayılana döner."""
    sim.reset()
    run(sim, stand_targets(sim), 1.5)
    yield sim
    sim.set_servo(1.0, 1.0)
    sim.latency_steps = 0


def test_gecikme_hedefi_geciktirir(ayakta):
    """Gecikmeli komutta eklem bir kontrol adımında daha az yol alır."""
    sim, stand = ayakta, stand_targets(ayakta)
    moved = []
    for latency in (0, sim.steps_per_action - 1):
        sim.reset()
        run(sim, stand, 1.5)
        sim.latency_steps = latency
        before = sim.step(stand).joint_pos[0]
        target = list(stand)
        target[0] += 0.3                                  # bacak 0 coxa
        after = sim.step(target)
        moved.append(after.joint_pos[0] - before)
        assert after.joint_target[0] == pytest.approx(target[0])   # adım sonunda uygulanmış
    assert moved[1] < 0.5 * moved[0]
    sim.latency_steps = sim.steps_per_action
    with pytest.raises(ValueError):
        sim.step(stand)


def test_zayif_servo_torku_sinirli(ayakta, model):
    sim = ayakta
    sim.set_servo(strength=0.5)
    target = list(stand_targets(sim))
    target[1] += 0.8                                      # bacak 0 femur, büyük adım
    s = sim.step(target)
    assert max(abs(e) for e in s.joint_effort) <= 0.5 * model.effort + 1e-9
    with pytest.raises(ValueError):
        sim.set_servo(strength=0.0)


def test_itme_govdeyi_kaydirir(ayakta):
    sim, stand = ayakta, stand_targets(ayakta)
    y0 = sim.step(stand).base_pos[1]
    sim.push((0.0, 15.0, 0.0), 0.1)                       # +y yönünde
    s = run(sim, stand, 0.2)
    assert s.base_pos[1] - y0 > 0.002
    s2 = run(sim, stand, 0.5)                             # itme bitti, yeni kuvvet yok
    assert s2.base_lin_vel[1] == pytest.approx(0.0, abs=0.05)


def test_surec_ici_dunya_ayri_gz_bolumunde(sim):
    """Eğitim dünyaları gz-transport'ta dışarıya (ROS'lu simülasyona) görünmemeli."""
    import os

    from hexapod_rl.sim import PARTITION_PREFIX
    assert os.environ["GZ_PARTITION"] == f"{PARTITION_PREFIX}{os.getpid()}"


def test_kapatinca_gazebo_birakilir(model, tmp_path):
    """close() sunucuyu bırakmalı: aynı süreçte kur/kapat tekrarlanınca iş
    parçacığı sayısı büyümemeli. Düzeltmeden önce her dünya 2 iş parçacığı ve
    ~31 MB bırakıyordu (eğitim içi zemin ölçümü süreçte onlarca dünya kurar)."""
    import gc
    import os

    def threads() -> int:
        return len(os.listdir(f"/proc/{os.getpid()}/task"))

    counts = []
    for i in range(4):
        s = HexapodSim(model, workdir=tmp_path / f"w{i}")
        s.reset()
        s.close()
        del s
        gc.collect()
        counts.append(threads())
    assert counts[-1] <= counts[1], counts
