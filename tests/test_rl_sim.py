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


def test_simulasyon_yurumeye_izin_veriyor(sim):
    """Açık döngü tripod (IK ile ayak yörüngesi) beklenen hızın en az %70'ine ulaşmalı.

    Bu test, hız komutlu servo modelinde ayakların kaydığını (beklenenin
    %12'si) yakalardı; o model yüzünden iki PPO eğitimi boşa gitti. Yürüyüş
    Samet'in işi (S2); buradaki yalnızca fiziği doğrulayan en basit yörünge.
    """
    from hexapod_rl.task import tripod_groups

    step_mm, lift_mm, hz, reach, height = 30.0, 25.0, 1.5, 130.0, 100.0
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

    sim.reset()
    s = run(sim, targets(0.0), 1.0)
    x0, phase = s.base_pos[0], 0.0
    for _ in range(int(round(4.0 / sim.dt))):
        phase = (phase + hz * sim.dt) % 1.0
        s = sim.step(targets(phase))
    speed = (s.base_pos[0] - x0) / 4.0
    expected = step_mm / 1000 / (0.5 / hz)
    assert speed > 0.7 * expected, f"{speed:.3f} m/s, beklenen {expected:.3f}"
    assert abs(s.base_pos[2] - height / 1000) < 0.005
