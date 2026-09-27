"""hexapod_policy.controller — ROS'suz politika çekirdeği. Her yerde koşar.

En önemli test gözlem sözleşmesi: düğümün kurduğu gözlem, politikanın
eğitildiği gözlemle (hexapod_rl.task.observation) birebir aynı olmalı.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from hexapod_policy.controller import PolicyController, gravity_in_base
from hexapod_policy.mlp import MlpPolicy, PolicyContract
from hexapod_rl.math3d import rotate_inverse
from hexapod_rl.state import SimState
from hexapod_rl.task import ACTION_SIZE, OBS_SIZE, observation

RANGES = {"vx": (0.05, 0.15), "vy": (0.0, 0.0), "wz": (0.0, 0.0)}
DEFAULT = tuple(0.01 * i for i in range(ACTION_SIZE))
LIMITS = [(-1.0, 1.0)] * ACTION_SIZE
LEVEL = (1.0, 0.0, 0.0, 0.0)


def constant_policy(bias) -> MlpPolicy:
    """Gözlemden bağımsız, eylemi hep 'bias' olan politika (tek doğrusal katman)."""
    c = PolicyContract(obs_size=OBS_SIZE, action_size=ACTION_SIZE, action_scale=0.5,
                       gait_hz=1.5, control_hz=50.0, default_rad=DEFAULT, command_ranges=RANGES)
    return MlpPolicy([(np.zeros((ACTION_SIZE, OBS_SIZE)), np.asarray(bias, dtype=float))],
                     "tanh", c)


def random_policy(seed=1) -> MlpPolicy:
    rng = np.random.default_rng(seed)
    c = constant_policy([0.0] * ACTION_SIZE).contract
    return MlpPolicy([(rng.normal(0, 0.3, (16, OBS_SIZE)), rng.normal(0, 0.1, 16)),
                      (rng.normal(0, 0.3, (ACTION_SIZE, 16)), rng.normal(0, 0.1, ACTION_SIZE))],
                     "tanh", c)


def walking(policy=None, t=0.0) -> PolicyController:
    c = PolicyController(policy or constant_policy([0.2] * ACTION_SIZE), LIMITS)
    c.on_imu(LEVEL, (0.0, 0.0, 0.0), t)
    c.on_command(0.1, 0.0, 0.0, t)
    return c


def small_tilt(rng) -> tuple[float, float, float, float]:
    """~20°'ye kadar yatık, herhangi bir yöne dönük gövde (devrilme eşiğinin altında)."""
    q = np.array([1.0, *rng.normal(0, 0.12, 2), rng.normal(0, 1.0)])
    q /= np.linalg.norm(q)
    return tuple(float(v) for v in q)


def unit_quat(rng) -> tuple[float, float, float, float]:
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    return tuple(float(v) for v in q)


# --- gözlem sözleşmesi -------------------------------------------------------------


def test_yercekimi_egitimdekiyle_ayni():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = unit_quat(rng)
        assert gravity_in_base(q) == pytest.approx(rotate_inverse(q, (0.0, 0.0, -1.0)), abs=1e-12)


def test_gozlem_egitim_gozlemiyle_birebir_ayni():
    """Birkaç adım yürüdükten sonra (saat ve son hedefler ilerlemişken) aynı durumdan
    task.observation ile controller.observation aynı vektörü vermeli."""
    rng = np.random.default_rng(3)
    policy = random_policy()
    c = walking(policy)
    for k in range(7):                                      # son adımda eğik ve dönük gövde
        c.on_imu(small_tilt(rng) if k == 6 else LEVEL, (0.1, -0.2, 0.3), k / 50)
        c.on_command(0.12, 0.0, 0.0, k / 50)
        c.tick(k / 50)
    q, gyro = c._imu
    state = SimState(
        time=0.0, joint_pos=(0.0,) * 18, joint_vel=(0.0,) * 18,
        joint_target=tuple(c._targets), joint_effort=(0.0,) * 18,
        base_pos=(0.0, 0.0, 0.1), base_quat=q, base_lin_vel=(0.0, 0.0, 0.0),
        # SimState dünya çerçevesinde açısal hız tutar; jiroskop gövde çerçevesinde
        base_ang_vel=_rotate(q, gyro), foot_pos=((0.0, 0.0, 0.0),) * 6,
        foot_contact=(True,) * 6)
    assert c.status == "yürüyor"
    expected = observation(state, (0.12, 0.0, 0.0), c._phase, list(DEFAULT), 0.5)
    assert c.observation((0.12, 0.0, 0.0)) == pytest.approx(expected, abs=1e-12)


def _rotate(q, v):
    """Gövde -> dünya (rotate_inverse'in tersi)."""
    w, x, y, z = q
    return rotate_inverse((w, -x, -y, -z), v)


# --- yürüme ------------------------------------------------------------------------


def test_yururken_hedef_varsayilan_arti_olcekli_eylem_ve_saat_ilerler():
    c = walking(constant_policy([0.2] * ACTION_SIZE))
    targets = c.tick(0.0)
    assert c.status == "yürüyor"
    assert targets == pytest.approx([d + 0.5 * 0.2 for d in DEFAULT])
    assert c._phase == pytest.approx(1.5 / 50)


def test_eylem_ve_hedef_kirpilir():
    policy = constant_policy([5.0] * ACTION_SIZE)            # eylem [-1, 1]'e kırpılır
    c = PolicyController(policy, [(-1.0, 0.3)] * ACTION_SIZE)  # hedef limite kırpılır
    c.on_imu(LEVEL, (0, 0, 0), 0.0)
    c.on_command(0.1, 0.0, 0.0, 0.0)
    targets = c.tick(0.0)
    assert targets == pytest.approx([min(d + 0.5, 0.3) for d in DEFAULT])


def test_komut_egitim_araligina_kirpilir():
    c = walking()
    c.on_command(0.4, 0.05, 0.0, 0.0)
    c.tick(0.0)
    assert c.clipped_command
    assert c.observation(c._effective_command())[24:27] == pytest.approx([0.15, 0.0, 0.0])
    c.on_command(0.1, 0.0, 0.0, 0.0)
    c.tick(0.0)
    assert not c.clipped_command


# --- güvenlik: ayakta bekle ----------------------------------------------------------


@pytest.mark.parametrize("setup, reason", [
    (lambda c: setattr(c, "_command", None), "komut yok"),
    (lambda c: c.on_command(0.1, 0.0, 0.0, -1.0), "komut zaman aşımı"),
    (lambda c: c.on_command(0.02, 0.0, 0.0, 0.0), "dur"),
    (lambda c: c.on_command(-0.1, 0.0, 0.0, 0.0), "dur"),        # geri: eğitilmedi
    (lambda c: c.on_command(0.0, 0.08, 0.0, 0.0), "dur"),        # yana: eğitilmedi
    (lambda c: setattr(c, "_imu", None), "IMU yok"),
    (lambda c: c.on_imu(LEVEL, (0, 0, 0), -1.0), "IMU bayat"),
    (lambda c: c.on_imu((math.cos(0.5), math.sin(0.5), 0.0, 0.0), (0, 0, 0), 0.0), "devrildi"),
])
def test_guvensiz_durumda_ayakta_bekler_ve_saat_sifirlanir(setup, reason):
    c = walking()
    for k in range(5):
        c.tick(0.0)
    assert c._phase > 0
    setup(c)
    targets = c.tick(0.0)
    assert c.status.startswith(reason)
    assert targets == pytest.approx(list(DEFAULT))
    assert c._phase == 0.0


def test_bozuk_imu_yok_sayilir():
    c = walking()
    c.on_imu((float("nan"), 0, 0, 0), (0, 0, 0), 0.0)
    c.on_imu((0.0, 0.0, 0.0, 0.0), (0, 0, 0), 0.0)
    assert c._imu[0] == LEVEL
    c.on_imu(LEVEL, (0, 0, 0), 5.0)
    c.on_command(0.1, 0.0, 0.0, 5.0)
    c.on_imu((float("nan"), 0, 0, 0), (0, 0, 0), 5.1)      # yok sayıldı: son geçerli 5.0
    c.tick(5.3)
    assert c.status == "IMU bayat"


def test_limit_sayisi_denetlenir():
    with pytest.raises(ValueError):
        PolicyController(constant_policy([0.0] * ACTION_SIZE), [(-1.0, 1.0)] * 17)


# --- artık eylem modu -------------------------------------------------------------


def test_artik_eylemde_sifir_duzeltme_tripodun_kendisi():
    """Eylem 0 iken hedef tam olarak eğitimdeki tripod (PhaseTripod); eylem
    residual_scale ile eklenir. Kinematik verilmezse düğüm başlamaz."""
    from pathlib import Path

    from hexapod_driver import RobotConfig
    from hexapod_kinematics import HexapodKinematics
    from hexapod_policy.tripod import PhaseTripod

    kin = HexapodKinematics.from_config(
        RobotConfig.load(Path(__file__).resolve().parent.parent / "config" / "robot.yaml"))
    base_gait = {"groups": [[0, 2, 4], [1, 3, 5]], "reach_mm": 130.0, "height_mm": 100.0,
                 "lift_mm": 25.0}
    c = PolicyContract(obs_size=OBS_SIZE, action_size=ACTION_SIZE, action_scale=0.5, gait_hz=1.5,
                       control_hz=50.0, default_rad=DEFAULT, command_ranges=RANGES,
                       action_mode="residual", residual_scale=0.2, base_gait=base_gait)
    tripod = PhaseTripod(kin, ((0, 2, 4), (1, 3, 5)), 1.5, 130.0, 100.0, 25.0)
    for bias, offset in ((0.0, 0.0), (0.5, 0.1)):
        policy = MlpPolicy([(np.zeros((ACTION_SIZE, OBS_SIZE)), np.full(ACTION_SIZE, bias))],
                           "tanh", c)
        ctl = PolicyController(policy, [(-3.0, 3.0)] * ACTION_SIZE, kin=kin)
        ctl.on_imu(LEVEL, (0, 0, 0), 0.0)
        ctl.on_command(0.1, 0.0, 0.0, 0.0)
        for k in range(10):
            expected = [t + offset for t in tripod.targets(ctl._phase, (0.1, 0.0, 0.0))]
            assert ctl.tick(0.0) == pytest.approx(expected)
    with pytest.raises(ValueError):
        PolicyController(policy, [(-3.0, 3.0)] * ACTION_SIZE)


# --- her yöne eğitilmiş politika (2026-09-26) -------------------------------------


OMNI = {"vx": (-0.15, 0.15), "vy": (-0.08, 0.08), "wz": (-0.5, 0.5)}


def omni_walking() -> PolicyController:
    from dataclasses import replace

    p = constant_policy([0.2] * ACTION_SIZE)
    c = replace(p.contract, command_ranges=OMNI, command_deadband=1 / 6)
    ctl = PolicyController(MlpPolicy(p.layers, "tanh", c), LIMITS)
    ctl.on_imu(LEVEL, (0.0, 0.0, 0.0), 0.0)
    return ctl


@pytest.mark.parametrize("command", [(-0.1, 0.0, 0.0), (0.0, 0.05, 0.0), (0.0, 0.0, -0.3),
                                     (0.03, 0.0, 0.2)])
def test_her_yon_politikasi_geri_yana_donuste_yurur(command):
    c = omni_walking()
    c.on_command(*command, 0.0)
    c.tick(0.0)
    assert c.status == "yürüyor"
    assert c.observation(c._effective_command())[24:27] == pytest.approx(list(command))


@pytest.mark.parametrize("command", [(0.0, 0.0, 0.0), (0.02, 0.01, 0.05)])
def test_her_yon_politikasi_olu_bolgede_durur(command):
    c = omni_walking()
    c.on_command(*command, 0.0)
    assert c.tick(0.0) == pytest.approx(list(DEFAULT))
    assert c.status.startswith("dur (komut ölü bölgede")


def test_ogrenilmis_ayak_kaldirma_tabana_gecer():
    """Ağın son çıkışı eğitimdeki eşlemeyle ayak kaldırmaya çevrilir ve tripod
    tabanı o yükseklikle hesaplanır (eğitimdeki env ile aynı)."""
    from pathlib import Path

    from hexapod_driver import RobotConfig
    from hexapod_kinematics import HexapodKinematics
    from hexapod_policy.tripod import PhaseTripod

    kin = HexapodKinematics.from_config(
        RobotConfig.load(Path(__file__).resolve().parent.parent / "config" / "robot.yaml"))
    base_gait = {"groups": [[0, 2, 4], [1, 3, 5]], "reach_mm": 130.0, "height_mm": 100.0,
                 "lift_mm": 50.0}
    c = PolicyContract(obs_size=OBS_SIZE, action_size=ACTION_SIZE, action_scale=0.5, gait_hz=1.5,
                       control_hz=50.0, default_rad=DEFAULT, command_ranges=RANGES,
                       action_mode="residual", residual_scale=0.2, base_gait=base_gait,
                       lift_range=(20.0, 60.0))
    bias = np.zeros(ACTION_SIZE + 1)
    bias[-1] = 0.5                                             # -> 20 + 0.75 x 40 = 50 mm
    policy = MlpPolicy([(np.zeros((ACTION_SIZE + 1, OBS_SIZE)), bias)], "tanh", c)
    ctl = PolicyController(policy, [(-3.0, 3.0)] * ACTION_SIZE, kin=kin)
    ctl.on_imu(LEVEL, (0, 0, 0), 0.0)
    ctl.on_command(0.1, 0.0, 0.0, 0.0)
    tripod = PhaseTripod(kin, ((0, 2, 4), (1, 3, 5)), 1.5, 130.0, 100.0, 25.0)
    for k in range(5):
        phase = ctl._phase
        targets = ctl.tick(0.0)
        lift = 50.0                                        # son katman doğrusal: çıkış = sapma
        assert ctl.lift_mm == pytest.approx(lift)
        assert targets == pytest.approx(tripod.targets(phase, (0.1, 0.0, 0.0), lift), abs=1e-9)


def test_ogrenilmis_ayak_kaldirma_salinim_boyunca_sabit():
    """Kaldırma eğitimdeki gibi yalnız salınımın ilk adımında seçilir: çıkış
    salınım ortasında değişse de yeni salınıma kadar eski değer kalır; durunca
    (bekleme) sıfırlanır ve ilk adımda yeniden seçilir."""
    from pathlib import Path

    from hexapod_driver import RobotConfig
    from hexapod_kinematics import HexapodKinematics

    kin = HexapodKinematics.from_config(
        RobotConfig.load(Path(__file__).resolve().parent.parent / "config" / "robot.yaml"))
    base_gait = {"groups": [[0, 2, 4], [1, 3, 5]], "reach_mm": 130.0, "height_mm": 100.0,
                 "lift_mm": 25.0}
    c = PolicyContract(obs_size=OBS_SIZE, action_size=ACTION_SIZE, action_scale=0.5, gait_hz=1.5,
                       control_hz=50.0, default_rad=DEFAULT, command_ranges=RANGES,
                       action_mode="residual", residual_scale=0.2, base_gait=base_gait,
                       lift_range=(20.0, 60.0))

    def policy(lift_out):
        bias = np.zeros(ACTION_SIZE + 1)
        bias[-1] = lift_out
        return MlpPolicy([(np.zeros((ACTION_SIZE + 1, OBS_SIZE)), bias)], "tanh", c)

    ctl = PolicyController(policy(-1.0), [(-3.0, 3.0)] * ACTION_SIZE, kin=kin)
    ctl.on_imu(LEVEL, (0, 0, 0), 0.0)
    ctl.on_command(0.1, 0.0, 0.0, 0.0)
    ctl.tick(0.0)
    assert ctl.lift_mm == pytest.approx(20.0)            # ilk adım: seçildi
    ctl.policy = policy(1.0)
    while ctl._phase < 0.5:                               # ilk salınım sürüyor
        ctl.tick(0.0)
        assert ctl.lift_mm == pytest.approx(20.0)
    ctl.tick(0.0)
    assert ctl.lift_mm == pytest.approx(60.0)            # öteki grubun salınımı başladı
    ctl.policy = policy(0.0)
    ctl.tick(10.0)                                        # IMU bayat: bekler
    assert ctl.lift_mm is None and ctl._phase == 0.0
    ctl.on_imu(LEVEL, (0, 0, 0), 10.0)
    ctl.on_command(0.1, 0.0, 0.0, 10.0)
    ctl.tick(10.0)
    assert ctl.lift_mm == pytest.approx(40.0)            # yeniden başlarken hemen seçilir


def test_salinim_grubu_ayak_yorungesiyle_uyumlu():
    """swing_group'un söylediği grup gerçekten havada, öteki yerde."""
    from pathlib import Path

    from hexapod_driver import RobotConfig
    from hexapod_kinematics import HexapodKinematics
    from hexapod_policy.tripod import PhaseTripod

    kin = HexapodKinematics.from_config(
        RobotConfig.load(Path(__file__).resolve().parent.parent / "config" / "robot.yaml"))
    groups = ((0, 2, 4), (1, 3, 5))
    tripod = PhaseTripod(kin, groups, 1.5, 130.0, 100.0, 25.0)
    for phase in (0.1, 0.3, 0.45, 0.6, 0.8, 0.95, 1.2):
        g = PhaseTripod.swing_group(phase)
        feet = tripod.feet(phase % 1.0, (0.1, 0.0, 0.0))
        assert all(feet[leg][2] > -100.0 for leg in groups[g]), phase
        assert all(feet[leg][2] == pytest.approx(-100.0) for leg in groups[1 - g]), phase


def _reflex_controller(lift_range=None, **kw):
    """Artık eylem, düzeltme 0; lift_range verilirse kaldırma çıkışı 20+0.5*... = 50 mm."""
    from pathlib import Path

    from hexapod_driver import RobotConfig
    from hexapod_kinematics import HexapodKinematics
    from hexapod_policy.lift_reflex import LiftReflex, RangeSensor

    kin = HexapodKinematics.from_config(
        RobotConfig.load(Path(__file__).resolve().parent.parent / "config" / "robot.yaml"))
    base_gait = {"groups": [[0, 2, 4], [1, 3, 5]], "reach_mm": 130.0, "height_mm": 100.0,
                 "lift_mm": 25.0}
    c = PolicyContract(obs_size=OBS_SIZE, action_size=ACTION_SIZE, action_scale=0.5, gait_hz=1.5,
                       control_hz=50.0, default_rad=DEFAULT, command_ranges=RANGES,
                       action_mode="residual", residual_scale=0.2, base_gait=base_gait,
                       lift_range=lift_range)
    n = ACTION_SIZE + (1 if lift_range else 0)
    bias = np.zeros(n)
    if lift_range:
        bias[-1] = 0.5                                        # -> 50 mm (20-60 aralığında)
    policy = MlpPolicy([(np.zeros((n, OBS_SIZE)), bias)], "tanh", c)
    sensor = RangeSensor(0.10, 0.0, 0.02, 0.0, 20.0, 1.0)
    ctl = PolicyController(policy, [(-3.0, 3.0)] * ACTION_SIZE, kin=kin, range_sensors=[sensor],
                           reflex=LiftReflex(), **kw)
    ctl.on_imu(LEVEL, (0, 0, 0), 0.0)
    ctl.on_command(0.1, 0.0, 0.0, 0.0)
    flat = 0.12 / math.sin(math.radians(20.0))                # düz zeminde ışının boyu
    step45 = (0.12 - 0.045) / math.sin(math.radians(20.0))    # 45 mm yüksekliğe çarpan
    return ctl, flat, step45


def _run(ctl, t0, n, distance=None):
    """n kontrol adımı; distance verilirse her adımda yeni ölçüm. Son zaman."""
    t = t0
    for _ in range(n):
        ctl.on_imu(LEVEL, (0, 0, 0), t)
        ctl.on_command(0.1, 0.0, 0.0, t)
        if distance is not None:
            ctl.on_ranges([distance], t)
        ctl.tick(t)
        t += 0.02
    return t


def test_refleks_duzde_dusuk_engelde_yuksek_kaldirma():
    """Düz zeminde refleks 25 mm; 45 mm engel görününce (art arda 2 ölçüm) bir
    sonraki salınımdan itibaren 45 + 15 = 60 mm."""
    ctl, flat, step45 = _reflex_controller()
    t = _run(ctl, 0.0, 40, flat)
    assert ctl.reflex_status == "görüyor" and ctl.lift_mm == pytest.approx(25.0)
    t = _run(ctl, t, 40, step45)                              # > bir salınım
    assert ctl.lift_mm == pytest.approx(60.0)


def test_refleks_ayni_olcumu_iki_kez_saymaz_ve_bayatlayinca_kor_davranisa_doner():
    """Sensör yavaşsa aynı ölçüm art arda sayılmaz (tek ölçüm = tek okuma);
    mesafe gelmeyince kaldırma çıkışlı politikada politikanın seçtiği (50 mm)."""
    ctl, flat, step45 = _reflex_controller(lift_range=(20.0, 60.0))
    t = _run(ctl, 0.0, 40, flat)
    assert ctl.lift_mm == pytest.approx(25.0)
    ctl.on_ranges([step45], t)                                # tek bir yeni ölçüm
    t = _run(ctl, t, 5)                                       # aynı ölçüm, yeni değil
    assert ctl._reflex_lift == pytest.approx(25.0)            # art arda 2 okuma olmadı
    t = _run(ctl, t + 0.3, 40)                                # 0.3 s ölçüm yok: bayat
    assert ctl.reflex_status == "mesafe bayat" and ctl.lift_mm == pytest.approx(50.0)


def test_refleks_mesafe_hic_gelmezse_tabanin_kaldirmasi():
    ctl, _, _ = _reflex_controller()
    _run(ctl, 0.0, 40)
    assert ctl.reflex_status == "mesafe yok" and ctl.lift_mm is None   # taban: 25 mm sabit


def test_refleks_yanlis_kurulum_reddedilir():
    from hexapod_policy.lift_reflex import LiftReflex, RangeSensor

    c = PolicyContract(obs_size=OBS_SIZE, action_size=ACTION_SIZE, action_scale=0.5, gait_hz=1.5,
                       control_hz=50.0, default_rad=DEFAULT, command_ranges=RANGES)
    policy = MlpPolicy([(np.zeros((ACTION_SIZE, OBS_SIZE)), np.zeros(ACTION_SIZE))], "tanh", c)
    sensor = RangeSensor(0.10, 0.0, 0.02, 0.0, 20.0, 1.0)
    with pytest.raises(ValueError):                           # mutlak mod: taban yok
        PolicyController(policy, [(-3.0, 3.0)] * ACTION_SIZE, range_sensors=[sensor],
                         reflex=LiftReflex())
    with pytest.raises(ValueError):                           # yerleşimsiz refleks
        PolicyController(policy, [(-3.0, 3.0)] * ACTION_SIZE, reflex=LiftReflex())
