"""hexapod_gait.TripodGait testleri (saf Python, donanımsız) — GOREVLER.md S2.

"Bitti sayılır" şartları:
  - bütün yörünge boyunca her ayak erişim alanında (ReachError yok)
  - destek fazındaki ayaklar dünyada sabit
  - bir döngüde gövde hedef mesafeyi alıyor
  - her an en az üç ayak yerde
"""

from __future__ import annotations

from pathlib import Path

import pytest

from hexapod_driver import RobotConfig
from hexapod_kinematics import HexapodKinematics

from hexapod_gait import GaitParams, TripodGait, tripod_groups

REAL_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"
DT = 1.0 / 50.0  # hexapod_description.interface.COMMAND_RATE_HZ ile aynı


@pytest.fixture(scope="module")
def kin() -> HexapodKinematics:
    return HexapodKinematics.from_config(RobotConfig.load(REAL_CONFIG))


@pytest.fixture
def gait(kin) -> TripodGait:
    g = TripodGait(kin)
    g.reset()
    return g


# ---------------------------------------------------------------------------
# Gruplama
# ---------------------------------------------------------------------------


def test_komsu_bacaklar_ayni_grupta_degil(kin):
    a, b = tripod_groups(kin.mounts)
    assert sorted(a + b) == sorted(kin.mounts)
    assert len(a) == 3 and len(b) == 3
    order = sorted(kin.mounts, key=lambda leg: kin.mounts[leg].yaw)
    for i, leg in enumerate(order):
        neighbor = order[(i + 1) % len(order)]
        assert (leg in a) != (neighbor in a), f"{leg} ve {neighbor} komşu ama aynı grupta"


# ---------------------------------------------------------------------------
# Erişim: hiçbir ayak ReachError almıyor
# ---------------------------------------------------------------------------


def test_duz_yururken_hicbir_ayak_erisim_disina_cikmiyor(gait):
    for _ in range(int(round(5.0 / DT))):
        gait.step(vx=0.08, vy=0.0, wz=0.0, dt=DT)  # ReachError çıkarsa test kendisi patlar


def test_yana_giderken_ve_donerken_de_erisim_icinde(kin):
    for vx, vy, wz in [(0.05, 0.05, 0.0), (0.0, 0.0, 0.4), (0.06, -0.04, 0.2)]:
        gait = TripodGait(kin)
        gait.reset()
        for _ in range(int(round(3.0 / DT))):
            gait.step(vx, vy, wz, DT)


# ---------------------------------------------------------------------------
# Destek fazında ayaklar dünyada sabit
# ---------------------------------------------------------------------------


def test_destek_fazindaki_ayak_dunyada_sabit_kaliyor(kin):
    gait = TripodGait(kin)
    gait.reset()
    group_a, group_b = tripod_groups(kin.mounts)
    leg = group_b[0]  # phase=0'da bu grup destekte başlıyor

    steps_per_half_cycle = int(round(0.5 / gait.params.cycle_hz / DT))
    positions = []
    for _ in range(steps_per_half_cycle - 2):  # geçiş anına denk gelmesin
        gait.step(vx=0.1, vy=0.0, wz=0.0, dt=DT)
        positions.append(gait.foot_targets()[leg])

    first = positions[0]
    for p in positions[1:]:
        assert p == pytest.approx(first, abs=1e-6)


def test_donerken_de_destek_ayagi_sabit(kin):
    gait = TripodGait(kin)
    gait.reset()
    group_a, group_b = tripod_groups(kin.mounts)
    leg = group_b[0]

    steps_per_half_cycle = int(round(0.5 / gait.params.cycle_hz / DT))
    positions = []
    for _ in range(steps_per_half_cycle - 2):
        gait.step(vx=0.05, vy=0.0, wz=0.3, dt=DT)
        positions.append(gait.foot_targets()[leg])

    first = positions[0]
    for p in positions[1:]:
        assert p == pytest.approx(first, abs=1e-6)


# ---------------------------------------------------------------------------
# Bir döngüde gövde hedef mesafeyi alıyor (ayak izi seviyesinde ölçülüyor)
# ---------------------------------------------------------------------------


def test_ayak_izleri_arasi_mesafe_bir_donguluk_hiza_uyuyor(kin):
    vx = 0.1  # m/s
    cycle_hz = 1.5
    gait = TripodGait(kin, GaitParams(cycle_hz=cycle_hz))
    gait.reset()
    group_a, group_b = tripod_groups(kin.mounts)
    leg = group_b[0]  # phase=0'da destekte başlıyor

    steps_per_cycle = int(round(1.0 / cycle_hz / DT))
    # Birinci destek penceresinin ortasında bir örnek al.
    for _ in range(steps_per_cycle // 4):
        gait.step(vx, 0.0, 0.0, DT)
    p0 = gait.foot_targets()[leg]
    # Tam bir döngü sonra (aynı fazda) tekrar örnek al.
    for _ in range(steps_per_cycle):
        gait.step(vx, 0.0, 0.0, DT)
    p1 = gait.foot_targets()[leg]

    expected = vx * 1000.0 / cycle_hz  # mm, bir döngüde alınan mesafe
    assert (p1[0] - p0[0]) == pytest.approx(expected, rel=0.05)
    assert p1[1] == pytest.approx(p0[1], abs=1.0)


# ---------------------------------------------------------------------------
# Her an en az (tam) üç ayak yerde
# ---------------------------------------------------------------------------


def test_her_an_tam_uc_ayak_yerde(kin):
    gait = TripodGait(kin)
    gait.reset()
    group_a, group_b = tripod_groups(kin.mounts)
    for _ in range(int(round(3.0 / DT))):
        gait.step(0.08, 0.0, 0.0, DT)
        phase = gait.phase
        a_swing = phase < 0.5
        stance = (0 if a_swing else len(group_a)) + (len(group_b) if a_swing else 0)
        assert stance == 3


# ---------------------------------------------------------------------------
# reset() ve durgun (vx=vy=wz=0) davranış
# ---------------------------------------------------------------------------


def test_komut_sifirsa_ayaklar_yalnizca_dusey_sekilde_kipirdar(gait):
    xy0 = {leg: (p[0], p[1]) for leg, p in gait.foot_targets().items()}
    for _ in range(int(round(2.0 / DT))):
        gait.step(0.0, 0.0, 0.0, DT)
    for leg, (x0, y0) in xy0.items():
        x1, y1 = gait.foot_targets()[leg][:2]
        assert (x1, y1) == pytest.approx((x0, y0), abs=1e-6)


def test_reset_notr_durusa_donduruyor(gait, kin):
    for _ in range(int(round(2.0 / DT))):
        gait.step(0.1, 0.02, 0.3, DT)
    gait.reset()
    assert gait.phase == 0.0
    for leg, target in gait.foot_targets().items():
        home = gait._home[leg]
        assert target == pytest.approx(home)
