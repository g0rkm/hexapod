"""Ters/düz kinematik testleri. Donanım gerektirmez."""

from __future__ import annotations

import itertools
import math
from pathlib import Path

import pytest

from hexapod_driver import RobotConfig
from hexapod_kinematics import (
    BodyPose,
    HexapodKinematics,
    JointAngles,
    LegGeometry,
    ReachError,
    ZERO,
    forward,
    inverse,
    wrap_deg,
)

REAL_CONFIG = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"
GEOM = LegGeometry(coxa=50.0, femur=80.0, tibia=126.6)


def close(a, b, tol=1e-6):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


# ---------------------------------------------------------------------------
# Tek bacak
# ---------------------------------------------------------------------------


def test_sifir_duruşunda_ayak_dogru_yerde():
    # coxa dışarı, femur yatay, tibia dümdüz aşağı
    x, y, z = forward(GEOM, ZERO)
    assert close((x, y, z), (50.0 + 80.0, 0.0, -126.6))


def test_sifir_duruşunun_tersi_sifir_aci():
    a = inverse(GEOM, 130.0, 0.0, -126.6)
    assert close((a.coxa, a.femur, a.tibia), (0.0, 0.0, 0.0))


def test_pozitif_yonler_tanimla_uyusuyor():
    x0, y0, z0 = forward(GEOM, ZERO)
    # coxa +  : yukarıdan bakınca saat yönünün tersine -> +y
    assert forward(GEOM, JointAngles(10, 0, 0))[1] > y0
    # femur + : bacak kalkar -> ayak yükselir
    assert forward(GEOM, JointAngles(0, 10, 0))[2] > z0
    # tibia + : diz açılır -> ayak dışarı gider
    assert forward(GEOM, JointAngles(0, 0, 10))[0] > x0


def test_gidis_donus_tum_calisma_araliginda():
    """IK(FK(q)) == q.

    Geçerli olduğu alan: diz yukarıda (-90 < tibia < 90) ve ayak coxa
    ekseninin dışında. Ayağın coxa ekseninin arkasına düştüğü pozlar
    "bacak 180 derece dönmüş" pozdan ayırt edilemez ve gerçek robotta
    gövdeye çarpar; bunlar test dışı.
    """
    tested = 0
    for q1, q2, q3 in itertools.product(range(-40, 41, 20),
                                        range(-60, 61, 15),
                                        range(-80, 81, 20)):
        psi = math.radians(q2 + q3 - 90)
        radial = GEOM.coxa + GEOM.femur * math.cos(math.radians(q2)) + GEOM.tibia * math.cos(psi)
        if radial <= GEOM.coxa:
            continue  # ayak femur ekleminin gerisinde / gövdenin altında
        tested += 1
        target = forward(GEOM, JointAngles(q1, q2, q3))
        back = inverse(GEOM, *target)
        assert close((back.coxa, back.femur, back.tibia), (q1, q2, q3), tol=1e-6), \
            (q1, q2, q3, back)
    assert tested > 200, tested  # filtre ızgaranın çoğunu atmamalı


def test_uzak_hedef_reddediliyor_kirpilmiyor():
    with pytest.raises(ReachError, match="uzakta"):
        inverse(GEOM, 50.0 + 80.0 + 126.6 + 1.0, 0.0, 0.0)


def test_cok_yakin_hedef_reddediliyor():
    # Femur ekleminin tam üstü: uzaklık 0
    with pytest.raises(ReachError, match="yakında"):
        inverse(GEOM, 50.0, 0.0, 0.0)


def test_erisim_sinirinda_calisiyor():
    # Bacak dümdüz yatay uzanmış: tibia = +90
    a = inverse(GEOM, 50.0 + 80.0 + 126.6, 0.0, 0.0)
    assert close((a.femur, a.tibia), (0.0, 90.0), tol=1e-4)


# ---------------------------------------------------------------------------
# Gövde + altı bacak
# ---------------------------------------------------------------------------


@pytest.fixture
def kin():
    return HexapodKinematics.from_config(RobotConfig.load(REAL_CONFIG))


def test_config_geometrisi_okunuyor(kin):
    assert kin.leg == GEOM
    assert len(kin.mounts) == 6
    for m in kin.mounts.values():
        assert math.hypot(m.x, m.y) == pytest.approx(100.0)
        assert m.z == pytest.approx(-10.05)


def test_bacaklar_govde_cercevesinde_dogru_yerde(kin):
    yaws = {i: round(math.degrees(m.yaw)) for i, m in kin.mounts.items()}
    # robot.yaml'daki tabloyla aynı: 0 sol orta, 1 sol ön, 2 sağ ön, ...
    assert yaws == {0: 90, 1: 30, 2: -30, 3: -90, 4: -150, 5: 150}


def test_sol_bacaklar_sol_tarafta(kin):
    # REP-103: +y sol. "sol" etiketli bacaklar +y tarafında olmalı.
    config = RobotConfig.load(REAL_CONFIG)
    for spec in config.legs:
        y = kin.mounts[spec.id].y
        if spec.label.startswith("sol"):
            assert y > 0, spec.label
        else:
            assert y < 0, spec.label


def test_notr_durusta_butun_eklemler_sifir(kin):
    angles = kin.inverse(kin.neutral_stance())
    for a in angles.values():
        assert close((a.coxa, a.femur, a.tibia), (0, 0, 0), tol=1e-9)


def test_govde_ileri_gidince_ayaklar_geride_kaliyor(kin):
    stance = kin.neutral_stance()
    angles = kin.inverse(stance, BodyPose(x=20.0))
    # Gövde ileri kaydı, ayaklar yerinde: gövdeye göre ayaklar geriye kaydı.
    back = kin.forward(angles)
    for i, p in back.items():
        assert p[0] == pytest.approx(stance[i][0] - 20.0)
        assert p[1] == pytest.approx(stance[i][1])


def test_govde_yukselince_butun_bacaklar_ayni_aciyi_aliyor(kin):
    angles = kin.inverse(kin.neutral_stance(), BodyPose(z=15.0))
    values = {(round(a.coxa, 9), round(a.femur, 9), round(a.tibia, 9))
              for a in angles.values()}
    assert len(values) == 1  # simetri
    (coxa, femur, tibia), = values
    assert coxa == pytest.approx(0.0)
    assert femur < 0  # gövde kalkınca femur aşağı iner


def test_govde_donunce_butun_coxalar_ayni_yone_donuyor(kin):
    angles = kin.inverse(kin.neutral_stance(), BodyPose(yaw=10.0))
    coxas = [a.coxa for a in angles.values()]
    # Gövde sola döndü, ayaklar yerinde: her bacak gövdeye göre sağa döner.
    assert all(c < 0 for c in coxas)
    assert max(coxas) - min(coxas) == pytest.approx(0.0, abs=1e-9)


def test_govde_pozu_gidis_donus(kin):
    """Poz uygulanmış IK sonucu FK'dan geçince aynı dünya noktalarına dönmeli."""
    stance = kin.neutral_stance()
    pose = BodyPose(x=10, y=-5, z=12, roll=4, pitch=-3, yaw=7)
    angles = kin.inverse(stance, pose)
    body = kin.forward(angles)
    r, p, y = (math.radians(v) for v in (pose.roll, pose.pitch, pose.yaw))
    for i, b in body.items():
        # gövde -> dünya: R·b + t
        cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                                  math.sin(p), math.cos(y), math.sin(y))
        R = ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
             (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
             (-sp, cp * sr, cp * cr))
        world = tuple(sum(R[k][j] * b[j] for j in range(3)) + t
                      for k, t in zip(range(3), (pose.x, pose.y, pose.z)))
        assert close(world, stance[i], tol=1e-6)


def test_erisilemeyen_bacak_adiyla_bildiriliyor(kin):
    stance = kin.neutral_stance()
    stance[3] = (stance[3][0], stance[3][1] - 500.0, stance[3][2])
    with pytest.raises(ReachError, match="bacak 3"):
        kin.inverse(stance)


def test_aci_sarma():
    assert wrap_deg(210.0) == pytest.approx(-150.0)
    assert wrap_deg(-190.0) == pytest.approx(170.0)
    assert wrap_deg(180.0) == pytest.approx(180.0)
