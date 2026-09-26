"""hexapod_rl.task — gözlem, eylem, ödül, devrilme. Saf Python, her yerde koşar."""

from __future__ import annotations

import math

import pytest

from hexapod_rl.state import SimState
from hexapod_rl.task import (
    ACTION_SIZE,
    OBS_SIZE,
    TaskConfig,
    action_to_targets,
    fallen,
    observation,
    reward,
)

LEVEL = (1.0, 0.0, 0.0, 0.0)


def state(z=0.1, quat=LEVEL, lin=(0.0, 0.0, 0.0), ang=(0.0, 0.0, 0.0), targets=None,
          vel=None, effort=None) -> SimState:
    return SimState(
        time=0.0, joint_pos=tuple([0.0] * 18), joint_vel=tuple(vel or [0.0] * 18),
        joint_target=tuple(targets or [0.0] * 18), joint_effort=tuple(effort or [0.0] * 18),
        base_pos=(0.0, 0.0, z), base_quat=quat,
        base_lin_vel=lin, base_ang_vel=ang, foot_pos=tuple([(0.0, 0.0, 0.0)] * 6),
        foot_contact=tuple([True] * 6),
    )


def test_eylem_hedefe_olcekli_ve_kirpik():
    default = [0.1] * 18
    limits = [(-0.5, 0.5)] * 18
    t = action_to_targets([1.0] * 18, default, 0.3, limits)
    assert t[0] == pytest.approx(0.4)
    t = action_to_targets([5.0] * 18, default, 0.3, limits)   # eylem [-1,1]'e kırpılır
    assert t[0] == pytest.approx(0.4)
    t = action_to_targets([1.0] * 18, default, 1.0, limits)   # hedef limite kırpılır
    assert t[0] == pytest.approx(0.5)
    with pytest.raises(ValueError):
        action_to_targets([0.0] * 17, default, 0.3, limits)


def test_gozlem_boyutu_ve_duz_govde():
    default = [0.2] * 18
    obs = observation(state(targets=[0.2] * 18), (0.1, 0.0, 0.0), 0.0, default, 0.5)
    assert len(obs) == OBS_SIZE == 29
    assert obs[0:3] == pytest.approx([0.0, 0.0, -1.0])     # yerçekimi aşağı
    assert obs[6:24] == pytest.approx([0.0] * 18)          # hedef = varsayılan
    assert obs[24:27] == pytest.approx([0.1, 0.0, 0.0])    # komut
    assert obs[27:29] == pytest.approx([0.0, 1.0])         # saat sin, cos


def test_gozlemde_olculen_aci_ve_temas_yok():
    """Gerçek robotta ölçülen eklem açısı ve ayak teması yok (docs/ARAYUZ.md)."""
    a = state()
    b = SimState(**{**a.__dict__, "joint_pos": tuple([1.0] * 18),
                    "foot_contact": tuple([False] * 6)})
    args = ((0.1, 0.0, 0.0), 0.3, [0.0] * 18, 0.5)
    assert observation(a, *args) == observation(b, *args)


def test_tam_izleme_en_yuksek_hiz_odulu():
    cfg = TaskConfig()
    _, perfect = reward(state(lin=(0.1, 0.0, 0.0)), [0.0] * 18, [0.0] * 18,
                        (0.1, 0.0, 0.0), cfg, False)
    _, still = reward(state(), [0.0] * 18, [0.0] * 18, (0.1, 0.0, 0.0), cfg, False)
    assert perfect["lin_vel"] == pytest.approx(cfg.w["lin_vel"])
    assert still["lin_vel"] < perfect["lin_vel"]


def test_cezalar_isaretli():
    cfg = TaskConfig()
    _, terms = reward(state(z=0.08, vel=[1.0] * 18, effort=[-0.5] * 18), [1.0] * 18, [0.0] * 18,
                      (0.0, 0.0, 0.0), cfg, True)
    assert terms["power"] == pytest.approx(cfg.w["power"] * 18 * 0.5)  # |tork x hız|, işaretsiz
    for k in ("height", "power", "action_rate", "fall"):
        assert terms[k] < 0, k


def test_devrilme():
    cfg = TaskConfig()
    assert not fallen(state(z=0.1), cfg)
    assert fallen(state(z=0.03), cfg)                      # gövde yerde
    a = math.radians(50)                                   # 50° yatık
    assert fallen(state(quat=(math.cos(a / 2), math.sin(a / 2), 0.0, 0.0)), cfg)
    a = math.radians(30)
    assert not fallen(state(quat=(math.cos(a / 2), math.sin(a / 2), 0.0, 0.0)), cfg)


def test_yukseklik_altindaki_zemine_gore():
    """Zeminli dünyada (S5) gövde yüksekliği dünya z'sine değil zemine göre ölçülür."""
    cfg = TaskConfig()
    yuksekte = SimState(**{**state(z=0.25).__dict__, "ground_z": 0.15})   # 15 cm'lik platform
    assert yuksekte.height_above_ground() == pytest.approx(0.1)
    assert not fallen(yuksekte, cfg)
    _, terms = reward(yuksekte, [0.0] * 18, [0.0] * 18, (0.1, 0.0, 0.0), cfg, False)
    assert terms["height"] == pytest.approx(0.0)
    cukurda = SimState(**{**state(z=0.1).__dict__, "ground_z": 0.07})     # gövde zemine 3 cm
    assert fallen(cukurda, cfg)


def test_eylem_boyutu():
    assert ACTION_SIZE == 18


# --- ödül v2 --------------------------------------------------------------------

from hexapod_rl.task import gait_score, tripod_groups  # noqa: E402


def test_tripod_gruplari_komsu_bacaklari_ayirir():
    # gövde azimutları (robot.yaml'daki gibi): 0:+90 1:+30 2:-30 3:-90 4:-150 5:+150
    yaws = {0: 90, 1: 30, 2: -30, 3: -90, 4: -150, 5: 150}
    a, b = tripod_groups({k: math.radians(v) for k, v in yaws.items()})
    assert set(a) | set(b) == set(range(6)) and len(a) == len(b) == 3
    around = sorted(yaws, key=lambda k: yaws[k])
    for i in range(6):  # gövde etrafında komşu iki bacak farklı grupta
        x, y = around[i], around[(i + 1) % 6]
        assert (x in a) != (y in a)


def test_ritim_puani():
    groups = ((0, 2, 4), (1, 3, 5))
    ideal_ilk = [False, True, False, True, False, True]   # ilk yarı: 0,2,4 havada
    assert gait_score(ideal_ilk, 0.2, groups) == 1.0
    assert gait_score(ideal_ilk, 0.7, groups) == 0.0      # ikinci yarıda tam tersi olmalı
    assert gait_score([True] * 6, 0.2, groups) == 0.5     # hepsi yerde: yalnız destek grubu doğru


def test_yerinde_durmak_yurumekten_az_kazandirir():
    """v1'in hatası: durmak neredeyse yürümek kadar kazandırıyordu."""
    cfg = TaskConfig()
    cmd = (0.1, 0.0, 0.0)
    duran, _ = reward(state(), [0.0] * 18, [0.0] * 18, cmd, cfg, False, phase=0.2)
    yuruyen_temas = tuple([False, True, False, True, False, True])
    yuruyen = SimState(**{**state(lin=(0.1, 0.0, 0.0)).__dict__, "foot_contact": yuruyen_temas})
    giden, terms = reward(yuruyen, [0.0] * 18, [0.0] * 18, cmd, cfg, False, phase=0.2)
    assert terms["progress"] == pytest.approx(cfg.w["progress"] * 0.1)
    assert giden > duran + 1.0


def test_ilerleme_komutla_sinirli_ve_geri_gitmek_sifir():
    cfg = TaskConfig()
    cmd = (0.1, 0.0, 0.0)
    _, hizli = reward(state(lin=(0.3, 0.0, 0.0)), [0.0] * 18, [0.0] * 18, cmd, cfg, False)
    _, geri = reward(state(lin=(-0.1, 0.0, 0.0)), [0.0] * 18, [0.0] * 18, cmd, cfg, False)
    assert hizli["progress"] == pytest.approx(cfg.w["progress"] * 0.1)
    assert geri["progress"] == 0.0


# --- ödül v3 --------------------------------------------------------------------


def test_donmek_artik_pahali():
    """v2'de ~12°/s dönüş neredeyse bedavaydı; v3'te belirgin puan kaybettirmeli."""
    cfg = TaskConfig()
    cmd = (0.1, 0.0, 0.0)
    _, duz = reward(state(lin=(0.1, 0.0, 0.0)), [0.0] * 18, [0.0] * 18, cmd, cfg, False)
    _, donen = reward(state(lin=(0.1, 0.0, 0.0), ang=(0.0, 0.0, -0.21)), [0.0] * 18,
                      [0.0] * 18, cmd, cfg, False)
    assert duz["yaw_rate"] - donen["yaw_rate"] > 0.5


def test_farkli_hizlar_ayirt_ediliyor():
    """0.087 m/s yürürken 0.15 komutu, tam izlemeye göre en az %70 hız izleme puanı kaybettirmeli."""
    cfg = TaskConfig()
    cmd = (0.15, 0.0, 0.0)
    _, tam = reward(state(lin=(0.15, 0.0, 0.0)), [0.0] * 18, [0.0] * 18, cmd, cfg, False)
    _, yavas = reward(state(lin=(0.087, 0.0, 0.0)), [0.0] * 18, [0.0] * 18, cmd, cfg, False)
    assert yavas["lin_vel"] < 0.3 * tam["lin_vel"]


# --- ödül v4 --------------------------------------------------------------------

from hexapod_rl.task import VelocityFilter  # noqa: E402


def test_izleme_suzulmus_hiza_bakar():
    """v4: anlık sarsıntı izleme terimlerini düşürmemeli; ortalama doğruysa tam puan."""
    cfg = TaskConfig()
    cmd = (0.1, 0.0, 0.0)
    sarsilan = state(lin=(0.3, 0.1, 0.0), ang=(0.0, 0.0, 0.8))
    _, anlik = reward(sarsilan, [0.0] * 18, [0.0] * 18, cmd, cfg, False)
    _, suzulmus = reward(sarsilan, [0.0] * 18, [0.0] * 18, cmd, cfg, False,
                         tracked=(0.1, 0.0, 0.0))
    assert anlik["yaw_rate"] < 0.01 and anlik["lin_vel"] < 0.01
    assert suzulmus["yaw_rate"] == pytest.approx(cfg.w["yaw_rate"])
    assert suzulmus["lin_vel"] == pytest.approx(cfg.w["lin_vel"])
    # v5: progress da süzülmüş hıza bakar (anlık 0.3 komutla kırpılıp tam puan alırdı)
    assert suzulmus["progress"] == pytest.approx(cfg.w["progress"] * 0.1)


def test_hiz_suzgeci_ortalamaya_yakinsar_ve_titresimi_bastirir():
    f = VelocityFilter(dt=0.02, tau=0.5)
    ileri = state(lin=(0.1, 0.0, 0.0), ang=(0.0, 0.0, 0.2))
    first = f.update(ileri)
    assert first == pytest.approx((0.1 * 0.04, 0.0, 0.2 * 0.04))
    for _ in range(500):
        f.update(ileri)
    assert f.value == pytest.approx((0.1, 0.0, 0.2), abs=1e-6)
    f.reset()
    for k in range(500):                      # ±0.5 rad/s titreşim, ortalama 0
        f.update(state(ang=(0.0, 0.0, 0.5 if k % 2 else -0.5)))
    assert abs(f.value[2]) < 0.03
    with pytest.raises(ValueError):
        VelocityFilter(dt=0.02, tau=0.0)


# --- ödül v5 ve alan rastgeleleştirme -------------------------------------------


def test_hedefi_asmak_progress_kazandirmaz():
    """v5: süzülmüş hız komutu aşınca progress artmaz, lin_vel düşer; yani hedefi
    aşmak toplamda kaybettirir."""
    cfg = TaskConfig()
    cmd = (0.1, 0.0, 0.0)
    _, tam = reward(state(), [0.0] * 18, [0.0] * 18, cmd, cfg, False, tracked=(0.1, 0.0, 0.0))
    _, asan = reward(state(), [0.0] * 18, [0.0] * 18, cmd, cfg, False, tracked=(0.13, 0.0, 0.0))
    assert asan["progress"] == tam["progress"]
    assert asan["lin_vel"] < tam["lin_vel"]


def test_kucuk_yon_sapmasi_da_kaybettirir():
    """v6: 2°/s'lik sabit dönüş (10 s'de 20°) dönüş teriminin en az %10'unu kaybettirmeli."""
    cfg = TaskConfig()
    cmd = (0.1, 0.0, 0.0)
    _, duz = reward(state(), [0.0] * 18, [0.0] * 18, cmd, cfg, False, tracked=(0.1, 0.0, 0.0))
    _, kayan = reward(state(), [0.0] * 18, [0.0] * 18, cmd, cfg, False,
                      tracked=(0.1, 0.0, math.radians(-2.0)))
    assert kayan["yaw_rate"] < 0.9 * duz["yaw_rate"]


def test_enerji_artik_pahali():
    """v5: 5 W fazladan güç (PPO v4 ile tripod farkı) adım başı en az 0.2 kaybettirmeli."""
    cfg = TaskConfig()
    effort, vel = [1.0] + [0.0] * 17, [5.0] + [0.0] * 17      # 1 N·m x 5 rad/s = 5 W
    _, r = reward(state(effort=effort, vel=vel), [0.0] * 18, [0.0] * 18, (0.1, 0.0, 0.0),
                  cfg, False)
    assert r["power"] <= -0.2


def test_rastgelelestirme_varsayilanda_kapali():
    from hexapod_rl.task import Randomization
    assert TaskConfig().randomization is None
    r = Randomization()
    for lo, hi in (r.servo_strength, r.servo_stiffness, r.latency_ms, r.push_force_n,
                   r.push_every_s):
        assert 0 <= lo <= hi


def test_artik_eylem_cezasi_yalniz_o_modda():
    cmd = (0.1, 0.0, 0.0)
    a = [0.5] * 18
    _, mutlak = reward(state(), a, a, cmd, TaskConfig(), False)
    _, artik = reward(state(), a, a, cmd, TaskConfig(action_mode="residual"), False)
    assert "residual" not in mutlak
    assert artik["residual"] == pytest.approx(TaskConfig().w["residual"] * 0.25)
