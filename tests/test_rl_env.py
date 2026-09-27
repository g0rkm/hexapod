"""hexapod_rl.env — Gymnasium ortamı. Yalnız gz.sim ve gymnasium olan yerde koşar
(WSL: source /opt/ros/lyrical/setup.bash; source ~/hexapod_venv/bin/activate).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

pytest.importorskip("gz.sim", reason="gz.sim yok (ROS 2 Lyrical kurulu Linux gerekir)")
pytest.importorskip("gymnasium", reason="gymnasium yok (tools/wsl/rl_kurulum.sh)")

from hexapod_rl.env import HexapodEnv  # noqa: E402
from hexapod_rl.task import OBS_SIZE  # noqa: E402


@pytest.fixture(scope="module")
def env(tmp_path_factory):
    e = HexapodEnv(workdir=tmp_path_factory.mktemp("env"))
    yield e
    e.close()


def test_sb3_denetimi(env):
    sb3 = pytest.importorskip("stable_baselines3")
    from stable_baselines3.common.env_checker import check_env
    check_env(env, warn=True, skip_render_check=True)
    assert sb3.__version__


def test_reset_sonrasi_ayakta_duruyor(env):
    obs, info = env.reset(seed=0)
    assert obs.shape == (OBS_SIZE,) and obs.dtype == np.float32
    assert env._state.base_pos[2] == pytest.approx(env.task.stand_height_mm / 1000, abs=0.003)
    assert all(env._state.foot_contact)
    assert obs[0:3] == pytest.approx([0.0, 0.0, -1.0], abs=0.01)


def test_sifir_eylemle_devrilmeden_durur(env):
    env.reset(seed=1, options={"command": (0.0, 0.0, 0.0)})
    for _ in range(100):  # 2 s
        obs, r, terminated, truncated, info = env.step(np.zeros(18, dtype=np.float32))
        assert not terminated
        assert np.isfinite(r) and np.all(np.isfinite(obs))
    assert info["base_pos"][2] == pytest.approx(0.100, abs=0.003)
    # hız komutu 0, robot duruyor: hız izleme ödülü neredeyse tam
    assert info["reward_terms"]["lin_vel"] == pytest.approx(env.task.w["lin_vel"], abs=0.05)


def test_ayni_tohum_ayni_bolum(env):
    rng = np.random.default_rng(0)
    actions = rng.uniform(-0.3, 0.3, size=(25, 18)).astype(np.float32)
    runs = []
    for _ in range(2):
        obs, info = env.reset(seed=42)
        seq = [obs]
        for a in actions:
            obs, *_ = env.step(a)
            seq.append(obs)
        runs.append((info["command"], np.stack(seq)))
    assert runs[0][0] == runs[1][0]
    np.testing.assert_allclose(runs[0][1], runs[1][1], atol=1e-6)


def test_bolum_suresi_dolunca_kesilir(env):
    env.reset(seed=2)
    env._steps = env.max_steps - 1
    *_, truncated, _ = env.step(np.zeros(18, dtype=np.float32))
    assert truncated


def test_gosterim_duz_yurur(env):
    """Taklit edilen gösterim (demo.py) ortamda gerçekten dümdüz yürümeli;
    ödül v4'te izleme terimlerinin çoğunu almalı."""
    import math

    from hexapod_rl.pretrain import demo_for

    demo = demo_for(env)
    cmd = (0.1, 0.0, 0.0)
    env.reset(seed=2, options={"command": cmd})
    x0 = env._state.base_pos[0]
    q = env._state.base_quat
    yaw0 = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
    yaw_terms, n = 0.0, int(round(4.0 / env.dt))
    for _ in range(n):
        _, _, terminated, _, info = env.step(np.asarray(demo.action(env._phase, cmd)))
        assert not terminated
        yaw_terms += info["reward_terms"]["yaw_rate"]
    q = env._state.base_quat
    yaw1 = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
    assert env._state.base_pos[0] - x0 > 0.7 * cmd[0] * 4.0
    assert abs(math.degrees(yaw1 - yaw0)) < 5.0
    assert yaw_terms / n > 0.9 * env.task.w["yaw_rate"]


def test_alan_rastgelelestirme(tmp_path_factory):
    from hexapod_rl.task import Randomization, TaskConfig

    r = Randomization()
    e = HexapodEnv(task=TaskConfig(randomization=r), workdir=tmp_path_factory.mktemp("dr"))
    try:
        seen = []
        for seed in (3, 4, 3):
            _, info = e.reset(seed=seed)
            d = info["dynamics"]
            assert r.servo_strength[0] <= d["servo_strength"] <= r.servo_strength[1]
            assert r.servo_stiffness[0] <= d["servo_stiffness"] <= r.servo_stiffness[1]
            assert 0.0 <= d["latency_ms"] <= r.latency_ms[1] + 1.0
            seen.append(d)
        assert seen[0] == seen[2] and seen[0] != seen[1]    # tohum aynı -> aynı dinamik
        e.reset(seed=5)
        pushes = 0
        for _ in range(int(round(6.0 / e.dt))):             # 6 s: en az bir itme gelmeli
            before = e._next_push
            e.step(np.zeros(18, dtype=np.float32))
            pushes += e._next_push != before
        assert pushes >= 1
    finally:
        e.close()


def _box_ground(top_z: float, pitch: float = 0.0) -> str:
    """Üst yüzü orijinden geçen (+top_z), y ekseni etrafında pitch kadar eğik kutu.
    pitch > 0: zemin +x yönünde alçalır, yüzey z = top_z - tan(pitch) x."""
    cx, cz = -0.1 * math.sin(pitch), top_z - 0.1 * math.cos(pitch)
    return f"""<model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <pose>{cx} 0 {cz} 0 {pitch} 0</pose>
          <geometry><box><size>6 6 0.2</size></box></geometry>
        </collision>
      </link>
    </model>"""


def test_zemin_sdf_ile_kurulur(tmp_path_factory):
    """S5'in zeminleri buradan girer: düz zemin yerine verilen statik model +
    yüzey yüksekliği. Burada üst yüzü z=3 cm'de bir kutu: robot zemine göre
    doğmalı, zeminin ~100 mm üstünde durmalı, altı ayağı da yerde sayılmalı."""
    top = 0.03
    e = HexapodEnv(workdir=tmp_path_factory.mktemp("zemin"), terrain_sdf=_box_ground(top),
                   terrain_height=lambda x, y: top)
    try:
        e.reset(seed=0)
        s = e._state
        assert s.ground_z == pytest.approx(top)
        assert s.base_pos[2] == pytest.approx(top + e.task.stand_height_mm / 1000, abs=0.004)
        assert all(s.foot_contact)
        for _ in range(50):
            _, _, terminated, _, info = e.step(np.zeros(18, dtype=np.float32))
            assert not terminated
        assert info["reward_terms"]["height"] > -1e-3
    finally:
        e.close()


def test_egimli_zeminde_temas_ve_yukseklik(tmp_path_factory):
    """10° eğimde ayakta: altı ayak yerde (dikey boşluk 0.1 mm), gövde yüzeye dik
    100 mm'de yani dikeyde 100/cos 10° = 101.5 mm; eğimle birlikte ~10° yatık."""
    pitch = math.radians(10.0)
    e = HexapodEnv(workdir=tmp_path_factory.mktemp("egim"), terrain_sdf=_box_ground(0.0, pitch),
                   terrain_height=lambda x, y: -math.tan(pitch) * x)
    try:
        e.reset(seed=0)
        s = e._state
        assert all(s.foot_contact)
        assert s.height_above_ground() == pytest.approx(0.1 / math.cos(pitch), abs=0.004)
        tilt = math.degrees(math.acos(-s.gravity_in_base()[2]))
        assert tilt == pytest.approx(10.0, abs=1.5)
    finally:
        e.close()


def test_zemin_yuksekligi_olmadan_zemin_reddedilir():
    """Yüksekliği bilinmeyen zeminde temas ölçülemez; z=0 varsayılmaz."""
    with pytest.raises(ValueError, match="terrain_height"):
        HexapodEnv(terrain_sdf=_box_ground(0.0))
    with pytest.raises(ValueError, match="terrain_height"):
        HexapodEnv(terrain_height=lambda x, y: 0.0)


def test_artik_eylem_modunda_sifir_eylem_tripod(tmp_path_factory):
    from hexapod_rl.task import TaskConfig

    e = HexapodEnv(task=TaskConfig(action_mode="residual"),
                   workdir=tmp_path_factory.mktemp("artik"))
    try:
        e.reset(seed=0, options={"command": (0.1, 0.0, 0.0)})
        x0 = e._state.base_pos[0]
        for _ in range(int(round(4.0 / e.dt))):
            _, _, terminated, _, info = e.step(np.zeros(18, dtype=np.float32))
            assert not terminated
            assert info["reward_terms"]["residual"] == 0.0
        assert e._state.base_pos[0] - x0 > 0.8 * 0.1 * 4.0
    finally:
        e.close()


class _SifirDuzeltme:
    """Artık eylem modunda eylem 0 = tripod'un kendisi."""

    def predict(self, obs, deterministic=True):
        return np.zeros(18, dtype=np.float32), None


def test_olcum_ayni_ortamda_tekrarlanir_ve_donus_olculur(tmp_path_factory):
    """Eğitimde ara kayıt seçimi tek bir ortamı yeniden kullanır; sonuç yeni
    ortamdakiyle aynı olmalı. Yerinde dönüş komutunda tripod gerçekten döner."""
    from hexapod_rl.evaluate import evaluate
    from hexapod_rl.task import TaskConfig

    task = TaskConfig(action_mode="residual")
    e = HexapodEnv(task=task, workdir=tmp_path_factory.mktemp("olcum"))
    try:
        a = evaluate(_SifirDuzeltme(), 3.0, vx=0.0, wz=0.4, env=e)
        evaluate(_SifirDuzeltme(), 1.0, vx=0.1, env=e)          # araya başka bir ölçüm
        b = evaluate(_SifirDuzeltme(), 3.0, vx=0.0, wz=0.4, env=e)
    finally:
        e.close()
    c = evaluate(_SifirDuzeltme(), 3.0, vx=0.0, wz=0.4, task=task)
    for k in a:
        assert a[k] == pytest.approx(b[k], abs=1e-9) and a[k] == pytest.approx(c[k], abs=1e-9)
    assert a["komut_wz"] == 0.4 and not a["devrildi"]
    assert a["donus_hizi_rad_s"] > 0.5 * 0.4
    assert abs(a["govde_vx_m_s"]) < 0.02 and abs(a["govde_vy_m_s"]) < 0.02


def test_en_iyi_ara_kayit_saklanir(tmp_path):
    pytest.importorskip("stable_baselines3")
    import csv

    from stable_baselines3 import PPO
    from stable_baselines3.common.logger import configure

    from hexapod_rl.pretrain import _spaces_only_env
    from hexapod_rl.task import TaskConfig
    from hexapod_rl.train import PPO_KWARGS, best_checkpoint_callback

    model = PPO("MlpPolicy", _spaces_only_env(), device="cpu", seed=0, verbose=0, **PPO_KWARGS)
    model.set_logger(configure(None, []))
    cb = best_checkpoint_callback(1, tmp_path, TaskConfig(action_mode="residual"), seconds=1.0)
    cb.init_callback(model)
    cb.measure()
    cb.best = cb.best + 1.0          # ikinci ölçüm daha kötü sayılsın: dosya değişmemeli
    stamp = (tmp_path / "best_model.zip").stat().st_mtime_ns
    cb.measure()
    cb.on_training_end()
    rows = list(csv.DictReader((tmp_path / "ara_degerlendirme.csv").open(encoding="utf-8")))
    assert len(rows) == 2 and len(rows[0]) == 3 + 3             # adım, skor, devrilen + 3 komut
    assert (tmp_path / "best_model.zip").stat().st_mtime_ns == stamp
    assert cb.best_step == 0


def test_agir_govde_servolari_daha_cok_yukler(tmp_path_factory):
    """Kütle rastgeleleştirmesi fiziğe gerçekten giriyor: aynı duruşu tutmak için
    servolar ağır gövdede daha çok tork uygular (gövde 0.70 -> 1.12 kg, robot
    2.13 -> 2.55 kg: +%20); ortamın dynamics bilgisi çarpanı söyler."""
    loads = []
    for scale in (1.0, 1.6):
        e = HexapodEnv(workdir=tmp_path_factory.mktemp(f"kutle{scale}"), body_mass_scale=scale)
        try:
            _, info = e.reset(seed=0)
            assert info["dynamics"]["body_mass_scale"] == scale
            for _ in range(50):
                e.step(np.zeros(18, dtype=np.float32))
            loads.append(sum(abs(t) for t in e._state.joint_effort))
        finally:
            e.close()
    assert loads[1] > 1.1 * loads[0]


def test_cukurda_dogar_ve_ayakta_durur(tmp_path_factory):
    """Zeminli eğitim denemesinin çukuru: robot çukurun tabanında (z=0) doğar,
    ayakları basamaklara değmez, altısı da yerde."""
    from hexapod_rl.terrain_probe import pit

    sdf, height = pit(0.045)
    e = HexapodEnv(workdir=tmp_path_factory.mktemp("cukur"), terrain_sdf=sdf,
                   terrain_height=height)
    try:
        e.reset(seed=0)
        assert all(e._state.foot_contact) and e._state.ground_z == 0.0
        assert e._state.base_pos[2] == pytest.approx(e.task.stand_height_mm / 1000, abs=0.004)
    finally:
        e.close()


def test_engebede_dogar_ve_ayakta_durur(tmp_path_factory):
    """Engebe (rastgele bloklar): robot ayaklarının altındaki en yüksek bloğa
    göre doğar, devrilmeden oturur, gövde zemine göre ~100 mm'de."""
    from hexapod_rl.terrain_probe import rough

    sdf, height = rough(0.04, seed=5)
    e = HexapodEnv(workdir=tmp_path_factory.mktemp("engebe"), terrain_sdf=sdf,
                   terrain_height=height)
    try:
        e.reset(seed=0)
        for _ in range(25):
            _, _, terminated, _, _ = e.step(np.zeros(18, dtype=np.float32))
            assert not terminated
        assert e._state.height_above_ground() == pytest.approx(0.1, abs=0.03)
        assert sum(e._state.foot_contact) >= 3
    finally:
        e.close()


def test_ogrenilmis_ayak_kaldirma_salinimi_degistirir(tmp_path_factory):
    """Eylem 19 boyutlu; son eylem -1 (20 mm) ile +1 (60 mm) arasında salınımdaki
    ayağın yerden en çok yüksekliği ~40 mm değişmeli."""
    from hexapod_rl.task import TaskConfig

    e = HexapodEnv(task=TaskConfig(action_mode="residual", lift_action=(20.0, 60.0)),
                   workdir=tmp_path_factory.mktemp("kaldirma"))
    try:
        assert e.action_space.shape == (19,)
        peaks = []
        for lift_a in (-1.0, 1.0):
            e.reset(seed=0, options={"command": (0.05, 0.0, 0.0)})
            a = np.zeros(19, dtype=np.float32)
            a[-1] = lift_a
            peak = 0.0
            for _ in range(int(round(1.5 / e.dt))):          # iki adım döngüsü
                _, _, terminated, _, info = e.step(a)
                assert not terminated
                peak = max(peak, max(f[2] for f in e._state.foot_pos))
            peaks.append(peak)
            assert info["lift_mm"] == pytest.approx(20.0 if lift_a < 0 else 60.0)
        assert peaks[1] - peaks[0] > 0.02, peaks
        # kaldırma salınım boyunca sabit: salınım ortasında eylem değişse de
        # yeni salınıma (öteki grup) kadar ilk adımda seçilen kalır
        e.reset(seed=0, options={"command": (0.05, 0.0, 0.0)})
        a[-1] = -1.0
        assert e.step(a)[4]["lift_mm"] == pytest.approx(20.0)
        a[-1] = 1.0
        while e._phase < 0.5:
            assert e.step(a)[4]["lift_mm"] == pytest.approx(20.0)
        assert e.step(a)[4]["lift_mm"] == pytest.approx(60.0)
    finally:
        e.close()


def test_bozulmalar(tmp_path_factory):
    """Sabit bozulmalar (dayanıklılık taraması): kalibrasyon ofseti eklemi kaydırır
    ama gözlem komutu görür; eğik IMU gözlemdeki yerçekimini döndürür; kontrol
    adımından uzun gecikmede komut servoya d adım sonra ulaşır, gözlem hemen görür;
    zayıf servo çarpanı dinamiğe yazılır."""
    from hexapod_rl.env import Perturbation

    ref = HexapodEnv(workdir=tmp_path_factory.mktemp("bozulma_ref"))
    offsets = [0.0] * 18
    offsets[1] = 5.0                                          # bir femur +5°
    e = HexapodEnv(workdir=tmp_path_factory.mktemp("bozulma"),
                   perturbation=Perturbation(joint_offset_deg=tuple(offsets),
                                             imu_tilt_deg=(0.0, 10.0), delay_ms=60.0,
                                             servo_strength=0.7))
    try:
        obs_ref, _ = ref.reset(seed=0)
        obs, info = e.reset(seed=0)
        assert info["dynamics"]["servo_strength"] == pytest.approx(0.7)
        assert info["dynamics"]["latency_ms"] == pytest.approx(60.0)
        assert e._delay_steps == 3 and e.sim.latency_steps == 0
        # ofset: eklem komut + ofset'e yakın, gözlemdeki eklem hedefleri aynı
        err = e._state.joint_pos[1] - e._state.joint_target[1]
        assert math.degrees(err) == pytest.approx(5.0, abs=1.5)
        np.testing.assert_allclose(obs[6:24], obs_ref[6:24], atol=1e-6)
        # eğik IMU (pitch 10°): düz duran gövdede yerçekimi x bileşeni sin(10°)
        assert abs(obs[0]) == pytest.approx(math.sin(math.radians(10.0)), abs=0.02)
        assert np.linalg.norm(obs[0:3]) == pytest.approx(1.0, abs=0.01)
        # gecikme: eylem gözlemde hemen, servoda 3 adım sonra
        a = np.zeros(18, dtype=np.float32)
        a[1] = 0.5
        before = e._state.joint_target[1]
        obs, *_ = e.step(a)
        assert obs[6 + 1] == pytest.approx(0.5, abs=1e-5)
        for _ in range(2):
            assert e._state.joint_target[1] == pytest.approx(before)
            e.step(a)
        assert e._state.joint_target[1] == pytest.approx(before)
        e.step(a)
        assert e._state.joint_target[1] == pytest.approx(before + 0.5 * e.task.action_scale)
    finally:
        e.close()
        ref.close()


def test_bozulmasiz_ortam_eskisiyle_ayni(tmp_path_factory):
    """Perturbation() (varsayılan) rastgeleleştirmeli ortamda dinamiği ve bölümü
    değiştirmez; gecikme eskisi gibi kontrol adımının içinde kalır."""
    from hexapod_rl.env import Perturbation
    from hexapod_rl.task import Randomization, TaskConfig

    task = TaskConfig(randomization=Randomization())
    a = HexapodEnv(task=task, workdir=tmp_path_factory.mktemp("b0"))
    b = HexapodEnv(task=task, workdir=tmp_path_factory.mktemp("b1"), perturbation=Perturbation())
    try:
        rng = np.random.default_rng(1)
        actions = rng.uniform(-0.3, 0.3, size=(20, 18)).astype(np.float32)
        for seed in (3, 4):
            oa, ia = a.reset(seed=seed)
            ob, ib = b.reset(seed=seed)
            assert ia["dynamics"] == ib["dynamics"]
            assert a._delay_steps == 0 and a.sim.latency_steps < a.sim.steps_per_action
            for act in actions:
                oa, *_ = a.step(act)
                ob, *_ = b.step(act)
            np.testing.assert_array_equal(oa, ob)
    finally:
        a.close()
        b.close()


def test_mufredat_seviye_degisir_ve_zemin_yeniden_kurulur(tmp_path_factory):
    """Bölüm sonunda doğduğu yerden yeterince uzaklaştıysa zorlaşır, devrildiyse
    kolaylaşır; zemin gerçekten değişir (yaylada gövde yüksekliği) ve kalibrasyon
    ofseti yeni dünyada korunur; en zoru geçince aralıkta bir seviyeye döner."""
    from functools import partial

    from hexapod_rl.env import CURRICULUM_PROMOTE_M, Perturbation
    from hexapod_rl.terrain_probe import plateau

    levels = [partial(plateau, 0.01), partial(plateau, 0.05)]
    offsets = tuple(0.5 * i for i in range(18))
    e = HexapodEnv(workdir=tmp_path_factory.mktemp("mufredat"), terrain_levels=levels,
                   perturbation=Perturbation(joint_offset_deg=offsets))
    try:
        _, info = e.reset(seed=0, options={"command": (0.1, 0.0, 0.0)})
        assert info["terrain_level"] == 0
        z0 = e._state.base_pos[2]
        e.step(np.zeros(18, dtype=np.float32))
        e._max_disp = CURRICULUM_PROMOTE_M + 0.01           # geçti sayılsın
        _, info = e.reset(seed=0, options={"command": (0.1, 0.0, 0.0)})
        assert info["terrain_level"] == 1
        assert e._state.base_pos[2] - z0 == pytest.approx(0.04, abs=0.005)   # 10 -> 50 mm yayla
        np.testing.assert_allclose(e.sim.joint_offset, [math.radians(v) for v in offsets])
        e.step(np.zeros(18, dtype=np.float32))
        e._max_disp = CURRICULUM_PROMOTE_M + 0.01           # en zoru geçti: aralıkta kalır
        _, info = e.reset(seed=0, options={"command": (0.1, 0.0, 0.0)})
        assert info["terrain_level"] in (0, 1)
        e.set_level(1)
        e.reset(seed=0, options={"command": (0.1, 0.0, 0.0)})
        e.step(np.zeros(18, dtype=np.float32))
        e._fell = True                                      # devrildi: kolaylaşır
        _, info = e.reset(seed=0, options={"command": (0.1, 0.0, 0.0)})
        assert info["terrain_level"] == 0
        for _ in range(50):                                 # 1 s, ~0 ilerleme, 0.1 m/s istendi
            e.step(np.zeros(18, dtype=np.float32))
        _, info = e.reset(seed=0, options={"command": (0.0, 0.0, 0.4)})
        assert info["terrain_level"] == 0                   # en kolayda kalır
        for _ in range(50):                                 # yerinde dönüş: karar yok
            e.step(np.zeros(18, dtype=np.float32))
        e.set_level(1)
        e._steps, e._max_disp, e._fell = 50, 0.0, False
        _, info = e.reset(seed=0)
        assert info["terrain_level"] == 1
    finally:
        e.close()


def _sockets() -> int:
    import os
    n = 0
    for fd in os.listdir("/proc/self/fd"):
        try:
            n += os.readlink(f"/proc/self/fd/{fd}").startswith("socket")
        except OSError:
            pass
    return n


def test_dunya_yeniden_kurulurken_soket_sizmaz(tmp_path_factory):
    """Aynı makinede başka bir süreç içi Gazebo varken dünyayı yeniden kurmak
    soket sızdırmaz. Keşif portları ortak olunca her kurmada öteki süreçlere
    ~32 soket açılıyordu (v23, 2.5M adımda "Too many open files")."""
    import subprocess
    import sys
    from functools import partial

    from hexapod_rl.sim import discovery_ports
    from hexapod_rl.terrain_probe import pit

    assert discovery_ports(123) != discovery_ports(124)
    other = subprocess.Popen(
        [sys.executable, "-c",
         "import time; from hexapod_rl.env import HexapodEnv; e = HexapodEnv(); "
         "e.reset(seed=0); print('hazır', flush=True); time.sleep(60)"],
        stdout=subprocess.PIPE, text=True)
    e = HexapodEnv(workdir=tmp_path_factory.mktemp("soket"),
                   terrain_levels=[partial(pit, 0.02), partial(pit, 0.04)])
    try:
        assert other.stdout.readline().strip() == "hazır"
        e.reset(seed=0)
        before = _sockets()
        for i in range(6):
            e.set_level((i + 1) % 2)
            e.reset(seed=0)
        assert _sockets() == before
    finally:
        e.close()
        other.kill()
        other.wait()
