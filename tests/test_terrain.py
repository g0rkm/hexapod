"""hexapod_terrain testleri (saf Python, Gazebo gerekmez) — GOREVLER.md S5.

En önemlisi test_sdf_ve_height_birebir_ayni: üretilen SDF geri okunup her
zeminin her yerinde height() ile karşılaştırılır. Bu ikisi ayrışırsa
hexapod_rl ayak temasını, gövde yüksekliğini ve devrilmeyi yanlış hesaplar,
robot havada temas eder ya da zemine gömülür — eğitim sessizce bozulur.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import pytest

from hexapod_terrain import sets, terrain, world


# ---------------------------------------------------------------------------
# Yardımcı: SDF'i geri oku
# ---------------------------------------------------------------------------


def boxes_of(sdf: str) -> list[tuple[tuple[float, ...], tuple[float, ...]]]:
    """SDF'teki çarpışma kutuları: [(poz xyz+rpy, boyut xyz)]."""
    root = ET.fromstring(sdf)
    out = []
    for collision in root.iter("collision"):
        box = collision.find("geometry/box")
        if box is None:
            continue
        pose = collision.find("pose")
        values = [float(v) for v in (pose.text or "").split()] if pose is not None else []
        values += [0.0] * (6 - len(values))
        size = tuple(float(v) for v in (box.find("size").text or "").split())
        out.append((tuple(values), size))
    return out


def surface_from_sdf(sdf: str, x: float, y: float) -> float | None:
    """(x, y) noktasının üstündeki en yüksek kutu yüzeyi; hiçbiri örtmüyorsa None.

    Yalnız dönmemiş kutular için (eğim ayrıca kontrol edilir).
    """
    top = None
    for (px, py, pz, roll, pitch, yaw), (sx, sy, sz) in boxes_of(sdf):
        if max(abs(roll), abs(pitch), abs(yaw)) > 1e-9:
            continue
        if abs(x - px) <= sx / 2 + 1e-9 and abs(y - py) <= sy / 2 + 1e-9:
            z = pz + sz / 2
            top = z if top is None else max(top, z)
    return top


#: Hücre/basamak sınırına denk gelmeyen bir kaydırma: sınırda yüzey dikey bir
#: duvar, iki yükseklik de doğru (terrain.py modül açıklaması). 0.0137 m hiçbir
#: hücre boyunun (0.12, 0.15) ya da basamak yerinin (0.25, 0.3, 0.4) katı değil.
OFFSET = 0.0137


def sample_points(n: int = 17, span: float = 1.6):
    for i in range(n):
        for j in range(n):
            yield (-span + 2 * span * i / (n - 1) + OFFSET,
                   -span + 2 * span * j / (n - 1) + OFFSET)


ALL_TERRAINS = [
    terrain.flat(),
    terrain.flat(mu=0.3),
    terrain.slope(10.0),
    terrain.slope(-20.0),
    terrain.slope(12.0, "y"),
    terrain.slope(-15.0, mu=0.3),
    terrain.step(0.030),
    terrain.step(0.060),
    terrain.step(-0.040),
    terrain.stairs(0.035, 0.25),
    terrain.stairs(0.020, 0.30, count=4),
    terrain.stairs(-0.030, 0.25, count=3),
    terrain.rough(0.040, seed=1),
    terrain.rough(0.060, cell=0.12, seed=7),
    terrain.rough(0.050, seed=2, smooth=0),
    terrain.pit(0.040),
    terrain.plateau(0.050),
]


# ---------------------------------------------------------------------------
# SDF ↔ height tutarlılığı (en kritik test)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("item", ALL_TERRAINS, ids=lambda z: z.label)
def test_sdf_ve_height_birebir_ayni(item):
    if item.params["kind"] == "slope":
        pytest.skip("eğim dönmüş kutu; ayrı test ediliyor")
    for x, y in sample_points():
        top = surface_from_sdf(item.sdf, x, y)
        assert top is not None, f"({x:.2f}, {y:.2f}) hiçbir kutunun üstünde değil"
        assert top == pytest.approx(item.height(x, y), abs=1e-6), \
            f"{item.label}: ({x:.2f}, {y:.2f}) SDF {top} != height {item.height(x, y)}"


@pytest.mark.parametrize("deg,axis", [(10.0, "x"), (-20.0, "x"), (12.0, "y"), (-5.0, "y")])
def test_egimde_sdf_duzlemi_ve_height_ayni(deg, axis):
    """SDF'teki dönmüş kutunun üst yüzeyi ile height aynı düzlem mi.

    Doğrudan açıyı karşılaştırmıyor: SDF sayıları 6 haneye yuvarlanıyor, o yüzden
    fiziksel olarak anlamlı olan ölçülüyor — zeminin her yerinde iki yüzeyin
    yükseklik farkı. 0.1 mm ölçüt: ayak küresi 5 mm, temas toleransı 2 mm.
    """
    item = terrain.slope(deg, axis)
    (pose, size), = boxes_of(item.sdf)
    px, py, pz, roll, pitch, yaw = pose

    # Dönmüş kutunun üst yüzeyi: merkezden yerel +z boyunca yarım kalınlık.
    # rpy (roll, pitch, yaw) -> yerel +z'nin dünyadaki yönü.
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    normal = (sp * cr, -sr, cr * cp)
    top = (px + normal[0] * size[2] / 2,
           py + normal[1] * size[2] / 2,
           pz + normal[2] * size[2] / 2)

    def sdf_surface(x: float, y: float) -> float:
        """Düzlem: normal · (p - top) = 0 -> z çöz."""
        return top[2] - (normal[0] * (x - top[0]) + normal[1] * (y - top[1])) / normal[2]

    for x, y in sample_points(9, span=6.0):
        assert sdf_surface(x, y) == pytest.approx(item.height(x, y), abs=1e-4), \
            f"({x:.2f}, {y:.2f}) SDF düzlemi ile height ayrışıyor"

    # Eğim yönü ve büyüklüğü: deg > 0 ileri/sola doğru ALÇALIR.
    d = 1.0
    drop = item.height(d, 0.0) if axis == "x" else item.height(0.0, d)
    assert drop == pytest.approx(-math.tan(math.radians(deg)) * d, abs=1e-9)
    assert item.height(0.0, 0.0) == 0.0, "orijin zemin seviyesinde"


# ---------------------------------------------------------------------------
# Tohum: aynı tohum aynı zemin
# ---------------------------------------------------------------------------


def test_ayni_tohum_birebir_ayni_zemin():
    a, b = terrain.rough(0.05, seed=42), terrain.rough(0.05, seed=42)
    assert a.sdf == b.sdf
    for x, y in sample_points(7):
        assert a.height(x, y) == b.height(x, y)


def test_farkli_tohum_farkli_zemin():
    a, b = terrain.rough(0.05, seed=1), terrain.rough(0.05, seed=2)
    assert a.sdf != b.sdf


def cell_centers(item):
    """Engebenin bütün hücre merkezleri (seyrek örnekleme en yüksek hücreyi ıskalar)."""
    p = item.params
    cell, x0, y0 = p["cell"], -0.8, -p["width"] / 2
    nx, ny = max(1, round(p["length"] / cell)), max(1, round(p["width"] / cell))
    return [(x0 + (i + 0.5) * cell, y0 + (j + 0.5) * cell)
            for i in range(nx) for j in range(ny)]


def test_engebe_genligi_asilmiyor_ve_kullaniliyor():
    item = terrain.rough(0.06, seed=3)
    tops = [item.height(x, y) for x, y in cell_centers(item)]
    assert min(tops) >= -1e-9
    assert max(tops) == pytest.approx(0.06, abs=1e-6), "en yüksek hücre genliğe eşit olmalı"


def test_yumusatma_komsu_farkini_azaltiyor():
    """smooth=0 beyaz gürültü (komşular tam genlik farklı olabilir); smooth>=1
    dalgalanma. Zorluğun yerel eğimle belirlendiği iddiasının testi."""
    def max_neighbor_jump(item, cell):
        xs = [-0.5 + i * cell for i in range(12)]
        return max(abs(item.height(x + cell, 0.4) - item.height(x, 0.4)) for x in xs)

    rough_white = terrain.rough(0.06, seed=5, smooth=0)
    rough_smooth = terrain.rough(0.06, seed=5, smooth=2)
    assert max_neighbor_jump(rough_smooth, 0.15) < max_neighbor_jump(rough_white, 0.15)


def test_dogus_bolgesi_duz():
    """Robot eğik ya da gömülü doğmasın: |x|,|y| < 0.3 düz olmalı."""
    item = terrain.rough(0.08, seed=9)
    for x, y in sample_points(9, span=0.3):
        assert item.height(x, y) == 0.0


# ---------------------------------------------------------------------------
# Parametreler gerçekten etkiliyor mu
# ---------------------------------------------------------------------------


def test_basamak_yuksekligi_ve_yeri():
    item = terrain.step(0.045, at_x=0.4)
    assert item.height(0.39, 0.0) == 0.0
    assert item.height(0.41, 0.0) == pytest.approx(0.045)
    assert item.height(3.0, 1.0) == pytest.approx(0.045)


def test_inen_basamak_negatif():
    item = terrain.step(-0.04)
    assert item.height(0.0, 0.0) == 0.0
    assert item.height(1.0, 0.0) == pytest.approx(-0.04)


def test_merdiven_basamak_basamak_yukseliyor():
    item = terrain.stairs(0.03, 0.25, count=4, at_x=0.4)
    assert item.height(0.2, 0.0) == 0.0
    for i in range(4):
        x = 0.4 + i * 0.25 + 0.1
        assert item.height(x, 0.0) == pytest.approx(0.03 * (i + 1)), f"{i}. basamak"
    assert item.height(5.0, 0.0) == pytest.approx(0.12), "sahanlık en üstte sürer"


def test_cukur_ve_yayla_ters():
    assert terrain.pit(0.05).height(0.0, 0.0) == 0.0
    assert terrain.pit(0.05).height(1.0, 0.0) == pytest.approx(0.05)
    assert terrain.plateau(0.05).height(0.0, 0.0) == pytest.approx(0.05)
    assert terrain.plateau(0.05).height(2.0, 0.0) == 0.0


def test_surtunme_sdfe_giriyor():
    assert "<mu>0.3</mu>" in terrain.slope(-10.0, mu=0.3).sdf
    assert "<mu>" not in terrain.slope(-10.0).sdf


def test_gecersiz_parametreler_reddediliyor():
    with pytest.raises(ValueError):
        terrain.slope(60.0)
    with pytest.raises(ValueError):
        terrain.rough(0.05, cell=0.0)
    with pytest.raises(ValueError):
        terrain.rough(-0.01)
    with pytest.raises(ValueError):
        terrain.stairs(0.03, run=0.0)
    with pytest.raises(ValueError):
        terrain.pit(-0.01)
    with pytest.raises(ValueError):
        terrain.flat(mu=0.0)


# ---------------------------------------------------------------------------
# Sözleşme: ikili gibi çözülebilmeli (Görkem'in beklediği kullanım)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("item", ALL_TERRAINS, ids=lambda z: z.label)
def test_ikili_gibi_cozulebiliyor(item):
    sdf, height = item
    assert sdf is item.sdf
    assert height(0.0, 0.0) == item.height(0.0, 0.0)
    assert isinstance(sdf, str) and "<model" in sdf
    assert 'name="ground"' in sdf, "hexapod_rl.sim düz zemini bu adla değiştiriyor"
    assert "<static>true</static>" in sdf


@pytest.mark.parametrize("item", ALL_TERRAINS, ids=lambda z: z.label)
def test_sdf_gecerli_xml_ve_sonlu_sayilar(item):
    ET.fromstring(item.sdf)  # bozuksa burada patlar
    assert "nan" not in item.sdf.lower() and "inf" not in item.sdf.lower()


@pytest.mark.parametrize("item", ALL_TERRAINS, ids=lambda z: z.label)
def test_parametreler_kayitli(item):
    assert item.params.get("kind")
    assert item.label


# ---------------------------------------------------------------------------
# Seviyeler ve setler
# ---------------------------------------------------------------------------


def test_her_turun_seviyeleri_var():
    for kind in sets.kinds():
        items = sets.levels(kind)
        assert items, kind
        for item in items:
            assert isinstance(item, terrain.Terrain)


def test_seviyeler_zorlasiyor():
    """Aynı türde sonraki seviye daha zor: yüzeyin en yüksek noktası artmalı."""
    for kind in ("basamak", "çukur", "engebe", "merdiven"):
        peaks = [max(abs(z.height(x, y)) for x, y in sample_points(11, span=1.2))
                 for z in sets.levels(kind)]
        assert peaks == sorted(peaks), f"{kind}: {peaks}"
        assert peaks[-1] > peaks[0], kind


def test_mufredat_basamaklari_buyuyor():
    assert len(sets.difficulty(0)) == 2  # yalnız düz
    assert len(sets.difficulty(3)) == 5
    labels = [z.label for z in sets.difficulty(0)]
    assert labels == ["düz", "düz"]
    # Düz zemin her basamakta var (ders 33/34: düz verim kaybolmasın)
    assert any(z.params["kind"] == "flat" for z in sets.difficulty(8))


def test_olcum_seti_sabit_ve_kapsayici():
    a, b = sets.evaluation_set(), sets.evaluation_set()
    assert [z.label for z in a] == [z.label for z in b]
    kinds = {z.params["kind"] for z in a}
    assert {"flat", "slope", "step", "rough", "pit", "plateau", "stairs"} <= kinds


def test_gecersiz_tur_ve_seviye():
    with pytest.raises(ValueError):
        sets.levels("yok")
    with pytest.raises(ValueError):
        sets.level("engebe", 99)


# ---------------------------------------------------------------------------
# Dünya dosyası
# ---------------------------------------------------------------------------


def test_dunya_dosyasi_gazebonun_bekledigi_gibi():
    text = world.world_sdf(terrain.step(0.03), name="deneme")
    root = ET.fromstring(text[text.index("<sdf"):])
    world_el = root.find("world")
    assert world_el.get("name") == "deneme"
    plugins = {p.get("filename") for p in world_el.findall("plugin")}
    assert {"gz-sim-physics-system", "gz-sim-imu-system", "gz-sim-contact-system"} <= plugins
    assert len(world_el.findall("model")) == 1
    assert world_el.find("physics/max_step_size").text == "0.001"


def test_dunya_dosyasi_yaziliyor_lf_ile(tmp_path):
    path = world.write_world(terrain.rough(0.03, seed=1), tmp_path / "z.sdf")
    raw = path.read_bytes()
    assert b"\r\n" not in raw, "satır sonu LF olmalı (robot Linux'ta)"
    assert b"ground" in raw
