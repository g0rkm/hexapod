"""hexapod_rl.olcum testleri (saf Python, Gazebo gerekmez) — GOREVLER.md S6.

Ölçümün kendisi Gazebo istiyor; burada onun ÜSTÜNDEKİ katman test ediliyor:
enerji hesabı, N tekrarın özetlenmesi, tablo ve CSV. Gazebo yerine sahte bir
evaluate enjekte ediliyor (olc(evaluate_fn=...)), böylece Windows'ta da koşar
ve sınır durumları (devrilme, yerinde sayma) istenen şekilde kurulabilir.
"""

from __future__ import annotations

import csv
import math

import pytest

from hexapod_rl import olcum


def sonuc(hiz=0.09, guc=3.5, sure=10.0, devrildi=False, sapma=2.0, yol=None) -> dict:
    """evaluate()'in döndürdüğü sözlüğün testte kullanılan alanları."""
    return {"ortalama_hiz_m_s": hiz, "ortalama_guc_w": guc, "sure_s": sure,
            "toplam_yol_m": hiz * sure if yol is None else yol,
            "devrildi": devrildi, "yon_sapmasi_derece": sapma}


class SahteZemin:
    def __init__(self, label="engebe 40 mm"):
        self.label = label
        self.sdf = f'<model name="ground"><!-- {label} --></model>'
        self.height = staticmethod(lambda x, y: 0.0)


# ---------------------------------------------------------------------------
# Enerji
# ---------------------------------------------------------------------------


def test_enerji_metre_basina_dogru():
    # 4 W x 10 s = 40 J, 2 m yol -> 20 J/m
    assert olcum.enerji_j_m(sonuc(guc=4.0, sure=10.0, yol=2.0)) == pytest.approx(20.0)


def test_yerinde_sayan_yuruyusun_enerjisi_sonsuz():
    """Yol ~0 iken J/m tanımsız; sıfıra bölme yerine inf (tabloda ∞)."""
    assert olcum.enerji_j_m(sonuc(guc=4.0, yol=0.0)) == math.inf


def test_hizli_ama_savurgan_ile_yavas_verimli_ayrisiyor():
    hizli = sonuc(hiz=0.12, guc=9.0, sure=10.0)     # 90 J / 1.2 m = 75 J/m
    yavas = sonuc(hiz=0.06, guc=3.0, sure=10.0)     # 30 J / 0.6 m = 50 J/m
    assert olcum.enerji_j_m(yavas) < olcum.enerji_j_m(hizli)


# ---------------------------------------------------------------------------
# Özet
# ---------------------------------------------------------------------------


def test_ozet_ortalama_ve_en_kotuyu_ayri_veriyor():
    kosular = [sonuc(hiz=0.10), sonuc(hiz=0.08), sonuc(hiz=0.03)]
    o = olcum.ozetle("düz", "tripod", (0.1, 0.0, 0.0), kosular)
    assert o.hiz_ort == pytest.approx(0.07, abs=1e-9)
    assert o.hiz_en_kotu == pytest.approx(0.03)
    assert o.tekrar == 3 and o.devrilme == 0 and o.gecti == "3/3"


def test_devrilme_sayiliyor_ve_en_az_yol_raporlaniyor():
    kosular = [sonuc(yol=0.9), sonuc(devrildi=True, sure=3.0, yol=0.2), sonuc(yol=0.85)]
    o = olcum.ozetle("basamak 60 mm", "tripod", (0.1, 0.0, 0.0), kosular)
    assert o.devrilme == 1 and o.gecti == "2/3"
    assert o.yol_en_az_m == pytest.approx(0.2), "düşmeden gidilen en az mesafe"


def test_sinir_durumu_ortalamada_kaybolmuyor():
    """Ders 32: 3 tekrarın 2'sinde geçip 1'inde devrilen bir zemin, tek
    ortalamaya bakılınca sağlam görünür; gecti ve en kötü bunu gösterir."""
    o = olcum.ozetle("basamak 60 mm", "politika", (0.1, 0.0, 0.0),
                     [sonuc(hiz=0.09), sonuc(hiz=0.09), sonuc(hiz=0.01, devrildi=True, yol=0.1)])
    assert o.hiz_ort > 0.06        # ortalama iyi görünüyor
    assert o.gecti == "2/3"        # ama tabloda bu da var
    assert o.hiz_en_kotu == pytest.approx(0.01)


def test_hepsi_sonsuz_enerjiyse_ozet_sonsuz():
    o = olcum.ozetle("düz", "x", (0.1, 0.0, 0.0), [sonuc(yol=0.0), sonuc(yol=0.0)])
    assert o.enerji_ort_j_m == math.inf


def test_bos_kosu_listesi_reddediliyor():
    with pytest.raises(ValueError):
        olcum.ozetle("düz", "x", (0.1, 0.0, 0.0), [])


def test_temiz_olcum_ayri_tutuluyor():
    o = olcum.ozetle("düz", "x", (0.1, 0.0, 0.0), [sonuc(hiz=0.08)], temiz=sonuc(hiz=0.09))
    assert o.hiz_ort == pytest.approx(0.08), "temiz ölçüm ortalamaya karışmamalı"
    assert o.temiz_hiz == pytest.approx(0.09)
    assert o.temiz_devrildi is False


# ---------------------------------------------------------------------------
# Tablo
# ---------------------------------------------------------------------------


def ornek_ozetler():
    return [
        olcum.ozetle("düz", "tripod", (0.1, 0, 0), [sonuc(hiz=0.09, guc=3.0)]),
        olcum.ozetle("düz", "politika", (0.1, 0, 0), [sonuc(hiz=0.10, guc=6.0)]),
        olcum.ozetle("basamak 60 mm", "tripod", (0.1, 0, 0),
                     [sonuc(hiz=0.01, devrildi=True, yol=0.1)]),
        olcum.ozetle("basamak 60 mm", "politika", (0.1, 0, 0), [sonuc(hiz=0.08)]),
    ]


def test_tablo_zemin_satir_denetleyici_sutun():
    t = olcum.tablo(ornek_ozetler())
    satirlar = t.splitlines()
    assert satirlar[0] == "| Zemin | tripod | politika |"
    assert satirlar[1] == "|---|---|---|"
    assert satirlar[2].startswith("| düz |")
    assert "basamak 60 mm" in satirlar[3]


def test_tabloda_devrilme_acikca_yaziyor():
    t = olcum.tablo(ornek_ozetler())
    satir = [s for s in t.splitlines() if "basamak" in s][0]
    assert "**devrildi**" in satir, "devrilen denetleyici tabloda göze çarpmalı"
    assert "0/1" in satir


def test_tabloda_hiz_enerji_ve_gecme_var():
    t = olcum.tablo(ornek_ozetler())
    duz = [s for s in t.splitlines() if s.startswith("| düz")][0]
    assert "m/s" in duz and "J/m" in duz and "1/1" in duz


def test_bos_tablo_cokmuyor():
    assert olcum.tablo([]) == "(ölçüm yok)"


def test_eksik_hucre_tire_oluyor():
    ozetler = [olcum.ozetle("düz", "a", (0.1, 0, 0), [sonuc()]),
               olcum.ozetle("engebe", "b", (0.1, 0, 0), [sonuc()])]
    t = olcum.tablo(ozetler)
    assert t.count("—") == 2, "a'nın engebesi ve b'nin düzü yok"


# ---------------------------------------------------------------------------
# CSV
# ---------------------------------------------------------------------------


def test_csv_her_kosuyu_ayri_satira_yaziyor(tmp_path):
    o = olcum.ozetle("düz", "tripod", (0.1, 0.0, 0.0),
                     [{**sonuc(hiz=0.09), "tohum": 1, "rastgele": 1},
                      {**sonuc(hiz=0.08), "tohum": 2, "rastgele": 1}])
    path = olcum.csv_yaz([o], tmp_path / "o.csv")
    with path.open(encoding="utf-8") as f:
        satirlar = list(csv.reader(f))
    assert satirlar[0][0] == "zemin" and "enerji_j_m" in satirlar[0]
    assert len(satirlar) == 3, "başlık + 2 koşu"
    assert satirlar[1][0] == "düz" and satirlar[1][5] == "1"


def test_csv_sonsuz_enerjiyi_bos_birakiyor(tmp_path):
    o = olcum.ozetle("düz", "x", (0.1, 0, 0), [{**sonuc(yol=0.0), "tohum": 1}])
    path = olcum.csv_yaz([o], tmp_path / "o.csv")
    satir = list(csv.reader(path.open(encoding="utf-8")))[1]
    assert satir[8] == "", "inf CSV'ye yazılmamalı"


# ---------------------------------------------------------------------------
# olc(): tekrarlar, rastgeleleştirme, temiz ölçüm
# ---------------------------------------------------------------------------


class SahteGorev:
    """task.TaskConfig yerine: replace() çalışsın diye dataclass benzeri."""

    def __init__(self, randomization=None):
        self.randomization = randomization


def sahte_evaluate(cagrilar):
    """Çağrıları kaydeden, tohuma göre biraz farklı sonuç veren sahte ölçüm."""
    def _fn(model, seconds, vx, vy, wz, seed, task, terrain_sdf, terrain_height):
        cagrilar.append({"seed": seed, "task": task, "sdf": terrain_sdf,
                         "komut": (vx, vy, wz), "seconds": seconds})
        return sonuc(hiz=0.09 + 0.001 * seed)
    return _fn


@pytest.fixture
def gorev():
    from dataclasses import make_dataclass
    Gorev = make_dataclass("Gorev", [("randomization", object)], frozen=True)
    return Gorev(randomization=None)


def test_olc_her_tohum_icin_bir_kosu(gorev):
    cagrilar = []
    o = olcum.olc(object(), gorev, SahteZemin(), tohumlar=(1, 2, 3), temiz=False,
                  denetleyici="tripod", evaluate_fn=sahte_evaluate(cagrilar))
    assert o.tekrar == 3
    assert [c["seed"] for c in cagrilar] == [1, 2, 3]
    assert o.zemin == "engebe 40 mm" and o.denetleyici == "tripod"


def test_olc_tekrarlarda_rastgelelestirmeyi_aciyor(gorev):
    """Kapalıyken tohumlar aynı sonucu verir (ölçüldü); tekrar anlamlı olsun diye açılmalı."""
    cagrilar = []
    olcum.olc(object(), gorev, SahteZemin(), tohumlar=(1, 2), temiz=True,
              evaluate_fn=sahte_evaluate(cagrilar))
    tekrarlar = [c for c in cagrilar if c["seed"] in (1, 2)]
    assert all(c["task"].randomization is not None for c in tekrarlar)
    temiz = [c for c in cagrilar if c["seed"] == 0]
    assert len(temiz) == 1 and temiz[0]["task"].randomization is None


def test_olc_zemini_ve_komutu_gecirliyor(gorev):
    cagrilar = []
    zemin = SahteZemin("basamak 45 mm")
    olcum.olc(object(), gorev, zemin, komut=(0.0, 0.0, 0.4), saniye=20.0,
              tohumlar=(7,), temiz=False, evaluate_fn=sahte_evaluate(cagrilar))
    c = cagrilar[0]
    assert c["sdf"] == zemin.sdf and c["komut"] == (0.0, 0.0, 0.4) and c["seconds"] == 20.0


def test_olc_duz_zeminde_bos_sdf_gonderiyor(gorev):
    cagrilar = []
    o = olcum.olc(object(), gorev, None, tohumlar=(1,), temiz=False,
                  evaluate_fn=sahte_evaluate(cagrilar))
    assert cagrilar[0]["sdf"] == "" and o.zemin == "düz"


def test_olc_temiz_kosu_ham_veriye_giriyor_ozete_girmiyor(gorev):
    o = olcum.olc(object(), gorev, SahteZemin(), tohumlar=(1, 2), temiz=True,
                  evaluate_fn=sahte_evaluate([]))
    assert o.tekrar == 2, "temiz koşu tekrar sayısına katılmamalı"
    assert len(o.kosular) == 3, "ama CSV'ye yazılsın diye saklanmalı"


# ---------------------------------------------------------------------------
# Refleksli denetleyici ("<tanım>+refleks[:AÇI]")
# ---------------------------------------------------------------------------


def test_refleks_tanimi_cozuluyor():
    assert olcum.denetleyici_coz("tripod:50") == ("tripod:50", None)
    assert olcum.denetleyici_coz("m/model.zip+refleks") == ("m/model.zip", olcum.REFLEKS_ACI)
    assert olcum.denetleyici_coz("m/model.zip+refleks:25") == ("m/model.zip", 25.0)
    with pytest.raises(ValueError):
        olcum.denetleyici_coz("m/model.zip+refleksler")


def test_olc_refleks_ayarini_tekrarlara_gurultulu_temize_ideal_geciriyor(gorev):
    """Tekrarlarda sensör gürültülü (gerçekçi), temiz ölçümde ideal: temiz sütun
    rastgeleleştirmesiz olduğu gibi sensörü de kusursuz."""
    cagrilar = []

    def _fn(model, seconds, vx, vy, wz, seed, task, terrain_sdf, terrain_height,
            sensor_kwargs=None):
        cagrilar.append({"seed": seed, "sensor": sensor_kwargs})
        return sonuc()

    ayar = {"range_sensors": ("s",), "lift_reflex": object(), "range_noise": 0.05,
            "range_drop": 0.1}
    olcum.olc(object(), gorev, SahteZemin(), tohumlar=(1, 2), temiz=True,
              evaluate_fn=_fn, sensor_kwargs=ayar)
    tekrarlar = [c["sensor"] for c in cagrilar if c["seed"] != 0]
    temiz = [c["sensor"] for c in cagrilar if c["seed"] == 0][0]
    assert all(s == ayar for s in tekrarlar)
    assert temiz["range_noise"] == 0.0 and temiz["range_drop"] == 0.0
    assert temiz["range_sensors"] == ayar["range_sensors"]
    assert ayar["range_noise"] == 0.05, "çağıranın sözlüğü değişmemeli"


# ---------------------------------------------------------------------------
# Kaygan zeminde geri kayma (yürüyüş değil)
# ---------------------------------------------------------------------------


def test_geri_kayma_hiz_negatifken_isaretleniyor():
    """Kaygan eğimde robot yürümeyip geri kayabilir; o zaman hız ve J/m
    yürüyüş başarısını değil kaymayı ölçer (ölçüldü: 15° mu0.3, -0.6 m/s)."""
    o = olcum.ozetle("kaygan 15°", "tripod", (0.1, 0.0, 0.0),
                     [sonuc(hiz=-0.60, yol=6.0), sonuc(hiz=-0.55, yol=5.5)])
    assert o.geri_kaydi is True
    hucre = olcum.tablo([o]).splitlines()[2]
    assert "geri kaydı" in hucre
    assert "J/m" not in hucre, "kayarken J/m yanıltıcı, gösterilmemeli"


def test_ileri_giden_geri_kayma_sayilmiyor():
    o = olcum.ozetle("düz", "tripod", (0.1, 0.0, 0.0), [sonuc(hiz=0.09)])
    assert o.geri_kaydi is False
    assert "geri kaydı" not in olcum.tablo([o])


def test_geri_komutta_negatif_hiz_kayma_degil():
    """Komut geriyse (vx<0) negatif hız beklenen davranış, kayma değil."""
    o = olcum.ozetle("düz", "politika", (-0.1, 0.0, 0.0), [sonuc(hiz=-0.09, yol=0.9)])
    assert o.geri_kaydi is False
