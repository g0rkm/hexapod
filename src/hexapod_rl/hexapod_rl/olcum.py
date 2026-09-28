"""Yürüyüş ölçüm aracı: denetleyicileri zeminlerde karşılaştır (GOREVLER.md S6).

    python -m hexapod_rl.olcum tripod:50
    python -m hexapod_rl.olcum tripod:50 models/ppo_refleks_1500k/model.zip --tohum 5
    python -m hexapod_rl.olcum tripod --zemin basamak --saniye 20 --csv olcum.csv

Denetleyici tanımı terrain_probe.load ile aynı: "tripod", "tripod:50",
"phase[:mm]", "<zip>", "<zip>:residual".

Ölçülenler (S6): ileri hız, enerji, devrilme sayısı, düşmeden gidilen mesafe.
Enerji METRE BAŞINA veriliyor (J/m): yavaş ama verimli bir yürüyüşle hızlı
ama savurgan olanı ancak bu ayırır. Toplam enerji = ortalama güç x süre
(güç, ödüldeki Σ|tork x açısal hız|); yol, gidilen düz mesafe. Robot daire
çizerse yol gerçek patikadan kısa görünür, J/m olduğundan yüksek çıkar;
komutlar düz olduğu için pratikte sorun değil.

Neden N tekrar ve neden rastgeleleştirme açık
----------------------------------------------
Süreç içi Gazebo deterministik: rastgeleleştirme KAPALIYKEN farklı tohumlar
BİREBİR aynı sonucu verir (ölçüldü 2026-09-28: üç tohum da 0.0869 m/s), yani
tekrar bilgi taşımaz. Açıkken (servo gücü, gecikme, itme, IMU gürültüsü her
bölümde tohumdan çekilir) tohumlar farklılaşır ve sınırdaki bir engelin
"bazen geçiliyor" olduğu görünür — ders 32: tek deterministik ölçüm 60 mm
gibi sınır durumlarda yanıltıcı. Bu yüzden varsayılan: rastgeleleştirme
açık, N tohum, tabloda ortalama YANINDA en kötü durum ve kaç tekrarda
devrildiği. Ayrıca gürültüsüz tek ölçüm ("temiz") ayrı sütunda verilir.
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[union-attr]
    except (AttributeError, OSError, ValueError):
        pass

#: Varsayılan komut: ileri 0.1 m/s (eğitim ölçümlerinin ortası, EVAL_FORWARD).
DEFAULT_COMMAND = (0.1, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Saf hesap: evaluate() sonuçlarından özet (Gazebo gerekmez, testlenebilir)
# ---------------------------------------------------------------------------


def enerji_j_m(result: dict) -> float:
    """Metre başına mekanik enerji (J/m). Yol ~0 ise inf (yerinde sayan yürüyüş)."""
    enerji = result["ortalama_guc_w"] * result["sure_s"]
    yol = result["toplam_yol_m"]
    return enerji / yol if yol > 1e-3 else math.inf


@dataclass(frozen=True)
class Ozet:
    """Bir (zemin, denetleyici, komut) üçlüsünün N tekrarlı ölçümü."""

    zemin: str
    denetleyici: str
    komut: tuple[float, float, float]
    tekrar: int
    devrilme: int                 # kaç tekrarda devrildi
    hiz_ort: float                # m/s, komut yönünde
    hiz_en_kotu: float
    enerji_ort_j_m: float
    yol_ort_m: float
    yol_en_az_m: float            # düşmeden gidilen en az mesafe (en kötü tekrar)
    yon_sapmasi_ort_derece: float
    temiz_hiz: float | None = None        # rastgeleleştirmesiz tek ölçüm
    temiz_enerji_j_m: float | None = None
    temiz_devrildi: bool | None = None
    kosular: tuple[dict, ...] = field(default=(), repr=False)

    @property
    def gecti(self) -> str:
        """Kaç tekrarda devrilmeden tamamladı, "3/3" gibi."""
        return f"{self.tekrar - self.devrilme}/{self.tekrar}"

    @property
    def geri_kaydi(self) -> bool:
        """İleri komut verilmişken ortalama hız negatif: robot geri kayıyor.

        Kaygan eğimde olur. Bu durumda hız ve J/m yürüyüş başarısını değil
        kaymayı ölçer, tabloda ayrıca işaretlenir.
        """
        return self.komut[0] > 0 and self.hiz_ort < 0


def ozetle(zemin: str, denetleyici: str, komut, kosular: list[dict],
           temiz: dict | None = None) -> Ozet:
    """evaluate() sonuçlarını özete indir. kosular boşsa hata."""
    if not kosular:
        raise ValueError("en az bir koşu gerekli")
    hizlar = [k["ortalama_hiz_m_s"] for k in kosular]
    yollar = [k["toplam_yol_m"] for k in kosular]
    enerjiler = [e for e in (enerji_j_m(k) for k in kosular) if math.isfinite(e)]
    return Ozet(
        zemin=zemin, denetleyici=denetleyici, komut=tuple(komut), tekrar=len(kosular),
        devrilme=sum(1 for k in kosular if k["devrildi"]),
        hiz_ort=statistics.fmean(hizlar), hiz_en_kotu=min(hizlar),
        enerji_ort_j_m=statistics.fmean(enerjiler) if enerjiler else math.inf,
        yol_ort_m=statistics.fmean(yollar), yol_en_az_m=min(yollar),
        yon_sapmasi_ort_derece=statistics.fmean([k["yon_sapmasi_derece"] for k in kosular]),
        temiz_hiz=None if temiz is None else temiz["ortalama_hiz_m_s"],
        temiz_enerji_j_m=None if temiz is None else enerji_j_m(temiz),
        temiz_devrildi=None if temiz is None else bool(temiz["devrildi"]),
        kosular=tuple(kosular),
    )


def _sayi(v: float, birim: str = "", basamak: int = 3) -> str:
    if v is None:
        return "—"
    if not math.isfinite(v):
        return "∞"
    return f"{v:.{basamak}f}{birim}"


def tablo(ozetler: list[Ozet]) -> str:
    """Zemin satır, denetleyici sütun; her hücrede hız / enerji / geçme."""
    if not ozetler:
        return "(ölçüm yok)"
    zeminler = list(dict.fromkeys(o.zemin for o in ozetler))
    denetleyiciler = list(dict.fromkeys(o.denetleyici for o in ozetler))
    kayit = {(o.zemin, o.denetleyici): o for o in ozetler}

    satirlar = ["| Zemin | " + " | ".join(denetleyiciler) + " |",
                "|---|" + "---|" * len(denetleyiciler)]
    for zemin in zeminler:
        hucreler = []
        for d in denetleyiciler:
            o = kayit.get((zemin, d))
            if o is None:
                hucreler.append("—")
                continue
            if o.geri_kaydi:
                # Komut ileriyken ortalama hız negatif: robot yürümüyor, zemin
                # onu geri kaydırıyor (kaygan eğim). Hız ve J/m burada
                # "ne kadar iyi yürüdü" değil "ne kadar kaydı" demek olur.
                hucreler.append(f"**geri kaydı** {_sayi(o.hiz_ort, ' m/s')}"
                                + (f", {o.gecti}" if o.devrilme else ""))
                continue
            if o.devrilme == o.tekrar:
                hucreler.append(f"**devrildi** ({o.gecti}), {_sayi(o.yol_ort_m, ' m', 2)}")
                continue
            metin = (f"{_sayi(o.hiz_ort, ' m/s')} · {_sayi(o.enerji_ort_j_m, ' J/m', 1)}"
                     f" · {o.gecti}")
            if o.devrilme:
                metin += f" · en az {_sayi(o.yol_en_az_m, ' m', 2)}"
            hucreler.append(metin)
        satirlar.append(f"| {zemin} | " + " | ".join(hucreler) + " |")
    return "\n".join(satirlar)


def csv_satirlari(ozetler: list[Ozet]) -> list[list]:
    """Ham veri: her koşu bir satır (sonradan grafik çizilebilsin)."""
    basliklar = ["zemin", "denetleyici", "komut_vx", "komut_vy", "komut_wz", "tohum",
                 "hiz_m_s", "yol_m", "enerji_j_m", "guc_w", "sure_s", "devrildi",
                 "yon_sapmasi_derece", "rastgele"]
    satirlar = [basliklar]
    for o in ozetler:
        for k in o.kosular:
            satirlar.append([
                o.zemin, o.denetleyici, *o.komut, k.get("tohum", ""),
                round(k["ortalama_hiz_m_s"], 5), round(k["toplam_yol_m"], 4),
                round(enerji_j_m(k), 2) if math.isfinite(enerji_j_m(k)) else "",
                round(k["ortalama_guc_w"], 3), round(k["sure_s"], 2),
                int(bool(k["devrildi"])), round(k["yon_sapmasi_derece"], 1),
                int(k.get("rastgele", 1)),
            ])
    return satirlar


def csv_yaz(ozetler: list[Ozet], path: str | Path) -> Path:
    import csv

    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", encoding="utf-8", newline="") as f:
        csv.writer(f).writerows(csv_satirlari(ozetler))
    return p


# ---------------------------------------------------------------------------
# Ölçüm (Gazebo)
# ---------------------------------------------------------------------------


def olc(model, task, zemin, komut=DEFAULT_COMMAND, saniye: float = 10.0,
        tohumlar=(1, 2, 3), temiz: bool = True, denetleyici: str = "",
        evaluate_fn=None) -> Ozet:
    """Bir denetleyiciyi bir zeminde N tohumla ölç.

    zemin: hexapod_terrain.Terrain (ya da None: düz). Tekrarlar rastgeleleştirme
    AÇIK koşar (yoksa tohumlar aynı sonucu verir, bkz. modül açıklaması);
    temiz=True ayrıca rastgeleleştirmesiz tek ölçüm ekler.
    evaluate_fn yalnız test için (gerçek ölçüm hexapod_rl.evaluate.evaluate).
    """
    from dataclasses import replace

    from .task import Randomization

    if evaluate_fn is None:
        from .evaluate import evaluate
        evaluate_fn = evaluate

    sdf = zemin.sdf if zemin is not None else ""
    height = zemin.height if zemin is not None else None
    ad = zemin.label if zemin is not None else "düz"
    vx, vy, wz = komut

    rastgele_gorev = replace(task, randomization=Randomization())
    kosular = []
    for tohum in tohumlar:
        r = evaluate_fn(model, seconds=saniye, vx=vx, vy=vy, wz=wz, seed=tohum,
                        task=rastgele_gorev, terrain_sdf=sdf, terrain_height=height)
        kosular.append({**r, "tohum": tohum, "rastgele": 1})

    temiz_sonuc = None
    if temiz:
        temiz_sonuc = {**evaluate_fn(model, seconds=saniye, vx=vx, vy=vy, wz=wz, seed=0,
                                     task=task, terrain_sdf=sdf, terrain_height=height),
                       "tohum": 0, "rastgele": 0}

    ozet = ozetle(ad, denetleyici or "denetleyici", komut, kosular, temiz_sonuc)
    if temiz_sonuc is not None:
        # Temiz koşu özete girmez (ortalamayı bozmasın) ama ham veriye girer.
        ozet = replace(ozet, kosular=ozet.kosular + (temiz_sonuc,))
    return ozet


def main(argv: list[str] | None = None) -> int:
    from hexapod_terrain import sets

    from .terrain_probe import load

    parser = argparse.ArgumentParser(description="Yürüyüş ölçüm aracı (S6)")
    parser.add_argument("denetleyiciler", nargs="+",
                        help='"tripod", "tripod:50", "phase[:mm]", "<zip>[:residual]"')
    parser.add_argument("--zemin", default="", help="yalnız bu türün seviyeleri (ör. basamak)")
    parser.add_argument("--tohum", type=int, default=3, help="tekrar sayısı (varsayılan 3)")
    parser.add_argument("--saniye", type=float, default=10.0)
    parser.add_argument("--vx", type=float, default=DEFAULT_COMMAND[0])
    parser.add_argument("--vy", type=float, default=DEFAULT_COMMAND[1])
    parser.add_argument("--wz", type=float, default=DEFAULT_COMMAND[2])
    parser.add_argument("--zemin-tohumu", type=int, default=1, help="engebenin tohumu")
    parser.add_argument("--temiz", action="store_true", help="rastgeleleştirmesiz ölçümü de al")
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument("-o", "--out", type=Path, default=None, help="tabloyu dosyaya yaz")
    args = parser.parse_args(argv)

    zeminler = (sets.levels(args.zemin, args.zemin_tohumu) if args.zemin
                else sets.evaluation_set(args.zemin_tohumu))
    komut = (args.vx, args.vy, args.wz)
    tohumlar = tuple(range(1, args.tohum + 1))

    ozetler = []
    for spec in args.denetleyiciler:
        model, task = load(spec)
        for zemin in zeminler:
            ozet = olc(model, task, zemin, komut, args.saniye, tohumlar,
                       temiz=args.temiz, denetleyici=spec)
            ozetler.append(ozet)
            print(f"  {spec:<28} {ozet.zemin:<22} {_sayi(ozet.hiz_ort, ' m/s')} "
                  f"{_sayi(ozet.enerji_ort_j_m, ' J/m', 1):>10}  {ozet.gecti}",
                  file=sys.stderr, flush=True)

    metin = tablo(ozetler)
    print(metin)
    if args.out:
        # Başlık: tablo tek başına okunduğunda hangi koşullarda ölçüldüğü belli
        # olsun. Özellikle ZEMİN KAYNAĞI: depoda iki ayrı zemin tanımı var
        # (hexapod_terrain ve hexapod_rl.terrain_probe), aynı adlı zeminler
        # farklı; iki tablodaki sayılar karşılaştırılamaz (docs/olcumler/).
        baslik = (f"<!-- ÜRETİLDİ: python -m hexapod_rl.olcum "
                  f"{' '.join(args.denetleyiciler)} -->\n"
                  f"zemin kaynağı: hexapod_terrain (S5) · "
                  f"komut vx={komut[0]:g} vy={komut[1]:g} wz={komut[2]:g} · "
                  f"{args.saniye:g} s · {len(tohumlar)} tohum, rastgeleleştirme açık"
                  f"{' · temiz ölçüm ayrıca alındı' if args.temiz else ''}\n\n")
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(baslik + metin + "\n", encoding="utf-8", newline="\n")
    if args.csv:
        csv_yaz(ozetler, args.csv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
