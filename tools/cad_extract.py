#!/usr/bin/env python3
"""CAD'den geometri çıkar — robot.yaml'daki sayıların denetlenebilir kaynağı.

Neden var
---------
config/robot.yaml içindeki coxa=50, femur=80, tibia=126.6, yarıçap=100
değerleri STEP assembly'sinden ve basılan STL'lerden türetildi. Bu araç o
türetmeyi yeniden çalıştırır, yani sayılar "birinin bir yerde hesapladığı"
değil, tekrar üretilebilir olur. TÜBİTAK projesi kapsamında bu izlenebilirlik
önemli.

Yöntem
------
STEP assembly ağacı çözülerek her parçanın montaj çerçevesindeki konumu
hesaplanır (tools/cadlib/assembly.py). Her eklemin dönme ekseni, o eklemin
servo horn'u ile karşısındaki bushing'i birleştiren doğru olarak alınır —
ikisi de eksen üzerinde oturur.

Kullanım
--------
    python tools/cad_extract.py            # özet
    python tools/cad_extract.py --verbose  # ham parça konumları da
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[union-attr]
    except (AttributeError, OSError, ValueError):
        pass

from cadlib import FULL_STEP, LEG_STEP  # noqa: E402
from cadlib.assembly import Assembly  # noqa: E402
from cadlib.frames import joint_axes  # noqa: E402
from cadlib.mesh import load_stl  # noqa: E402
from cadlib.transform import apply  # noqa: E402


def analyse_leg(verbose: bool = False) -> None:
    print("=" * 72)
    print("BACAK GEOMETRİSİ —", LEG_STEP.name)
    print("=" * 72)
    leg = Assembly(LEG_STEP)

    if verbose:
        for occ in sorted(leg, key=lambda o: o.name):
            p = occ.origin
            print(f"  {occ.name:<40} ({p[0]:8.2f},{p[1]:8.2f},{p[2]:8.2f})")
        print()

    try:
        axes = joint_axes(leg)
    except ValueError as exc:
        print(f"UYARI: {exc}")
        return

    horns = sorted((o.origin for o in leg.matching("servo horn")), key=lambda p: -p[2])
    bushings = sorted((o.origin for o in leg.matching("bushing")), key=lambda p: -p[2])
    print("Eklem eksenleri (horn merkezi <-> bushing merkezi):")
    for index, (mid, horn, bushing) in enumerate(zip(axes, horns, bushings), start=1):
        direction = tuple(horn[i] - bushing[i] for i in range(3))
        length = math.sqrt(sum(d * d for d in direction))
        label = "XYZ"[max(range(3), key=lambda i: abs(direction[i]))]
        print(f"  J{index}: orta nokta ({mid[0]:7.2f},{mid[1]:7.2f},"
              f"{mid[2]:7.2f})  eksen ~{label}  açıklık {length:.1f} mm")

    j1, j2, j3 = axes
    coxa = abs(j2[2] - j1[2])
    femur = math.hypot(j3[1] - j2[1], j3[2] - j2[2])

    print()
    print(f"  coxa  (J1 -> J2, yatay)           = {coxa:.3f} mm")
    print(f"  femur (J2 -> J3)                  = {femur:.3f} mm")
    print(f"  femur ekleminin dikey ofseti      = {j2[1]:.2f} mm")

    analyse_tibia(leg, (j3[1], j3[2]))
    print()


def analyse_tibia(leg: Assembly, j3_yz: tuple[float, float]) -> None:
    """Tibia uzunluğu: J3 ekseninden ayak kapağının en uç noktasına.

    Ayak ucu STEP'te serbest form (B-spline) bir yüzey; kontrol noktaları
    yüzeyin dışında durduğu için STEP noktalarından en uç nokta bulunamaz.
    Bunun yerine basılan STL'ler kullanılıyor: mesh köşeleri yüzeyin tam
    üstündedir. STL'ler parçanın kendi çerçevesinde; STEP'teki montaj
    dönüşümü uygulanınca bacak çerçevesine geçiyorlar.

    DİKKAT: STEP içinde parça adı -> geometri eşlemesi tibia_tip ile
    tibia_main için ters çıkıyor (ilk türetmede 121 mm hatası buradan geldi).
    Bu fonksiyon STEP'teki geometriye değil, adı doğru olan STL'lere bakar.
    """
    tip_occ = next((o for o in leg if "Tibia_tip" in o.name), None)
    tip_stl = LEG_STEP.parent / "pla_tibia_tip.stl"
    main_stl = LEG_STEP.parent / "pla_tibia_main.stl"
    if tip_occ is None or not tip_stl.is_file() or not main_stl.is_file():
        print("  tibia: hesaplanamadı (montajda ayak kapağı ya da STL dosyaları yok)")
        return

    def farthest(stl: Path):
        pts = set(load_stl(stl, tip_occ.matrix))
        return max(((math.hypot(q[1] - j3_yz[0], q[2] - j3_yz[1]), q) for q in pts),
                   key=lambda t: t[0])

    tip_d, tip_p = farthest(tip_stl)
    main_d, _ = farthest(main_stl)
    print(f"  tibia (J3 -> ayak ucu)            = {tip_d:.3f} mm")
    print(f"        ayak kapağı gövdenin {tip_d - main_d:.1f} mm dışına taşıyor "
          f"(gövde ucu {main_d:.2f} mm)")

    # Tutarlılık: uç bacağın orta düzleminde olmalı ve kapak gövdeden
    # daha dışarıda olmalı. Değilse STL/çerçeve eşleşmesi bozulmuştur.
    if abs(tip_p[0]) > 1.0 or tip_d <= main_d:
        print("  -> UYARI: ayak ucu beklenen yerde değil; STL'ler montajla")
        print("     eşleşmiyor olabilir, bu sayıya güvenmeyin.")
    else:
        print("  -> tutarlılık kontrolü GEÇTİ: uç bacak orta düzleminde, "
              "kapak gövdenin dışında.")


def analyse_body(verbose: bool = False) -> None:
    print("=" * 72)
    print("GÖVDE YERLEŞİMİ —", FULL_STEP.name)
    print("=" * 72)
    legs = [o for o in Assembly(FULL_STEP) if o.name.lower().startswith("leg")]

    if not legs:
        print("UYARI: montajda bacak bulunamadı.")
        return

    # Coxa yaw ekseni, bacak yerel çerçevesinde (0,0,+20)'de.
    #
    # Aynalı bacaklar Fusion'da ayrı gövde olarak dışa aktarılmış: dönüşüm
    # matrisi düzgün bir dönme (determinant +1), yansıma GEOMETRİDE. Yani
    # aynalı bacağın kendi çerçevesinde bacak +Z yönünde uzanır, -Z değil.
    # Bu yüzden eksen ofseti aynalı bacaklarda ters işaretli alınır.
    #
    # Doğrulama, aşağıdaki tutarlılık kontrolü: 6 bacak da aynı yarıçapta
    # çıkmıyorsa bu varsayım yanlıştır ve araç bunu söyler.
    print(f"{'bacak':<26}{'konum (x,y,z)':<30}{'yarıçap':>9}{'azimut':>9}  aynalı")
    print("-" * 82)
    rows = []
    for occ in sorted(legs, key=lambda o: o.name):
        mirrored = "mirror" in occ.name.lower()
        axis = apply(occ.matrix, (0, 0, -20) if mirrored else (0, 0, 20))
        radius = math.hypot(axis[0], axis[2])
        azimuth = math.degrees(math.atan2(axis[2], axis[0]))
        rows.append((radius, azimuth, mirrored))
        pos = occ.origin
        print(f"{occ.name:<26}({pos[0]:7.1f},{pos[1]:6.1f},{pos[2]:7.1f})   "
              f"{radius:>8.1f}{azimuth:>9.1f}   {'evet' if mirrored else 'hayır'}")

    radii = [r for r, _, _ in rows]
    spread = max(radii) - min(radii)
    print()
    print(f"  coxa ekseni yarıçapı: {min(radii):.1f} .. {max(radii):.1f} mm")
    print(f"  azimutlar: {sorted(round(a) for _, a, _ in rows)}")

    if spread < 0.5:
        print(f"  -> {len(rows)} bacak, radyal altıgen yerleşim, yarıçap "
              f"{sum(radii) / len(radii):.1f} mm")
        print("  -> tutarlılık kontrolü GEÇTİ: altı bacak da aynı yarıçapta.")
    else:
        print(f"  -> UYARI: yarıçaplar {spread:.1f} mm ayrışıyor.")
        print("     Simetrik bir robotta bu olmaz. Aynalama varsayımı yanlış")
        print("     olabilir; bu sayıya güvenmeyin, CAD'i elle kontrol edin.")
    print()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="CAD'den hexapod geometrisi çıkar")
    parser.add_argument("--verbose", action="store_true", help="Ham parça konumlarını da yaz")
    parser.add_argument("--leg-only", action="store_true")
    parser.add_argument("--body-only", action="store_true")
    args = parser.parse_args(argv)

    missing = [p for p in (LEG_STEP, FULL_STEP) if not p.is_file()]
    if missing and not (args.leg_only or args.body_only):
        for path in missing:
            print(f"HATA: CAD dosyası bulunamadı: {path}", file=sys.stderr)
        return 2

    if not args.body_only:
        analyse_leg(args.verbose)
    if not args.leg_only:
        print("(tam montaj 85 MB, çözümleme biraz sürer...)\n")
        analyse_body(args.verbose)

    print("Bu sayılar config/robot.yaml içinde measured:false olarak duruyor.")
    print("Kumpasla ölçülmüyor: doğruluğu belirleyen servo horn'unun mile oturma")
    print("hatası ve onu kalibrasyon gideriyor (bkz. CLAUDE.md).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
