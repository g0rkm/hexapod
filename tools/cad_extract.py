#!/usr/bin/env python3
"""CAD'den geometri çıkar — robot.yaml'daki sayıların denetlenebilir kaynağı.

Neden var
---------
config/robot.yaml içindeki coxa=50, femur=80, yarıçap=100 gibi değerler
STEP assembly'sinden türetildi. Bu araç o türetmeyi yeniden çalıştırır, yani
sayılar "birinin bir yerde hesapladığı" değil, tekrar üretilebilir olur.
TÜBİTAK projesi kapsamında bu izlenebilirlik önemli.

Yöntem
------
STEP assembly ağacı (NEXT_ASSEMBLY_USAGE_OCCURRENCE + ITEM_DEFINED_TRANSFORMATION)
çözülerek her parçanın montaj çerçevesindeki konumu hesaplanır. Her eklemin
dönme ekseni, o eklemin servo horn'u ile karşısındaki bushing'i birleştiren
doğru olarak alınır — ikisi de eksen üzerinde oturur.

Kullanım
--------
    python tools/cad_extract.py            # özet
    python tools/cad_extract.py --verbose  # ham parça konumları da
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[union-attr]
    except (AttributeError, OSError, ValueError):
        pass

import stepasm as S  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
CAD_ROOT = REPO_ROOT / "Kerem Baltacı" / "Hexapod"
LEG_STEP = CAD_ROOT / "leg" / "leg-v2-v20.step"
FULL_STEP = CAD_ROOT / "Full Hexapod Model" / "hexapod-v8.step"


# ---------------------------------------------------------------------------
# Assembly ağacı çözümü
# ---------------------------------------------------------------------------


def occurrences(path: Path) -> list[tuple[str, tuple[float, float, float], tuple]]:
    """(parça adı, konum, dönüşüm matrisi) listesi döndür."""
    ents = S.load(path)

    prod, pdf, pd, pds, nauo, cdsr, idt, a2p = {}, {}, {}, {}, {}, {}, {}, {}
    rel: dict[int, int] = {}
    for eid, (etype, arg) in ents.items():
        if etype == "PRODUCT":
            prod[eid] = S.name_of(arg)
        elif etype == "PRODUCT_DEFINITION_FORMATION":
            pdf[eid] = S.refs(arg)
        elif etype == "PRODUCT_DEFINITION":
            pd[eid] = S.refs(arg)
        elif etype == "PRODUCT_DEFINITION_SHAPE":
            pds[eid] = S.refs(arg)
        elif etype == "NEXT_ASSEMBLY_USAGE_OCCURRENCE":
            nauo[eid] = (S.split_top(arg), S.refs(arg))
        elif etype == "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION":
            cdsr[eid] = S.refs(arg)
        elif etype == "ITEM_DEFINED_TRANSFORMATION":
            idt[eid] = S.refs(arg)
        elif etype == "AXIS2_PLACEMENT_3D":
            a2p[eid] = S.refs(arg)
        elif etype == "COMPLEX" and "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION" in arg:
            match = re.search(r"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(\s*#(\d+)", arg)
            if match:
                rel[eid] = int(match.group(1))

    def pd_name(pdid: int) -> str:
        refs = pd.get(pdid)
        formation = pdf.get(refs[0]) if refs else None
        return prod.get(formation[0], "?") if formation else "?"

    edges: dict[int, tuple[int, int, str]] = {}
    for eid, (parts, refs) in nauo.items():
        pdrefs = [x for x in refs if x in pd]
        if len(pdrefs) >= 2:
            label = parts[1].strip().strip("'") if len(parts) > 1 else ""
            edges[eid] = (pdrefs[0], pdrefs[1], label)

    need: set[int] = set()
    xf_axes: dict[int, tuple[int, int]] = {}
    for refs in cdsr.values():
        relid = next((x for x in refs if x in rel), None)
        pdsid = next((x for x in refs if x in pds), None)
        if relid is None or pdsid is None:
            continue
        nid = next((x for x in pds[pdsid] if x in nauo), None)
        axes = [x for x in idt.get(rel[relid], []) if x in a2p]
        if nid is None or len(axes) < 2:
            continue
        xf_axes[nid] = (axes[0], axes[1])
        need.update(a2p[axes[0]])
        need.update(a2p[axes[1]])

    points = S.fetch(path, need)

    def vector(eid: int):
        if eid not in points:
            return None
        nums = re.findall(r"-?\d+\.?\d*(?:[Ee][-+]?\d+)?", S.split_top(points[eid][1])[-1])
        return tuple(float(x) for x in nums[:3]) if len(nums) >= 3 else None

    def axis_matrix(axid: int):
        refs = a2p[axid]
        return S.mat_from_axis(
            vector(refs[0]) or (0, 0, 0),
            vector(refs[1]) if len(refs) > 1 else None,
            vector(refs[2]) if len(refs) > 2 else None,
        )

    transforms = {
        nid: S.mat_mul(axis_matrix(a2), S.mat_inv(axis_matrix(a1)))
        for nid, (a1, a2) in xf_axes.items()
    }

    children: dict[int, list] = {}
    all_children = set()
    for nid, (parent, child, label) in edges.items():
        children.setdefault(parent, []).append((nid, child, label))
        all_children.add(child)

    result: list[tuple[str, tuple[float, float, float], tuple]] = []
    for root in [p for p in children if p not in all_children]:
        for nid, child, label in children.get(root, []):
            matrix = transforms.get(nid, S.IDENT)
            result.append((label or pd_name(child), S.apply(matrix, (0, 0, 0)), matrix))
    return result


# ---------------------------------------------------------------------------
# Türetmeler
# ---------------------------------------------------------------------------


def analyse_leg(verbose: bool = False) -> None:
    print("=" * 72)
    print("BACAK GEOMETRİSİ —", LEG_STEP.name)
    print("=" * 72)
    items = occurrences(LEG_STEP)

    horns = sorted((p for name, p, _ in items if "servo horn" in name.lower()),
                   key=lambda p: -p[2])
    bushings = sorted((p for name, p, _ in items if "bushing" in name.lower()),
                      key=lambda p: -p[2])

    if verbose:
        for name, pos, _ in sorted(items, key=lambda it: it[0]):
            print(f"  {name:<40} ({pos[0]:8.2f},{pos[1]:8.2f},{pos[2]:8.2f})")
        print()

    if len(horns) < 3 or len(bushings) < 3:
        print(f"UYARI: 3 horn + 3 bushing bekleniyordu, "
              f"{len(horns)} horn / {len(bushings)} bushing bulundu.")
        return

    print("Eklem eksenleri (horn merkezi <-> bushing merkezi):")
    axes = []
    for index, (horn, bushing) in enumerate(zip(horns, bushings), start=1):
        direction = tuple(horn[i] - bushing[i] for i in range(3))
        length = math.sqrt(sum(d * d for d in direction))
        unit = tuple(d / length for d in direction)
        midpoint = tuple((horn[i] + bushing[i]) / 2 for i in range(3))
        label = {0: "X", 1: "Y", 2: "Z"}[max(range(3), key=lambda i: abs(unit[i]))]
        axes.append((midpoint, unit))
        print(f"  J{index}: orta nokta ({midpoint[0]:7.2f},{midpoint[1]:7.2f},"
              f"{midpoint[2]:7.2f})  eksen ~{label}  açıklık {length:.1f} mm")

    j1, j2, j3 = (a[0] for a in axes)
    coxa = abs(j2[2] - j1[2])
    femur = math.hypot(j3[1] - j2[1], j3[2] - j2[2])

    print()
    print(f"  coxa  (J1 -> J2, yatay)           = {coxa:.3f} mm")
    print(f"  femur (J2 -> J3)                  = {femur:.3f} mm")
    print(f"  femur ekleminin dikey ofseti      = {j2[1]:.2f} mm")

    tip = max(
        (S.apply(matrix, (0, 0, 0)) for name, _, matrix in items if "Tibia_tip" in name),
        default=None,
    )
    if tip is not None:
        print(f"  tibia: ayak kapağı yerleşimi bulundu; en uç nokta ölçümü için")
        print(f"         --verbose ile parça geometrisine bakın. CAD ~121 mm,")
        print(f"         ayak ucu yuvarlak olduğu için KUMPASLA doğrulanmalı.")
    print()


def analyse_body(verbose: bool = False) -> None:
    print("=" * 72)
    print("GÖVDE YERLEŞİMİ —", FULL_STEP.name)
    print("=" * 72)
    items = [it for it in occurrences(FULL_STEP) if it[0].lower().startswith("leg")]

    if not items:
        print("UYARI: montajda bacak bulunamadı.")
        return

    # Coxa yaw ekseni, bacak yerel çerçevesinde (0,0,+20)'de.
    #
    # Aynalı bacaklar Fusion'da ayrı gövde olarak dışa aktarılmış: dönüşüm
    # matrisi düzgün bir dönme (determinant +1), yansıma GEOMETRİDE. Yani
    # aynalı bacağın kendi çerçevesinde bacak +Z yönünde uzanır, +Z değil.
    # Bu yüzden eksen ofseti aynalı bacaklarda ters işaretli alınır.
    #
    # Doğrulama, aşağıdaki tutarlılık kontrolü: 6 bacak da aynı yarıçapta
    # çıkmıyorsa bu varsayım yanlıştır ve araç bunu söyler.
    print(f"{'bacak':<26}{'konum (x,y,z)':<30}{'yarıçap':>9}{'azimut':>9}  aynalı")
    print("-" * 82)
    rows = []
    for name, pos, matrix in sorted(items, key=lambda it: it[0]):
        mirrored = "mirror" in name.lower()
        offset = (0, 0, -20) if mirrored else (0, 0, 20)
        axis = S.apply(matrix, offset)
        radius = math.hypot(axis[0], axis[2])
        azimuth = math.degrees(math.atan2(axis[2], axis[0]))
        rows.append((radius, azimuth, mirrored))
        print(f"{name:<26}({pos[0]:7.1f},{pos[1]:6.1f},{pos[2]:7.1f})   "
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
    print("Brifin uyarısı geçerli: baskı toleransı ve horn kalınlığı yüzünden")
    print("monte robottan kumpasla doğrulanmadan nihai kabul edilmemeli.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
