"""STEP montaj ağacı çözümü: her parçanın montaj çerçevesindeki yeri.

NEXT_ASSEMBLY_USAGE_OCCURRENCE (hangi parça hangi montajın içinde) ve
ITEM_DEFINED_TRANSFORMATION (hangi eksenden hangi eksene) birleştirilerek
her occurrence için bir dönüşüm matrisi hesaplanır.

DİKKAT (PROJE_DEVIR §12): occurrence adları güvenilir, ama STEP içindeki
brep -> ürün adı eşlemesi değil (tibia_tip ile tibia_main ters çıkıyor).
Parça geometrisi için STEP yüzeyleri yerine adı doğru olan STL'leri kullan.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

from . import step
from .transform import IDENT, Mat, apply, mat_from_axis, mat_inv, mat_mul


@dataclass(frozen=True)
class Occurrence:
    """Montajdaki bir parça örneği."""

    name: str
    matrix: Mat  # parça çerçevesi -> montaj çerçevesi

    @property
    def origin(self) -> tuple[float, float, float]:
        return apply(self.matrix, (0, 0, 0))


class Assembly:
    """Bir STEP dosyasının üst düzey parçaları (kök montajın doğrudan çocukları)."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.parts: list[Occurrence] = _resolve(self.path)

    def __iter__(self):
        return iter(self.parts)

    def get(self, name: str) -> Occurrence:
        for occ in self.parts:
            if occ.name == name:
                return occ
        raise KeyError(f"{self.path.name} içinde '{name}' yok.")

    def matching(self, text: str) -> list[Occurrence]:
        """Adında text geçenler (büyük/küçük harf duyarsız)."""
        text = text.lower()
        return [occ for occ in self.parts if text in occ.name.lower()]


def _resolve(path: Path) -> list[Occurrence]:
    ents = step.load(path)

    prod, pdf, pd, pds, nauo, cdsr, idt, a2p = {}, {}, {}, {}, {}, {}, {}, {}
    rel: dict[int, int] = {}
    for eid, (etype, arg) in ents.items():
        if etype == "PRODUCT":
            prod[eid] = step.name_of(arg)
        elif etype == "PRODUCT_DEFINITION_FORMATION":
            pdf[eid] = step.refs(arg)
        elif etype == "PRODUCT_DEFINITION":
            pd[eid] = step.refs(arg)
        elif etype == "PRODUCT_DEFINITION_SHAPE":
            pds[eid] = step.refs(arg)
        elif etype == "NEXT_ASSEMBLY_USAGE_OCCURRENCE":
            nauo[eid] = (step.split_top(arg), step.refs(arg))
        elif etype == "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION":
            cdsr[eid] = step.refs(arg)
        elif etype == "ITEM_DEFINED_TRANSFORMATION":
            idt[eid] = step.refs(arg)
        elif etype == "AXIS2_PLACEMENT_3D":
            a2p[eid] = step.refs(arg)
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

    points = step.fetch(path, need)

    def vector(eid: int):
        if eid not in points:
            return None
        nums = re.findall(r"-?\d+\.?\d*(?:[Ee][-+]?\d+)?", step.split_top(points[eid][1])[-1])
        return tuple(float(x) for x in nums[:3]) if len(nums) >= 3 else None

    def axis_matrix(axid: int):
        refs = a2p[axid]
        return mat_from_axis(
            vector(refs[0]) or (0, 0, 0),
            vector(refs[1]) if len(refs) > 1 else None,
            vector(refs[2]) if len(refs) > 2 else None,
        )

    transforms = {
        nid: mat_mul(axis_matrix(a2), mat_inv(axis_matrix(a1)))
        for nid, (a1, a2) in xf_axes.items()
    }

    children: dict[int, list] = {}
    all_children = set()
    for nid, (parent, child, label) in edges.items():
        children.setdefault(parent, []).append((nid, child, label))
        all_children.add(child)

    result: list[Occurrence] = []
    for root in [p for p in children if p not in all_children]:
        for nid, child, label in children.get(root, []):
            result.append(Occurrence(label or pd_name(child), transforms.get(nid, IDENT)))
    return result
