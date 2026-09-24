"""cadlib — CAD araçlarının ortak kütüphanesi.

tools/cad_extract.py ve tools/cad_sim_model.py yalnızca komut satırı
araçlarıdır; asıl iş buradadır:

    step.py       STEP metnini ayrıştırma (montaj ağacı varlıkları)
    assembly.py   montaj ağacı çözümü -> Assembly / Occurrence
    transform.py  3x4 katı dönüşüm cebiri
    mesh.py       STL okuma, Box, MassProps (hacim/ağırlık merkezi/atalet)
    frames.py     CAD çerçeveleri <-> IK/gövde çerçeveleri

CAD dosyaları cad/ altında, depoda. Kaynak: Printables 606030 (Sir
Kuhnhero, CC BY-SA 4.0).
"""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def find_cad_root() -> Path:
    """CAD klasörü. Yeni yer cad/Hexapod; eski düzenler de denenir."""
    for candidate in (REPO_ROOT / "cad" / "Hexapod", REPO_ROOT / "Hexapod",
                      REPO_ROOT / "Kerem Baltacı" / "Hexapod"):
        if (candidate / "leg" / "leg-v2-v20.step").is_file():
            return candidate
    return REPO_ROOT / "cad" / "Hexapod"  # yoksa araç bu yolu "bulunamadı" diye raporlar


CAD_ROOT = find_cad_root()
LEG_STEP = CAD_ROOT / "leg" / "leg-v2-v20.step"
BODY_STEP = CAD_ROOT / "body" / "body-v35.step"
FULL_STEP = CAD_ROOT / "Full Hexapod Model" / "hexapod-v8.step"
