"""Zeminleri listele, dünya dosyası yaz, önizleme çiz (GOREVLER.md S5).

    python -m hexapod_terrain liste
    python -m hexapod_terrain dunya engebe 2 -o /tmp/engebe.sdf
    python -m hexapod_terrain onizleme --out docs/zeminler

Dünya dosyası ROS'lu simde açılır:
    ros2 launch hexapod_gazebo sim.launch.py world:=/tmp/engebe.sdf
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Robot Linux'ta ama araç Windows'tan da çağrılıyor; oradaki konsol kod sayfası
# Türkçe karakterleri ve μ'yü bozmasın (tools/calibrate.py ile aynı).
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[union-attr]
    except (AttributeError, OSError, ValueError):
        pass

from . import sets
from .world import write_world


def _list() -> None:
    for kind in sets.kinds():
        items = sets.levels(kind)
        print(f"{kind} ({len(items)} seviye):")
        for i, item in enumerate(items):
            boxes = item.sdf.count("<collision")
            print(f"  {i}  {item.label:<24} {boxes:>4} kutu")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="hexapod_terrain", description="Zemin üreteci (S5)")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("liste", help="zemin türleri ve seviyeleri")

    w = sub.add_parser("dunya", help="ROS'lu sim için dünya dosyası yaz")
    w.add_argument("kind", help=f"zemin türü ({', '.join(sets.kinds())})")
    w.add_argument("level", type=int, nargs="?", default=0, help="seviye (0 = en kolay)")
    w.add_argument("--seed", type=int, default=0)
    w.add_argument("-o", "--out", type=Path, required=True)

    p = sub.add_parser("onizleme", help="her zeminin yükseklik haritası ve yan kesiti (PNG)")
    p.add_argument("--out", type=Path, default=Path("docs/zeminler"))
    p.add_argument("--seed", type=int, default=0)

    args = parser.parse_args(argv)

    if args.cmd == "liste":
        _list()
        return 0

    if args.cmd == "dunya":
        item = sets.level(args.kind, args.level, args.seed)
        path = write_world(item, args.out, name=args.kind)
        print(f"{item.label} -> {path}")
        print(f"ros2 launch hexapod_gazebo sim.launch.py world:={path}")
        return 0

    from .preview import write_previews  # matplotlib yalnız burada gerekli

    for path in write_previews(args.out, seed=args.seed):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
