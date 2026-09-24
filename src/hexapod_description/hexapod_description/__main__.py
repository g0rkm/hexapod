"""robot.yaml -> URDF, komut satırından.

    python -m hexapod_description -o hexapod.urdf                 # mesh'siz
    python -m hexapod_description -o hexapod.urdf --meshes package
    python -m hexapod_description -o hexapod.urdf --meshes file

--meshes:
  none     görseller çarpışma kutularından (STL gerekmez)
  package  package://hexapod_description/meshes/ (ROS kurulumunda)
  file     file:// ile paketin meshes/ klasörü (ROS'suz görüntüleyiciler için)

Özet ve uyarılar stderr'e yazılır; -o verilmezse URDF stdout'a gider.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from hexapod_driver.config import RobotConfig

from .model import RobotModel
from .urdf import MeshSet, build_urdf

PACKAGE_MESHES = Path(__file__).resolve().parent.parent / "meshes"


def mesh_set(mode: str) -> MeshSet | None:
    if mode == "none":
        return None
    if mode == "package":
        return MeshSet.load("package://hexapod_description/meshes/")
    meshes = MeshSet.load(PACKAGE_MESHES.as_uri() + "/")
    missing = sorted(f for f in meshes.files() if not (PACKAGE_MESHES / f).is_file())
    if missing:
        raise SystemExit(
            f"HATA: {len(missing)} STL yok ({', '.join(missing[:3])}...). Önce:\n"
            "  python tools/cad_sim_model.py --copy-meshes"
        )
    return meshes


def main(argv: list[str] | None = None) -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[union-attr]
        except (AttributeError, OSError, ValueError):
            pass

    parser = argparse.ArgumentParser(description="robot.yaml'dan URDF üret")
    parser.add_argument("-o", "--output", type=Path, help="yazılacak dosya (yoksa stdout)")
    parser.add_argument("--config", type=Path, help="robot.yaml yolu (yoksa otomatik bulunur)")
    parser.add_argument("--meshes", choices=("none", "package", "file"), default="none")
    args = parser.parse_args(argv)

    model = RobotModel.from_config(RobotConfig.load(args.config))
    text = build_urdf(model, mesh_set(args.meshes))

    if args.output:
        args.output.write_text(text, encoding="utf-8", newline="\n")
    else:
        sys.stdout.write(text)

    provisional = model.provisional_joints()
    info = [
        f"URDF: {len(model.mounts)} bacak, {3 * len(model.mounts)} eklem, "
        f"toplam kütle {model.total_mass():.3f} kg (TAHMİN, robot.yaml -> simulation)",
    ]
    if provisional:
        info.append(f"UYARI: {len(provisional)}/18 eklemde kalibrasyon limiti yok, geçici "
                    "limit kullanıldı (simulation.provisional_joint_limits_deg).")
    if args.output:
        info.append(f"yazıldı -> {args.output}")
    print("\n".join(info), file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
