#!/usr/bin/env python3
"""URDF önizlemesi, ROS kurulu olmadan: robotu bir PNG'ye çizer.

Neden var
---------
Testler URDF'in eklem zincirini IK ile karşılaştırır, ama görsel mesh'lerin
(CAD parçalarının) doğru yere oturduğunu göremez. Bu araç URDF'i okuyup
her linkin konumunu hesaplar, STL'leri oraya yerleştirip çizer. RViz'in
yerini tutmaz; ROS kurulana kadar ve hızlı kontrol için.

    python tools/preview_urdf.py                      # onizleme.png, ayakta duruş
    python tools/preview_urdf.py --pose sifir -o sifir.png
    python tools/preview_urdf.py --collision          # mesh yerine çarpışma kutuları
    python tools/preview_urdf.py --all-views          # + yandan ve üstten (daha yavaş)

Gerekenler: numpy, matplotlib (yalnızca bu araç için; robotta gerekmez).
STL'ler pakette olmalı: python tools/cad_sim_model.py --copy-meshes
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import unquote, urlparse

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
for _pkg in ("hexapod_driver", "hexapod_kinematics", "hexapod_description"):
    sys.path.insert(0, str(REPO_ROOT / "src" / _pkg))

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[union-attr]
    except (AttributeError, OSError, ValueError):
        pass

import numpy as np  # noqa: E402

from cadlib.mesh import read_stl  # noqa: E402
from hexapod_description import RobotModel  # noqa: E402
from hexapod_description.__main__ import mesh_set  # noqa: E402
from hexapod_description.urdf import build_urdf, joint_name  # noqa: E402
from hexapod_driver.config import RobotConfig  # noqa: E402

# Duruşlar, derece: (coxa, femur, tibia). "ayakta" bir yürüyüş duruşu değil,
# yalnızca bacakların yere indiği anlaşılır bir görüntü.
POSES = {"sifir": (0.0, 0.0, 0.0), "ayakta": (0.0, 20.0, 10.0)}

COLORS = {"govde": (0.25, 0.25, 0.28), "bacak": (0.88, 0.88, 0.85)}


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = map(float, (np.cos(r), np.sin(r), np.cos(p), np.sin(p),
                                         np.cos(y), np.sin(y)))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _tf(rot, xyz):
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = xyz
    return T


def _origin(el):
    o = el.find("origin")
    if o is None:
        return np.eye(4)
    xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
    return _tf(_rpy(*rpy), xyz)


def _axis_angle(axis, q):
    a = np.asarray(axis, dtype=float)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


def link_poses(root: ET.Element, q: dict[str, float]) -> dict[str, np.ndarray]:
    joints = root.findall("joint")
    by_child = {j.find("child").get("link"): j for j in joints}
    poses: dict[str, np.ndarray] = {}

    def pose(link: str) -> np.ndarray:
        if link in poses:
            return poses[link]
        joint = by_child.get(link)
        if joint is None:
            T = np.eye(4)
        else:
            T = pose(joint.find("parent").get("link")) @ _origin(joint)
            if joint.get("type") == "revolute":
                axis = [float(v) for v in joint.find("axis").get("xyz").split()]
                T = T @ _tf(_axis_angle(axis, q.get(joint.get("name"), 0.0)), (0, 0, 0))
        poses[link] = T
        return T

    for link in root.findall("link"):
        pose(link.get("name"))
    return poses


def _box_triangles(size):
    sx, sy, sz = (s / 2 for s in size)
    v = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return np.array([[v[a], v[b], v[c]] for f in faces for a, b, c in ((f[0], f[1], f[2]),
                                                                       (f[0], f[2], f[3]))])


def gather(root: ET.Element, poses, use_collision: bool):
    """(üçgenler Nx3x3 metre, renk) listesi."""
    cache: dict[str, np.ndarray] = {}
    out = []
    for link in root.findall("link"):
        T_link = poses[link.get("name")]
        items = link.findall("collision") if use_collision else link.findall("visual")
        for item in items:
            T = T_link @ _origin(item)
            geom = item.find("geometry")
            mesh, box, sphere = geom.find("mesh"), geom.find("box"), geom.find("sphere")
            if mesh is not None:
                # file:///C:/x (Windows) -> C:/x ; file:///home/x (Linux) -> /home/x
                raw = unquote(urlparse(mesh.get("filename")).path)
                path = Path(raw.lstrip("/")) if raw[2:3] == ":" else Path(raw)
                key = str(path)
                if key not in cache:
                    cache[key] = np.asarray(read_stl(path), dtype=float).reshape(-1, 3, 3)
                scale = [float(v) for v in mesh.get("scale", "1 1 1").split()]
                tri = cache[key] * scale
            elif box is not None:
                tri = _box_triangles([float(v) for v in box.get("size").split()])
            elif sphere is not None:
                r = float(sphere.get("radius"))
                tri = _box_triangles([2 * r] * 3)
            else:
                continue
            pts = tri.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3]
            mat = item.find("material")
            color = COLORS.get(mat.get("name") if mat is not None else "", (0.6, 0.6, 0.9))
            out.append((pts.reshape(-1, 3, 3), color))
    return out


def render(parts, out: Path, title: str, all_views: bool = False) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    light = np.array([0.3, -0.4, 0.85])
    light /= np.linalg.norm(light)
    views = [(25, -60, "perspektif")]
    if all_views:
        views += [(0, -90, "yandan"), (90, -90, "üstten")]
    fig = plt.figure(figsize=(6 * len(views), 6), dpi=110)
    # Tek koleksiyon: her parça için ayrı çizim nesnesi çok yavaş.
    tris = np.concatenate([p for p, _ in parts])
    colors = np.concatenate([np.tile(c, (len(p), 1)) for p, c in parts])
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, norm, out=np.zeros_like(n), where=norm > 0)
    facecolors = np.clip(colors * (0.35 + 0.65 * np.abs(n @ light))[:, None], 0, 1)
    allpts = tris.reshape(-1, 3)
    lo, hi = allpts.min(axis=0), allpts.max(axis=0)
    center, half = (lo + hi) / 2, (hi - lo).max() / 2
    for k, (elev, azim, label) in enumerate(views):
        ax = fig.add_subplot(1, len(views), k + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(tris, facecolors=facecolors, linewidths=0))
        for i, setter in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
            setter(center[i] - half, center[i] + half)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(label, fontsize=10)
        ax.set_xlabel("x (ileri)")
        ax.set_ylabel("y (sol)")
        ax.set_zlabel("z")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="URDF önizlemesi (ROS'suz)")
    parser.add_argument("-o", "--output", type=Path, default=Path("onizleme.png"))
    parser.add_argument("--pose", choices=sorted(POSES), default="ayakta")
    parser.add_argument("--all-views", action="store_true",
                        help="perspektife ek olarak yandan ve üstten (daha yavaş)")
    parser.add_argument("--collision", action="store_true",
                        help="görsel mesh yerine çarpışma geometrisini çiz")
    args = parser.parse_args(argv)

    model = RobotModel.from_config(RobotConfig.load())
    meshes = None if args.collision else mesh_set("file")
    root = ET.fromstring(build_urdf(model, meshes))

    coxa, femur, tibia = (math.radians(v) for v in POSES[args.pose])
    q = {}
    for leg in model.mounts:
        q[joint_name(leg, "coxa")] = coxa
        q[joint_name(leg, "femur")] = femur
        q[joint_name(leg, "tibia")] = tibia

    parts = gather(root, link_poses(root, q), args.collision)
    kind = "çarpışma geometrisi" if args.collision else "görsel mesh'ler"
    render(parts, args.output,
           f"{model.name} — {kind}, duruş '{args.pose}' "
           f"(coxa/femur/tibia = {'/'.join(f'{v:g}' for v in POSES[args.pose])}°)",
           args.all_views)
    print(f"yazıldı -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
