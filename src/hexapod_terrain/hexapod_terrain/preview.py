"""Zeminlerin önizlemesi: yükseklik haritası + yan kesit (GOREVLER.md S5).

Gazebo açmadan zeminin nasıl bir şey olduğunu gösterir; "bitti sayılır"
şartındaki örnek görüntüler bununla üretilir (docs/zeminler/).

Çizilen şey `height(x, y)`, yani robotun gerçekten bastığı yüzey. SDF ile
tutarlılığı ayrıca test ediliyor (tests/test_terrain.py), o yüzden burada
görülen zemin Gazebo'daki zeminle aynıdır.

matplotlib yalnız bu dosya için gerekli (robotta ve eğitimde gerekmez);
tools/preview_urdf.py ile aynı yaklaşım.
"""

from __future__ import annotations

from pathlib import Path

from . import sets
from .terrain import SPAWN_FLAT_HALF, Terrain

#: Önizleme penceresi (m): robot 10 s'de ~1 m gider, 3 m'lik pencere yeter.
_X = (-1.0, 3.0)
_Y = (-1.2, 1.2)
_GRID = 240


def figure(item: Terrain):
    """Bir zeminin önizleme şekli (matplotlib Figure)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    xs = np.linspace(*_X, _GRID)
    ys = np.linspace(*_Y, _GRID // 2)
    zs = np.array([[item.height(float(x), float(y)) for x in xs] for y in ys])

    fig, (top, side) = plt.subplots(
        2, 1, figsize=(7.2, 5.4), height_ratios=(2.2, 1.0), constrained_layout=True)

    span = max(abs(zs.min()), abs(zs.max()), 1e-3)
    mesh = top.pcolormesh(xs, ys, zs * 1000.0, cmap="terrain", shading="nearest",
                          vmin=-span * 1000.0, vmax=span * 1000.0)
    fig.colorbar(mesh, ax=top, label="yükseklik (mm)")
    # Robotun doğduğu kare ve yürüyüş yönü.
    top.add_patch(plt.Rectangle((-SPAWN_FLAT_HALF, -SPAWN_FLAT_HALF), 2 * SPAWN_FLAT_HALF,
                                2 * SPAWN_FLAT_HALF, fill=False, ec="red", lw=1.2, ls="--"))
    top.annotate("", xy=(1.1, 0.0), xytext=(0.36, 0.0),
                 arrowprops={"arrowstyle": "->", "color": "red", "lw": 1.4})
    top.text(0.0, -SPAWN_FLAT_HALF - 0.12, "robot burada doğar, +x yönüne yürür",
             color="red", fontsize=8, ha="center", va="top")
    top.set_aspect("equal")
    top.set_xlabel("x (m, ileri)")
    top.set_ylabel("y (m, sol)")
    top.set_title(f"{item.label}   —   yukarıdan yükseklik haritası")

    mid = zs[zs.shape[0] // 2]
    side.fill_between(xs, mid * 1000.0, -60.0, color="0.75", step="mid")
    side.plot(xs, mid * 1000.0, color="0.2", lw=1.2, drawstyle="steps-mid")
    side.axvspan(-SPAWN_FLAT_HALF, SPAWN_FLAT_HALF, color="red", alpha=0.08)
    side.set_xlim(*_X)
    side.set_ylim(min(-20.0, mid.min() * 1000.0 - 15.0), max(20.0, mid.max() * 1000.0 + 15.0))
    side.set_xlabel("x (m, ileri)")
    side.set_ylabel("yükseklik (mm)")
    side.set_title("y = 0 boyunca yan kesit", fontsize=10)
    side.grid(alpha=0.3)

    boxes = item.sdf.count("<collision")
    fig.text(0.99, 0.01, f"{boxes} çarpışma kutusu · hexapod_terrain (S5)",
             ha="right", va="bottom", fontsize=7, color="0.4")
    return fig


def write_preview(item: Terrain, path: str | Path) -> Path:
    import matplotlib.pyplot as plt

    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    fig = figure(item)
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


def write_previews(out_dir: str | Path, seed: int = 0) -> list[Path]:
    """Her zemin türünün en zor seviyesi için bir önizleme (örnek görüntüler)."""
    out = Path(out_dir)
    paths = []
    for kind in sets.kinds():
        items = sets.levels(kind, seed)
        item = items[-1]
        name = {"düz": "duz", "eğim": "egim", "yan eğim": "yan-egim", "kaygan": "kaygan",
                "basamak": "basamak", "merdiven": "merdiven", "engebe": "engebe",
                "çukur": "cukur", "yayla": "yayla"}.get(kind, kind)
        paths.append(write_preview(item, out / f"{name}.png"))
    return paths


__all__ = ["figure", "write_preview", "write_previews"]
