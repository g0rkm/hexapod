#!/usr/bin/env python3
"""CAD'den simülasyon modeli verisi: kütle, ağırlık merkezi, atalet, çarpışma
kutuları ve görsel mesh yerleşimi.

Neden var
---------
URDF'in her linki için kütle, ağırlık merkezi ve atalet tensörü gerekiyor;
robot tartılmadı, parçalar ayrı ayrı hiç tartılmayacak. Bu araç onları
basılan parçaların STL hacminden hesaplar, yani sayılar "birinin uydurduğu"
değil, CAD + açıkça yazılmış girdilerden (robot.yaml -> simulation.mass_inputs)
tekrar üretilebilir olur. cad_extract.py'nin kardeşidir; ortak kod
tools/cadlib/ altında.

Çıktılar
--------
1. robot.yaml'a yapıştırılacak simulation.links bloğu (ekrana basılır).
2. src/hexapod_description/hexapod_description/meshes.yaml: her linkin görsel
   mesh'lerinin link çerçevesindeki yerleşimi (dosyaya yazılır).
3. --copy-meshes ile STL'leri src/hexapod_description/meshes/ altına kopyalar
   (URDF'te package:// ile kullanılır).

Hangi parça hangi linkte
------------------------
CAD'deki vida ve insert konumlarından çıkarıldı:
  - Coxa servosunun horn'u (üstte) "Coxa_top" üzerinden, bushing'i (altta)
    gövdenin alt plakasından gövdeye bağlı. Yani coxa servosunun GÖVDESİ
    bacakla birlikte döner. Coxa_top ve bushing:1 gövde linkine sayılır.
    (Kanıt: Coxa_top'taki M5 insert'ler bacak çerçevesinde y=+22.56'da,
    gövdedeki M5 insert'ler de aynı yükseklikte ve aynı yarıçapta.)
  - Trochanter servo yuvası coxa ve femur servolarını taşır -> coxa linki.
  - Femur plakaları iki horn'u birbirine bağlar -> femur linki.
  - Tibia servo yuvası tibia servosunu taşır -> tibia linki.
Horn'lar, vidalar, insert'ler ve LED şeritleri ihmal edildi (her biri
birkaç gram).

Çerçeveler (ayrıntı: tools/cadlib/frames.py)
----------
  Link çerçeveleri (URDF): coxa = IK bacak çerçevesi; femur orijini J2'de,
    +x femur boyunca; tibia orijini J3'te, tibia -z boyunca uzanır. Hepsi
    SIFIR DURUŞUNDA tanımlı; CAD bacağı başka bir pozda (femur ~15° yukarıda)
    çizilmiş, araç bu pozu eksenlerden hesaplayıp geri alır.
  Gövde (body-v35.step): +Z yukarı; hexapod-v8.step içindeki yerleşimiyle
    REP-103 gövde çerçevesine taşınır.

Aynalı bacaklar
---------------
Aynalı bacak, normal bacağın kendi orta düzlemine göre yansıması (bkz.
tools/cadlib/frames.py). Kütle özellikleri yalnızca normal bacak için
basılır; URDF üreticisi aynalı bacaklarda y'yi ters çevirir.

Kullanım
--------
    python tools/cad_sim_model.py                 # hesapla, blokları bas
    python tools/cad_sim_model.py --copy-meshes   # STL'leri pakete de kopyala
"""

from __future__ import annotations

import argparse
import math
import shutil
import statistics
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
REPO_ROOT = TOOLS.parent
sys.path.insert(0, str(TOOLS))
for _pkg in ("hexapod_driver", "hexapod_kinematics"):
    sys.path.insert(0, str(REPO_ROOT / "src" / _pkg))

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[union-attr]
    except (AttributeError, OSError, ValueError):
        pass

from cadlib import BODY_STEP, CAD_ROOT, FULL_STEP, LEG_STEP  # noqa: E402
from cadlib.assembly import Assembly  # noqa: E402
from cadlib.frames import (  # noqa: E402
    cad_leg_to_leg_frame,
    cad_to_body_frame,
    coxa_axis_in_body,
    joint_axes,
    leg_mounts,
)
from cadlib.mesh import Box, MassProps, aabb, flip_winding, load_stl  # noqa: E402
from cadlib.transform import (  # noqa: E402
    IDENT,
    MIRROR_Y,
    apply,
    mat_inv,
    mat_mul,
    rigid,
    rot_about_neg_y,
    rpy_of,
)
from hexapod_driver.config import RobotConfig, Value  # noqa: E402
from hexapod_kinematics import HexapodKinematics  # noqa: E402

PACKAGE_DIR = REPO_ROOT / "src" / "hexapod_description"
MESHES_YAML = PACKAGE_DIR / "hexapod_description" / "meshes.yaml"
MESH_DIR = PACKAGE_DIR / "meshes"

LINKS = ("body", "coxa", "femur", "tibia")

# (STEP occurrence adı, STL yolu, link)
LEG_PARTS = [
    ("PLA-Coxa_top:1", CAD_ROOT / "leg" / "pla-coxa_top.stl", "body"),
    ("PLA_bushing:1", CAD_ROOT / "leg" / "pla_bushing.stl", "body"),
    ("PLA-Trochanter_Servo mount:1", CAD_ROOT / "leg" / "pla-trochanter_servo-mount.stl", "coxa"),
    ("PLA_Femur_left:1", CAD_ROOT / "leg" / "pla_femur_left.stl", "femur"),
    ("PLA-Femur_right:1", CAD_ROOT / "leg" / "pla-femur_right.stl", "femur"),
    ("PLA_bushing:2", CAD_ROOT / "leg" / "pla_bushing.stl", "femur"),
    ("PLA_bushing:3", CAD_ROOT / "leg" / "pla_bushing.stl", "femur"),
    ("PLA-Tibia_Servo mount:1", CAD_ROOT / "leg" / "pla-tibia_servo-mount.stl", "tibia"),
    ("PLA_Tibia_main:1", CAD_ROOT / "leg" / "pla_tibia_main.stl", "tibia"),
    ("PLA_Tibia_tip:1", CAD_ROOT / "leg" / "pla_tibia_tip.stl", "tibia"),
]
LEG_SERVOS = [("MG996R v8:1", "coxa"), ("MG996R v8:2", "coxa"), ("MG996R v8:3", "tibia")]

# Kapak (PLA_body lid) YOK: STL'i yalnızca "Baskı Dosyaları/Gövde/" altında
# ve baskı tablası konumunda dışa aktarılmış; CAD'deki yerine oturtulamıyor.
# ~57 cm³, yani ~43 g eksik. Yerini tahmin etmek yerine dışarıda bırakıldı.
BODY_PARTS = [
    ("PLA_body main_bottom:1", CAD_ROOT / "body" / "pla_body-main_bottom.stl"),
    ("PLA_body main_top:1", CAD_ROOT / "body" / "pla_body-main_top.stl"),
    ("PLA_controll platform:1", CAD_ROOT / "body" / "pla_controll-platform.stl"),
    ("PLA_power pack:1", CAD_ROOT / "body" / "pla_power-pack.stl"),
] + [(f"PLA_body spacer:{i}", CAD_ROOT / "body" / "pla_body-spacer.stl") for i in range(1, 7)]

# Çarpışma kutusuna girmeyen parçalar: ayak kapağı ayrı bir küreyle
# modelleniyor, bushing'ler ve Coxa_top zaten kutuların içinde.
NO_COLLISION = {"PLA_Tibia_tip:1", "PLA_bushing:1", "PLA_bushing:2", "PLA_bushing:3",
                "PLA-Coxa_top:1"}


class MassInputs:
    """robot.yaml -> simulation.mass_inputs. Eksik değer MissingValue fırlatır."""

    def __init__(self, config: RobotConfig) -> None:
        raw = (config.raw.get("simulation") or {}).get("mass_inputs") or {}

        def need(key):
            return Value.parse(raw.get(key), f"simulation.mass_inputs.{key}").require()

        self.fill_ratio = float(need("print_fill_ratio"))
        self.density = float(need("petg_density_g_cm3")) * self.fill_ratio * 1e-6  # kg/mm³
        self.servo_mass = float(need("servo_mass_kg"))
        self.servo_size = [float(v) for v in need("servo_size_mm")]
        self.electronics = float(need("electronics_mass_kg"))


class SimModelExtractor:
    """CAD'den link başına kütle özellikleri, çarpışma kutuları ve mesh yerleşimi.

    Kullanım: SimModelExtractor(config).run(); sonuçlar props, collisions,
    meshes, foot_radius ve report alanlarında.
    """

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.inputs = MassInputs(config)
        self.kinematics = HexapodKinematics.from_config(config)
        self.props = {k: MassProps() for k in LINKS}
        self.collisions: dict[str, list] = {}
        self.meshes: dict[str, list] = {k: [] for k in LINKS}
        self.foot_radius = 0.0
        self.report: list[str] = []
        self._bbox: dict[str, list] = {k: [] for k in LINKS}
        self._body_leg_parts: list = []  # gövdeye bağlı bacak parçaları, bacak çerçevesinde
        self._j1_offset = 0.0

    def run(self) -> "SimModelExtractor":
        self._leg()
        self._body()
        return self

    def total_mass(self) -> float:
        legs = sum(self.props[k].m for k in ("coxa", "femur", "tibia"))
        return self.props["body"].m + len(self.kinematics.mounts) * legs

    # -- bacak ---------------------------------------------------------------

    def _leg(self) -> None:
        leg = Assembly(LEG_STEP)
        j1, j2, j3 = joint_axes(leg)
        self._j1_offset = j1[2]
        to_leg = cad_leg_to_leg_frame(j1, j2)
        j2l, j3l = apply(to_leg, j2), apply(to_leg, j3)
        j2l = (j2l[0], 0.0, j2l[2])  # eksen x boyunca; orta noktanın x'i anlamsız
        j3l = (j3l[0], 0.0, j3l[2])

        tip_pts = load_stl(CAD_ROOT / "leg" / "pla_tibia_tip.stl",
                           mat_mul(to_leg, leg.get("PLA_Tibia_tip:1").matrix))
        tip = max(tip_pts, key=lambda p: math.hypot(p[0] - j3l[0], p[2] - j3l[2]))

        q2 = math.atan2(j3l[2] - j2l[2], j3l[0] - j2l[0])  # CAD pozunda femur açısı
        psi = math.atan2(tip[2] - j3l[2], tip[0] - j3l[0])  # tibianın yataya göre açısı
        q3 = psi + math.pi / 2 - q2
        self.report.append(f"CAD pozu: femur {math.degrees(q2):+.2f}°, tibia "
                           f"{math.degrees(q3):+.2f}° (link çerçeveleri sıfır duruşuna "
                           "geri alındı)")
        self.report.append(f"kontrol: coxa {j2l[0]:.3f} mm, femur "
                           f"{math.hypot(j3l[0] - j2l[0], j3l[2] - j2l[2]):.3f} mm, tibia "
                           f"{math.hypot(tip[0] - j3l[0], tip[2] - j3l[2]):.3f} mm")

        # bacak çerçevesi -> link çerçevesi (link çerçeveleri CAD pozunda)
        to_link = {
            "coxa": IDENT,
            "femur": mat_inv(rigid(rot_about_neg_y(q2), j2l)),
            "tibia": mat_inv(rigid(rot_about_neg_y(q2 + q3), j3l)),
        }

        for occ, stl, link in LEG_PARTS:
            m = mat_mul(to_leg, leg.get(occ).matrix)
            if link == "body":
                self._body_leg_parts.append((stl, m))
                continue
            m = mat_mul(to_link[link], m)
            pts = load_stl(stl, m)
            self.props[link].add_mesh(pts, self.inputs.density)
            if occ not in NO_COLLISION:
                self._bbox[link].extend(pts)
            self.meshes[link].append((stl, m))

        horns = [o.origin for o in leg.matching("servo horn")]
        tibia_servos = []
        for occ, link in LEG_SERVOS:
            center = leg.get(occ).origin
            horn = min(horns, key=lambda h: math.dist(h, center))
            box = Box.servo(center, horn, self.inputs.servo_size).moved(
                mat_mul(to_link[link], to_leg))
            self.props[link].add_box(box, self.inputs.servo_mass)
            self._bbox[link].extend(box.corners())
            if link == "tibia":
                tibia_servos.append(box)

        tip_link = [apply(to_link["tibia"], p) for p in tip_pts]
        apex = min(tip_link, key=lambda p: p[2])
        self.foot_radius = _sphere_cap_radius(tip_link, apex)
        self._leg_collisions(tibia_servos, apex[2] + 2 * self.foot_radius)

    def _leg_collisions(self, tibia_servos: list[Box], foot_top: float) -> None:
        """Çarpışma kutuları.

        Tibia tek kutuyla kötü temsil ediliyor: üstte servo yuvası geniş,
        altta uca doğru incelip dışa kavis yapıyor. Tek kutu uca kadar tam
        genişlikte inerse, tibia ~25°'den fazla eğilince ayaktan önce kutunun
        köşesi yere değer. Bu yüzden iki kutu: servo bölgesi ve altı. Alt
        kutu ayak küresinin üst ucunda biter; oradan aşağısı küre.
        """
        for link in ("coxa", "femur"):
            self.collisions[link] = [aabb(self._bbox[link])]
        split = min(c[2] for b in tibia_servos for c in b.corners())
        upper = [p for p in self._bbox["tibia"] if p[2] >= split]
        lower = [p for p in self._bbox["tibia"] if foot_top <= p[2] < split]
        self.collisions["tibia"] = [aabb(upper), aabb(lower)]
        self.report.append(f"tibia çarpışması iki kutu: servo bölgesi z >= {split:.1f} mm, "
                           f"gövde {foot_top:.1f} .. {split:.1f} mm, altı ayak küresi")

    # -- gövde ---------------------------------------------------------------

    def _body(self) -> None:
        full = Assembly(FULL_STEP)
        ref_height = statistics.mean(o.origin[1] for o in full.matching("leg"))
        to_body = cad_to_body_frame(float(self.config.forward_offset_deg.require()), ref_height)
        body_occ = next(o for o in full if o.name.lower().startswith("body"))
        b35_to_body = mat_mul(to_body, body_occ.matrix)

        body = Assembly(BODY_STEP)
        pack = None
        for occ, stl in BODY_PARTS:
            if not stl.is_file():
                raise SystemExit(f"HATA: STL bulunamadı: {stl}")
            m = mat_mul(b35_to_body, body.get(occ).matrix)
            pts = load_stl(stl, m)
            self.props["body"].add_mesh(pts, self.inputs.density)
            self._bbox["body"].extend(pts)
            self.meshes["body"].append((stl, m))
            if "power pack" in occ:
                pack = aabb(pts)
        self.collisions["body"] = [aabb(self._bbox["body"])]

        # elektronik: güç paketinin kutusunu dolduran düzgün kütle
        self.props["body"].add_box(Box.aligned(pack[:3], pack[3:]), self.inputs.electronics)
        self.report.append("elektronik kütlesi güç paketi kutusuna yayıldı: merkez "
                           f"({pack[0]:.1f}, {pack[1]:.1f}, {pack[2]:.1f}) mm")

        # Gövdeye bağlı bacak parçaları, her bacak montajında. Aynalı bacaklarda
        # kütle y'si ters çevrilir (üçgen sırası da, yoksa hacim negatif çıkar);
        # mesh çevrilemez (URDF yansıtmaz), ama Coxa_top ve bushing orta
        # düzleme göre neredeyse simetrik.
        mounts = leg_mounts(self.kinematics)
        for spec in self.config.legs:
            mount = mounts[spec.id]
            for stl, m in self._body_leg_parts:
                if spec.mirrored:
                    pts = flip_winding(load_stl(stl, mat_mul(mount, mat_mul(MIRROR_Y, m))))
                else:
                    pts = load_stl(stl, mat_mul(mount, m))
                self.props["body"].add_mesh(pts, self.inputs.density)
                self.meshes["body"].append((stl, mat_mul(mount, m)))

        # tutarlılık: CAD'deki coxa eksenleri IK montajlarıyla örtüşmeli
        worst = max(
            min(math.hypot(p[0] - mt[0][3], p[1] - mt[1][3]) for mt in mounts.values())
            for _, p, _ in coxa_axis_in_body(full, to_body, self._j1_offset)
        )
        self.report.append("kontrol: CAD coxa eksenleri ile robot.yaml montajları arasındaki "
                           f"en büyük fark {worst:.3f} mm"
                           + ("  -> GEÇTİ" if worst < 0.5 else "  -> UYARI!"))


def _sphere_cap_radius(points, apex) -> float:
    """Ayak ucu küre yarıçapı.

    Uçtan h derinlikteki noktanın eksene uzaklığı ρ ise r = (ρ² + h²) / 2h.
    Uca yakın (0.3-2 mm) noktaların ortancası alınır.
    """
    radii = []
    for p in points:
        h = p[2] - apex[2]
        if 0.3 < h < 2.0:
            rho2 = (p[0] - apex[0]) ** 2 + (p[1] - apex[1]) ** 2
            radii.append((rho2 + h * h) / (2 * h))
    return statistics.median(radii)


# ---------------------------------------------------------------------------
# Çıktı
# ---------------------------------------------------------------------------


def _fmt(values, nd=2) -> str:
    return "[" + ", ".join(f"{v:.{nd}f}" for v in values) + "]"


def print_links_block(model: SimModelExtractor) -> None:
    fill = model.inputs.fill_ratio
    print("  # --- tools/cad_sim_model.py çıktısı: elle düzenlemeyin, aracı yeniden çalıştırın ---")
    print("  # Link çerçeveleri URDF'teki gibi, sıfır duruşunda. Birimler mm, kg, kg·mm².")
    print("  # inertia sırası: [ixx, iyy, izz, ixy, ixz, iyz], ağırlık merkezinde.")
    print("  # collision_boxes: her biri [merkez x, y, z, boy x, y, z]; ayak ucu ayrıca")
    print("  # küre (leg.foot_tip_radius). Aynalı bacaklarda URDF üreticisi y'yi çevirir.")
    print("  links:")
    notes = {
        "body": "gövde plakaları + 6x Coxa_top/bushing (kapak hariç) + elektronik tahmini",
        "coxa": "trochanter yuvası + 2x MG996R (katalog)",
        "femur": "iki femur plakası",
        "tibia": "tibia gövdesi + ayak kapağı + MG996R (katalog)",
    }
    for link in LINKS:
        mp = model.props[link]
        src = f"TAHMİN - CAD STL x PETG x doluluk {fill}; {notes[link]} (tools/cad_sim_model.py)"
        boxes = "[" + ", ".join(_fmt(b, 1) for b in model.collisions[link]) + "]"
        print(f"    {link}:")
        print(f"      mass_kg: {{value: {mp.m:.4f}, source: \"{src}\", measured: false}}")
        print(f"      com_mm: {{value: {_fmt(mp.com())}, source: \"{src}\", measured: false}}")
        print(f"      inertia_kg_mm2: {{value: {_fmt(mp.inertia_at_com(), 1)}, "
              f"source: \"{src}\", measured: false}}")
        print(f"      collision_boxes_mm: {{value: {boxes}, source: \"CAD STL + servo kutusu "
              f"sınırları, link çerçevesinde (tools/cad_sim_model.py)\", measured: false}}")


def write_meshes_yaml(meshes: dict[str, list]) -> None:
    lines = [
        "# BU DOSYA ÜRETİLDİ: tools/cad_sim_model.py. Elle düzenlemeyin.",
        "#",
        "# Görsel mesh'lerin link çerçevesindeki yerleşimi (URDF <visual><origin>).",
        "# xyz metre, rpy radyan (URDF sırası). Mesh'ler mm cinsinden, URDF'te",
        "# 0.001 ölçekle kullanılır. STL'ler pakete",
        "# `python tools/cad_sim_model.py --copy-meshes` ile kopyalanır.",
        "#",
        "# Aynalı bacaklar da normal bacağın mesh'lerini kullanır: URDF bir mesh'i",
        "# yansıtamaz. Yalnızca görünüş; kütle ve çarpışma hesabı aynalamayı",
        "# hesaba katar.",
        "",
    ]
    for link in LINKS:
        lines.append(f"{link}:")
        for stl, m in meshes[link]:
            xyz = [m[i][3] / 1000.0 for i in range(3)]
            lines.append(f"  - {{file: {stl.name}, xyz: {_fmt(xyz, 6)}, "
                         f"rpy: {_fmt(rpy_of(m), 6)}}}")
    MESHES_YAML.parent.mkdir(parents=True, exist_ok=True)
    MESHES_YAML.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def copy_meshes(meshes: dict[str, list]) -> None:
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    names = {stl.name: stl for items in meshes.values() for stl, _ in items}
    for name, stl in names.items():
        shutil.copy2(stl, MESH_DIR / name)
    print(f"{len(names)} STL kopyalandı -> {MESH_DIR}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="CAD'den simülasyon modeli verisi")
    parser.add_argument("--copy-meshes", action="store_true",
                        help="STL'leri src/hexapod_description/meshes/ altına kopyala")
    args = parser.parse_args(argv)

    missing = [p for p in (LEG_STEP, FULL_STEP, BODY_STEP) if not p.is_file()]
    for path in missing:
        print(f"HATA: CAD dosyası bulunamadı: {path}", file=sys.stderr)
    if missing:
        return 2

    model = SimModelExtractor(RobotConfig.load()).run()

    print("=" * 72)
    print("SİMÜLASYON MODELİ — CAD'den")
    print("=" * 72)
    for line in model.report:
        print("  " + line)
    print(f"  ayak ucu küre yarıçapı: {model.foot_radius:.2f} mm")
    print(f"  toplam kütle tahmini: {model.total_mass():.3f} kg")
    print()
    print("robot.yaml -> simulation altına:")
    print()
    print_links_block(model)
    print()

    write_meshes_yaml(model.meshes)
    print(f"mesh yerleşimi yazıldı -> {MESHES_YAML.relative_to(REPO_ROOT)}")
    if args.copy_meshes:
        copy_meshes(model.meshes)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
