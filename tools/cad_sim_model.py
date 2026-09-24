#!/usr/bin/env python3
"""CAD'den simülasyon modeli verisi: kütle, ağırlık merkezi, atalet, çarpışma
kutuları ve görsel mesh yerleşimi.

Neden var
---------
URDF'in her linki için kütle, ağırlık merkezi ve atalet tensörü gerekiyor;
robot tartılmadı, parçalar ayrı ayrı hiç tartılmayacak. Bu araç onları
basılan parçaların STL hacminden hesaplar, yani sayılar "birinin uydurduğu"
değil, CAD + açıkça yazılmış girdilerden (robot.yaml -> simulation.mass_inputs)
tekrar üretilebilir olur. cad_extract.py'nin kardeşidir.

Çıktılar
--------
1. robot.yaml'a yapıştırılacak simulation.links bloğu (ekrana basılır).
2. src/hexapod_description/hexapod_description/meshes.yaml: her linkin görsel
   mesh'lerinin link çerçevesindeki yerleşimi (dosyaya yazılır).
3. --copy-meshes ile STL'leri src/hexapod_description/meshes/ altına kopyalar
   (git'e girmez; URDF'te package:// ile kullanılır).

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

Çerçeveler
----------
  CAD bacak çerçevesi (leg-v2-v20.step): +Y yukarı, bacak -Z yönünde dışarı.
  IK bacak çerçevesi (hexapod_kinematics.leg): orijin coxa ekseni üzerinde,
    femur ekleminin yüksekliğinde; +x dışarı, +z yukarı.
    Dönüşüm: x = -Z_cad, y = -X_cad, z = +Y_cad (orijin J1/J2'den).
  Link çerçeveleri (URDF): coxa = bacak çerçevesi; femur orijini J2'de, +x
    femur boyunca; tibia orijini J3'te, tibia -z boyunca uzanır. Hepsi
    SIFIR DURUŞUNDA tanımlı; CAD bacağı başka bir pozda (femur ~15° yukarıda)
    çizilmiş, araç bu pozu eksenlerden hesaplayıp geri alır.
  Gövde (body-v35.step): +Z yukarı; hexapod-v8.step içindeki yerleşimiyle
    REP-103 gövde çerçevesine taşınır.

Aynalı bacaklar
---------------
Aynalı bacak, normal bacağın kendi orta düzlemine göre yansıması (bkz.
cad_extract.py). Kütle özellikleri yalnızca normal bacak için basılır;
URDF üreticisi aynalı bacaklarda y'yi ters çevirir.

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

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(REPO_ROOT / "src" / "hexapod_driver"))

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[union-attr]
    except (AttributeError, OSError, ValueError):
        pass

import stepasm as S  # noqa: E402
from cad_extract import CAD_ROOT, FULL_STEP, LEG_STEP, occurrences, read_stl  # noqa: E402
from hexapod_driver.config import RobotConfig, Value  # noqa: E402

BODY_STEP = CAD_ROOT / "body" / "body-v35.step"
PACKAGE_DIR = REPO_ROOT / "src" / "hexapod_description"
MESHES_YAML = PACKAGE_DIR / "hexapod_description" / "meshes.yaml"
MESH_DIR = PACKAGE_DIR / "meshes"

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
# modelleniyor, bushing'ler zaten kutunun içinde.
NO_COLLISION = {"PLA_Tibia_tip:1", "PLA_bushing:1", "PLA_bushing:2", "PLA_bushing:3",
                "PLA-Coxa_top:1"}

Mat = tuple  # 3x4, stepasm biçimi


# ---------------------------------------------------------------------------
# Küçük lineer cebir
# ---------------------------------------------------------------------------


def rigid(rot, origin) -> Mat:
    """3x3 dönme (satırlar) + öteleme -> 3x4."""
    return tuple(tuple(rot[i]) + (origin[i],) for i in range(3))


def rot_about_neg_y(theta: float):
    """-y ekseni etrafında theta: +x yukarı (+z) döner. URDF femur/tibia ekseni."""
    c, s = math.cos(theta), math.sin(theta)
    return ((c, 0.0, -s), (0.0, 1.0, 0.0), (s, 0.0, c))


def rpy_of(m: Mat) -> tuple[float, float, float]:
    """3x4'ün dönme kısmı -> URDF rpy (R = Rz(y) Ry(p) Rx(r))."""
    r20 = max(-1.0, min(1.0, m[2][0]))
    pitch = -math.asin(r20)
    if abs(r20) < 1 - 1e-9:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    else:  # gimbal kilidi; yaw'ı sıfır al
        roll = math.atan2(-m[1][2], m[1][1])
        yaw = 0.0
    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Kütle özellikleri
# ---------------------------------------------------------------------------


class MassProps:
    """Kütle, birinci moment ve ikinci moment (∫ x xᵀ dm) toplayıcısı.

    Hepsi aynı çerçevede tutulur; atalet en sonda ağırlık merkezine taşınır.
    Birimler: kg, mm.
    """

    def __init__(self) -> None:
        self.m = 0.0
        self.s = [0.0, 0.0, 0.0]
        self.c = [[0.0] * 3 for _ in range(3)]

    def add_mesh(self, points, density_kg_mm3: float) -> None:
        """Kapalı üçgen ağ; her üçgen orijinle bir dörtyüzlü oluşturur."""
        for k in range(0, len(points) - 2, 3):
            a, b, c = points[k], points[k + 1], points[k + 2]
            v = (a[0] * (b[1] * c[2] - b[2] * c[1])
                 - a[1] * (b[0] * c[2] - b[2] * c[0])
                 + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
            dm = v * density_kg_mm3
            tot = [a[i] + b[i] + c[i] for i in range(3)]
            self.m += dm
            for i in range(3):
                self.s[i] += dm * tot[i] / 4.0
                for j in range(3):
                    self.c[i][j] += dm / 20.0 * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j]
                                                 + tot[i] * tot[j])

    def add_box(self, box: "Box", mass: float) -> None:
        """Düzgün yoğunluklu, herhangi bir yönde duran kutu."""
        self.m += mass
        for i in range(3):
            self.s[i] += mass * box.center[i]
            for j in range(3):
                # kendi ekseninde ∫ u² dm = m·boy²/12; çerçeveye döndür
                own = sum(box.axes[k][i] * box.axes[k][j] * mass * box.size[k] ** 2 / 12.0
                          for k in range(3))
                self.c[i][j] += mass * box.center[i] * box.center[j] + own

    def com(self):
        return tuple(v / self.m for v in self.s)

    def inertia_at_com(self):
        """(ixx, iyy, izz, ixy, ixz, iyz), kg·mm², ağırlık merkezinde."""
        g = self.com()
        cc = [[self.c[i][j] - self.m * g[i] * g[j] for j in range(3)] for i in range(3)]
        tr = cc[0][0] + cc[1][1] + cc[2][2]
        return (tr - cc[0][0], tr - cc[1][1], tr - cc[2][2],
                -cc[0][1], -cc[0][2], -cc[1][2])


class Box:
    """Merkez, üç birim eksen ve eksen başına boy (mm)."""

    def __init__(self, center, axes, size) -> None:
        self.center = tuple(center)
        self.axes = [tuple(a) for a in axes]
        self.size = tuple(size)

    def corners(self):
        out = []
        for sx in (-0.5, 0.5):
            for sy in (-0.5, 0.5):
                for sz in (-0.5, 0.5):
                    f = (sx * self.size[0], sy * self.size[1], sz * self.size[2])
                    out.append(tuple(self.center[i] + sum(f[k] * self.axes[k][i] for k in range(3))
                                     for i in range(3)))
        return out

    def moved(self, m: Mat) -> "Box":
        rot = [row[:3] for row in m]
        turn = [tuple(sum(rot[i][k] * a[k] for k in range(3)) for i in range(3)) for a in self.axes]
        return Box(S.apply(m, self.center), turn, self.size)


def servo_box(center, horn, size) -> Box:
    """MG996R gövdesi, CAD'deki konumu ve horn'unun yerinden.

    Servo modelinin orijini gövdenin merkezinde; horn mil ekseni üzerinde,
    merkezden mil yönünde ~17 mm ve gövdenin uzun kenarı boyunca ~10 mm
    kaçık (MG996R'da mil gövdenin bir ucuna yakın). Yani merkezden horn'a
    giden vektörün baskın bileşeni mil (yükseklik) eksenini, kalanı uzun
    kenarı verir. size = [uzunluk, genişlik, yükseklik(mil yönü)].
    """
    d = [horn[i] - center[i] for i in range(3)]
    k = max(range(3), key=lambda i: abs(d[i]))
    shaft = [0.0, 0.0, 0.0]
    shaft[k] = math.copysign(1.0, d[k])
    rest = [d[i] - (d[k] if i == k else 0.0) for i in range(3)]
    n = math.sqrt(sum(v * v for v in rest))
    length = [v / n for v in rest]
    width = S._cross(shaft, length)
    return Box(center, (length, width, shaft), size)


def aabb(points) -> list[float]:
    """Eksenlere hizalı sınır kutusu: [merkez x, y, z, boy x, y, z]."""
    lo = [min(p[i] for p in points) for i in range(3)]
    hi = [max(p[i] for p in points) for i in range(3)]
    return [(lo[i] + hi[i]) / 2 for i in range(3)] + [hi[i] - lo[i] for i in range(3)]


# ---------------------------------------------------------------------------
# Çerçeveler
# ---------------------------------------------------------------------------


def joint_axes(items):
    """J1, J2, J3 eksen orta noktaları, CAD bacak çerçevesinde (cad_extract ile aynı yöntem)."""
    horns = sorted((p for n, p, _ in items if "servo horn" in n.lower()), key=lambda p: -p[2])
    bushings = sorted((p for n, p, _ in items if "bushing" in n.lower()), key=lambda p: -p[2])
    if len(horns) != 3 or len(bushings) != 3:
        raise SystemExit("HATA: bacak STEP'inde 3 horn + 3 bushing bulunamadı.")
    return [tuple((h[i] + b[i]) / 2 for i in range(3)) for h, b in zip(horns, bushings)]


def cad_leg_to_leg_frame(j1, j2) -> Mat:
    """CAD bacak çerçevesi -> IK bacak çerçevesi.  x=-Z, y=-X, z=+Y."""
    rot = ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    origin_cad = (0.0, j2[1], j1[2])  # coxa ekseni üstünde, femur ekleminin yüksekliğinde
    return rigid(rot, tuple(-sum(rot[i][k] * origin_cad[k] for k in range(3)) for i in range(3)))


def cad_to_body_frame(forward_offset_deg: float, ref_height: float) -> Mat:
    """hexapod-v8.step çerçevesi (+Y yukarı) -> REP-103 gövde çerçevesi.

    gövde_azimut = forward_offset - cad_azimut (robot.yaml frames.body).
    """
    f = math.radians(forward_offset_deg)
    c, s = math.cos(f), math.sin(f)
    rot = ((c, 0.0, s), (s, 0.0, -c), (0.0, 1.0, 0.0))
    return rigid(rot, (0.0, 0.0, -ref_height))


def load_stl_in(m: Mat, stl: Path):
    return [S.apply(m, p) for p in read_stl(stl)]


# ---------------------------------------------------------------------------
# Ana hesap
# ---------------------------------------------------------------------------


def build(config: RobotConfig):
    sim = (config.raw.get("simulation") or {}).get("mass_inputs") or {}

    def need(key):
        return Value.parse(sim.get(key), f"simulation.mass_inputs.{key}").require()

    density = float(need("petg_density_g_cm3")) * float(need("print_fill_ratio")) * 1e-6  # kg/mm³
    servo_mass = float(need("servo_mass_kg"))
    servo_size = [float(v) for v in need("servo_size_mm")]
    electronics = float(need("electronics_mass_kg"))

    report: list[str] = []

    # --- bacak --------------------------------------------------------------
    leg_items = occurrences(LEG_STEP)
    by_name = {n: m for n, _, m in leg_items}
    j1, j2, j3 = joint_axes(leg_items)
    to_leg = cad_leg_to_leg_frame(j1, j2)
    j2l, j3l = S.apply(to_leg, j2), S.apply(to_leg, j3)
    j2l = (j2l[0], 0.0, j2l[2])  # eksen x boyunca; orta noktanın x'i anlamsız
    j3l = (j3l[0], 0.0, j3l[2])

    tip_matrix = S.mat_mul(to_leg, by_name["PLA_Tibia_tip:1"])
    tip_pts = load_stl_in(tip_matrix, CAD_ROOT / "leg" / "pla_tibia_tip.stl")
    tip = max(tip_pts, key=lambda p: math.hypot(p[0] - j3l[0], p[2] - j3l[2]))

    q2 = math.atan2(j3l[2] - j2l[2], j3l[0] - j2l[0])  # CAD pozunda femur açısı
    psi = math.atan2(tip[2] - j3l[2], tip[0] - j3l[0])  # tibianın yataya göre açısı
    q3 = psi + math.pi / 2 - q2
    report.append(f"CAD pozu: femur {math.degrees(q2):+.2f}°, tibia {math.degrees(q3):+.2f}° "
                  f"(link çerçeveleri sıfır duruşuna geri alındı)")
    report.append(f"kontrol: coxa {j2l[0]:.3f} mm, femur "
                  f"{math.hypot(j3l[0] - j2l[0], j3l[2] - j2l[2]):.3f} mm, tibia "
                  f"{math.hypot(tip[0] - j3l[0], tip[2] - j3l[2]):.3f} mm")

    # bacak çerçevesi -> link çerçevesi (link çerçevesi CAD pozunda)
    link_in_leg = {
        "coxa": rigid(((1, 0, 0), (0, 1, 0), (0, 0, 1)), (0.0, 0.0, 0.0)),
        "femur": rigid(rot_about_neg_y(q2), j2l),
        "tibia": rigid(rot_about_neg_y(q2 + q3), j3l),
    }
    leg_to_link = {k: S.mat_inv(v) for k, v in link_in_leg.items()}

    props = {k: MassProps() for k in ("body", "coxa", "femur", "tibia")}
    bbox: dict[str, list] = {k: [] for k in props}
    meshes: dict[str, list] = {k: [] for k in props}
    body_leg_parts = []  # gövdeye bağlı bacak parçaları, bacak çerçevesinde

    for occ, stl, link in LEG_PARTS:
        if occ not in by_name:
            raise SystemExit(f"HATA: bacak STEP'inde '{occ}' yok.")
        if link == "body":
            m = S.mat_mul(to_leg, by_name[occ])
            body_leg_parts.append((occ, stl, m))
            continue
        m = S.mat_mul(leg_to_link[link], S.mat_mul(to_leg, by_name[occ]))
        pts = load_stl_in(m, stl)
        props[link].add_mesh(pts, density)
        if occ not in NO_COLLISION:
            bbox[link].extend(pts)
        meshes[link].append((stl, m))

    horns = [p for n, p, _ in leg_items if "servo horn" in n.lower()]
    servo_boxes: dict[str, list[Box]] = {k: [] for k in props}
    for occ, link in LEG_SERVOS:
        center = next(p for n, p, _ in leg_items if n == occ)
        horn = min(horns, key=lambda h: math.dist(h, center))
        box = servo_box(center, horn, servo_size).moved(S.mat_mul(leg_to_link[link], to_leg))
        props[link].add_box(box, servo_mass)
        bbox[link].extend(box.corners())
        servo_boxes[link].append(box)

    # ayak ucu küre yarıçapı: uçtan h derinlikteki noktanın eksene uzaklığı ρ
    # ise r = (ρ² + h²) / 2h
    tip_link = [S.apply(leg_to_link["tibia"], p) for p in tip_pts]
    apex = min(tip_link, key=lambda p: p[2])
    radii = []
    for p in tip_link:
        h = p[2] - apex[2]
        if 0.3 < h < 2.0:
            rho2 = (p[0] - apex[0]) ** 2 + (p[1] - apex[1]) ** 2
            radii.append((rho2 + h * h) / (2 * h))
    foot_r = statistics.median(radii)

    # Çarpışma kutuları. Tibia tek kutuyla kötü temsil ediliyor: üstte servo
    # yuvası geniş, altta uca doğru incelip dışa kavis yapıyor. Tek kutu uca
    # kadar tam genişlikte inerse, tibia ~25°'den fazla eğilince ayaktan önce
    # kutunun köşesi yere değer. Bu yüzden iki kutu: servo bölgesi ve altı.
    # Alt kutu ayak küresinin üst ucunda biter; oradan aşağısı küre.
    collisions = {k: [aabb(bbox[k])] for k in ("coxa", "femur")}
    split = min(c[2] for b in servo_boxes["tibia"] for c in b.corners())
    foot_top = apex[2] + 2 * foot_r
    upper = [p for p in bbox["tibia"] if p[2] >= split]
    lower = [p for p in bbox["tibia"] if foot_top <= p[2] < split]
    collisions["tibia"] = [aabb(upper), aabb(lower)]
    report.append(f"tibia çarpışması iki kutu: servo bölgesi z >= {split:.1f} mm, "
                  f"gövde {foot_top:.1f} .. {split:.1f} mm, altı ayak küresi")

    # --- gövde --------------------------------------------------------------
    full = occurrences(FULL_STEP)
    leg_heights = [pos[1] for n, pos, _ in full if n.lower().startswith("leg")]
    ref_height = statistics.mean(leg_heights)
    to_body = cad_to_body_frame(float(config.forward_offset_deg.require()), ref_height)
    body_occ = next(m for n, _, m in full if n.lower().startswith("body"))
    b35_to_body = S.mat_mul(to_body, body_occ)

    body_items = {n: m for n, _, m in occurrences(BODY_STEP)}
    for occ, stl in BODY_PARTS:
        if not stl.is_file():
            raise SystemExit(f"HATA: STL bulunamadı: {stl}")
        m = S.mat_mul(b35_to_body, body_items[occ])
        pts = load_stl_in(m, stl)
        props["body"].add_mesh(pts, density)
        bbox["body"].extend(pts)
        meshes["body"].append((stl, m))
        if "power pack" in occ:
            pack = pts

    collisions["body"] = [aabb(bbox["body"])]

    # elektronik: güç paketinin kutusunu dolduran düzgün kütle
    pack_box = aabb(pack)
    props["body"].add_box(Box(pack_box[:3], ((1, 0, 0), (0, 1, 0), (0, 0, 1)), pack_box[3:]),
                          electronics)
    report.append("elektronik kütlesi güç paketi kutusuna yayıldı: merkez "
                  f"({pack_box[0]:.1f}, {pack_box[1]:.1f}, {pack_box[2]:.1f}) mm")

    # gövdeye bağlı bacak parçaları, her bacak montajında. Aynalı bacaklarda
    # kütle y'si ters çevrilir; mesh çevrilemez (URDF yansıtmaz), ama Coxa_top
    # ve bushing orta düzleme göre neredeyse simetrik.
    kin_mounts = _mounts(config)
    mirror = ((1, 0, 0, 0), (0, -1, 0, 0), (0, 0, 1, 0))
    for leg in config.legs:
        mount = kin_mounts[leg.id]
        for _occ, stl, m in body_leg_parts:
            flip = mirror if leg.mirrored else S.IDENT
            # add_mesh üçgen yönünden hacim işaretini alır; yansıma yönü
            # ters çevirir, o yüzden aynalıda üçgen sırası da ters çevrilir.
            pts = [S.apply(mount, S.apply(flip, p)) for p in load_stl_in(m, stl)]
            if leg.mirrored:
                pts = [q for k in range(0, len(pts) - 2, 3) for q in (pts[k], pts[k + 2], pts[k + 1])]
            props["body"].add_mesh(pts, density)
            meshes["body"].append((stl, S.mat_mul(mount, m)))

    # --- tutarlılık: CAD'deki coxa eksenleri IK montajlarıyla örtüşmeli ------
    worst = 0.0
    for n, _, m in full:
        if not n.lower().startswith("leg"):
            continue
        mirrored = "mirror" in n.lower()
        axis = S.apply(S.mat_mul(to_body, m), (0, 0, -j1[2] if mirrored else j1[2]))
        best = min(math.hypot(axis[0] - mt[0][3], axis[1] - mt[1][3]) for mt in kin_mounts.values())
        worst = max(worst, best)
    report.append(f"kontrol: CAD coxa eksenleri ile robot.yaml montajları arasındaki en büyük "
                  f"fark {worst:.3f} mm" + ("  -> GEÇTİ" if worst < 0.5 else "  -> UYARI!"))

    return props, collisions, meshes, foot_r, report


def _mounts(config) -> dict[int, Mat]:
    """IK bacak çerçevesi -> gövde çerçevesi, her bacak için (body.py ile aynı formül)."""
    radius = float(config.coxa_axis_radius.require())
    height = float(config.femur_joint_z_offset.require())
    forward = float(config.forward_offset_deg.require())
    out = {}
    for leg in config.legs:
        yaw = math.radians(forward - leg.azimuth_deg)
        c, s = math.cos(yaw), math.sin(yaw)
        out[leg.id] = rigid(((c, -s, 0), (s, c, 0), (0, 0, 1)),
                            (radius * c, radius * s, height))
    return out


# ---------------------------------------------------------------------------
# Çıktı
# ---------------------------------------------------------------------------


def _fmt(values, nd=2) -> str:
    return "[" + ", ".join(f"{v:.{nd}f}" for v in values) + "]"


def print_links_block(props, collisions, config) -> None:
    fill = config.raw["simulation"]["mass_inputs"]["print_fill_ratio"]["value"]
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
    for link in ("body", "coxa", "femur", "tibia"):
        mp = props[link]
        src = f"TAHMİN - CAD STL x PETG x doluluk {fill}; {notes[link]} (tools/cad_sim_model.py)"
        boxes = "[" + ", ".join(_fmt(b, 1) for b in collisions[link]) + "]"
        print(f"    {link}:")
        print(f"      mass_kg: {{value: {mp.m:.4f}, source: \"{src}\", measured: false}}")
        print(f"      com_mm: {{value: {_fmt(mp.com())}, source: \"{src}\", measured: false}}")
        print(f"      inertia_kg_mm2: {{value: {_fmt(mp.inertia_at_com(), 1)}, "
              f"source: \"{src}\", measured: false}}")
        print(f"      collision_boxes_mm: {{value: {boxes}, source: \"CAD STL + servo kutusu "
              f"sınırları, link çerçevesinde (tools/cad_sim_model.py)\", measured: false}}")


def write_meshes_yaml(meshes) -> None:
    lines = [
        "# BU DOSYA ÜRETİLDİ: tools/cad_sim_model.py. Elle düzenlemeyin.",
        "#",
        "# Görsel mesh'lerin link çerçevesindeki yerleşimi (URDF <visual><origin>).",
        "# xyz metre, rpy radyan (URDF sırası). Mesh'ler mm cinsinden, URDF'te",
        "# 0.001 ölçekle kullanılır. STL'ler git'te değil (boyut); pakete",
        "# `python tools/cad_sim_model.py --copy-meshes` ile kopyalanır.",
        "#",
        "# Aynalı bacaklar da normal bacağın mesh'lerini kullanır: URDF bir mesh'i",
        "# yansıtamaz. Yalnızca görünüş; kütle ve çarpışma hesabı aynalamayı",
        "# hesaba katar.",
        "",
    ]
    for link in ("body", "coxa", "femur", "tibia"):
        lines.append(f"{link}:")
        for stl, m in meshes[link]:
            xyz = [m[i][3] / 1000.0 for i in range(3)]
            rpy = rpy_of(m)
            lines.append(f"  - {{file: {stl.name}, xyz: {_fmt(xyz, 6)}, rpy: {_fmt(rpy, 6)}}}")
    MESHES_YAML.parent.mkdir(parents=True, exist_ok=True)
    MESHES_YAML.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def copy_meshes(meshes) -> None:
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    seen = set()
    for items in meshes.values():
        for stl, _ in items:
            if stl.name not in seen:
                shutil.copy2(stl, MESH_DIR / stl.name)
                seen.add(stl.name)
    print(f"{len(seen)} STL kopyalandı -> {MESH_DIR}")


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

    config = RobotConfig.load()
    props, collisions, meshes, foot_r, report = build(config)

    print("=" * 72)
    print("SİMÜLASYON MODELİ — CAD'den")
    print("=" * 72)
    for line in report:
        print("  " + line)
    total = props["body"].m + 6 * sum(props[k].m for k in ("coxa", "femur", "tibia"))
    print(f"  ayak ucu küre yarıçapı: {foot_r:.2f} mm")
    print(f"  toplam kütle tahmini: {total:.3f} kg")
    print()
    print("robot.yaml -> simulation altına:")
    print()
    print_links_block(props, collisions, config)
    print()

    write_meshes_yaml(meshes)
    print(f"mesh yerleşimi yazıldı -> {MESHES_YAML.relative_to(REPO_ROOT)}")
    if args.copy_meshes:
        copy_meshes(meshes)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
