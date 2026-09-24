"""CAD çerçeveleri ile robot çerçeveleri arasındaki dönüşümler.

  CAD bacak çerçevesi (leg-v2-v20.step): +Y yukarı, bacak -Z yönünde dışarı.
  IK bacak çerçevesi (hexapod_kinematics.leg): orijin coxa ekseni üzerinde,
    femur ekleminin yüksekliğinde; +x dışarı, +z yukarı.
    Dönüşüm: x = -Z_cad, y = -X_cad, z = +Y_cad.
  Tam montaj (hexapod-v8.step): +Y yukarı, bacak azimutu atan2(z, x).
  Gövde çerçevesi (REP-103): +x ileri, +y sol, +z yukarı;
    gövde_azimut = forward_offset - cad_azimut (robot.yaml frames.body).
"""

from __future__ import annotations

import math

from .assembly import Assembly
from .transform import Mat, apply, rigid


def joint_axes(leg: Assembly):
    """J1, J2, J3 eksenlerinin orta noktaları, CAD bacak çerçevesinde.

    Her eklemin dönme ekseni, servo horn'u ile karşısındaki bushing'i
    birleştiren doğru; ikisi de eksen üzerinde oturur. Eksenler yukarıdan
    aşağı (bacak boyunca dışarı) sıralı.
    """
    horns = sorted((o.origin for o in leg.matching("servo horn")), key=lambda p: -p[2])
    bushings = sorted((o.origin for o in leg.matching("bushing")), key=lambda p: -p[2])
    if len(horns) != 3 or len(bushings) != 3:
        raise ValueError(f"3 horn + 3 bushing bekleniyordu, {len(horns)} horn / "
                         f"{len(bushings)} bushing bulundu.")
    return [tuple((h[i] + b[i]) / 2 for i in range(3)) for h, b in zip(horns, bushings)]


def cad_leg_to_leg_frame(j1, j2) -> Mat:
    """CAD bacak çerçevesi -> IK bacak çerçevesi.  x=-Z, y=-X, z=+Y."""
    rot = ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    origin_cad = (0.0, j2[1], j1[2])  # coxa ekseni üstünde, femur ekleminin yüksekliğinde
    return rigid(rot, tuple(-sum(rot[i][k] * origin_cad[k] for k in range(3)) for i in range(3)))


def cad_to_body_frame(forward_offset_deg: float, ref_height: float) -> Mat:
    """hexapod-v8.step çerçevesi (+Y yukarı) -> REP-103 gövde çerçevesi.

    ref_height: bacak montaj referans düzleminin CAD'deki yüksekliği (Y);
    gövde çerçevesinin orijini bu yükseklikte.
    """
    f = math.radians(forward_offset_deg)
    c, s = math.cos(f), math.sin(f)
    rot = ((c, 0.0, s), (s, 0.0, -c), (0.0, 1.0, 0.0))
    return rigid(rot, (0.0, 0.0, -ref_height))


def leg_mounts(kinematics) -> dict[int, Mat]:
    """IK bacak çerçevesi -> gövde çerçevesi, her bacak için.

    kinematics: hexapod_kinematics.HexapodKinematics. Montajlar IK'dan
    alınır ki CAD hesabı ile IK aynı yerleşimi görsün.
    """
    out = {}
    for leg_id, m in kinematics.mounts.items():
        c, s = math.cos(m.yaw), math.sin(m.yaw)
        out[leg_id] = rigid(((c, -s, 0), (s, c, 0), (0, 0, 1)), (m.x, m.y, m.z))
    return out


def coxa_axis_in_body(full: Assembly, to_body: Mat, j1_offset: float):
    """Tam montajdaki her bacağın coxa ekseni, gövde çerçevesinde (x, y).

    Aynalı bacaklar Fusion'da ayrı gövde olarak dışa aktarılmış: dönüşüm
    matrisi düzgün bir dönme, yansıma GEOMETRİDE; yerel Z ters yönde uzanır.
    Bu yüzden eksen ofseti aynalılarda ters işaretli alınır.
    """
    out = []
    for occ in full.parts:
        if not occ.name.lower().startswith("leg"):
            continue
        mirrored = "mirror" in occ.name.lower()
        local = (0.0, 0.0, -j1_offset if mirrored else j1_offset)
        p = apply(to_body, apply(occ.matrix, local))
        out.append((occ.name, p, mirrored))
    return out
