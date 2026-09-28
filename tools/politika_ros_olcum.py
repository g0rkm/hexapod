#!/usr/bin/env python3
"""ROS'lu simde çalışan bir yürüyüş düğümünü komut komut sür, gövde hızını ölç.

Doğrudan değil, tools/wsl/politika_ros_olcum.sh içinden çalıştırılır (o betik
simülasyonu ve politika düğümünü başlatır). Gereken: ROS 2 Lyrical ortamı
(rclpy) ve gz.transport; simülasyonla aynı ROS_DOMAIN_ID ve GZ_PARTITION.

Yöntem (PROJE_DEVIR ders 23): robotun pozu gz poz yayınından
(/world/<dünya>/pose/info) sim zamanı damgasıyla alınır, her komutun
hareketinin orta %80'inde hız hesaplanır. Her adımın yer değiştirmesi o
anki gövde yönüne çevrilir: dönerken ilerleme komutunda da gövde
çerçevesindeki hız doğru çıkar. Sonuç Markdown tablosu.

Komutlar AYNI dünyada, robot sıfırlanmadan art arda koşulur. Düz zeminde
hız ölçümü için sorun değil; zeminde (çukur, basamak) her komutu ayrı bir
koşuda verin, yoksa ikinci komut ilkinin bıraktığı yerden başlar (ders 38:
çukurdan "yana çıkış" böyle yanlış ölçüldü). Yükseklik sütunu ortalama ve
başlangıç → bitiş z'sini yazar.

Sim mesafe sensörü (--mesafe CONFIG): ROS'lu simde mesafe sensörü yok; araç,
robotun gz pozundan ve zeminin yükseklik fonksiyonundan (--zemin TÜR:SEVİYE,
hexapod_terrain; verilmezse düz) her sensörün ölçümünü hesaplayıp
/range<kimlik>'e (sensor_msgs/Range) yayınlar: robottaki sürücü düğümünün
(hexapod_sensors) yerine. Yerleşim CONFIG'ten (politika düğümüne verilen
aynı dosya; hexapod_rl.deneysel_yerlesim). Işın ideal (hexapod_rl.rangefinder:
gürültü, koni ve ölçüm süresi yok), kontrol hızında yayınlanır.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

COMMANDS = [(0.10, 0.0, 0.0), (-0.10, 0.0, 0.0), (0.0, 0.06, 0.0), (0.0, -0.06, 0.0),
            (0.0, 0.0, 0.4), (0.0, 0.0, -0.4), (0.10, 0.04, 0.25), (0.0, 0.0, 0.0)]


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def body_velocity(samples) -> tuple[float, float, float]:
    """samples: (t, x, y, z, yaw) dizisi -> gövde çerçevesinde ortalama (vx, vy, wz)."""
    bx = by = dyaw = 0.0
    for p0, p1 in zip(samples, samples[1:]):
        c, s = math.cos(p0[4]), math.sin(p0[4])
        ex, ey = p1[1] - p0[1], p1[2] - p0[2]
        bx += c * ex + s * ey
        by += -s * ex + c * ey
        dyaw += math.atan2(math.sin(p1[4] - p0[4]), math.cos(p1[4] - p0[4]))
    dt = samples[-1][0] - samples[0][0]
    return bx / dt, by / dt, dyaw / dt


def _range_publisher(node, config_path, terrain, pose, lock):
    """--mesafe verildiyse: çağrıldıkça son pozdan sensör ölçümlerini hesaplayıp
    /range<kimlik>'e yayınlayan fonksiyon; verilmediyse None."""
    if not config_path:
        return None
    from sensor_msgs.msg import Range

    from hexapod_driver.config import RobotConfig
    from hexapod_policy.ranges import range_sensors_from_config
    from hexapod_rl.rangefinder import read

    placed = range_sensors_from_config(RobotConfig.load(config_path))
    height = lambda x, y: 0.0  # noqa: E731  düz zemin
    if terrain:
        from hexapod_terrain import sets
        kind, _, level = terrain.partition(":")
        height = sets.level(kind, int(level or 0)).height
    pubs = [node.create_publisher(Range, f"/range{i}", 10) for i, _ in placed]
    sensors = [s for _, s in placed]

    def publish() -> None:
        with lock:
            p = pose[0]
        if p is None:
            return
        for pub, s, d in zip(pubs, sensors, read(sensors, p[0], p[1], height)):
            msg = Range()
            msg.radiation_type = Range.INFRARED
            msg.min_range, msg.max_range, msg.range = 0.0, float(s.max_m), float(d)
            pub.publish(msg)

    return publish


def main(argv=None) -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from gz.msgs.pose_v_pb2 import Pose_V
    from gz.transport import Node as GzNode

    parser = argparse.ArgumentParser(description="ROS'lu simde yürüyüş ölçümü")
    parser.add_argument("--world", default="flat")
    parser.add_argument("--model", default="hexapod")
    parser.add_argument("--seconds", type=float, default=8.0, help="komut başına sim saniyesi")
    parser.add_argument("--settle", type=float, default=3.0, help="başta komutsuz bekleme")
    parser.add_argument("--commands", default=None,
                        help='komut listesi "vx,vy,wz;vx,vy,wz" (varsayılan: ileri, geri, '
                             'iki yana, iki dönüş, karışık, sıfır)')
    parser.add_argument("--mesafe", default=None, metavar="CONFIG",
                        help="sim mesafe sensörü: yerleşimi bu robot.yaml'dan, /range<kimlik> yayınla")
    parser.add_argument("--zemin", default=None, metavar="TÜR:SEVİYE",
                        help="--mesafe için zeminin yüksekliği (hexapod_terrain; verilmezse düz)")
    args = parser.parse_args(argv)
    commands = COMMANDS if not args.commands else [
        tuple(float(v) for v in c.split(",")) for c in args.commands.split(";")]

    lock = threading.Lock()
    samples: list[tuple[float, float, float, float, float]] = []
    pose: list = [None]   # son (konum, quat (w, x, y, z)); mesafe hesabı için

    def on_pose(msg) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == args.model:
                q = p.orientation
                with lock:
                    samples.append((t, p.position.x, p.position.y, p.position.z,
                                    _yaw(q)))
                    pose[0] = ((p.position.x, p.position.y, p.position.z), (q.w, q.x, q.y, q.z))
                return

    gz = GzNode()
    if not gz.subscribe(Pose_V, f"/world/{args.world}/pose/info", on_pose):
        print("HATA: poz yayınına abone olunamadı", file=sys.stderr)
        return 1
    rclpy.init()
    node = rclpy.create_node("politika_ros_olcum")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    publish_ranges = _range_publisher(node, args.mesafe, args.zemin, pose, lock)
    period = 0.02 if publish_ranges else 0.05   # mesafe kontrol hızında (refleks >= 25 Hz ister)

    def now():
        with lock:
            return samples[-1][0] if samples else None

    t0 = time.time()
    while now() is None and time.time() - t0 < 30:
        time.sleep(0.1)
    if now() is None:
        print("HATA: poz yayını yok (GZ_PARTITION aynı mı?)", file=sys.stderr)
        return 1
    start = now()
    while now() - start < args.settle:
        time.sleep(0.05)

    print("| Komut (vx, vy, wz) | gövde vx | gövde vy | dönüş rad/s | izleme | yol (tüm süre) | yükseklik |")
    print("|---|---|---|---|---|---|---|")
    for cmd in commands:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = cmd
        s0 = now()
        while now() - s0 < args.seconds:
            pub.publish(msg)
            if publish_ranges:
                publish_ranges()
            time.sleep(period)
        with lock:
            seg = [s for s in samples if s0 <= s[0] <= s0 + args.seconds]
        mid = seg[int(0.1 * len(seg)):int(0.9 * len(seg))]
        vx, vy, wz = body_velocity(mid)
        ratios = [m / c for m, c in zip((vx, vy, wz), cmd) if abs(c) > 1e-9]
        moved = math.hypot(mid[-1][1] - mid[0][1], mid[-1][2] - mid[0][2])
        track = (" / ".join(f"%{100 * r:.0f}" for r in ratios) if ratios
                 else f"hareket {1000 * moved:.1f} mm")
        height = sum(s[3] for s in seg) / len(seg)
        path = math.hypot(seg[-1][1] - seg[0][1], seg[-1][2] - seg[0][2])
        print(f"| {cmd} | {vx:+.3f} | {vy:+.3f} | {wz:+.3f} | {track} | {path:.2f} m | "
              f"{1000 * height:.0f} mm (z {1000 * seg[0][3]:.0f} → {1000 * seg[-1][3]:.0f}) |",
              flush=True)
    pub.publish(Twist())
    sys.stdout.flush()
    # gz.transport + rclpy birlikte kapanırken çöküyor (segfault); ölçüm bitti,
    # temizliği işletim sistemine bırak.
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
