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
    args = parser.parse_args(argv)

    lock = threading.Lock()
    samples: list[tuple[float, float, float, float, float]] = []

    def on_pose(msg) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == args.model:
                with lock:
                    samples.append((t, p.position.x, p.position.y, p.position.z,
                                    _yaw(p.orientation)))
                return

    gz = GzNode()
    if not gz.subscribe(Pose_V, f"/world/{args.world}/pose/info", on_pose):
        print("HATA: poz yayınına abone olunamadı", file=sys.stderr)
        return 1
    rclpy.init()
    node = rclpy.create_node("politika_ros_olcum")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

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

    print("| Komut (vx, vy, wz) | gövde vx | gövde vy | dönüş rad/s | izleme | yükseklik |")
    print("|---|---|---|---|---|---|")
    for cmd in COMMANDS:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = cmd
        s0 = now()
        while now() - s0 < args.seconds:
            pub.publish(msg)
            time.sleep(0.05)
        with lock:
            seg = [s for s in samples if s0 <= s[0] <= s0 + args.seconds]
        mid = seg[int(0.1 * len(seg)):int(0.9 * len(seg))]
        vx, vy, wz = body_velocity(mid)
        ratios = [m / c for m, c in zip((vx, vy, wz), cmd) if abs(c) > 1e-9]
        moved = math.hypot(mid[-1][1] - mid[0][1], mid[-1][2] - mid[0][2])
        track = (" / ".join(f"%{100 * r:.0f}" for r in ratios) if ratios
                 else f"hareket {1000 * moved:.1f} mm")
        height = sum(s[3] for s in seg) / len(seg)
        print(f"| {cmd} | {vx:+.3f} | {vy:+.3f} | {wz:+.3f} | {track} | "
              f"{1000 * height:.0f} mm |", flush=True)
    pub.publish(Twist())
    sys.stdout.flush()
    # gz.transport + rclpy birlikte kapanırken çöküyor (segfault); ölçüm bitti,
    # temizliği işletim sistemine bırak.
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
