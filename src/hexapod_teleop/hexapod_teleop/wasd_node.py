"""WASD kumanda düğümü: klavyeden /cmd_vel (geometry_msgs/Twist) yayınlar.

    ros2 run hexapod_teleop wasd
    ros2 run hexapod_teleop wasd --ros-args -p rate_hz:=10.0

Tuşlar ve davranış: wasd.py (W ileri, S geri, A sola dön, D sağa dön, K dur,
Q/E hız). Bir TERMİNALDE çalışır, tuşları oradan okur: robotun Pi'sinde (SSH
ile bağlanıp) ya da aynı ağdaki ROS kurulu bir bilgisayarda. /cmd_vel'i hem
politika düğümü (hexapod_policy) hem tripod düğümü (hexapod_teleop teleop)
dinler; simülasyonda da robotta da aynı.

Komut rate_hz ile SÜREKLİ yayınlanır (dururken sıfır). Politika düğümü 0.5 s
komut alamazsa durur: kumanda kapanır, çöker ya da Wi-Fi koparsa robot
kendiliğinden durur. Ctrl+C'de önce dur komutu gönderilir, terminal eski
hâline döner.

Linux'a özgü (termios); Windows'ta çekirdek (wasd.py) test edilir.
"""

from __future__ import annotations

import os
import sys


def _run(argv: list[str] | None = None) -> int:
    if not sys.stdin.isatty():
        print("HATA: wasd bir terminalde çalışmalı (tuşları terminalden okur). "
              "Terminalde çalıştırın: ros2 run hexapod_teleop wasd", file=sys.stderr)
        return 2

    import select
    import termios
    import time
    import tty

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.signals import SignalHandlerOptions

    from .wasd import HELP, WasdTeleop

    # rclpy'nin sinyal işleyicisi kapalı: Ctrl+C / SIGTERM'i main (run_node)
    # karşılar, finally'deki dur komutu ve terminal onarımı yarıda kalmaz.
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("hexapod_wasd")
    node.declare_parameter("rate_hz", 10.0)
    rate = float(node.get_parameter("rate_hz").value)
    if not rate > 2.0:
        # 0.5 s zaman aşımında en az bir komut gitmeli; yoksa robot kesik kesik yürür
        print(f"HATA: rate_hz 2'den büyük olmalı: {rate}", file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
        return 2
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    teleop = WasdTeleop()

    def send(command) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = (float(v) for v in command)
        pub.publish(msg)

    def show() -> None:
        # \r + satır sonuna kadar sil: durum satırı yerinde güncellenir
        sys.stdout.write("\r" + teleop.status() + "\033[K")
        sys.stdout.flush()

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        # cbreak: tuşlar Enter beklemeden gelir, ekrana yazılmaz; Ctrl+C yine sinyal
        tty.setcbreak(fd)
        print(HELP)
        show()
        next_publish = 0.0
        while True:
            ready, _, _ = select.select([fd], [], [], 0.02)
            if ready:
                data = os.read(fd, 32)
                if not data:
                    break                       # terminal kapandı
                changed = False
                for ch in data.decode("utf-8", errors="ignore"):
                    changed |= teleop.press(ch)
                if changed:
                    show()
                    next_publish = 0.0          # yeni komut hemen gitsin
            now = time.monotonic()
            if now >= next_publish:
                send(teleop.command())
                next_publish = now + 1.0 / rate
    except KeyboardInterrupt:
        pass
    finally:
        teleop.stop()
        for _ in range(3):                      # dur komutu (keşif gecikmesine karşı birkaç kez)
            send(teleop.command())
            time.sleep(0.05)
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        sys.stdout.write("\n")
        sys.stdout.flush()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def main(argv: list[str] | None = None) -> int:
    from hexapod_driver.stop_signals import run_node
    return run_node(_run, argv)


if __name__ == "__main__":
    raise SystemExit(main())
