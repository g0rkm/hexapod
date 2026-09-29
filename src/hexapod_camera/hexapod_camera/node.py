"""Kamera yayın düğümü: kameranın JPEG karelerini tarayıcıya canlı yayınlar.

    ros2 run hexapod_camera stream                              # camera_ros'u dinler
    ros2 run hexapod_camera stream --ros-args -p test_pattern:=true   # kamerasız deneme
    ros2 run hexapod_camera stream --ros-args -p port:=8081

Genelde tek başına değil, kamera sürücüsüyle birlikte açılır:
`ros2 launch hexapod_camera kamera.launch.py` (ya da robot.launch.py camera:=true).

Dinlediği: topic (varsayılan /camera/image_raw/compressed, sensor_msgs/
CompressedImage; camera_ros'un JPEG yayını). Yayınladığı: yok. Açtığı: HTTP
port (varsayılan 8080), sayfa ve MJPEG akışı (mjpeg.py). Adres günlüğe yazılır.

Kamera yalnız izlemek için (2026-09-29 kararı): politika görüntüyü kullanmaz,
bu düğüm çökse de robot yürümeye devam eder (robot.launch.py onu "biri
çıkarsa hepsi kapansın" kuralının dışında tutar).
"""

from __future__ import annotations

import sys


def _run(argv: list[str] | None = None) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import CompressedImage

    from .mjpeg import FrameBuffer, MjpegServer, local_urls

    class StartupError(Exception):
        pass

    class CameraStreamNode(Node):
        def __init__(self) -> None:
            super().__init__("hexapod_camera")
            self.declare_parameter("topic", "/camera/image_raw/compressed")
            self.declare_parameter("port", 8080)
            self.declare_parameter("host", "0.0.0.0")
            self.declare_parameter("test_pattern", False)
            self.declare_parameter("pattern_fps", 15.0)
            self.declare_parameter("width", 640)     # yalnız deneme deseni
            self.declare_parameter("height", 480)
            topic = str(self.get_parameter("topic").value)
            port = int(self.get_parameter("port").value)
            host = str(self.get_parameter("host").value)
            test_pattern = bool(self.get_parameter("test_pattern").value)

            self.buffer = FrameBuffer()
            try:
                self.server = MjpegServer(self.buffer, host, port,
                                          log=lambda t: self.get_logger().info(t))
            except OSError as exc:
                raise StartupError(f"{port} numaralı port açılamadı ({exc}); başka bir "
                                   f"program kullanıyor olabilir: -p port:=8081 deneyin") from exc
            self.server.start()
            self._warned_format = False
            self._source = "deneme deseni" if test_pattern else topic

            if test_pattern:
                from .pattern import TestPattern
                try:
                    self._pattern = TestPattern(int(self.get_parameter("width").value),
                                                int(self.get_parameter("height").value))
                except (RuntimeError, ValueError) as exc:
                    self.server.stop()
                    raise StartupError(str(exc)) from exc
                fps = float(self.get_parameter("pattern_fps").value)
                self.create_timer(1.0 / fps, lambda: self.buffer.put(self._pattern.frame()))
            else:
                # sensor_data (en iyi çaba): yayıncı güvenilir de olsa en iyi çaba da
                # olsa uyuşur; kaybolan kare beklenmez, canlı görüntüde doğrusu bu.
                self.create_subscription(CompressedImage, topic, self._on_image,
                                         qos_profile_sensor_data)
                self.create_timer(5.0, self._check_stale)

            urls = "  ".join(local_urls(self.server.port))
            self.get_logger().info(
                f"hexapod_camera hazır: kaynak {self._source}. Aynı Wi-Fi'deki tarayıcıda "
                f"açın: {urls}")

        def _on_image(self, msg) -> None:
            fmt = (msg.format or "").lower()
            if fmt and "jpeg" not in fmt and "jpg" not in fmt:
                if not self._warned_format:
                    self.get_logger().error(
                        f"kare biçimi '{msg.format}': tarayıcı yalnız JPEG gösterir")
                    self._warned_format = True
                return
            self.buffer.put(bytes(msg.data))

        def _check_stale(self) -> None:
            age = self.buffer.age()
            if age is None or age > 5.0:
                self.get_logger().warning(
                    f"kameradan görüntü gelmiyor ({self._source}). Kamera sürücüsü "
                    f"(camera_ros) çalışıyor mu? Deneme: ros2 topic hz {self._source}",
                    throttle_duration_sec=30.0)

        def shutdown(self) -> None:
            self.server.stop()

    # rclpy'nin sinyal işleyicisi kapalı: Ctrl+C / SIGTERM'i main (run_node)
    # karşılar; ikinci sinyal kapanışı yarıda kesmesin (hexapod_driver.stop_signals).
    try:
        rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
        node = CameraStreamNode()
    except StartupError as exc:
        print(f"HATA: kamera yayın düğümü başlatılamadı: {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        return 2
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # SIGTERM bağlamı kapatınca spin ExternalShutdownException / RCLError ile çıkar
        if rclpy.ok():
            raise
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def main(argv: list[str] | None = None) -> int:
    from hexapod_driver.stop_signals import run_node
    return run_node(_run, argv)


if __name__ == "__main__":
    raise SystemExit(main())
