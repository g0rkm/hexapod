"""hexapod_camera — Pi kamerasından tarayıcıda canlı görüntü (2026-09-29).

mjpeg.py    ROS'suz: son kareyi tutan tampon + MJPEG web sunucusu (sayfa,
            akış, tek kare, durum).
pattern.py  kamerasız deneme deseni (Pillow).
node.py     ince rclpy kabuğu: /camera/image_raw/compressed -> web sunucusu.

Kullanım: ros2 launch hexapod_camera kamera.launch.py [deneme:=true]
          sonra tarayıcıda http://<robotun IP'si>:8080
Kamera yalnız izlemek için; robotun yürüyüşü görüntüyü kullanmaz.
"""

from .mjpeg import FrameBuffer, MjpegServer, local_urls

__all__ = ["FrameBuffer", "MjpegServer", "local_urls"]

__version__ = "0.1.0"
