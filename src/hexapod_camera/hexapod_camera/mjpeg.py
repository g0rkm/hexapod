"""Tarayıcıda canlı görüntü: MJPEG yayını yapan küçük web sunucusu, ROS'suz.

Kamera JPEG kareleri üretir (camera_ros, /camera/image_raw/compressed); bu
sunucu son kareyi tutar ve aynı ağdaki her tarayıcıya "multipart/x-mixed-replace"
ile akıtır. İzlemek için ROS ya da program gerekmez: telefonda/bilgisayarda
http://<robotun IP'si>:8080 açılır.

Adresler:
  /          sayfa: canlı görüntü + durum satırı (kare/s, "görüntü gelmiyor")
  /yayin     MJPEG akışı (<img src="/yayin">)
  /kare.jpg  son kare (tek resim); henüz kare yoksa 503
  /durum     JSON: kare sayısı, son karenin yaşı, kare/s, izleyici sayısı

Tasarım:
  - Kareler çözülmez, olduğu gibi iletilir (Pi'de işlemci harcamaz).
  - Her izleyici hep EN SON kareyi alır; yavaş bağlantı (zayıf Wi-Fi) kare
    atlar, gecikme birikmez.
  - Sunucu yalnız okur; robota komut gönderemez. Aynı ağdaki herkes
    görüntüyü izleyebilir (kulüp ağı için yeterli; dışarı açmayın).
"""

from __future__ import annotations

import json
import socket
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BOUNDARY = "kare"
#: Bu kadar süredir kare gelmiyorsa sayfa "görüntü gelmiyor" der.
STALE_S = 2.0


class FrameBuffer:
    """Son JPEG kare; izleyiciler yenisini bekler (iş parçacığı güvenli)."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._frame: bytes | None = None
        self._seq = 0
        self._stamp: float | None = None
        self._times: deque[float] = deque(maxlen=30)
        self._closed = False

    def put(self, jpeg: bytes) -> None:
        now = time.monotonic()
        with self._cond:
            self._frame, self._stamp = bytes(jpeg), now
            self._seq += 1
            self._times.append(now)
            self._cond.notify_all()

    def latest(self) -> tuple[int, bytes | None]:
        with self._cond:
            return self._seq, self._frame

    def wait_newer(self, seq: int, timeout: float) -> tuple[int, bytes] | None:
        """seq'ten yeni bir kare gelene kadar bekle; zaman aşımında ya da
        kapanınca None."""
        with self._cond:
            self._cond.wait_for(lambda: self._seq > seq or self._closed, timeout)
            if self._closed or self._seq <= seq or self._frame is None:
                return None
            return self._seq, self._frame

    def age(self) -> float | None:
        """Son karenin yaşı (s); hiç kare yoksa None."""
        with self._cond:
            return None if self._stamp is None else time.monotonic() - self._stamp

    def fps(self) -> float:
        """Son ~30 karenin hızı; kareler bayatsa 0."""
        with self._cond:
            if len(self._times) < 2 or time.monotonic() - self._times[-1] > STALE_S:
                return 0.0
            span = self._times[-1] - self._times[0]
            return (len(self._times) - 1) / span if span > 0 else 0.0

    @property
    def count(self) -> int:
        with self._cond:
            return self._seq

    @property
    def closed(self) -> bool:
        return self._closed

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()


PAGE = """<!doctype html>
<html lang="tr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Hexapod kamera</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; background: #111; color: #eee; font: 15px/1.4 system-ui, sans-serif; }
  header { padding: 10px 14px; display: flex; gap: 12px; align-items: baseline; flex-wrap: wrap; }
  h1 { font-size: 17px; margin: 0; }
  #durum { color: #9ad; }
  #durum.yok { color: #f86; }
  img { display: block; width: 100%%; max-width: 1280px; margin: 0 auto; background: #000; }
</style></head>
<body>
<header><h1>Hexapod kamera</h1><span id="durum">bağlanıyor…</span></header>
<img src="/yayin" alt="canlı görüntü">
<script>
const d = document.getElementById("durum");
async function yenile() {
  try {
    const r = await (await fetch("/durum", {cache: "no-store"})).json();
    if (r.son_kare_s === null) { d.textContent = "görüntü yok: kamera henüz kare göndermedi"; d.className = "yok"; }
    else if (r.son_kare_s > %(stale)s) { d.textContent = "görüntü gelmiyor (" + r.son_kare_s.toFixed(0) + " s): kamera bağlı mı?"; d.className = "yok"; }
    else { d.textContent = "canlı · " + r.kare_s.toFixed(0) + " kare/s · " + r.izleyici + " izleyici"; d.className = ""; }
  } catch (e) { d.textContent = "robota ulaşılamıyor"; d.className = "yok"; }
}
setInterval(yenile, 1000); yenile();
</script>
</body></html>
""" % {"stale": STALE_S}


class MjpegServer:
    """FrameBuffer'daki kareleri HTTP ile yayınlar.

    Kullanım:
        server = MjpegServer(buffer, port=8080)
        server.start()            # arka planda
        ...
        server.stop()
    """

    def __init__(self, buffer: FrameBuffer, host: str = "0.0.0.0", port: int = 8080,
                 log=None) -> None:
        self.buffer = buffer
        self._viewers = 0
        self._lock = threading.Lock()
        self._log = log or (lambda text: None)
        handler = _make_handler(self)
        self._httpd = ThreadingHTTPServer((host, port), handler)
        self._httpd.daemon_threads = True
        self._thread: threading.Thread | None = None

    @property
    def port(self) -> int:
        """Gerçek port (port=0 verilince işletim sisteminin seçtiği)."""
        return self._httpd.server_address[1]

    @property
    def viewers(self) -> int:
        with self._lock:
            return self._viewers

    def start(self) -> None:
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        name="mjpeg", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.buffer.close()              # akışlar beklemeyi bıraksın
        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=5)

    def _viewer(self, delta: int) -> None:
        with self._lock:
            self._viewers += delta


def _make_handler(server: MjpegServer):
    buffer = server.buffer

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):   # her isteği terminale basma
            pass

        def do_GET(self):  # noqa: N802 (http.server adı)
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self._send(HTTPStatus.OK, "text/html; charset=utf-8", PAGE.encode("utf-8"))
            elif path == "/kare.jpg":
                _, frame = buffer.latest()
                if frame is None:
                    self._send(HTTPStatus.SERVICE_UNAVAILABLE, "text/plain; charset=utf-8",
                               "henüz görüntü yok: kamera kare göndermedi\n".encode("utf-8"))
                else:
                    self._send(HTTPStatus.OK, "image/jpeg", frame)
            elif path == "/durum":
                age = buffer.age()
                body = json.dumps({"kare_sayisi": buffer.count,
                                   "son_kare_s": None if age is None else round(age, 3),
                                   "kare_s": round(buffer.fps(), 2),
                                   "izleyici": server.viewers}).encode("utf-8")
                self._send(HTTPStatus.OK, "application/json", body)
            elif path == "/yayin":
                self._stream()
            else:
                self._send(HTTPStatus.NOT_FOUND, "text/plain; charset=utf-8", b"yok\n")

        def _send(self, status, ctype: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _stream(self) -> None:
            self.close_connection = True
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            server._viewer(+1)
            server._log(f"izleyici bağlandı: {self.client_address[0]}")
            seq = 0
            try:
                seq, frame = buffer.latest()
                if frame is not None:
                    self._part(frame)
                while not buffer.closed:
                    got = buffer.wait_newer(seq, timeout=1.0)
                    if got is None:
                        continue
                    seq, frame = got
                    self._part(frame)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                pass                              # izleyici sayfayı kapattı
            finally:
                server._viewer(-1)
                server._log(f"izleyici ayrıldı: {self.client_address[0]}")

        def _part(self, frame: bytes) -> None:
            self.wfile.write(
                f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
            self.wfile.write(frame)
            self.wfile.write(b"\r\n")
            self.wfile.flush()

    return Handler


def local_urls(port: int) -> list[str]:
    """Bu makineye aynı ağdan ulaşılacak adres(ler): http://IP:port.

    Varsayılan yolun çıktığı arayüzün IP'si (bağlantı kurulmaz, UDP soketi
    yalnız yönlendirme tablosuna sorar) + makine adı (.local; ağda mDNS varsa
    çalışır)."""
    urls = []
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))
            ip = s.getsockname()[0]
        if not ip.startswith("127."):
            urls.append(f"http://{ip}:{port}")
    except OSError:
        pass
    urls.append(f"http://{socket.gethostname()}.local:{port}")
    return urls


__all__ = ["BOUNDARY", "FrameBuffer", "MjpegServer", "PAGE", "local_urls"]
