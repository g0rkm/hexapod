"""hexapod_camera: tarayıcıda canlı görüntü.

ROS'suz kısım (tampon, web sunucusu, deneme deseni) her yerde; düğüm ve
başlatma dosyası gerçek süreç olarak (Linux, rclpy + ros2). Gerçek kamera
(camera_ros + Pi Camera V2) burada yok: kamerayı taklit eden taraf ya testin
yayınladığı JPEG kareler ya da deneme deseni.
"""

from __future__ import annotations

import http.client
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from hexapod_camera.mjpeg import BOUNDARY, FrameBuffer, MjpegServer, local_urls

# Gerçek JPEG olması gerekmez: sunucu kareyi çözmeden iletir.
FAKE = [b"\xff\xd8kare-%d\xff\xd9" % i for i in range(10)]


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def get(port: int, path: str, timeout: float = 5.0) -> tuple[int, str, bytes]:
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=timeout)
    try:
        conn.request("GET", path)
        r = conn.getresponse()
        return r.status, r.getheader("Content-Type", ""), r.read()
    finally:
        conn.close()


def read_part(resp) -> bytes:
    """multipart/x-mixed-replace akışından bir kare oku."""
    line = resp.readline()
    while line.strip() != f"--{BOUNDARY}".encode():
        if not line:
            raise EOFError("akış kapandı")
        line = resp.readline()
    headers = {}
    while True:
        line = resp.readline().strip()
        if not line:
            break
        k, v = line.decode().split(":", 1)
        headers[k.strip().lower()] = v.strip()
    assert headers["content-type"] == "image/jpeg"
    data = resp.read(int(headers["content-length"]))
    assert resp.readline() == b"\r\n"
    return data


# ---------------------------------------------------------------------------
# Tampon
# ---------------------------------------------------------------------------


def test_tampon_son_kareyi_tutar_yenisini_bekler():
    b = FrameBuffer()
    assert b.latest() == (0, None) and b.age() is None and b.fps() == 0.0
    assert b.wait_newer(0, timeout=0.05) is None              # kare yok: zaman aşımı
    b.put(FAKE[0])
    b.put(FAKE[1])
    assert b.latest() == (2, FAKE[1]) and b.count == 2        # yalnız EN SON kare
    assert b.wait_newer(1, timeout=0.05) == (2, FAKE[1])
    assert b.wait_newer(2, timeout=0.05) is None

    got = []
    t = threading.Thread(target=lambda: got.append(b.wait_newer(2, timeout=5.0)))
    t.start()
    time.sleep(0.05)
    b.put(FAKE[2])
    t.join(2)
    assert got == [(3, FAKE[2])]


def test_tampon_kapaninca_bekleyen_birakilir():
    b = FrameBuffer()
    got = []
    t = threading.Thread(target=lambda: got.append(b.wait_newer(0, timeout=10.0)))
    t.start()
    time.sleep(0.05)
    b.close()
    t.join(2)
    assert not t.is_alive() and got == [None]


def test_kare_hizi():
    b = FrameBuffer()
    for f in FAKE[:6]:
        b.put(f)
        time.sleep(0.02)
    assert 20 < b.fps() < 60


# ---------------------------------------------------------------------------
# Web sunucusu
# ---------------------------------------------------------------------------


@pytest.fixture
def server():
    s = MjpegServer(FrameBuffer(), "127.0.0.1", 0)
    s.start()
    yield s
    s.stop()


def test_sayfa_tek_kare_durum_ve_404(server):
    status, ctype, body = get(server.port, "/")
    assert status == 200 and ctype.startswith("text/html")
    page = body.decode("utf-8")
    assert "Hexapod kamera" in page and 'src="/yayin"' in page

    status, _, body = get(server.port, "/kare.jpg")
    assert status == 503 and "henüz görüntü yok" in body.decode("utf-8")
    status, _, body = get(server.port, "/durum")
    d = json.loads(body)
    assert status == 200 and d["kare_sayisi"] == 0 and d["son_kare_s"] is None

    server.buffer.put(FAKE[3])
    status, ctype, body = get(server.port, "/kare.jpg")
    assert status == 200 and ctype == "image/jpeg" and body == FAKE[3]
    d = json.loads(get(server.port, "/durum")[2])
    assert d["kare_sayisi"] == 1 and 0 <= d["son_kare_s"] < 1.0

    assert get(server.port, "/yok")[0] == 404


def test_akis_son_kareyle_baslar_yenileri_sirayla_gonderir(server):
    server.buffer.put(FAKE[0])
    conn = http.client.HTTPConnection("127.0.0.1", server.port, timeout=5)
    resp = None
    try:
        conn.request("GET", "/yayin")
        resp = conn.getresponse()
        assert resp.status == 200
        assert resp.getheader("Content-Type") == f"multipart/x-mixed-replace; boundary={BOUNDARY}"
        assert read_part(resp) == FAKE[0]                     # bağlanınca hemen son kare
        deadline = time.monotonic() + 3
        while server.viewers != 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert server.viewers == 1
        for f in FAKE[1:4]:
            server.buffer.put(f)
            assert read_part(resp) == f
    finally:
        if resp is not None:
            resp.close()      # yanıt açıkken conn.close() soketi gerçekten kapatmaz
        conn.close()
    deadline = time.monotonic() + 5
    while server.viewers != 0 and time.monotonic() < deadline:
        server.buffer.put(FAKE[5])                            # kopan izleyici yazarken fark edilir
        time.sleep(0.05)
    assert server.viewers == 0


def test_iki_izleyici_ayni_anda(server):
    conns = [http.client.HTTPConnection("127.0.0.1", server.port, timeout=5) for _ in range(2)]
    try:
        resps = []
        for c in conns:
            c.request("GET", "/yayin")
            resps.append(c.getresponse())
        time.sleep(0.1)
        server.buffer.put(FAKE[7])
        assert [read_part(r) for r in resps] == [FAKE[7], FAKE[7]]
    finally:
        for r in resps:
            r.close()
        for c in conns:
            c.close()


def test_durdurunca_akis_kapanir():
    s = MjpegServer(FrameBuffer(), "127.0.0.1", 0)
    s.start()
    conn = http.client.HTTPConnection("127.0.0.1", s.port, timeout=5)
    conn.request("GET", "/yayin")
    resp = conn.getresponse()
    t0 = time.monotonic()
    s.stop()
    assert time.monotonic() - t0 < 5.0
    with pytest.raises((EOFError, OSError, http.client.HTTPException)):
        read_part(resp)
    resp.close()
    conn.close()


def test_adresler():
    urls = local_urls(8080)
    assert urls and all(u.startswith("http://") and u.endswith(":8080") for u in urls)
    assert not any("127." in u for u in urls)


def test_deneme_deseni_gecerli_ve_degisen_jpeg():
    pytest.importorskip("PIL", reason="Pillow yok")
    from PIL import Image
    import io

    from hexapod_camera.pattern import TestPattern

    p = TestPattern(320, 240)
    a, b = p.frame(), p.frame()
    assert a[:2] == b"\xff\xd8" and a[-2:] == b"\xff\xd9"
    assert a != b                                             # kare sayısı değişiyor
    assert Image.open(io.BytesIO(a)).size == (320, 240)
    with pytest.raises(ValueError):
        TestPattern(10, 10)


# ---------------------------------------------------------------------------
# Düğüm ve başlatma dosyası: gerçek süreç (Linux + rclpy)
# ---------------------------------------------------------------------------

SRC = Path(__file__).resolve().parent.parent / "src"
LAUNCH = SRC / "hexapod_camera" / "launch" / "kamera.launch.py"
STARTUP_S = 30.0


@pytest.fixture(scope="module")
def ros():
    if sys.platform == "win32":
        pytest.skip("ROS düğüm testleri Linux'ta")
    os.environ["ROS_DOMAIN_ID"] = "89"
    rclpy = pytest.importorskip("rclpy", reason="rclpy yok (ROS 2 Lyrical kaynaklı ortam gerekir)")
    rclpy.init()
    node = rclpy.create_node("test_camera")
    yield rclpy, node
    node.destroy_node()
    rclpy.try_shutdown()


def _env() -> dict:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [str(SRC / "hexapod_camera"), str(SRC / "hexapod_driver"), env.get("PYTHONPATH", "")])
    env["ROS_DOMAIN_ID"] = "89"
    return env


def start_node(*params: str) -> subprocess.Popen:
    args = []
    for p in params:
        args += ["-p", p]
    return subprocess.Popen([sys.executable, "-m", "hexapod_camera.node", "--ros-args", *args],
                            env=_env(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def wait_http(port: int, path: str, ok, timeout_s: float, each=None) -> tuple[int, bytes]:
    end = time.monotonic() + timeout_s
    last = (0, b"")
    while time.monotonic() < end:
        if each is not None:
            each()
        try:
            status, _, body = get(port, path, timeout=2.0)
            last = (status, body)
            if ok(status, body):
                return last
        except OSError:
            pass
        time.sleep(0.1)
    return last


def test_dugum_kameranin_jpeg_karelerini_tarayiciya_verir(ros):
    from sensor_msgs.msg import CompressedImage

    rclpy, node = ros
    port = free_port()
    topic = "/test_kamera/compressed"
    proc = start_node(f"port:={port}", f"topic:={topic}")
    pub = node.create_publisher(CompressedImage, topic, 10)
    try:
        def send():
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = FAKE[4]
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)

        status, body = wait_http(port, "/kare.jpg", lambda s, b: s == 200, STARTUP_S, send)
        assert (status, body) == (200, FAKE[4]), proc.poll()
    finally:
        proc.send_signal(signal.SIGTERM)
        out, err = proc.communicate(timeout=15)
    assert proc.returncode == 0, err
    assert "Traceback" not in err
    assert f":{port}" in out + err                             # adres günlükte


def test_dugum_deneme_deseni(ros):
    pytest.importorskip("PIL", reason="Pillow yok")
    port = free_port()
    proc = start_node(f"port:={port}", "test_pattern:=true", "width:=320", "height:=240")
    try:
        status, body = wait_http(port, "/kare.jpg", lambda s, b: s == 200, STARTUP_S)
        assert status == 200 and body[:2] == b"\xff\xd8", proc.poll()
    finally:
        proc.send_signal(signal.SIGTERM)
        proc.communicate(timeout=15)
    assert proc.returncode == 0


def test_dugum_port_doluysa_aciklayip_cikar(ros):
    with socket.socket() as busy:
        busy.bind(("0.0.0.0", 0))
        busy.listen()
        port = busy.getsockname()[1]
        proc = start_node(f"port:={port}")
        out, err = proc.communicate(timeout=STARTUP_S)
    assert proc.returncode == 2
    assert "port" in err and "Traceback" not in err


def test_kamera_launch_deneme_modunda_tarayiciya_yayin_yapar(ros):
    pytest.importorskip("PIL", reason="Pillow yok")
    if shutil.which("ros2") is None:
        pytest.skip("ros2 komutu yok")
    port = free_port()
    proc = subprocess.Popen(["ros2", "launch", str(LAUNCH), "deneme:=true", f"port:={port}"],
                            env={**os.environ, "ROS_DOMAIN_ID": "89"}, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        status, body = wait_http(port, "/kare.jpg", lambda s, b: s == 200, STARTUP_S)
        assert status == 200 and body[:2] == b"\xff\xd8"
        assert "Hexapod kamera" in get(port, "/")[2].decode("utf-8")   # sayfa açılıyor
    finally:
        os.killpg(proc.pid, signal.SIGINT)                    # terminalde Ctrl+C gibi
        try:
            out, _ = proc.communicate(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            out, _ = proc.communicate()
            pytest.fail(f"kamera launch SIGINT'e kapanmadı:\n{out[-2000:]}")
    assert "Traceback" not in out, out[-2000:]
    assert "hexapod_camera hazır: kaynak deneme deseni" in out, out[-2000:]
    assert "camera_node" not in out                           # deneme: kamera sürücüsü açılmaz
    assert "process has died" not in out, out[-2000:]
