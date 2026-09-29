"""Deneme deseni: kamera olmadan hareketli JPEG kareler (Pillow ile).

Ne için: kamera takılmadan ya da çalışmadan önce görüntü yolunun geri kalanını
(düğüm -> Wi-Fi -> tarayıcı) denemek. Desende saat ve kare sayısı döner,
kayan bir çubuk var: görüntü donuyorsa ya da geç geliyorsa gözle görülür.

    ros2 launch hexapod_camera kamera.launch.py deneme:=true
"""

from __future__ import annotations

import io
import time


class TestPattern:
    """frame() her çağrıda yeni bir JPEG (bytes) döndürür."""

    __test__ = False     # pytest bunu test sınıfı sanmasın

    def __init__(self, width: int = 640, height: int = 480, quality: int = 80) -> None:
        try:
            from PIL import Image, ImageDraw, ImageFont
        except ImportError as exc:   # pragma: no cover - Pi'de python3-pil kurulu
            raise RuntimeError("deneme deseni için Pillow gerekir: sudo apt install python3-pil "
                               "(tools/pi/pi_kurulum.sh kurar)") from exc
        if width < 64 or height < 48:
            raise ValueError(f"desen en az 64x48 olmalı: {width}x{height}")
        self._Image, self._Draw = Image, ImageDraw
        self.width, self.height, self.quality = width, height, quality
        size = max(12, height // 14)
        try:
            self._font = ImageFont.load_default(size=size)
        except TypeError:            # eski Pillow: boyutsuz varsayılan yazı tipi
            self._font = ImageFont.load_default()
        self._n = 0
        self._t0 = time.monotonic()

    def frame(self) -> bytes:
        w, h = self.width, self.height
        img = self._Image.new("RGB", (w, h), (20, 24, 32))
        d = self._Draw.Draw(img)
        # renk şeritleri (klasik test deseni) + kayan çubuk
        colors = [(200, 200, 200), (200, 200, 0), (0, 200, 200), (0, 200, 0),
                  (200, 0, 200), (200, 0, 0), (0, 0, 200)]
        band = h // 2
        for i, c in enumerate(colors):
            d.rectangle((i * w // len(colors), 0, (i + 1) * w // len(colors), band), fill=c)
        t = time.monotonic() - self._t0
        x = int((t * 0.25 % 1.0) * w)
        d.rectangle((x - 8, band, x + 8, h), fill=(255, 140, 0))
        lines = ["HEXAPOD KAMERA - DENEME DESENI",
                 time.strftime("%H:%M:%S"),
                 f"kare {self._n}"]
        y = band + h // 16
        for text in lines:
            d.text((w // 20, y), text, fill=(235, 235, 235), font=self._font)
            y += int(self._font.size * 1.4) if hasattr(self._font, "size") else 16
        self._n += 1
        out = io.BytesIO()
        img.save(out, format="JPEG", quality=self.quality)
        return out.getvalue()


__all__ = ["TestPattern"]
