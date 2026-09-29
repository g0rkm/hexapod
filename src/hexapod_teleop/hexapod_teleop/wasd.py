"""WASD klavye kumandası çekirdeği: tuş -> hız komutu, ROS'suz.

Tuşlar (2026-09-29'da istenen düzen):

    W ileri    S geri    A sola dön    D sağa dön    K dur
    Q hızı artır    E hızı azalt

Davranış: bir yön tuşuna BİR KEZ basmak yeter; robot K'ye ya da başka bir yön
tuşuna basılana kadar o yönde gider. Basılı tutma düzeni kullanılmadı: terminal
tuşun bırakıldığını bildirmez, tuşu basılı tutunca da işletim sistemi ilk
basıştan sonra ~0.5 s bekleyip tekrar etmeye başlar; o boşlukta politika
düğümünün 0.5 s'lik komut zaman aşımı (hexapod_policy.controller) robotu
durdurup yeniden kaldırırdı.

Hız 5 kademe, en yüksek kademe TeleopLimits: tripod'un test edildiği ve
politikaların eğitildiği aralık (0.15 m/s, 0.5 rad/s). En düşük kademe (1/3)
politika düğümünün ölü bölgesinin (aralığın 1/6'sı) iki katı; yani her kademede
yürür. Başlangıç kademesi 0.10 m/s: modellerin ölçüldüğü hız
(docs/olcumler/). Q/E kademe değiştirince süren hareket yeni hızla devam eder.

Yayınlama ve terminal wasd_node.py'de; bu dosya yalnız tuşun anlamı.
"""

from __future__ import annotations

from .controller import Command, TeleopLimits

#: Hız kademeleri, en yüksek hızın (TeleopLimits) kesri.
LEVELS = (1 / 3, 1 / 2, 2 / 3, 5 / 6, 1.0)
#: Başlangıç kademesi: 2/3 x 0.15 = 0.10 m/s.
DEFAULT_LEVEL = 2

#: Yön tuşu -> (hareketin adı, ileri işareti, dönüş işareti). Dönüş + = sola
#: (yukarıdan bakınca saat yönünün tersi, REP-103).
MOTIONS = {
    "w": ("ileri", 1, 0),
    "s": ("geri", -1, 0),
    "a": ("sola dön", 0, 1),
    "d": ("sağa dön", 0, -1),
}
STOP_KEY = "k"
FASTER_KEY = "q"
SLOWER_KEY = "e"

HELP = """\
Hexapod WASD kumandası (/cmd_vel)
  W ileri    S geri    A sola dön    D sağa dön    K dur
  Q hızı artır    E hızı azalt    Ctrl+C çıkış
Yön tuşuna bir kez basın; robot K'ye basılana kadar o yönde gider."""


class WasdTeleop:
    """Tuşları hız komutuna çeviren durum.

    Kullanım:
        t = WasdTeleop()
        t.press("w")            # tanınan tuşta True
        vx, vy, wz = t.command()
    """

    def __init__(self, limits: TeleopLimits | None = None, level: int = DEFAULT_LEVEL) -> None:
        if not 0 <= level < len(LEVELS):
            raise ValueError(f"kademe 0..{len(LEVELS) - 1} olmalı: {level}")
        self.limits = limits or TeleopLimits()
        self.level = level
        self.motion: str | None = None     # MOTIONS anahtarı; None = duruyor

    def press(self, key: str) -> bool:
        """Bir tuşu işle. Tanınan tuşta True; öteki tuşlar yok sayılır (False).
        Büyük/küçük harf fark etmez (Caps Lock açık da olsa çalışsın)."""
        k = key.lower()
        if k in MOTIONS:
            self.motion = k
        elif k == STOP_KEY:
            self.motion = None
        elif k == FASTER_KEY:
            self.level = min(self.level + 1, len(LEVELS) - 1)
        elif k == SLOWER_KEY:
            self.level = max(self.level - 1, 0)
        else:
            return False
        return True

    def stop(self) -> None:
        self.motion = None

    def speeds(self) -> tuple[float, float]:
        """Bu kademedeki (ileri m/s, dönüş rad/s)."""
        f = LEVELS[self.level]
        return self.limits.vx_max * f, self.limits.wz_max * f

    def command(self) -> Command:
        """(vx m/s, vy m/s, wz rad/s); duruyorsa sıfır."""
        if self.motion is None:
            return (0.0, 0.0, 0.0)
        _, forward, turn = MOTIONS[self.motion]
        v, w = self.speeds()
        return (forward * v, 0.0, turn * w)

    def status(self) -> str:
        """Terminalde gösterilecek tek satır."""
        v, w = self.speeds()
        kademe = f"hız kademesi {self.level + 1}/{len(LEVELS)} ({v:.3f} m/s, {w:.2f} rad/s)"
        if self.motion is None:
            return f"DUR · {kademe}"
        return f"{_tr_upper(MOTIONS[self.motion][0])} · {kademe}"


def _tr_upper(text: str) -> str:
    """Türkçe büyük harf: str.upper() 'i'yi 'I' yapıyor ("GERI")."""
    return text.replace("i", "İ").upper()


__all__ = ["DEFAULT_LEVEL", "HELP", "LEVELS", "MOTIONS", "WasdTeleop"]
