"""Zemin üreteci: parametreli, tohumlu Gazebo zeminleri (GOREVLER.md S5).

Saf Python: gz, ROS, numpy gerektirmez (yalnız önizleme aracı matplotlib ister).
Bu yüzden testleri her yerde, Gazebo kurulu olmayan makinede de koşar.

Sözleşme (Görkem'in G7 notu, GOREVLER.md S5)
--------------------------------------------
Her zemin bir `Terrain`: `sdf` (düz zeminin yerine geçen statik <model> SDF
parçası) ve `height(x, y)` (aynı zeminin üst yüzeyinin dünya z'si, metre).
İkisi HEP BİRLİKTE kullanılır ve BİREBİR tutarlı olmak zorundadır; `hexapod_rl`
ayak temasını, gövde yüksekliğini ve devrilmeyi `height`'a göre hesaplıyor,
robotu da ona göre doğuruyor (`sim.spawn_height`). Tutarsızlık sessizce
eğitimi bozar: robot havada temas eder ya da zemine gömülür.

`Terrain` ikili gibi de çözülebilir, böylece Görkem'in beklediği
`sdf, height = terrain.slope(10)` kullanımı çalışır:

    from hexapod_terrain import terrain
    sdf, height = terrain.slope(10.0)
    env = HexapodEnv(terrain_sdf=sdf, terrain_height=height)

Neden hepsi kutu (box)
----------------------
Kutuların yanları dik olduğu için `height` parça parça sabittir ve SDF ile
BİREBİR aynıdır. Gazebo'nun <heightmap>'i ara değer hesapladığı için (bilinear)
aynı garantiyi vermez. Zorluk yükseklikle değil, komşu hücreler arasındaki
FARKLA belirlenir (bkz. rough).

Yönler: robot +x yönünde ilerler (REP-103), +y sol. Birimler metre, açı derece.

Hücre/basamak sınırları yarı açıktır: bir sınır çizgisinin tam üstünde yüzey
dikey bir duvardır, iki yükseklik de geometrik olarak doğrudur. `height`
[başlangıç, bitiş) kuralını kullanır. Ölçüde sıfır bir küme olduğu için fizik
açısından önemsiz; testler sınır üstünde örnek almaz.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Callable, Iterator

#: Zemin kutularının kalınlığı (m). Robot ayağı 5 mm yarıçaplı bir küre;
#: ince zemin fizik motorunda tünelleme riski taşır, kalın olması bedava.
THICKNESS = 0.2

#: Sonsuz düz zemin yerine kullanılan kutuların yatay boyu (m). Robot 60 s'de
#: ~5 m gidebiliyor (S6 ölçümleri); kenarından çıkmasın diye geniş.
EXTENT = 14.0

#: Robotun doğduğu kare (yarı kenar, m) engebede düz bırakılır: gömülü ya da
#: eğik doğmasın, engebeye yürüyerek girsin.
SPAWN_FLAT_HALF = 0.35

#: Düzlükten tam engebeye geçiş mesafesi (m). Bkz. rough().
SPAWN_RAMP = 0.5


@dataclass(frozen=True)
class Terrain:
    """Bir zemin: Gazebo modeli + yüzey yüksekliği fonksiyonu.

    label: insan için ad (tablolarda, dosya adlarında).
    params: üretim parametreleri (kaydedilsin, tekrar üretilebilsin).
    """

    label: str
    sdf: str
    height: Callable[[float, float], float]
    params: dict = field(default_factory=dict)

    def __iter__(self) -> Iterator:
        """`sdf, height = terrain.slope(10)` çalışsın diye."""
        return iter((self.sdf, self.height))


# ---------------------------------------------------------------------------
# SDF parçaları
# ---------------------------------------------------------------------------


def _surface(mu: float | None) -> str:
    """Sürtünme katsayısı. None: Gazebo varsayılanı (1).

    Ölçüldü (Görkem, 2026-09-26): düz zeminde mu 1.0-0.15 arası yürüyüşü
    etkilemiyor, kayganlık ancak eğimle anlamlı.
    """
    if mu is None:
        return ""
    if not mu > 0:
        raise ValueError(f"sürtünme katsayısı pozitif olmalı: {mu}")
    return (f"<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction>"
            "</surface>")


def _num(v: float) -> str:
    """SDF'e yazılan sayı: kısa ama kayıpsıza yakın, -0 yazmaz.

    Tohumlu üretimin BİREBİR tekrarlanabilir olması buna bağlı (aynı sayı hep
    aynı metin).
    """
    v = float(v)
    if not math.isfinite(v):
        raise ValueError(f"SDF'e sonlu olmayan sayı yazılamaz: {v}")
    if abs(v) < 5e-7:
        return "0"
    return f"{v:.6f}".rstrip("0").rstrip(".")


def _box(index: int, center: tuple[float, float, float], size: tuple[float, float, float],
         mu: float | None, rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> str:
    pose = " ".join(_num(v) for v in (*center, *rpy))
    dims = " ".join(_num(v) for v in size)
    geo = f"<pose>{pose}</pose><geometry><box><size>{dims}</size></box></geometry>"
    return (f'<collision name="box{index}">{geo}{_surface(mu)}</collision>'
            f'<visual name="box{index}">{geo}</visual>')


def _model(boxes: list[str]) -> str:
    """Kutuları tek statik modelde topla.

    Tek <link>: Gazebo her link için ayrı gövde tutar, binlerce link yavaşlatır.
    Model adı "ground": hexapod_rl.sim düz zemini bu adla değiştiriyor.
    """
    return ('<model name="ground"><static>true</static><link name="link">'
            + "".join(boxes) + "</link></model>")


# ---------------------------------------------------------------------------
# Zeminler
# ---------------------------------------------------------------------------


def flat(mu: float | None = None) -> Terrain:
    """Düz zemin. mu verilirse kaygan düz (ölçüm tabanı)."""
    label = "düz" if mu is None else f"kaygan düz μ{_num(mu)}"
    sdf = _model([_box(0, (0.0, 0.0, -THICKNESS / 2), (EXTENT, EXTENT, THICKNESS), mu)])
    return Terrain(label, sdf, lambda x, y: 0.0, {"kind": "flat", "mu": mu})


def slope(deg: float, axis: str = "x", mu: float | None = None) -> Terrain:
    """Orijinden geçen düz eğim; üst yüzey (0, 0)'da z=0.

    deg > 0: ileri (+x) yönünde ALÇALIR, yani yokuş aşağı. deg < 0 yokuş yukarı.
    axis "y": sol (+y) yönünde alçalır (yan eğim).
    Not: eğimli yüzeyde ayak, eğimi hesaplamayan bir yüksekliğe basarsa
    yanlış temas okunur; height burada tam çözümü verir (düzlem denklemi).
    """
    if axis not in ("x", "y"):
        raise ValueError(f"eksen x ya da y olmalı: {axis!r}")
    if not -45.0 <= deg <= 45.0:
        raise ValueError(f"eğim -45..45 derece olmalı: {deg}")
    t = math.radians(deg)
    # Kutu kendi merkezinden döner; üst yüz orijinden geçsin diye normal boyunca
    # yarım kalınlık aşağı kaydırılır.
    normal = (math.sin(t), 0.0, math.cos(t)) if axis == "x" else (0.0, math.sin(t), math.cos(t))
    rpy = (0.0, t, 0.0) if axis == "x" else (-t, 0.0, 0.0)
    center = tuple(-0.5 * THICKNESS * n for n in normal)
    sdf = _model([_box(0, center, (EXTENT, EXTENT, THICKNESS), mu, rpy)])

    k = math.tan(t)
    which = 0 if axis == "x" else 1

    def height(x: float, y: float) -> float:
        return -k * (x, y)[which]

    yon = {"x": "yokuş aşağı" if deg > 0 else "yokuş yukarı", "y": "yan eğim"}[axis]
    label = f"{yon} {_num(abs(deg))}°" + (f" μ{_num(mu)}" if mu is not None else "")
    return Terrain(label, sdf, height, {"kind": "slope", "deg": deg, "axis": axis, "mu": mu})


def step(height_m: float, at_x: float = 0.4, mu: float | None = None) -> Terrain:
    """Tek basamak: x >= at_x'te height_m yükseklik (negatifse iniş).

    Robot orijinde doğar; ileri yürürken önce ön ayaklar basamağa gelir.
    """
    lo, hi = min(0.0, height_m), max(0.0, height_m)

    def height(x: float, y: float) -> float:
        return height_m if x >= at_x else 0.0

    # Yüzeyi height'tan üret: taban alçak seviyede her yeri kaplar, üstüne
    # yüksek seviyenin OLDUĞU bölgeye bir kutu. İniş ve çıkışta yüksek bölge
    # ters taraftadır; bu yüzden hangisi olduğu height'a sorulur.
    high_ahead = height(at_x + 1.0, 0.0) == hi
    start = at_x if high_ahead else -EXTENT / 2
    end = EXTENT / 2 if high_ahead else at_x
    boxes = [_box(0, (0.0, 0.0, lo - THICKNESS / 2), (EXTENT, EXTENT, THICKNESS), mu)]
    if hi > lo:
        boxes.append(_box(1, ((start + end) / 2, 0.0, lo + (hi - lo) / 2),
                          (end - start, EXTENT, hi - lo), mu))

    sign = "" if height_m >= 0 else "inen "
    label = f"{sign}basamak {_num(abs(height_m) * 1000)} mm"
    return Terrain(label, _model(boxes), height,
                   {"kind": "step", "height_m": height_m, "at_x": at_x, "mu": mu})


def stairs(rise: float, run: float, count: int = 6, at_x: float = 0.4,
           mu: float | None = None) -> Terrain:
    """Merdiven: at_x'ten sonra count basamak, her biri rise yüksek, run derin.

    Tek basamaktan farkı sürekliliği: robot bir basamağı çıkınca öteki geliyor,
    ritmini korumak zorunda. rise < 0 inen merdiven.
    """
    if count < 1:
        raise ValueError(f"basamak sayısı en az 1 olmalı: {count}")
    if not run > 0:
        raise ValueError(f"basamak derinliği pozitif olmalı: {run}")

    # Basamak i, [at_x + i*run, at_x + (i+1)*run) aralığında bu yükseklikte;
    # son basamak merdiven sonundan ileriye kadar sürer (sahanlık).
    levels = [rise * (i + 1) for i in range(count)]

    def height(x: float, y: float) -> float:
        if x < at_x:
            return 0.0
        i = int((x - at_x) // run)
        return levels[min(i, count - 1)]

    # Yüzeyi height'tan üret: her dilim için tabandan o yüksekliğe bir kutu.
    # Böylece SDF ile height aynı kaynaktan gelir, ayrışamaz.
    lo = min(0.0, *levels) - THICKNESS
    boxes = []
    edges = [at_x + i * run for i in range(count)] + [at_x + count * run + EXTENT / 2]
    # Merdivenden önceki düz kısım (z=0'a kadar).
    boxes.append(_box(0, ((at_x - EXTENT / 2) / 2, 0.0, (lo + 0.0) / 2),
                      (EXTENT / 2 + at_x, EXTENT, -lo), mu))
    for i in range(count):
        start, end = edges[i], edges[i + 1]
        top_i = levels[min(i, count - 1)]
        boxes.append(_box(i + 1, ((start + end) / 2, 0.0, (lo + top_i) / 2),
                          (end - start, EXTENT, top_i - lo), mu))

    yon = "merdiven" if rise > 0 else "inen merdiven"
    label = f"{yon} {count}x{_num(abs(rise) * 1000)} mm"
    return Terrain(label, _model(boxes), height,
                   {"kind": "stairs", "rise": rise, "run": run, "count": count,
                    "at_x": at_x, "mu": mu})


def rough(amplitude: float, cell: float = 0.15, seed: int = 0, smooth: int = 1,
          length: float = 5.0, width: float = 2.0, mu: float | None = None) -> Terrain:
    """Engebe: hücre hücre rastgele yükseklik, komşular arasında YUMUŞAK geçiş.

    amplitude: en yüksek ile en alçak nokta arasındaki fark (m).
    cell: hücre boyu (m). Küçük hücre = daha sık ama daha küçük adımlar.
    smooth: komşu ortalaması kaç kez uygulanır. 0 = bağımsız rastgele
        (beyaz gürültü: yan yana iki hücre tam amplitude farklı olabilir, yani
        "engebe" değil rastgele duvarlar). 1-2 = gerçek engebeye benzer
        dalgalanma; zorluk amplitude/cell oranıyla (yerel eğimle) belirlenir.
    length, width: engebeli koridorun boyu (+x yönünde) ve genişliği. Kare
        yerine koridor: robot ileri yürür, kutu sayısı boşa patlamaz.
    Aynı seed aynı zemini verir (birebir aynı SDF metni). Koridor dışı düzdür;
        robot dışarı çıkarsa çökmez, zemin kolaylaşır.

    Fizik maliyeti (ölçüldü, 2026-09-27, süreç içi Gazebo, tek ortam):
    kutu sayısı simülasyonu yavaşlatıyor — 716 kutu 3.05x, 393 kutu 2.03x,
    315 kutu 1.88x, 184 kutu 1.59x (düz zemine göre). Kazanç ~300-400 kutudan
    sonra azalıyor. Varsayılan 5.0 x 2.0 m / 15 cm = ~393 kutu: 20 s'lik
    eğitim bölümüne (robot ~2 m gider) bol bol yeter. Paralel eğitimde bütün
    ortamlar adım başına birbirini beklediği için EN YAVAŞ ortam hızı belirler;
    tek engebeli ortam bile toplam hızı düşürür. Uzun ölçümlerde (S6) length
    büyütülebilir, maliyeti bilerek kabul edilir.

    Robotun doğduğu yer düz bırakılır (|x| < 0.35, |y| < 0.35): robot zemine
    gömülmüş ya da eğik doğmasın, yürüyüş engebeye yürüyerek girsin.
    """
    if amplitude < 0:
        raise ValueError(f"genlik negatif olamaz: {amplitude}")
    if not cell > 0:
        raise ValueError(f"hücre boyu pozitif olmalı: {cell}")

    nx, ny = max(1, round(length / cell)), max(1, round(width / cell))
    x0, y0 = -0.8, -width / 2  # koridor robotun biraz gerisinden başlar

    rng = random.Random(seed)
    grid = [[rng.random() for _ in range(ny)] for _ in range(nx)]
    for _ in range(max(0, smooth)):
        grid = _smoothed(grid, nx, ny)

    # Doğuş bölgesi düz, çevresi RAMPALI: engebe sıfırdan başlayıp RAMP boyunca
    # tam genliğe çıkar. Rampasız, düzlüğün hemen kenarında yarım genlikte bir
    # duvar oluşuyor (robot duvarla çevrili bir çukurda doğuyor); rampayla
    # robot engebeye yürüyerek giriyor.
    def ramp(cx: float, cy: float) -> float:
        d = max(abs(cx), abs(cy)) - SPAWN_FLAT_HALF - cell / 2
        return min(1.0, max(0.0, d / SPAWN_RAMP))

    lo, hi = min(map(min, grid)), max(map(max, grid))
    span = (hi - lo) or 1.0
    shaped = [[(grid[i][j] - lo) / span * ramp(x0 + (i + 0.5) * cell, y0 + (j + 0.5) * cell)
               for j in range(ny)] for i in range(nx)]
    # En yüksek nokta tam olarak amplitude olsun (istenen zorluk elde edilsin).
    peak = max(map(max, shaped)) or 1.0
    tops = [[round(amplitude * v / peak, 6) for v in col] for col in shaped]

    boxes = [_box(0, (0.0, 0.0, -THICKNESS / 2), (EXTENT, EXTENT, THICKNESS), mu)]
    for i in range(nx):
        for j in range(ny):
            h = tops[i][j]
            if h <= 0.0:
                continue  # taban zaten z=0
            boxes.append(_box(len(boxes), (x0 + (i + 0.5) * cell, y0 + (j + 0.5) * cell, h / 2),
                              (cell, cell, h), mu))

    def height(x: float, y: float) -> float:
        i = math.floor((x - x0) / cell)
        j = math.floor((y - y0) / cell)
        if 0 <= i < nx and 0 <= j < ny:
            return tops[i][j]
        return 0.0

    label = f"engebe {_num(amplitude * 1000)} mm"
    return Terrain(label, _model(boxes), height,
                   {"kind": "rough", "amplitude": amplitude, "cell": cell, "seed": seed,
                    "smooth": smooth, "length": length, "width": width, "mu": mu})


def _smoothed(grid: list[list[float]], nx: int, ny: int) -> list[list[float]]:
    """Her hücreyi kendisi ve 4 komşusunun ortalamasıyla değiştir (kenarda kırp)."""
    out = [[0.0] * ny for _ in range(nx)]
    for i in range(nx):
        for j in range(ny):
            total, n = grid[i][j], 1
            for di, dj in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                a, b = i + di, j + dj
                if 0 <= a < nx and 0 <= b < ny:
                    total += grid[a][b]
                    n += 1
            out[i][j] = total / n
    return out


def pit(depth: float, half: float = 0.35, mu: float | None = None) -> Terrain:
    """Çukur: robot |x|,|y| < half karesinde doğar, her yönde depth'lik basamak
    çıkar. Her yöne komutla eğitimde (--omni) her yön engele varır."""
    if not depth > 0:
        raise ValueError(f"derinlik pozitif olmalı: {depth}")
    L = EXTENT / 2
    boxes = [
        _box(0, (0.0, 0.0, -THICKNESS / 2), (EXTENT, EXTENT, THICKNESS), mu),
        _box(1, (half + L / 2, 0.0, depth / 2), (L, 2 * (half + L), depth), mu),
        _box(2, (-half - L / 2, 0.0, depth / 2), (L, 2 * (half + L), depth), mu),
        _box(3, (0.0, half + L / 2, depth / 2), (2 * half, L, depth), mu),
        _box(4, (0.0, -half - L / 2, depth / 2), (2 * half, L, depth), mu),
    ]

    def height(x: float, y: float) -> float:
        return depth if max(abs(x), abs(y)) >= half else 0.0

    return Terrain(f"çukur {_num(depth * 1000)} mm", _model(boxes), height,
                   {"kind": "pit", "depth": depth, "half": half, "mu": mu})


def plateau(height_m: float, half: float = 0.5, mu: float | None = None) -> Terrain:
    """Yayla: robot height_m yüksekliğinde bir karenin üstünde doğar, her yönde
    basamak iner (çukurun tersi: inmeyi öğrenir)."""
    if not height_m > 0:
        raise ValueError(f"yükseklik pozitif olmalı: {height_m}")
    boxes = [
        _box(0, (0.0, 0.0, -THICKNESS / 2), (EXTENT, EXTENT, THICKNESS), mu),
        _box(1, (0.0, 0.0, height_m / 2), (2 * half, 2 * half, height_m), mu),
    ]

    def height(x: float, y: float) -> float:
        return height_m if max(abs(x), abs(y)) < half else 0.0

    return Terrain(f"yayla {_num(height_m * 1000)} mm", _model(boxes), height,
                   {"kind": "plateau", "height_m": height_m, "half": half, "mu": mu})


__all__ = ["EXTENT", "THICKNESS", "Terrain", "flat", "pit", "plateau", "rough", "slope",
           "stairs", "step"]
