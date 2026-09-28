"""Zorluk seviyeleri ve hazır zemin setleri (GOREVLER.md S5; G7'nin müfredatı için).

Seviyeler kolaydan zora sıralı: `levels("engebe")[0]` en kolay. Eğitimde
müfredat (kolaydan zora geçiş) ve S6'nın ölçüm tablosu bunları kullanır.

Sayılar nereden: Görkem'in deneme zeminlerinde ölçülenler (PROJE_DEVIR §3.3).
Tripod 50 mm adımla 30 mm basamağı geçiyor, 45 mm'yi geçemiyor; politika
(ppo_lift50_3750k) 60 mm'ye kadar çıkıyor. Yani 15-60 mm aralığı kolaydan
"kimsenin geçemediği"ne kadar uzanıyor; seviyeler bu aralığa yayıldı.
Eğim tarafında 10° kolay, 20° politikanın önde olduğu yer, 30° sınır.
"""

from __future__ import annotations

from . import terrain as t
from .terrain import Terrain

#: Zemin türü -> kolaydan zora seviye üreticileri. Her biri Terrain döndürür.
LEVELS: dict[str, tuple] = {
    "düz": (
        lambda seed=0: t.flat(),
    ),
    "eğim": (
        lambda seed=0: t.slope(5.0),
        lambda seed=0: t.slope(10.0),
        lambda seed=0: t.slope(-10.0),
        lambda seed=0: t.slope(-20.0),
        lambda seed=0: t.slope(-30.0),
    ),
    "yan eğim": (
        lambda seed=0: t.slope(5.0, "y"),
        lambda seed=0: t.slope(10.0, "y"),
        lambda seed=0: t.slope(20.0, "y"),
    ),
    # Ölçüldü (2026-09-28, tripod:50, 10 s): kayma, sürtünme katsayısı eğimin
    # tanjantına yaklaşınca başlıyor. 10° μ0.5 -> +0.084 m/s (rahat),
    # 10° μ0.3 -> +0.051 (yürüyor, yavaşlıyor), 15° μ0.4 -> +0.017 (zar zor),
    # 15° μ0.3 -> -0.600 (tan15°=0.268, μ payı yok: robot kayıyor, yürüme yok).
    # Sonuncusu bilerek listede: "kimsenin çıkamadığı" ucu gösteriyor.
    "kaygan": (
        lambda seed=0: t.slope(-10.0, mu=0.5),
        lambda seed=0: t.slope(-10.0, mu=0.3),
        lambda seed=0: t.slope(-15.0, mu=0.4),
        lambda seed=0: t.slope(-15.0, mu=0.3),
    ),
    "basamak": (
        lambda seed=0: t.step(0.015),
        lambda seed=0: t.step(0.030),
        lambda seed=0: t.step(0.045),
        lambda seed=0: t.step(0.060),
    ),
    "merdiven": (
        lambda seed=0: t.stairs(0.020, 0.25),
        lambda seed=0: t.stairs(0.035, 0.25),
        lambda seed=0: t.stairs(0.050, 0.30),
    ),
    "engebe": (
        lambda seed=0: t.rough(0.020, seed=seed),
        lambda seed=0: t.rough(0.040, seed=seed),
        lambda seed=0: t.rough(0.060, seed=seed),
        lambda seed=0: t.rough(0.080, cell=0.12, seed=seed),
    ),
    "çukur": (
        lambda seed=0: t.pit(0.020),
        lambda seed=0: t.pit(0.040),
        lambda seed=0: t.pit(0.060),
    ),
    "yayla": (
        lambda seed=0: t.plateau(0.030),
        lambda seed=0: t.plateau(0.050),
    ),
}


def kinds() -> list[str]:
    return list(LEVELS)


def levels(kind: str, seed: int = 0) -> list[Terrain]:
    """Bir türün bütün seviyeleri, kolaydan zora."""
    if kind not in LEVELS:
        raise ValueError(f"bilinmeyen zemin türü {kind!r}; olanlar: {kinds()}")
    return [make(seed) for make in LEVELS[kind]]


def level(kind: str, index: int, seed: int = 0) -> Terrain:
    """Bir türün tek seviyesi. index 0 = en kolay; son seviyeyi aşarsa hata."""
    items = LEVELS.get(kind)
    if items is None:
        raise ValueError(f"bilinmeyen zemin türü {kind!r}; olanlar: {kinds()}")
    if not 0 <= index < len(items):
        raise ValueError(f"{kind!r} için seviye 0..{len(items) - 1} olmalı: {index}")
    return items[index](seed)


def difficulty(step_index: int, seed: int = 0) -> list[Terrain]:
    """Müfredat basamağı: 0 = yalnız düz, sonra her basamakta bir zorluk eklenir.

    Eğitim ortamlarına dağıtmak için (train.py --terrains gibi): erken
    basamaklarda kolay zeminler, ileride hepsi. Düz zemin her basamakta var:
    politikanın düz verimini kaybetmemesi için (ders 33, 34).
    """
    if step_index < 0:
        raise ValueError(f"basamak negatif olamaz: {step_index}")
    out = [t.flat(), t.flat()]
    order = ("eğim", "basamak", "engebe", "çukur", "yan eğim", "merdiven", "kaygan", "yayla")
    for i in range(min(step_index, 99)):
        kind = order[i % len(order)]
        idx = min(i // len(order), len(LEVELS[kind]) - 1)
        out.append(level(kind, idx, seed))
    return out


def evaluation_set(seed: int = 0) -> list[Terrain]:
    """S6'nın tablosu ve G7'nin "bitti" ölçümü için sabit zemin listesi.

    Her türden temsili seviyeler. **Bu liste değişirse depodaki eski ölçüm
    tabloları (docs/olcumler/) yeni ölçümlerle karşılaştırılamaz**; zemin
    eklemek/çıkarmak gerekirse tabloları da yeniden üretin. Yeni bir zorluk
    denemek için bunu değiştirmeyin, levels()/level() kullanın.
    """
    return [
        t.flat(),
        t.slope(-10.0), t.slope(-20.0), t.slope(10.0),
        t.slope(10.0, "y"),
        # İkisi birden: μ0.3'lü 10° yürünebiliyor (denetleyicileri ayırır),
        # 15°'de kimse tutunamıyor (sınırı gösterir). Bkz. LEVELS["kaygan"].
        t.slope(-10.0, mu=0.3),
        t.slope(-15.0, mu=0.3),
        t.step(0.030), t.step(0.045), t.step(0.060),
        t.stairs(0.035, 0.25),
        t.rough(0.040, seed=seed), t.rough(0.060, seed=seed),
        t.pit(0.040), t.plateau(0.050),
    ]


__all__ = ["LEVELS", "difficulty", "evaluation_set", "kinds", "level", "levels"]
