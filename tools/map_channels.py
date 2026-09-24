#!/usr/bin/env python3
"""Kanal haritası çıkarıcı: hangi servo hangi kanala takılı?

Ne yapar
--------
18 servo kablosunu tek tek takip etmek yerine: program servo kartlarını
I2C'de kendisi bulur, her kanaldaki servoyu sırayla kıpırdatır, kullanıcı
da hangi bacağın hangi ekleminin kıpırdadığını yazar. Sonunda harita
ekrana basılır.

Bu yüzden servo kablolarının kartlara HANGİ SIRAYLA takıldığı önemli
değildir; sıra ne olursa olsun program bulur.

Güvenlik
--------
- Robot bir kutunun üstünde, bacaklar havada ve serbest olmalı. Servo ilk
  sinyali aldığında bulunduğu yerden orta konuma zıplar.
- Aynı anda yalnızca bir servo beslenir; kıpırdatma bitince bırakılır.
- Ctrl+C her an güvenli: bütün kanallar kapatılır, o ana kadar bulunanlar
  yine de ekrana basılır.

Kullanım
--------
    python3 tools/map_channels.py            # gerçek donanım
    python3 tools/map_channels.py --dry-run  # donanımsız deneme
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path
from typing import Callable

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS.parent / "src" / "hexapod_driver"))
sys.path.insert(0, str(TOOLS))

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[union-attr]
    except (AttributeError, OSError, ValueError):
        pass

from hexapod_driver import DryRunBackend, HexapodError, PCA9685, SMBusBackend  # noqa: E402

import hwcheck  # noqa: E402

LEG_COUNT = 6
JOINTS = ("coxa", "femur", "tibia")
TOTAL = LEG_COUNT * len(JOINTS)
CHANNELS = 16

# Kıpırdatma: ortaya git, iki kez ±100 us salla, ortaya dön, bırak.
# ±100 us MG996R'de ~±9 derece — gözle rahat görülür, bir yere çarpmaz.
CENTER_US = 1500
WIGGLE_US = 100
STEP_DELAY_S = 0.3

JOINT_ALIASES = {
    "c": "coxa", "coxa": "coxa",
    "f": "femur", "femur": "femur",
    "t": "tibia", "tibia": "tibia",
}
ANSWER = re.compile(r"^([1-6])\s*([a-z]+)$")

INTRO = """\
============================================================
  Kanal haritası çıkarıcı
============================================================
Program servoları sırayla, tek tek kıpırdatacak. Sen hangi
bacağın hangi ekleminin kıpırdadığını yazacaksın.

Başlamadan önce:
  [ ] Robot bir kutunun üstünde, bacaklar havada ve serbest.
      (Servo ilk sinyalde bulunduğu yerden orta konuma zıplar.)
  [ ] Bacaklara 1'den 6'ya numara bandı yapıştırıldı.
  [ ] Servo gücü açık.

Cevap nasıl yazılır:
  1c     bacak 1, coxa   (gövdeye en yakın; sağa sola döner)
  3f     bacak 3, femur  (ortadaki; bacağı kaldırır)
  6t     bacak 6, tibia  (en dıştaki; dizi büker)
  Enter  hiçbir şey kıpırdamadı (kanal boş)
  t      bir daha kıpırdat
  q      bitir

Ctrl+C her an güvenli: bütün servolar bırakılır.
"""


def parse_answer(text: str) -> tuple[str, object]:
    """Kullanıcının cevabını çöz.

    Dönüş (tür, değer):
      ("joint", (bacak, eklem))  ("none", None)  ("repeat", None)
      ("quit", None)             ("invalid", açıklama)
    """
    t = text.strip().lower()
    if t == "":
        return ("none", None)
    if t in ("t", "tekrar"):
        return ("repeat", None)
    if t in ("q", "çık", "cik"):
        return ("quit", None)
    match = ANSWER.match(t)
    if not match:
        return ("invalid", "Anlaşılmadı. Örnek: 1c, 3f, 6t — ya da boşsa Enter.")
    joint = JOINT_ALIASES.get(match.group(2))
    if joint is None:
        return ("invalid", "Eklem c (coxa), f (femur) ya da t (tibia) olmalı.")
    return ("joint", (int(match.group(1)), joint))


def wiggle(board: PCA9685, channel: int, delay: float) -> None:
    """Kanaldaki servoyu gözle görülür biçimde salla, sonra bırak."""
    board.set_pulse_us(channel, CENTER_US)
    time.sleep(2 * delay)
    for _ in range(2):
        board.set_pulse_us(channel, CENTER_US + WIGGLE_US)
        time.sleep(delay)
        board.set_pulse_us(channel, CENTER_US - WIGGLE_US)
        time.sleep(delay)
    board.set_pulse_us(channel, CENTER_US)
    time.sleep(delay)
    board.set_off(channel)


def run(
    boards: list[PCA9685],
    mapping: dict[tuple[int, str], tuple[int, int]],
    ask: Callable[[str], str] = input,
    out: Callable[[str], None] = print,
    delay: float = STEP_DELAY_S,
) -> bool:
    """Kartların kanallarını sırayla dolaş, cevapları mapping'e yaz.

    mapping dışarıdan verilir ki Ctrl+C'de o ana kadar bulunanlar kaybolmasın.
    Dönüş: bütün kanallar dolaşıldıysa ya da 18 eklem bulunduysa True,
    kullanıcı 'q' ile çıktıysa False.
    """
    for board in boards:
        for channel in range(CHANNELS):
            if len(mapping) == TOTAL:
                return True
            out(f"\nkart 0x{board.address:02x} · kanal {channel}: kıpırdatılıyor...")
            wiggle(board, channel, delay)
            while True:
                kind, value = parse_answer(ask("  hangi eklem kıpırdadı? > "))
                if kind == "repeat":
                    wiggle(board, channel, delay)
                    continue
                if kind == "invalid":
                    out(f"  {value}")
                    continue
                if kind == "quit":
                    return False
                if kind == "none":
                    break
                if value in mapping:
                    addr, prev = mapping[value]
                    out(f"  bacak {value[0]} {value[1]} zaten kart 0x{addr:02x} kanal "
                        f"{prev} olarak yazıldı. 't' ile tekrar bak ya da başka cevap ver.")
                    continue
                mapping[value] = (board.address, channel)
                out(f"  kaydedildi: bacak {value[0]} {value[1]}   ({len(mapping)}/{TOTAL})")
                break
    return True


def report(mapping: dict[tuple[int, str], tuple[int, int]],
           out: Callable[[str], None] = print) -> None:
    out("\n" + "=" * 60)
    out("  KANAL HARİTASI — bu bölümü kopyalayıp gönder")
    out("=" * 60)
    by_board: dict[int, list[tuple[int, int, str]]] = {}
    for (leg, joint), (addr, channel) in mapping.items():
        by_board.setdefault(addr, []).append((channel, leg, joint))
    for addr in sorted(by_board):
        out(f"kart 0x{addr:02x}:")
        for channel, leg, joint in sorted(by_board[addr]):
            out(f"  kanal {channel:>2} -> bacak {leg} {joint}")
    missing = [(leg, j) for leg in range(1, LEG_COUNT + 1) for j in JOINTS
               if (leg, j) not in mapping]
    if missing:
        out("bulunamayan: " + ", ".join(f"bacak {leg} {j}" for leg, j in missing))
    else:
        out(f"{TOTAL}/{TOTAL} eklem bulundu.")
    out("=" * 60)


def find_board_addresses(bus: int) -> list[int]:
    """Veri yolundaki PCA9685 kartlarının adreslerini bul."""
    found = hwcheck.scan_bus(bus)
    boards = sorted(a for a in found
                    if a in hwcheck.PCA_RANGE and a != hwcheck.PCA_ALLCALL)
    if not boards:
        raise HexapodError(
            "Hiç servo kartı bulunamadı.\n"
            "  - Kartların VCC, SDA, SCL, GND kabloları Pi'ye takılı mı?\n"
            "  - I2C açık mı? (Ubuntu'da: ls /dev/i2c-1)"
        )
    if len(boards) == 1:
        raise HexapodError(
            f"Tek kart bulundu (0x{boards[0]:02x}). İki kart takılıysa ikisi de aynı\n"
            "adreste demektir; o zaman bir kanal iki servoyu birden oynatır ve\n"
            "harita çıkarılamaz. Kartlardan birinin A0 noktasını lehimleyip\n"
            "tekrar deneyin."
        )
    return boards


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Hangi servo hangi kanala takılı, bul")
    parser.add_argument("--bus", type=int, default=1, help="I2C veri yolu (varsayılan 1)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Donanıma yazma; akışı denemek için sahte iki kart kullan")
    args = parser.parse_args(argv)

    print(INTRO)
    mapping: dict[tuple[int, str], tuple[int, int]] = {}

    if args.dry_run:
        backend = DryRunBackend()
        addresses = [0x40, 0x41]
        delay = 0.0
        print("(--dry-run: donanıma hiçbir şey gönderilmiyor)\n")
    else:
        try:
            addresses = find_board_addresses(args.bus)
            backend = SMBusBackend(args.bus)
        except HexapodError as exc:
            print(f"HATA: {exc}", file=sys.stderr)
            return 2
        delay = STEP_DELAY_S
        print("Bulunan servo kartları: " + ", ".join(f"0x{a:02x}" for a in addresses))
        if input("Hazırsan Enter'a bas, vazgeçmek için q > ").strip().lower() == "q":
            backend.close()
            return 0

    boards = [PCA9685(backend, addr, 50.0) for addr in addresses]
    try:
        for board in boards:
            board.begin()
        run(boards, mapping, delay=delay)
    except (KeyboardInterrupt, EOFError):
        print("\nKesildi.")
    except HexapodError as exc:
        print(f"\nHATA: {exc}", file=sys.stderr)
    finally:
        for board in boards:
            try:
                board.all_off()
            except Exception:
                pass
        backend.close()

    report(mapping)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
