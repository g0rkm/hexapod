#!/usr/bin/env python3
"""Servo kalibrasyon aracı.

Ne yapar
--------
18 servoyu tek tek, elle merkeze almanı sağlar ve bulunan değerleri
config/calibration.yaml dosyasına yazar.

Nasıl çalışır
-------------
Aynı anda YALNIZCA bir servo beslenir. Diğerleri serbesttir. Bunun iki
sebebi var: 18 servo aynı anda tork uygularsa besleme çöker (brifteki
brownout riski), ve tek servo hareket ederken ne olduğunu görebilirsin.

Başlangıç darbesi 1500 us'dir. Bu bir robot parametresi değil, hobi RC
servolarının ortak nötr darbesidir; sadece bir yerden başlamak için.
Gerçek merkez, senin gözünle ayarlayıp kaydettiğin değerdir.

Kullanım
--------
    python tools/calibrate.py                # gerçek donanım
    python tools/calibrate.py --dry-run      # donanımsız deneme
    python tools/calibrate.py --config yol/robot.yaml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "hexapod_driver"))

# Robot Linux'ta çalışıyor ama araç Windows'tan da denenebiliyor; oradaki
# konsol kod sayfası Türkçe karakterleri bozmasın.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[union-attr]
    except (AttributeError, OSError, ValueError):
        pass

from hexapod_driver import (  # noqa: E402
    Calibration,
    ConfigError,
    DryRunBackend,
    HexapodError,
    JOINT_NAMES,
    RobotConfig,
    ServoBus,
)

START_PULSE_US = 1500
DEFAULT_STEP_US = 10

HELP = """\
Merkez (c) ne demek
-------------------
IK'daki sıfır duruşu. Her eklemi şu konuma getirip 'c' ile kaydet:
  coxa  : bacak gövdeden dümdüz dışarı bakıyor
  femur : femur yere paralel
  tibia : tibia femura dik (femur yataysa tibia dümdüz aşağı)
Bu tanım hexapod_kinematics/leg.py ile aynı olmak ZORUNDA.

Pozitif yönler (dir için)
-------------------------
  coxa  + : yukarıdan bakınca saat yönünün tersine
  femur + : bacak yukarı kalkar
  tibia + : diz açılır, ayak dışarı gider

Komutlar
--------
  +  /  -          darbeyi adım kadar artır / azalt
  +N /  -N         darbeyi N us artır / azalt        (ör. +25)
  =N               darbeyi doğrudan N us yap          (ör. =1480)
  step N           adım büyüklüğünü N us yap          (ör. step 5)

  c                bu konumu MERKEZ olarak kaydet ve sıradakine geç
  dir + | dir -    bu eklemin yön işaretini belirle
  off              bu servoyu serbest bırak

  limit min        bu konumu eklemin ALT mekanik ucu olarak kaydet
  limit max        bu konumu eklemin ÜST mekanik ucu olarak kaydet
  span D           şu anki konumun merkeze göre D derece olduğunu söyle
                   (açıölçerle ölç; us/derece katsayısını hesaplar)

  n / p            sonraki / önceki eklem
  go L J           doğrudan git   (ör. go 2 femur)
  list             tüm eklemlerin kalibrasyon durumu
  gaps             yapılandırmada hâlâ eksik olan alanlar
  limits           ölçülen limitleri robot.yaml formatında yazdır

  save             calibration.yaml'a yaz
  q                kaydetmeyi sorup çık
  ?                bu yardım

Limit bulma sırası: önce 'c' ile merkezle, sonra 'dir' ile yönü belirle,
sonra 'span' ile us/derece katsayısını ölç, en son iki uca yürüyüp
'limit min' / 'limit max'. Ardından 'limits' çıktısını robot.yaml'a yapıştır.
"""


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Hexapod servo kalibrasyon aracı")
    parser.add_argument("--config", type=Path, default=None, help="robot.yaml yolu")
    parser.add_argument("--calibration", type=Path, default=None, help="calibration.yaml yolu")
    parser.add_argument(
        "--dry-run", action="store_true", help="Donanıma yazma, sadece ne yapacağını göster"
    )
    args = parser.parse_args(argv)

    try:
        config = RobotConfig.load(args.config)
    except ConfigError as exc:
        print(f"HATA: {exc}", file=sys.stderr)
        return 2

    if not config.wiring_is_complete():
        print("Kalibrasyona başlanamıyor: kablolama bilgisi eksik.\n")
        print("config/robot.yaml içinde şu alanlar doldurulmalı:")
        for gap in config.wiring_gaps():
            note = f"  ({gap.source})" if gap.source else ""
            print(f"  - {gap.path}{note}")
        print(
            "\nBunlar ölçüm değil, kablolama kararı: hangi servo hangi kartın "
            "hangi kanalına bağlı ve kartların I2C adresleri ne."
        )
        return 2

    calibration = Calibration.load(args.calibration)

    backend = DryRunBackend() if args.dry_run else None
    bus = ServoBus(config, calibration, backend=backend, dry_run=args.dry_run)

    try:
        bus.start()
    except HexapodError as exc:
        print(f"HATA: Sürücü kartları başlatılamadı.\n{exc}", file=sys.stderr)
        return 2

    session = Session(config, calibration, bus, dry_run=args.dry_run)
    try:
        session.run()
    except (KeyboardInterrupt, EOFError):
        print("\nKesildi.")
    finally:
        try:
            bus.release_all()
            bus.stop()
        except Exception:
            pass
    return 0


class Session:
    """Etkileşimli kalibrasyon oturumu."""

    def __init__(self, config: RobotConfig, calibration: Calibration,
                 bus: ServoBus, dry_run: bool = False):
        self.config = config
        self.calibration = calibration
        self.bus = bus
        self.dry_run = dry_run
        self.order = [(s.leg, s.joint) for s in config.joints]
        self.index = 0
        self.step = DEFAULT_STEP_US
        self.pulse = START_PULSE_US
        self.dirty = False

    # -- akış -------------------------------------------------------------

    def run(self) -> None:
        print(banner(self.config, self.calibration, self.dry_run))
        print(HELP)
        self.enter_joint()
        while True:
            try:
                raw = input(f"{self.prompt()} > ").strip()
            except EOFError:
                raw = "q"
            if not raw:
                continue
            if self.dispatch(raw) is False:
                return

    def dispatch(self, raw: str) -> bool | None:
        cmd, _, rest = raw.partition(" ")
        cmd = cmd.lower()
        rest = rest.strip()

        if cmd in ("?", "help", "h"):
            print(HELP)
        elif cmd == "q":
            return self.quit()
        elif cmd == "save":
            self.save()
        elif cmd == "list":
            self.print_table()
        elif cmd == "gaps":
            self.print_gaps()
        elif cmd == "limits":
            self.print_limits()
        elif cmd == "c":
            self.mark_center()
        elif cmd == "dir":
            self.set_direction(rest)
        elif cmd == "limit":
            self.mark_limit(rest)
        elif cmd == "span":
            self.mark_span(rest)
        elif cmd == "off":
            self.release()
        elif cmd == "n":
            self.goto(self.index + 1)
        elif cmd == "p":
            self.goto(self.index - 1)
        elif cmd == "go":
            self.goto_named(rest)
        elif cmd == "step":
            self.set_step(rest)
        elif raw[0] in "+-=":
            self.nudge(raw)
        else:
            print(f"Bilinmeyen komut: {raw!r}. Yardım için '?'.")
        return None

    # -- eklem seçimi -----------------------------------------------------

    @property
    def current(self) -> tuple[int, str]:
        return self.order[self.index]

    def prompt(self) -> str:
        leg, joint = self.current
        key = f"leg{leg}_{joint}"
        cal = self.calibration.joints.get(key)
        mark = "*" if cal and cal.center_us is not None else " "
        return f"[{self.index + 1}/{len(self.order)}]{mark} bacak{leg} {joint} @{self.pulse}us"

    def enter_joint(self) -> None:
        """Seçili ekleme geç: öncekini bırak, bunu bilinen merkezinden başlat."""
        leg, joint = self.current
        key = f"leg{leg}_{joint}"
        cal = self.calibration.joints.get(key)
        self.pulse = cal.center_us if (cal and cal.center_us is not None) else START_PULSE_US
        spec = self.config.joint(leg, joint)
        print(
            f"\n-> bacak {leg} / {joint}  "
            f"(kart {spec.driver.value}, kanal {spec.channel.value})"
        )
        if cal and cal.center_us is not None:
            print(f"   kayıtlı merkez: {cal.center_us} us"
                  + (f", yön {cal.direction:+d}" if cal.direction else ", yön belirsiz"))
        self.apply()

    def goto(self, new_index: int) -> None:
        if not 0 <= new_index < len(self.order):
            print("Liste sınırındasın.")
            return
        self.release(quiet=True)
        self.index = new_index
        self.enter_joint()

    def goto_named(self, rest: str) -> None:
        parts = rest.split()
        if len(parts) != 2:
            print("Kullanım: go <bacak 0-5> <coxa|femur|tibia>")
            return
        try:
            leg = int(parts[0])
        except ValueError:
            print("Bacak numarası sayı olmalı.")
            return
        joint = parts[1].lower()
        if joint not in JOINT_NAMES:
            print(f"Eklem adı şunlardan biri olmalı: {', '.join(JOINT_NAMES)}")
            return
        if (leg, joint) not in self.order:
            print(f"Böyle bir eklem yok: bacak {leg} {joint}")
            return
        self.goto(self.order.index((leg, joint)))

    # -- hareket ----------------------------------------------------------

    def set_step(self, rest: str) -> None:
        try:
            value = int(rest)
        except ValueError:
            print("Kullanım: step <us>")
            return
        if not 1 <= value <= 200:
            print("Adım 1-200 us arasında olmalı.")
            return
        self.step = value
        print(f"Adım {self.step} us.")

    def nudge(self, raw: str) -> None:
        sign, rest = raw[0], raw[1:].strip()
        if sign == "=":
            try:
                target = int(rest)
            except ValueError:
                print("Kullanım: =<us>  (ör. =1480)")
                return
        else:
            if rest:
                try:
                    delta = int(rest)
                except ValueError:
                    print("Kullanım: +<us> veya -<us>")
                    return
            else:
                delta = self.step
            target = self.pulse + (delta if sign == "+" else -delta)

        lo, hi = self.config.pulse_us_min, self.config.pulse_us_max
        if not lo <= target <= hi:
            print(f"{target} us güvenlik aralığının ({lo}-{hi}) dışında, gönderilmedi.")
            return
        self.pulse = target
        self.apply()

    def apply(self) -> None:
        leg, joint = self.current
        try:
            self.bus.set_pulse_us(leg, joint, self.pulse)
        except HexapodError as exc:
            print(f"HATA: {exc}")

    def release(self, quiet: bool = False) -> None:
        leg, joint = self.current
        try:
            self.bus.release(leg, joint)
            if not quiet:
                print("Servo serbest.")
        except HexapodError as exc:
            print(f"HATA: {exc}")

    # -- kayıt ------------------------------------------------------------

    def mark_center(self) -> None:
        leg, joint = self.current
        key = f"leg{leg}_{joint}"
        self.calibration.set_center(key, self.pulse)
        self.dirty = True
        print(f"Merkez kaydedildi: {key} = {self.pulse} us")
        if self.index + 1 < len(self.order):
            self.goto(self.index + 1)
        else:
            print("Son eklem. 'save' ile yazmayı unutma.")

    def set_direction(self, rest: str) -> None:
        token = rest.strip()
        if token not in ("+", "-"):
            print(
                "Kullanım: dir +  veya  dir -\n"
                "  Darbeyi artırdığında eklem POZİTİF yönde dönüyorsa '+',\n"
                "  ters yönde dönüyorsa '-'. Pozitif yönler:\n"
                "    coxa  + : yukarıdan bakınca saat yönünün tersine\n"
                "    femur + : bacak yukarı kalkar\n"
                "    tibia + : diz açılır, ayak dışarı gider"
            )
            return
        leg, joint = self.current
        key = f"leg{leg}_{joint}"
        self.calibration.set_direction(key, 1 if token == "+" else -1)
        self.dirty = True
        print(f"Yön kaydedildi: {key} = {token}1")

    def mark_limit(self, rest: str) -> None:
        which = rest.strip().lower()
        if which not in ("min", "max"):
            print(
                "Kullanım: limit min  veya  limit max\n"
                "  Eklemi mekanik olarak durduğu yere kadar yürüt, sonra kaydet.\n"
                "  DİKKAT: servoyu dayanağa zorlamayın; direnç hissedince durun."
            )
            return
        leg, joint = self.current
        key = f"leg{leg}_{joint}"
        self.calibration.set_limit(key, which, self.pulse)
        self.dirty = True
        print(f"Limit kaydedildi: {key}.{which} = {self.pulse} us")

        degrees = self.calibration.get(key).limits_deg()
        if degrees:
            print(f"  -> derece karşılığı: {degrees[0]:.1f} .. {degrees[1]:.1f}")

    def mark_span(self, rest: str) -> None:
        try:
            degrees = float(rest.replace(",", "."))
        except ValueError:
            print(
                "Kullanım: span <derece>\n"
                "  Eklemi merkezden belirgin biçimde uzaklaştır, açıölçerle\n"
                "  gerçek açıyı ölç ve buraya yaz (ör. span 45)."
            )
            return
        leg, joint = self.current
        key = f"leg{leg}_{joint}"
        try:
            us_per_deg = self.calibration.set_span(key, self.pulse, degrees)
        except (HexapodError, ValueError) as exc:
            print(f"HATA: {exc}")
            return
        self.dirty = True
        print(f"us/derece hesaplandı: {key} = {us_per_deg:.3f}")
        print(f"  (MG996R için beklenen mertebe ~11; çok sapıyorsa ölçümü tekrarla)")

    def print_limits(self) -> None:
        """Ölçülen limitleri robot.yaml'a yapıştırılacak biçimde yazdır."""
        rows = []
        for leg, joint in self.order:
            cal = self.calibration.joints.get(f"leg{leg}_{joint}")
            degrees = cal.limits_deg() if cal else None
            rows.append((leg, joint, degrees))

        done = sum(1 for _, _, d in rows if d)
        if not done:
            print(
                "Henüz dereceye çevrilebilir limit yok.\n"
                "Bir eklem için sırasıyla: 'c' (merkez), 'dir', 'span', "
                "'limit min', 'limit max'."
            )
            return

        print(f"\n# config/robot.yaml -> joints ({done}/{len(rows)} eklem hazır)")
        for leg, joint, degrees in rows:
            if degrees:
                limits = f"{{min: {degrees[0]:.1f}, max: {degrees[1]:.1f}}}"
            else:
                limits = "{min: null, max: null}"
            print(f"  - {{leg: {leg}, joint: {joint:<6} ..., limits_deg: {limits}}}")
        print("\n(driver/channel alanlarını kendi satırlarından koru, sadece "
              "limits_deg kısmını güncelle.)\n")

    def save(self) -> None:
        path = self.calibration.save(self.calibration.path)
        self.dirty = False
        centered, directed = self.calibration.summary()
        total = len(self.order)
        print(f"Yazıldı: {path}")
        print(f"  merkezlenen: {centered}/{total}   yönü belirlenen: {directed}/{total}")
        if self.dry_run:
            print("  (--dry-run: donanıma hiçbir şey gönderilmedi, dosya gerçek)")

    def quit(self) -> bool:
        if self.dirty:
            answer = input("Kaydedilmemiş değişiklik var. Yazılsın mı? [E/h] ").strip().lower()
            if answer in ("", "e", "evet", "y", "yes"):
                self.save()
        print("Çıkılıyor, bütün servolar serbest bırakılıyor.")
        return False

    # -- raporlar ---------------------------------------------------------

    def print_table(self) -> None:
        print(f"\n{'eklem':<16}{'kanal':<12}{'merkez':>10}{'yön':>7}"
              f"{'us/derece':>12}{'limitler':>20}")
        print("-" * 77)
        for leg, joint in self.order:
            spec = self.config.joint(leg, joint)
            cal = self.calibration.joints.get(f"leg{leg}_{joint}")
            channel = f"{spec.driver.value}:{spec.channel.value}"
            center = f"{cal.center_us} us" if cal and cal.center_us is not None else "-"
            direction = f"{cal.direction:+d}" if cal and cal.direction else "-"
            per_deg = f"{cal.us_per_deg:.2f}" if cal and cal.us_per_deg else "-"
            degrees = cal.limits_deg() if cal else None
            if degrees:
                limits = f"{degrees[0]:.0f} .. {degrees[1]:.0f}"
            elif cal and (cal.limit_min_us or cal.limit_max_us):
                limits = f"{cal.limit_min_us or '?'}/{cal.limit_max_us or '?'} us"
            else:
                limits = "-"
            print(f"bacak{leg} {joint:<9}{channel:<12}{center:>10}{direction:>7}"
                  f"{per_deg:>12}{limits:>20}")
        centered, directed = self.calibration.summary()
        print(f"\nmerkezlenen {centered}/{len(self.order)}, "
              f"yönü belirlenen {directed}/{len(self.order)}\n")

    def print_gaps(self) -> None:
        unknowns = self.config.unknowns()
        if not unknowns:
            print("config/robot.yaml içinde eksik alan yok.")
        else:
            print(f"\nconfig/robot.yaml içinde {len(unknowns)} alan hâlâ boş:")
            for value in unknowns:
                note = f"  <- {value.source}" if value.source else ""
                print(f"  {value.path}{note}")
        unverified = self.config.unverified()
        if unverified:
            print(f"\nDeğeri olan ama robottan ölçülmemiş {len(unverified)} alan:")
            for value in unverified:
                print(f"  {value.path} = {value.value}")
        print()


def banner(config: RobotConfig, calibration: Calibration, dry_run: bool) -> str:
    centered, _ = calibration.summary()
    lines = [
        "",
        "=" * 60,
        "  Hexapod servo kalibrasyonu",
        "=" * 60,
        f"  yapılandırma : {config.path}",
        f"  servo        : {config.servo_model} @ {config.pwm_frequency_hz:.0f} Hz",
        f"  kartlar      : "
        + ", ".join(
            f"#{d.id} 0x{int(d.address.value):02x}" for d in config.drivers.values()
        ),
        f"  merkezlenen  : {centered}/{len(config.joints)}",
    ]
    if dry_run:
        lines.append("  MOD          : --dry-run (donanıma yazılmıyor)")
    else:
        lines.append("  MOD          : GERÇEK DONANIM - servolar hareket edecek")
    lines.append("=" * 60)
    lines.append("Aynı anda yalnızca seçili servo beslenir; diğerleri serbesttir.")
    return "\n".join(lines)


if __name__ == "__main__":
    raise SystemExit(main())
