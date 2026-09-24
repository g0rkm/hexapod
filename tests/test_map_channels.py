"""tools/map_channels.py testleri — donanımsız, DryRunBackend ile."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import map_channels as mc  # noqa: E402

from hexapod_driver import DryRunBackend, PCA9685  # noqa: E402

OFF = [0x00, 0x00, 0x00, 0x10]  # PCA9685'te "kanal tamamen kapalı" deseni


def make_boards(*addresses):
    backend = DryRunBackend()
    boards = [PCA9685(backend, a, 50.0) for a in addresses]
    for board in boards:
        board.begin()
    backend.writes.clear()
    return backend, boards


def scripted(answers):
    it = iter(answers)
    return lambda _prompt: next(it)


# ---------------------------------------------------------------------------
# Cevap çözümleme
# ---------------------------------------------------------------------------


def test_kisa_ve_uzun_cevaplar_cozuluyor():
    assert mc.parse_answer("1c") == ("joint", (1, "coxa"))
    assert mc.parse_answer(" 3 femur ") == ("joint", (3, "femur"))
    assert mc.parse_answer("6T") == ("joint", (6, "tibia"))


def test_ozel_cevaplar():
    assert mc.parse_answer("") == ("none", None)
    assert mc.parse_answer("t") == ("repeat", None)
    assert mc.parse_answer("q") == ("quit", None)


def test_gecersiz_cevaplar_reddediliyor():
    for text in ("7c", "0f", "1x", "coxa", "abc"):
        kind, _ = mc.parse_answer(text)
        assert kind == "invalid", text


# ---------------------------------------------------------------------------
# Oturum akışı
# ---------------------------------------------------------------------------


def test_cevaplar_haritaya_yaziliyor():
    _, boards = make_boards(0x40, 0x41)
    mapping = {}
    done = mc.run(boards, mapping, ask=scripted(["1c", "1f", "", "1t", "q"]),
                  out=lambda _m: None, delay=0)
    assert done is False  # 'q' ile çıkıldı
    assert mapping == {
        (1, "coxa"): (0x40, 0),
        (1, "femur"): (0x40, 1),
        (1, "tibia"): (0x40, 3),  # kanal 2 boştu
    }


def test_her_kanal_kipirdatildiktan_sonra_birakiliyor():
    """Aynı anda tek servo kuralı: her kanalın son yazması 'kapalı' olmalı."""
    backend, boards = make_boards(0x40)
    mc.run(boards, {}, ask=scripted(["1c", "2c", "3c", "q"]),
           out=lambda _m: None, delay=0)
    for channel in range(3):
        register = 0x06 + 4 * channel
        last = [data for addr, reg, data in backend.writes if reg == register][-1]
        assert last == OFF, f"kanal {channel} bırakılmamış"


def test_ayni_eklem_iki_kez_yazilamiyor():
    _, boards = make_boards(0x40)
    mapping = {}
    messages = []
    mc.run(boards, mapping, ask=scripted(["1c", "1c", "2c", "q"]),
           out=messages.append, delay=0)
    assert mapping == {(1, "coxa"): (0x40, 0), (2, "coxa"): (0x40, 1)}
    assert any("zaten" in m for m in messages)


def test_gecersiz_cevaptan_sonra_tekrar_soruluyor():
    _, boards = make_boards(0x40)
    mapping = {}
    mc.run(boards, mapping, ask=scripted(["bilmiyorum", "4f", "q"]),
           out=lambda _m: None, delay=0)
    assert mapping == {(4, "femur"): (0x40, 0)}


def test_ikinci_karta_geciliyor_ve_18_eklemde_duruyor():
    _, boards = make_boards(0x40, 0x41)
    answers = [f"{leg}{j[0]}" for leg in range(1, 7) for j in mc.JOINTS]  # 18 cevap
    mapping = {}
    done = mc.run(boards, mapping, ask=scripted(answers), out=lambda _m: None, delay=0)
    assert done is True
    assert len(mapping) == 18
    assert mapping[(6, "tibia")] == (0x41, 1)  # 17. ve 18. servo ikinci kartta


def test_rapor_eksikleri_listeliyor():
    lines = []
    mc.report({(1, "coxa"): (0x40, 0)}, out=lines.append)
    text = "\n".join(lines)
    assert "kanal  0 -> bacak 1 coxa" in text
    assert "bulunamayan:" in text and "bacak 6 tibia" in text
