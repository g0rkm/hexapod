"""hexapod_driver.stop_signals — ilk Ctrl+C/SIGTERM kapanışı başlatır, sonrakiler yok sayılır.

Gerçek süreçte sinyal gönderilerek (POSIX); Windows'ta sinyal gönderimi farklı, atlanır.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(sys.platform == "win32", reason="POSIX sinyalleri gerekir")

SRC = Path(__file__).resolve().parent.parent / "src" / "hexapod_driver"

# Terminal + launch gibi: iki sinyal art arda. İlki KeyboardInterrupt, kapanış
# sırasında gelen ikinci SIGINT ve bir SIGTERM süreci öldürmemeli.
SCRIPT = """
import os, signal, time
from hexapod_driver.stop_signals import stop_on_first_signal
stop_on_first_signal()
try:
    os.kill(os.getpid(), signal.SIGINT)
    time.sleep(5)
    print("sinyal gelmedi")
except KeyboardInterrupt:
    os.kill(os.getpid(), signal.SIGINT)
    os.kill(os.getpid(), signal.SIGTERM)
    time.sleep(0.2)
    print("kapanis tamamlandi")
"""


def test_ilk_sinyal_kapanisi_baslatir_sonrakiler_oldurmez():
    proc = subprocess.run([sys.executable, "-c", SCRIPT], capture_output=True, text=True,
                          timeout=30, env={"PYTHONPATH": str(SRC)})
    assert proc.returncode == 0, proc.stderr
    assert proc.stdout.strip() == "kapanis tamamlandi"


def test_sigterm_de_kapanisi_baslatir():
    script = SCRIPT.replace("os.kill(os.getpid(), signal.SIGINT)\n    time.sleep(5)",
                            "os.kill(os.getpid(), signal.SIGTERM)\n    time.sleep(5)")
    proc = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True,
                          timeout=30, env={"PYTHONPATH": str(SRC)})
    assert proc.returncode == 0, proc.stderr
    assert proc.stdout.strip() == "kapanis tamamlandi"
