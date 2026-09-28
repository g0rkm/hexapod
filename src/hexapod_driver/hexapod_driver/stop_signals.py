"""Düğümlerin Ctrl+C / SIGTERM kapanışı (saf Python; ROS düğümleri kullanır).

Sorun (2026-09-29, robot.launch.py denemesi): terminalde Ctrl+C'ye basınca
SIGINT bütün süreç grubuna gider, `ros2 launch` da her düğüme bir SIGINT daha
iletir. rclpy'nin kendi işleyicisi ilkinde bağlamı kapatıp çekiliyor; ikinci
sinyal varsayılan davranışla süreci ÖLDÜRÜYOR (çıkış kodu -2). Kapanış adımı
(sürücüde servoları bırakma) yarıda kalabilir. Sinyal yalnız launch'a
gönderilince (tek sinyal) aynı düğümler temiz kapanıyordu.

Çözüm: rclpy'nin işleyicisi kapatılır (rclpy.init(...,
signal_handler_options=SignalHandlerOptions.NO)) ve bunun yerine
stop_on_first_signal(): ilk SIGINT/SIGTERM KeyboardInterrupt fırlatır (spin'den
çıkılır, düğümün finally'sindeki kapanış çalışır), sonrakiler yok sayılır.

Kapanış düğüm daha kurulurken de gelebilir (launch, başka bir düğüm eksik
config yüzünden çıkınca ötekileri kapatır; o an biri hâlâ ROS modüllerini
yüklüyor olabilir). run_node bunu hata izi basmadan karşılar.
"""

from __future__ import annotations

import signal
import sys
from typing import Callable


def stop_on_first_signal() -> None:
    """İlk SIGINT ya da SIGTERM KeyboardInterrupt fırlatır; sonrakiler yok sayılır.
    Ana iş parçacığından çağrılmalı (Python sinyal işleyicisi kuralı)."""

    def _stop(signum, frame):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)


def run_node(run: Callable[[list[str] | None], int], argv: list[str] | None = None) -> int:
    """Düğümün main'i: sinyal işleyicisini ilk iş kur, run(argv)'yi çağır.
    Kapanış sinyali run kendi try'ına girmeden (kurulurken) gelirse hata izi
    basmadan 0 ile çık. run, rclpy'yi SignalHandlerOptions.NO ile başlatmalı."""
    stop_on_first_signal()
    try:
        code = run(argv)
    except KeyboardInterrupt:
        rclpy = sys.modules.get("rclpy")
        if rclpy is not None:
            rclpy.try_shutdown()
        return 0
    # İş bitti; çıkarken gelen sinyal çıkış kodunu (ör. eksik config: 2) bozmasın.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    return code


__all__ = ["run_node", "stop_on_first_signal"]
