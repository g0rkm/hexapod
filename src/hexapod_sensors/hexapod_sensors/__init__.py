"""hexapod_sensors — mesafe sensörü ve IMU sürücüleri (GOREVLER.md S7).

Saf Python, ROS'a bağımlı değil; I2C ve GPIO erişimi hexapod_driver'ın
arka uçlarından gelir, böylece robotsuz test edilir (CLAUDE.md "Donanımsız
geliştirme"). hexapod_driver'daki desenin aynısı: donanıma dokunan katman
saf Python, ROS sarmalayıcısı ayrı (node.py).

vl53l0x.py      tek mesafe sensörü (kimlik, adresleme, sürekli ölçüm)
rangefinders.py üç sensörü XSHUT ile ayırıp adresleme ve topluca okuma
bno055.py       IMU: yerçekimi yönü, açısal hız, yönelim, kalibrasyon
mount.py        IMU sensör çerçevesi -> gövde çerçevesi (robot.yaml'dan)
fake.py         veri sayfasına göre davranan taklit cihazlar (test için)

DONANIMDA DENENMEDİ: bu paket yazmaç düzeyinde doğrulandı (taklit cihazlarla),
gerçek sensörle hiç çalıştırılmadı. Donanımda ayağa kaldırma D8'in işi;
bilinen riskler README ve modül açıklamalarında.
"""

from .rangefinders import RangeFinders, RangeFinderSpec, check_addresses, specs_from_config
from .vl53l0x import Vl53l0x

__all__ = [
    "RangeFinders", "RangeFinderSpec", "check_addresses", "specs_from_config",
    "Vl53l0x",
]

__version__ = "0.1.0"
