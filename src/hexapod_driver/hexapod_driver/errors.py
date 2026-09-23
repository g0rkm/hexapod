"""Sürücü katmanının hata tipleri."""


class HexapodError(Exception):
    """Tüm hexapod hatalarının atası."""


class ConfigError(HexapodError):
    """robot.yaml / calibration.yaml okunamadı veya tutarsız."""


class MissingValue(ConfigError):
    """Gerekli bir yapılandırma değeri henüz girilmemiş (null).

    Bu hata bilerek fırlatılır: eksik bir parametrenin yerine varsayılan
    koymak, sonraki katmanların sessizce yanlış davranmasına yol açar.
    """

    def __init__(self, path: str, source: str = ""):
        self.path = path
        self.source = source
        msg = f"'{path}' değeri yapılandırmada yok (null)."
        if source:
            msg += f" Not: {source}"
        super().__init__(msg)


class LimitError(HexapodError):
    """İstenen açı/darbe, tanımlı sınırların dışında."""


class BackendError(HexapodError):
    """I2C veri yolu hatası."""
