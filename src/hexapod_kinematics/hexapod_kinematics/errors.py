"""Kinematik katmanının hata tipleri."""

from hexapod_driver.errors import HexapodError


class ReachError(HexapodError):
    """Ayak hedefi bacağın erişim alanının dışında."""
