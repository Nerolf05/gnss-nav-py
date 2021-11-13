from enum import IntEnum, auto


class GnssId(IntEnum):
    """Enum defining different GNSS-system standards. TODO should be already defined in gnss_utility"""
    GPS = auto()
    GLONASS = auto()
    GALILEO = auto()
    BEIDOU = auto()
    IRNSS = auto()
    SBAS = auto()
    QZSS = auto()
