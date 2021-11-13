from dataclasses import dataclass, field
from numbers import Number
from typing import Optional, ClassVar, Dict

from pyubx2.pyubx_nav_msg_decoder.ubx_base_types import GnssId


@dataclass(frozen=False)
class BaseAlmanac:
    """Base type of a GNSS-Almanac"""
    svid:   int
    sv_health: bool
    gnss_id: GnssId
    pass


@dataclass(frozen=False)
class GpsAlmanac(BaseAlmanac):
    """Parameters of the GPS Almanac."""
    # TODO Weeknumber relevant?
    e: Number
    toa: Number
    delta_i: Number  # relative to i_zero
    omega_dot: Number
    sqrt_a: Number
    omega_zero: Number
    omega: Number
    m_zero: Number
    af_0: Number
    af_1: Number
    gnss_id: GnssId = field(default=GnssId.GPS, init=False)

    # _scale: ClassVar[Dict[str, float]] = {
    #     "e": 2 ** -21, "toa": 2 ** 12, "delta_i": 2 ** -19, "omega_dot": 2 ** -38, "sqrt_a": 2 ** -11,
    #     "omega_zero": 2 ** -23, "omega": 2 ** -23, "m_zero": 2 ** -23, "af_0": 2 ** -20, "af_1": 2 ** -38}

    # def __post_init__(self) -> None:
    #     for key, scale in self._scale.items():
    #         try:
    #             setattr(self, key, self._scale[key] * scale)
    #         except TypeError as e:
    #             warnings.warn(f"Could not scale alm data: {key} vs {getattr(self, key)} error: {e}")
    #     return None


@dataclass(frozen=False)
class GalileoAlmanac(BaseAlmanac):
    """Parameters of the Galileo Almanac."""
    delta_sqrt_a: Number
    e: Number
    delta_i: Number
    omega_zero: Number
    omega_dot: Number
    omega: Number
    m_zero: Number
    a_f0: Number
    a_f1: Number
    iod_a: Number
    t_oa: Number
    wn_a: Number
    e5_b_hs: Number
    e1_b_hs: Number
    e5_a_hs: Optional[Number] = None  # only transmitted by F/Nav
    gnss_id: GnssId = field(default=GnssId.GALILEO, init=False)
    sv_health: bool = field(default=True, init=False)  # more differentiate signal components described by e5


@dataclass(frozen=False)
class GlonassAlmanac(BaseAlmanac):
    """Parameters of Glonass Almanac."""
    # Non immediate data of string 5 -> probably not part of almanac TODO discuss
    # tau_c: Number
    # tau_gps: Number
    # n_4: Number  # 4-year interval
    # N_a: Number  # days
    n_a: Number
    h_n_a: Number
    lambda_n_a: Number
    t_lambda_n_a: Number
    delta_i_n_a: Number
    delta_t_n_a: Number
    delta_t_dot_n_a: Number
    epsilon_n_a: Number
    omega_n_a: Number
    m_n_a: Number
    # b1: Number  # part of frame 5 -> 14/15 string
    # b2: Number
    # kp: Number
    tau_n_a: Number
    c_n_a: Number
    gnss_id: GnssId = field(default=GnssId.GLONASS, init=False)
    pass


@dataclass(frozen=False)
class BdsAlmanac(BaseAlmanac):
    """Parameters of BeiDou Navigation Satellite System Almanac."""
    t_oa: Number
    sqrt_a: Number
    e: Number
    omega: Number
    m_zero: Number
    omega_zero: Number
    omega_dot: Number
    delta_i: Number
    a_zero: Number
    a_one: Number
    am_id: Number  # identification of time sharing broadcast
    sv_health: int
    gnss_id: GnssId = field(default=GnssId.BEIDOU, init=False)


@dataclass(frozen=False)
class GpsMidiAlmanac(GpsAlmanac):
    """Almost equal to GpsAlmanac but as Midi almanac are relative to reference values, scales are different."""
    _scale: ClassVar[Dict[str, float]] = {
        "toa": 2 ** 12, "e": 2 ** -16, "delta_i": 2 ** -14, "omega_dot": 2 ** -32, "sqrt_a": 2 ** -4,
        "omega_zero": 2 ** -15, "omega": 2 ** -15, "m_zero": 2 ** -15, "af_0": 2 ** -20, "af_1": 2 ** -37}
    pass


@dataclass(frozen=False)
class GpsReducedAlmanac(BaseAlmanac):
    _scale: ClassVar[Dict[str, float]] = {"delta_a": 2 ** 9, "omega_zero": 2 ** -6, "phi_zero": 5 ** -6}
    delta_a: Number
    omega_zero: Number
    phi_zero: Number
