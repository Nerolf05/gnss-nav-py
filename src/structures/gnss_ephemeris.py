from dataclasses import dataclass, field
from numbers import Number

from pyubx2.pyubx_nav_msg_decoder.ubx_base_types import GnssId


@dataclass(frozen=True)
class BaseEphemeris:
    """Base type for GNSS-Ephemeris."""
    svid: int
    gnss_id: GnssId
    pass


@dataclass(frozen=True)
class GpsEphemeris(BaseEphemeris):
    """GPS-Base Ephemeris."""
    delta_n: Number
    m_zero: Number
    crs: Number
    cuc: Number
    e: Number
    cus: Number
    toe: Number
    cic: Number
    omega_zero: Number
    cis: Number
    i_zero: Number
    crc: Number
    omega: Number
    gnss_id: GnssId = field(default=GnssId.GPS, init=False)
    pass


@dataclass(frozen=True)
class LNavGpsEphemeris(GpsEphemeris):
    """GPS-L/Nav ephemeris"""
    iode: Number
    sqrt_a: Number
    omega_dot: Number
    idot: Number
    pass


@dataclass(frozen=True)
class CNavGpsEphemeris(GpsEphemeris):
    """GPS-C/Nav ephemeris"""
    delta_omega_dot: Number
    i_zero_dot: Number
    wn: Number
    ura_ed: Number
    sig_health_L1: Number
    sig_health_L2: Number
    sig_health_L5: Number
    top: Number
    delta_a: Number
    a_dot: Number
    delta_n_zero_dot: Number
    pass


@dataclass(frozen=True)
class GalileoEphemeris(BaseEphemeris):
    """Define arguments of Galileo ephemeris data."""
    # TODO discuss if clock-correction params should be included too
    m_zero: Number
    delta_n: Number
    e:  Number
    sqrt_a: Number
    omega_zero: Number
    i_zero: Number
    omega: Number
    omega_dot: Number
    i_dot: Number
    cuc: Number
    cus: Number
    crc: Number
    crs: Number
    cic: Number
    cis: Number
    toe: Number
    gnss_id: GnssId = field(default=GnssId.GALILEO, init=False)


@dataclass(frozen=True)
class GlonassEphemeris(BaseEphemeris):
    """Define arguments of Glonass immediate information (ephemeris parameters)"""
    # TODO discuss actual ephemeris parameters - vs ICD payload
    string_number: Number  # m
    t_k_hour: Number
    t_k_min: Number
    t_k_sec: Number
    t_b: Number
    m: Number  # M(1) - Yielded words are transmitted in navigational message of Glonass - M SV
    gamma_n: Number
    tau_n: Number
    x_n: Number
    y_n: Number
    z_n: Number
    x_dot_n: Number
    y_dot_n: Number
    z_dot_n: Number
    x_dot_dot_n: Number
    y_dot_dot_n: Number
    z_dot_dot_n: Number
    b_n: Number
    p: Number
    n_t: Number
    f_t: Number
    n: Number
    delta_tau_n: Number
    e_n: Number
    p1: Number
    p2: Number
    p3: Number
    p4: Number
    l_3rd_n: Number  # health flag for n_th satellite: 0 indicates healthy
    gnss_id: GnssId = field(default=GnssId.GLONASS, init=False)


@dataclass(frozen=True)
class BdsEphemeris(BaseEphemeris):
    """Define arguments of BeiDou ephemeris"""
    t_oe: Number
    sqrt_a: Number
    e: Number
    omega: Number
    delta_n: Number
    m_zero: Number
    omega_zero: Number
    omega_dot: Number
    i_zero: Number
    idot: Number
    cuc: Number
    cus: Number
    crc: Number
    crs: Number
    cic: Number
    cis: Number
    t_oc: Number
    a_0: Number
    a_1: Number
    a_2: Number
    aode: Number  # age of data ephemeris
    gnss_id: GnssId = field(default=GnssId.BEIDOU, init=False)
