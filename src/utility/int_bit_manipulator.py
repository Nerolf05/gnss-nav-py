from typing import Optional
from .
from pyubx2.pyubx_nav_msg_decoder.navmsg_payload import MsgItem


def shift_mask_int(int_: int, r_shift: int, l_shift: int = 0, bits_oi: int = 0) -> int:
    """
    Shift integer right, extract bits_oi and left shift
    :param int_: integer to manipulate
    :param r_shift: bit shift right
    :param l_shift: final left shift
    :param bits_oi: bits of interest to keep
    :return: integer
    """
    return int_ >> r_shift & (2 ** bits_oi - 1) << l_shift


def decode_msg_item(data: int, msg_item: MsgItem, ref_len: int) -> int:
    """
    Lift item from raw_data while considering MsgItem attributes like scale or two_comp.
    :param data:        raw_data
    :param msg_item:    definition of msg item
    :param ref_len:     reference length to calculate correct right shift
    :return: Item
    """
    end_pos = msg_item.start + msg_item.len_
    v = shift_mask_int(int_=data, r_shift=(ref_len - end_pos), bits_oi=msg_item.len_)
    # Check two complement of value
    if msg_item.two_comp:
        v = uint_to_two_compl(value=v, length=msg_item.len_)
    elif msg_item.signed_bin:
        v = uint_to_signed_bin(value=v, length=msg_item.len_)
    # Apply scale at value
    if msg_item.scale:
        v = v * msg_item.scale
    return v


def uint_to_two_compl(value: int, length: int) -> int:
    """Convert int to two complement integer with binary operations."""
    if value >> (length - 1) & 1 == 0:  # check sign bit
        return value & (2 ** length - 1)
    else:
        return value | (~0 << length)


def uint_to_signed_bin(value: int, length: int) -> int:
    """Retrieve msb sign bit and apply it onto value[1:]"""
    value = shift_mask_int(int_=value, r_shift=0, bits_oi=length - 1)
    sign_bit = shift_mask_int(int_=value, r_shift=length - 1, bits_oi=1)
    sign = -1 if sign_bit == 0 else +1
    return value * sign


def int_append_int(int_a: int, int_b: int, l_shift_a: int = 0, bits_oi_b: Optional[int] = None) -> int:
    """
    Shift a (leading 1 of a) x left and append bits_oi_b b
    :param int_a: integer
    :param int_b: integer to append
    :param l_shift_a: left shift a
    :param bits_oi_b: bits of interest to append to a of b
    :return: interger
    """
    bits_oi_b = bits_oi_b if bits_oi_b is not None else l_shift_a
    return int_a << l_shift_a | (int_b & 2 ** bits_oi_b - 1)


def two_complement(buffer: bytes, pos: int, length: int, byteorder: str = "little") -> int:
    """
    This function calculates the two complement for bitarray, which length is not a multiple of 8.
    Note: buffer must stop after "pos + length";
    :param buffer: array of bytes
    :param pos: bit offset
    :param length: how many bits, starting at the offset, should be used.
    :param byteorder: encoding
    :return: two complement integer
    """
    value = int.from_bytes(buffer, byteorder=byteorder, signed=True) >> pos
    if value >> (length - 1) & 1 == 0:  # check sign bit
        return value & (2 ** length - 1)
    else:
        return value | (~0 << length)


def shift_mask_bytes(buffer: bytes, r_shift: int, l_shift: int = 0, bits_oi: int = 0,
                     byteorder: str = "little") -> int:
    """
    This function shifts an array of bytes and returns the corresponding unsigned integer.
    :param buffer: array of bytes
    :param r_shift: number of bits
    :param l_shift: possibility to concatenate two byte arrays with an 'or' later
    :param bits_oi: how many bits are relevant
    :param byteorder: encoding
    :return: unsigned integer
    """
    return (((int.from_bytes(buffer, byteorder=byteorder, signed=False)
              >> r_shift)
             & 2 ** bits_oi - 1)
            << l_shift)


