from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class MsgItem:
    """Base type to define navigation message payload"""
    start: int
    len_: int
    two_comp: bool = False  # two complement - used for GPS/Gal/BDS
    signed_bin: bool = False  # signed binary used for Glonass to check highest bit for sign
    scale: Optional[float] = None

    def __post_init__(self):
        assert (self.two_comp and self.signed_bin) is False, "2-comp or signed_binary - Only one is allowed to be set"
