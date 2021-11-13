import warnings
from abc import ABCMeta, abstractmethod, ABC
from enum import IntEnum
from typing import List, Dict, Optional, Any, Tuple, ClassVar, Set, Sequence

from src.utility.int_bit_manipulator import  \
        shift_mask_int, \
        decode_msg_item, \
        int_append_int
from src.structures.msg_item import MsgItem
from src.messages.navmsg_payload import\
    Payload_GPS_CNAV, \
    Payload_GPS_LNAV, \
    Payload_GPS_CNAV_RED_ALM, \
    Payload_GPS_CNAV_CDC, \
    Payload_GPS_CNAV_EDC, \
    Payload_GAL_INAV_NOMINAL_FRAME_LAYOUT, \
    Payload_GAL_INAV_WORDS, \
    Payload_GLO_NAV, \
    Payload_BDS_D1_NAV, \
    PayloadBaseType

from src.structures.gnss_base_basetypes import GnssId
from src.structures.gnss_ephemeris import \
    BaseEphemeris, \
    GlonassEphemeris, \
    LNavGpsEphemeris, \
    CNavGpsEphemeris, \
    GalileoEphemeris, \
    BdsEphemeris

from src.structures.gnss_almanac import \
    BaseAlmanac, \
    GpsAlmanac, \
    GalileoAlmanac, \
    GlonassAlmanac, \
    BdsAlmanac


class NavMsgError(Exception):
    """Custom error for Navigation messages"""
    pass


class GnssNavMsgId(IntEnum):
    UNDEFINED = 0
    # GPS:
    GPS_LNAV = 1
    GPS_CNAV = 2
    # GLON:
    GLON_NAV = 4
    # GAL:
    GAL_INAV = 6
    GAL_FNAV = 7
    # BeiDou:  # TODO check: B1I D1 == B2I D2
    BEID_CNAV = 9
    BEID_D1 = 10
    BEID_D2 = 11


GnssNavMsgType = int
UuidType = int
GnssSvidType = int


class GnssNavMessage(metaclass=ABCMeta):
    # specific items in nav-msg payload which require advanced postproc -> updated by childs
    items_require_spec_postproc: ClassVar[Dict[str, PayloadBaseType]] = {}

    def __init__(self, gnss_id: GnssId, msg_name: str = "GNSS Navigation Message", svid: int = -1):
        """
        Abstract base class for GNSS Navigation Message.
        :param gnss_id: Gnss system identifier
        :param msg_name: Message identifier
        :param svid: Satellite identifier, default of -1 represents unknown id
        """
        self.gnss: GnssId = gnss_id
        self.msg_name: str = msg_name
        self.svid: int = svid
        self._ephemeris: Optional[BaseEphemeris] = None
        self._almanac: Dict[int, BaseAlmanac] = {}
        self._all_almanac_avail: bool = False

        self.data: Dict = {}  # TODO check if pandas df, xarray or sth else more suited
        pass

    def __str__(self):
        return f"{self.msg_name} for svid = {self.svid}"

    @property
    def ephemeris(self) -> Optional[BaseEphemeris]:
        """Getter for ephemeris data. Returns None in case not all necessary data available to construct ephemeris."""
        if not self._ephemeris:
            self._build_ephemeris()
        return self._ephemeris

    @property
    def almanac(self) -> Dict[int, BaseAlmanac]:
        """Getter BaseAlmanac."""
        if not self._almanac:
            self._build_almanac()
        return self._almanac

    @property
    @abstractmethod
    def len_frame(self) -> int:
        """Return number of bits defining one subframe, message, ... and so on."""
        pass

    @abstractmethod
    def _build_ephemeris(self) -> None:
        """Virtual method to construct specific ephemeris data."""
        pass

    @abstractmethod
    def _build_almanac(self) -> None:
        """Virtual method to construct specific almanac data."""
        pass

    @classmethod
    @abstractmethod
    def get_uuid(cls, payload: Any) -> Any:
        """
        Abstract static calculation method to identify universal-unique identifier to define whole nav-msg.
        Payload generic datatype depends on nav-msg.
        """
        pass

    @classmethod
    @abstractmethod
    def uuid_to_payload(cls, uuid: int) -> PayloadBaseType:
        """Get corresponding payload definition based on uuid."""
        pass

    @classmethod
    @abstractmethod
    def rename_specific_package(cls, key: str, pck: Dict) -> Dict:
        """Rename keys of dictionary as id is often inside dict."""
        pass

    def add_frame(self, uuid: int, payload: int) -> bool:
        """Decode individual frame by parsing payload based on uuid blueprint and add items to msg."""
        result: bool = False
        blueprint = self.uuid_to_payload(uuid=uuid)

        if blueprint:
            try:
                items = self._decode_items(blueprint=blueprint, payload=payload)

                # Some items like msb|lsb or packed structures require an additional post_processing step
                items = self._post_process_items(items=items, uuid=uuid)
            except NavMsgError as e:
                raise NavMsgError(f"Decoding frame: {uuid} error: {e} occurred.")
            else:
                # Store item with unique name
                for item_name, item in items.items():
                    self.data.update({f"{uuid}_{item_name}": item})
                result = True

        return result

    def _decode_items(self, blueprint: PayloadBaseType, payload: int) -> Dict:
        """Decode payload with passed blueprint aka payload definition."""
        items = dict()
        for item_name, msg_item in blueprint.items():
            items.update(
                self._decode_msg_item(name=item_name, msg_item=msg_item, val=payload)
            )
        return items

    def _post_process_items(self, items: Dict[str, Any], uuid: int) -> Dict:
        """
        Specific items like packed ones (edc, cdc, ...) or msb|lsb require post_processing.
        :param items: Dictionary of all items
        :param uuid: universal unique identifier for subframe|page etc.
        :return:
        """
        result = {}
        for name, item in items.items():
            # Preprocessing steps are required in case of "_msb"
            new_name = name.rsplit("_msb", 1)[0]
            if name.endswith("_lsb"):
                continue  # discard lsbs as only msbs are processed

            elif name.endswith("_msb"):
                try:  # Connect msb + lsb
                    msb_msg_item, msb_val = item
                    lsb_msg_item, lsb_val = items.get(f"{new_name}_lsb")  # type: ignore
                    val = int_append_int(int_a=msb_val, int_b=lsb_val, l_shift_a=lsb_msg_item.len_)
                    dummy_msg_item = MsgItem(start=0, len_=msb_msg_item.len_ + lsb_msg_item.len_,
                                             two_comp=msb_msg_item.two_comp, scale=msb_msg_item.scale)
                except (TypeError, ValueError) as e:
                    raise NavMsgError(f"Item: {item} is no tuple - {e}")
            else:
                val = item
                dummy_msg_item = None  # type: ignore

            # Actual postprocessing
            if name.startswith(tuple(self.items_require_spec_postproc.keys())):  # do specific stuff based on nav-msg
                d = self._specific_unpacking(key=new_name, val=val)
            else:
                if name.endswith("_msb"):  # common post_processing for lsb and msb which don't require spec_postproc
                    val = decode_msg_item(data=val, msg_item=dummy_msg_item, ref_len=dummy_msg_item.len_)  # alr shifted
                d = {new_name: val}

            result.update(d)  # store already processed item
        return result

    def _specific_unpacking(self, key: str, val: int) -> Dict:
        """Unpack packed list of msg_item and get unique-id based on abstract rename-method."""
        result = dict()
        blueprint = getattr(self.items_require_spec_postproc, key, None)
        if blueprint:
            values = self._decode_items(blueprint=blueprint, payload=val)
            try:  # Update d to have unique name
                d = self.rename_specific_package(key=key, pck=values)
            except NavMsgError as e:
                raise NavMsgError(f"Could not rename dict - {e}")
            else:
                result.update(d)
        else:
            raise NavMsgError(f"Error occurred while trying to find blueprint ({key}) for advanced processing")
        return result

    def _decode_msg_item(self, name: str, msg_item: MsgItem, val: int) -> Dict:
        """
        Decode Message item if it does not require msb|lsb postprocessing
        :param name: Attribute name
        :param msg_item:    Message Item defining how to decode passed value
        :param val: raw_data as an integer
        :return: {name: decoded or raw_value}
        """
        if name.endswith(("_msb", "_lsb")):
            # Extract specific integer of interest
            val = shift_mask_int(
                int_=val,
                r_shift=self.len_frame - (msg_item.start + msg_item.len_),
                bits_oi=msg_item.len_
            )
            # Store msg_item and value for postprocessing
            result = (msg_item, val)
        else:
            # Msg_item is not required for postprocessing as data can be lifted directly
            result = decode_msg_item(data=val, msg_item=msg_item, ref_len=self.len_frame)  # type: ignore
        return {name: result}


class GpsNavMessage(ABC, GnssNavMessage):
    Preamble = 0b10001011

    def __init__(self, msg_name: str = "GPS Navigation Message", svid: int = -1):
        """Construct abstract GPS Navigation Message"""
        super(GpsNavMessage, self).__init__(gnss_id=GnssId.GPS, msg_name=msg_name, svid=svid)

    @property
    def len_frame(self) -> int:
        return 300


class LNavGpsMessage(GpsNavMessage):
    nof_pages: ClassVar[int] = 25
    # Transform subframe_id and page_id to unique page_id
    page_lookup: ClassVar[Dict] = {  # keys: Subframe, SV ID; NB not all pages due to key:  62: 12 & 62: 24
        4: {57: 1, 25: 2, 26: 3, 27: 4, 28: 5, 29: 7, 30: 8, 31: 9, 32: 10, 62: 12, 52: 13, 53: 14, 54: 15, 55: 17,
            56: 18, 58: 19, 59: 20, 60: 22, 61: 23, 63: 25},  # Note not all pages as svid indicate format (page 24==12)
        5: {1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8, 9: 9, 10: 10, 11: 11, 12: 12, 13: 13, 14: 14, 15: 15,
            16: 16, 17: 17, 18: 18, 19: 19, 20: 20, 21: 21, 22: 22, 23: 23, 24: 24, 51: 25},
    }

    # Transform unique page_id to ambiguous sheet_id for payload definition
    page_to_sheet: ClassVar[Dict] = {
        4: {(1, 6, 11, 16, 21): 100, (12, 19, 20, 22, 23, 24): 111, (18,): 117, (25,): 124, (13,): 112,
            (14, 15, 17): 113, (2, 3, 4, 5, 7, 8, 9, 10): 125},  # Note: (2,3..., 10): 125 'footnote in ICD'
        5: {tuple(range(1, 25)): 125, (25,): 149}
    }

    # set of unique pages of interest to get full nav-msg: subframe * 25 + (page - 1)
    uuid_pages: ClassVar[Set[int]] = {
        25, 50, 75,
        # subframe 4
        100, 101, 102, 103, 104, 106, 107, 108, 109, 110, 115, 120, 111, 118, 119, 121, 122, 123, 112, 113, 114, 116,
        117, 124,
        # subframe 5
        125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146,
        147, 148, 149
    }

    DataId: ClassVar[int] = 0b01
    NofSVs: ClassVar[int] = 32  # Number of satellites transmitting Legacy Navigation Message

    def __init__(self, svid: int):
        """Legacy navigation message with lower set of PRN(1-32)."""
        super(LNavGpsMessage, self).__init__(msg_name="GPS-L/NAV message", svid=svid)
        pass

    @classmethod
    def rename_specific_package(cls, key: str, pck: Dict) -> Dict:
        # already separable - nevertheless for LNAV there should not be any packed package
        return {key: pck}

    @classmethod
    def get_uuid(cls, payload: int) -> UuidType:
        """
        Extract uuid from payload
        :param payload: integer representing whole page
        :return: universal unique identifier for specific page
        """
        sf_id: int = shift_mask_int(int_=payload, r_shift=(300 - (30 * 1 + 22)), bits_oi=3)
        if sf_id not in range(1, 6):
            raise NavMsgError(f"Invalid GPS-LNAV Subframe: {sf_id}.")
        uuid = sf_id * cls.nof_pages
        if sf_id in (4, 5,):  # determine page
            data_id = shift_mask_int(int_=payload, r_shift=(300 - (30 * 2 + 2)), bits_oi=2)
            sv_id = shift_mask_int(int_=payload, r_shift=(300 - (30 * 2 + 8)), bits_oi=6)
            if sv_id == 0:  # TODO "0" indicates dummy sv
                raise NavMsgError(f"Dummy satellite {sv_id} not yet implemented: sure how to handle/what todo")
            if not data_id == cls.DataId:  # must be 0b01
                raise NavMsgError(f"Data-id: {data_id} != {cls.DataId}")
            # Calculate uuid based on page_id
            try:
                page_id = cls.page_lookup[sf_id][sv_id]
                uuid += page_id - 1
            except KeyError as e:
                raise NavMsgError(f"Lookup LNAV-pages with: {sv_id} failed: {e}")

        return uuid

    @classmethod
    def uuid_to_payload(cls, uuid: int) -> Dict[str, MsgItem]:
        key, result = "", {}
        if uuid >= 100:  # subframe 4/5: many-to-one relation: extract sheet id
            sf_id, page_id = divmod(uuid, cls.nof_pages)
            page_id += 1
            try:
                for pages in cls.page_to_sheet[sf_id].keys():
                    if page_id in pages:
                        sheet_id = cls.page_to_sheet[sf_id][pages]  # type: ignore
                        key = f"page_{sheet_id}"
                        break
            except KeyError as e:
                warnings.warn(f"KeyError: {e} occurred")

        else:  # direct mapping for subframes 1-3
            key = f"page_{uuid}"

        try:
            result = Payload_GPS_LNAV[key]
        except KeyError:
            warnings.warn(f"Could not find LNAV-payload for {key}")
        finally:
            return result

    def _build_ephemeris(self) -> None:
        """Construct LNav specific ephemeris data object."""
        try:  # directly access via keys to raise KeyError in case of missing item
            ephemeris = LNavGpsEphemeris(
                svid=self.svid,
                delta_n=self.data["50_delta_n"],
                m_zero=self.data["50_m_zero"],
                crs=self.data["50_crs"],
                cuc=self.data["50_cuc"],
                e=self.data["50_e"],
                cus=self.data["50_cus"],
                toe=self.data["50_toe"],
                cic=self.data["75_cic"],
                omega_zero=self.data["75_omega_zero"],
                cis=self.data["75_cis"],
                i_zero=self.data["75_i_zero"],
                crc=self.data["75_crc"],
                omega=self.data["75_omega"],
                iode=self.data["50_iode"],
                sqrt_a=self.data["50_sqrt_a"],
                omega_dot=self.data["75_idot"],
                idot=self.data["75_idot"],
            )
        except KeyError as e:
            raise ValueError(f"Value {e} not yet parsed into data.")
        else:
            self._ephemeris = ephemeris
        finally:
            return None

    def _build_almanac(self) -> None:

        sf_4_map_uuid_sv = {25: 2, 26: 3, 27: 4, 28: 5, 29: 7, 30: 8, 31: 9, 32: 10}  # map uuid of subframe 4 to svid
        for prn in range(1, 32 + 1):
            if prn <= 24:
                key = 125 + prn - 1
            else:
                key = 100 + sf_4_map_uuid_sv.get(prn, 999) - 1

            try:
                almanac = GpsAlmanac(
                    svid=prn,
                    sv_health=self.data[f"{key}_sv_health"],
                    e=self.data[f"{key}_e"],
                    toa=self.data[f"{key}_toa"],
                    delta_i=self.data[f"{key}_delta_i"],
                    omega_dot=self.data[f"{key}_omega_dot"],
                    sqrt_a=self.data[f"{key}_sqrt_a"],
                    omega_zero=self.data[f"{key}_omega_zero"],
                    omega=self.data[f"{key}_omega"],
                    m_zero=self.data[f"{key}_m_zero"],
                    af_0=self.data[f"{key}_af0"],
                    af_1=self.data[f"{key}_af1"],
                )
            except KeyError as e:
                warnings.warn(f"KeyError: {e}")
                continue
            else:
                self._almanac.update(
                    {prn: almanac}
                )
        return None


class CNavGpsMessage(GpsNavMessage):
    uuid_msg_types: ClassVar[Set[int]] = {10, 11, 30, 31, 32, 33, 34, 35, 36, 37, 12, 13, 14}  # 15 text-only

    uuid_dict: ClassVar[Dict[str, Set[int]]] = {
        "uuid_ephemeris": {10, 11},
        "uuid_clock": {30, 31, 32, 33, 34, 35, 36, 37},
        "uuid_isc_iono": {30},
        "uuid_reduced_almanac": {31, 12},
        "uuid_midi_almanac": {37},
        "uuid_eop": {32},
        "uuid_utc": {33},
        "uuid_diff_correction": {34, 13, 14},
        "uuid_ggto": {35},
        "uuid_text": {36, 15},
    }

    # Items which requires more advanced postprocessing
    items_require_spec_postproc = {
        "red_alm": Payload_GPS_CNAV_RED_ALM,
        "cdc": Payload_GPS_CNAV_CDC,
        "edc": Payload_GPS_CNAV_EDC,
    }

    # MaxSvsConstellation: ClassVar[int] = 2 ** 6  # max svs in constellation representable by transmitted prn (6 bits)

    def __init__(self, svid: int):
        """Construct GPS C/Navigation Message"""
        super(CNavGpsMessage, self).__init__(msg_name="GPS-L/NAV message", svid=svid)
        pass

    @classmethod
    def get_uuid(cls, payload: int) -> Tuple[UuidType, int, List[int], int]:
        """
        Extract message_type id and in case of almanac (midi or reduced) extracts prn_almanac
        :param payload: 300 bit long integer
        :return: MessageTypeId, prn_midi_alm (if msg_id=37), [prn_red_alm] (if msg_id=12/31), prn-transmitting sv
        """
        midi_alm_prn: int = 0
        red_alm_prns: List[int] = list()
        prn = shift_mask_int(int_=payload, r_shift=(300 - 14), bits_oi=6)
        msg_type_id: int = shift_mask_int(int_=payload, r_shift=(300 - 20), bits_oi=6)
        if msg_type_id == 0 or msg_type_id not in cls.uuid_msg_types:
            raise NavMsgError(f"MsgTypeId ({msg_type_id}) not defined or Default Message zero - Generation failure.")
        if msg_type_id == 37:  # Check midi - almanac for prn
            midi_alm_prn = shift_mask_int(int_=payload, r_shift=(300 - 154), bits_oi=6)
        elif msg_type_id in (12, 31,):  # Check reduced almanac
            mask = (65, 96, 127, 158, 189, 220, 251) if msg_type_id == 12 else (154, 185, 216, 247)
            for red_alm in mask:
                red_alm_prns.append(
                    shift_mask_int(int_=payload, r_shift=(300 - red_alm), bits_oi=6)
                )
        return msg_type_id, midi_alm_prn, red_alm_prns, prn

    @classmethod
    def uuid_to_payload(cls, uuid: int) -> Dict[str, MsgItem]:
        key = f"message_{uuid}"
        return Payload_GPS_CNAV[key]

    @classmethod
    def rename_specific_package(cls, key: str, pck: Dict) -> Dict:
        """Extract prn number of pck dict for unique identification."""
        prn = -1
        for key in pck.keys():
            if key.startswith("prn"):
                prn = pck[key]
                break
        if prn == -1:
            raise NavMsgError(f"Could not find prn in {pck}")
        new_key = f"{key}_sv{prn}"
        return {new_key: pck}

    def _build_ephemeris(self) -> None:
        try:
            assert self.data["10_t_oe"] == self.data["11_t_oe"], f"Time of ephemeris in msg 10 & 11 are not equal"
            ephemeris = CNavGpsEphemeris(
                svid=self.svid,
                delta_omega_dot=self.data["11_delta_omega_dot"],
                i_zero_dot=self.data["11_i_o_n_dot"],
                wn=self.data["10_wn"],
                ura_ed=self.data["10_ura_ed"],
                sig_health_L1=self.data["10_L1_health"],
                sig_health_L2=self.data["10_L2_health"],
                sig_health_L5=self.data["10_L5_health"],
                top=self.data["10_t_op"],
                delta_a=self.data["10_delta_a"],
                a_dot=self.data["10_a_dot"],
                delta_n_zero_dot=self.data["10_delta_n_zero_dot"],
                delta_n=self.data["10_delta_n_zero"],
                m_zero=self.data["10_m_zero_n"],
                crs=self.data["11_crc_n"],
                cuc=self.data["11_cuc_n"],
                e=self.data["10_e_n"],
                cus=self.data["11_cus_n"],
                toe=self.data["10_t_oe"],
                cic=self.data["11_cic_n"],
                omega_zero=self.data["11_omega_zero_n"],
                cis=self.data["11_cis_n"],
                i_zero=self.data["11_i_zero_n"],
                crc=self.data["11_crc_n"],
                omega=self.data["10_omega_n"],
            )
        except KeyError as e:
            raise NavMsgError(f"KeyError building ephemeris {e}")
        except AssertionError as e:
            raise NavMsgError(f"TOE are not equal: {e}")
        else:
            self._ephemeris = ephemeris
        finally:
            return None

    def _build_almanac(self) -> None:
        # TODO continue here and build different almanac sets -> but current dataset does not provide data
        pass


class GalNavMessage(ABC, GnssNavMessage):
    GAL_SYNC_PATTERN: ClassVar[bytes] = b""

    def __init__(self, msg_name: str = "Galileo Navigation Message", svid: int = -1):
        """Construct abstract Galileo Navigation Message"""
        super(GalNavMessage, self).__init__(gnss_id=GnssId.GALILEO, msg_name=msg_name, svid=svid)

    @property
    def len_frame(self) -> int:
        return 128


class INavGalMessage(GalNavMessage):
    GAL_SYNC_PATTERN = b"0101100000"

    def __init__(self, msg_name="I/Nav Galileo Message", svid: int = -1):
        """Construct Galileo I/Navigation Message"""
        super(INavGalMessage, self).__init__(msg_name=msg_name, svid=svid)
        pass

    def _build_almanac(self) -> None:
        """Check and build almanac."""
        # 1. Check whether all data is available (word 7, 8, 9, 10) of same iod_a
        alm_ids: List[Tuple[int, int, int, int]] = list()
        for d_key in self.data.keys():
            if not (d_key.endswith("iod_a") and d_key.startswith("7")):
                continue
            uuid_str = d_key.split("_", 1)
            uuid = int(uuid_str[0])
            next_sf_uuid = uuid + 1
            # check if other words are also available
            sfs_2_check: Tuple[str, str, str] = (
                d_key.replace("7", "8", 1),
                f"{str(next_sf_uuid).replace('7', '9', 1)}_{uuid_str[1]}",
                f"{str(next_sf_uuid).replace('7', '10', 1)}_{uuid_str[1]}",
            )
            if all(x in self.data.keys() for x in sfs_2_check):
                uuid_8_9_10: Sequence[int] = tuple([int(x.split("_", 1)[0]) for x in sfs_2_check])
                # uuid_8_9_10_mypy: Tuple[int, int, int] = cast(Tuple[int, int, int], uuid_8_9_10)  # help mypy
                alm_ids.append((uuid, ) + uuid_8_9_10)  # type: ignore

        # 2. Build actual almanacs
        for alm_id in alm_ids:
            w_7, w_8, w_9, w_10 = alm_id
            try:
                prn_1 = self.data[f"{w_7}_svid_1"]
                prn_2 = self.data[f"{w_8}_svid_2"]
                prn_3 = self.data[f"{w_9}_svid_3"]

                alm_1 = GalileoAlmanac(
                    svid=prn_1,
                    delta_sqrt_a=self.data[f"{w_7}_delta_sqrt_a"],
                    e=self.data[f"{w_7}_e"],
                    delta_i=self.data[f"{w_7}_delta_i"],
                    omega_zero=self.data[f"{w_7}_omega_zero"],
                    omega_dot=self.data[f"{w_7}_omega_dot"],
                    omega=self.data[f"{w_7}_omega"],
                    m_zero=self.data[f"{w_7}_m_zero"],
                    a_f0=self.data[f"{w_8}_a_f0"],
                    a_f1=self.data[f"{w_8}_a_f1"],
                    iod_a=self.data[f"{w_7}_iod_a"],
                    t_oa=self.data[f"{w_7}_t_oa"],
                    wn_a=self.data[f"{w_7}_wn_a"],
                    e5_b_hs=self.data[f"{w_8}_e5b_hs"],
                    e1_b_hs=self.data[f"{w_8}_e1b_hs"],
                )
                alm_2 = GalileoAlmanac(
                    svid=prn_2,
                    delta_sqrt_a=self.data[f"{w_8}_delta_sqrt_a"],
                    e=self.data[f"{w_8}_e"],
                    delta_i=self.data[f"{w_8}_delta_i"],
                    omega_zero=self.data[f"{w_8}_omega_zero"],
                    omega_dot=self.data[f"{w_8}_omega_dot"],
                    omega=self.data[f"{w_8}_omega"],
                    m_zero=self.data[f"{w_9}_m_zero"],
                    a_f0=self.data[f"{w_9}_a_f0"],
                    a_f1=self.data[f"{w_9}_a_f1"],
                    iod_a=self.data[f"{w_9}_iod_a"],
                    t_oa=self.data[f"{w_9}_t_oa"],
                    wn_a=self.data[f"{w_9}_wn_a"],
                    e5_b_hs=self.data[f"{w_9}_e5b_hs"],
                    e1_b_hs=self.data[f"{w_9}_e1b_hs"],
                )
                alm_3 = GalileoAlmanac(
                    svid=prn_3,
                    delta_sqrt_a=self.data[f"{w_9}_delta_sqrt_a"],
                    e=self.data[f"{w_9}_e"],
                    delta_i=self.data[f"{w_9}_delta_i"],
                    omega_zero=self.data[f"{w_10}_omega_zero"],
                    omega_dot=self.data[f"{w_10}_omega_dot"],
                    omega=self.data[f"{w_9}_omega"],
                    m_zero=self.data[f"{w_10}_m_zero"],
                    a_f0=self.data[f"{w_10}_a_f0"],
                    a_f1=self.data[f"{w_10}_a_f1"],
                    iod_a=self.data[f"{w_9}_iod_a"],
                    t_oa=self.data[f"{w_9}_t_oa"],
                    wn_a=self.data[f"{w_9}_wn_a"],
                    e5_b_hs=self.data[f"{w_10}_e5b_hs"],
                    e1_b_hs=self.data[f"{w_10}_e1b_hs"],
                )
            except KeyError as e:
                warnings.warn(f"{e} occured while building almanac {alm_id}")
            else:
                for prn, almanac in zip((prn_1, prn_2, prn_3), (alm_1, alm_2, alm_3)):
                    if prn != 0 and almanac:  # prn = 0 indicates dummy almanac
                        self._almanac.update(
                            {prn: almanac}
                        )
        return None

    def _build_ephemeris(self) -> None:
        """Build Galileo Ephemeris if all required arguments are available."""
        try:
            ephemeris = GalileoEphemeris(
                svid=self.svid,
                m_zero=self.data["1_m_zero"],
                omega_dot=self.data["3_omega_dot"],
                delta_n=self.data["3_delta_n"],
                i_zero=self.data["2_i_zero"],
                sqrt_a=self.data["1_sqrt_a"],
                omega_zero=self.data["2_omega_zero"],
                omega=self.data["2_omega"],
                i_dot=self.data["2_i_dot"],
                e=self.data["1_e"],
                cic=self.data["4_cic"],
                cis=self.data["4_cis"],
                crc=self.data["3_crc"],
                crs=self.data["3_crs"],
                cuc=self.data["3_cuc"],
                cus=self.data["3_cus"],
                toe=self.data["1_t_oe"],
            )
        except KeyError as e:
            warnings.warn(f"{e} while trying to build Gal Ephemeris for {self.svid}")
        else:
            self._ephemeris = ephemeris
        finally:
            return None

    @classmethod
    def get_uuid(cls, payload: Any) -> Any:
        """
        Determine unique identifier based on payload (word).
        Additionally determined are (if available):
            * issue of data for navigation or almanac to disseminate data batches
            * sub-frame-id of almanac
            * almanac sv id
        """
        word = payload.get("word")
        freq_id = payload.get("freq_id", -1)
        alm_sv_id = payload.get("alm_sv_id", 0)

        iod: int = -1  # issue of date
        sf_id_final: int = -1
        uuid: int = -1

        word_type = shift_mask_int(int_=word, r_shift=122, bits_oi=6)
        # Determine uuid
        if word_type < 7 or word_type in (16, 17, 18, 19, 20):
            uuid = word_type
            if 0 < word_type < 5:
                iod = shift_mask_int(int_=word, r_shift=128 - 16, bits_oi=10)

        elif word_type in (7, 8, 9, 10):  # for almanac determine sub-frame-id
            iod = shift_mask_int(int_=word, r_shift=118, bits_oi=4)
            # Determine svid or use passed one
            if word_type == 7:
                alm_sv_id = shift_mask_int(int_=word, r_shift=100, bits_oi=6)
            elif word_type == 8:
                alm_sv_id = shift_mask_int(int_=word, r_shift=79, bits_oi=6)
            elif word_type == 9:
                alm_sv_id = shift_mask_int(int_=word, r_shift=51, bits_oi=6)

            if alm_sv_id != 0 and freq_id in (1, 5):
                key = "E5b" if freq_id == 5 else "E1B"
                word_7_8 = word_type in (7, 8)

                # search for svid in subframe layout to determine sub-frame_id
                for sf_id, sf in Payload_GAL_INAV_NOMINAL_FRAME_LAYOUT.items():
                    if not (("7" in sf[key].keys()) == word_7_8):
                        continue  # skip subframes which don't contain word_type
                    if alm_sv_id in sf[key][str(word_type)]:
                        sf_id_final = int(sf_id)
                        break

                uuid = word_type
                # if sf_id_final != 0 and sv_id != 0:
                #     uuid = str(f"{word_type}_{iod_a}_{sf_id_final}")
        return uuid, iod, sf_id_final, alm_sv_id

    @classmethod
    def uuid_to_payload(cls, uuid: int) -> PayloadBaseType:
        """Transform uuid to payload definition"""
        result = {}
        key_prefix = "word_type"
        uuid_str = str(uuid)
        if uuid_str.startswith(("7", "8", "9", "10")):  # advanced unpacking required
            word_type = uuid_str[:2] if uuid_str.startswith("10") else uuid_str[:1]
            key = f"{key_prefix}_{word_type}"
        else:
            key = f"{key_prefix}_{uuid}"
        try:
            result = Payload_GAL_INAV_WORDS[key]
        except KeyError:
            warnings.warn(f"Could not find I/Nav-payload for {key}")
        finally:
            return result

    @classmethod
    def rename_specific_package(cls, key: str, pck: Dict) -> Dict:
        pass


class GlonNavMessage(GnssNavMessage):
    nof_frames: ClassVar[int] = 5
    nof_str_per_frame: ClassVar[int] = 15
    uuid: Set[int] = {1, 2, 3, 4, 5}.union(set([int(f"{f}{s}") for f in range(1, 5 + 1) for s in range(6, 15 + 1)]))

    def __init__(self, svid: int):
        """Construct Glonass Navigation Message"""
        super(GlonNavMessage, self).__init__(gnss_id=GnssId.GLONASS, msg_name="GLONASS NAV message", svid=svid)
        pass

    @property
    def len_frame(self) -> int:
        return 85

    def _build_ephemeris(self) -> None:
        """Build Glonass Ephemeris if all required arguments are available."""
        try:
            ephemeris = GlonassEphemeris(
                svid=self.svid,
                string_number=self.data["1_string_nr"],
                t_k_hour=self.data["1_t_k_hour"],
                t_k_min=self.data["1_t_k_min"],
                t_k_sec=self.data["1_t_k_sec"],
                t_b=self.data["2_t_b"],
                m=self.data["4_m"],
                gamma_n=self.data["3_gamma_n"],
                tau_n=self.data["4_tau_n"],
                x_n=self.data["1_x"],
                y_n=self.data["2_y"],
                z_n=self.data["3_z"],
                x_dot_n=self.data["1_x_dot"],
                y_dot_n=self.data["2_y_dot"],
                z_dot_n=self.data["3_z_dot"],
                x_dot_dot_n=self.data["1_x_dot_dot"],
                y_dot_dot_n=self.data["2_y_dot_dot"],
                z_dot_dot_n=self.data["3_z_dot_dot"],
                b_n=self.data["2_b_n"],
                p=self.data["3_p"],
                n_t=self.data["4_n_t"],
                f_t=self.data["4_f_t"],
                n=self.data["4_n"],
                delta_tau_n=self.data["4_delta_tau_n"],
                e_n=self.data["4_e_n"],
                p1=self.data["1_p1"],
                p2=self.data["2_p2"],
                p3=self.data["3_p3"],
                p4=self.data["4_p4"],
                l_3rd_n=self.data["3_l_n"],
            )
        except KeyError as e:
            warnings.warn(f"{e} while trying to build Glonass Ephemeris for {self.svid}")
        else:
            self._ephemeris = ephemeris
        finally:
            return None

    def _build_almanac(self) -> None:
        for prn_alm in range(1, 24 + 1):
            frame_id, alm_in_frame = divmod(prn_alm - 1, 5)
            frame_id += 1
            string = 6 + alm_in_frame * 2  # identify string in frame
            uuid_key_1, uuid_key_2 = f"{frame_id}{string}", f"{frame_id}{string+1}"
            if f"{uuid_key_1}_c_n" not in self.data.keys():  # string was not received skip
                continue
            try:
                almanac = GlonassAlmanac(
                    svid=prn_alm,
                    sv_health=self.data[f"{uuid_key_2}_l_n"],
                    n_a=self.data[f"{uuid_key_1}_n-a"],
                    h_n_a=self.data[f"{uuid_key_2}_h-a_n"],
                    lambda_n_a=self.data[f"{uuid_key_1}_lambda-a_n"],
                    t_lambda_n_a=self.data[f"{uuid_key_2}_t-a_lambda-n"],
                    delta_i_n_a=self.data[f"{uuid_key_1}_delta_i-a_n"],
                    delta_t_n_a=self.data[f"{uuid_key_2}_delta_t-a_n"],
                    delta_t_dot_n_a=self.data[f"{uuid_key_2}_delta_t_dot-a_n"],
                    epsilon_n_a=self.data[f"{uuid_key_1}_epsilon-a_n"],
                    omega_n_a=self.data[f"{uuid_key_2}_omega-a_n"],
                    m_n_a=self.data[f"{uuid_key_1}_m_n-a"],
                    tau_n_a=self.data[f"{uuid_key_1}_tau-a_n"],
                    c_n_a=self.data[f"{uuid_key_1}_c_n"],
                )
            except KeyError as e:  # specific item was not received
                warnings.warn(f"{e} while trying to build Glonass Almanac for {prn_alm}")
            else:
                self._almanac.update(
                    {prn_alm: almanac}
                )
        return None

    @classmethod
    def get_uuid(cls, payload: Any) -> UuidType:
        """Determine uuid based on Tuple[frame_number, string_number]"""
        frame_nr, string_nr = payload[0], payload[1]
        uuid = -1
        if string_nr <= 5:
            uuid = string_nr
        elif string_nr in range(6, 15 + 1):  # differentiate frame for almanac data
            if frame_nr > 0:  # frame_nr must be available to differentiate 5th frame otherwise discard
                uuid = int(f"{frame_nr}{string_nr}")
        return uuid

    @classmethod
    def uuid_to_payload(cls, uuid: int) -> PayloadBaseType:
        """Determine string-payload definition of Glonass Navigation message based on uuid."""
        result = {}
        if uuid <= 5:
            key = f"string_{uuid}"
        else:
            uuid_str = str(uuid)
            f_id, s_id = int(uuid_str[0]), int(uuid_str[1:])
            if f_id == 5 and s_id in (14, 15):  # differentiate string-payload definition
                key = f"string_{f_id * s_id}"
            else:
                key = f"string_{s_id}"
        try:
            result = Payload_GLO_NAV[key]
        except KeyError as _:
            warnings.warn(f"Glonass payload-key {key} is invalid")
        finally:
            return result

    @classmethod
    def rename_specific_package(cls, key: str, pck: Dict) -> Dict:
        pass


class BdsNavMessage(ABC, GnssNavMessage):
    Preamble: ClassVar[int] = 0b11100010010  # modified Barker Code
    AmpEpId: ClassVar[int] = 0b11  # identification of expanded almanacs in format
    SV_IDs: Set[int] = set()

    def __init__(self, gnss_id: GnssId = GnssId.BEIDOU, msg_name: str = "BeiDou Navigation Message", svid: int = -1):
        """Construct abstract BeiDou Navigation Message"""
        super(BdsNavMessage, self).__init__(gnss_id, msg_name, svid)
        pass

    @property
    def len_frame(self) -> int:
        return 300

    @staticmethod
    def _get_subframe_id(data: int) -> int:
        """Extract subframe identification (FraID)."""
        return shift_mask_int(int_=data, r_shift=300 - 18, bits_oi=3)

    @classmethod
    def rename_specific_package(cls, key: str, pck: Dict) -> Dict:
        pass


class BdsD1NavMessage(BdsNavMessage):
    SV_IDs: ClassVar[Set[int]] = set(range(6, 59))
    NofFrames: ClassVar[int] = 24

    def __init__(self, msg_name="D1 BeiDou Message", svid: int = -1):
        """Construct BeiDou D1 Navigation Message"""
        super(BdsD1NavMessage, self).__init__(msg_name=msg_name, svid=svid)
        pass

    @classmethod
    def get_uuid(cls, payload: Any) -> Any:
        """Get unique identifier for sub_frame."""
        sf_id = cls._get_subframe_id(data=payload)
        uuid = 0
        if 0 < sf_id < 4:
            uuid = sf_id
        elif sf_id in (4, 5):  # sub_frame page is determined
            page_num = shift_mask_int(int_=payload, r_shift=300 - 50, bits_oi=7)
            if page_num not in range(1, 25):
                return 0
            uuid = sf_id * cls.NofFrames + page_num
            if sf_id == 5:
                if page_num in range(11, 24):
                    am_id: Optional[int] = shift_mask_int(int_=payload, r_shift=8, bits_oi=2)
                elif page_num == 24:
                    am_id = shift_mask_int(int_=payload, r_shift=83, bits_oi=2)
                else:
                    am_id = None

                if am_id is not None:
                    if am_id == 0:  # discard reserved sub_frames
                        return 0
                    else:  # differentiate for am_id {1, 2, 3}
                        uuid = int(str(f"{uuid}{page_num}{am_id}"))
        return uuid

    @classmethod
    def uuid_to_payload(cls, uuid: int) -> PayloadBaseType:
        result = {}
        k = 0
        if uuid <= 3:
            k = uuid
        elif uuid < 400:
            sf_id, page_num = divmod(uuid, cls.NofFrames)
            if sf_id == 4 or (sf_id == 5 and page_num in (1, 2, 3, 4, 5, 6)):
                k = 4
            elif sf_id == 5:
                if page_num in (7, 8, 9, 10):
                    k = page_num
        else:
            page_num = int(str(uuid)[-3:-1])
            if page_num in (11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23):
                k = 11
            elif page_num == 24:
                k = page_num

        key = f"subframe_{k}"
        try:
            result = Payload_BDS_D1_NAV[key]
        except KeyError as e:
            warnings.warn(f"KeyError: {e} while asking for sub_frame payload definition")
        finally:
            return result

    def _calculate_toe(self) -> int:
        """Calculate time of ephemeris as it is separated onto different sub-frames"""
        # TODO check/discuss if msg global post_processing might be clearer / reasonable
        toe = 0
        try:
            toe_2 = self.data["2_t_oe"]
            toe_3 = self.data["3_t_oe"]
            toe = int_append_int(int_a=toe_2, int_b=toe_3, l_shift_a=10, bits_oi_b=10)
            toe *= 2 ** 3
        except KeyError as _:
            pass
        finally:
            return toe

    def _build_ephemeris(self) -> None:
        try:
            ephemeris = BdsEphemeris(
                svid=self.svid,
                t_oe=self._calculate_toe(),  # type: ignore
                sqrt_a=self.data[f"2_sqrt_a"],
                e=self.data[f"2_e"],
                omega=self.data[f"3_omega"],
                delta_n=self.data[f"2_delta_n"],
                m_zero=self.data[f"2_m_zero"],
                omega_zero=self.data[f"3_omega_zero"],
                omega_dot=self.data[f"3_omega_dot"],
                i_zero=self.data[f"3_i_zero"],
                idot=self.data[f"3_idot"],
                cuc=self.data[f"2_cuc"],
                cus=self.data[f"2_cus"],
                crc=self.data[f"2_crc"],
                crs=self.data[f"2_crs"],
                cic=self.data[f"3_cic"],
                cis=self.data[f"3_cis"],
                t_oc=self.data[f"1_t_oc"],
                a_0=self.data[f"1_a_0"],
                a_1=self.data[f"1_a_1"],
                a_2=self.data[f"1_a_2"],
                aode=self.data[f"1_aode"],
            )
        except KeyError as e:
            raise ValueError(f"Value {e} not yet parsed into data.")
        else:
            self._ephemeris = ephemeris
        finally:
            return None

    def _build_almanac(self) -> None:
        """Build BeiDou Almanac, hereby AmEpId is tested"""
        def _build_data_tuple(uuid_: int, prn_: int, health_key_: str) -> Tuple[int, int, int]:
            # get health value and return as tuple
            health_ = self.data[health_key_]
            return uuid_, prn_, health_

        # 1. Determine alm-prn 1 - 30 and check if expanded Almanac is broad_casted (AmEpId == 3)
        alm_uuids: List[Tuple[int, int, int]] = list()
        for prn, uuid in enumerate(range(97, 127)):
            prn += 1
            health_key = f"12{7 if prn < 20 else 8}_hea_{prn}"
            if all(k in self.data.keys() for k in (f"{uuid}_pre", health_key)):  # msg was received
                alm_uuids.append(_build_data_tuple(uuid_=uuid, prn_=prn, health_key_=health_key))

        # Assume that either All received msgs or None signalizes a valid/invalid AmpEpId
        use_amp_ep_id = all((self.data[f"{uuid[0]}_amp_ep_id"] == self.AmpEpId for uuid in alm_uuids))

        # 2. Check for extended alm-prn 31-63
        if use_amp_ep_id and alm_uuids:  # cross-check that at least one alm_uuid was received
            for page in range(11, 24):
                uuid_prefix = 5 * self.NofFrames + page
                for am_id in range(1, 4):
                    uuid_str = f"{uuid_prefix}{page}{am_id}"
                    if am_id == 3 and page > 18:  # skip reserved values
                        continue
                    else:
                        prn_modulo_health = 20 + page  # as svid in page 24 only 31-43
                        prn = prn_modulo_health + 13 * (am_id - 1)
                        health_key = f"{5 * self.NofFrames + 24}{24}{am_id}_hea_{prn_modulo_health}"
                    if all(k in self.data.keys() for k in (f"{uuid_str}_pre", health_key)):
                        alm_uuids.append(_build_data_tuple(uuid_=int(uuid_str), prn_=prn, health_key_=health_key))

        # 3. Build single Almanac
        for alm_uuid, prn_alm, health in alm_uuids:
            try:
                almanac = BdsAlmanac(
                    svid=prn_alm,
                    sv_health=health,
                    t_oa=self.data[f"{alm_uuid}_t_oa"],
                    sqrt_a=self.data[f"{alm_uuid}_sqrt_a"],
                    e=self.data[f"{alm_uuid}_e"],
                    omega=self.data[f"{alm_uuid}_omega"],
                    m_zero=self.data[f"{alm_uuid}_m_zero"],
                    omega_zero=self.data[f"{alm_uuid}_omega_zero"],
                    omega_dot=self.data[f"{alm_uuid}_omega_dot"],
                    delta_i=self.data[f"{alm_uuid}_delta_i"],
                    a_zero=self.data[f"{alm_uuid}_a_0"],
                    a_one=self.data[f"{alm_uuid}_a_1"],
                    am_id=self.data[f"{alm_uuid}_amp_ep_id" if prn_alm < 31 else f"{alm_uuid}_am_id"],
                )
            except KeyError as e:  # specific item was not received
                warnings.warn(f"{e} while trying to build BdsAlmanac D1-msg for {prn_alm}")
            else:
                self._almanac.update(
                    {prn_alm: almanac}
                )
        pass


class BdsD2NavMessage(BdsNavMessage):
    SV_IDs = set(range(1, 6)).union(set(range(59, 64)))

    def __init__(self, msg_name="D2 BeiDou Message", svid: int = -1):
        """Construct BeiDou D2 Navigation Message"""
        super(BdsD2NavMessage, self).__init__(msg_name=msg_name, svid=svid)
        pass

    def _build_ephemeris(self) -> None:
        pass

    def _build_almanac(self) -> None:
        pass

    @classmethod
    def get_uuid(cls, payload: Any) -> Any:
        pass

    @classmethod
    def uuid_to_payload(cls, uuid: int) -> PayloadBaseType:
        pass
