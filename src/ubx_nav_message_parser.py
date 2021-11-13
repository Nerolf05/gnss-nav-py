from enum import IntEnum, auto
from typing import Dict, Tuple, List, Optional, ClassVar, Union
from pyubx2 import UBXMessage
from logging import getLogger
from pyubx2.pyubx_nav_msg_decoder.gnss_nav_message import \
    GnssNavMsgType, \
    GnssSvidType, \
    GnssNavMsgId, \
    LNavGpsMessage,\
    CNavGpsMessage,\
    GpsNavMessage,\
    GnssNavMessage,\
    GlonNavMessage, \
    INavGalMessage, \
    BdsD1NavMessage, \
    BdsD2NavMessage, \
    NavMsgError, \
    UuidType

from pyubx2.pyubx_nav_msg_decoder.int_bit_manipulator import \
    shift_mask_int, \
    int_append_int

from pyubx2.pyubx_nav_msg_decoder.ubx_base_types import \
    UbxGnssId, \
    UbxSignalId, \
    UbxTypeSvid

"""
TODOs:
    * remove requirement to read all ubx-rxm-sfrbx pages in advance -> filtering should be placed outside
    * more advanced error msgs than (Ubx)NavMsgError
"""


class UbxNavDecodeError(Exception):
    """
    Ubx Navigation Message decoding error.
    """


class NavMsgDecodingError(Exception):
    """
    Navigation Message decoding error.
    """


class UbxNavMessageParserHandle:
    """
    Manages parsing of ubx-rxm-sfrbx into corresponding GNSS-Navigation message.
    """

    UbxSignalGnssNavMsgMap: ClassVar[Dict[UbxGnssId, Dict[int, GnssNavMsgId]]] = {
        # TODO CNAV vs. CNAV2 not yet considerd (based on svid)
        UbxGnssId.GPS: {0: GnssNavMsgId.GPS_LNAV, 3: GnssNavMsgId.GPS_CNAV, 4: GnssNavMsgId.GPS_CNAV},
        UbxGnssId.GLONASS: {0: GnssNavMsgId.GLON_NAV, 2: GnssNavMsgId.GLON_NAV},
        UbxGnssId.BeiDou: {0: GnssNavMsgId.BEID_D1, 1: GnssNavMsgId.BEID_D2, 2: GnssNavMsgId.BEID_D1,
                           3: GnssNavMsgId.BEID_D2},
        UbxGnssId.Galileo: {0: GnssNavMsgId.GAL_INAV, 1: GnssNavMsgId.GAL_INAV, 5: GnssNavMsgId.GAL_INAV,
                            6: GnssNavMsgId.GAL_INAV},
    }

    def __init__(self):
        self._raw_data: Dict = {}
        self.gnss_msgs: Dict = {}
        self.nav_msg_parser: NavMessageParser = NavMessageParser()
        self.logger = getLogger()
        # Should be changed
        self._static_galileo_alm_helper: Dict = {
            1: {},  # "E1B"
            5: {},  # "E5b"
        }
        pass

    def add_subframe(self, ubx_msg: UBXMessage) -> bool:
        """
        Extract raw data from UBXMessage and stores it.
        :param ubx_msg: UBX-RXM-SFRBX
        :return: bool
        """

        # TODO store GnssNavmsg and update instead of

        if isinstance(ubx_msg, UBXMessage) and ubx_msg.identity == "RXM-SFRBX":
            gnss_id = getattr(ubx_msg, "gnssId", None)
            sv_id = getattr(ubx_msg, "svId")

            # ubx_raw_nav_data_id is used as uuid to identify pages of interest
            prep_ubx = self.preprocess_ubx(ubx_msg=ubx_msg, gnss_id=gnss_id, sv_id=sv_id)
            if prep_ubx:
                nav_msg_id, ubx_raw_nav_data_id, raw_nav_data = prep_ubx
            else:
                return False
            data = {ubx_raw_nav_data_id: raw_nav_data}

            # add data to dict for debug only
            if gnss_id in self._raw_data.keys():
                if sv_id in self._raw_data[gnss_id].keys():
                    if nav_msg_id in self._raw_data[gnss_id][sv_id].keys():
                        self._raw_data[gnss_id][sv_id][nav_msg_id].update(data)  # old pages are replaced by new ones
                    else:
                        self._raw_data[gnss_id][sv_id].update({nav_msg_id: data})
                        self.gnss_msgs[gnss_id][sv_id].update({nav_msg_id: self.nav_msg_parser.determine_nav_msg(
                            nav_msg_id=nav_msg_id, svid=sv_id
                        )})
                else:
                    self._raw_data[gnss_id].update({sv_id: {nav_msg_id: data}})
                    self.gnss_msgs[gnss_id].update({sv_id: {nav_msg_id: self.nav_msg_parser.determine_nav_msg(
                        nav_msg_id=nav_msg_id, svid=sv_id
                    )}})
            else:
                self._raw_data.update({gnss_id: {sv_id: {nav_msg_id: data}}})
                self.gnss_msgs.update({gnss_id: {sv_id: {nav_msg_id: self.nav_msg_parser.determine_nav_msg(
                    nav_msg_id=nav_msg_id, svid=sv_id
                )}}})
            # Add subframe to gnss_nav_msg
            self.nav_msg_parser.decode_subframe(
                gnss_nav_msg=self.gnss_msgs[gnss_id][sv_id][nav_msg_id], uuid=ubx_raw_nav_data_id, payload=raw_nav_data
            )

            return True
        else:
            return False

    def preprocess_ubx(self, ubx_msg: UBXMessage, gnss_id: UbxGnssId, sv_id: UbxTypeSvid
                       ) -> Optional[Tuple[GnssNavMsgId, UuidType, Union[bytes, int]]]:
        """
        Lift raw-navigation bytes from ubx-rawx-sfrbx msg and identify nav_msg
        :param ubx_msg:     UBXMessage defined by pyubx2-lib
        :param gnss_id:     Gnss-sys identifier
        :param sv_id:       Satellite vehicle identifier
        :return: (GnssNavMsgId, Uuid-frame, raw data or None
        """
        result = None
        try:
            if gnss_id == UbxGnssId.GPS:
                nav_msg_id, ubx_raw_nav_data_id, raw_data = self._preprocess_ubx_gps(ubx_msg=ubx_msg)
            elif gnss_id == UbxGnssId.GLONASS:
                nav_msg_id, ubx_raw_nav_data_id, raw_data = self._preprocess_ubx_glon(ubx_msg=ubx_msg)
            elif gnss_id == UbxGnssId.Galileo:
                nav_msg_id, ubx_raw_nav_data_id, raw_data = self._preprocess_ubx_gal(ubx_msg=ubx_msg)
            elif gnss_id == UbxGnssId.BeiDou:
                nav_msg_id, ubx_raw_nav_data_id, raw_data = self._preprocess_ubx_bds(ubx_msg=ubx_msg)
            else:
                raise NotImplementedError(f"GNSS: {gnss_id} not implemented")
        except (UbxNavDecodeError, NotImplementedError, Exception) as e:
            self.logger.warning(f"UbxNavDecoding error: {e} while trying to preprocess: {gnss_id}.")
        else:
            if nav_msg_id != GnssNavMsgId.UNDEFINED:
                result = nav_msg_id, ubx_raw_nav_data_id, raw_data
            else:
                self.logger.warning(f"Passed sivd: {sv_id} !NavMsg could not be identified")
        finally:
            return result

    def _preprocess_ubx_gal(self, ubx_msg: UBXMessage) -> Tuple[GnssNavMsgId, int, int]:
        """
        Lift raw-data from I/Nav Galileo message (Note right now F/Nav could not be tracked by ubx ZED-F9P.
        IMPORTANT:
            * despite freqId should indicate SignalId, ublox F9P does not seem to differentiate between E1B & E5b
            * by reverse engineering (checking word-types of sub-frame layout i.e. alms): numWords => 8: E5b-I; 9: E1B
            * not documented by Ublox integration manual
        :param ubx_msg: UBX-RXM-SFRBX
        :return: (NavMsgId, uuid_page, data_payload as int, prn number
        """
        nav_msg_id: GnssNavMsgId = GnssNavMsgId.UNDEFINED
        uuid_data: int = -1
        raw_data: int = 0
        freq_id: int = getattr(ubx_msg, "freqId", -1)
        n_words: int = getattr(ubx_msg, "numWords", 0)
        sv_id: int = getattr(ubx_msg, "svId", 0)

        # Derived from subframe - layout (not documented)
        if n_words == 8:
            freq_id = 5  # E5b-I
        elif n_words == 9:
            freq_id = 1  # E1B

        # no F/Nav differentiation as not tracked F9P - but there already exists a ubx-prototype with bands to track L5 band
        if freq_id not in UbxSignalId[getattr(ubx_msg, "gnssId")].keys():
            raise UbxNavDecodeError(f"Galileo message cannot be decoded")

        words = self._ubx_rxm_sfrbx_dwrds(ubx_msg=ubx_msg)

        words = words[:8]  # trim to constant length of 8 words
        # Extract odd/even and page bits
        odd_1, odd_2 = shift_mask_int(words[0], 31, 1), shift_mask_int(int_=words[4], r_shift=31, bits_oi=1)
        p_t_1, p_t_2 = shift_mask_int(words[0], 30, 1), shift_mask_int(int_=words[4], r_shift=30, bits_oi=1)
        if p_t_1 == 1 or p_t_2 == 1:  # nominal/alert page
            raise UbxNavDecodeError(f"Skip alert page Galileo")
        elif odd_1 != 0 or odd_2 != 1:  # Note vertical page layout between E5b-I & E1-B not considered by ubx-manual
            raise UbxNavDecodeError(f"Odd-even text failed")
        else:  # extract word - note CRC & SAR wont be used
            word, _, _ = self._sfrbx_lift_gal_inav(dwrds=words)

            try:  # Get latest alm svid of current sat and correct freq_id
                alm_svid = self._static_galileo_alm_helper[freq_id][sv_id]
            except KeyError as e:
                alm_svid = 0

            # extract unique word type to differentiate different alm
            word_type, iod, sf_id, alm_svid = INavGalMessage.get_uuid(
                payload={"word": word, "freq_id": freq_id, "alm_sv_id": alm_svid}
            )
            if word_type > 0:  # store only relevant data
                if word_type in (7, 8, 9, 10):  # alm with sub frame-id
                    if sf_id > 0 and alm_svid > 0:  # process only valid words -> iod == iod_a: {1-15}
                        self._static_galileo_alm_helper[freq_id].update({sv_id: alm_svid})
                        # pack multiple info into uuid to simplify unpacking: based on iod_a and sub_frame-id
                        uuid_data = int(f"{word_type}{str(iod).zfill(2)}{str(sf_id).zfill(2)}")
                        assert 10001 < uuid_data <= 201524, f"Invalid I/Nav uuid: {uuid_data}"
                    else:  # discard invalid words
                        uuid_data = -1
                else:
                    uuid_data = word_type

                if uuid_data > 0:
                    nav_msg_id = GnssNavMsgId.GAL_INAV
                    raw_data = word

        return nav_msg_id, uuid_data, raw_data

    def _preprocess_ubx_gps(self, ubx_msg: UBXMessage) -> Tuple[GnssNavMsgId, int, int]:
        """
        Lift raw-data from C/L-Nav GPS message -> either C or L-Nav message
        :param ubx_msg: UBX-RXM-SFRBX
        :return: (NavMsgId, uuid_page, data_payload as int, prn number
        """
        if getattr(ubx_msg, "numWords", 0) != 10:
            raise UbxNavDecodeError(f"Number of Words != 10")
        nav_msg_id: GnssNavMsgId = GnssNavMsgId.UNDEFINED
        uuid_data: int = -1
        raw_data: int = 0

        data = self._ubx_rxm_sfrbx_dwrds(ubx_msg=ubx_msg)

        # L-Nav
        if shift_mask_int(int_=data[0], r_shift=22, bits_oi=8) == LNavGpsMessage.Preamble:  # L/NAV
            for idx, word in enumerate(data):
                raw_data = int_append_int(int_a=raw_data, int_b=word, l_shift_a=30, bits_oi_b=30)

            try:
                uuid = LNavGpsMessage.get_uuid(payload=raw_data)
            except NavMsgError as e:
                raise UbxNavDecodeError(f"Lookup LNAV-pages failed: {e}")

            # check if uuid of interest otherwise skip subframe
            if uuid in LNavGpsMessage.uuid_pages:  # TODO check if sv_only
                nav_msg_id = GnssNavMsgId.GPS_LNAV
                uuid_data = uuid
            else:
                raise UbxNavDecodeError(f"LNAV-uuid: {uuid} not in pages")

        # C-Nav
        elif shift_mask_int(int_=data[0], r_shift=24, bits_oi=8) == CNavGpsMessage.Preamble:  # C/NAV
            for word in data[:-1]:
                raw_data = int_append_int(int_a=raw_data, int_b=word, l_shift_a=32)
            raw_data = int_append_int(int_a=raw_data, int_b=data[-1], l_shift_a=12)

            try:
                msg_type_id, prn_midi, prns_red, prn = CNavGpsMessage.get_uuid(payload=raw_data)
            except NavMsgError as e:
                raise UbxNavDecodeError(f"Lookup CNAV-pages failed: {e}")

            if msg_type_id in CNavGpsMessage.uuid_msg_types:
                allowed, discard = set(), set()

                for _, s in CNavGpsMessage.uuid_dict.items():
                    if s not in discard:
                        allowed.update(s)

                if msg_type_id in allowed:
                    nav_msg_id = GnssNavMsgId.GPS_CNAV
                    if prn_midi:
                        uuid_data = msg_type_id * 10 + prn_midi
                    elif prns_red:  # build integer containing all red_svs separated with 0s
                        uuid_data = int(f"{msg_type_id}0{'0'.join(map(str, prns_red))}")
                    else:
                        uuid_data = msg_type_id
                else:
                    self.logger.error(f"Discard: {msg_type_id}")
                    raise UbxNavDecodeError(f"CNAV {msg_type_id} not in allowed ones.")
            else:
                raise UbxNavDecodeError(f"CNav - discard {msg_type_id}")
        else:
            raise UbxNavDecodeError(f"GPS-Preamble {GpsNavMessage.Preamble} was not found for C/L-Nav")

        return nav_msg_id, uuid_data, raw_data

    def _preprocess_ubx_glon(self, ubx_msg: UBXMessage) -> Tuple[GnssNavMsgId, int, int]:
        """Lift ublox-glon Nav_msg_id, string_id and payload (discard unknown sv=255)"""
        data = self._ubx_rxm_sfrbx_dwrds(ubx_msg=ubx_msg)
        string, str_id, sf_nr, f_nr = self._sfrbx_lift_glon_nav(dwrds=data)
        uuid = GlonNavMessage.get_uuid(payload=(f_nr, str_id))

        if uuid in GlonNavMessage.uuid and getattr(ubx_msg, "svId", 255) != 255:
            str_id = uuid
            nav_msg_id: GnssNavMsgId = GnssNavMsgId.GLON_NAV
        else:
            str_id = -1
            nav_msg_id = GnssNavMsgId.UNDEFINED

        return nav_msg_id, str_id, string

    def _preprocess_ubx_bds(self, ubx_msg: UBXMessage) -> Tuple[GnssNavMsgId, int, int]:
        """Determine navigation message type and extract subframe from ubx-rxm-sfrbx message"""
        data = self._ubx_rxm_sfrbx_dwrds(ubx_msg=ubx_msg)
        svid = getattr(ubx_msg, "svId", 0)
        sub_frame = self._sfrbx_lift_bds_nav(dwrds=data)
        uuid = 0
        nav_msg_id = GnssNavMsgId.UNDEFINED
        if svid in BdsD1NavMessage.SV_IDs:  # meo/igso satellite
            uuid = BdsD1NavMessage.get_uuid(payload=sub_frame)
            if uuid > 0:
                nav_msg_id = GnssNavMsgId.BEID_D1
        elif svid in BdsD2NavMessage.SV_IDs:  # Geo satellite
            pass
            uuid = BdsD2NavMessage.get_uuid(payload=sub_frame)
            nav_msg_id = GnssNavMsgId.BEID_D2
        else:
            pass
            # raise NotImplementedError(f"BeiDou preprocessing ubx-rxm-sfrbx")
        return nav_msg_id, uuid, sub_frame

    @staticmethod
    def _sfrbx_lift_gal_inav(dwrds: List[int]) -> Tuple[int, int, int]:
        """Extract word, SAR and CRC from dwrds of ubx-rxm-sfrbx galileo I/NAV msg. Note SAR not defined for E5 BI/Q"""
        word, sar, crc = 0, 0, 0
        for idx, dwrd in enumerate(dwrds):
            if idx == 0:
                word = shift_mask_int(int_=dwrd, r_shift=0, bits_oi=30)
            elif idx in (1, 2):
                word = int_append_int(int_a=word, int_b=dwrd, l_shift_a=32, bits_oi_b=32)
            elif idx == 3:
                mod_dwrd = shift_mask_int(int_=dwrd, r_shift=14, bits_oi=18)
                word = int_append_int(int_a=word, int_b=mod_dwrd, l_shift_a=18)
            elif idx == 4:
                mod_dwrd = shift_mask_int(int_=dwrd, r_shift=14, bits_oi=16)
                word = int_append_int(int_a=word, int_b=mod_dwrd, l_shift_a=16, bits_oi_b=16)
            elif idx == 5:
                sar = shift_mask_int(int_=dwrd, r_shift=0, bits_oi=6)
            elif idx == 6:
                mod_dwrd = shift_mask_int(int_=dwrd, r_shift=16, bits_oi=16)
                sar = int_append_int(int_a=sar, int_b=mod_dwrd, l_shift_a=16, bits_oi_b=16)
                crc = shift_mask_int(int_=dwrd, r_shift=0, bits_oi=14)
            elif idx == 7:
                mod_dwrd = shift_mask_int(int_=dwrd, r_shift=22, bits_oi=10)
                crc = int_append_int(int_a=crc, int_b=mod_dwrd, l_shift_a=10, bits_oi_b=10)

        return word, sar, crc

    @staticmethod
    def _sfrbx_lift_glon_nav(dwrds: List[int]) -> Tuple[int, int, int, int]:
        """Extract string, string_nr, super_frame_nr and frame_nr"""
        string, string_nr, sf_nr, f_nr = 0, 0, 0, 0
        for idx, word in enumerate(dwrds):
            if idx in (0, 1):
                string = int_append_int(int_a=string, int_b=word, l_shift_a=32, bits_oi_b=32)
                if idx == 0:
                    string_nr = shift_mask_int(int_=word, r_shift=27, bits_oi=4)
            elif idx == 2:
                string = int_append_int(int_a=string, int_b=word, l_shift_a=11, bits_oi_b=21)
            elif idx == 3 and word != 0:  # note this word is deduced by receiver and thus not always available
                sf_nr = shift_mask_int(int_=word, r_shift=16, bits_oi=16)
                f_nr = shift_mask_int(int_=word, r_shift=0, bits_oi=8)

        return string, string_nr, sf_nr, f_nr

    @staticmethod
    def _sfrbx_lift_bds_nav(dwrds: List[int]) -> int:
        """Extract sub_frame data"""
        sub_frame = 0
        for idx, dwrd in enumerate(dwrds):
            sub_frame = int_append_int(int_a=sub_frame, int_b=dwrd, l_shift_a=30, bits_oi_b=30)
        return sub_frame

    @staticmethod
    def _ubx_rxm_sfrbx_dwrds(ubx_msg: UBXMessage) -> List[int]:
        """Extract all dwrds from ubx-rxm-sfrbx UBXMessage as a list."""
        data = []
        try:
            data = [getattr(ubx_msg, key, None)
                    for key in (f"dwrd_{str(v).zfill(2)}" for v in range(1, getattr(ubx_msg, "numWords") + 1))
                    if key in dir(ubx_msg)]
        except KeyError as _:
            pass
        finally:
            return data

    def decode_nav_msgs(self) -> Dict[UbxGnssId, Dict[UbxTypeSvid, Dict[GnssNavMsgId, GnssNavMessage]]]:
        """
        Decode navigation message if all necessary data is available
        :return: Dict[UbxGnssId, Dict[UbxTypeSvid, Dict[GnssNavMsgId, GnssNavMessage]]]
        """
        decoded_msgs: Dict[UbxGnssId, Dict[UbxTypeSvid, Dict[GnssNavMsgId, GnssNavMessage]]] = {}
        try:
            for gnss in self._raw_data.keys():
                for svid in self._raw_data[gnss].keys():
                    for nav_msg_id in self._raw_data[gnss][svid].keys():
                        decoded_msg: GnssNavMessage = self.decode_nav_msg(
                            gnss_id=gnss, sv_id=svid, nav_msg_id=nav_msg_id)
                        if gnss not in decoded_msgs.keys():
                            decoded_msgs.update({gnss: {svid: {nav_msg_id: decoded_msg}}})
                        elif svid not in decoded_msgs[gnss].keys():
                            decoded_msgs[gnss].update({svid: {nav_msg_id: decoded_msg}})
                        else:
                            decoded_msgs[gnss][svid].update({nav_msg_id: decoded_msg})
        except KeyError as e:
            self.logger.warning(f"{e}")
        except UbxNavDecodeError as e:
            self.logger.warning(f"Navigation-msg decode-error: {e}")
        finally:
            return decoded_msgs

    def decode_nav_msg(self, gnss_id: UbxGnssId, sv_id: UbxTypeSvid, nav_msg_id: GnssNavMsgId) -> GnssNavMessage:
        """NavMessageParser decode message with GnssNavMsgId."""
        try:
            uuids = list()
            payloads = list()
            for uuid in self._raw_data[gnss_id][sv_id][nav_msg_id].keys():
                uuids.append(uuid)
                payloads.append(self._raw_data[gnss_id][sv_id][nav_msg_id][uuid])
        except KeyError as e:
            raise ValueError(f"Key-Tuple: {gnss_id}|{sv_id}|{nav_msg_id} - {e}")
        else:
            try:
                msg: GnssNavMessage = self.nav_msg_parser.decode_nav_msg(
                    nav_msg_id=nav_msg_id, svid=sv_id, uuids=uuids, payloads=payloads
                )
            except NavMsgDecodingError as e:
                raise ValueError(f"Key-Tuple: {gnss_id}|{nav_msg_id} not supported {self.UbxSignalGnssNavMsgMap} ({e})")
        return msg


class NavMessageParser:
    """
    GNSS-Navigation message parser which decodes raw data into corresponding navigation message.
    Requires only raw nav-msg data. Is receiver independent and works on actual raw nav-data payload.
    """
    def __init__(self):
        self.byteorder = "little"  # protocol is implemented in "little-endian"
        pass

    @staticmethod
    def determine_nav_msg(nav_msg_id: GnssNavMsgType, svid: GnssSvidType) -> GnssNavMessage:
        """Adapter to create empty GnssNavMsg object which will be filled later subframe by subframe."""

        if nav_msg_id == GnssNavMsgId.GPS_LNAV:
            msg: GnssNavMessage = LNavGpsMessage(svid=svid)
        elif nav_msg_id == GnssNavMsgId.GPS_CNAV:
            msg = CNavGpsMessage(svid=svid)
        elif nav_msg_id == GnssNavMsgId.GAL_INAV:
            msg = INavGalMessage(svid=svid)
        elif nav_msg_id == GnssNavMsgId.GLON_NAV:
            msg = GlonNavMessage(svid=svid)
        elif nav_msg_id == GnssNavMsgId.BEID_D1:
            msg = BdsD1NavMessage(svid=svid)
        elif nav_msg_id == GnssNavMsgId.BEID_D2:
            msg = BdsD2NavMessage(svid=svid)
        else:
            raise NotImplementedError(f"Message: {nav_msg_id} not yet implemented.")
        return msg

    @staticmethod
    def decode_subframe(gnss_nav_msg: GnssNavMessage, uuid: int, payload: int) -> None:
        """
        Add subframe to  GNSS-Navigation Message based on passed payload and identifiers.
        :param gnss_nav_msg: GnssNavMsg onto which subframe is added
        :param uuid: Identifying specific page|word|frame of nav message
        :param payload: raw data of page|word|frame
        :return: None
        """
        try:
            gnss_nav_msg.add_frame(uuid=uuid, payload=payload)
        except (NavMsgError, Exception) as e:
            raise NavMsgDecodingError(e)
        finally:
            return None

    def decode_nav_msg(self, nav_msg_id: GnssNavMsgType, svid: GnssSvidType, uuids: List[int], payloads: List[int]
                       ) -> GnssNavMessage:
        # legacy method -> remove
        if nav_msg_id == GnssNavMsgId.GPS_LNAV:
            msg: GnssNavMessage = LNavGpsMessage(svid=svid)
        elif nav_msg_id == GnssNavMsgId.GPS_CNAV:
            msg = CNavGpsMessage(svid=svid)
        elif nav_msg_id == GnssNavMsgId.GAL_INAV:
            msg = INavGalMessage(svid=svid)
        elif nav_msg_id == GnssNavMsgId.GLON_NAV:
            msg = GlonNavMessage(svid=svid)
        elif nav_msg_id == GnssNavMsgId.BEID_D1:
            msg = BdsD1NavMessage(svid=svid)
        elif nav_msg_id == GnssNavMsgId.BEID_D2:
            msg = BdsD2NavMessage(svid=svid)
        else:
            raise NotImplementedError(f"Message: {nav_msg_id} not yet implemented.")

        try:
            for uuid, payload in zip(uuids, payloads):
                msg.add_frame(uuid=uuid, payload=payload)
            e = msg.ephemeris
            a = msg.almanac
        except (NavMsgError, Exception) as e:
            print(f"Error {e} occurred")
        else:
            print(f"Build {msg} with {len(msg.data)} items.")
        finally:
            return msg

    # TODO move parsing to this class - such that GnssNavMsg holds only data and data related member but no msgitem
