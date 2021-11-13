from enum import IntEnum


class UbxGnssId(IntEnum):
    GPS = 0
    SBAS = 1
    Galileo = 2
    BeiDou = 3
    IMES = 4
    QZSS = 5
    GLONASS = 6


UbxSignalId = {
    UbxGnssId.GPS: {0: "GPS L1C/A", 3: "GPS L2 CL", 4: "GPS L2 CM"},
    UbxGnssId.SBAS: {0: "SBAS L1C/A"},
    UbxGnssId.Galileo: {0: "Galileo E1 C", 1: "Galileo E1 B", 5: "Galileo E5 bI", 6: "Galileo E5 bQ"},
    UbxGnssId.BeiDou: {0: "BeiDou B1I D1", 1: "BeiDou B1I D2", 2: "BeiDou B2I D1", 3: "BeiDou B2I D2"},
    UbxGnssId.QZSS: {0: "QZSS L1C/A", 1: "QZSS L1S", 4: "QZSS L2 CM", 5: "QZSS L2 CL"},
    UbxGnssId.GLONASS: {0: "GLONASS L1 OF", 2: "GLONASS L2 OF"},
}

UbxTypeSignalId = int
UbxTypeSvid = int
UbxTypeRawNavDataId = int

