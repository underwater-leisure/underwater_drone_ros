import struct
from binascii import hexlify, unhexlify
from dataclasses import dataclass
from typing import List

# IDs
# 0x01: 인식모듈 통합 SW
# 0x02: 통합모듈 통합 SW
# 0x03: 젠티 알고리즘
# 0x04: 공주대 알고리즘

SOT = 0xABABABAB
EOT = 0xAABBAABB

RECOG_ID = 0x01
UNI_ID = 0x02
JENTI_ID = 0x03
KONGJU_ID = 0x04

RECOG_TO_JENTI_MSG_SIZE = 168
RECOG_TO_JENTI_CMD = 0x03

JENTI_TO_RECOG_MSG_SIZE = 784
JENTI_TO_RECOG_CMD = 0x04


@dataclass
class RecogToJentiData:
    sot: int = SOT  # unsigned int 4-byte
    sender_id: int = RECOG_ID  # B: unsigned char 1-byte
    receiver_id: int = JENTI_ID  # B: unsigned char 1-byte
    cmd: int = RECOG_TO_JENTI_CMD  # B: unsigned char 1-byte
    spare: int = 0x00  # B: unsigned char 1-byte
    size: int = RECOG_TO_JENTI_MSG_SIZE  # I: unsigned int 4-byte

    # AUV xyz & roll, pitch, yaw. double 8-byte
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # AUV velocity & angular velocity. double 8-byte
    u: float = 0.0
    v: float = 0.0
    w: float = 0.0
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0

    # AUV acceleration & angular acceleration. double 8-byte
    du: float = 0.0
    dv: float = 0.0
    dw: float = 0.0
    dp: float = 0.0
    dq: float = 0.0
    dr: float = 0.0

    mission_state: int = 0  # I: unsigned int 4-byte
    command_state: int = 0  # I: unsigned int 4-byte

    eot: int = EOT  # I: unsigned int 4-byte

    @classmethod
    def from_bytes(cls, hex_bytes):
        try:
            assert (len(hex_bytes) == RECOG_TO_JENTI_MSG_SIZE)

            d = struct.unpack('<I4BI18d3I', hex_bytes)

            assert (d[0] == SOT)
            assert (d[-1] == EOT)
            assert (d[1] == RECOG_ID)
            assert (d[2] == JENTI_ID)
            assert (d[5] == RECOG_TO_JENTI_MSG_SIZE)

            return cls(*d)
        except:  # noqa: E722
            return None

    @classmethod
    def from_str(cls, s: str):
        hex_bytes = unhexlify(s)
        return cls.from_bytes(hex_bytes)

    def to_dict(self) -> dict:
        dic = vars(self)
        return dic

    def to_bytes(self) -> bytes:
        all_dict = vars(self)
        hex_bytes = struct.pack('<I4BI18d3I',
                                *all_dict.values())
        return hex_bytes

    def to_str(self) -> str:
        return hexlify(self.to_bytes()).upper().decode()


@dataclass
class JentiToRecogData:
    sot: int = SOT
    sender_id: int = JENTI_ID
    receiver_id: int = RECOG_ID
    cmd: int = JENTI_TO_RECOG_CMD
    spare: int = 0
    size: int = JENTI_TO_RECOG_MSG_SIZE
    diver_positions: List = None
    eot: int = EOT

    @classmethod
    def from_bytes(cls, hex_bytes: bytes):
        try:
            assert (len(hex_bytes) == JENTI_TO_RECOG_MSG_SIZE)
            d = struct.unpack('<I4BI96dI', hex_bytes)
            positions = [[d[6 + jj + 6 * ii]
                          for jj in range(0, 6)] for ii in range(0, 16)]

            assert (d[0] == SOT)
            assert (d[-1] == EOT)

            assert (d[1] == JENTI_ID)
            assert (d[2] == RECOG_ID)
            assert (d[5] == JENTI_TO_RECOG_MSG_SIZE)

            return cls(sot=d[0], sender_id=d[1], receiver_id=d[2], cmd=d[3], spare=d[4], size=d[5],
                       diver_positions=positions, eot=d[-1])
        except:  # noqa: E722
            return None

    @classmethod
    def from_str(cls, s: str):
        hex_bytes = unhexlify(s)
        return cls.from_bytes(hex_bytes)

    def to_dict(self) -> dict:
        dic = vars(self)
        return dic

    def to_bytes(self) -> bytes:
        try:
            positions = sum(self.diver_positions, [])  # flatten
            assert (len(positions) == 96)
            return struct.pack('<I4BI96dI', self.sot, self.sender_id, self.receiver_id, self.cmd, self.spare, self.size,
                               *positions, self.eot)
        except:  # noqa: E722
            return None

    def to_str(self) -> str:
        return hexlify(self.to_bytes()).upper().decode()
