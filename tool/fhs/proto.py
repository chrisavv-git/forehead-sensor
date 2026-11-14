"""Wire and flash formats of the forehead sensor (mirror of firmware/core/format.h)."""
import struct
from dataclasses import dataclass

WINDOW_S = 0.5
BLOCK_SIZE = 4096
HEADER_SIZE = 48
RECORD_SIZE = 16
RECS_PER_BLOCK = 253
BLOCK_MAGIC = 0x31534846
FORMAT_VERSION = 3   # 2: the motor byte became the nudge byte; 1 is still read; 3: roll's sign fixed (1-2 flipped)
N_OPEN = 0xFFFF
NUDGE_SHADOW = 0x80
STEP_TEST = 5
STEP_CUE = 6    # the bedtime buzz when a session starts (not a nudge)
STEP_ZERO = 7   # the spoken zero talking (variant = the phrase; not a nudge)

F_UPRIGHT, F_USB, F_LOWBAT, F_TAP, F_ZERO, F_CLOCK, F_SHORT, F_NOMIC = 1, 2, 4, 8, 16, 32, 64, 128
FLAG_NAMES = {F_UPRIGHT: 'upright', F_USB: 'usb', F_LOWBAT: 'lowbat', F_TAP: 'tap', F_ZERO: 'zero',
              F_CLOCK: 'clock', F_SHORT: 'short', F_NOMIC: 'nomic'}

_REC = struct.Struct('<hhHHHHBBH')
_HDR = struct.Struct('<IIIIqH3hBBBBBBHH6x')
assert _REC.size == RECORD_SIZE and _HDR.size == HEADER_SIZE


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC16-CCITT (0x1021, init 0xFFFF), as the firmware computes it"""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Record:
    roll: float      # deg; 0 = face up, + = left ear down, +-180 = face down
    pitch: float     # deg; + = top of the head up, 90 = upright
    move: float      # deg/s: gyro magnitude RMS, or (watch, no gyro) how fast gravity's direction turns
    buzz: float      # mg, accel above 80 Hz, magnitude RMS
    mic: int         # mic RMS, raw PCM
    snore: int       # mic 100-600 Hz RMS, raw PCM
    motor: int       # the nudge step that ran in this window: 0 none, 1-4, 5 a test, 6 the bedtime buzz, 7 the spoken
                     # zero talking (0 in shadow mode unless a test, the bedtime buzz or the zero ran)
    would: int       # the step the therapy rules asked for (= motor in therapy mode; 0 for a bedtime buzz or the zero)
    variant: int     # 0-15: which vibration pattern + sound that step played
    shadow: bool
    flags: int
    seq16: int

    @classmethod
    def unpack(cls, b: bytes, version: int = FORMAT_VERSION) -> 'Record':
        roll, pitch, move, buzz, mic, snore, nudge, flags, seq16 = _REC.unpack(b)
        shadow = bool(nudge & NUDGE_SHADOW)
        if version == 1:   # bits 0-6 were a motor level 0-100 %: as a step (25 % a step), no variants
            step, variant = min(4, ((nudge & 0x7F) + 24) // 25), 0
        else:
            step, variant = (nudge >> 4) & 7, nudge & 15
        if version < 3:    # firmware before format 3 had + = right ear down
            roll = -roll if roll != -32768 else 32767
        return cls(roll / 100, pitch / 100, move / 10, buzz / 10, mic, snore, 0 if shadow else step,
                   0 if step in (STEP_CUE, STEP_ZERO) else step, variant,
                   shadow, flags, seq16)

    def flag_names(self):
        return [n for f, n in FLAG_NAMES.items() if self.flags & f]


def record_ok(raw: bytes, first_seq: int, i: int) -> bool:
    """a record slot really holds record first_seq+i (not erased or half-written flash)"""
    seq16 = raw[14] | raw[15] << 8
    return seq16 == (first_seq + i) & 0xFFFF and raw[12] != 0xFF


@dataclass
class Header:
    block_no: int
    boot_id: int
    first_seq: int
    t_seq0_ms: int   # unix ms of record 0 of this boot; 0 = clock not set
    batt_mv: int
    zero: tuple
    mode: int
    head_axis: int
    n_closed: int
    version: int

    @classmethod
    def unpack(cls, b: bytes):
        """None if this isn't a valid block header"""
        if len(b) < HEADER_SIZE:
            return None
        (magic, block_no, boot, first_seq, t0, batt, z0, z1, z2, ver, rsz, rpb, mode, axis, _hf, hcrc,
         n_closed) = _HDR.unpack(b[:HEADER_SIZE])
        if magic != BLOCK_MAGIC or not 1 <= ver <= FORMAT_VERSION or rsz != RECORD_SIZE or rpb != RECS_PER_BLOCK:
            return None
        if hcrc != crc16(b[:38]):
            return None
        return cls(block_no, boot, first_seq, t0, batt, (z0 / 1e4, z1 / 1e4, z2 / 1e4), mode, axis, n_closed, ver)


def block_records(block: bytes):
    """(header, [(seq, Record), ...]) of a 4 KB block; header None if invalid"""
    h = Header.unpack(block)
    if h is None:
        return None, []
    out = []
    n = h.n_closed if h.n_closed != N_OPEN and h.n_closed <= RECS_PER_BLOCK else RECS_PER_BLOCK
    for i in range(n):
        raw = block[HEADER_SIZE + i * RECORD_SIZE: HEADER_SIZE + (i + 1) * RECORD_SIZE]
        if not record_ok(raw, h.first_seq, i):
            break   # an open block (or one cut by power loss) ends at its first unwritten slot
        out.append((h.first_seq + i, Record.unpack(raw, h.version)))
    return h, out


def parse_kv(line: str) -> dict:
    """'S a=1 b=x ...' -> {'a': '1', 'b': 'x'}"""
    return dict(t.split('=', 1) for t in line.split()[1:] if '=' in t)


def parse_R(line: str):
    """'R <boot> <seq> <32 hex>' -> (boot, seq, Record) or None"""
    p = line.split()
    if len(p) != 4 or p[0] != 'R' or len(p[3]) != 32:
        return None
    try:
        raw = bytes.fromhex(p[3])
        boot, seq = int(p[1]), int(p[2])
    except ValueError:
        return None
    if raw[14] | raw[15] << 8 != seq & 0xFFFF:
        return None
    return boot, seq, Record.unpack(raw)
