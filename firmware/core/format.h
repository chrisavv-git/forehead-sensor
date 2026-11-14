// On-flash and on-wire data format. Shared by the firmware, the host simulator and (mirrored) tool/fhs/proto.py.
// Everything is little-endian, as on the ESP32-S3 and the laptop.
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace fhs {

constexpr uint32_t WINDOW_MS = 500;          // one record per 0.5 s
constexpr uint32_t BLOCK_SIZE = 4096;        // one flash sector
constexpr uint32_t HEADER_SIZE = 48;
constexpr uint32_t RECORD_SIZE = 16;
constexpr uint32_t RECS_PER_BLOCK = (BLOCK_SIZE - HEADER_SIZE) / RECORD_SIZE;   // 253
constexpr uint32_t BLOCK_MAGIC = 0x31534846;  // "FHS1"
constexpr uint8_t FORMAT_VERSION = 3;         // 2: the record's motor byte became the nudge byte (1 is still read);
                                              // 3: roll's sign fixed (1-2 had + = RIGHT ear down; the tool flips them)
constexpr uint16_t N_OPEN = 0xFFFF;           // n_closed of a block that was never closed (still open, or power was lost)

// one 0.5 s window
#pragma pack(push, 1)
struct Record {
  int16_t roll;     // 0.01 deg. 0 = face up; + = left ear down (lying on the left side); +-180 = face down
  int16_t pitch;    // 0.01 deg. + = top of the head up (sitting up), 90 = upright
  uint16_t move;    // head movement, 0.1 deg/s: gyro magnitude RMS, or (no gyro) how fast gravity's direction turns
  uint16_t buzz;    // fast vibration: accel high-passed above 80 Hz, magnitude RMS, 0.1 mg
  uint16_t mic;     // mic loudness: RMS, raw PCM units
  uint16_t snore;   // mic 100-600 Hz band RMS, raw PCM units
  uint8_t nudge;    // bits 4-6: step (0 none, 1-4 therapy steps, 5 a test, 6 the bedtime buzz: variant 0 on duty,
                    // 1 battery low); bits 0-3: variant; bit 7 set = shadow
                    // (what therapy would do, nothing ran). Format 1: bits 0-6 were the motor level 0-100 %
  uint8_t flags;    // F_* below
  uint16_t seq;     // low 16 bits of the record number in this boot (checks a record really was written)
};

// first 48 bytes of each block; records follow at offset 48 (16-byte aligned, so none crosses a 256-byte page)
struct BlockHeader {
  uint32_t magic;
  uint32_t block_no;   // counts up forever: the newest block has the largest
  uint32_t boot_id;    // counts up on every power-up
  uint32_t first_seq;  // record number (in this boot) of the block's first record
  int64_t t_seq0_ms;   // unix ms of record 0 of this boot; 0 = the clock was not set (yet) in this boot
  uint16_t batt_mv;
  int16_t zero[3];     // the angle zero: gravity in board axes when lying flat on the back, 1/10000 g
  uint8_t version;
  uint8_t rec_size;
  uint8_t rec_per_block;
  uint8_t mode;        // 0 shadow, 1 therapy
  uint8_t head_axis;   // board axis that points to the top of the head: 0 +X, 1 -X, 2 +Y, 3 -Y
  uint8_t hflags;      // reserved, 0xFF
  uint16_t hcrc;       // CRC16-CCITT of bytes 0-37
  uint16_t n_closed;   // record count, written when the block is closed; N_OPEN if it never was
  uint8_t reserved[6];
};
#pragma pack(pop)

static_assert(sizeof(Record) == RECORD_SIZE, "record must be 16 bytes");
static_assert(sizeof(BlockHeader) == HEADER_SIZE, "header must be 48 bytes");
static_assert(offsetof(BlockHeader, hcrc) == 38, "hcrc covers bytes 0-37");
static_assert(HEADER_SIZE + RECS_PER_BLOCK * RECORD_SIZE == BLOCK_SIZE, "a block fills its sector");

enum : uint8_t {
  F_UPRIGHT = 1,     // pitch above upright_deg (sitting or standing)
  F_USB = 2,         // on USB power (charger or laptop)
  F_LOWBAT = 4,      // battery below low_mv
  F_TAP = 8,         // the button asked for the spoken zero in this window (a double press)
  F_ZERO = 16,       // the angle zero was set in this window
  F_CLOCK = 32,      // the clock is set in this boot
  F_SHORT = 64,      // fewer motion samples than expected (angles less sure; none at all = roll/pitch kept from before)
  F_NOMIC = 128,     // no mic samples in this window
};
constexpr uint8_t NUDGE_SHADOW = 0x80;
constexpr uint8_t STEP_TEST = 5;
constexpr uint8_t STEP_CUE = 6;    // the bedtime buzz when a session starts (not a nudge)
constexpr uint8_t STEP_ZERO = 7;   // the spoken zero talking (variant = the phrase, zeroguide.h; not a nudge)
inline uint8_t nudge_step(uint8_t n) { return (n >> 4) & 7; }
inline uint8_t nudge_variant(uint8_t n) { return n & 15; }

inline uint16_t crc16(const void* data, size_t n, uint16_t crc = 0xFFFF) {   // CRC16-CCITT (0x1021), init 0xFFFF
  const uint8_t* p = static_cast<const uint8_t*>(data);
  while (n--) {
    crc ^= uint16_t(*p++) << 8;
    for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}

inline bool header_ok(const BlockHeader& h) {
  return h.magic == BLOCK_MAGIC && h.version >= 1 && h.version <= FORMAT_VERSION && h.rec_size == RECORD_SIZE &&
         h.rec_per_block == RECS_PER_BLOCK && h.hcrc == crc16(&h, offsetof(BlockHeader, hcrc));
}

// a record slot really holds record (first_seq + i): the seq matches and the nudge byte isn't erased flash
inline bool record_ok(const Record& r, uint32_t first_seq, uint32_t i) {
  return r.seq == uint16_t(first_seq + i) && r.nudge != 0xFF;
}

}  // namespace fhs
