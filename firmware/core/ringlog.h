// Records go into 4 KB blocks, one per flash sector, written round the whole flash as a ring (the oldest block is
// erased when the ring comes round). A block's header is written when it opens; its records are programmed a
// 256-byte page at a time (so at most 16 records, 8 s, live only in RAM), and its count when it closes.
#pragma once
#include <stdint.h>
#include "format.h"

namespace fhs {

struct Flash {   // NOR flash: erase sets a 4 KB sector to 0xFF; program only clears bits, never across a 256-byte page
  virtual uint32_t size() = 0;
  virtual bool read(uint32_t addr, void* buf, uint32_t n) = 0;
  virtual bool program(uint32_t addr, const void* buf, uint32_t n) = 0;
  virtual bool erase_sector(uint32_t addr) = 0;
  virtual ~Flash() {}
};

struct BlockInfo {   // what goes into a new block's header besides the counters
  int64_t t_seq0_ms = 0;
  uint16_t batt_mv = 0;
  int16_t zero[3] = {0, 0, 10000};
  uint8_t mode = 0, head_axis = 0;
};

#if defined(ESP32)
constexpr int MAX_SECTORS = 3824;  // the watch: 15 MB (partitions.csv)
#else
constexpr int MAX_SECTORS = 512;   // the laptop's tests: 2 MB
#endif

class RingLog {
 public:
  struct Entry {
    uint32_t block_no, boot_id, first_seq;
    uint16_t n;       // records in the block
    bool valid;
  };
  // scan the flash; boot_floor/block_floor: counters must stay above these (kept outside the flash, so an erase
  // never reuses a boot id)
  bool begin(Flash* f, uint32_t boot_floor = 0, uint32_t block_floor = 0);
  uint32_t boot_id() const { return boot_id_; }
  uint32_t next_block_no() const { return next_block_no_; }
  // add record seq of this boot; opens a block if needed (a gap in seq closes the old one first)
  void append(const Record& r, uint32_t seq, const BlockInfo& info);
  void close();                      // close the open block early (logging paused)
  void erase_all();                  // wipe the log (counters keep counting up)
  bool is_open() const { return open_ >= 0; }
  int sectors() const { return n_sect_; }
  const Entry& entry(int s) const { return idx_[s]; }
  int open_sector() const { return open_; }
  bool read_block(int s, uint8_t* out);           // whole 4 KB block; the open one comes from RAM
  bool read_record(int s, int i, Record* r);      // record i of the block in sector s
  bool read_part(int s, uint32_t off, void* out, uint32_t n);   // bytes [off, off+n) of the block in sector s
  int oldest() const;                // sector of the oldest valid block, -1 if none
  int next_in_order(int s) const;    // the sector after s in block order, -1 at the newest
  int first_from(uint32_t block_no) const;   // the sector of the first block at or after block_no, -1 if none
  // the sector of the oldest block of this boot that still holds a record at or after seq, -1 if none
  int first_since(uint32_t boot, uint32_t seq) const;
  int used() const;                  // valid blocks
  uint32_t errors = 0;               // flash calls that failed

 private:
  void open_block(uint32_t seq, const BlockInfo& info);
  void flush_to(uint32_t end);
  Flash* f_ = nullptr;
  Entry idx_[MAX_SECTORS];
  uint8_t buf_[BLOCK_SIZE];
  int n_sect_ = 0, open_ = -1, next_sector_ = 0;
  uint32_t flushed_ = 0, boot_id_ = 1, next_block_no_ = 0;
};

}  // namespace fhs
