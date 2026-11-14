#include "ringlog.h"
#include <string.h>

namespace fhs {

bool RingLog::begin(Flash* f, uint32_t boot_floor, uint32_t block_floor) {
  f_ = f;
  n_sect_ = f ? int(f->size() / BLOCK_SIZE) : 0;
  if (n_sect_ > MAX_SECTORS) n_sect_ = MAX_SECTORS;
  open_ = -1;
  int newest = -1;
  uint32_t max_boot = 0;
  for (int s = 0; s < n_sect_; s++) {
    BlockHeader h;
    Entry& e = idx_[s];
    e.valid = false;
    if (!f->read(uint32_t(s) * BLOCK_SIZE, &h, sizeof h)) { errors++; continue; }
    if (!header_ok(h)) continue;
    e.valid = true;
    e.block_no = h.block_no;
    e.boot_id = h.boot_id;
    e.first_seq = h.first_seq;
    if (h.n_closed != N_OPEN && h.n_closed <= RECS_PER_BLOCK) {
      e.n = h.n_closed;
    } else {   // never closed (power lost): count the records that made it
      e.n = 0;
      Record r;
      while (e.n < RECS_PER_BLOCK && read_record(s, e.n, &r) && record_ok(r, e.first_seq, e.n)) e.n++;
    }
    if (newest < 0 || e.block_no > idx_[newest].block_no) newest = s;
    if (e.boot_id > max_boot) max_boot = e.boot_id;
  }
  boot_id_ = (max_boot > boot_floor ? max_boot : boot_floor) + 1;
  next_block_no_ = newest >= 0 ? idx_[newest].block_no + 1 : 0;
  if (next_block_no_ < block_floor) next_block_no_ = block_floor;
  next_sector_ = newest >= 0 ? (newest + 1) % n_sect_ : 0;
  return n_sect_ > 0;
}

bool RingLog::read_record(int s, int i, Record* r) {
  if (s == open_) {
    memcpy(r, buf_ + HEADER_SIZE + i * RECORD_SIZE, RECORD_SIZE);
    return true;
  }
  if (!f_->read(uint32_t(s) * BLOCK_SIZE + HEADER_SIZE + i * RECORD_SIZE, r, RECORD_SIZE)) { errors++; return false; }
  return true;
}

bool RingLog::read_part(int s, uint32_t off, void* out, uint32_t n) {
  if (off + n > BLOCK_SIZE) return false;
  if (s == open_) { memcpy(out, buf_ + off, n); return true; }
  if (!f_->read(uint32_t(s) * BLOCK_SIZE + off, out, n)) { errors++; return false; }
  return true;
}

bool RingLog::read_block(int s, uint8_t* out) {
  if (s == open_) { memcpy(out, buf_, BLOCK_SIZE); return true; }
  if (!f_->read(uint32_t(s) * BLOCK_SIZE, out, BLOCK_SIZE)) { errors++; return false; }
  return true;
}

void RingLog::open_block(uint32_t seq, const BlockInfo& info) {
  int s = next_sector_;
  next_sector_ = (s + 1) % n_sect_;
  idx_[s].valid = false;   // the oldest block (if any) is gone from here on
  if (!f_->erase_sector(uint32_t(s) * BLOCK_SIZE)) errors++;
  memset(buf_, 0xFF, BLOCK_SIZE);
  BlockHeader h;
  memset(&h, 0xFF, sizeof h);
  h.magic = BLOCK_MAGIC;
  h.block_no = next_block_no_++;
  h.boot_id = boot_id_;
  h.first_seq = seq;
  h.t_seq0_ms = info.t_seq0_ms;
  h.batt_mv = info.batt_mv;
  for (int i = 0; i < 3; i++) h.zero[i] = info.zero[i];
  h.version = FORMAT_VERSION;
  h.rec_size = RECORD_SIZE;
  h.rec_per_block = RECS_PER_BLOCK;
  h.mode = info.mode;
  h.head_axis = info.head_axis;
  h.hcrc = crc16(&h, offsetof(BlockHeader, hcrc));
  memcpy(buf_, &h, sizeof h);
  if (!f_->program(uint32_t(s) * BLOCK_SIZE, buf_, HEADER_SIZE)) errors++;
  flushed_ = HEADER_SIZE;
  open_ = s;
  idx_[s] = Entry{h.block_no, h.boot_id, h.first_seq, 0, true};
}

void RingLog::flush_to(uint32_t end) {   // program buf_[flushed_, end): always inside one page
  if (end > flushed_) {
    if (!f_->program(uint32_t(open_) * BLOCK_SIZE + flushed_, buf_ + flushed_, end - flushed_)) errors++;
    flushed_ = end;
  }
}

void RingLog::append(const Record& r, uint32_t seq, const BlockInfo& info) {
  if (n_sect_ == 0) return;   // no flash (it didn't answer at boot): live streaming still works
  if (open_ >= 0) {
    const Entry& e = idx_[open_];
    if (e.boot_id != boot_id_ || seq != e.first_seq + e.n) close();
  }
  if (open_ < 0) open_block(seq, info);
  Entry& e = idx_[open_];
  uint32_t off = HEADER_SIZE + uint32_t(e.n) * RECORD_SIZE;
  memcpy(buf_ + off, &r, RECORD_SIZE);
  e.n++;
  off += RECORD_SIZE;
  if (off % 256 == 0) flush_to(off);
  if (e.n == RECS_PER_BLOCK) close();
}

void RingLog::close() {
  if (open_ < 0) return;
  Entry& e = idx_[open_];
  flush_to(HEADER_SIZE + uint32_t(e.n) * RECORD_SIZE);
  uint16_t n = e.n;
  memcpy(buf_ + offsetof(BlockHeader, n_closed), &n, 2);
  if (!f_->program(uint32_t(open_) * BLOCK_SIZE + offsetof(BlockHeader, n_closed), &n, 2)) errors++;
  open_ = -1;
}

void RingLog::erase_all() {
  // clearing each header's magic (bits 1->0, a quick program) is enough: the scan skips the block, and a sector
  // is erased anyway before it is written again. Erasing all 3824 sectors would block for minutes.
  close();
  const uint32_t zero = 0;
  for (int s = 0; s < n_sect_; s++) {
    if (idx_[s].valid && !f_->program(uint32_t(s) * BLOCK_SIZE, &zero, 4)) errors++;
    idx_[s].valid = false;
  }
}

int RingLog::oldest() const {
  int best = -1;
  for (int s = 0; s < n_sect_; s++)
    if (idx_[s].valid && (best < 0 || idx_[s].block_no < idx_[best].block_no)) best = s;
  return best;
}

int RingLog::next_in_order(int s) const {   // blocks are written in sector order, but check rather than assume
  int best = -1;
  for (int t = 0; t < n_sect_; t++)
    if (idx_[t].valid && idx_[t].block_no > idx_[s].block_no && (best < 0 || idx_[t].block_no < idx_[best].block_no)) best = t;
  return best;
}

int RingLog::first_from(uint32_t block_no) const {   // one pass: walking next_in_order there is slow on 3824 sectors
  int best = -1;
  for (int t = 0; t < n_sect_; t++)
    if (idx_[t].valid && idx_[t].block_no >= block_no && (best < 0 || idx_[t].block_no < idx_[best].block_no)) best = t;
  return best;
}

// one pass, as first_from: walking there from the oldest with next_in_order is sectors x blocks steps (14.6 million
// with the watch's ring full), and a catch-up did that walk at every block
int RingLog::first_since(uint32_t boot, uint32_t seq) const {
  int best = -1;
  for (int t = 0; t < n_sect_; t++) {
    const Entry& e = idx_[t];
    if (e.valid && e.boot_id == boot && e.first_seq + e.n > seq && (best < 0 || e.block_no < idx_[best].block_no)) best = t;
  }
  return best;
}

int RingLog::used() const {
  int n = 0;
  for (int s = 0; s < n_sect_; s++) n += idx_[s].valid;
  return n;
}

}  // namespace fhs
