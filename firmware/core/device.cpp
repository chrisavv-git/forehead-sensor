#include "device.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace fhs {

// ---------- helpers (no %f / %lld: newlib-nano's printf on the chip has neither) ----------

void hex_encode(char* out, const void* data, size_t n) {
  static const char H[] = "0123456789abcdef";
  const uint8_t* p = static_cast<const uint8_t*>(data);
  for (size_t i = 0; i < n; i++) { out[2 * i] = H[p[i] >> 4]; out[2 * i + 1] = H[p[i] & 15]; }
  out[2 * n] = 0;
}

int u64_to_dec(char* out, uint64_t v) {
  char t[21];
  int n = 0;
  do { t[n++] = char('0' + v % 10); v /= 10; } while (v);
  for (int i = 0; i < n; i++) out[i] = t[n - 1 - i];
  out[n] = 0;
  return n;
}

int i64_to_dec(char* out, int64_t v) {
  if (v < 0) { out[0] = '-'; return 1 + u64_to_dec(out + 1, uint64_t(-(v + 1)) + 1); }
  return u64_to_dec(out, uint64_t(v));
}

int fixed_to_dec(char* out, float v, int decimals) {
  int64_t scale = 1;
  for (int i = 0; i < decimals; i++) scale *= 10;
  int64_t x = llroundf(fabsf(v) * float(scale));
  int n = 0;
  if (v < 0 && x) out[n++] = '-';
  n += u64_to_dec(out + n, uint64_t(x / scale));
  if (decimals) {
    out[n++] = '.';
    int64_t f = x % scale;
    for (int64_t d = scale / 10; d; d /= 10) { out[n++] = char('0' + (f / d) % 10); }
  }
  out[n] = 0;
  return n;
}

bool TxRing::put(const char* s, size_t n) {
  if (n > room()) return false;
  for (size_t i = 0; i < n; i++) { buf_[head_] = s[i]; head_ = (head_ + 1) % SIZE; }
  return true;
}

size_t TxRing::peek(const char** p) const {
  *p = buf_ + tail_;
  return head_ >= tail_ ? head_ - tail_ : SIZE - tail_;
}

bool params_valid(const Params& p) {
  for (int i = 0; i < N_PARAM_DEFS; i++) {
    const ParamDef& d = PARAM_DEFS[i];
    const uint8_t* f = reinterpret_cast<const uint8_t*>(&p) + d.offset;
    float x;
    if (d.type == PT_F32) memcpy(&x, f, 4);
    else if (d.type == PT_U8) x = f[0];
    else if (d.type == PT_U16) { uint16_t v; memcpy(&v, f, 2); x = v; }
    else { for (int k = 0; k < 4; k++) if (f[k] > d.hi) return false; continue; }
    if (!(x >= d.lo && x <= d.hi)) return false;   // also catches NaN
  }
  float n = p.zero[0] * p.zero[0] + p.zero[1] * p.zero[1] + p.zero[2] * p.zero[2];
  return p.mode <= MODE_THERAPY && p.head_axis <= 3 && n > 0.81f && n < 1.21f;
}

// ---------- setup ----------

void Device::begin(Hal* hal, Flash* flash) {
  hal_ = hal;
  if (!hal->load_params(p_) || !params_valid(p_)) p_ = Params();
  if (!hal->load_state(st_)) st_ = State();
  uint8_t k[8];
  if (hal->chip_key(k)) {   // from the chip itself: survives a wiped settings store, the laptop stays paired
    hex_encode(st_.key, k, 8);
  } else if (strlen(st_.key) != 16) {   // first boot: a random key the laptop learns over USB ("key")
    for (int i = 0; i < 8; i += 4) { uint32_t r = hal->random32(); memcpy(k + i, &r, 4); }
    hex_encode(st_.key, k, 8);
  }
  log_.begin(flash, st_.boot_id, st_.block_no);
  st_.boot_id = log_.boot_id();
  if (log_.next_block_no() > st_.block_no) st_.block_no = log_.next_block_no();
  hal->save_state(st_);
  seq0_ms_ = hal->ms();
  seq_ = 0;
  for (auto& g : g_hist_) { g[0] = g[1] = 0; g[2] = 1; }
  sense_.config(hal->imu_hz(), hal->acc_g_per_lsb(), hal->has_gyro());
  resume_ = hal->resumed();
  therapy_.seed(hal->random32());
  uint32_t len[N_PROMPTS];
  for (int i = 0; i < N_PROMPTS; i++) len[i] = hal->prompt_ms(uint8_t(i));
  guide_.lengths(len);
  int64_t rtc;
  if (hal->rtc_get(&rtc) && rtc >= 1600000000000LL) set_clock(rtc, st_.tz_min);   // the watch knows the time at once
}

void Device::set_clock(int64_t unix_ms, int16_t tz_min) {
  t_seq0_ms_ = unix_ms - int64_t(uint32_t(hal_->ms() - seq0_ms_));
  tz_min_ = tz_min;
  clock_ = true;
}

BlockInfo Device::info() const {
  BlockInfo b;
  b.t_seq0_ms = clock_ ? t_seq0_ms_ : 0;
  b.batt_mv = batt_;
  for (int i = 0; i < 3; i++) b.zero[i] = int16_t(lrintf(p_.zero[i] * 10000));
  b.mode = p_.mode;
  b.head_axis = p_.head_axis;
  return b;
}

// ---------- inputs ----------

void Device::imu(const int16_t raw[6]) {
  sense_.imu(raw);
  Port& u = ports_[PORT_USB];
  if (u.raw_imu && u.up) {   // raw motion samples for the bench test: 20 samples per X line
    memcpy(rawi_ + rawi_n_ * 12, raw, 12);
    if (++rawi_n_ == 20) {
      char line[2 + 11 + 480 + 2];
      int n = snprintf(line, 16, "X %lu ", (unsigned long)seq_);
      hex_encode(line + n, rawi_, 240);
      send(PORT_USB, line);
      rawi_n_ = 0;
    }
  }
}

void Device::mic(const int16_t* pcm, int n) {
  sense_.mic(pcm, n);
  Port& u = ports_[PORT_USB];
  if (u.raw_mic && u.up) {   // raw 16 kHz samples: 128 per M line
    for (int k = 0; k < n; k++) {
      rawm_[rawm_n_++] = pcm[k];
      if (rawm_n_ == 128) {
        char line[2 + 11 + 512 + 2];
        int m = snprintf(line, 16, "M %lu ", (unsigned long)seq_);
        hex_encode(line + m, rawm_, 256);
        send(PORT_USB, line);
        rawm_n_ = 0;
      }
    }
  }
}

void Device::button(bool down) {
  uint32_t now = hal_->ms();
  if (down) {
    second_ = short_pending_ && now - short_up_ms_ <= DOUBLE_GAP_MS;   // the second press of a double press
    short_pending_ = false;
    btn_down_ = true;
    btn_ms_ = now;
    off_warned_ = false;
    return;
  }
  if (!btn_down_) return;
  btn_down_ = false;
  uint32_t held = now - btn_ms_;
  bool second = second_;
  second_ = false;
  if (off_warned_) { note("switch off cancelled"); return; }   // let go before the power chip's 10 s: it stays on
  if (held > DOUBLE_MAX_PRESS_MS) return;   // too long for a double press (holding does nothing until 5 s)
  if (second) {
    ack_ = true;
    ack_ms_ = now;
    note("button: double press");
    double_tap();
  } else {
    short_pending_ = true;
    short_up_ms_ = now;
  }
}

void Device::double_tap() {
  tap_flag_ = true;
  if (!p_.tap_zero) return;
  // any time of the night (no press was ever logged in sleep: the button faces the hairline)
  ZeroGuide::Event e = guide_.press(hal_->ms());
  if (e == ZeroGuide::EV_STARTED) note("zero: started");
  else if (e == ZeroGuide::EV_CANCELLED) note("zero: cancelled");
}

void Device::rx(int port, const char* data, size_t n) {
  Port& pt = ports_[port];
  for (size_t i = 0; i < n; i++) {
    char c = data[i];
    if (c == '\r') continue;
    if (c == '\n') {
      pt.line[pt.len] = 0;
      if (pt.overflow) send(port, "ERR line too long");
      else if (pt.len) command(port, pt.line);
      pt.len = 0;
      pt.overflow = false;
    } else if (pt.len < sizeof pt.line - 1) {
      pt.line[pt.len++] = c;
    } else {
      pt.overflow = true;
    }
  }
}

void Device::link(int port, bool up) {
  Port& pt = ports_[port];
  pt.up = up;
  pt.live = pt.raw_imu = pt.raw_mic = false;
  pt.authed = port == PORT_USB;   // USB is trusted; Bluetooth must send the key first
  pt.job = Job();
  pt.len = 0;
  pt.overflow = false;
  pt.tx.clear();
  pt.taken_seen = pt.tx.taken();
  pt.tx_ms = hal_ ? hal_->ms() : 0;
  if (port == PORT_USB) rawi_n_ = rawm_n_ = 0;
}

// ---------- the 0.5 s tick ----------

// the bedtime buzz, t ms in: vibration %, or -1 once it's over. Variant 0: two pulses (on duty); 1: three long
// ones (the battery is too low for a night). Full strength and 0.4 s: 2 x 120 ms at 60 % was never felt and didn't
// even start the coin motor on battery (31 Oct)
static int cue_motor(uint8_t variant, uint32_t t) {
  static const uint16_t OK_MS[] = {400, 300, 400, 0}, LOW_MS[] = {600, 300, 600, 300, 600, 0};   // on, off, on...
  const uint16_t* seg = variant ? LOW_MS : OK_MS;
  for (int i = 0; seg[i]; i++) {
    if (t < seg[i]) return i % 2 ? 0 : 100;
    t -= seg[i];
  }
  return -1;
}

void Device::poll() {
  uint32_t now = hal_->ms();
  // close every window that has ended; a stalled loop gives empty (F_SHORT) windows, so seq always = time
  while (int32_t(now - (seq0_ms_ + (seq_ + 1) * WINDOW_MS)) >= 0) tick();
  Output o;
  bool playing = false;
  if (btn_down_ && !off_warned_ && now - btn_ms_ >= OFF_WARN_MS) {   // held past 5 s: about to switch off
    off_warned_ = true;
    say_id_ = PR_OFF;
    say_ms_ = now;
    note("switching off");
    if (guide_.active()) { guide_.stop(); note("zero: stopped (switching off)"); }   // a zero still talking
  }
  // + 1.5 s: drawing the screen message blocks the watch's loop for a moment, and a 0.6 s phrase was over (cut) before
  // it got to play (31 Oct); the clip plays to its end, then silence
  if (say_id_ >= 0 && now - say_ms_ >= hal_->prompt_ms(uint8_t(say_id_)) + 1500) say_id_ = -1;
  if (say_id_ >= 0) {   // a phrase on its own, loud, over everything else
    o.hz = hz_prompt(uint8_t(say_id_));
    o.vol = 100;
    playing = true;
    guide_in_window_ = true;
    guide_prompt_ = uint8_t(say_id_);
  } else if (test_until_ && int32_t(now - test_until_) < 0) {
    if (test_step_) o = nudge_output(uint8_t(STEP_TEST << 4 | test_variant_), now - test_ms_, p_, test_step_);
    else o.motor = test_level_;
    playing = true;
    test_in_window_ = true;
  } else if (int pr = guide_.poll(now); pr >= 0 || guide_.active()) {
    test_until_ = 0;
    playing = true;
    if (pr >= 0) {   // the spoken zero, always loud (it is said lying awake, before any earplugs go in)
      o.hz = hz_prompt(uint8_t(pr));
      o.vol = 100;
      guide_in_window_ = true;
      guide_prompt_ = uint8_t(pr);
    }
  } else {
    test_until_ = 0;
    if (p_.mode == MODE_THERAPY && session_ && !cutoff_ && therapy_.step()) {
      o = nudge_output(therapy_.nudge(), now - therapy_.step_ms(), p_);
      playing = true;
    } else if (cue_) {
      // the session (and cue_ms_) may start inside this poll's ticks, a few ms after now was read (the backfill
      // writes the flash): then now - cue_ms_ is "negative", which skipped the whole buzz on the watch (31 Oct)
      int m = cue_motor(cue_variant_, int32_t(now - cue_ms_) < 0 ? 0 : now - cue_ms_);
      if (m < 0) cue_ = false;
      else { o.motor = uint8_t(m); cue_in_window_ = true; playing = true; }
    }
  }
  if (ack_) {   // the button was taken: a short full buzz, over whatever else runs (the voice is the speaker)
    if (now - ack_ms_ < ACK_MS) { o.motor = 100; playing = true; }
    else ack_ = false;
  }
  playing_ = playing;
  if (o.motor != motor_out_) { hal_->motor(o.motor); motor_out_ = o.motor; }
  if (o.hz != hz_out_ || o.vol != vol_out_) { hal_->tone(o.hz, o.vol); hz_out_ = o.hz; vol_out_ = o.vol; }
  for (int i = 0; i < N_PORTS; i++) {
    Port& pt = ports_[i];
    if (!pt.up) continue;
    if (pt.tx.count() == 0 || pt.tx.taken() != pt.taken_seen) {   // nothing waiting, or the link took some
      pt.taken_seen = pt.tx.taken();
      pt.tx_ms = now;
    } else if (int32_t(now - pt.tx_ms) >= int32_t(TX_STALL_MS)) {   // nobody is reading: stop sending, drop what waits
      pt.job = Job();
      pt.live = pt.raw_imu = pt.raw_mic = false;
      pt.tx.clear();
      pt.tx_ms = now;
      pt.stalls++;
      if (i == PORT_USB) rawi_n_ = rawm_n_ = 0;
      hal_->link_stalled(i);
    }
    if (pt.job.kind != Job::NONE) step_job(i);
  }
}

void Device::tick() {
  uint32_t end_ms = seq0_ms_ + (seq_ + 1) * WINDOW_MS;
  Record r;
  sense_.close(r, p_);
  r.seq = uint16_t(seq_);
  bool valid = sense_.last_imu_n > 0 && sense_.have_grav;
  uint16_t mv = hal_->batt_mv();
  if (mv) { batt_f_ = batt_f_ > 0 ? batt_f_ * 0.9f + mv * 0.1f : mv; batt_ = uint16_t(batt_f_ + 0.5f); }
  usb_ = hal_->usb_power();
  if (usb_) r.flags |= F_USB;
  if (batt_ && batt_ < p_.low_mv) r.flags |= F_LOWBAT;
  if (clock_) r.flags |= F_CLOCK;
  if (tap_flag_) { r.flags |= F_TAP; tap_flag_ = false; }
  if (valid) {   // gravity + movement of the last 4 windows, for "zero"
    for (int k = 3; k > 0; k--) { memcpy(g_hist_[k], g_hist_[k - 1], 12); m_hist_[k] = m_hist_[k - 1]; }
    memcpy(g_hist_[0], sense_.grav, 12);
    m_hist_[0] = sense_.move_dps;
  } else {
    m_hist_[0] = 99;
  }
  if (guide_.active()) {
    float g[3];
    switch (guide_.window(end_ms, valid && sense_.move_dps <= p_.still_dps, sense_.grav, g)) {
      case ZeroGuide::EV_MEASURED: guide_.result(end_ms, set_zero(g, true)); break;
      case ZeroGuide::EV_RETRY: note(guide_.tries() == 2 ? "zero: moved, try 2 of 3" : "zero: moved, try 3 of 3"); break;
      case ZeroGuide::EV_GAVE_UP: note("zero: not set (moved)"); break;
      default: break;
    }
    therapy_.hold(end_ms + ZERO_HOLD_MS);   // while it runs and 5 min after its last window
  }
  if (zero_flag_) { r.flags |= F_ZERO; zero_flag_ = false; }
  bool upright = r.flags & F_UPRIGHT;
  update_session(valid && !upright, end_ms);
  uint8_t nudge = therapy_.tick(end_ms, r.roll / 100.0f, upright, valid, p_);
  if (cutoff_) nudge = 0;
  if (test_in_window_) r.nudge = uint8_t(STEP_TEST << 4 | (test_step_ ? test_variant_ : 0));
  else if (guide_in_window_) r.nudge = uint8_t(STEP_ZERO << 4 | guide_prompt_);   // its own voice, not the bell
  else if (p_.mode == MODE_SHADOW && nudge) r.nudge = uint8_t(NUDGE_SHADOW | nudge);
  else if (nudge) r.nudge = nudge;
  else if (cue_in_window_) r.nudge = uint8_t(STEP_CUE << 4 | cue_variant_);   // (the analysis knows it's not the bell)
  else r.nudge = 0;
  test_in_window_ = test_until_ != 0;
  cue_in_window_ = cue_;
  guide_in_window_ = false;
  hist_[seq_ % HIST] = r;
  if (session_) log_.append(r, seq_, info());
  last_ = r;
  for (int i = 0; i < N_PORTS; i++) {
    Port& pt = ports_[i];
    if (pt.up && pt.live && pt.authed) {
      char line[64];
      record_line(line, log_.boot_id(), seq_, r);
      if (!send(i, line)) pt.dropped++;
    }
  }
  seq_++;
}

void Device::update_session(bool lying, uint32_t now) {
  // the battery recovers a little once the load drops: stay cut off until 100 mV above the limit (or on USB),
  // or logging would flap on and off near empty
  if (usb_ || !batt_) cutoff_ = false;
  else if (batt_ < p_.cutoff_mv) cutoff_ = true;
  else if (batt_ > p_.cutoff_mv + 100) cutoff_ = false;
  bool win = true;
  if (clock_ && p_.win_from_h != p_.win_to_h) {
    int64_t t = unix_ms_of(seq_) / 1000 + int64_t(tz_min_) * 60;
    int h = int(((t / 3600) % 24 + 24) % 24);
    win = p_.win_from_h < p_.win_to_h ? (h >= p_.win_from_h && h < p_.win_to_h) : (h >= p_.win_from_h || h < p_.win_to_h);
  }
  bool base = (p_.log_usb || !usb_) && win && !cutoff_;
  if (lying) { lie_n_++; unlie_n_ = 0; } else { unlie_n_++; lie_n_ = 0; }
  if (!base) {
    if (session_) end_session();
    return;
  }
  // back from a hang or crash in the night: the session was cut, so carry on without the 2 min and the start wait
  // (only in the first 10 min after the restart; a later lie-down is a new night)
  if (resume_ && now - seq0_ms_ > 10u * 60 * 1000) resume_ = false;
  if (!session_ && resume_ && (lying || !p_.lie_gate)) {
    resume_ = false;
    start_session(now, 0, true);
    note("session resumed after a restart");
    return;
  }
  if (!p_.lie_gate) {
    if (!session_) start_session(now, 0);
    return;
  }
  if (!session_ && lie_n_ >= 240) start_session(now, lie_n_ - 1);   // 2 min lying: log from when it began
  else if (session_ && unlie_n_ >= 2400) end_session();             // 20 min not lying (up for the day)
}

void Device::start_session(uint32_t now, uint32_t backfill, bool skip_wait) {
  session_ = true;
  session_ms_ = now;
  therapy_.session_start(now, skip_wait);
  if (!skip_wait) {   // the bedtime buzz (not after a hang in the night: that carries on quietly)
    cue_ = true;
    cue_variant_ = batt_ && !usb_ && batt_ < p_.night_mv ? 1 : 0;
    cue_ms_ = hal_->ms();
  }
  if (backfill > uint32_t(HIST - 1)) backfill = HIST - 1;
  if (backfill > seq_) backfill = seq_;
  for (uint32_t s = seq_ - backfill; s < seq_; s++) log_.append(hist_[s % HIST], s, info());
}

void Device::end_session() {
  session_ = false;
  log_.close();
  therapy_.session_end();
}

bool Device::set_zero(const float g[3], bool by_tap) {
  float n = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
  if (n < 0.5f) return false;
  float z[3] = {g[0] / n, g[1] / n, g[2] / n};
  // no limit to how far it may move from the old zero (it was 30 deg for the button): the button needs a double press
  // and the watch says out loud that it is zeroing, so it can't happen unnoticed (31 Oct)
  memcpy(p_.zero, z, 12);
  p_.zero_set = 1;
  hal_->save_params(p_);
  zero_flag_ = true;
  note(by_tap ? "zero set by tap" : "zero set");
  return true;
}

void Device::note(const char* msg) {   // event lines for whoever is listening
  char line[96];
  snprintf(line, sizeof line, "N %s", msg);
  hal_->event(msg);
  for (int i = 0; i < N_PORTS; i++)
    if (ports_[i].up && ports_[i].authed) send(i, line);
}

// ---------- output ----------

bool Device::send(int port, const char* s) {
  Port& pt = ports_[port];
  if (!pt.up) return false;
  size_t n = strlen(s);
  if (pt.tx.room() < n + 1) return false;
  pt.tx.put(s, n);
  pt.tx.put("\n", 1);
  return true;
}

void Device::sendf(int port, const char* fmt, ...) {
  char line[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof line, fmt, ap);
  va_end(ap);
  send(port, line);
}

void Device::record_line(char* out, uint32_t boot, uint32_t seq, const Record& r) {
  int n = snprintf(out, 32, "R %lu %lu ", (unsigned long)boot, (unsigned long)seq);
  hex_encode(out + n, &r, RECORD_SIZE);
}

void Device::status(int port) {
  char t0[24], roll[12], pitch[12], z0[12], z1[12], z2[12], fd[12];
  i64_to_dec(t0, clock_ ? t_seq0_ms_ : 0);
  fixed_to_dec(roll, last_.roll / 100.0f, 1);
  fixed_to_dec(pitch, last_.pitch / 100.0f, 1);
  fixed_to_dec(z0, p_.zero[0], 3);
  fixed_to_dec(z1, p_.zero[1], 3);
  fixed_to_dec(z2, p_.zero[2], 3);
  fixed_to_dec(fd, therapy_.flat_ms(seq0_ms_ + seq_ * WINDOW_MS) / 1000.0f, 1);
  int oldest = log_.oldest();
  unsigned long ob = oldest >= 0 ? (unsigned long)log_.entry(oldest).block_no : 0;
  sendf(port,
        "S fw=" FHS_FW_VERSION " boot=%lu seq=%lu mode=%s batt=%u usb=%d chg=%d session=%d cutoff=%d clock=%d t0=%s tz=%d "
        "blocks=%d sectors=%d oldest=%lu next=%lu open=%d err=%lu",
        (unsigned long)log_.boot_id(), (unsigned long)seq_, p_.mode == MODE_THERAPY ? "therapy" : "shadow", batt_,
        usb_, hal_->charging(), session_, cutoff_, clock_, t0, tz_min_, log_.used(), log_.sectors(), ob,
        (unsigned long)log_.next_block_no(), log_.is_open(), (unsigned long)log_.errors);
  if (hal_->info()[0]) sendf(port, "S hw %s", hal_->info());
  sendf(port, "S roll=%s pitch=%s flat_s=%s step=%u variant=%u imu=%d mic=%d zero=%s,%s,%s zero_set=%d axis=%u "
        "dropped=%lu stalls=%lu", roll, pitch, fd, therapy_.step(), therapy_.nudge() & 15, sense_.last_imu_n, sense_.last_mic_n,
        z0, z1, z2, p_.zero_set, p_.head_axis, (unsigned long)ports_[port].dropped, (unsigned long)ports_[port].stalls);
}

void Device::params_line(int port) {
  char line[400];
  int n = snprintf(line, sizeof line, "P");
  for (int i = 0; i < N_PARAM_DEFS; i++) {
    const ParamDef& d = PARAM_DEFS[i];
    const uint8_t* f = reinterpret_cast<const uint8_t*>(&p_) + d.offset;
    char v[24];
    if (d.type == PT_F32) { float x; memcpy(&x, f, 4); fixed_to_dec(v, x, 1); }
    else if (d.type == PT_U8) snprintf(v, sizeof v, "%u", f[0]);
    else if (d.type == PT_U16) { uint16_t x; memcpy(&x, f, 2); snprintf(v, sizeof v, "%u", x); }
    else snprintf(v, sizeof v, "%u,%u,%u,%u", f[0], f[1], f[2], f[3]);
    n += snprintf(line + n, sizeof line - n, " %s=%s", d.key, v);
  }
  snprintf(line + n, sizeof line - n, " mode=%s axis=%u", p_.mode == MODE_THERAPY ? "therapy" : "shadow", p_.head_axis);
  send(port, line);
}

bool Device::set_param(const char* key, const char* val) {
  for (int i = 0; i < N_PARAM_DEFS; i++) {
    const ParamDef& d = PARAM_DEFS[i];
    if (strcmp(d.key, key)) continue;
    uint8_t* f = reinterpret_cast<uint8_t*>(&p_) + d.offset;
    char* end;
    if (d.type == PT_LEVELS) {
      uint8_t lv[4];
      const char* s = val;
      for (int k = 0; k < 4; k++) {
        long x = strtol(s, &end, 10);
        if (end == s || x < d.lo || x > d.hi || (k < 3 && *end != ',') || (k == 3 && *end)) return false;
        lv[k] = uint8_t(x);
        s = end + 1;
      }
      memcpy(f, lv, 4);
      return true;
    }
    float x = strtof(val, &end);
    if (end == val || *end || !(x >= d.lo && x <= d.hi)) return false;
    if (d.type == PT_F32) memcpy(f, &x, 4);
    else {
      if (x != floorf(x)) return false;
      if (d.type == PT_U8) f[0] = uint8_t(x);
      else { uint16_t v = uint16_t(x); memcpy(f, &v, 2); }
    }
    return true;
  }
  return false;
}

// ---------- commands ----------

void Device::command(int port, char* line) {
  char* argv[6];
  int argc = 0;
  for (char* t = strtok(line, " \t"); t && argc < 6; t = strtok(nullptr, " \t")) argv[argc++] = t;
  if (!argc) return;
  Port& pt = ports_[port];
  const char* c = argv[0];
  uint32_t now = hal_->ms();
  if (!strcmp(c, "hello")) {
    sendf(port, "OK hello fw=" FHS_FW_VERSION " boot=%lu auth=%d", (unsigned long)log_.boot_id(), pt.authed);
    return;
  }
  if (!strcmp(c, "auth")) {
    pt.authed = pt.authed || (argc == 2 && strlen(argv[1]) == 16 && !strcmp(argv[1], st_.key));
    send(port, pt.authed ? "OK auth" : "ERR auth");
    return;
  }
  if (!pt.authed) { send(port, "ERR auth needed"); return; }

  if (!strcmp(c, "key")) {
    if (port != PORT_USB) { send(port, "ERR key only over USB"); return; }
    sendf(port, "OK key %s", st_.key);
  } else if (!strcmp(c, "status")) {
    status(port);
    send(port, "OK status");
  } else if (!strcmp(c, "time") && argc >= 2) {
    char* end;
    long long unix_ms = strtoll(argv[1], &end, 10);
    long tz = argc >= 3 ? strtol(argv[2], nullptr, 10) : 0;
    if (*end || unix_ms < 1600000000000LL || tz < -900 || tz > 900) { send(port, "ERR time"); return; }
    set_clock(unix_ms, int16_t(tz));
    hal_->rtc_set(unix_ms);
    if (st_.tz_min != tz) { st_.tz_min = int16_t(tz); hal_->save_state(st_); }
    char t0[24];
    i64_to_dec(t0, t_seq0_ms_);
    sendf(port, "OK time t0=%s seq=%lu boot=%lu", t0, (unsigned long)seq_, (unsigned long)log_.boot_id());
  } else if (!strcmp(c, "get")) {
    params_line(port);
    send(port, "OK get");
  } else if (!strcmp(c, "set") && argc == 3) {
    Params old = p_;
    if (!set_param(argv[1], argv[2])) { sendf(port, "ERR set %s", argv[1]); return; }
    if (!params_valid(p_)) { p_ = old; sendf(port, "ERR set %s", argv[1]); return; }
    hal_->save_params(p_);
    sendf(port, "OK set %s=%s", argv[1], argv[2]);
  } else if (!strcmp(c, "mode") && argc == 2 && (!strcmp(argv[1], "shadow") || !strcmp(argv[1], "therapy"))) {
    p_.mode = !strcmp(argv[1], "therapy") ? MODE_THERAPY : MODE_SHADOW;
    hal_->save_params(p_);
    sendf(port, "OK mode %s", argv[1]);
  } else if (!strcmp(c, "axis") && argc == 2 && argv[1][0] >= '0' && argv[1][0] <= '3' && !argv[1][1]) {
    p_.head_axis = uint8_t(argv[1][0] - '0');
    hal_->save_params(p_);
    sendf(port, "OK axis %u", p_.head_axis);
  } else if (!strcmp(c, "zero")) {
    float g[3] = {0, 0, 0};
    for (int k = 0; k < 4; k++) {
      if (m_hist_[k] > p_.still_dps) { send(port, "ERR zero: not still (lie flat on the back, keep still 2 s)"); return; }
      for (int i = 0; i < 3; i++) g[i] += g_hist_[k][i];
    }
    set_zero(g, false);
    send(port, "OK zero");
  } else if (!strcmp(c, "buzz") && argc == 3) {
    long lv = strtol(argv[1], nullptr, 10), ms = strtol(argv[2], nullptr, 10);
    if (lv < 1 || lv > 100 || ms < 1 || ms > 5000) { send(port, "ERR buzz <1-100 %> <1-5000 ms>"); return; }
    test_level_ = uint8_t(lv);
    test_step_ = 0;
    test_until_ = now + uint32_t(ms);
    if (!test_until_) test_until_ = 1;
    send(port, "OK buzz");
  } else if (!strcmp(c, "nudge") && argc == 4) {   // one step of a nudge, as therapy would play it
    long st = strtol(argv[1], nullptr, 10), v = strtol(argv[2], nullptr, 10), ms = strtol(argv[3], nullptr, 10);
    if (st < 1 || st > 4 || v < 0 || v >= N_VARIANTS || ms < 1 || ms > 10000) {
      send(port, "ERR nudge <step 1-4> <variant 0-15> <1-10000 ms>");
      return;
    }
    test_step_ = uint8_t(st);
    test_variant_ = uint8_t(v);
    test_ms_ = now;
    test_until_ = now + uint32_t(ms);
    if (!test_until_) test_until_ = 1;
    send(port, "OK nudge");
  } else if (!strcmp(c, "diag")) {
    char out[200];
    if (!hal_->motor_diag(out, sizeof out)) { send(port, "ERR diag: no motor self-test here"); return; }
    sendf(port, "OK diag %s", out);
  } else if (!strcmp(c, "stop")) {
    test_until_ = 0;
    pt.job = Job();
    pt.live = pt.raw_imu = pt.raw_mic = false;
    send(port, "OK stop");
  } else if (!strcmp(c, "live") && argc == 2) {
    pt.live = !strcmp(argv[1], "on");
    sendf(port, "OK live %s", pt.live ? "on" : "off");
  } else if (!strcmp(c, "since") && argc >= 3) {
    pt.job = Job();
    pt.job.kind = Job::SINCE;
    pt.job.boot = strtoul(argv[1], nullptr, 10);
    pt.job.seq = strtoul(argv[2], nullptr, 10);
    pt.job.then_live = argc >= 4 && !strcmp(argv[3], "live");
    pt.live = false;   // records come in order: catch-up first, then live
    sendf(port, "OK since %lu %lu", (unsigned long)pt.job.boot, (unsigned long)pt.job.seq);
  } else if (!strcmp(c, "blocks")) {
    pt.job = Job();
    pt.job.kind = Job::BLOCKS;
  } else if (!strcmp(c, "dump")) {
    pt.job = Job();
    pt.job.kind = Job::DUMP;
    pt.job.block_no = argc >= 2 ? strtoul(argv[1], nullptr, 10) : 0;
    pt.job.limit = argc >= 3 ? strtoul(argv[2], nullptr, 10) : 0;
  } else if (!strcmp(c, "raw") && argc == 2) {
    if (port != PORT_USB) { send(port, "ERR raw only over USB"); return; }
    pt.raw_imu = !strcmp(argv[1], "imu");
    pt.raw_mic = !strcmp(argv[1], "mic");
    rawi_n_ = rawm_n_ = 0;
    sendf(port, "OK raw %s", argv[1]);
  } else if (!strcmp(c, "session")) {
    if (session_) { session_ms_ = now; therapy_.session_start(now); }
    sendf(port, "OK session %d", session_);
  } else if (!strcmp(c, "erase") && argc == 2 && !strcmp(argv[1], "yes")) {
    end_session();
    log_.erase_all();
    lie_n_ = 0;
    st_.block_no = log_.next_block_no();
    hal_->save_state(st_);
    send(port, "OK erase");
  } else if (!strcmp(c, "reboot")) {
    send(port, "OK reboot");
    hal_->reboot();
  } else {
    sendf(port, "ERR unknown command: %s", c);
  }
}

// ---------- catch-up, dump, block list: a few lines per poll, so sampling never stalls ----------

void Device::step_job(int port) {
  Port& pt = ports_[port];
  Job& j = pt.job;
  for (int budget = 0; budget < 8; budget++) {
    if (pt.tx.room() < 600) return;
    if (j.kind == Job::BLOCKS) {
      j.sector = j.sector < 0 ? log_.oldest() : log_.next_in_order(j.sector);
      if (j.sector < 0) { sendf(port, "OK blocks n=%lu", (unsigned long)j.count); j.kind = Job::NONE; return; }
      BlockHeader h;
      if (!log_.read_part(j.sector, 0, &h, sizeof h)) continue;
      const RingLog::Entry& e = log_.entry(j.sector);
      char t0[24];
      i64_to_dec(t0, j.sector == log_.open_sector() ? info().t_seq0_ms : h.t_seq0_ms);
      sendf(port, "I %d %lu %lu %lu %u %s %d", j.sector, (unsigned long)e.block_no, (unsigned long)e.boot_id,
            (unsigned long)e.first_seq, e.n, t0, j.sector == log_.open_sector());
      j.count++;
    } else if (j.kind == Job::DUMP) {
      if (j.limit && j.count >= j.limit && j.off == 0) {
        sendf(port, "OK dump n=%lu", (unsigned long)j.count);
        j.kind = Job::NONE;
        return;
      }
      if (j.sector < 0 || !log_.entry(j.sector).valid || log_.entry(j.sector).block_no != j.block_no) {
        // find the first block at or after j.block_no (the start, or the ring overwrote the next one)
        int s = log_.first_from(j.block_no);
        if (s < 0) { sendf(port, "OK dump n=%lu", (unsigned long)j.count); j.kind = Job::NONE; return; }
        if (log_.entry(s).block_no != j.block_no) j.off = 0;
        j.sector = s;
        j.block_no = log_.entry(s).block_no;
      }
      uint8_t part[256];
      if (!log_.read_part(j.sector, j.off, part, 256)) { j.off = 0; j.block_no++; j.sector = -1; continue; }
      char line[2 + 11 + 5 + 512 + 2];
      int n = snprintf(line, 32, "B %lu %lu ", (unsigned long)j.block_no, (unsigned long)j.off);
      hex_encode(line + n, part, 256);
      send(port, line);
      j.off += 256;
      if (j.off >= BLOCK_SIZE) { j.off = 0; j.count++; j.block_no++; j.sector = -1; }
    } else if (j.kind == Job::SINCE) {
      if (j.sector < 0) {
        int s = log_.first_since(j.boot, j.seq);
        if (s < 0) {   // nothing (more) in the log: caught up
          sendf(port, "C %lu %lu", (unsigned long)j.boot, (unsigned long)j.seq);
          pt.live = j.then_live;
          j.kind = Job::NONE;
          return;
        }
        j.sector = s;
        j.block_no = log_.entry(s).block_no;
      }
      const RingLog::Entry& e = log_.entry(j.sector);
      if (!e.valid || e.block_no != j.block_no) { j.sector = -1; continue; }   // overwritten meanwhile
      if (j.seq < e.first_seq) j.seq = e.first_seq;   // records before this block were not logged
      uint32_t i = j.seq - e.first_seq;
      if (i >= e.n) { j.sector = -1; continue; }
      Record r;
      if (!log_.read_record(j.sector, int(i), &r) || !record_ok(r, e.first_seq, i)) { j.seq++; continue; }
      char line[64];
      record_line(line, e.boot_id, j.seq, r);
      send(port, line);
      j.seq++;
    } else {
      return;
    }
  }
}

}  // namespace fhs
