// Nudge rules: when the head stays flat (face up), step 1 after delay_s, then every step_s the next step, up to
// step 4, which repeats until the head turns. Steps escalate from vibration to sound (levels, volumes), and each
// step plays a randomly picked variant (a vibration pattern + a sound) at a slightly random time, so the nudges
// never settle into one the sleeper gets used to, as happens with a steady buzz.
// Runs in both modes; in shadow mode the nudge is only logged, nothing runs.
#pragma once
#include <stdint.h>
#include "params.h"

namespace fhs {

class Therapy {
 public:
  static constexpr int UPRIGHT_WINDOWS = 6;        // head up 3 s in a row = sat up (roll blips were 0.5-1 s)
  static constexpr uint32_t PAUSE_NET_MS = 60000;  // the pause after sitting up ends after 60 s flat anyway (2 min until 0.0.3)
  void seed(uint32_t s) { rng_ = s ? s : 1; }
  // a night session started (or the laptop asked for a fresh start): the start wait runs from now, unless
  // the device came back from a hang or crash in the night (skip_wait: protect at once)
  void session_start(uint32_t now_ms, bool skip_wait = false) {
    session_ms_ = now_ms;
    skip_wait_ = skip_wait;
    in_session_ = true;
    stop();
  }
  void session_end() { in_session_ = false; stop(); }
  // no nudge until until_ms (the spoken zero: while it talks, and 5 min after, to put on the vest and settle in)
  // (tick() drops the hold once it has passed: left set, the comparison wrapped ~25 days later and blocked the
  // nudges for the next ~25 days)
  void hold(uint32_t until_ms) { hold_until_ = until_ms; held_ = true; }
  // once per 0.5 s window; returns the nudge the rules ask for: step << 4 | variant (0 = none)
  uint8_t tick(uint32_t now_ms, float roll_deg, bool upright, bool valid, const Params& p);
  uint8_t nudge() const { return step_ ? uint8_t(step_ << 4 | variant_) : 0; }
  uint8_t step() const { return step_; }
  uint32_t step_ms() const { return step_ms_; }   // when the current step (and its variant) started
  bool flat() const { return flat_; }
  uint32_t flat_ms(uint32_t now_ms) const { return flat_ ? now_ms - flat_since_ : 0; }

 private:
  void stop() { flat_ = false; step_ = 0; }
  uint32_t rand() { rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5; return rng_; }
  uint32_t after(uint16_t s, const Params& p, uint32_t min_ms);   // s seconds, moved by up to jitter_s
  bool in_session_ = false, flat_ = false, upright_seen_ = false, gave_up_ = false, skip_wait_ = false, held_ = false;
  uint32_t session_ms_ = 0, flat_since_ = 0, upright_ms_ = 0, next_ms_ = 0, step_ms_ = 0, first_ms_ = 0, hold_until_ = 0;
  uint32_t rng_ = 0x2545F491;
  uint8_t step_ = 0, variant_ = 0;
  int up_n_ = 0;   // windows head up in a row
};

// what the outputs do t_ms into a nudge (step << 4 | variant; a STEP_TEST nudge plays step test_step)
struct Output {
  uint8_t motor = 0;   // vibration %
  uint16_t hz = 0;     // tone, 0 = silent
  uint8_t vol = 0;     // tone volume %
};
Output nudge_output(uint8_t nudge, uint32_t t_ms, const Params& p, uint8_t test_step = 4);
constexpr int N_VARIANTS = 16;
uint8_t vib_pattern(uint8_t variant);     // 0-7
uint8_t sound_pattern(uint8_t variant);   // 0-6: every variant is a different (vibration, sound) pair

}  // namespace fhs
