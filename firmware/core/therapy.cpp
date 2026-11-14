#include "therapy.h"
#include <math.h>
#include "format.h"

namespace fhs {

uint32_t Therapy::after(uint16_t s, const Params& p, uint32_t min_ms) {
  int32_t ms = int32_t(s) * 1000;
  if (p.jitter_s) ms += int32_t(rand() % (4u * p.jitter_s + 1)) * 500 - int32_t(p.jitter_s) * 1000;   // 0.5 s steps
  return ms < int32_t(min_ms) ? min_ms : uint32_t(ms);
}

uint8_t Therapy::tick(uint32_t now, float roll, bool upright, bool valid, const Params& p) {
  // sitting up counts only after UPRIGHT_WINDOWS in a row: a fast roll can read "head up" for one window (30 Oct
  // 05:40: pitch 63 for 0.5 s while lying, the 5 min pause then hid a flat back from the watch)
  up_n_ = upright ? up_n_ + 1 : 0;
  if (up_n_ >= UPRIGHT_WINDOWS) { upright_ms_ = now; upright_seen_ = true; }
  if (held_ && int32_t(now - hold_until_) >= 0) held_ = false;   // the hold is over, for good
  float lim = flat_ ? p.angle_deg + p.hyst_deg : p.angle_deg;
  bool flat = valid && !upright && fabsf(roll) < lim;
  bool blocked = !in_session_ || (!skip_wait_ && now - session_ms_ < uint32_t(p.start_wait_s) * 1000) ||
                 (held_ && int32_t(now - hold_until_) < 0);
  if (!flat || blocked) {   // a flat stay only counts from the moment nothing blocks it
    stop();
    return 0;
  }
  if (!flat_) {
    flat_ = true;
    flat_since_ = now;
    gave_up_ = false;
    next_ms_ = now + after(p.delay_s, p, 0);
  }
  // after sitting up: quiet for upright_pause_s (getting back into bed), but never more than PAUSE_NET_MS flat on
  // the back: whatever fooled the pause, the back is unguarded for 60 s at most
  if (upright_seen_ && now - upright_ms_ < uint32_t(p.upright_pause_s) * 1000 && now - flat_since_ < PAUSE_NET_MS)
    return 0;
  if (gave_up_) return 0;
  if (step_ && p.max_buzz_s && now - first_ms_ >= uint32_t(p.max_buzz_s) * 1000) {
    gave_up_ = true;   // quiet until this flat stay ends
    step_ = 0;
    return 0;
  }
  if (int32_t(now - next_ms_) >= 0) {
    if (step_ < 4) step_++;
    if (step_ == 1) first_ms_ = now;
    uint8_t v = 0;
    if (p.vary) {   // never the same variant twice in a row
      v = uint8_t(rand() % N_VARIANTS);
      if (v == variant_) v = uint8_t((v + 1 + rand() % (N_VARIANTS - 1)) % N_VARIANTS);
    }
    variant_ = v;
    step_ms_ = now;
    next_ms_ = now + after(p.step_s, p, 1000);
  }
  return nudge();
}

// ---------- patterns: each loops for as long as its step lasts ----------

struct Seg { uint16_t ms; uint8_t a0, a1; };       // vibration %, ramping from a0 to a1; ms 0 ends the list
struct Tone { uint16_t ms, f0, f1; };              // Hz, sweeping from f0 to f1; 0 = silent; ms 0 ends the list

static const Seg VIB[8][7] = {
    {{500, 100, 100}, {500, 0, 0}},                                                     // steady pulse
    {{150, 100, 100}, {150, 0, 0}},                                                     // fast pulse
    {{120, 100, 100}, {100, 0, 0}, {120, 100, 100}, {660, 0, 0}},                       // double knock
    {{80, 100, 100}, {80, 0, 0}, {80, 100, 100}, {80, 0, 0}, {80, 100, 100}, {600, 0, 0}},   // triple knock
    {{800, 20, 100}, {400, 0, 0}},                                                      // ramp up
    {{1200, 100, 100}, {600, 0, 0}},                                                    // long
    {{100, 100, 100}, {120, 0, 0}, {250, 100, 100}, {530, 0, 0}},                       // heartbeat
    {{300, 40, 40}, {300, 100, 100}, {300, 40, 40}, {300, 0, 0}},                       // wobble
};

static const Tone SND[7][10] = {
    {{150, 2000, 2000}, {100, 0, 0}, {150, 2000, 2000}, {600, 0, 0}},                   // double beep
    {{400, 600, 2500}, {400, 0, 0}},                                                    // rising chirp
    {{500, 520, 520}, {500, 0, 0}, {500, 520, 520}, {500, 0, 0}, {500, 520, 520}, {1500, 0, 0}},   // low buzz (*)
    {{250, 900, 900}, {250, 1300, 1300}},                                               // siren
    {{60, 3000, 3000}, {60, 0, 0}, {60, 3000, 3000}, {60, 0, 0}, {60, 3000, 3000}, {60, 0, 0},
     {60, 3000, 3000}, {520, 0, 0}},                                                    // fast beeps
    {{80, 1000, 1000}, {80, 1500, 1500}, {80, 1000, 1000}, {80, 1500, 1500}, {80, 1000, 1000},
     {80, 1500, 1500}, {500, 0, 0}},                                                    // warble
    {{200, 700, 700}, {200, 2800, 2800}, {400, 0, 0}},                                  // low-high
};
// (*) the 520 Hz three-beat signal that woke deep sleepers better than high beeps (Bruck & Thomas); at a high
// volume the speaker drives it square, as in their tests

template <class S>
static const S* segment_at(const S* list, uint32_t t, uint32_t* into) {
  uint32_t cycle = 0;
  for (const S* s = list; s->ms; s++) cycle += s->ms;
  t %= cycle;
  const S* s = list;
  while (t >= s->ms) { t -= s->ms; s++; }
  *into = t;
  return s;
}

uint8_t vib_pattern(uint8_t v) { return v % 8; }
uint8_t sound_pattern(uint8_t v) {
  uint8_t s = uint8_t((v * 3 + v / 8) % 8);
  return s < 7 ? s : (v == 5 ? 5 : 4);   // the two variants that played the name clip until 0.0.2: warble, fast beeps
}

Output nudge_output(uint8_t nudge, uint32_t t, const Params& p, uint8_t test_step) {
  Output o;
  uint8_t step = nudge_step(nudge), v = nudge_variant(nudge);
  if (step == STEP_TEST) step = test_step;
  if (step < 1 || step > 4) return o;
  uint32_t in;
  const Seg* s = segment_at(VIB[vib_pattern(v)], t, &in);
  uint32_t a = s->a0 + (int32_t(s->a1) - int32_t(s->a0)) * int32_t(in) / int32_t(s->ms);
  o.motor = uint8_t(p.level[step - 1] * a / 100);
  if (p.volume[step - 1]) {
    const Tone* n = segment_at(SND[sound_pattern(v)], t, &in);
    if (n->f0) {
      o.hz = uint16_t(n->f0 + (int32_t(n->f1) - int32_t(n->f0)) * int32_t(in) / int32_t(n->ms));
      o.vol = p.volume[step - 1];
    }
  }
  return o;
}

}  // namespace fhs
