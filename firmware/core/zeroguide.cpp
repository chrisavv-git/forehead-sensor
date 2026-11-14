#include "zeroguide.h"

namespace fhs {

void ZeroGuide::say(uint32_t now, uint8_t id, State then) {
  st_ = OUTRO;
  prompt_ = id;
  then_ = then;
  t0_ = now;
}

ZeroGuide::Event ZeroGuide::press(uint32_t now) {
  if (st_ == IDLE) {
    tries_ = 1;
    st_ = INTRO;
    t0_ = now;
    return EV_STARTED;
  }
  if (st_ == OUTRO && prompt_ != PR_MOVED) return EV_NONE;   // already saying the result
  say(now, PR_CANCEL, IDLE);
  return EV_CANCELLED;
}

ZeroGuide::Event ZeroGuide::window(uint32_t end_ms, bool still, const float grav[3], float g_out[3]) {
  if (st_ != MEASURE || int32_t(end_ms - 500 - t0_) < 0) return EV_NONE;   // only windows wholly in the quiet
  if (still) {
    for (int i = 0; i < 3; i++) g_[i] += grav[i];
    if (++n_ >= STILL_WINDOWS) {
      for (int i = 0; i < 3; i++) g_out[i] = g_[i] / n_;
      st_ = WAIT;
      return EV_MEASURED;
    }
    return EV_NONE;
  }
  n_ = 0;   // a move starts the 3 s over
  g_[0] = g_[1] = g_[2] = 0;
  if (end_ms - t0_ < MEASURE_MAX_MS) return EV_NONE;
  if (tries_ < TRIES) {
    tries_++;
    say(end_ms, PR_MOVED, COUNT);
    return EV_RETRY;
  }
  say(end_ms, PR_FAILED, IDLE);
  return EV_GAVE_UP;
}

void ZeroGuide::result(uint32_t now, bool set) {
  if (st_ == WAIT) say(now, set ? PR_DONE : PR_FAILED, IDLE);
}

int ZeroGuide::poll(uint32_t now) {
  uint32_t t = now - t0_;
  switch (st_) {
    case IDLE:
    case WAIT:
    case MEASURE:
      return -1;
    case INTRO:
      if (t < len_[PR_START] + SPARE_MS) return PR_START;
      st_ = COUNT;
      t0_ = now;
      return -1;
    case COUNT: {
      uint32_t k = t / COUNT_STEP_MS;
      if (k < 3) return PR_THREE + k;   // each word has its whole second
      if (t >= 3 * COUNT_STEP_MS + QUIET_MS) {
        st_ = MEASURE;
        t0_ = now;
        n_ = 0;
        g_[0] = g_[1] = g_[2] = 0;
      }
      return -1;
    }
    case OUTRO:
      if (t < len_[prompt_] + SPARE_MS) return prompt_;
      st_ = then_;
      t0_ = now;
      return -1;
  }
  return -1;
}

}  // namespace fhs
