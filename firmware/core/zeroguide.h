// The spoken zero: a double press of the button starts it (Device::button). The watch says what it does
// ("Zeroing. Lie flat on your back, and keep still."), counts down ("Three. Two. One."), measures 3 s of stillness
// in silence (its own speaker shakes the motion sensor), then says the result ("Zero set.", "You moved. Let's try
// again." up to 3 tries, "Zero not set."). Any head position counts (no limit to the old zero: the voice tells you it
// is zeroing, 31 Oct). Another double press while it runs: "Cancelled."
// It only keeps the timeline and the stillness count; Device sets the zero, mutes the nudges and logs the windows.
#pragma once
#include <stdint.h>

namespace fhs {

// the phrases, in the order of the clips in firmware/twatch/prompts.h (fhs voice makes them)
enum Prompt : uint8_t {
  PR_START = 0,   // "Zeroing. Lie flat on your back, and keep still."
  PR_THREE,       // "Three."
  PR_TWO,         // "Two."
  PR_ONE,         // "One."
  PR_DONE,        // "Zero set."
  PR_MOVED,       // "You moved. Let's try again."
  PR_FAILED,      // "Zero not set."
  PR_CANCEL,      // "Cancelled."
  PR_OFF,         // "Switching off." (the button held past 5 s: the power chip cuts the power at 10 s)
  N_PROMPTS
};
// a tone of this "frequency" plays prompt clip id, from its start
constexpr uint16_t HZ_PROMPT_BASE = 0xFF00;
inline uint16_t hz_prompt(uint8_t id) { return uint16_t(HZ_PROMPT_BASE | id); }
inline bool is_prompt_hz(uint16_t hz) { return (hz & 0xFF00) == HZ_PROMPT_BASE && (hz & 0xFF) < N_PROMPTS; }

class ZeroGuide {
 public:
  static constexpr uint32_t COUNT_STEP_MS = 1000;   // "three", "two", "one" a second apart
  // a phrase is held this long past its length, before the next: the clock starts at the press, but the audio can
  // start late (the screen waking blocks the loop) and whatever hadn't played when the time ran out was lost
  static constexpr uint32_t SPARE_MS = 1000;
  static constexpr uint32_t QUIET_MS = 300;         // after the last word, before measuring (the speaker rings out)
  static constexpr int STILL_WINDOWS = 6;           // 3 s still in a row
  static constexpr uint32_t MEASURE_MAX_MS = 10000; // no 3 s still within this: "You moved"
  static constexpr int TRIES = 3;

  enum Event { EV_NONE, EV_STARTED, EV_CANCELLED, EV_MEASURED, EV_RETRY, EV_GAVE_UP };

  void lengths(const uint32_t ms[N_PROMPTS]) { for (int i = 0; i < N_PROMPTS; i++) len_[i] = ms[i]; }
  bool active() const { return st_ != IDLE; }
  int tries() const { return tries_; }
  // the button: starts the guide, or cancels a running one (not while it says the result)
  Event press(uint32_t now);
  // silently, at once (the button held into "Switching off.")
  void stop() { st_ = IDLE; n_ = 0; g_[0] = g_[1] = g_[2] = 0; }
  // every 0.5 s window, [end_ms - 500, end_ms): EV_MEASURED (g_out = mean gravity) after 3 s still, EV_RETRY or
  // EV_GAVE_UP when no 3 s were still in time
  Event window(uint32_t end_ms, bool still, const float grav[3], float g_out[3]);
  // after EV_MEASURED: Device set the zero ("Zero set."), or couldn't (no gravity reading: "Zero not set.")
  void result(uint32_t now, bool set);
  // call often: moves the timeline on; the prompt to play right now, or -1 for silence
  int poll(uint32_t now);

 private:
  enum State { IDLE, INTRO, COUNT, MEASURE, WAIT, OUTRO };
  void say(uint32_t now, uint8_t id, State then);
  State st_ = IDLE, then_ = IDLE;
  uint32_t t0_ = 0;
  uint8_t prompt_ = 0;
  int tries_ = 0, n_ = 0;
  float g_[3] = {0, 0, 0};
  uint32_t len_[N_PROMPTS] = {3000, 700, 700, 700, 1000, 2000, 1200, 1000, 1000};
};

}  // namespace fhs
