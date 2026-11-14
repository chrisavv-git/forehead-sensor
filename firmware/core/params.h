// Settings that persist across reboots and can be changed from the laptop ("set <key> <value>").
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace fhs {

enum Mode : uint8_t { MODE_SHADOW = 0, MODE_THERAPY = 1 };

struct Params {
  // therapy rules: flat for delay_s, then step 1; every step_s the next step, up to step 4, which repeats until the
  // head turns. A steady buzz gets ignored over time, so the steps escalate from vibration to sound and
  // every step plays a randomly picked variant (vibration pattern + sound) at a slightly random time
  float angle_deg = 60;          // "flat": head within this many degrees of face up (|roll|), and not upright. 60, not
                                 // 45: the half-back positions in between must wake too (side ~90)
  float hyst_deg = 5;            // once flat, stays flat until |roll| > angle_deg + hyst_deg
  float upright_deg = 60;        // pitch above this = sitting up / standing
  uint16_t delay_s = 10;         // flat this long before step 1 (the flat stays to catch can last only ~20 s)
  uint16_t step_s = 10;          // the next step comes after this long
  uint16_t start_wait_s = 900;   // no nudge for this long after a night session starts (time to fall asleep)
  uint16_t upright_pause_s = 300;// no nudge for this long after sitting up
  uint8_t level[4] = {50, 100, 100, 100};   // vibration % in steps 1-4
  uint8_t volume[4] = {0, 0, 30, 100};      // sound volume % in steps 1-4 (0 = silent)
  uint8_t jitter_s = 3;          // the delay and every step move by up to this much, at random
  uint8_t vary = 1;              // 1: a random variant each step; 0: always variant 0 (a steady pulse)
  uint16_t max_buzz_s = 0;       // give up after this long of nudging in one flat stay (0 = never)
  uint8_t mode = MODE_SHADOW;    // shadow: nothing runs (the would-be nudge is logged); therapy: it nudges

  // angle zero (set by "zero", or the button (a double press) while lying flat on the back)
  uint8_t head_axis = 0;         // board axis that points to the top of the head: 0 +X, 1 -X, 2 +Y, 3 -Y
  float zero[3] = {0, 0, 1};     // gravity direction (board axes, unit) when lying flat on the back
  uint8_t zero_set = 0;          // 0 = still the default (board flat, chip side away from the skin)
  uint8_t tap_zero = 1;          // the button (a double press) starts the spoken zero (zeroguide.h)
  float still_dps = 15;          // movement below this counts as still (zero). 4 refused every bedtime zero on
                                 // 30 Oct: breathing and the head settling into the pillow are 5-15 deg/s

  // logging gate: only log while it is worth it, so the ring (15 MB, ~134 h of records) holds whole nights
  uint8_t log_usb = 0;           // also log while on USB power (default: on the charger = not worn, no log)
  uint8_t win_from_h = 0;        // with the clock set, only log between these local hours (from == to: always).
  uint8_t win_to_h = 0;          // Always by default: naps count too, and abroad the watch can't know the local time
  uint8_t lie_gate = 1;          // only log while lying: a session starts after 2 min lying, ends after 20 min not
  uint16_t low_mv = 3500;        // F_LOWBAT below this
  uint16_t cutoff_mv = 3300;     // below this (off USB): stop logging and nudging, to end the night cleanly
  uint16_t night_mv = 3800;      // the bedtime buzz warns (three long) below this (~40 %): not enough for a night
  uint8_t mic_duty = 25;         // % of every 0.5 s window the mic listens (the chip sleeps the rest)
};

// key table for "get"/"set": every key, its type, range and the field it changes
enum ParamType : uint8_t { PT_F32, PT_U8, PT_U16, PT_LEVELS };   // PT_LEVELS: 4 x uint8, "a,b,c,d"
struct ParamDef {
  const char* key;
  ParamType type;
  float lo, hi;
  size_t offset;
};

#define FHS_P(k, t, lo, hi, field) {k, t, lo, hi, offsetof(Params, field)}
static const ParamDef PARAM_DEFS[] = {
    FHS_P("angle", PT_F32, 5, 90, angle_deg),
    FHS_P("hyst", PT_F32, 0, 20, hyst_deg),
    FHS_P("upright", PT_F32, 30, 85, upright_deg),
    FHS_P("delay", PT_U16, 0, 600, delay_s),
    FHS_P("step", PT_U16, 5, 600, step_s),
    FHS_P("start_wait", PT_U16, 0, 7200, start_wait_s),
    FHS_P("upright_pause", PT_U16, 0, 3600, upright_pause_s),
    FHS_P("levels", PT_LEVELS, 0, 100, level),
    FHS_P("volumes", PT_LEVELS, 0, 100, volume),
    FHS_P("jitter", PT_U8, 0, 30, jitter_s),
    FHS_P("vary", PT_U8, 0, 1, vary),
    FHS_P("max_buzz", PT_U16, 0, 3600, max_buzz_s),
    FHS_P("tap_zero", PT_U8, 0, 1, tap_zero),
    FHS_P("still", PT_F32, 0.5, 50, still_dps),
    FHS_P("log_usb", PT_U8, 0, 1, log_usb),
    FHS_P("win_from", PT_U8, 0, 23, win_from_h),
    FHS_P("win_to", PT_U8, 0, 23, win_to_h),
    FHS_P("lie_gate", PT_U8, 0, 1, lie_gate),
    FHS_P("low_mv", PT_U16, 3000, 4200, low_mv),
    FHS_P("cutoff_mv", PT_U16, 2900, 4000, cutoff_mv),
    FHS_P("night_mv", PT_U16, 3300, 4200, night_mv),
    FHS_P("mic_duty", PT_U8, 10, 100, mic_duty),
};
#undef FHS_P
constexpr int N_PARAM_DEFS = sizeof(PARAM_DEFS) / sizeof(PARAM_DEFS[0]);

}  // namespace fhs
