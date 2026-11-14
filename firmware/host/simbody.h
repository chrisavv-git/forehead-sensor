// Host-side stand-ins for the hardware: NOR flash in RAM, a HAL with a virtual clock, and a body model that
// makes motion (default: a 6-axis sensor, 833 Hz accel + gyro; "watch": the T-Watch, 800 Hz 12-bit accel only) and 16 kHz mic samples from a scenario (head angles over time, a separate vibrating device ("bell") with its own
// rules, snoring). Used by the unit tests and by fhsim.
#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <string>
#include <vector>
#include "../core/device.h"

namespace sim {

struct MemFlash : fhs::Flash {
  std::vector<uint8_t> mem;
  std::vector<uint8_t> programs;   // program calls per 256-byte page since its sector was erased
  int max_programs = 0, violations = 0;
  bool fail = false;               // make every call fail (flash broken)
  explicit MemFlash(uint32_t size = 2u << 20) : mem(size, 0xFF), programs(size / 256, 0) {}
  uint32_t size() override { return uint32_t(mem.size()); }
  bool read(uint32_t a, void* b, uint32_t n) override {
    if (fail || a + n > mem.size()) return false;
    memcpy(b, &mem[a], n);
    return true;
  }
  bool program(uint32_t a, const void* b, uint32_t n) override {
    if (fail || a + n > mem.size()) return false;
    if (n == 0) return true;
    if (a / 256 != (a + n - 1) / 256) violations++;   // across a page boundary: real NOR wraps inside the page
    const uint8_t* p = static_cast<const uint8_t*>(b);
    for (uint32_t i = 0; i < n; i++) mem[a + i] &= p[i];   // can only clear bits
    uint8_t& c = programs[a / 256];
    if (c < 255) c++;
    if (c > max_programs) max_programs = c;
    return true;
  }
  bool erase_sector(uint32_t a) override {
    if (fail || a % 4096 || a >= mem.size()) return false;
    memset(&mem[a], 0xFF, 4096);
    memset(&programs[a / 256], 0, 16);
    return true;
  }
};

struct SimHal : fhs::Hal {
  uint32_t now = 0;
  uint8_t motor_level = 0;
  std::vector<std::pair<uint32_t, uint8_t>> motor_log;   // (ms, level) on every change
  uint16_t tone_hz = 0;
  uint8_t tone_vol = 0;
  std::vector<std::pair<uint32_t, uint8_t>> tone_log;    // (ms, volume) on every change
  std::vector<std::pair<uint32_t, uint16_t>> hz_log;     // (ms, hz) on every change: which phrase the zero says
  uint32_t prompt_len[fhs::N_PROMPTS] = {2830, 250, 240, 230, 880, 1500, 990, 510, 700};   // as the real clips
  std::vector<std::string> events;
  float imu_rate = 833, acc_lsb = 0.000122f;
  bool gyro = true;
  int64_t rtc_ms = 0;    // the watch's clock chip: 0 = not set; runs with now
  bool resumed_ = false; // the next boot is after a watchdog reset or crash
  uint16_t batt = 4000;
  bool usb = false, chg = false;
  bool have_params = false, have_state = false;
  fhs::Params saved_params;
  fhs::State saved_state;
  uint32_t rnd = 0x12345678;
  int reboots = 0;
  uint32_t ms_step = 0;   // > 0: the clock moves on by this much at every read inside one step (as on the watch,
                          // where writing the flash takes time); 0: it stands still within a step
  uint32_t ms_now = 0, ms_off = 0;
  uint32_t ms() override {
    if (now != ms_now) { ms_now = now; ms_off = 0; }
    ms_off += ms_step;
    return now + ms_off;
  }
  void motor(uint8_t pct) override { motor_level = pct; motor_log.push_back({now, pct}); }
  void tone(uint16_t hz, uint8_t vol) override {
    if (vol != tone_vol) tone_log.push_back({now, vol});
    if (hz != tone_hz) hz_log.push_back({now, hz});
    tone_hz = hz;
    tone_vol = vol;
  }
  void event(const char* msg) override { events.push_back(msg); }
  uint32_t prompt_ms(uint8_t id) override { return id < fhs::N_PROMPTS ? prompt_len[id] : 0; }
  float imu_hz() override { return imu_rate; }
  float acc_g_per_lsb() override { return acc_lsb; }
  bool has_gyro() override { return gyro; }
  int64_t rtc_at = 0;   // virtual ms when rtc_ms was set
  bool rtc_get(int64_t* t) override { if (!rtc_ms) return false; *t = rtc_ms + (now - rtc_at); return true; }
  void rtc_set(int64_t t) override { rtc_ms = t; rtc_at = now; }
  bool resumed() override { return resumed_; }
  uint16_t batt_mv() override { return batt; }
  bool usb_power() override { return usb; }
  bool charging() override { return chg; }
  bool load_params(fhs::Params& p) override { if (have_params) p = saved_params; return have_params; }
  bool save_params(const fhs::Params& p) override { saved_params = p; have_params = true; return true; }
  bool load_state(fhs::State& s) override { if (have_state) s = saved_state; return have_state; }
  bool save_state(const fhs::State& s) override { saved_state = s; have_state = true; return true; }
  uint32_t random32() override { rnd ^= rnd << 13; rnd ^= rnd >> 17; rnd ^= rnd << 5; return rnd; }
  void reboot() override { reboots++; }
  int stalled[fhs::N_PORTS] = {0, 0};   // link_stalled calls per port
  void link_stalled(int port) override { stalled[port]++; }
};

struct Rng {
  uint64_t s = 88172645463325252ull;
  double uni() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return (s >> 11) * (1.0 / 9007199254740992.0); }
  double gauss() { double u = uni() + 1e-12, v = uni(); return sqrt(-2 * log(u)) * cos(6.283185307179586 * v); }
};

// a separate vibrating positional device ("bell") with its own (hidden) rules, acting on the true head angle
struct Bell {
  double angle = 40, delay = 20, step = 60, freq = 200, amp_mg = 30;   // amp at 100 %
  int levels[4] = {25, 50, 75, 100};
  double on_s = 0.7, period_s = 1.0;   // buzz pattern
  double start_wait = 900, upright_pause = 300;   // its manual: quiet for 15 min after switching on, 5 min after sitting up
  double last_up = -1e9;
  double flat_since = -1;
  bool ringing = false;
  double ring_start = 0;
  std::vector<std::pair<double, double>> rings;   // (start s, end s) truth
  int level_at(double t, double roll, bool upright) {   // evaluated every 0.5 s; switched on at t = 0
    if (upright) last_up = t;
    bool flat = !upright && fabs(roll) < angle && t >= start_wait && t - last_up >= upright_pause;
    if (!flat) { if (ringing) rings.push_back({ring_start, t}); ringing = false; flat_since = -1; return 0; }
    if (flat_since < 0) flat_since = t;
    double d = t - flat_since;
    if (d < delay) return 0;
    if (!ringing) { ringing = true; ring_start = t; }
    int k = int((d - delay) / step);
    return levels[k < 3 ? k : 3];
  }
};

struct Key { double t, roll, pitch; };

struct Scenario {
  std::vector<Key> keys{{0, 0, 80}};
  std::vector<std::pair<double, double>> snore;    // (start, end) of snoring
  std::vector<std::pair<double, double>> usb_on;   // (start, end)
  std::vector<std::pair<double, double>> ble_down; // (start, end)
  std::vector<double> taps;
  double end = 0, snore_amp = 800, batt_from = 4100, batt_to = 3800;
  double mount[3] = {0, 0, 0};       // board on the head: roll, pitch, yaw offsets in degrees
  double bias[3] = {1.2, -1.8, 0.7}; // gyro zero-rate offset, deg/s
  // the motion sensor ("watch": BMA423 at 800 Hz, +-2 g in 12 bits, no gyro, noisier)
  double imu_hz = 833, imu_lsb = 0.000122, imu_noise = 0.0015;
  bool imu_gyro = true;
  int64_t t0_ms = 0;                 // unix ms at virtual 0 (0 = not given)
  Bell bell;
  bool has_bell = false;
  std::string error;
  bool parse(const std::string& text);
  void pose(double t, double* roll, double* pitch, size_t* cursor) const;   // cursor: speeds up rising t
  bool in(const std::vector<std::pair<double, double>>& v, double t) const {
    for (auto& p : v) if (t >= p.first && t < p.second) return true;
    return false;
  }
};

// samples for a time span; drives the device and the bell
class Body {
 public:
  Body(Scenario& sc, uint64_t seed = 1) : sc_(sc) { rng_.s ^= seed * 0x9E3779B97F4A7C15ull; init(); }
  // advance to virtual time t_ms, feeding the device every sample due; motor = our motor's level right now
  void run_to(fhs::Device& dev, uint32_t t_ms, uint8_t motor);
  int bell_level() const { return bell_level_; }
 private:
  void init();
  Scenario& sc_;
  Rng rng_;
  double t_imu_ = 0, t_mic_ = 0, next_bell_ = 0.5;
  size_t cur_ = 0, cur2_ = 0, tap_i_ = 0;
  uint64_t n_imu_ = 0, n_mic_ = 0;
  int bell_level_ = 0;
  double R_[3][3];
  // mic filters
  fhs::Biquad bp_hp_, bp_lp_;
  double bell_phase_ = 0, motor_phase_ = 0, mic_bell_phase_ = 0;
};

}  // namespace sim
