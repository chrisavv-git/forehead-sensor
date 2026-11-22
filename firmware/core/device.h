// The whole device behaviour, independent of the hardware: the Arduino sketch and the host simulator both feed it
// samples and drain its text links. Protocol: docs/PROTOCOL.md.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "format.h"
#include "params.h"
#include "sensing.h"
#include "therapy.h"
#include "ringlog.h"
#include "zeroguide.h"

namespace fhs {

#define FHS_FW_VERSION "0.0.5"

struct State {          // kept outside the log flash (internal flash on the chip), survives an erase
  uint32_t boot_id = 0;     // last boot id handed out
  uint32_t block_no = 0;    // block counter floor
  char key[17] = {0};       // Bluetooth key (16 hex digits); generated on the first boot
  int16_t tz_min = 0;       // local time offset from the last "time": the watch's clock chip keeps UTC across reboots
};

struct Hal {
  virtual uint32_t ms() = 0;
  virtual void motor(uint8_t pct) = 0;          // vibration, 0 = off
  virtual bool motor_diag(char* out, int n) { (void)out; (void)n; return false; }   // the driver's self-test, if any
  virtual void tone(uint16_t hz, uint8_t vol) { (void)hz; (void)vol; }   // sound, 0 Hz = silent,
                                                                          // hz_prompt(id) = a spoken zero phrase
  virtual uint32_t prompt_ms(uint8_t id) { (void)id; return 1000; }      // how long that phrase lasts
  virtual uint16_t batt_mv() = 0;               // 0 = can't tell
  virtual bool usb_power() = 0;
  virtual bool charging() = 0;
  virtual bool load_params(Params& p) = 0;
  virtual bool save_params(const Params& p) = 0;
  virtual bool load_state(State& s) = 0;
  virtual bool save_state(const State& s) = 0;
  virtual uint32_t random32() = 0;
  virtual bool chip_key(uint8_t k[8]) { (void)k; return false; }   // a key that is the same on every boot, if any
  virtual void reboot() = 0;
  virtual const char* info() { return ""; }    // hardware details for "status"
  virtual void event(const char* msg) { (void)msg; }   // the "N" notes, for the watch's own feedback
  // the motion sensor: sample rate, accel scale, and whether there is a gyro (the watch's BMA423 has none: no
  // gyro values, movement then comes from the accelerometer)
  virtual float imu_hz() { return 833; }
  virtual float acc_g_per_lsb() { return 0.000122f; }   // LSM6DS3TR-C at +-4 g
  virtual bool has_gyro() { return true; }
  // a clock that runs through reboots (UTC); false = none, or it lost the time
  virtual bool rtc_get(int64_t* unix_ms) { (void)unix_ms; return false; }
  virtual void rtc_set(int64_t unix_ms) { (void)unix_ms; }
  // the last reset was the watchdog (a hang) or a crash, not a power-up or a reboot asked for: carry on at once
  virtual bool resumed() { return false; }
  // nothing was taken from this link for TX_STALL_MS (the laptop closed the port, or just a charger): the core has
  // dropped what it was sending; the hardware layer drops what its own driver still holds
  virtual void link_stalled(int port) { (void)port; }
  virtual ~Hal() {}
};

class TxRing {   // outgoing bytes of one link; the hardware layer drains it as fast as the link takes them
 public:
  static constexpr size_t SIZE = 4096;
  size_t room() const { return SIZE - 1 - count(); }
  size_t count() const { return (head_ + SIZE - tail_) % SIZE; }
  bool put(const char* s, size_t n);    // all or nothing
  size_t peek(const char** p) const;    // contiguous bytes ready to send
  void drop(size_t n) { tail_ = (tail_ + n) % SIZE; taken_ += uint32_t(n); }
  uint32_t taken() const { return taken_; }   // bytes the link has taken, ever
  void clear() { head_ = tail_ = 0; }
 private:
  char buf_[SIZE];
  size_t head_ = 0, tail_ = 0;
  uint32_t taken_ = 0;
};

enum PortId { PORT_USB = 0, PORT_BLE = 1, N_PORTS = 2 };

class Device {
 public:
  void begin(Hal* hal, Flash* flash);
  // inputs from the hardware layer
  void imu(const int16_t raw[6]);               // one sample: ax ay az gx gy gz (gyro 0 without one)
  void mic(const int16_t* pcm, int n);
  void mic_start() { sense_.mic_start(); }      // the mic was switched on again (a burst)
  uint32_t seq0_ms() const { return seq0_ms_; }   // when window 0 began (hal ms): the hardware times its mic bursts on it
  void double_tap();                            // the button asked for the spoken zero (starts it, or cancels it)
  // the button (the watch's power chip reports press and release). A DOUBLE PRESS (two short presses, the second
  // within DOUBLE_GAP_MS of letting go of the first) starts the spoken zero, with a short buzz the moment it is
  // taken; the same again while it talks cancels it. (1 Nov: the long press of 2.5-5 s gave no signal while held:
  // "nothing was happening ... i dont know when to release"; holding to zero was then dropped the same day: "makes
  // no sense, the double click works perfectly".) Holding does nothing until 5 s: "Switching off." (a zero still
  // talking is stopped; the power chip switches off at 10 s); let go before 10 s = it stays on.
  void button(bool down);
  static constexpr uint32_t OFF_WARN_MS = 5000;
  static constexpr uint32_t DOUBLE_MAX_PRESS_MS = 600;   // a press this short counts toward a double press
  static constexpr uint32_t DOUBLE_GAP_MS = 800;         // from letting go of the first to pressing the second
  // bytes waiting on a link and none taken for this long: nobody is reading. The job (catch-up, dump, block list),
  // live and raw stop and the waiting bytes go, so the watch does not keep pushing into a closed port (the USB chip
  // cannot see the laptop close it; the cable going out is link()). fhs live asks again after 10 s of silence
  static constexpr uint32_t TX_STALL_MS = 5000;
  static constexpr uint32_t ACK_MS = 250;                // the buzz that says the button was taken (full strength)
  void rx(int port, const char* data, size_t n);   // bytes received on a link
  void link(int port, bool up);                 // link connected / dropped
  void poll();                                  // call often (every loop)
  TxRing& tx(int port) { return ports_[port].tx; }
  // for tests and the simulator
  const Params& params() const { return p_; }
  RingLog& log() { return log_; }
  uint32_t seq() const { return seq_; }
  bool session() const { return session_; }
  bool zeroing() const { return guide_.active(); }
  // something is being played or said right now, the silent gaps of its pattern included: the hardware layer must
  // not sleep through those (a nap in a gap started the next pulse late, and it still ended on time)
  bool playing() const { return playing_; }
  bool raw_imu_on() const { return ports_[PORT_USB].raw_imu; }
  bool raw_mic_on() const { return ports_[PORT_USB].raw_mic; }
  const Record& last() const { return last_; }

 private:
  struct Job {
    enum Kind { NONE, SINCE, DUMP, BLOCKS } kind = NONE;
    int sector = -1;
    uint32_t block_no = 0, boot = 0, seq = 0, off = 0, count = 0, limit = 0;   // limit: blocks to dump (0 = all)
    bool then_live = false;
  };
  struct Port {
    TxRing tx;
    char line[192];
    size_t len = 0;
    bool overflow = false, up = false, live = false, authed = false, raw_imu = false, raw_mic = false;
    Job job;
    uint32_t dropped = 0;   // live records that didn't fit (the laptop catches up with "since")
    uint32_t taken_seen = 0, tx_ms = 0;   // the ring's taken() when it last moved, and when that was
    uint32_t stalls = 0;    // times the link was given up as not read (TX_STALL_MS)
  };
  void tick();
  void command(int port, char* line);
  bool send(int port, const char* s);           // a whole line (adds \n); false (dropped) if the link is full
  void sendf(int port, const char* fmt, ...);
  void note(const char* msg);
  void step_job(int port);
  void record_line(char* out, uint32_t boot, uint32_t seq, const Record& r);
  void status(int port);
  void params_line(int port);
  bool set_param(const char* key, const char* val);
  void update_session(bool lying, uint32_t now);
  void start_session(uint32_t now, uint32_t backfill, bool skip_wait = false);
  void end_session();
  bool set_zero(const float g[3], bool by_tap);   // false: no usable gravity
  void set_clock(int64_t unix_ms, int16_t tz_min);
  BlockInfo info() const;
  int64_t unix_ms_of(uint32_t seq) const { return t_seq0_ms_ + int64_t(seq) * WINDOW_MS; }

  Hal* hal_ = nullptr;
  Params p_;
  State st_;
  Sensing sense_;
  Therapy therapy_;
  RingLog log_;
  Port ports_[N_PORTS];
  Record last_{};
  // history of the last windows, so a session can start from the moment lying began
  static constexpr int HIST = 256;
  Record hist_[HIST];
  uint32_t seq_ = 0, seq0_ms_ = 0;
  int64_t t_seq0_ms_ = 0;
  int16_t tz_min_ = 0;
  bool clock_ = false, session_ = false, usb_ = false, cutoff_ = false;
  bool resume_ = false;   // came back from a hang or crash: lying down starts nudging at once
  uint32_t session_ms_ = 0;
  uint32_t lie_n_ = 0, unlie_n_ = 0;            // windows lying / not lying in a row
  uint16_t batt_ = 0;
  float batt_f_ = 0;
  // outputs; a test is a buzz (test_level_ %) or a whole nudge step (test_step_, test_variant_)
  uint8_t test_level_ = 0, test_step_ = 0, test_variant_ = 0, motor_out_ = 0, vol_out_ = 0;
  uint16_t hz_out_ = 0;
  uint32_t test_until_ = 0, test_ms_ = 0;
  bool test_in_window_ = false, playing_ = false;
  // the bedtime buzz when a session starts: two short soft taps (on duty) or three long ones (battery low)
  bool cue_ = false, cue_in_window_ = false;
  uint8_t cue_variant_ = 0;   // 0 on duty, 1 battery low
  uint32_t cue_ms_ = 0;
  // the spoken zero (the button): the guide keeps the timeline, the device sets the zero and holds the nudges
  static constexpr uint32_t ZERO_HOLD_MS = 300000;   // no nudge for 5 min after it (vest, mask, settling in)
  ZeroGuide guide_;
  bool tap_flag_ = false, zero_flag_ = false, guide_in_window_ = false;
  bool short_pending_ = false, second_ = false;   // a first short press waits for its second; this is the second
  uint32_t short_up_ms_ = 0;
  bool ack_ = false;                         // the "taken" buzz is on, from ack_ms_
  uint32_t ack_ms_ = 0;
  bool btn_down_ = false, off_warned_ = false;
  uint32_t btn_ms_ = 0;
  int say_id_ = -1;          // a phrase said on its own (not the zero guide): PR_OFF
  uint32_t say_ms_ = 0;
  uint8_t guide_prompt_ = 0;
  float g_hist_[4][3];
  float m_hist_[4] = {99, 99, 99, 99};
  // raw samples for the bench test (USB only)
  uint8_t rawi_[240];
  int rawi_n_ = 0;
  int16_t rawm_[128];
  int rawm_n_ = 0;
};

bool params_valid(const Params& p);   // every field in range (settings read back from flash are checked)

// small helpers shared with the simulator
void hex_encode(char* out, const void* data, size_t n);   // 2n chars + NUL
int u64_to_dec(char* out, uint64_t v);
int i64_to_dec(char* out, int64_t v);
int fixed_to_dec(char* out, float v, int decimals);

}  // namespace fhs
