#include "simbody.h"
#include <sstream>

namespace sim {

static constexpr double D2R = 3.141592653589793 / 180;

bool Scenario::parse(const std::string& text) {
  std::istringstream in(text);
  std::string line;
  double t = 0;
  bool usb = false, ble = true;
  double usb_t = 0, ble_t = 0;
  int ln = 0;
  while (std::getline(in, line)) {
    ln++;
    size_t h = line.find('#');
    if (h != std::string::npos) line.resize(h);
    std::istringstream w(line);
    std::string cmd;
    if (!(w >> cmd)) continue;
    auto bad = [&](const char* why) { error = "line " + std::to_string(ln) + ": " + why; return false; };
    if (cmd == "t0") { double s; if (!(w >> s)) return bad("t0 <unix s>"); t0_ms = int64_t(s * 1000); }
    else if (cmd == "mount") { if (!(w >> mount[0] >> mount[1] >> mount[2])) return bad("mount <roll> <pitch> <yaw>"); }
    else if (cmd == "bias") { if (!(w >> bias[0] >> bias[1] >> bias[2])) return bad("bias <x> <y> <z>"); }
    else if (cmd == "watch") { imu_hz = 800; imu_lsb = 1.0 / 1024; imu_noise = 0.0025; imu_gyro = false; }
    else if (cmd == "bell") {
      if (!(w >> bell.angle >> bell.delay >> bell.step >> bell.freq >> bell.amp_mg)) return bad("bell <angle> <delay> <step> <hz> <mg>");
      has_bell = true;
    } else if (cmd == "bellwait") {
      if (!(w >> bell.start_wait >> bell.upright_pause)) return bad("bellwait <start s> <after sitting up s>");
    } else if (cmd == "batt") { if (!(w >> batt_from >> batt_to)) return bad("batt <from mV> <to mV>"); }
    else if (cmd == "snore_amp") { if (!(w >> snore_amp)) return bad("snore_amp <pcm>"); }
    else if (cmd == "tap") taps.push_back(t);
    else if (cmd == "usb" || cmd == "ble") {
      std::string v;
      w >> v;
      bool on = v == "on";
      if (v != "on" && v != "off") return bad("usb|ble on|off");
      if (cmd == "usb") { if (on && !usb) usb_t = t; if (!on && usb) usb_on.push_back({usb_t, t}); usb = on; }
      else { if (!on && ble) ble_t = t; if (on && !ble) ble_down.push_back({ble_t, t}); ble = on; }
    } else if (cmd == "seg") {
      double dur;
      if (!(w >> dur) || dur <= 0) return bad("seg <seconds> ...");
      double r = keys.back().roll, p = keys.back().pitch;
      bool ramp = false, sn = false;
      std::string k;
      while (w >> k) {
        if (k == "roll") { if (!(w >> r)) return bad("roll <deg>"); }
        else if (k == "pitch") { if (!(w >> p)) return bad("pitch <deg>"); }
        else if (k == "ramp") ramp = true;
        else if (k == "snore") sn = true;
        else return bad("unknown seg option");
      }
      if (ramp) keys.push_back({t + dur, r, p});
      else { keys.push_back({t + (dur < 1 ? dur : 1), r, p}); keys.push_back({t + dur, r, p}); }
      if (sn) snore.push_back({t, t + dur});
      t += dur;
    } else return bad("unknown command");
  }
  if (usb) usb_on.push_back({usb_t, t + 1e9});
  if (!ble) ble_down.push_back({ble_t, t + 1e9});
  end = t;
  return true;
}

void Scenario::pose(double t, double* roll, double* pitch, size_t* cur) const {
  size_t i = cur ? *cur : 0;
  if (i >= keys.size() || keys[i].t > t) i = 0;
  while (i + 1 < keys.size() && keys[i + 1].t <= t) i++;
  if (cur) *cur = i;
  if (i + 1 >= keys.size()) { *roll = keys.back().roll; *pitch = keys.back().pitch; return; }
  const Key &a = keys[i], &b = keys[i + 1];
  double f = b.t > a.t ? (t - a.t) / (b.t - a.t) : 1;
  *roll = a.roll + (b.roll - a.roll) * f;
  *pitch = a.pitch + (b.pitch - a.pitch) * f;
}

void Body::init() {
  // board axes from head axes: a_board = R * a_head (roll about X, pitch about Y, then yaw about Z)
  double r = sc_.mount[0] * D2R, p = sc_.mount[1] * D2R, y = sc_.mount[2] * D2R;
  double Rx[3][3] = {{1, 0, 0}, {0, cos(r), -sin(r)}, {0, sin(r), cos(r)}};
  double Ry[3][3] = {{cos(p), 0, sin(p)}, {0, 1, 0}, {-sin(p), 0, cos(p)}};
  double Rz[3][3] = {{cos(y), -sin(y), 0}, {sin(y), cos(y), 0}, {0, 0, 1}};
  double T[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) { T[i][j] = 0; for (int k = 0; k < 3; k++) T[i][j] += Ry[i][k] * Rx[k][j]; }
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) { R_[i][j] = 0; for (int k = 0; k < 3; k++) R_[i][j] += Rz[i][k] * T[k][j]; }
  bp_hp_.highpass(100, 16000);
  bp_lp_.lowpass(600, 16000);
}

static int16_t clamp16(double v) { return v > 32767 ? 32767 : v < -32768 ? -32768 : int16_t(lrint(v)); }

void Body::run_to(fhs::Device& dev, uint32_t t_ms, uint8_t motor) {
  double t = t_ms / 1000.0;
  while (tap_i_ < sc_.taps.size() && sc_.taps[tap_i_] <= t) { dev.double_tap(); tap_i_++; }
  const double dt = 1.0 / sc_.imu_hz;
  while (t_imu_ < t) {
    double roll, pitch, r2, p2;
    sc_.pose(t_imu_, &roll, &pitch, &cur_);
    sc_.pose(t_imu_ + 0.02, &r2, &p2, &cur2_);
    while (sc_.has_bell && t_imu_ >= next_bell_) {
      bell_level_ = sc_.bell.level_at(next_bell_, roll, pitch > 60);
      next_bell_ += 0.5;
    }
    double rr = roll * D2R, pp = pitch * D2R;
    double ah[3] = {sin(pp), sin(rr) * cos(pp), cos(rr) * cos(pp)};   // board Y = Z x X: the right ear
    bool bell_on = bell_level_ && fmod(t_imu_, sc_.bell.period_s) < sc_.bell.on_s;
    if (bell_on) {
      double amp = sc_.bell.amp_mg / 1000 * bell_level_ / 100, s = sin(bell_phase_);
      ah[2] += amp * s;
      ah[0] += 0.3 * amp * s;
    }
    bell_phase_ += 2 * 3.141592653589793 * sc_.bell.freq * dt;
    if (motor) ah[2] += 0.07 * motor / 100 * sin(motor_phase_);
    motor_phase_ += 2 * 3.141592653589793 * 175 * dt;
    double wh[3] = {(r2 - roll) / 0.02, (p2 - pitch) / 0.02, 0};
    int16_t raw[6];
    for (int i = 0; i < 3; i++) {
      double a = 0, w = 0;
      for (int k = 0; k < 3; k++) { a += R_[i][k] * ah[k]; w += R_[i][k] * wh[k]; }
      a += sc_.imu_noise * rng_.gauss();
      w += sc_.bias[i] + 0.08 * rng_.gauss();
      raw[i] = clamp16(a / sc_.imu_lsb);
      raw[3 + i] = sc_.imu_gyro ? clamp16(w / fhs::GYR_DPS_PER_LSB) : 0;
    }
    dev.imu(raw);
    n_imu_++;
    t_imu_ = n_imu_ * dt;
  }
  int16_t buf[160];
  while (t_mic_ < t) {
    double roll, pitch, r2, p2;
    sc_.pose(t_mic_, &roll, &pitch, nullptr);
    sc_.pose(t_mic_ + 0.1, &r2, &p2, nullptr);
    bool moving = fabs(r2 - roll) + fabs(p2 - pitch) > 0.5;   // > 5 deg/s: bedding rustles
    bool sn = sc_.in(sc_.snore, t_mic_);
    for (int k = 0; k < 160; k++) {
      double tt = (n_mic_ + k) / 16000.0, x = 15 * rng_.gauss();
      double nb = bp_lp_.step(bp_hp_.step(rng_.gauss()));
      if (sn) {   // one snore of 1.3 s every 4 s
        double ph = fmod(tt, 4.0);
        if (ph < 1.3) x += sc_.snore_amp * 3 * nb * sin(3.141592653589793 * ph / 1.3);
      }
      if (moving) x += 300 * rng_.gauss();
      if (bell_level_ && fmod(tt, sc_.bell.period_s) < sc_.bell.on_s) x += 120.0 * bell_level_ / 100 * sin(mic_bell_phase_);
      mic_bell_phase_ += 2 * 3.141592653589793 * sc_.bell.freq / 16000;
      if (motor) x += 250.0 * motor / 100 * sin(2 * 3.141592653589793 * 175 * tt);
      buf[k] = clamp16(x);
    }
    dev.mic(buf, 160);
    n_mic_ += 160;
    t_mic_ = n_mic_ / 16000.0;
  }
}

}  // namespace sim
