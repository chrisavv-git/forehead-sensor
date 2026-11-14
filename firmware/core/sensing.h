// Turns raw motion and mic samples into one Record per 0.5 s window.
#pragma once
#include <stdint.h>
#include "format.h"
#include "params.h"

namespace fhs {

constexpr float MIC_HZ = 16000;
constexpr float GYR_DPS_PER_LSB = 0.0175f;    // LSM6DS3TR-C at +-500 deg/s

struct Biquad {   // direct form I, coefficients from the RBJ cookbook
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  void highpass(float fc, float fs);
  void lowpass(float fc, float fs);
  void reset() { x1 = x2 = y1 = y2 = 0; }
  float step(float x) {
    float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = x; y2 = y1; y1 = y;
    return y;
  }
};

// head angles from a gravity vector in board axes, given the zero and the head axis
void head_angles(const float g[3], const float zero[3], uint8_t head_axis, float* roll_deg, float* pitch_deg);

class Sensing {
 public:
  Sensing();
  // the motion sensor: sample rate, accel scale; no gyro = movement from how fast gravity's direction turns
  void config(float imu_hz, float acc_g_per_lsb, bool gyro);
  void imu(const int16_t raw[6]);              // one sample: ax ay az gx gy gz, raw LSB
  void mic(const int16_t* pcm, int n);         // 16 kHz mono
  // the mic was switched on again (a burst): its filters start fresh at the first sample's level, and the next
  // MIC_SETTLE_N samples only settle them. Carried over, the jump between bursts rang as 750-930 "snore" in
  // silence (30-31 Oct)
  void mic_start() { mic_primed_ = false; mic_skip_ = MIC_SETTLE_N; }
  static constexpr int MIC_SETTLE_N = 256;     // 16 ms
  // close the 0.5 s window: fills roll, pitch, move, buzz, mic, snore and the F_UPRIGHT/F_SHORT/F_NOMIC flags
  void close(Record& r, const Params& p);
  // gravity (board axes, unit vector) and movement of the last closed window
  float grav[3] = {0, 0, 1};
  float move_dps = 0;
  int last_imu_n = 0, last_mic_n = 0;
  bool have_grav = false;

 private:
  float hz_ = 833, g_lsb_ = 0.000122f;
  bool gyro_ = true;
  Biquad hp_[3];                 // accel high-pass for the buzz
  Biquad mic_dc_, snore_hp_, snore_lp_;
  bool mic_primed_ = true;
  int mic_skip_ = 0;
  double acc_sum_[3] = {0, 0, 0}, a_sq_[3] = {0, 0, 0}, w_sum_[3] = {0, 0, 0}, w_sq_[3] = {0, 0, 0};
  double buzz_sq_ = 0, mic_sq_ = 0, snore_sq_ = 0;
  int n_imu_ = 0, n_mic_ = 0, buzz_n_ = 0, settle_ = 0;
  float bias_[3] = {0, 0, 0};    // gyro zero-rate offset, learnt while still
  // no gyro: gravity's direction in 50 ms pieces, and the squared turn rate between them
  double sub_[3] = {0, 0, 0}, turn_sq_ = 0;
  float prev_dir_[3] = {0, 0, 0};
  int sub_n_ = 0, sub_len_ = 40, turn_n_ = 0;
  bool have_prev_ = false;
  bool bias_init_ = false;
  float roll_ = 0, pitch_ = 0;
};

}  // namespace fhs
