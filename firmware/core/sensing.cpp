#include "sensing.h"
#include <math.h>

namespace fhs {

static constexpr float PI_F = 3.14159265f;
static constexpr float DEG = 180.0f / PI_F;

void Biquad::highpass(float fc, float fs) {
  float w = 2 * PI_F * fc / fs, c = cosf(w), al = sinf(w) / (2 * 0.70710678f), a0 = 1 + al;
  b0 = (1 + c) / 2 / a0; b1 = -(1 + c) / a0; b2 = b0; a1 = -2 * c / a0; a2 = (1 - al) / a0;
}

void Biquad::lowpass(float fc, float fs) {
  float w = 2 * PI_F * fc / fs, c = cosf(w), al = sinf(w) / (2 * 0.70710678f), a0 = 1 + al;
  b0 = (1 - c) / 2 / a0; b1 = (1 - c) / a0; b2 = b0; a1 = -2 * c / a0; a2 = (1 - al) / a0;
}

static void norm3(float v[3]) {
  float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (n > 1e-6f) { v[0] /= n; v[1] /= n; v[2] /= n; }
}

void head_angles(const float g[3], const float zero[3], uint8_t head_axis, float* roll_deg, float* pitch_deg) {
  // head frame: Z out of the forehead (= gravity's direction when lying flat on the back), X to the top of the
  // head, Y = X x Z to the left ear (Z x X is the right ear). The accelerometer reads +1 g pointing up.
  float Z[3] = {zero[0], zero[1], zero[2]};
  norm3(Z);
  static const float AX[4][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}};
  const float* h = AX[head_axis < 4 ? head_axis : 0];
  float d = h[0] * Z[0] + h[1] * Z[1] + h[2] * Z[2];
  float X[3] = {h[0] - d * Z[0], h[1] - d * Z[1], h[2] - d * Z[2]};
  if (X[0] * X[0] + X[1] * X[1] + X[2] * X[2] < 0.01f) {   // head axis nearly along the zero: use the board axis next to it
    const float* h2 = AX[(head_axis + 2) % 4];
    d = h2[0] * Z[0] + h2[1] * Z[1] + h2[2] * Z[2];
    X[0] = h2[0] - d * Z[0]; X[1] = h2[1] - d * Z[1]; X[2] = h2[2] - d * Z[2];
  }
  norm3(X);
  float Y[3] = {X[1] * Z[2] - X[2] * Z[1], X[2] * Z[0] - X[0] * Z[2], X[0] * Z[1] - X[1] * Z[0]};
  float ax = X[0] * g[0] + X[1] * g[1] + X[2] * g[2];
  float ay = Y[0] * g[0] + Y[1] * g[1] + Y[2] * g[2];
  float az = Z[0] * g[0] + Z[1] * g[1] + Z[2] * g[2];
  *roll_deg = atan2f(-ay, az) * DEG;                      // + = left ear down
  *pitch_deg = atan2f(ax, sqrtf(ay * ay + az * az)) * DEG;   // + = top of the head up
}

Sensing::Sensing() {
  config(833, 0.000122f, true);
  mic_dc_.highpass(20, MIC_HZ);
  snore_hp_.highpass(100, MIC_HZ);
  snore_lp_.lowpass(600, MIC_HZ);
}

void Sensing::config(float imu_hz, float acc_g_per_lsb, bool gyro) {
  hz_ = imu_hz;
  g_lsb_ = acc_g_per_lsb;
  gyro_ = gyro;
  for (auto& b : hp_) { b.highpass(80, hz_); b.reset(); }
  // no gyro: the turn rate from the gravity direction every 125 ms (over 50 ms the accelerometer noise read as
  // 2-9 deg/s on a still table, above still_dps, so the zero was refused)
  sub_len_ = int(hz_ / 8 + 0.5f);
  settle_ = 0;
}

void Sensing::imu(const int16_t raw[6]) {
  float a[3], w[3];
  for (int i = 0; i < 3; i++) {
    a[i] = raw[i] * g_lsb_;
    w[i] = raw[3 + i] * GYR_DPS_PER_LSB;
  }
  if (!gyro_) {
    for (int i = 0; i < 3; i++) sub_[i] += a[i];
    if (++sub_n_ >= sub_len_) {
      float d[3] = {float(sub_[0]), float(sub_[1]), float(sub_[2])};
      norm3(d);
      if (have_prev_) {   // the angle between two unit vectors, as atan2(|cross|, dot): exact for small angles too
        float cx = prev_dir_[1] * d[2] - prev_dir_[2] * d[1], cy = prev_dir_[2] * d[0] - prev_dir_[0] * d[2],
              cz = prev_dir_[0] * d[1] - prev_dir_[1] * d[0];
        float dot = prev_dir_[0] * d[0] + prev_dir_[1] * d[1] + prev_dir_[2] * d[2];
        double rate = atan2f(sqrtf(cx * cx + cy * cy + cz * cz), dot) * DEG * hz_ / sub_n_;
        turn_sq_ += rate * rate;
        turn_n_++;
      }
      for (int i = 0; i < 3; i++) prev_dir_[i] = d[i];
      have_prev_ = true;
      sub_[0] = sub_[1] = sub_[2] = 0;
      sub_n_ = 0;
    }
  }
  double bsq = 0;
  for (int i = 0; i < 3; i++) {
    acc_sum_[i] += a[i];
    float y = hp_[i].step(a[i]);
    bsq += double(y) * y;
  }
  // the high-pass filters ring for a few samples after power-up; leave those out of the buzz
  if (settle_ < 64) settle_++; else { buzz_sq_ += bsq; buzz_n_++; }
  for (int i = 0; i < 3; i++) {
    w_sum_[i] += w[i];
    w_sq_[i] += double(w[i]) * w[i];
    a_sq_[i] += double(a[i]) * a[i];
  }
  n_imu_++;
}

void Sensing::mic(const int16_t* pcm, int n) {
  for (int k = 0; k < n; k++) {
    if (!mic_primed_) {   // a new burst: the DC filter as if this level had always been there, the band pass at rest
      mic_dc_.x1 = mic_dc_.x2 = float(pcm[k]);
      mic_dc_.y1 = mic_dc_.y2 = 0;
      snore_hp_.reset();
      snore_lp_.reset();
      mic_primed_ = true;
    }
    float x = mic_dc_.step(float(pcm[k]));
    float s = snore_lp_.step(snore_hp_.step(x));
    if (mic_skip_ > 0) { mic_skip_--; continue; }
    mic_sq_ += double(x) * x;
    snore_sq_ += double(s) * s;
    n_mic_++;
  }
}

static uint16_t sat16(double v) { return v >= 65535 ? 65535 : v <= 0 ? 0 : uint16_t(v + 0.5); }
static int16_t sat_cdeg(float deg) {
  float c = deg * 100;
  return c >= 32767 ? 32767 : c <= -32767 ? -32767 : int16_t(lrintf(c));
}

void Sensing::close(Record& r, const Params& p) {
  r.flags = 0;
  if (n_imu_ > 0) {
    float g[3] = {float(acc_sum_[0] / n_imu_), float(acc_sum_[1] / n_imu_), float(acc_sum_[2] / n_imu_)};
    norm3(g);
    for (int i = 0; i < 3; i++) grav[i] = g[i];
    have_grav = true;
    head_angles(g, p.zero, p.head_axis, &roll_, &pitch_);
    // gyro zero-rate offset: learnt from windows where the accelerometer says the head is still (a wrong
    // start value, e.g. moving at power-up, is corrected by the first still window)
    double mw[3], m2 = 0;
    bool still = gyro_;
    int full = int(hz_ * WINDOW_MS / 1000 * 0.8f);   // 80 % of a window's samples
    for (int i = 0; i < 3; i++) {
      mw[i] = w_sum_[i] / n_imu_;
      double am = acc_sum_[i] / n_imu_, av = a_sq_[i] / n_imu_ - am * am, wv = w_sq_[i] / n_imu_ - mw[i] * mw[i];
      if (av > 0.006 * 0.006 || wv > 1.5 * 1.5) still = false;
      // once known, the offset only drifts slowly: a steady turn (gravity barely changing) is not a new offset
      if (bias_init_ && fabs(mw[i] - bias_[i]) > 3) still = false;
    }
    if (still && n_imu_ >= full)
      for (int i = 0; i < 3; i++) bias_[i] = bias_init_ ? bias_[i] + 0.2f * float(mw[i] - bias_[i]) : float(mw[i]);
    if (still && n_imu_ >= full) bias_init_ = true;
    if (gyro_) {
      for (int i = 0; i < 3; i++) m2 += w_sq_[i] / n_imu_ - 2 * bias_[i] * mw[i] + double(bias_[i]) * bias_[i];
      move_dps = m2 > 0 ? float(sqrt(m2)) : 0;
    } else {
      move_dps = turn_n_ ? float(sqrt(turn_sq_ / turn_n_)) : 0;
    }
    if (n_imu_ < full) r.flags |= F_SHORT;   // samples were lost
  } else {
    r.flags |= F_SHORT;   // no motion data: keep the last angles, report no movement
    move_dps = 0;
  }
  r.roll = sat_cdeg(roll_);
  r.pitch = sat_cdeg(pitch_);
  r.move = sat16(move_dps * 10);
  r.buzz = buzz_n_ ? sat16(sqrt(buzz_sq_ / buzz_n_) * 1000 * 10) : 0;   // g -> 0.1 mg
  if (n_mic_ > 0) {
    r.mic = sat16(sqrt(mic_sq_ / n_mic_));
    r.snore = sat16(sqrt(snore_sq_ / n_mic_));
  } else {
    r.mic = r.snore = 0;
    r.flags |= F_NOMIC;
  }
  if (pitch_ > p.upright_deg) r.flags |= F_UPRIGHT;
  last_imu_n = n_imu_;
  last_mic_n = n_mic_;
  for (auto& s : acc_sum_) s = 0;
  for (int i = 0; i < 3; i++) w_sum_[i] = w_sq_[i] = a_sq_[i] = 0;
  buzz_sq_ = mic_sq_ = snore_sq_ = turn_sq_ = 0;
  n_imu_ = n_mic_ = buzz_n_ = turn_n_ = 0;
}

}  // namespace fhs
