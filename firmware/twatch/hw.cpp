// Forehead sensor firmware for the LilyGO T-Watch S3, taped to the forehead (the strap comes off): the hardware
// glue. Everything the device does is in src/core (firmware/core, tested on the laptop). Here: the
// BMA423 accelerometer (no gyro), the DRV2605 vibration driver, the MAX98357 speaker, the PDM mic, the PCF8563
// clock chip, the AXP2101 power chip and its button, the screen, the log partition and USB. No Bluetooth: the
// night stays in the flash and is read in the morning over USB (fhs dump --usb).
// Off USB the chip sleeps between motion reads and mic bursts; the screen, touch, radio and GPS rails are off.
// Pins and power rails: LilyGO's T-Watch S3 library (utilities.h, LilyGoLib.cpp beginPower).
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>
#include <esp_partition.h>
#include <esp_sleep.h>
#include <esp_mac.h>
#include <esp_random.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <driver/i2s_pdm.h>
#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>
#include <time.h>
#include "src/core/device.h"
#include "key_salt.h"   // FHS_KEY_SALT: made by build.sh on the first build, never committed
#include "prompts.h"    // PROMPT_CLIPS: the spoken zero's phrases (fhs voice makes them)

using namespace fhs;

// ---------- pins and parts ----------
static const int I2C_SDA = 10, I2C_SCL = 11;          // BMA423, PCF8563, AXP2101, DRV2605
static const int PMU_IRQ = 21;                        // AXP2101 interrupt (button, USB), low active
static const int TFT_MOSI = 13, TFT_SCLK = 18, TFT_CS = 12, TFT_DC = 38, TFT_BL = 45;
static const int SPK_BCK = 48, SPK_WS = 15, SPK_DOUT = 46;   // MAX98357A
static const int MIC_CLK = 44, MIC_DATA = 47;         // PDM mic
static const uint8_t DRV_ADDR = 0x5A, RTC_ADDR = 0x51;
static const float IMU_HZ_WATCH = 800;
static const uint32_t SPK_RATE = 16000;

XPowersPMU pmu;
Preferences prefs;
Device dev;
static bool pmu_ok = false, drv_ok = false, rtc_ok = false;
static uint8_t imu_addr = 0;
static char hw_info[192];
static esp_reset_reason_t reset_why = ESP_RST_UNKNOWN;

static const char* reset_name(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_SW: return "reboot";
    case ESP_RST_USB: return "usb";
    case ESP_RST_PANIC: return "CRASH";
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "HANG";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    default: return "other";
  }
}

// ---------- I2C helpers ----------
static bool reg_write(uint8_t addr, uint8_t reg, uint8_t v) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

static int reg_read(uint8_t addr, uint8_t reg, uint8_t* buf, int n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  int got = Wire.requestFrom(uint16_t(addr), size_t(n));   // (a uint8_t count would cut the FIFO reads at 255)
  for (int i = 0; i < got; i++) buf[i] = Wire.read();
  return got;
}

// ---------- the log: a 15 MB partition of the 16 MB flash (partitions.csv) ----------
struct PartFlash : Flash {
  const esp_partition_t* p = nullptr;
  uint32_t size() override { return p ? p->size : 0; }
  bool read(uint32_t a, void* b, uint32_t n) override { return esp_partition_read(p, a, b, n) == ESP_OK; }
  bool program(uint32_t a, const void* b, uint32_t n) override { return esp_partition_write(p, a, b, n) == ESP_OK; }
  bool erase_sector(uint32_t a) override { return esp_partition_erase_range(p, a, BLOCK_SIZE) == ESP_OK; }
} logflash;

// ---------- settings in NVS (each key is written atomically) ----------
template <class T>
static bool load_blob(const char* key, T& out) {
  uint8_t buf[sizeof(T) + 2];
  if (prefs.getBytesLength(key) != sizeof buf || prefs.getBytes(key, buf, sizeof buf) != sizeof buf) return false;
  uint16_t crc;
  memcpy(&crc, buf + sizeof(T), 2);
  if (crc16(buf, sizeof(T)) != crc) return false;
  memcpy(&out, buf, sizeof(T));
  return true;
}

template <class T>
static bool save_blob(const char* key, const T& v) {
  uint8_t buf[sizeof(T) + 2];
  memcpy(buf, &v, sizeof(T));
  uint16_t crc = crc16(&v, sizeof(T));
  memcpy(buf + sizeof(T), &crc, 2);
  return prefs.putBytes(key, buf, sizeof buf) == sizeof buf;
}

// ---------- motion sensor (BMA423), registers set directly: accel only, 800 Hz into its 1 KB FIFO ----------
// The feature engine (steps, taps) needs a config upload; the plain accelerometer and FIFO don't.
static bool imu_ok = false;
static uint32_t imu_fail = 0, imu_ovr = 0, imu_last_ms = 0;
static uint32_t imu_data_ms = 0, imu_quiet = 0;   // when the last sample came; times it was set up again for silence

static bool imu_begin() {
  imu_addr = 0;
  for (uint8_t a : {0x19, 0x18}) {
    uint8_t id = 0;
    if (reg_read(a, 0x00, &id, 1) == 1 && id == 0x13) { imu_addr = a; break; }   // CHIP_ID
  }
  if (!imu_addr) return false;
  reg_write(imu_addr, 0x7E, 0xB6);   // CMD: soft reset
  delay(20);
  bool ok = reg_write(imu_addr, 0x7C, 0x00);   // PWR_CONF: no advanced power save (the FIFO keeps filling)
  delay(2);
  ok = ok && reg_write(imu_addr, 0x40, 0xAB)   // ACC_CONF: 800 Hz, performance mode (continuous filter)
       && reg_write(imu_addr, 0x41, 0x00)      // ACC_RANGE: +-2 g (12 bits: ~1 mg a step)
       && reg_write(imu_addr, 0x48, 0x00)      // FIFO_CONFIG_0: stream (oldest dropped when full), no sensor time
       && reg_write(imu_addr, 0x49, 0x40)      // FIFO_CONFIG_1: accel only, no frame headers: 6 bytes a sample
       && reg_write(imu_addr, 0x7D, 0x04);     // PWR_CTRL: accel on
  delay(5);
  reg_write(imu_addr, 0x7E, 0xB0);   // CMD: flush the FIFO
  imu_last_ms = imu_data_ms = millis();
  return ok;
}

static void imu_poll() {
  if (!imu_ok || millis() - imu_last_ms < 25) return;   // ~20 samples a read, not 1-2 every loop
  imu_last_ms = millis();
  static uint8_t b[504];   // 84 samples a read (the Wire buffer is set to 512)
  for (int k = 0; k < 6; k++) {
    uint8_t l[2];
    if (reg_read(imu_addr, 0x24, l, 2) != 2) { imu_fail++; return; }   // FIFO_LENGTH, bytes
    int n = (l[0] | (l[1] & 0x3F) << 8);
    if (n >= 1014) imu_ovr++;   // (nearly) full: samples were dropped (the windows show F_SHORT)
    n = n / 6 * 6;
    if (n == 0) return;
    if (n > int(sizeof b)) n = sizeof b;
    if (reg_read(imu_addr, 0x26, b, n) != n) { imu_fail++; return; }
    imu_fail = 0;
    for (int i = 0; i < n; i += 6) {
      int16_t raw[6] = {0, 0, 0, 0, 0, 0};
      for (int ax = 0; ax < 3; ax++) raw[ax] = int16_t(b[i + 2 * ax] | b[i + 2 * ax + 1] << 8) >> 4;   // 12 bits
      if (raw[0] == -2048 && raw[1] == -2048 && raw[2] == -2048) continue;   // 0x8000s: an empty-FIFO frame
      dev.imu(raw);
      imu_data_ms = imu_last_ms;
    }
    if (n < int(sizeof b)) return;
  }
}

// ---------- vibration (DRV2605, real-time playback: the core plays its own patterns) ----------
static uint8_t motor_core = 0, motor_fb = 0, motor_now = 0xFF;
static uint32_t fb_until = 0;
static int fb_pulses = 0;

static bool drv_begin() {
  uint8_t id = 0;
  if (reg_read(DRV_ADDR, 0x00, &id, 1) != 1) return false;   // STATUS: answers when its rail (BLDO2) is on
  uint8_t fb = 0, c3 = 0;
  reg_read(DRV_ADDR, 0x1A, &fb, 1);
  reg_read(DRV_ADDR, 0x1D, &c3, 1);
  return reg_write(DRV_ADDR, 0x01, 0x00)                // MODE: out of standby
         && reg_write(DRV_ADDR, 0x02, 0x00)             // RTP input: 0
         && reg_write(DRV_ADDR, 0x1A, fb & 0x7F)        // FEEDBACK: ERM motor (as LilyGO sets it)
         && reg_write(DRV_ADDR, 0x1D, c3 | 0x20 | 0x08) // CONTROL3: ERM open loop, unsigned RTP (0-255)
         && reg_write(DRV_ADDR, 0x01, 0x45);            // MODE: real-time playback, standby until needed
}

static void motor_apply() {
  uint8_t pct = max(motor_core, motor_fb);
  if (pct == motor_now || !drv_ok) return;
  if (pct) {
    if (!motor_now || motor_now == 0xFF) reg_write(DRV_ADDR, 0x01, 0x05);   // wake, real-time playback
    reg_write(DRV_ADDR, 0x02, uint8_t(pct >= 100 ? 255 : pct * 255 / 100));
  } else {
    reg_write(DRV_ADDR, 0x02, 0x00);
    reg_write(DRV_ADDR, 0x01, 0x45);   // standby: a few uA
  }
  motor_now = pct;
}

// the DRV2605's own test: it drives the motor briefly and checks it is there and not shorted
static bool motor_diag(char* out, int n) {
  bool rail = pmu_ok && pmu.isEnableBLDO2();
  uint8_t st = 0, vb = 0;
  if (!rail || reg_read(DRV_ADDR, 0x00, &st, 1) != 1) {
    snprintf(out, n, "drv not answering, rail BLDO2 %s", rail ? "on" : "OFF");
    return true;
  }
  reg_write(DRV_ADDR, 0x02, 0x00);
  reg_write(DRV_ADDR, 0x01, 0x06);   // MODE: diagnostics
  reg_write(DRV_ADDR, 0x0C, 0x01);   // GO
  uint8_t go = 1;
  for (int t = 0; t < 150 && (go & 1); t++) { delay(10); reg_read(DRV_ADDR, 0x0C, &go, 1); }
  reg_read(DRV_ADDR, 0x00, &st, 1);
  reg_read(DRV_ADDR, 0x21, &vb, 1);  // VBAT: the chip's supply, 5.6 V full scale
  // then a plain full-strength buzz, as the nudges play it: the mode it is in and its status while it runs
  uint8_t md = 0, st2 = 0, rtp = 0;
  reg_write(DRV_ADDR, 0x0C, 0x00);
  reg_write(DRV_ADDR, 0x01, 0x05);
  reg_write(DRV_ADDR, 0x02, 0xFF);
  delay(300);
  reg_read(DRV_ADDR, 0x01, &md, 1);
  reg_read(DRV_ADDR, 0x02, &rtp, 1);
  reg_read(DRV_ADDR, 0x00, &st2, 1);
  reg_write(DRV_ADDR, 0x02, 0x00);
  reg_write(DRV_ADDR, 0x01, 0x45);   // back to real-time playback, standby
  motor_now = 0;
  snprintf(out, n, "motor %s%s%s, supply %.2f V, status 0x%02x; buzzing: mode 0x%02x rtp 0x%02x status 0x%02x%s",
           (go & 1) ? "TIMEOUT" : (st & 0x08) ? "FAIL (open or shorted)" : "ok",
           (st & 0x02) ? ", OVER TEMP" : "", (st & 0x01) ? ", OVER CURRENT" : "", vb * 5.6f / 255, st, md, rtp, st2,
           (st2 & 0x01) ? " OVER CURRENT" : "");
  return true;
}

// the watch's own feedback (zero set, ...): n short pulses, on top of whatever the core plays
static void feedback(int pulses) {
  fb_pulses = pulses * 2;
  fb_until = millis();
}

static void feedback_service(uint32_t now) {
  if (!fb_pulses || int32_t(now - fb_until) < 0) return;
  fb_pulses--;
  motor_fb = (fb_pulses % 2) ? 80 : 0;
  fb_until = now + 120;
  motor_apply();
}

// ---------- speaker (MAX98357A on I2S1): a tone the core sets, written ahead into the DMA buffers ----------
static i2s_chan_handle_t spk = nullptr;
static bool spk_on = false;
static volatile uint16_t tone_hz = 0;
static volatile uint8_t tone_vol = 0;
static float tone_phase = 0;
static uint32_t spk_quiet_ms = 0;
static volatile uint32_t voice_pos = 0;   // the next clip sample while tone_hz is a clip (a prompt)

// the recorded clip a tone "frequency" stands for: a spoken zero phrase; null for a real tone
static const int8_t* clip_of(uint16_t hz, uint32_t* n) {
  if (is_prompt_hz(hz)) { *n = PROMPT_CLIPS[hz & 0xFF].n; return PROMPT_CLIPS[hz & 0xFF].pcm; }
  return nullptr;
}

static bool spk_begin() {
  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  cc.dma_desc_num = 4;
  cc.dma_frame_num = 160;   // 4 x 10 ms: a tone change is heard within ~40 ms
  cc.auto_clear = true;     // silence, not a repeated buffer, if the loop falls behind
  if (i2s_new_channel(&cc, &spk, nullptr) != ESP_OK) return false;
  i2s_std_config_t sc = {};
  sc.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SPK_RATE);
  sc.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  sc.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  sc.gpio_cfg.bclk = gpio_num_t(SPK_BCK);
  sc.gpio_cfg.ws = gpio_num_t(SPK_WS);
  sc.gpio_cfg.dout = gpio_num_t(SPK_DOUT);
  sc.gpio_cfg.din = I2S_GPIO_UNUSED;
  return i2s_channel_init_std_mode(spk, &sc) == ESP_OK;
}

static void spk_service(uint32_t now) {
  if (!spk) return;
  bool want = tone_hz && tone_vol;
  if (want && !spk_on) {
    if (pmu_ok) pmu.enableDLDO1();   // the amplifier's rail: only while there is sound (~2-3 mA idle)
    delay(2);
    i2s_channel_enable(spk);
    spk_on = true;
  }
  if (!spk_on) return;
  if (want) spk_quiet_ms = now;
  else if (now - spk_quiet_ms > 1000) {   // a second of silence: amplifier off (the patterns pause shorter)
    i2s_channel_disable(spk);
    if (pmu_ok) pmu.disableDLDO1();
    spk_on = false;
    return;
  }
  // a sine, louder by driving it into clipping above 50 % (a square-ish, harsher tone at 100 %)
  static int16_t buf[160];
  for (int k = 0; k < 4; k++) {
    float hz = tone_hz, vol = tone_vol / 100.0f;
    float amp = vol <= 0.5f ? vol * 2 : 1, drive = vol <= 0.5f ? 1 : 1 + (vol - 0.5f) * 6;
    uint32_t clip_n = 0;
    if (const int8_t* clip = clip_of(tone_hz, &clip_n)) {   // as it is (fhs voice made it loud), silence after its end
      uint32_t at = voice_pos;
      for (int i = 0; i < 160; i++, at++) buf[i] = at < clip_n ? int16_t(clip[at] * 256 * amp) : 0;
      size_t w = 0;
      if (i2s_channel_write(spk, buf, sizeof buf, &w, 0) != ESP_OK || w < sizeof buf) {
        voice_pos += w / 2;   // the DMA is full: go on from the first sample that didn't go
        return;
      }
      voice_pos = at;
      continue;
    }
    for (int i = 0; i < 160; i++) {
      float x = hz && vol > 0 ? sinf(tone_phase) * drive : 0;
      if (x > 1) x = 1;
      if (x < -1) x = -1;
      buf[i] = int16_t(x * amp * 32000);
      tone_phase += 2 * PI * hz / SPK_RATE;
      if (tone_phase > 2 * PI) tone_phase -= 2 * PI;
    }
    size_t w = 0;
    if (i2s_channel_write(spk, buf, sizeof buf, &w, 0) != ESP_OK || w < sizeof buf) {
      // the DMA is full: put the phase back for the samples that didn't go
      tone_phase -= 2 * PI * hz / SPK_RATE * float(160 - w / 2);
      while (tone_phase < 0) tone_phase += 2 * PI;
      return;
    }
  }
}

// ---------- mic (PDM on I2S0): bursts of mic_duty % of every 0.5 s window ----------
static i2s_chan_handle_t mic = nullptr;
static bool mic_on = false, mic_seen = false;
static uint32_t mic_base = 0, mic_burst_end = 0, mic_burst_at = 0;
static int mic_drop = 0;   // samples still to throw away after switching on

static bool mic_begin() {
  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);   // PDM RX is on I2S0 only
  cc.dma_desc_num = 6;
  cc.dma_frame_num = 256;   // 6 x 16 ms
  if (i2s_new_channel(&cc, nullptr, &mic) != ESP_OK) return false;
  i2s_pdm_rx_config_t pc = {};
  pc.clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(16000);
  pc.slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  pc.gpio_cfg.clk = gpio_num_t(MIC_CLK);
  pc.gpio_cfg.din = gpio_num_t(MIC_DATA);
  pc.gpio_cfg.invert_flags.clk_inv = false;
  return i2s_channel_init_pdm_rx_mode(mic, &pc) == ESP_OK;
}

// the next burst start: 50 ms into each of the core's 0.5 s windows (they start at mic_base)
static uint32_t mic_next(uint32_t now) {
  uint32_t k = (now - mic_base) / WINDOW_MS + 1;
  return mic_base + k * WINDOW_MS + 50;
}

static void mic_service(uint32_t now) {
  if (!mic) return;
  uint8_t duty = dev.params().mic_duty;
  bool always = duty >= 100;
  if (!mic_on && (always || int32_t(now - mic_burst_at) >= 0)) {
    mic_burst_at = mic_next(now);
    i2s_channel_enable(mic);
    mic_on = true;
    mic_drop = 320;   // the PDM filter's first 20 ms are its start-up, not sound: counted in samples, not by the clock
                      // (read late, the start-up sat in the buffer and counted as sound, 30-31 Oct)
    dev.mic_start();
    mic_burst_end = now + WINDOW_MS * duty / 100;
  }
  if (!mic_on) return;
  int16_t buf[256];
  size_t got = 0;
  while (i2s_channel_read(mic, buf, sizeof buf, &got, 0) == ESP_OK && got) {
    int n = int(got / 2), skip = min(n, mic_drop);
    mic_drop -= skip;
    if (n > skip) dev.mic(buf + skip, n - skip);
    mic_seen = true;
  }
  if (!always && int32_t(now - mic_burst_end) >= 0) {
    i2s_channel_disable(mic);
    mic_on = false;
  }
}

// ---------- clock chip (PCF8563): UTC ----------
static uint8_t bcd(uint8_t v) { return uint8_t((v / 10) << 4 | v % 10); }
static uint8_t unbcd(uint8_t v) { return uint8_t((v >> 4) * 10 + (v & 15)); }

static int64_t days_from_civil(int y, unsigned m, unsigned d) {   // Howard Hinnant's algorithm
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = unsigned(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return int64_t(era) * 146097 + int64_t(doe) - 719468;
}

static bool rtc_read(int64_t* unix_ms) {
  uint8_t r[7];
  if (reg_read(RTC_ADDR, 0x02, r, 7) != 7) return false;
  if (r[0] & 0x80) return false;   // VL: the clock lost power, the time is not valid
  int sec = unbcd(r[0] & 0x7F), min = unbcd(r[1] & 0x7F), hour = unbcd(r[2] & 0x3F), day = unbcd(r[3] & 0x3F);
  int mon = unbcd(r[5] & 0x1F), year = 2000 + unbcd(r[6]);
  if (mon < 1 || mon > 12 || day < 1 || day > 31) return false;
  *unix_ms = ((days_from_civil(year, mon, day) * 24 + hour) * 60 + min) * 60000LL + sec * 1000LL;
  return true;
}

static void rtc_write(int64_t unix_ms) {
  time_t t = time_t(unix_ms / 1000);
  struct tm g;
  gmtime_r(&t, &g);
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x02);
  Wire.write(bcd(g.tm_sec));             // VL cleared
  Wire.write(bcd(g.tm_min));
  Wire.write(bcd(g.tm_hour));
  Wire.write(bcd(g.tm_mday));
  Wire.write(uint8_t(g.tm_wday));
  Wire.write(bcd(g.tm_mon + 1));         // century bit 0: 20xx
  Wire.write(bcd(g.tm_year % 100));
  Wire.endTransmission();
}

// ---------- screen (ST7789, 240 x 240): a few lines of big text when the button is pressed ----------
static const uint8_t FONT[][5] = {   // 5 x 7, ' ' to 'Z' (classic GLCD font), columns, bit 0 at the top
    {0, 0, 0, 0, 0}, {0, 0, 0x5F, 0, 0}, {0, 7, 0, 7, 0}, {0x14, 0x7F, 0x14, 0x7F, 0x14},
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, {0x23, 0x13, 0x08, 0x64, 0x62}, {0x36, 0x49, 0x56, 0x20, 0x50}, {0, 8, 7, 3, 0},
    {0, 0x1C, 0x22, 0x41, 0}, {0, 0x41, 0x22, 0x1C, 0}, {0x2A, 0x1C, 0x7F, 0x1C, 0x2A}, {8, 8, 0x3E, 8, 8},
    {0, 0x80, 0x70, 0x30, 0}, {8, 8, 8, 8, 8}, {0, 0, 0x60, 0x60, 0}, {0x20, 0x10, 8, 4, 2},
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0, 0x42, 0x7F, 0x40, 0}, {0x72, 0x49, 0x49, 0x49, 0x46},
    {0x21, 0x41, 0x49, 0x4D, 0x33}, {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x31}, {0x41, 0x21, 0x11, 9, 7}, {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x46, 0x49, 0x49, 0x29, 0x1E}, {0, 0, 0x14, 0, 0}, {0, 0x40, 0x34, 0, 0}, {0, 8, 0x14, 0x22, 0x41},
    {0x14, 0x14, 0x14, 0x14, 0x14}, {0, 0x41, 0x22, 0x14, 8}, {2, 1, 0x59, 9, 6}, {0x3E, 0x41, 0x5D, 0x59, 0x4E},
    {0x7C, 0x12, 0x11, 0x12, 0x7C}, {0x7F, 0x49, 0x49, 0x49, 0x36}, {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 9, 9, 9, 1},
    {0x3E, 0x41, 0x41, 0x51, 0x73}, {0x7F, 8, 8, 8, 0x7F}, {0, 0x41, 0x7F, 0x41, 0}, {0x20, 0x40, 0x41, 0x3F, 1},
    {0x7F, 8, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40}, {0x7F, 2, 0x1C, 2, 0x7F},
    {0x7F, 4, 8, 0x10, 0x7F}, {0x3E, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 9, 9, 9, 6}, {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 9, 0x19, 0x29, 0x46}, {0x26, 0x49, 0x49, 0x49, 0x32}, {3, 1, 0x7F, 1, 3}, {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, {0x3F, 0x40, 0x38, 0x40, 0x3F}, {0x63, 0x14, 8, 0x14, 0x63},
    {3, 4, 0x78, 4, 3}, {0x61, 0x59, 0x49, 0x4D, 0x43},
};
static const SPISettings TFT_SPI(40000000, MSBFIRST, SPI_MODE0);
static bool screen_on = false, screen_full = false;   // screen_full: redraw every row (just switched on)
static uint32_t screen_until = 0, screen_drawn = 0;
static char screen_msg[2][14];   // a message on top (zero: keep still / set / rejected), for a few seconds
static uint32_t screen_msg_until = 0;

static void tft_cmd(uint8_t c, const uint8_t* d = nullptr, int n = 0) {
  SPI.beginTransaction(TFT_SPI);
  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, LOW);
  SPI.write(c);
  digitalWrite(TFT_DC, HIGH);
  for (int i = 0; i < n; i++) SPI.write(d[i]);
  digitalWrite(TFT_CS, HIGH);
  SPI.endTransaction();
}

static void tft_window(int x, int y, int w, int h) {
  y += 80;   // rotation 2 (as LilyGO uses it): the 240 x 240 panel sits 80 rows into the 320-row memory
  uint8_t ca[4] = {uint8_t(x >> 8), uint8_t(x), uint8_t((x + w - 1) >> 8), uint8_t(x + w - 1)};
  uint8_t ra[4] = {uint8_t(y >> 8), uint8_t(y), uint8_t((y + h - 1) >> 8), uint8_t(y + h - 1)};
  tft_cmd(0x2A, ca, 4);
  tft_cmd(0x2B, ra, 4);
}

static void tft_fill(int x, int y, int w, int h, uint16_t color) {
  tft_window(x, y, w, h);
  SPI.beginTransaction(TFT_SPI);
  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, LOW);
  SPI.write(0x2C);
  digitalWrite(TFT_DC, HIGH);
  for (int i = 0; i < w * h; i++) SPI.write16(color);
  digitalWrite(TFT_CS, HIGH);
  SPI.endTransaction();
}

static void tft_text(int row, const char* s, uint16_t color) {   // 13 characters of 18 x 21 px a row, 8 rows
  const int S = 3, H = 7 * S, y = 4 + row * 29;
  static uint16_t lb[240 * H];   // the row in RAM, sent in one go (pixels big-endian, as the panel takes them)
  memset(lb, 0, sizeof lb);
  uint16_t px = uint16_t(color << 8 | color >> 8);
  for (int c = 0; s[c] && c < 13; c++) {
    char ch = s[c] >= 'a' && s[c] <= 'z' ? char(s[c] - 32) : s[c];
    if (ch < ' ' || ch > 'Z') ch = '?';
    const uint8_t* g = FONT[ch - ' '];
    for (int col = 0; col < 5; col++)
      for (int r = 0; r < 7; r++)
        if (g[col] >> r & 1)
          for (int dy = 0; dy < S; dy++)
            for (int dx = 0; dx < S; dx++) lb[(r * S + dy) * 240 + c * 18 + col * S + dx] = px;
  }
  tft_window(0, y, 240, H);
  SPI.beginTransaction(TFT_SPI);
  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, LOW);
  SPI.write(0x2C);
  digitalWrite(TFT_DC, HIGH);
  SPI.writeBytes(reinterpret_cast<const uint8_t*>(lb), sizeof lb);
  digitalWrite(TFT_CS, HIGH);
  SPI.endTransaction();
}

static void tft_begin() {
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft_cmd(0x01);   // SWRESET
  delay(150);
  tft_cmd(0x10);   // SLPIN: dark and asleep until the button
}

static void screen_show(uint32_t now, uint32_t ms) {
  if (!screen_on) {
    tft_cmd(0x11);   // SLPOUT
    delay(120);
    uint8_t v = 0x55;
    tft_cmd(0x3A, &v, 1);   // 16-bit colour
    v = 0xC0;
    tft_cmd(0x36, &v, 1);   // MADCTL: rotation 2
    tft_cmd(0x21);          // inversion on (IPS)
    tft_cmd(0x13);
    tft_fill(0, 0, 240, 240, 0);
    tft_cmd(0x29);          // DISPON
    digitalWrite(TFT_BL, HIGH);
    screen_on = true;
    screen_full = true;
    screen_drawn = 0;
  }
  screen_until = now + ms;
}

static void screen_service(uint32_t now) {
  if (!screen_on) return;
  if (int32_t(now - screen_until) >= 0) {
    digitalWrite(TFT_BL, LOW);
    tft_cmd(0x28);   // DISPOFF
    tft_cmd(0x10);   // SLPIN
    screen_on = false;
    return;
  }
  if (screen_drawn && now - screen_drawn < 500) return;
  screen_drawn = now;
  const Params& p = dev.params();
  const Record& r = dev.last();
  char l[8][16];
  int pct = pmu_ok ? pmu.getBatteryPercent() : -1;
  snprintf(l[0], 16, "BATT %d%%%s", pct, pmu_ok && pmu.isCharging() ? " CHG" : "");
  snprintf(l[1], 16, "%s", p.mode == MODE_THERAPY ? "THERAPY" : "SHADOW");
  snprintf(l[2], 16, "ANGLE %d", int(p.angle_deg + 0.5f));
  snprintf(l[3], 16, "ZERO %s", p.zero_set ? "SET" : "NOT SET");
  snprintf(l[4], 16, "ROLL %d", r.roll / 100);
  snprintf(l[5], 16, "LOG %dH", int(dev.log().used() * RECS_PER_BLOCK / 7200));
  snprintf(l[6], 16, "%s", r.flags & F_CLOCK ? "CLOCK OK" : "NO CLOCK");
  snprintf(l[7], 16, "%s", dev.session() ? "LOGGING" : imu_ok ? "" : "NO MOTION!");
  bool msg = int32_t(now - screen_msg_until) < 0;
  static char shown[8][18];
  for (int i = 0; i < 8; i++) {
    const char* t = msg && i < 2 ? screen_msg[i] : l[i];
    char key[18];
    key[0] = msg && i < 2 ? '!' : ' ';   // a message row redraws in its own colour
    strncpy(key + 1, t, sizeof key - 2);
    key[sizeof key - 1] = 0;
    if (screen_full || strcmp(key, shown[i])) {
      tft_text(i, t, msg && i < 2 ? 0xFFE0 : i == 7 && !imu_ok ? 0xF800 : 0xFFFF);
      memcpy(shown[i], key, sizeof key);
    }
  }
  screen_full = false;
}

static void screen_message(uint32_t now, const char* a, const char* b, uint32_t ms) {
  snprintf(screen_msg[0], sizeof screen_msg[0], "%s", a);
  snprintf(screen_msg[1], sizeof screen_msg[1], "%s", b);
  screen_msg_until = now + ms;
  screen_show(now, ms + 2000);
  screen_drawn = 0;
}

// ---------- power chip: rails, battery, the button, USB ----------
static bool usb_in = false;

static bool pmu_begin() {
  if (!pmu.init(Wire, I2C_SDA, I2C_SCL)) return false;
  pmu.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);
  pmu.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_900MA);
  pmu.setSysPowerDownVoltage(2600);
  pmu.setALDO1Voltage(3300); pmu.enableALDO1();    // clock chip backup
  pmu.setALDO2Voltage(3300); pmu.enableALDO2();    // backlight supply (the pin switches it)
  pmu.setALDO3Voltage(3300); pmu.enableALDO3();    // screen + touch: on, as LilyGO has it (the second watch's
                                                   // screen stayed black with it off, 31 Oct)
  pmu.disableALDO4();                              // LoRa radio: unused
  pmu.disableBLDO1();                              // GPS (Plus model): unused
  pmu.setBLDO2Voltage(3300); pmu.enableBLDO2();    // DRV2605 enable
  pmu.setDLDO1Voltage(3300); pmu.disableDLDO1();   // speaker amplifier: on only while sounding
  pmu.disableDC2(); pmu.disableDC3(); pmu.disableDC4(); pmu.disableDC5();   // DC3: GPS on the Plus, unused
  pmu.disableCPUSLDO(); pmu.disableDLDO2();
  pmu.setPowerKeyPressOffTime(XPOWERS_POWEROFF_10S);   // holding the button 10 s switches the watch off:
  pmu.enableLongPressShutdown();                       // the time alone did nothing (it kept nudging, 30 Oct)
  pmu.setLongPressPowerOFF();                          // off, not a restart
  pmu.setPowerKeyPressOnTime(XPOWERS_POWERON_128MS);
  pmu.setIrqLevelTime(XPOWERS_AXP2101_PRESSOFF_2S5);   // the chip's own long press, 2.5 s (unused: the core times
                                                       // and counts the presses from the edges)
  pmu.disableTSPinMeasure();                       // no battery thermistor (else charging misbehaves)
  pmu.enableBattDetection();
  pmu.enableVbusVoltageMeasure();
  pmu.enableBattVoltageMeasure();
  pmu.enableSystemVoltageMeasure();
  pmu.setChargingLedMode(XPOWERS_CHG_LED_OFF);
  pmu.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
  pmu.enableIRQ(XPOWERS_AXP2101_VBUS_INSERT_IRQ | XPOWERS_AXP2101_VBUS_REMOVE_IRQ |
                XPOWERS_AXP2101_PKEY_SHORT_IRQ | XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ);
  pmu.clearIrqStatus();
  pmu.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);   // LilyGO's charging settings
  pmu.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_125MA);
  pmu.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);
  pmu.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V35);
  pmu.setButtonBatteryChargeVoltage(3300);
  pmu.enableButtonBatteryCharge();
  return true;
}

static void pmu_poll(uint32_t now) {
  if (!pmu_ok) return;
  static bool key_down = false;          // as last told to the core
  static uint32_t screen_due = 0;        // a short press shows the status screen, but only after the double-press
  if (digitalRead(PMU_IRQ) == LOW) {     // window: switching the screen on blocks the loop ~0.2 s, and a second
    pmu.getIrqStatus();                  // press in that time would merge with the first (1 Nov)
    bool shortp = pmu.isPekeyShortPressIrq();
    bool down = pmu.isPekeyNegativeIrq(), up = pmu.isPekeyPositiveIrq();   // the key's edges: pressed, let go
    pmu.clearIrqStatus();
    // the core times and counts the presses: double press = the spoken zero, past 5 s = "Switching
    // off" (the chip cuts at 10 s). Both edges in one read: their order follows from the state before
    if (down && up && key_down) {        // let go, then a whole new press (or a press still held: its release
      dev.button(false);                 // comes in a later read and is ignored as a release with no press)
      dev.button(true);
      dev.button(false);
      key_down = false;
    } else {
      if (down) { dev.button(true); key_down = true; }
      if (up) { dev.button(false); key_down = false; }
    }
    if (shortp) screen_due = now + Device::DOUBLE_GAP_MS + 100;
  }
  if (screen_due && int32_t(now - screen_due) >= 0) {
    screen_due = 0;
    if (!dev.zeroing()) screen_show(now, 8000);   // a double press shows ZERO: LIE FLAT itself
  }
  static uint32_t last = 0;
  if (now - last >= 250) { last = now; usb_in = pmu.isVbusIn(); }
}

struct WatchHal : Hal {
  uint32_t ms() override { return millis(); }
  void motor(uint8_t pct) override { motor_core = pct; motor_apply(); }
  bool motor_diag(char* out, int n) override { return ::motor_diag(out, n); }
  void tone(uint16_t hz, uint8_t vol) override {
    uint32_t n;
    if (hz != tone_hz && clip_of(hz, &n)) voice_pos = 0;   // a phrase from its start
    tone_hz = hz;
    tone_vol = vol;
  }
  uint32_t prompt_ms(uint8_t id) override { return id < N_PROMPTS ? PROMPT_CLIPS[id].n * 1000 / SPK_RATE : 0; }
  uint16_t batt_mv() override { return pmu_ok ? pmu.getBattVoltage() : 0; }
  bool usb_power() override { return usb_in; }
  bool charging() override { return pmu_ok && pmu.isCharging(); }
  bool load_params(Params& p) override { return load_blob("params", p); }
  bool save_params(const Params& p) override { return save_blob("params", p); }
  bool load_state(State& s) override { return load_blob("state", s); }
  bool save_state(const State& s) override { return save_blob("state", s); }
  uint32_t random32() override { return esp_random(); }
  bool chip_key(uint8_t k[8]) override {   // from the chip's factory MAC, mixed with this build's secret salt
    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);
    uint64_t x = FHS_KEY_SALT;
    for (int i = 0; i < 6; i++) x ^= uint64_t(mac[i]) << (8 * i);
    for (int round = 0; round < 4; round++) {   // splitmix64 rounds
      x += 0x9E3779B97F4A7C15ULL;
      x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
      x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
      x ^= x >> 31;
    }
    memcpy(k, &x, 8);
    return true;
  }
  void reboot() override { delay(100); ESP.restart(); }
  const char* info() override {
    snprintf(hw_info, sizeof hw_info, "imu=%s@%02x imu_fail=%lu ovr=%lu quiet=%lu flash=%luK mic=%s drv=%s rtc=%s pmu=%s reset=%s",
             imu_ok ? "ok" : "MISSING", imu_addr, (unsigned long)imu_fail, (unsigned long)imu_ovr, (unsigned long)imu_quiet,
             (unsigned long)(logflash.size() / 1024), mic_seen ? "ok" : "none", drv_ok ? "ok" : "MISSING",
             rtc_ok ? "ok" : "MISSING", pmu_ok ? "ok" : "MISSING", reset_name(reset_why));
    return hw_info;
  }
  void event(const char* msg) override {
    uint32_t now = millis();
    if (!strncmp(msg, "zero set", 8)) {
      screen_message(now, "ZERO SET", "", 3000);
      feedback(2);
    } else if (!strcmp(msg, "zero: started")) {
      screen_message(now, "ZERO:", "LIE FLAT", 8000);
    } else if (!strncmp(msg, "zero: moved", 11)) {
      screen_message(now, "MOVED:", "AGAIN", 6000);
    } else if (!strncmp(msg, "zero: not set", 13)) {
      screen_message(now, "ZERO", "NOT SET", 3000);
    } else if (!strcmp(msg, "zero: cancelled")) {
      screen_message(now, "ZERO", "CANCELLED", 3000);
    } else if (!strcmp(msg, "switching off")) {
      screen_message(now, "SWITCHING OFF", "KEEP HOLDING", 8000);
    } else if (!strcmp(msg, "switch off cancelled")) {
      screen_message(now, "STAYING ON", "", 3000);
    }
  }
  float imu_hz() override { return IMU_HZ_WATCH; }
  float acc_g_per_lsb() override { return 1.0f / 1024; }   // +-2 g in 12 bits
  bool has_gyro() override { return false; }
  bool rtc_get(int64_t* t) override { return rtc_ok && rtc_read(t); }
  bool resumed() override {   // a hang (watchdog), a crash or a brown-out: not a power-up, not a reboot asked for
    return reset_why == ESP_RST_TASK_WDT || reset_why == ESP_RST_INT_WDT || reset_why == ESP_RST_WDT ||
           reset_why == ESP_RST_PANIC || reset_why == ESP_RST_BROWNOUT;
  }
  void rtc_set(int64_t t) override { if (rtc_ok) rtc_write(t); }
  // nobody reads the USB port: empty the driver's own queue too and wake its sender (with the 0 ms timeout set in
  // setup() flush() does not wait: it re-arms the send interrupt and throws away what is queued), so the next
  // reply is not stuck behind old bytes when the laptop opens the port again
  void link_stalled(int port) override { if (port == PORT_USB) Serial.flush(); }
} hal;

static void drain() {
  TxRing& tx = dev.tx(PORT_USB);
  const char* p;
  for (int k = 0; k < 4; k++) {
    size_t n = tx.peek(&p);
    if (!n) return;
    int room = Serial.availableForWrite();
    if (room <= 0) return;
    size_t w = Serial.write((const uint8_t*)p, min(n, size_t(room)));
    if (!w) return;
    tx.drop(w);
  }
}

void setup() {
  reset_why = esp_reset_reason();
  setCpuFrequencyMhz(80);
  Serial.setTxTimeoutMs(0);   // never wait for a laptop that isn't reading
  Serial.begin(115200);
  Wire.setBufferSize(512);
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  pmu_ok = pmu_begin();
  Wire.setClock(400000);
  delay(10);
  drv_ok = drv_begin();
  if (!drv_ok) motor_now = 0;   // (else 0xFF reads as "playing" and the chip never sleeps)
  motor_apply();
  uint8_t ctl = 0;
  rtc_ok = reg_read(RTC_ADDR, 0x00, &ctl, 1) == 1 && reg_write(RTC_ADDR, 0x0D, 0x00);   // CLKOUT off: saves its backup
  pinMode(PMU_IRQ, INPUT_PULLUP);
  tft_begin();
  prefs.begin("fhs", false);
  logflash.p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, esp_partition_subtype_t(0x40), "fhslog");
  spk_begin();
  mic_begin();
  usb_in = pmu_ok && pmu.isVbusIn();
  mic_base = millis();
  mic_burst_at = mic_next(mic_base);
  dev.begin(&hal, &logflash);
  imu_ok = imu_begin();
  if (!hal.resumed()) screen_show(millis(), 5000);   // power-up: show the status once (not in the night after a hang)
  // the watchdog: if loop() stops coming round for 5 s (a hang), the chip restarts, and the core carries on at
  // once (resumed()): no 2 min wait for the session, no 15 min start wait
  enableLoopWDT();
}

void loop() {
  static bool usb_up = false;
  static uint32_t last_retry = 0;
  uint32_t now = millis();
  pmu_poll(now);
  imu_poll();
  mic_service(now);
  // the USB link is up while the cable is in: commands are answered at once, no port-open handshake needed
  if (usb_in != usb_up) { dev.link(PORT_USB, usb_in); usb_up = usb_in; }
  char buf[64];
  while (usb_up && Serial.available()) {
    int n = Serial.read((uint8_t*)buf, min(Serial.available(), int(sizeof buf)));
    if (n <= 0) break;
    dev.rx(PORT_USB, buf, n);
  }
  dev.poll();
  now = millis();
  spk_service(now);
  feedback_service(now);
  screen_service(now);
  if (usb_up) drain();
  // the motion sensor stopped answering (or never did), or answers but has had no sample for 3 s (a sensor that
  // reset itself is switched off and reports an empty FIFO for ever, with no error: every window invalid, so no
  // nudge for the rest of the night): set it up again, every 5 s at most
  bool quiet = imu_ok && int32_t(now - imu_data_ms) > 3000;
  if ((!imu_ok || imu_fail > 20 || quiet) && now - last_retry > 5000) {
    last_retry = now;
    if (quiet) imu_quiet++;
    imu_ok = imu_begin();
    imu_fail = 0;
  }
  // Off USB, with nothing playing or showing, sleep until the next mic burst or FIFO read (the FIFO holds 0.21 s).
  // dev.playing(): not in the silent gaps of a vibration pattern either (motor_now is 0 there, and a nap of up to
  // 120 ms cut into the next 80-120 ms pulse)
  bool busy = usb_in || screen_on || spk_on || motor_now || fb_pulses || mic_on || Serial.available() || dev.playing();
  if (!busy) {
    uint32_t wake = min(mic_next(now), imu_last_ms + 120);
    int32_t ms = int32_t(wake - millis());
    if (ms > 3) {
      esp_sleep_enable_timer_wakeup(uint64_t(ms) * 1000);
      gpio_wakeup_enable(gpio_num_t(PMU_IRQ), GPIO_INTR_LOW_LEVEL);   // the button and the USB cable wake it
      esp_sleep_enable_gpio_wakeup();
      esp_light_sleep_start();
      return;
    }
  }
  delay(usb_in ? 2 : 1);
}
