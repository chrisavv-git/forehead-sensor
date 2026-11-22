// Unit tests for the firmware core, run on the laptop: make -C firmware/host test
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <memory>
#include <algorithm>
#include "simbody.h"

using namespace fhs;

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
#define CHECK_NEAR(a, b, tol) do { checks++; double _a = (a), _b = (b); if (fabs(_a - _b) > (tol)) { fails++; \
  printf("  FAIL %s:%d: %s = %g, want %g +- %g\n", __FILE__, __LINE__, #a, _a, _b, double(tol)); } } while (0)

// ---------- a device on a simulated body ----------
struct Rig {
  sim::MemFlash flash;
  sim::SimHal hal;
  sim::Scenario sc;
  std::unique_ptr<sim::Body> body;
  std::unique_ptr<Device> dev;
  std::vector<std::string> out[N_PORTS];
  std::string part[N_PORTS];
  explicit Rig(const std::string& scen, uint32_t flash_size = 2u << 20) : flash(flash_size) {
    if (!sc.parse(scen)) { printf("scenario: %s\n", sc.error.c_str()); exit(2); }
    boot();
  }
  void boot() {   // power-up (also a reboot: flash and saved settings stay)
    hal.imu_rate = float(sc.imu_hz);
    hal.acc_lsb = float(sc.imu_lsb);
    hal.gyro = sc.imu_gyro;
    body.reset(new sim::Body(sc));
    dev.reset(new Device());
    dev->begin(&hal, &flash);
    dev->link(PORT_USB, true);
    dev->link(PORT_BLE, true);
    for (auto& o : out) o.clear();
  }
  void drain() {
    for (int p = 0; p < N_PORTS; p++) {
      const char* d;
      size_t n;
      while ((n = dev->tx(p).peek(&d)) > 0) { part[p].append(d, n); dev->tx(p).drop(n); }
      size_t k;
      while ((k = part[p].find('\n')) != std::string::npos) { out[p].push_back(part[p].substr(0, k)); part[p].erase(0, k + 1); }
    }
  }
  void run(double seconds) {
    uint32_t end = hal.now + uint32_t(seconds * 1000);
    while (hal.now < end) {
      hal.now += 10;
      double t = hal.now / 1000.0;
      hal.usb = sc.in(sc.usb_on, t);
      hal.batt = uint16_t(sc.batt_from + (sc.batt_to - sc.batt_from) * (sc.end > 0 ? std::min(1.0, t / sc.end) : 0));
      body->run_to(*dev, hal.now, hal.motor_level);
      dev->poll();
      drain();
    }
  }
  std::vector<std::string> cmd(const std::string& c, int port = PORT_USB, double wait = 0.2) {
    out[port].clear();
    std::string l = c + "\n";
    dev->rx(port, l.data(), l.size());
    run(wait);
    return out[port];
  }
  static bool starts(const std::string& s, const char* p) { return s.compare(0, strlen(p), p) == 0; }
  std::string find(int port, const char* prefix) {
    for (auto& l : out[port]) if (starts(l, prefix)) return l;
    return "";
  }
};

static bool parse_R(const std::string& l, uint32_t* boot, uint32_t* seq, Record* r) {
  char hex[40];
  unsigned long b, s;
  if (sscanf(l.c_str(), "R %lu %lu %32s", &b, &s, hex) != 3 || strlen(hex) != 32) return false;
  uint8_t* p = reinterpret_cast<uint8_t*>(r);
  for (int i = 0; i < 16; i++) { unsigned v; sscanf(hex + 2 * i, "%2x", &v); p[i] = uint8_t(v); }
  *boot = uint32_t(b);
  *seq = uint32_t(s);
  return true;
}

static bool only_cue(const std::vector<std::pair<uint32_t, uint8_t>>& log) {   // just the bedtime buzz ran
  int on = 0;
  for (auto& e : log) { if (e.second && e.second != 100) return false; on += e.second > 0; }
  return on <= 2;
}

static std::string kv(const std::string& line, const std::string& key) {
  size_t i = line.find(" " + key + "=");
  if (i == std::string::npos) return "";
  i += key.size() + 2;
  return line.substr(i, line.find(' ', i) - i);
}

// ---------- tests ----------

static void test_format() {
  printf("format\n");
  CHECK(sizeof(Record) == 16);
  CHECK(sizeof(BlockHeader) == 48);
  CHECK(RECS_PER_BLOCK == 253);
  CHECK(crc16("123456789", 9) == 0x29B1);   // CRC16-CCITT-FALSE check value
  char buf[32];
  i64_to_dec(buf, -1234567890123LL); CHECK(std::string(buf) == "-1234567890123");
  i64_to_dec(buf, INT64_MIN); CHECK(std::string(buf) == "-9223372036854775808");
  u64_to_dec(buf, 0); CHECK(std::string(buf) == "0");
  fixed_to_dec(buf, -3.14159f, 2); CHECK(std::string(buf) == "-3.14");
  fixed_to_dec(buf, 0.05f, 1); CHECK(std::string(buf) == "0.1");
  fixed_to_dec(buf, -0.04f, 1); CHECK(std::string(buf) == "0.0");
  fixed_to_dec(buf, 45, 1); CHECK(std::string(buf) == "45.0");
  uint8_t d[3] = {0x00, 0xAB, 0xFF};
  hex_encode(buf, d, 3); CHECK(std::string(buf) == "00abff");
}

static void test_angles() {
  printf("angles\n");
  const float D2R = 3.14159265f / 180;
  float z[3] = {0, 0, 1}, r, p;
  for (float roll : {-170.f, -90.f, -30.f, 0.f, 20.f, 45.f, 90.f, 135.f})
    for (float pitch : {-20.f, 0.f, 30.f, 70.f}) {
      float g[3] = {sinf(pitch * D2R), sinf(roll * D2R) * cosf(pitch * D2R), cosf(roll * D2R) * cosf(pitch * D2R)};
      head_angles(g, z, 0, &r, &p);
      CHECK_NEAR(r, roll, 0.01);
      CHECK_NEAR(p, pitch, 0.01);
    }
  // head axis -X: sitting up reads negative X on the board, still pitch +
  float g[3] = {-1, 0, 0};
  head_angles(g, z, 1, &r, &p);
  CHECK_NEAR(p, 90, 0.01);
  // a crooked zero: after zeroing on it, flat reads 0 and a turn reads the turn
  float zc[3] = {0.17f, -0.12f, 0.978f};
  float n = sqrtf(zc[0] * zc[0] + zc[1] * zc[1] + zc[2] * zc[2]);
  for (auto& v : zc) v /= n;
  head_angles(zc, zc, 0, &r, &p);
  CHECK_NEAR(r, 0, 0.01);
  CHECK_NEAR(p, 0, 0.01);
  // head axis along the zero (impossible mounting): falls back, gives finite angles
  float zx[3] = {1, 0, 0};
  head_angles(zx, zx, 0, &r, &p);
  CHECK(r == r && p == p);
  CHECK_NEAR(r, 0, 0.01);
}

static void test_sensing() {
  printf("sensing\n");
  Params P;
  Sensing s;
  Record r{};
  sim::Rng rng;
  auto feed = [&](double secs, double roll, double pitch, double vib_mg, double hz, double dps) {
    const float D2R = 3.14159265f / 180;
    for (int k = 0; k < int(secs * 833); k++) {
      double t = k / 833.0;
      double ax = sin(pitch * D2R), ay = sin(roll * D2R) * cos(pitch * D2R), az = cos(roll * D2R) * cos(pitch * D2R);
      az += vib_mg / 1000 * sin(2 * M_PI * hz * t);
      int16_t raw[6] = {int16_t(ax / 0.000122f + rng.gauss() * 12), int16_t(ay / 0.000122f + rng.gauss() * 12),
                        int16_t(az / 0.000122f + rng.gauss() * 12), int16_t((dps + 2.0) / GYR_DPS_PER_LSB),
                        int16_t(-1.5 / GYR_DPS_PER_LSB), int16_t(0.5 / GYR_DPS_PER_LSB)};
      s.imu(raw);
    }
  };
  for (int w = 0; w < 4; w++) { feed(0.5, 0, 0, 0, 0, 0); s.close(r, P); }
  CHECK_NEAR(r.roll / 100.0, 0, 0.5);
  CHECK_NEAR(r.pitch / 100.0, 0, 0.5);
  CHECK(r.move < 10);                 // bias learnt: still reads < 1 deg/s
  CHECK(r.buzz < 30);                 // noise only: < 3 mg
  CHECK(r.flags & F_NOMIC);
  CHECK(!(r.flags & F_SHORT));
  feed(0.5, 0, 0, 30, 200, 0); s.close(r, P);
  CHECK_NEAR(r.buzz / 10.0, 30 / sqrt(2), 3);   // 30 mg sine -> 21 mg RMS
  feed(0.5, 0, 0, 30, 20, 0); s.close(r, P);
  CHECK(r.buzz < 40);                 // slow (20 Hz) motion is not a buzz
  feed(0.5, 60, 0, 0, 0, 30); s.close(r, P);
  CHECK_NEAR(r.roll / 100.0, 60, 0.5);
  CHECK_NEAR(r.move / 10.0, 30, 1.5); // turning at 30 deg/s
  feed(0.5, 0, 75, 0, 0, 0); s.close(r, P);
  CHECK(r.flags & F_UPRIGHT);
  feed(0.1, 0, 0, 0, 0, 0); s.close(r, P);
  CHECK(r.flags & F_SHORT);
  int16_t tone[1600];
  for (int k = 0; k < 1600; k++) tone[k] = int16_t(1000 * sin(2 * M_PI * 300 * k / 16000.0));
  for (int i = 0; i < 5; i++) s.mic(tone, 1600);
  feed(0.5, 0, 0, 0, 0, 0); s.close(r, P);
  CHECK_NEAR(r.mic, 707, 30);
  CHECK_NEAR(r.snore, 707, 60);       // 300 Hz is inside the snore band
  for (int k = 0; k < 1600; k++) tone[k] = int16_t(1000 * sin(2 * M_PI * 3000 * k / 16000.0));
  for (int i = 0; i < 5; i++) s.mic(tone, 1600);
  feed(0.5, 0, 0, 0, 0, 0); s.close(r, P);
  CHECK_NEAR(r.mic, 707, 30);
  CHECK(r.snore < 120);               // 3 kHz is not
  // bursts (mic_duty 25): each starts at a different DC level after the mic's switch-on. In silence the readings
  // stay near 0 (30-31 Oct: 750-930 "snore" in silence, the filters rang at every jump), a sound still shows
  int16_t burst[2000];
  int worst_mic = 0, worst_snore = 0;
  for (int b = 0; b < 8; b++) {
    for (int k = 0; k < 2000; k++) burst[k] = int16_t((b % 2 ? 1800 : -900) + (k % 7) - 3);
    s.mic_start();
    s.mic(burst, 2000);
    feed(0.5, 0, 0, 0, 0, 0); s.close(r, P);
    worst_mic = std::max(worst_mic, int(r.mic));
    worst_snore = std::max(worst_snore, int(r.snore));
  }
  CHECK(worst_mic < 15);
  CHECK(worst_snore < 15);
  for (int k = 0; k < 2000; k++) burst[k] = int16_t(1800 + 1000 * sin(2 * M_PI * 300 * k / 16000.0));
  s.mic_start();
  s.mic(burst, 2000);
  feed(0.5, 0, 0, 0, 0, 0); s.close(r, P);
  CHECK(r.snore > 500);               // a 300 Hz sound in a burst is still heard
}

static void test_therapy() {
  printf("therapy\n");
  Params P;
  P.angle_deg = 45;
  P.jitter_s = 0;
  P.vary = 0;
  Therapy T;
  auto at = [&](double s) { return uint32_t(s * 1000); };
  CHECK(T.tick(at(0), 0, false, true, P) == 0);   // no session: nothing
  T.session_start(at(0));
  std::map<int, int> first;   // step -> first second it appeared
  for (double t = 0; t < 1200; t += 0.5) {
    int n = T.tick(at(t), 2, false, true, P);
    if (n && !first.count(nudge_step(n))) first[nudge_step(n)] = int(t);
  }
  CHECK(first[1] == 910);     // 900 s start wait + 10 s delay
  CHECK(first[2] == 920);
  CHECK(first[3] == 930);
  CHECK(first[4] == 940);
  CHECK(T.step() == 4);                                 // step 4 repeats until the head turns
  CHECK(T.tick(at(1200), 48, false, true, P) == 0x40);  // inside the 5 deg hysteresis: still flat
  CHECK(T.tick(at(1200.5), 51, false, true, P) == 0);  // turned: stops
  CHECK(T.tick(at(1201), 46, false, true, P) == 0);    // 46 deg: not flat again until under 45
  for (double t = 1201; t < 1211.5; t += 0.5) T.tick(at(t), 0, false, true, P);
  CHECK(T.step() == 1);                                 // a new flat stay: 10 s again, starts at step 1
  T.tick(at(1240), 0, true, true, P);                   // head "up" for one window: a roll's blip, not sitting up
  int after = 0;
  for (double t = 1240.5; t < 1240 + 400; t += 0.5) if (T.tick(at(t), 0, false, true, P)) { after = int(t - 1240); break; }
  CHECK(after == 10);                                   // no pause: the normal 10 s delay
  T.tick(at(1300), 60, false, true, P);                 // turned away: stops
  for (double t = 1300.5; t < 1303; t += 0.5) T.tick(at(t), 0, true, true, P);   // 2.5 s up: still not a sit-up
  after = 0;
  for (double t = 1303; t < 1303 + 400; t += 0.5) if (T.tick(at(t), 0, false, true, P)) { after = int(t - 1303); break; }
  CHECK(after == 10);
  T.tick(at(1400), 60, false, true, P);
  for (double t = 1400.5; t < 1403.5; t += 0.5) T.tick(at(t), 0, true, true, P);   // 3 s up in a row: sat up
  after = 0;
  for (double t = 1403.5; t < 1403.5 + 400; t += 0.5) if (T.tick(at(t), 0, false, true, P)) { after = int(t - 1403.5); break; }
  CHECK(after == 60);                                   // the pause (300 s) ends after 60 s flat: the safety net
  T.tick(at(1600), 60, false, true, P);
  for (double t = 1600.5; t < 1610.5; t += 0.5) T.tick(at(t), 0, true, true, P);   // sat up 10 s
  for (double t = 1610.5; t < 1700; t += 0.5) T.tick(at(t), 80, false, true, P);   // back on the side 90 s
  after = 0;
  for (double t = 1700; t < 1700 + 400; t += 0.5) if (T.tick(at(t), 0, false, true, P)) { after = int(t - 1700); break; }
  CHECK(after == 60);                                   // the net counts flat time, not time since sitting up
  CHECK(T.tick(at(2000), 0, false, false, P) == 0);     // no motion data: never nudge
  {   // the hold after a zero ends for good: it must not come back when the 32-bit clock comparison wraps, about
      // 25 days later (the watch runs for weeks; only a new zero or a restart used to reset it)
    Therapy H;
    H.session_start(0, true);
    H.hold(at(300));
    int on = 0, first = 0;
    for (double t = 0; t < 330; t += 0.5)
      if (H.tick(at(t), 0, false, true, P)) { on++; if (!first) first = int(t); }
    CHECK(first == 310 && on == 40);                    // held 300 s, then the normal 10 s delay
    H.tick(at(330), 90, false, true, P);                // turned to the side
    const uint32_t day = 86400000u;
    for (uint32_t d : {24u, 25u, 26u, 40u, 48u}) {      // on the back again, weeks later: nudged as ever
      first = -1;
      for (int k = 0; k < 60; k++)
        if (H.tick(d * day + uint32_t(k) * 500, 0, false, true, P)) { first = k / 2; break; }
      CHECK(first == 10);
      H.tick(d * day + 40000, 90, false, true, P);
    }
  }
  {
    Params Q = P;
    Q.max_buzz_s = 60;
    Therapy U;
    U.session_start(0);
    int on = 0;
    for (double t = 0; t < 1200; t += 0.5) on += U.tick(at(t), 0, false, true, Q) > 0;
    CHECK(on == 120);                                   // gives up after 60 s of nudging
  }
  {   // the defaults: a random variant every step, never the same twice in a row; every delay and step moves
      // by up to 3 s
    Params Q;
    Therapy U;
    U.seed(99);
    U.session_start(0);
    std::set<int> variants, delays, lengths;
    int last = -1, repeats = 0, stays = 0;
    double t = 0, flat_from = 0, step_from = -1;
    int step = 0;
    for (; stays < 200; t += 0.5) {
      double cycle = fmod(t, 100);   // 60 s flat, 40 s on the side
      int n = U.tick(at(900 + t), cycle < 60 ? 0 : 90, false, true, Q);
      if (cycle == 0) { flat_from = t; step = 0; }
      if (cycle == 60) stays++;
      if (!n) continue;
      if (nudge_step(n) == 1 && step == 0) delays.insert(int((t - flat_from) * 2));
      if (step && nudge_variant(n) != last) lengths.insert(int((t - step_from) * 2));
      if (!step || nudge_variant(n) != last) {
        if (nudge_variant(n) == last) repeats++;
        variants.insert(nudge_variant(n));
        last = nudge_variant(n);
        step_from = t;
      }
      step = nudge_step(n);
    }
    CHECK(repeats == 0);
    CHECK(variants.size() == 16);
    CHECK(!delays.empty() && *delays.begin() == 14 && *delays.rbegin() == 26);    // 7-13 s, in half seconds
    CHECK(!lengths.empty() && *lengths.begin() == 14 && *lengths.rbegin() == 26);
  }
  // what a nudge plays
  std::set<int> pairs;
  for (int v = 0; v < N_VARIANTS; v++) pairs.insert(vib_pattern(uint8_t(v)) * 8 + sound_pattern(uint8_t(v)));
  CHECK(pairs.size() == 16);                            // every variant is a different (vibration, sound) pair
  Output o = nudge_output(0x10, 100, P);                // step 1, variant 0 (steady pulse): 50 %, no sound
  CHECK(o.motor == 50 && o.hz == 0 && o.vol == 0);
  CHECK(nudge_output(0x10, 600, P).motor == 0);         // the pulse's off half
  CHECK(nudge_output(0x10, 1100, P).motor == 50);       // and it loops
  o = nudge_output(0x30, 100, P);                       // step 3: vibration + the double beep at 30 %
  CHECK(o.motor == 100 && o.hz == 2000 && o.vol == 30);
  CHECK(nudge_output(0x30, 200, P).hz == 0);            // between the beeps
  CHECK(nudge_output(0x40, 100, P).vol == 100);         // step 4: loud
  o = nudge_output(0x44, 400, P);                       // variant 4: the ramp (20 -> 100 % over 0.8 s), half way
  CHECK(o.motor == 60);
  o = nudge_output(0x43, 200, P);                       // variant 3's sound: rising chirp 600 -> 2500 Hz over 0.4 s
  CHECK(o.hz == 1550);
  for (int v = 0; v < N_VARIANTS; v++) CHECK(sound_pattern(uint8_t(v)) < 7);   // no name clip any more
  o = nudge_output(0x45, 100, P);                       // variant 5, the name clip until 0.0.2: the warble now
  CHECK(o.hz == 1500 && o.vol == 100);
  CHECK(nudge_output(0x4A, 10, P).hz == 3000);          // variant 10, the same: the fast beeps
  CHECK(nudge_output(0x46, 100, P).hz == 520);          // variant 6's sound: the low buzz
  CHECK(nudge_output(0, 100, P).motor == 0 && nudge_output(0x80, 100, P).motor == 0);
  CHECK(nudge_output(STEP_TEST << 4, 100, P, 3).vol == 30);   // a test plays the step it was given
}

static void test_ringlog() {
  printf("ringlog\n");
  sim::MemFlash f(16 * 4096);   // 16 sectors: wraps quickly
  RingLog L;
  CHECK(L.begin(&f));
  CHECK(L.boot_id() == 1 && L.used() == 0 && L.oldest() < 0);
  BlockInfo bi;
  auto rec = [](uint32_t seq) { Record r{}; r.roll = int16_t(seq % 30000); r.nudge = 0x80; r.seq = uint16_t(seq); return r; };
  for (uint32_t s = 0; s < 253 * 20 + 100; s++) L.append(rec(s), s, bi);   // 20.4 blocks into 16 sectors
  CHECK(f.violations == 0);
  CHECK(f.max_programs <= 3);       // header, first page's records, the closing count
  CHECK(L.used() == 16);
  int s = L.oldest(), n = 0;
  uint32_t expect = L.entry(s).first_seq, prev_block = 0;
  bool ok = true;
  while (s >= 0) {
    const RingLog::Entry& e = L.entry(s);
    if (n && e.block_no != prev_block + 1) ok = false;
    for (int i = 0; i < e.n; i++) {
      Record r;
      L.read_record(s, i, &r);
      if (!record_ok(r, e.first_seq, i) || e.first_seq + i != expect || r.roll != int16_t(expect % 30000)) ok = false;
      expect++;
    }
    prev_block = e.block_no;
    s = L.next_in_order(s);
    n++;
  }
  CHECK(ok);
  CHECK(expect == 253 * 20 + 100);
  CHECK(n == 16);
  // power lost mid-block: the scan finds every record of the flushed pages, the boot id goes up
  RingLog L2;
  L2.begin(&f);
  CHECK(L2.boot_id() == 2);
  int newest = -1;
  for (int k = 0; k < 16; k++) if (L2.entry(k).valid && (newest < 0 || L2.entry(k).block_no > L2.entry(newest).block_no)) newest = k;
  CHECK(L2.entry(newest).n == 93);   // 100 records: 48 + 93*16 = 1536 = 6 full pages; 7 records were in RAM
  // a gap in seq closes the block; a new boot writes its own blocks
  L2.append(rec(0), 0, bi);
  L2.append(rec(1), 1, bi);
  L2.append(rec(10), 10, bi);
  int a = L2.open_sector();
  CHECK(L2.entry(a).first_seq == 10 && L2.entry(a).boot_id == 2);
  L2.close();
  CHECK(!L2.is_open());
  // a catch-up's next block: the same one the walk from the oldest finds, for any boot and seq (the ring has
  // wrapped, so sector order is not block order)
  bool same = true;
  int found = 0;
  for (uint32_t boot = 0; boot <= 3; boot++)
    for (uint32_t seq : {0u, 1u, 2u, 9u, 10u, 11u, 1300u, 1517u, 1518u, 3000u, 5065u, 5152u, 5153u, 5160u, 9999u}) {
      int w = L2.oldest();
      while (w >= 0 && (L2.entry(w).boot_id != boot || L2.entry(w).first_seq + L2.entry(w).n <= seq)) w = L2.next_in_order(w);
      if (L2.first_since(boot, seq) != w) same = false;
      found += w >= 0;
    }
  CHECK(same);
  CHECK(found >= 12);   // (boot 1 up to its last record, boot 2 up to seq 10)
  // erase: nothing left, and the next boot doesn't reuse the counters
  uint32_t nb = L2.next_block_no();
  L2.erase_all();
  CHECK(L2.used() == 0);
  RingLog L3;
  L3.begin(&f, 2, nb);
  CHECK(L3.used() == 0 && L3.boot_id() == 3 && L3.next_block_no() == nb);
  // broken flash: no crash, errors counted
  f.fail = true;
  L3.append(rec(0), 0, bi);
  CHECK(L3.errors > 0);
}

static const char* NIGHT =
    "t0 1792623600\n"           // 2026-10-22 01:00 local (UTC+2)... time is set by the test
    "bell 40 20 60 200 30\nbellwait 0 0\n"
    "seg 30 pitch 80\n"         // sitting
    "seg 5 pitch 0 ramp\n"      // lies down flat on the back
    "seg 1200 roll 3\n"         // 20 min on the back
    "seg 3 roll 80 ramp\n"      // turns to the left
    "seg 600 roll 80 snore\n"
    "seg 3 roll -5 ramp\n"
    "seg 400 roll -5\n";

static void test_device_protocol() {
  printf("device protocol\n");
  Rig g("seg 3600 pitch 0\n");
  auto r = g.cmd("hello");
  CHECK(!r.empty() && Rig::starts(r[0], "OK hello fw=" FHS_FW_VERSION));
  CHECK(kv(r[0], "auth") == "1");
  // Bluetooth needs the key
  r = g.cmd("status", PORT_BLE);
  CHECK(!r.empty() && r[0] == "ERR auth needed");
  r = g.cmd("key", PORT_USB);
  CHECK(!r.empty() && Rig::starts(r[0], "OK key "));
  std::string key = r[0].substr(7);
  CHECK(key.size() == 16);
  CHECK(g.cmd("key", PORT_BLE)[0] == "ERR auth needed");
  CHECK(g.cmd("auth 0000000000000000", PORT_BLE)[0] == "ERR auth");
  CHECK(g.cmd("auth " + key, PORT_BLE)[0] == "OK auth");
  CHECK(g.cmd("key", PORT_BLE)[0] == "ERR key only over USB");
  CHECK(g.cmd("raw imu", PORT_BLE)[0] == "ERR raw only over USB");
  g.dev->link(PORT_BLE, false);   // reconnect: key again
  g.dev->link(PORT_BLE, true);
  CHECK(g.cmd("status", PORT_BLE)[0] == "ERR auth needed");
  // status and time
  r = g.cmd("status");
  CHECK(r.size() == 3 && kv(r[0], "mode") == "shadow" && kv(r[0], "clock") == "0");
  r = g.cmd("time 1792623600000 120");
  CHECK(!r.empty() && Rig::starts(r[0], "OK time t0="));
  long long t0 = atoll(kv(r[0], "t0").c_str());
  CHECK(llabs(t0 + (long long)g.hal.now - 200 - 1792623600000LL) < 20);   // t0 = unix - uptime
  CHECK(g.cmd("time 12")[0] == "ERR time");
  CHECK(g.cmd("time 1792623600000x")[0] == "ERR time");
  // settings: ranges and types are checked
  CHECK(g.cmd("set angle 40")[0] == "OK set angle=40");
  CHECK(g.dev->params().angle_deg == 40);
  CHECK(g.cmd("set angle 400")[0] == "ERR set angle");
  CHECK(g.cmd("set angle abc")[0] == "ERR set angle");
  CHECK(g.cmd("set delay 2.5")[0] == "ERR set delay");
  CHECK(g.cmd("set levels 10,20,30,40")[0] == "OK set levels=10,20,30,40");
  CHECK(g.dev->params().level[3] == 40);
  CHECK(g.cmd("set levels 10,20,30")[0] == "ERR set levels");
  CHECK(g.cmd("set levels 10,20,30,400")[0] == "ERR set levels");
  CHECK(g.cmd("set nosuch 1")[0] == "ERR set nosuch");
  r = g.cmd("get");
  CHECK(!r.empty() && kv(r[0], "angle") == "40.0" && kv(r[0], "levels") == "10,20,30,40" && kv(r[0], "mode") == "shadow");
  CHECK(g.hal.have_params && g.hal.saved_params.angle_deg == 40);   // persisted
  CHECK(g.cmd("mode therapy")[0] == "OK mode therapy");
  CHECK(g.cmd("mode bogus")[0] == "ERR unknown command: mode");
  CHECK(g.cmd("mode shadow")[0] == "OK mode shadow");
  CHECK(g.cmd("axis 2")[0] == "OK axis 2" && g.dev->params().head_axis == 2);
  CHECK(g.cmd("axis 4")[0] == "ERR unknown command: axis");
  g.cmd("axis 0");
  // a too-long line is refused, not cut
  std::string lng(300, 'a');
  CHECK(g.cmd(lng)[0] == "ERR line too long");
  CHECK(g.cmd("frobnicate")[0] == "ERR unknown command: frobnicate");
  // buzz test drives the motor for the asked time, then stops; limits are checked
  CHECK(g.cmd("buzz 101 100")[0] == "ERR buzz <1-100 %> <1-5000 ms>");
  CHECK(g.cmd("buzz 50 6000")[0] == "ERR buzz <1-100 %> <1-5000 ms>");
  g.hal.motor_log.clear();
  g.cmd("buzz 60 300", PORT_USB, 1.0);
  CHECK(g.hal.motor_log.size() == 2 && g.hal.motor_log[0].second == 60 && g.hal.motor_log[1].second == 0);
  if (g.hal.motor_log.size() == 2) CHECK_NEAR(g.hal.motor_log[1].first - g.hal.motor_log[0].first, 300, 20);
  // zero: needs stillness; lying flat it works
  r = g.cmd("zero");
  CHECK(r.size() >= 1 && r.back() == "OK zero");
  CHECK(g.dev->params().zero_set == 1);
}

static void hal_run_without_drain(Rig& g, double secs) {
  uint32_t end = g.hal.now + uint32_t(secs * 1000);
  while (g.hal.now < end) { g.hal.now += 10; g.body->run_to(*g.dev, g.hal.now, 0); g.dev->poll(); }
}

static void test_live_and_catchup() {
  printf("live, since, dump\n");
  Rig g("seg 4000 pitch 0\n");
  g.cmd("set win_from 0");   // logging gate: no window
  g.cmd("set win_to 0");
  g.run(200);                // lying 200 s: the session starts after 60 s and logs from the start of lying
  CHECK(g.dev->session());
  uint32_t boot = g.dev->log().boot_id();
  // live
  g.cmd("live on");
  g.out[PORT_USB].clear();
  g.run(10);
  int live = 0;
  uint32_t b, s, last = 0;
  Record r;
  for (auto& l : g.out[PORT_USB]) if (parse_R(l, &b, &s, &r)) { live++; CHECK(b == boot); if (live > 1) CHECK(s == last + 1); last = s; }
  CHECK(live == 20);
  g.cmd("live off");
  // catch up from the start of the boot: every record logged, once, in order, then live
  g.out[PORT_USB].clear();
  std::string c = "since " + std::to_string(boot) + " 0 live\n";
  g.dev->rx(PORT_USB, c.data(), c.size());
  g.run(30);
  std::vector<uint32_t> seqs;
  bool caught = false;
  for (auto& l : g.out[PORT_USB]) {
    if (parse_R(l, &b, &s, &r)) { seqs.push_back(s); CHECK(record_ok(r, 0, s)); }
    if (Rig::starts(l, "C ")) caught = true;
  }
  CHECK(caught);
  CHECK(!seqs.empty() && seqs[0] <= 2);   // backfilled from when lying began (the first second is sitting)
  bool inorder = true;
  for (size_t i = 1; i < seqs.size(); i++) if (seqs[i] != seqs[i - 1] + 1) inorder = false;
  CHECK(inorder);
  CHECK(!seqs.empty() && seqs.back() + 1 == g.dev->seq());   // ... and then live up to now, no gap
  // block list and dump
  g.cmd("live off");
  g.out[PORT_USB].clear();
  g.dev->rx(PORT_USB, "blocks\n", 7);
  g.run(2);
  int iblocks = 0;
  for (auto& l : g.out[PORT_USB]) iblocks += Rig::starts(l, "I ");
  CHECK(iblocks == g.dev->log().used());
  CHECK(g.find(PORT_USB, "OK blocks") == "OK blocks n=" + std::to_string(iblocks));
  g.out[PORT_USB].clear();
  g.dev->rx(PORT_USB, "dump\n", 5);
  g.run(5);
  int chunks = 0;
  for (auto& l : g.out[PORT_USB]) chunks += Rig::starts(l, "B ");
  CHECK(chunks == 16 * iblocks);
  CHECK(g.find(PORT_USB, "OK dump") == "OK dump n=" + std::to_string(iblocks));
  // a range: "dump <from> <n>" gives exactly those blocks
  g.out[PORT_USB].clear();
  int first = g.dev->log().entry(g.dev->log().oldest()).block_no;
  std::string dc = "dump " + std::to_string(first + 1) + " 1\n";
  g.dev->rx(PORT_USB, dc.data(), dc.size());
  g.run(3);
  chunks = 0;
  for (auto& l : g.out[PORT_USB]) if (Rig::starts(l, "B ")) { chunks++; CHECK(atoi(l.c_str() + 2) == first + 1); }
  CHECK(chunks == 16 && g.find(PORT_USB, "OK dump") == "OK dump n=1");
  // since a seq in the middle: starts exactly there
  g.out[PORT_USB].clear();
  c = "since " + std::to_string(boot) + " 300\n";
  g.dev->rx(PORT_USB, c.data(), c.size());
  g.run(5);
  for (auto& l : g.out[PORT_USB]) if (parse_R(l, &b, &s, &r)) { CHECK(s == 300); break; }
  // a full link drops live records instead of blocking, and status counts them
  g.cmd("live on");
  std::string junk(TxRing::SIZE - 100, 'x');
  g.dev->tx(PORT_USB).put(junk.data(), junk.size());   // the laptop stopped reading
  hal_run_without_drain(g, 4);                         // (under TX_STALL_MS: the link is not given up yet)
  g.dev->tx(PORT_USB).clear();
  auto st = g.cmd("status");
  CHECK(st.size() >= 2 && atoi(kv(st[1], "dropped").c_str()) >= 5);   // ~8 records, 2 fit in the last 99 bytes
  CHECK(kv(st[1], "stalls") == "0");
  g.cmd("stop");
  // the laptop closes the port in the middle of a dump (the watch cannot see that, only that nothing is taken):
  // after TX_STALL_MS the dump, live and the waiting bytes are dropped, and the next command is answered at once
  g.cmd("live on");
  g.dev->rx(PORT_USB, "dump\n", 5);
  hal_run_without_drain(g, 2);
  CHECK(g.dev->tx(PORT_USB).room() < 600);              // the ring is full of dump lines, the job waits for room
  CHECK(g.hal.stalled[PORT_USB] == 0);
  hal_run_without_drain(g, 4);
  CHECK(g.hal.stalled[PORT_USB] == 1);                  // told the hardware layer once
  CHECK(g.dev->tx(PORT_USB).count() == 0);
  g.part[PORT_USB].clear();
  g.out[PORT_USB].clear();
  g.run(3);                                             // the laptop reads again: nothing comes by itself
  CHECK(g.out[PORT_USB].empty());
  st = g.cmd("status");
  CHECK(st.size() >= 2 && st.back() == "OK status" && kv(st[1], "stalls") == "1");
  // a reader that is only slow is not given up: a dump read at 200 bytes/s for 12 s goes on, and ends whole
  g.out[PORT_USB].clear();
  g.dev->rx(PORT_USB, "dump\n", 5);
  for (int k = 0; k < 1200; k++) {
    g.hal.now += 10;
    g.body->run_to(*g.dev, g.hal.now, 0);
    g.dev->poll();
    const char* d;
    size_t n = g.dev->tx(PORT_USB).peek(&d);
    if (n > 2) n = 2;
    g.part[PORT_USB].append(d, n);
    g.dev->tx(PORT_USB).drop(n);
  }
  CHECK(g.hal.stalled[PORT_USB] == 1);
  g.run(10);
  chunks = 0;
  for (auto& l : g.out[PORT_USB]) chunks += Rig::starts(l, "B ");
  CHECK(chunks == 16 * g.dev->log().used());
  CHECK(Rig::starts(g.find(PORT_USB, "OK dump"), "OK dump n="));
  // a charger (cable in, nobody there): notes pile up unread, are dropped, and the link works when someone comes
  g.dev->double_tap();
  hal_run_without_drain(g, 8);
  CHECK(g.hal.stalled[PORT_USB] == 2 && g.dev->tx(PORT_USB).count() < 200);
  g.part[PORT_USB].clear();
  st = g.cmd("status");
  CHECK(!st.empty() && st.back() == "OK status");
}

static void test_session_gate() {
  printf("logging gate\n");
  {   // on USB power (charger): no log
    Rig g("usb on\nseg 400 pitch 0\n");
    g.run(300);
    CHECK(!g.dev->session() && g.dev->log().used() == 0);
    g.cmd("set log_usb 1");
    g.run(130);
    CHECK(g.dev->session());
  }
  {   // sitting up: no session; lying 1 min: session
    Rig g("seg 300 pitch 80\nseg 400 pitch 0\n");
    g.cmd("set win_from 0"); g.cmd("set win_to 0");
    g.run(290);
    CHECK(!g.dev->session());
    g.run(50);
    CHECK(!g.dev->session());
    g.run(30);
    CHECK(g.dev->session());
  }
  {   // the night window: with the clock at 14:00 local, nothing; at 23:00 it logs
    Rig g("seg 900 pitch 0\n");
    g.cmd("set win_from 19"); g.cmd("set win_to 12");
    g.cmd("time 1792584000000 120");   // 2026-10-22 12:40 UTC = 14:40 local
    g.run(300);
    CHECK(!g.dev->session());
    g.cmd("time 1792616400000 120");   // 21:40 UTC = 23:40 local
    g.run(130);
    CHECK(g.dev->session());
    g.cmd("time 1792656000000 120");   // 08:40 UTC = 10:40 local, still inside 19-12
    g.run(5);
    CHECK(g.dev->session());
    g.cmd("time 1792663200000 120");   // 12:40 local: window over, session ends
    g.run(5);
    CHECK(!g.dev->session());
  }
  {   // no window by default: a nap at 14:40 local logs too
    Rig g("seg 300 pitch 0\n");
    g.cmd("time 1792584000000 120");
    g.run(130);
    CHECK(g.dev->session());
  }
  {   // a bathroom trip keeps the session; 20 min up ends it
    Rig g("seg 300 pitch 0\nseg 600 pitch 85\nseg 300 pitch 0\nseg 1300 pitch 85\n");
    g.cmd("set win_from 0"); g.cmd("set win_to 0");
    g.run(290);
    CHECK(g.dev->session());
    g.run(600);
    CHECK(g.dev->session());
    g.run(300 + 1220);
    CHECK(!g.dev->session());
  }
  {   // empty battery: logging stops cleanly (block closed), motor stays off
    Rig g("batt 3400 3200\nseg 1000 pitch 0\n");
    g.cmd("set win_from 0"); g.cmd("set win_to 0");
    g.run(300);
    CHECK(g.dev->session());
    g.run(600);
    CHECK(!g.dev->session() && !g.dev->log().is_open());
    int used = g.dev->log().used();
    g.hal.batt = 3350;   // recovers a little: no restart (no new tiny blocks)
    Rig* rg = &g;
    for (int k = 0; k < 300 * 100; k++) { rg->hal.now += 10; rg->body->run_to(*rg->dev, rg->hal.now, 0); rg->dev->poll(); }
    CHECK(!g.dev->session() && g.dev->log().used() == used);
  }
}

static void test_shadow_and_therapy() {
  printf("shadow and therapy\n");
  for (int mode = 0; mode < 2; mode++) {
    Rig g(NIGHT);
    g.cmd("set win_from 0"); g.cmd("set win_to 0");
    g.cmd("set start_wait 0");
    g.cmd("set upright_pause 0");   // (sitting at the start would hold it off for 5 min)
    if (mode) g.cmd("mode therapy");
    g.hal.motor_log.clear();
    g.hal.tone_log.clear();
    g.run(30 + 5 + 400);   // on the back from 35 s; session at ~155 s; 10 s delay
    uint8_t n = g.dev->last().nudge;
    if (!mode) {
      CHECK(only_cue(g.hal.motor_log) && g.hal.tone_log.empty());   // shadow: nothing runs (but the bedtime buzz)
      CHECK((n & NUDGE_SHADOW) && nudge_step(n) == 4);             // but the would-be nudge is logged
    } else {
      CHECK(!g.hal.motor_log.empty());
      CHECK(nudge_step(n) == 4 && !(n & NUDGE_SHADOW));
      int ons = 0, loud = 0;
      for (auto& m : g.hal.motor_log) ons += m.second > 0;
      for (auto& m : g.hal.tone_log) loud += m.second == 100;
      CHECK(ons > 100);                 // patterns, not one long buzz
      CHECK(loud > 10);                 // step 4 beeps at full volume
    }
  }
  {   // while a nudge plays, playing() holds through the silent gaps of its pattern too (the watch must not nap
      // there), and it is off again once the head has turned
    Rig p(NIGHT);
    p.cmd("set win_from 0"); p.cmd("set win_to 0"); p.cmd("set start_wait 0"); p.cmd("set upright_pause 0");
    p.cmd("mode therapy");
    p.run(300);
    CHECK(nudge_step(p.dev->last().nudge) >= 1);
    int gaps = 0, naps = 0;
    for (int k = 0; k < 2000; k++) {   // 20 s of nudging, every 10 ms
      p.run(0.01);
      if (p.hal.motor_level == 0) { gaps++; naps += !p.dev->playing(); }
    }
    CHECK(gaps > 200 && naps == 0);
    p.run(1260 - p.hal.now / 1000.0);   // on the left side from 1238 s
    CHECK(p.hal.motor_level == 0 && !p.dev->playing());
    p.cmd("buzz 100 300", PORT_USB, 0.1);   // a test buzz plays, then it is quiet again
    CHECK(p.dev->playing());
    p.run(0.5);
    CHECK(!p.dev->playing());
    p.dev->rx(PORT_USB, "nudge 1 3 2000\n", 15);
    gaps = naps = 0;
    for (int k = 0; k < 150; k++) { p.run(0.01); if (p.hal.motor_level == 0) { gaps++; naps += !p.dev->playing(); } }
    CHECK(gaps > 20 && naps == 0);      // variant 3, the triple knock: 80 ms pulses with 80 ms gaps
    p.run(1);
    CHECK(!p.dev->playing());
  }
  // stopping: in therapy mode, switching to shadow stops the motor and the sound at once
  Rig g(NIGHT);
  g.cmd("set win_from 0"); g.cmd("set win_to 0"); g.cmd("set start_wait 0"); g.cmd("mode therapy");
  g.run(400);
  g.cmd("mode shadow");
  g.run(2);
  CHECK(g.hal.motor_level == 0 && g.hal.tone_vol == 0);
  // turning stops it too: on the left side (from 1238 s) nothing runs
  Rig h(NIGHT);
  h.cmd("set win_from 0"); h.cmd("set win_to 0"); h.cmd("set start_wait 0"); h.cmd("set upright_pause 0");
  h.cmd("mode therapy");
  h.run(1230);
  CHECK(h.hal.motor_log.size() > 100);
  h.run(15);
  h.hal.motor_log.clear();
  h.hal.tone_log.clear();
  h.run(60);
  CHECK(h.hal.motor_log.empty() && h.hal.tone_log.empty() && h.hal.motor_level == 0 && h.hal.tone_vol == 0);
}

static void test_watch() {
  printf("watch (accelerometer only, clock chip, sound)\n");
  // the same night on the watch's BMA423: 800 Hz, 12 bits, no gyro
  Rig g(std::string("watch\nmount 5 -3 0\n") + NIGHT);
  g.cmd("set win_from 0"); g.cmd("set win_to 0");
  g.run(60);
  CHECK(g.cmd("zero").back() == "OK zero");
  g.run(60);
  CHECK_NEAR(g.dev->last().roll / 100.0, 0, 1.5);   // zeroed while lying at roll 3
  CHECK(!(g.dev->last().flags & F_SHORT));
  CHECK(g.dev->last().move < 20);                   // lying still: under 2 deg/s from the accelerometer alone
  double peak = 0;
  g.run(1230 - g.hal.now / 1000.0);
  for (int k = 0; k < 20; k++) { g.run(0.5); peak = std::max(peak, g.dev->last().move / 10.0); }
  CHECK(peak > 15);                                 // the turn (77 deg in 3 s, ~26 deg/s) shows
  g.run(30);
  CHECK_NEAR(g.dev->last().roll / 100.0, 77, 3);
  // the clock chip: after a reboot the time is known without the laptop
  CHECK(!g.dev->last().flags || !(g.dev->last().flags & F_CLOCK));
  CHECK(g.cmd("time 1792623600000 120")[0].rfind("OK time", 0) == 0);
  g.run(1);
  CHECK(g.dev->last().flags & F_CLOCK);
  g.boot();
  g.run(1);
  CHECK(g.dev->last().flags & F_CLOCK);
  auto st = g.cmd("status");
  CHECK(st.size() >= 1 && st[0].find(" tz=120 ") != std::string::npos);
  // a nudge test plays one step with its sound, then stops; limits are checked
  CHECK(g.cmd("nudge 5 0 100")[0] == "ERR nudge <step 1-4> <variant 0-15> <1-10000 ms>");
  CHECK(g.cmd("nudge 3 16 100")[0] == "ERR nudge <step 1-4> <variant 0-15> <1-10000 ms>");
  g.hal.tone_log.clear();
  g.hal.motor_log.clear();
  CHECK(g.cmd("nudge 4 0 2000", PORT_USB, 3)[0] == "OK nudge");
  CHECK(!g.hal.tone_log.empty() && g.hal.tone_log[0].second == 100);
  CHECK(!g.hal.motor_log.empty() && g.hal.motor_level == 0 && g.hal.tone_vol == 0);
  CHECK(g.hal.tone_log.back().first - g.hal.tone_log.front().first < 2100);
  // a hang in the night (watchdog restart): nudging resumes within seconds, not after 1 min + the 15 min wait
  for (int hang = 0; hang < 2; hang++) {
    Rig h(std::string("watch\n") + NIGHT);
    h.cmd("set win_from 0"); h.cmd("set win_to 0"); h.cmd("set upright_pause 0"); h.cmd("mode therapy");
    h.run(600);                                     // on the back since 35 s; session from ~95 s; still in the wait
    CHECK(only_cue(h.hal.motor_log));              // the bedtime buzz only
    h.hal.resumed_ = hang;
    h.boot();
    h.hal.motor_log.clear();
    h.run(15);                                      // 10 s delay (+-3 s jitter)
    CHECK(hang ? !h.hal.motor_log.empty() : h.hal.motor_log.empty());
    CHECK(h.dev->session() == bool(hang));          // a power-up waits 1 min lying for its session
    if (hang) CHECK(std::find(h.hal.events.begin(), h.hal.events.end(), "session resumed after a restart") != h.hal.events.end());
  }
  // the watch hears the notes (it confirms a zero with its own buzz)
  g.hal.events.clear();
  g.cmd("zero");
  CHECK(!g.hal.events.empty() && g.hal.events.back() == "zero set");
}

static void test_tap_zero() {
  printf("tap zero\n");
  // crooked tape: 12 deg roll, 8 deg pitch off. A double tap lying flat, then 3 s still, sets the zero
  Rig g("mount 12 8 0\nseg 20 pitch 0\ntap\nseg 30 pitch 0\nseg 3 roll 40 ramp\nseg 30 roll 40\ntap\nseg 30 roll 40\n"
        "seg 3 roll 10 ramp\nseg 10 roll 10\ntap\nseg 30 roll 10\n");
  g.run(15);
  CHECK(fabs(g.dev->last().roll / 100.0) > 8);   // before: the tape's tilt shows
  g.run(20);
  CHECK(g.dev->params().zero_set == 1);
  CHECK_NEAR(g.dev->last().roll / 100.0, 0, 1);
  CHECK_NEAR(g.dev->last().pitch / 100.0, 0, 1);
  g.run(40);
  CHECK_NEAR(g.dev->last().roll / 100.0, 40, 1.5);   // a real turn still reads right
  // no limit any more (31 Oct): a press lying 40 deg to the side takes that as the zero (the voice says so out loud)
  g.run(35);
  CHECK_NEAR(g.dev->last().roll / 100.0, 0, 1.5);
  g.run(3 + 10 + 30);
  CHECK_NEAR(g.dev->last().roll / 100.0, 0, 1);
  // moving for a second right after the press (the arm coming down) no longer spoils it: the spoken countdown
  // gives the time, and the zero is measured after it
  Rig h("seg 20 pitch 0\ntap\nseg 0.5 roll 6 ramp\nseg 0.5 roll 0 ramp\nseg 30 roll 0\n");
  h.run(40);
  CHECK(h.dev->params().zero_set == 1);
}

// the phrases the spoken zero said, in order
static std::vector<int> said(const sim::SimHal& hal) {
  std::vector<int> v;
  for (auto& e : hal.hz_log) if (is_prompt_hz(e.second)) v.push_back(e.second & 0xFF);
  return v;
}
static uint32_t said_at(const sim::SimHal& hal, int id, int nth = 0) {   // ms of the nth time it said id (0 = never)
  for (auto& e : hal.hz_log) if (is_prompt_hz(e.second) && (e.second & 0xFF) == id && nth-- == 0) return e.first;
  return 0;
}
// a head that never keeps still: +-10 deg roll every half second (40 deg/s), for secs seconds
static std::string wobble(double secs) {
  std::string s;
  for (int i = 0; i < int(secs * 2); i++) s += i % 2 ? "seg 0.5 roll -10 ramp\n" : "seg 0.5 roll 10 ramp\n";
  return s;
}

static void test_button() {
  printf("button: double press = the spoken zero (with a buzz), holding does nothing until 5 s = switching off\n");
  auto has = [](const sim::SimHal& h, const char* e) { return std::find(h.events.begin(), h.events.end(), e) != h.events.end(); };
  // the "taken" buzz: full strength from about t, off again ~ACK_MS later
  auto acked = [](const sim::SimHal& h, uint32_t t) {
    for (size_t i = 0; i + 1 < h.motor_log.size(); i++)
      if (h.motor_log[i].second == 100 && h.motor_log[i].first + 20 >= t && h.motor_log[i].first <= t + 50)
        for (size_t j = i + 1; j < h.motor_log.size(); j++)
          if (h.motor_log[j].second == 0) return h.motor_log[j].first - h.motor_log[i].first >= Device::ACK_MS - 20 &&
                                                h.motor_log[j].first - h.motor_log[i].first <= Device::ACK_MS + 30;
    return false;
  };
  auto press = [](Rig& g, double s) { g.dev->button(true); g.run(s); g.dev->button(false); };
  {   // 1 s: nothing (too long for a double press; the screen is the watch's own)
    Rig g("watch\nseg 60 pitch 0\n");
    g.run(10); press(g, 1); g.run(10);
    CHECK(!g.dev->zeroing() && said(g.hal).empty());
  }
  {   // one short press: nothing
    Rig g("watch\nseg 60 pitch 0\n");
    g.run(10); press(g, 0.15); g.run(10);
    CHECK(!g.dev->zeroing() && said(g.hal).empty() && !has(g.hal, "button: double press"));
  }
  {   // double press: buzz at the second release, the spoken zero, zero set
    Rig g("watch\nseg 60 pitch 0\n");
    g.run(10); press(g, 0.15); g.run(0.3); press(g, 0.15);
    uint32_t t = g.hal.now;
    g.run(0.5);
    CHECK(g.dev->zeroing() && has(g.hal, "button: double press") && has(g.hal, "zero: started"));
    CHECK(acked(g.hal, t));
    g.run(20);
    CHECK(g.dev->params().zero_set == 1 && !g.dev->zeroing());
  }
  {   // two short presses too far apart (1.2 s between them): nothing
    Rig g("watch\nseg 60 pitch 0\n");
    g.run(10); press(g, 0.15); g.run(1.2); press(g, 0.15); g.run(10);
    CHECK(!g.dev->zeroing() && said(g.hal).empty());
  }
  {   // a long first press (0.8 s) and a short one: not a double press
    Rig g("watch\nseg 60 pitch 0\n");
    g.run(10); press(g, 0.8); g.run(0.3); press(g, 0.15); g.run(10);
    CHECK(!g.dev->zeroing() && said(g.hal).empty());
  }
  {   // three quick presses: the zero starts once (the third press is a new first press, nothing more)
    Rig g("watch\nseg 60 pitch 0\n");
    g.run(10); press(g, 0.15); g.run(0.3); press(g, 0.15); g.run(0.3); press(g, 0.15); g.run(1);
    CHECK(g.dev->zeroing() && !has(g.hal, "zero: cancelled"));
  }
  {   // held 3 s (the old long press) and let go: nothing at all, no buzz, no zero
    Rig g("watch\nseg 60 pitch 0\n");
    g.run(10); press(g, 3); g.run(10);
    CHECK(!g.dev->zeroing() && said(g.hal).empty() && !has(g.hal, "zero: started"));
    bool buzz = false;
    for (auto& m : g.hal.motor_log) buzz |= m.second != 0;
    CHECK(!buzz);
  }
  {   // the same again while it talks cancels it: a double press during the countdown
    Rig g("watch\nseg 60 pitch 0\n");
    g.run(10); press(g, 0.15); g.run(0.3); press(g, 0.15); g.run(1.5);
    CHECK(g.dev->zeroing());
    press(g, 0.15); g.run(0.3); press(g, 0.15); g.run(4);
    CHECK(has(g.hal, "zero: cancelled") && !g.dev->zeroing());
    g.run(20);
    CHECK(g.dev->params().zero_set == 0);
  }
  {   // held 7 s: nothing until "Switching off." at 5 s; let go before 10 s: it stays on
    Rig g("watch\nseg 60 pitch 0\n");
    g.run(10); g.dev->button(true); g.run(4.8);
    CHECK(!g.dev->zeroing() && said(g.hal).empty());
    g.run(0.5);
    std::vector<int> s = said(g.hal);
    CHECK(s.size() == 1 && s.back() == PR_OFF);
    CHECK(has(g.hal, "switching off") && !g.dev->zeroing());
    g.run(1.7); g.dev->button(false); g.run(20);
    CHECK(has(g.hal, "switch off cancelled"));
    CHECK(g.dev->params().zero_set == 0);
  }
  {   // a zero still talking, then the button held into "Switching off.": the zero is stopped, no zero set
    Rig g("watch\nseg 60 pitch 0\n");
    g.run(10); press(g, 0.15); g.run(0.3); press(g, 0.15); g.run(0.5);
    CHECK(g.dev->zeroing());
    g.dev->button(true); g.run(5.3);
    CHECK(has(g.hal, "zero: stopped (switching off)") && !g.dev->zeroing());
    g.run(1); g.dev->button(false); g.run(20);
    CHECK(has(g.hal, "switch off cancelled") && g.dev->params().zero_set == 0);
  }
  {   // a release with no press (an edge the watch saw late): nothing
    Rig g("watch\nseg 60 pitch 0\n");
    g.run(10); g.dev->button(false); g.run(5);
    CHECK(!g.dev->zeroing() && said(g.hal).empty());
  }
}

static void test_zero_guide() {
  printf("spoken zero\n");
  // 1. crooked tape, lying flat and still: start, 3, 2, 1, (silence while measuring), zero set; level after it
  {
    Rig g("watch\nmount 12 8 0\nseg 20 pitch 0\ntap\nseg 40 pitch 0\n");
    bool step7 = false;
    for (int i = 0; i < 70; i++) { g.run(0.5); step7 |= nudge_step(g.dev->last().nudge) == STEP_ZERO; }
    std::vector<int> want = {PR_START, PR_THREE, PR_TWO, PR_ONE, PR_DONE};
    CHECK(said(g.hal) == want);
    CHECK(g.dev->params().zero_set == 1);
    CHECK_NEAR(g.dev->last().roll / 100.0, 0, 1.5);
    CHECK_NEAR(g.dev->last().pitch / 100.0, 0, 1.5);
    CHECK(step7);                                    // its own voice is marked in the records (not the bell)
    CHECK(!g.dev->zeroing());
    // nothing at all sounds between "one" ending and "zero set": the measuring is silent
    uint32_t one = said_at(g.hal, PR_ONE), done = said_at(g.hal, PR_DONE);
    CHECK(one && done && done - one >= 3000 + 250);
    int between = 0;
    for (auto& e : g.hal.hz_log) between += e.first > one && e.first < done && e.second != 0;
    CHECK(between == 0);
    // a phrase stays on past its length: the audio can start late (the screen waking) and must still get to its end
    auto held = [&](int id) {
      for (size_t i = 0; i + 1 < g.hal.hz_log.size(); i++)
        if (is_prompt_hz(g.hal.hz_log[i].second) && (g.hal.hz_log[i].second & 0xFF) == id)
          return g.hal.hz_log[i + 1].first - g.hal.hz_log[i].first;
      return uint32_t(0);
    };
    CHECK(held(PR_START) >= g.hal.prompt_len[PR_START] + ZeroGuide::SPARE_MS);
    CHECK(held(PR_DONE) >= g.hal.prompt_len[PR_DONE] + ZeroGuide::SPARE_MS);
    for (int id : {PR_THREE, PR_TWO, PR_ONE}) CHECK(held(id) + 50 >= ZeroGuide::COUNT_STEP_MS);   // a whole second
    CHECK(std::find(g.hal.events.begin(), g.hal.events.end(), "zero: started") != g.hal.events.end());
  }
  // 2. the 30 Oct bedtime: the head moves for 2 s after the press (arm down, settling into the pillow), then still
  {
    Rig g("watch\nseg 20 pitch 0\ntap\nseg 0.5 roll 8 ramp\nseg 0.5 roll -6 ramp\nseg 0.5 roll 5 ramp\n"
          "seg 0.5 roll 0 ramp\nseg 30 roll 0\n");
    g.run(40);
    CHECK(g.dev->params().zero_set == 1);
    CHECK(said(g.hal).back() == PR_DONE);
  }
  // 3. never still: "you moved" twice (3 tries in all), then "zero not set"; nothing changes
  {
    Rig g("watch\nseg 20 pitch 0\ntap\n" + wobble(70));
    g.run(90);
    std::vector<int> s = said(g.hal);
    CHECK(std::count(s.begin(), s.end(), int(PR_MOVED)) == 2);
    CHECK(!s.empty() && s.back() == PR_FAILED);
    CHECK(std::count(s.begin(), s.end(), int(PR_THREE)) == 3);
    CHECK(g.dev->params().zero_set == 0);
    CHECK(!g.dev->zeroing());
    CHECK(std::find(g.hal.events.begin(), g.hal.events.end(), "zero: not set (moved)") != g.hal.events.end());
  }
  // 4. no limit to the old zero: after a desk zero 50 deg off (31 Oct), lying flat sets it straight away
  {
    Rig g("watch\nseg 20 pitch 0 roll 50\ntap\nseg 20 roll 50\nseg 3 roll 0 ramp\nseg 5 roll 0\ntap\nseg 20 roll 0\n");
    g.run(70);
    CHECK(said(g.hal).back() == PR_DONE);
    CHECK_NEAR(g.dev->last().roll / 100.0, 0, 2);
    CHECK(std::count(g.hal.events.begin(), g.hal.events.end(), std::string("zero set by tap")) == 2);
  }
  // 5. a second double press while it counts: "cancelled", no zero
  {
    Rig g("watch\nseg 20 pitch 0\ntap\nseg 4 pitch 0\ntap\nseg 30 pitch 0\n");
    g.run(50);
    CHECK(said(g.hal).back() == PR_CANCEL);
    CHECK(g.dev->params().zero_set == 0);
    CHECK(!g.dev->zeroing());
  }
  // 6. therapy, flat on the back after "zero set": no nudge for 5 min, then the nudges as normal (30 s flat)
  {
    Rig g("watch\nseg 200 roll 80 pitch 0\nseg 2 roll 0 ramp\ntap\nseg 600 roll 0\n");
    g.cmd("set win_from 0"); g.cmd("set win_to 0");
    g.cmd("mode therapy"); g.cmd("set start_wait 0"); g.cmd("set delay 30");
    g.run(202 + 60);
    uint32_t done = said_at(g.hal, PR_DONE);
    CHECK(done != 0);
    g.hal.motor_log.clear();
    g.run(500);
    uint32_t first = 0;
    for (auto& m : g.hal.motor_log) if (m.second > 0) { first = m.first; break; }
    CHECK(first != 0);
    CHECK(first >= done + 300000);                  // quiet for the 5 min
    CHECK(first <= done + 300000 + 40000);          // then flat 30 s (+ jitter) -> the first step
  }
  // 7. no "first 15 min only" any more: a press 20 min into the night still works
  {
    Rig g("watch\nseg 1200 roll 80 pitch 0\nseg 3 roll 0 ramp\ntap\nseg 30 roll 0\n");
    g.cmd("set win_from 0"); g.cmd("set win_to 0");
    g.run(1240);
    CHECK(g.dev->session());
    CHECK(g.dev->params().zero_set == 1);
  }
}

static void test_reboot_and_power_loss() {
  printf("reboot, power loss\n");
  Rig g("seg 3000 pitch 0\n");
  g.cmd("set win_from 0"); g.cmd("set win_to 0");
  g.cmd("set angle 33");
  g.run(1000);
  uint32_t boot1 = g.dev->log().boot_id(), seq1 = g.dev->seq();
  int used1 = g.dev->log().used();
  g.boot();   // power lost: no close
  CHECK(g.dev->log().boot_id() == boot1 + 1);
  CHECK(g.dev->params().angle_deg == 33);   // settings survive
  CHECK(g.dev->log().used() == used1);
  // the old boot's records are all there except at most the last 16 (8 s) that were still in RAM
  uint32_t have = 0;
  for (int s = 0; s < g.dev->log().sectors(); s++)
    if (g.dev->log().entry(s).valid && g.dev->log().entry(s).boot_id == boot1) have += g.dev->log().entry(s).n;
  CHECK(have + 16 >= seq1 && have <= seq1);
  CHECK(g.flash.violations == 0 && g.flash.max_programs <= 3);
  g.run(300);
  CHECK(g.dev->session());   // logging again after the reboot
  // erase keeps counters going up
  g.cmd("erase yes");
  CHECK(g.dev->log().used() == 0);
  uint32_t nb = g.dev->log().next_block_no();
  g.boot();
  CHECK(g.dev->log().boot_id() == boot1 + 2 && g.dev->log().next_block_no() >= nb);
}

static void test_night_records() {
  printf("night: angles, buzz, snore in the records\n");
  Rig g(NIGHT);
  g.cmd("set win_from 0"); g.cmd("set win_to 0");
  g.cmd("live on");
  g.out[PORT_USB].clear();
  g.run(g.sc.end);
  std::vector<Record> rec;
  uint32_t b, s;
  Record r;
  for (auto& l : g.out[PORT_USB]) if (parse_R(l, &b, &s, &r)) rec.push_back(r);
  CHECK(rec.size() > 4400);
  auto at = [&](double t) { size_t i = size_t(t * 2); return i < rec.size() ? rec[i] : Record{}; };
  CHECK(at(20).flags & F_UPRIGHT);
  CHECK_NEAR(at(600).roll / 100.0, 3, 1);
  CHECK_NEAR(at(1500).roll / 100.0, 80, 1.5);
  CHECK_NEAR(at(2000).roll / 100.0, -5, 1);
  // the bell (40 deg, 20 s) rings on the back from 55 s: big buzz there, none on the side
  double quiet = 0, ring = 0;
  for (double t = 1300; t < 1800; t += 0.5) quiet = std::max(quiet, at(t).buzz / 10.0);
  for (double t = 60; t < 1200; t += 0.5) ring = std::max(ring, at(t).buzz / 10.0);
  CHECK(quiet < 4);
  CHECK(ring > 15);
  // snoring on the side shows in the snore band
  double sn = 0, sn_quiet = 0;
  for (double t = 1300; t < 1800; t += 0.5) sn = std::max(sn, double(at(t).snore));
  for (double t = 2000; t < 2300; t += 0.5) sn_quiet = std::max(sn_quiet, double(at(t).snore));
  CHECK(sn > 5 * sn_quiet);
  // truth: flat once the pitch drops below 60 deg (31.25 s, ramping 80 -> 0 over 30-35 s) + 20 s
  CHECK(!g.sc.bell.rings.empty() && fabs(g.sc.bell.rings[0].first - 51.5) < 1);
}

static void test_bedtime_cue() {
  printf("bedtime buzz\n");
  for (int low = 0; low < 2; low++) {   // on duty: two 0.4 s pulses; battery low: three long ones, all full strength
    Rig g(low ? "batt 3700 3700\nseg 400 roll 80 pitch 0\n" : "seg 400 roll 80 pitch 0\n");
    g.run(50);
    CHECK(!g.dev->session() && g.hal.motor_log.empty());
    g.run(20);
    CHECK(g.dev->session());   // 1 min lying on the side: the session starts, and the buzz with it
    std::vector<uint8_t> on;
    for (auto& e : g.hal.motor_log) if (e.second) on.push_back(e.second);
    CHECK(on.size() == (low ? 3u : 2u));
    CHECK(!on.empty() && on[0] == 100);
    CHECK(!g.hal.motor_log.empty() && g.hal.motor_log.back().second == 0);   // and it ends
    g.cmd("live on");
    g.run(30);
    CHECK(g.hal.motor_log.size() == (low ? 6u : 4u));   // once a session, in shadow mode too
    // its windows are marked step 6 (variant 0 on duty, 1 low), so the analysis never takes it for the bell
    int cue = 0;
    for (int s = 0; s < g.dev->log().sectors(); s++) {
      const RingLog::Entry& e = g.dev->log().entry(s);
      for (int i = 0; e.valid && i < e.n; i++) {
        Record r;
        g.dev->log().read_record(s, i, &r);
        if (nudge_step(r.nudge) == STEP_CUE) { cue++; CHECK(nudge_variant(r.nudge) == low); }
      }
    }
    CHECK(cue >= 1 && cue <= (low ? 6 : 3));
  }
  {   // the watch's clock moves while the session starts (the backfill writes the flash): the buzz still plays in
      // full (31 Oct: it was skipped every time, now - cue_ms_ went "negative")
    Rig g("seg 400 roll 80 pitch 0\n");
    g.hal.ms_step = 1;
    g.run(70);
    CHECK(g.dev->session());
    std::vector<uint8_t> on;
    for (auto& e : g.hal.motor_log) if (e.second) on.push_back(e.second);
    CHECK(on.size() == 2u && on[0] == 100);
  }
  {   // back from a hang in the night: no buzz, it carries on quietly
    Rig g("seg 400 roll 80 pitch 0\n");
    g.run(130);
    size_t n = g.hal.motor_log.size();
    g.hal.resumed_ = true;
    g.boot();
    g.run(5);
    CHECK(g.dev->session() && g.hal.motor_log.size() == n);
  }
}

int main() {
  test_format();
  test_angles();
  test_sensing();
  test_therapy();
  test_ringlog();
  test_device_protocol();
  test_live_and_catchup();
  test_session_gate();
  test_bedtime_cue();
  test_shadow_and_therapy();
  test_tap_zero();
  test_zero_guide();
  test_button();
  test_watch();
  test_reboot_and_power_loss();
  test_night_records();
  printf("%d checks, %d failed\n", checks, fails);
  return fails ? 1 : 0;
}
