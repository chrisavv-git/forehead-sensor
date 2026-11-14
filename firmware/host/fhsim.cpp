// fhsim: the firmware core on a simulated head, talking over pseudo-terminals, so the laptop tool can be tested
// without the chip. The USB pty is trusted; the Bluetooth pty needs the key, as the real link does.
//
//   fhsim --scenario night.txt [--flash f.bin] [--state s.bin] [--usb LINK] [--ble LINK] [--speed X]
//         [--batch] [--clock TZ_MIN] [--kill-at S] [--truth FILE] [--quiet]
//
// --speed X     virtual seconds per wall second (default 1; 0 = as fast as possible)
// --batch       run the scenario to its end, then exit (with --usb/--ble the ports stay open until then)
// --clock TZ    set the device clock at start from the scenario's t0 (a laptop sync before the night)
// --kill-at S   stop at virtual second S as if the battery died (no clean close; the flash is saved as it is)
// --truth FILE  write the simulated bell's real rings ("ring <start s> <end s>") at the end
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <string>
#include "simbody.h"

using namespace fhs;

static volatile sig_atomic_t stop_flag = 0;
static void on_signal(int) { stop_flag = 1; }

struct Pty {
  int master = -1, slave = -1;
  std::string path, link;
  bool open_pty(const std::string& lnk) {
    master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || grantpt(master) || unlockpt(master)) return false;
    path = ptsname(master);
    slave = open(path.c_str(), O_RDWR | O_NOCTTY);   // held open, so the master never sees EIO between clients
    termios t;
    tcgetattr(slave, &t);
    cfmakeraw(&t);
    tcsetattr(slave, TCSANOW, &t);
    fcntl(master, F_SETFL, O_NONBLOCK);
    link = lnk;
    if (!link.empty()) { unlink(link.c_str()); if (symlink(path.c_str(), link.c_str())) return false; }
    return true;
  }
  void close_pty() {
    if (!link.empty()) unlink(link.c_str());
    if (slave >= 0) close(slave);
    if (master >= 0) close(master);
  }
};

static bool load_bin(const std::string& f, std::vector<uint8_t>& v) {
  std::ifstream in(f, std::ios::binary);
  if (!in) return false;
  std::vector<uint8_t> d((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (d.size() != v.size()) return false;
  v = d;
  return true;
}

static void save_bin(const std::string& f, const void* p, size_t n) {
  std::string tmp = f + ".tmp";
  std::ofstream out(tmp, std::ios::binary);
  out.write(static_cast<const char*>(p), std::streamsize(n));
  out.close();
  rename(tmp.c_str(), f.c_str());
}

struct Saved {   // what the chip keeps in its internal flash
  uint32_t magic = 0x53494D31;
  bool have_params = false, have_state = false;
  Params params;
  State state;
};

int main(int argc, char** argv) {
  std::string scen, flash_file, state_file, usb_link, ble_link, truth_file;
  double speed = 1, kill_at = -1;
  bool batch = false, quiet = false;
  int clock_tz = 9999;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() -> std::string { if (i + 1 >= argc) { fprintf(stderr, "%s needs a value\n", a.c_str()); exit(2); } return argv[++i]; };
    if (a == "--scenario") scen = next();
    else if (a == "--flash") flash_file = next();
    else if (a == "--state") state_file = next();
    else if (a == "--usb") usb_link = next();
    else if (a == "--ble") ble_link = next();
    else if (a == "--speed") speed = atof(next().c_str());
    else if (a == "--batch") batch = true;
    else if (a == "--clock") clock_tz = atoi(next().c_str());
    else if (a == "--kill-at") kill_at = atof(next().c_str());
    else if (a == "--truth") truth_file = next();
    else if (a == "--quiet") quiet = true;
    else { fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
  }
  if (scen.empty()) { fprintf(stderr, "usage: fhsim --scenario FILE [options]\n"); return 2; }
  std::ifstream sf(scen);
  if (!sf) { fprintf(stderr, "can't read %s\n", scen.c_str()); return 2; }
  std::stringstream ss;
  ss << sf.rdbuf();
  sim::Scenario sc;
  if (!sc.parse(ss.str())) { fprintf(stderr, "%s: %s\n", scen.c_str(), sc.error.c_str()); return 2; }

  static sim::MemFlash flash;
  if (!flash_file.empty() && load_bin(flash_file, flash.mem) && !quiet) fprintf(stderr, "fhsim: flash from %s\n", flash_file.c_str());
  sim::SimHal hal;
  Saved saved;
  if (!state_file.empty()) {
    std::vector<uint8_t> v(sizeof(Saved));
    if (load_bin(state_file, v)) memcpy(&saved, v.data(), sizeof saved);
    if (saved.magic != 0x53494D31) saved = Saved();
    hal.have_params = saved.have_params; hal.saved_params = saved.params;
    hal.have_state = saved.have_state; hal.saved_state = saved.state;
  }
  hal.rnd ^= uint32_t(time(nullptr));
  hal.imu_rate = float(sc.imu_hz);
  hal.acc_lsb = float(sc.imu_lsb);
  hal.gyro = sc.imu_gyro;
  static Device dev;
  dev.begin(&hal, &flash);
  sim::Body body(sc, uint64_t(time(nullptr)));
  Pty usb, ble;
  dev.link(PORT_USB, true);   // with no pty (--batch), what the device says over USB is thrown away
  if (!usb_link.empty() || !batch) {
    if (!usb.open_pty(usb_link)) { perror("pty"); return 1; }
    fprintf(stderr, "fhsim: usb on %s%s\n", usb.path.c_str(), usb_link.empty() ? "" : (" (" + usb_link + ")").c_str());
  }
  if (!ble_link.empty()) {
    if (!ble.open_pty(ble_link)) { perror("pty"); return 1; }
    fprintf(stderr, "fhsim: ble on %s (%s)\n", ble.path.c_str(), ble_link.c_str());
  }
  if (clock_tz != 9999 && sc.t0_ms) {
    std::string c = "time " + std::to_string(sc.t0_ms) + " " + std::to_string(clock_tz) + "\n";
    dev.rx(PORT_USB, c.data(), c.size());
  }
  if (!quiet) fprintf(stderr, "fhsim: boot %u, %d blocks in the log, scenario %.0f s\n", dev.log().boot_id(), dev.log().used(), sc.end);
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGPIPE, SIG_IGN);

  timespec w0;
  clock_gettime(CLOCK_MONOTONIC, &w0);
  bool ble_up = false, killed = false;
  uint64_t steps = 0;
  while (!stop_flag) {
    double t = hal.now / 1000.0;
    if (batch && t >= sc.end) break;
    if (kill_at >= 0 && t >= kill_at) { killed = true; break; }
    hal.now += 10;
    t = hal.now / 1000.0;
    hal.usb = sc.in(sc.usb_on, t);
    hal.batt = uint16_t(sc.batt_from + (sc.batt_to - sc.batt_from) * (sc.end > 0 ? std::min(1.0, t / sc.end) : 0));
    bool b = ble.master >= 0 && !sc.in(sc.ble_down, t);
    if (b != ble_up) { dev.link(PORT_BLE, b); ble_up = b; }
    body.run_to(dev, hal.now, hal.motor_level);
    for (Pty* p : {&usb, &ble}) {
      if (p->master < 0) continue;
      int port = p == &usb ? PORT_USB : PORT_BLE;
      char buf[512];
      ssize_t n;
      while ((n = read(p->master, buf, sizeof buf)) > 0)
        if (port == PORT_USB || ble_up) dev.rx(port, buf, size_t(n));   // a dropped link loses what is sent
    }
    dev.poll();
    if (usb.master < 0) dev.tx(PORT_USB).clear();
    for (Pty* p : {&usb, &ble}) {
      if (p->master < 0) continue;
      TxRing& tx = dev.tx(p == &usb ? PORT_USB : PORT_BLE);
      const char* d;
      size_t n;
      while ((n = tx.peek(&d)) > 0) {
        ssize_t w = write(p->master, d, n);
        if (w <= 0) break;
        tx.drop(size_t(w));
      }
    }
    if (speed > 0 && (++steps % 4) == 0) {   // pace: virtual = speed x wall
      timespec w;
      clock_gettime(CLOCK_MONOTONIC, &w);
      double wall = (w.tv_sec - w0.tv_sec) + (w.tv_nsec - w0.tv_nsec) / 1e9, ahead = t / speed - wall;
      if (ahead > 0.0005) usleep(useconds_t(ahead * 1e6));
    }
  }
  if (!killed) dev.log().close();   // a clean stop closes the open block; a killed one leaves it as power loss would
  if (!flash_file.empty()) save_bin(flash_file, flash.mem.data(), flash.mem.size());
  if (!state_file.empty()) {
    saved.have_params = hal.have_params; saved.params = hal.saved_params;
    saved.have_state = hal.have_state; saved.state = hal.saved_state;
    save_bin(state_file, &saved, sizeof saved);
  }
  if (!truth_file.empty()) {
    std::ofstream tf(truth_file);
    if (sc.bell.ringing) sc.bell.rings.push_back({sc.bell.ring_start, hal.now / 1000.0});
    for (auto& r : sc.bell.rings) tf << "ring " << r.first << " " << r.second << "\n";
  }
  if (!quiet) fprintf(stderr, "fhsim: stopped at %.1f s%s, %d blocks, flash program max %d/page, %d page violations\n",
                      hal.now / 1000.0, killed ? " (killed)" : "", dev.log().used(), flash.max_programs, flash.violations);
  usb.close_pty();
  ble.close_pty();
  return 0;
}
