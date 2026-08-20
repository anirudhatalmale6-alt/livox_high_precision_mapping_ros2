// Port auto-detection: does the probe actually recognise a UM982, and does it
// refuse everything else?
//
// Uses a real pty so SerialPort's termios setup runs for real - a plain file
// would pass open() and then fail tcgetattr, which is not the case under test.
#include <fcntl.h>
#include <glob.h>
#include <limits.h>
#include <pty.h>
#include <stdlib.h>
#include <unistd.h>

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "um982_driver/serial_port.hpp"

// ---- the code under test (kept identical to um982_driver_node.cpp) --------
static std::string realPath(const std::string & p)
{
  char buf[PATH_MAX];
  const char * r = ::realpath(p.c_str(), buf);
  return r ? std::string(r) : p;
}

static bool speaksGnss(const std::string & dev, int baud)
{
  um982::SerialPort probe;
  try
  {
    probe.open(dev, baud);
  }
  catch (const std::exception &)
  {
    return false;
  }
  for (int i = 0; i < 6; ++i)
  {
    std::string line;
    if (!probe.readLine(line) || line.empty())
    {
      continue;
    }
    if (line[0] == '#')
    {
      return true;
    }
    if (line[0] == '$' &&
        (line.find("GGA") != std::string::npos ||
         line.find("RMC") != std::string::npos ||
         line.find("GSV") != std::string::npos))
    {
      return true;
    }
  }
  return false;
}

// Filtering half of candidatePorts(), with the glob results injected so the
// test does not depend on what is plugged into the machine running it.
static std::vector<std::string> filterCandidates(
  const std::vector<std::string> & found,
  const std::vector<std::string> & configured_others)
{
  std::vector<std::string> blocked;
  for (const std::string & p : configured_others)
  {
    if (!p.empty()) { blocked.push_back(realPath(p)); }
  }
  std::vector<std::string> keep;
  for (const std::string & c : found)
  {
    const std::string rc = realPath(c);
    bool skip = false;
    for (const std::string & b : blocked)
    {
      if (rc == b) { skip = true; break; }
    }
    if (!skip) { keep.push_back(c); }
  }
  return keep;
}

// ---- harness -------------------------------------------------------------
static int failures = 0;
static void check(bool cond, const char * what)
{
  std::printf("  [%s] %s\n", cond ? "ok" : "FAIL", what);
  if (!cond) { ++failures; }
}

// Open a pty and STREAM `payload` into the master end until told to stop.
//
// It has to keep repeating rather than write once: SerialPort::open() ends with
// tcflush(TCIFLUSH), which throws away anything already sitting in the buffer.
// A real receiver streams continuously so the probe always sees the next
// sentence; a single pre-loaded write would be flushed away and the test would
// be measuring the flush, not the detection.
struct FakePort
{
  int master_fd = -1;
  std::string device;
  std::atomic<bool> stop{false};
  std::thread writer;

  explicit FakePort(const std::string & payload)
  {
    int slave_fd = -1;
    char name[PATH_MAX];
    if (openpty(&master_fd, &slave_fd, name, nullptr, nullptr) != 0)
    {
      std::perror("openpty");
      std::exit(2);
    }
    ::close(slave_fd);            // the probe reopens it by name
    device = name;
    if (payload.empty())
    {
      return;                     // a silent adapter
    }
    writer = std::thread([this, payload]() {
      while (!stop.load())
      {
        ssize_t n = ::write(master_fd, payload.data(), payload.size());
        (void)n;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
  }

  ~FakePort()
  {
    stop.store(true);
    if (writer.joinable()) { writer.join(); }
    if (master_fd >= 0) { ::close(master_fd); }
  }
};

int main()
{
  std::printf("port auto-detection\n");

  // A: a receiver sending GGA is recognised.
  {
    FakePort p("$GNGGA,123519.00,4807.038,N,01131.000,E,4,15,0.9,545.4,M,46.9,M,,*4F\r\n");
    check(speaksGnss(p.device, 115200), "GGA is recognised as a GNSS receiver");
  }

  // B: RMC alone is enough (a receiver with position output not yet enabled).
  {
    FakePort p("$GNRMC,123519.00,A,4807.038,N,01131.000,E,0.0,0.0,230394,,,A*6A\r\n");
    check(speaksGnss(p.device, 115200), "RMC alone is enough");
  }

  // C: a Unicore '#' response counts - that is what the receiver replies with
  // when it is alive but streaming nothing yet.
  {
    FakePort p("#UNIHEADINGA,COM1,0,60.0,FINE,2190\r\n");
    check(speaksGnss(p.device, 115200), "a Unicore # response counts");
  }

  // D: a serial device that is NOT the GNSS must be rejected, or the driver
  // would latch onto the IMU and report "connected" while publishing nothing.
  {
    FakePort p("\xa5\x5a\x11\x22 binary imu frame \r\n");
    check(!speaksGnss(p.device, 115200), "a non-GNSS serial device is rejected");
  }

  // E: silence is rejected (an adapter with nothing wired to it).
  {
    FakePort p("");
    check(!speaksGnss(p.device, 115200), "a silent port is rejected");
  }

  // F: a device that does not exist is rejected rather than throwing out.
  check(!speaksGnss("/dev/does-not-exist-9999", 115200),
        "a missing device is rejected, not fatal");

  // G: ports belonging to other devices are excluded by RESOLVED path - the
  // whole point, since /dev/rtcm and /dev/ttyUSB1 can be the same device.
  {
    auto keep = filterCandidates({"/dev/ttyUSB0", "/dev/ttyUSB1"},
                                 {"/dev/ttyUSB1"});
    check(keep.size() == 1 && keep[0] == "/dev/ttyUSB0",
          "a port configured for another device is excluded");
  }

  // H: an unset/empty other-port must not block anything.
  {
    auto keep = filterCandidates({"/dev/ttyUSB0"}, {""});
    check(keep.size() == 1, "an empty other-port setting blocks nothing");
  }

  // I: a symlink and its target are recognised as the same device.
  {
    char tmpl[] = "/tmp/um982_test_XXXXXX";
    char * dir = mkdtemp(tmpl);
    assert(dir != nullptr);
    std::string target = std::string(dir) + "/ttyUSB7";
    std::string link = std::string(dir) + "/rtcm";
    ::close(::open(target.c_str(), O_CREAT | O_WRONLY, 0600));
    ::symlink(target.c_str(), link.c_str());
    auto keep = filterCandidates({target}, {link});
    check(keep.empty(), "a symlink to a port excludes the port itself");
    ::unlink(link.c_str());
    ::unlink(target.c_str());
    ::rmdir(dir);
  }

  std::printf("%s (%d failure(s))\n", failures ? "FAILED" : "all passed", failures);
  return failures ? 1 : 0;
}
