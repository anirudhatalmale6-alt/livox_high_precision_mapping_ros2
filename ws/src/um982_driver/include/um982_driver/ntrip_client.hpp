// Minimal, dependency-free NTRIP v1/v2 client for pulling an RTCM3 correction
// stream from a caster (e.g. a state CORS network like NYSNet, or any NTRIP
// caster fed by your own base station).
//
// It runs its own thread: connects to host:port, requests the mountpoint with
// HTTP Basic auth, then forwards every RTCM3 byte it receives to a user-supplied
// sink (which writes them into the UM982's serial port). For network / VRS
// mountpoints it periodically pushes the rover's latest GGA sentence back to the
// caster so the network can compute corrections for the rover's actual location.
//
// Uses only POSIX sockets + libc — no external dependency, so it builds cleanly
// under colcon on Ubuntu 22.04 with zero extra ament packages.
#ifndef UM982_DRIVER__NTRIP_CLIENT_HPP_
#define UM982_DRIVER__NTRIP_CLIENT_HPP_

#include <netdb.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cerrno>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <utility>

namespace um982
{

// Standard base64 encoder (used for the NTRIP Basic-auth header).
inline std::string base64Encode(const std::string & in)
{
  static const char tbl[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  int val = 0;
  int bits = -6;
  for (unsigned char c : in)
  {
    val = (val << 8) + c;
    bits += 8;
    while (bits >= 0)
    {
      out.push_back(tbl[(val >> bits) & 0x3F]);
      bits -= 6;
    }
  }
  if (bits > -6)
  {
    out.push_back(tbl[((val << 8) >> (bits + 8)) & 0x3F]);
  }
  while (out.size() % 4 != 0)
  {
    out.push_back('=');
  }
  return out;
}

struct NtripConfig
{
  std::string host;
  int port = 2101;
  std::string mountpoint;
  std::string user;
  std::string password;
  bool send_gga = true;   // push rover GGA to the caster (needed by VRS/network mounts)
};

class NtripClient
{
public:
  using ByteSink = std::function<void(const unsigned char *, size_t)>;
  using GgaProvider = std::function<std::string()>;  // latest GGA line, or "" if none yet
  using LogFn = std::function<void(const std::string &)>;

  NtripClient(NtripConfig cfg, ByteSink sink, GgaProvider gga, LogFn info, LogFn warn)
  : cfg_(std::move(cfg)), sink_(std::move(sink)), gga_(std::move(gga)),
    info_(std::move(info)), warn_(std::move(warn)) {}

  ~NtripClient() { stop(); }

  void start()
  {
    running_ = true;
    thread_ = std::thread(&NtripClient::run, this);
  }

  void stop()
  {
    running_ = false;
    if (sock_ >= 0)
    {
      ::shutdown(sock_, SHUT_RDWR);
    }
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

private:
  void run()
  {
    while (running_)
    {
      if (!connectAndRequest())
      {
        closeSock();
        // ~2s backoff before retrying, but stay responsive to stop().
        for (int i = 0; i < 20 && running_; ++i)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        continue;
      }
      info_("NTRIP connected to " + cfg_.host + ":" + std::to_string(cfg_.port) +
            "/" + cfg_.mountpoint + " - streaming RTCM corrections.");
      streamLoop();
      closeSock();
      if (running_)
      {
        warn_("NTRIP stream dropped - reconnecting.");
        // Back off ~1.5s before redialling. Prevents hammering (and getting
        // rate-limited/banned by) a caster that closes early, and stops the
        // console being flooded if a stream genuinely keeps dropping.
        for (int i = 0; i < 15 && running_; ++i)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    }
  }

  bool connectAndRequest()
  {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo * res = nullptr;
    if (getaddrinfo(cfg_.host.c_str(), std::to_string(cfg_.port).c_str(), &hints, &res) != 0)
    {
      warn_("NTRIP: DNS lookup failed for " + cfg_.host);
      return false;
    }
    int s = -1;
    for (addrinfo * p = res; p != nullptr; p = p->ai_next)
    {
      s = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
      if (s < 0)
      {
        continue;
      }
      if (::connect(s, p->ai_addr, p->ai_addrlen) == 0)
      {
        break;
      }
      ::close(s);
      s = -1;
    }
    freeaddrinfo(res);
    if (s < 0)
    {
      warn_("NTRIP: cannot connect to caster " + cfg_.host);
      return false;
    }
    sock_ = s;

    // 1s recv timeout so streamLoop can tick GGA even when idle.
    timeval tv{};
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // NTRIP v1 request (HTTP/1.0). Two things matter here:
    //  * The User-Agent MUST start with "NTRIP" so the caster treats this as a
    //    streaming mount and holds the socket open, rather than a one-shot HTTP
    //    request. (Trimble Pivot / Leica casters key off this.)
    //  * We must NOT send "Connection: close" — HTTP-compliant casters honour it
    //    and hang up right after the response, which showed up as the stream
    //    "connecting" then "dropping" many times a second and never locking RTK.
    // A Host header is included for casters that require it; harmless otherwise.
    std::string req =
      "GET /" + cfg_.mountpoint + " HTTP/1.0\r\n"
      "Host: " + cfg_.host + ":" + std::to_string(cfg_.port) + "\r\n"
      "User-Agent: NTRIP livox_hp_mapping/1.0\r\n"
      "Accept: */*\r\n";
    if (!cfg_.user.empty())
    {
      req += "Authorization: Basic " +
             base64Encode(cfg_.user + ":" + cfg_.password) + "\r\n";
    }
    req += "\r\n";
    if (!sendAll(req))
    {
      warn_("NTRIP: request send failed.");
      return false;
    }

    // Read the response header up to the blank line separating it from the body.
    std::string hdr;
    int idle = 0;
    while (running_ &&
           hdr.find("\r\n\r\n") == std::string::npos &&
           hdr.find("\n\n") == std::string::npos)
    {
      char c = 0;
      ssize_t n = ::recv(sock_, &c, 1, 0);
      if (n == 1)
      {
        hdr.push_back(c);
        idle = 0;
        if (hdr.size() > 4096)
        {
          break;
        }
      }
      else if (n == 0)
      {
        warn_("NTRIP: caster closed connection during handshake.");
        return false;
      }
      else
      {
        if (errno == EINTR)
        {
          continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
          if (++idle > 10)
          {
            warn_("NTRIP: no response from caster (timeout).");
            return false;
          }
          continue;
        }
        warn_("NTRIP: recv error during handshake.");
        return false;
      }
    }
    if (hdr.find("200") == std::string::npos && hdr.find("ICY 200") == std::string::npos)
    {
      warn_("NTRIP: caster rejected request - check mountpoint and credentials.");
      return false;
    }
    // Prime the network with an initial position so a VRS base can be picked.
    pushGga();
    return true;
  }

  void streamLoop()
  {
    unsigned char buf[2048];
    auto last_gga = std::chrono::steady_clock::now();
    while (running_)
    {
      ssize_t n = ::recv(sock_, buf, sizeof(buf), 0);
      if (n > 0)
      {
        sink_(buf, static_cast<size_t>(n));
      }
      else if (n == 0)
      {
        return;  // caster closed the stream
      }
      else
      {
        if (errno == EINTR)
        {
          continue;
        }
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
          return;  // real socket error
        }
        // else: recv timeout — fall through to the periodic GGA push
      }
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_gga).count() >= 10)
      {
        pushGga();
        last_gga = now;
      }
    }
  }

  void pushGga()
  {
    if (!cfg_.send_gga || !gga_)
    {
      return;
    }
    std::string g = gga_();
    if (g.empty())
    {
      return;
    }
    if (g.size() < 2 || g.substr(g.size() - 2) != "\r\n")
    {
      g += "\r\n";
    }
    sendAll(g);
  }

  bool sendAll(const std::string & s)
  {
    size_t off = 0;
    while (off < s.size())
    {
      ssize_t n = ::send(sock_, s.data() + off, s.size() - off, 0);
      if (n < 0)
      {
        if (errno == EINTR)
        {
          continue;
        }
        return false;
      }
      off += static_cast<size_t>(n);
    }
    return true;
  }

  void closeSock()
  {
    if (sock_ >= 0)
    {
      ::close(sock_);
      sock_ = -1;
    }
  }

  NtripConfig cfg_;
  ByteSink sink_;
  GgaProvider gga_;
  LogFn info_;
  LogFn warn_;

  std::atomic<bool> running_{false};
  std::thread thread_;
  int sock_ = -1;
};

}  // namespace um982

#endif  // UM982_DRIVER__NTRIP_CLIENT_HPP_
