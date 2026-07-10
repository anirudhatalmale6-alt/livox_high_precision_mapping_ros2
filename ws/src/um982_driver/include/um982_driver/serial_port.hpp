// Minimal, dependency-free blocking serial port reader for Linux (termios).
//
// The original ROS1 project vendored William Woodall's cross-platform "serial"
// library. On Ubuntu 22.04 / ROS2 we only ever run on Linux, so a small termios
// wrapper keeps the dependency surface at zero and builds cleanly with colcon.
#ifndef UM982_DRIVER__SERIAL_PORT_HPP_
#define UM982_DRIVER__SERIAL_PORT_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

namespace um982
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort() { close(); }

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Open the port at the given baud. Throws std::runtime_error on failure.
  // Pass writable=true to also allow write() (RTCM injection into the UM982).
  // The same physical device may be opened twice — once read-only for the NMEA
  // reader and once writable for correction injection — which is fine on Linux
  // since we never call read() on the writable handle.
  void open(const std::string & device, int baud, bool writable = false)
  {
    int flags = (writable ? O_WRONLY : O_RDONLY) | O_NOCTTY;
    fd_ = ::open(device.c_str(), flags);
    if (fd_ < 0)
    {
      throw std::runtime_error("failed to open " + device + ": " + std::strerror(errno));
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0)
    {
      close();
      throw std::runtime_error("tcgetattr failed on " + device);
    }

    speed_t speed = baudConstant(baud);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;             // 8 data bits
    tty.c_cflag &= ~PARENB;        // no parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;       // no hardware flow control

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);   // raw mode
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;           // 0.5s read timeout

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
      close();
      throw std::runtime_error("tcsetattr failed on " + device);
    }
    tcflush(fd_, TCIFLUSH);
    device_ = device;
  }

  bool isOpen() const { return fd_ >= 0; }

  // Write a raw buffer (e.g. an RTCM3 correction stream) to the port. Returns
  // false if the port is closed or the underlying write fails. Loops until the
  // full buffer is written or an error occurs.
  bool writeRaw(const unsigned char * data, size_t len)
  {
    if (fd_ < 0)
    {
      return false;
    }
    size_t off = 0;
    while (off < len)
    {
      ssize_t n = ::write(fd_, data + off, len - off);
      if (n < 0)
      {
        if (errno == EINTR) { continue; }
        return false;
      }
      off += static_cast<size_t>(n);
    }
    return true;
  }

  bool writeStr(const std::string & s)
  {
    return writeRaw(reinterpret_cast<const unsigned char *>(s.data()), s.size());
  }

  // Read up to len raw bytes (for a binary RTCM source such as a radio link).
  // Returns the number of bytes read, 0 on timeout, or -1 on error/closed.
  ssize_t readRaw(unsigned char * buf, size_t len)
  {
    if (fd_ < 0)
    {
      return -1;
    }
    return ::read(fd_, buf, len);
  }

  void close()
  {
    if (fd_ >= 0)
    {
      ::close(fd_);
      fd_ = -1;
    }
  }

  // Read a single '\n'-terminated line (without the newline). Returns false on
  // timeout / EOF with no complete line yet buffered.
  bool readLine(std::string & line)
  {
    char c = 0;
    while (true)
    {
      ssize_t n = ::read(fd_, &c, 1);
      if (n < 0)
      {
        if (errno == EINTR) { continue; }
        return false;
      }
      if (n == 0)
      {
        return false;  // timeout
      }
      if (c == '\n')
      {
        line = buffer_;
        buffer_.clear();
        return true;
      }
      if (c != '\r')
      {
        buffer_.push_back(c);
        if (buffer_.size() > 4096)
        {
          buffer_.clear();  // runaway guard against a stuck line
        }
      }
    }
  }

private:
  static speed_t baudConstant(int baud)
  {
    switch (baud)
    {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      case 921600: return B921600;
      default: return B230400;  // UM982 default
    }
  }

  int fd_ = -1;
  std::string device_;
  std::string buffer_;
};

}  // namespace um982

#endif  // UM982_DRIVER__SERIAL_PORT_HPP_
