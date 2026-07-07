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
  void open(const std::string & device, int baud)
  {
    fd_ = ::open(device.c_str(), O_RDONLY | O_NOCTTY);
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
