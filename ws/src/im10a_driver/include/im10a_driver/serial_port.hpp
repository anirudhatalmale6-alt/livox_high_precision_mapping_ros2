// Minimal, dependency-free blocking serial reader for Linux (termios).
//
// The IM10A speaks a binary protocol, so unlike the UM982's line-based reader
// this one hands back raw bytes. Kept header-only and Linux-only to keep the
// dependency surface at zero (colcon builds it with no extra packages).
#ifndef IM10A_DRIVER__SERIAL_PORT_HPP_
#define IM10A_DRIVER__SERIAL_PORT_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace im10a
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
    tty.c_cflag |= CS8;         // 8 data bits
    tty.c_cflag &= ~PARENB;     // no parity
    tty.c_cflag &= ~CSTOPB;     // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;    // no hardware flow control

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // raw mode
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL);
    tty.c_iflag &= ~(ISTRIP | IGNBRK | BRKINT | PARMRK | INPCK);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;        // 0.1s read timeout

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

  // Read up to `n` bytes into `buf`. Returns the number read (0 on timeout,
  // negative on error).
  ssize_t readBytes(uint8_t * buf, std::size_t n)
  {
    while (true)
    {
      ssize_t r = ::read(fd_, buf, n);
      if (r < 0 && errno == EINTR)
      {
        continue;
      }
      return r;
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
      default: return B115200;  // IM10A default
    }
  }

  int fd_ = -1;
  std::string device_;
};

}  // namespace im10a

#endif  // IM10A_DRIVER__SERIAL_PORT_HPP_
