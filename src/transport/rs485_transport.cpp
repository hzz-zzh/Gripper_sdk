#include "transport/rs485_transport.h"

#include <cerrno>
#include <cstring>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

namespace gripper
{
namespace
{
bool baudrateToSpeed(int baudrate, speed_t& speed)
{
    switch (baudrate)
    {
        case 9600:   speed = B9600; break;
        case 19200:  speed = B19200; break;
        case 38400:  speed = B38400; break;
        case 57600:  speed = B57600; break;
        case 115200: speed = B115200; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default: return false;
    }
    return true;
}
}

Rs485Transport::Rs485Transport(const std::string& port_name, int baudrate)
    : port_name_(port_name), baudrate_(baudrate), fd_(-1)
{
}

Rs485Transport::~Rs485Transport()
{
    close();
}

bool Rs485Transport::open()
{
    if (fd_ >= 0)
    {
        return true;
    }

    speed_t speed{};
    if (!baudrateToSpeed(baudrate_, speed))
    {
        return false;
    }

    fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0)
    {
        return false;
    }

    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0)
    {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    ::cfmakeraw(&tty);
    ::cfsetispeed(&tty, speed);
    ::cfsetospeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    ::tcflush(fd_, TCIOFLUSH);
    return true;
}

void Rs485Transport::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

bool Rs485Transport::isOpen() const
{
    return fd_ >= 0;
}

int Rs485Transport::writeBytes(const uint8_t* data, std::size_t size)
{
    if (fd_ < 0)
    {
        return -1;
    }

    const ssize_t ret = ::write(fd_, data, size);
    if (ret < 0)
    {
        return -1;
    }

    ::tcdrain(fd_);
    return static_cast<int>(ret);
}

int Rs485Transport::readBytes(uint8_t* data, std::size_t size, int timeout_ms)
{
    if (fd_ < 0)
    {
        return -1;
    }

    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;

    const int poll_ret = ::poll(&pfd, 1, timeout_ms);
    if (poll_ret <= 0)
    {
        return 0;
    }

    const ssize_t ret = ::read(fd_, data, size);
    if (ret < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return 0;
        }
        return -1;
    }

    return static_cast<int>(ret);
}
}