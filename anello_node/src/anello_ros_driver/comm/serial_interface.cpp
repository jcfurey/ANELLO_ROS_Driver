/********************************************************************************
 * File Name:   serial_interface.cpp
 * Description: Defines the serial_interface class.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 ********************************************************************************/

#include "../main_anello_ros_driver.h"
#include "../bit_tools.h"

#include <fcntl.h>
#include <termios.h>
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <string>
#include <sys/ioctl.h>
#include <dirent.h>
#include <vector>
#include <stdexcept>

#include "serial_interface.h"

#define MAX_READ_NUM 1000
#define SER_PORT_FLUSH_COUNT 20

serial_interface::serial_interface()
{
    this->portname = "";
    this->usb_fd = -1;
    this->port_enabled = false;
    this->baud_rate_ = 230400;
}

void serial_interface::init(std::string portname, uint32_t baud_rate)
{
    this->baud_rate_ = baud_rate;
    this->portname = portname;

    if (this->portname == "OFF")
    {
        this->port_enabled = false;
        return;
    }

    this->usb_fd = open(this->portname.c_str(), O_RDWR);
    if (this->usb_fd < 0)
    {
        throw std::runtime_error("Failed to open serial port: " + this->portname +
                                 " (" + std::string(strerror(errno)) + ")");
    }

    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr(this->usb_fd, &options) != 0)
    {
        close(this->usb_fd);
        this->usb_fd = -1;
        throw std::runtime_error("tcgetattr failed on " + this->portname +
                                 ": " + std::string(strerror(errno)));
    }

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CLOCAL | CREAD;
    options.c_iflag = 0;
    options.c_lflag = 0;
    options.c_oflag = 0;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 5;     // 0.5 seconds read timeout

    speed_t speed;
    switch (this->baud_rate_) {
      case 115200:   speed = B115200;   break;
      case 230400:   speed = B230400;   break;
      case 460800:   speed = B460800;   break;
      case 921600:   speed = B921600;   break;
      default:
        WARNING_PRINT("Unsupported baud rate %u, falling back to 230400", this->baud_rate_);
        speed = B230400;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    if (tcsetattr(this->usb_fd, TCSANOW, &options) != 0)
    {
        close(this->usb_fd);
        this->usb_fd = -1;
        throw std::runtime_error("tcsetattr failed on " + this->portname +
                                 ": " + std::string(strerror(errno)));
    }

    for (int i = 0; i < SER_PORT_FLUSH_COUNT; i++)
    {
        usleep(1000);
        tcflush(this->usb_fd, TCIOFLUSH);
    }

    this->port_enabled = true;
}

size_t serial_interface::get_data(char *buf, size_t buf_len)
{
    if (!this->port_enabled || this->usb_fd < 0 || buf_len == 0)
    {
        return 0;
    }

    ssize_t bytes_read = read(this->usb_fd, buf, buf_len - 1);
    if (bytes_read < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
        {
            return 0;
        }
        // EIO/ENXIO: the tty lost its device (USB unplug, or the unit
        // power-cycled and re-enumerated). This fd can never produce
        // data again — close so the owner reopens or rescans.
        WARNING_PRINT("Serial port %s read failed (%s) — closing port",
                      this->portname.c_str(), strerror(errno));
        this->close_port();
        return 0;
    }
    buf[bytes_read] = '\0';
    return static_cast<size_t>(bytes_read);
}

size_t serial_interface::get_data(char *buf, size_t buf_len, int timeout)
{
    if (!this->port_enabled || this->usb_fd < 0)
    {
        return 0;
    }

    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(this->usb_fd, &readSet);

    // Split into sec/usec: a timeout >= 1000 ms would otherwise push
    // tv_usec past 1e6, which select() rejects with EINVAL.
    struct timeval tv;
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;

    int ready = select(this->usb_fd + 1, &readSet, nullptr, nullptr, &tv);
    if (ready <= 0)
    {
        return 0;
    }

    size_t bytes_read = serial_interface::get_data(buf, buf_len);
    if (bytes_read == 0 && this->port_enabled)
    {
        // select() reported readable but read() produced nothing: that is
        // the tty hangup signature (device gone), not a timeout — a
        // timeout returns above with ready == 0. The EIO case has already
        // closed the port inside get_data(); this catches the EOF form.
        WARNING_PRINT("Serial port %s hangup — closing port",
                      this->portname.c_str());
        this->close_port();
    }
    return bytes_read;
}

void serial_interface::write_data(const char *buf, size_t buf_len)
{
    if (!this->port_enabled || this->usb_fd < 0)
    {
        return;
    }
    ssize_t written = write(usb_fd, buf, buf_len);
    if (written < 0 || static_cast<size_t>(written) != buf_len)
    {
        WARNING_PRINT("Serial write: expected %zu bytes, wrote %zd", buf_len, written);
    }
}

const std::string serial_interface::get_portname() const
{
    return this->portname;
}

bool serial_interface::get_port_enabled()
{
    return this->port_enabled;
}

void serial_interface::close_port()
{
    if (this->usb_fd > 0)
    {
        tcflush(this->usb_fd, TCIOFLUSH);
        close(this->usb_fd);
        this->usb_fd = -1;
    }
    this->port_enabled = false;
}

serial_interface::~serial_interface()
{
    this->close_port();
}
