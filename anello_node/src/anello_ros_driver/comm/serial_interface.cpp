#include "serial_interface.h"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>
namespace {
struct FileDescriptor {
    int value;
    bool exclusive=false;
    ~FileDescriptor() {
        if (value>=0) {
            if (exclusive) ioctl(value,TIOCNXCL);
            ::close(value);
        }
    }
};
}
void serial_interface::init(const std::string &name, uint32_t baud) {
    close_port();
    std::lock_guard<std::mutex> lock(mutex_);
    portname=name;
    if (name=="OFF") return;
    speed_t speed;
    switch (baud) {
    case 115200: speed=B115200; break;
    case 230400: speed=B230400; break;
    case 460800: speed=B460800; break;
    case 921600: speed=B921600; break;
    default: throw std::invalid_argument("Unsupported serial baud rate");
    }
    FileDescriptor fd{::open(name.c_str(),O_RDWR|O_NOCTTY|O_CLOEXEC|O_NONBLOCK)};
    if (fd.value<0) throw std::runtime_error("Open "+name+": "+std::strerror(errno));
    // flock also covers symlink aliases and cooperating processes that opened
    // before TIOCEXCL. Acquire ownership before changing termios or flushing.
    if (flock(fd.value,LOCK_EX|LOCK_NB)!=0) throw std::runtime_error("Serial port already owned: "+name);
    if (ioctl(fd.value,TIOCEXCL)!=0) throw std::runtime_error("Cannot exclusively claim serial port: "+name);
    // Read by the RAII destructor if a later termios operation throws.
    // cppcheck-suppress unreadVariable
    fd.exclusive=true;
    termios options{};
    if (tcgetattr(fd.value,&options)!=0) throw std::runtime_error("tcgetattr: "+name);
    cfmakeraw(&options);
    // cfmakeraw clears IXON but can retain IXOFF/IXANY from a prior user.
    // The line discipline must not inject XON/XOFF into the EVK input stream.
    options.c_iflag &= ~(IXON|IXOFF|IXANY);
    // Do not inherit a hangup-on-close setting that changes modem control
    // lines during the driver's reconnect loop. Never explicitly pulse DTR/RTS.
    options.c_cflag &= ~(PARENB|CSTOPB|CSIZE|CRTSCTS|HUPCL);
    options.c_cflag |= CS8|CLOCAL|CREAD;
    options.c_cc[VMIN]=0; options.c_cc[VTIME]=0;
    cfsetispeed(&options,speed); cfsetospeed(&options,speed);
    if (tcsetattr(fd.value,TCSANOW,&options)!=0) throw std::runtime_error("tcsetattr: "+name);
    tcflush(fd.value,TCIOFLUSH);
    usb_fd=fd.value; fd.value=-1; ++generation_;
}
int serial_interface::duplicate_fd() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return usb_fd<0?-1:fcntl(usb_fd,F_DUPFD_CLOEXEC,0);
}
size_t serial_interface::get_data(char *buf, size_t size, int timeout_ms) {
    if (size<2) return 0;
    const auto opened_generation=generation_.load();
    FileDescriptor fd{duplicate_fd()};
    if (fd.value<0) return 0;
    pollfd pending{fd.value,POLLIN,0};
    const auto result=poll(&pending,1,std::max(0,timeout_ms));
    if (result<=0) return 0;
    const auto count=::read(fd.value,buf,size-1);
    if (count>0 && opened_generation==generation_) { buf[count]=0; return static_cast<size_t>(count); }
    if ((pending.revents&(POLLHUP|POLLERR|POLLNVAL)) ||
        (count<0 && errno!=EINTR && errno!=EAGAIN && errno!=EWOULDBLOCK)) {
        close_generation(opened_generation);
    }
    return 0;
}
bool serial_interface::write_data(const char *buf, size_t size) {
    return write_data(buf,size,generation_.load());
}
bool serial_interface::write_data(const char *buf, size_t size, uint64_t expected_generation) {
    if (size==0) return true;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(100);
    std::unique_lock<std::timed_mutex> writer(write_mutex_,std::defer_lock);
    if (!writer.try_lock_until(deadline) || expected_generation!=generation_) return false;
    FileDescriptor fd{duplicate_fd()};
    if (fd.value<0) return false;
    // Duplicate ownership lets read/reopen continue independently while a
    // transmitter is backpressured. The old fd can never target a new port.
    size_t sent=0;
    while (sent<size && expected_generation==generation_) {
        if (std::chrono::steady_clock::now()>=deadline) return false;
        auto n=::write(fd.value,buf+sent,size-sent);
        if (n>0) { sent+=static_cast<size_t>(n); continue; }
        if (n<0 && errno==EINTR) continue;
        if (n<0 && (errno==EAGAIN || errno==EWOULDBLOCK)) {
            pollfd pending{fd.value,POLLOUT,0};
            auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline-std::chrono::steady_clock::now()).count();
            if (remaining<=0 || poll(&pending,1,static_cast<int>(remaining))==0) return false;
            continue;
        }
        close_generation(expected_generation);
        return false;
    }
    return sent==size && expected_generation==generation_;
}
std::string serial_interface::get_portname() const {
    std::lock_guard<std::mutex> lock(mutex_); return portname;
}
bool serial_interface::get_port_enabled() const {
    std::lock_guard<std::mutex> lock(mutex_); return usb_fd>=0;
}
void serial_interface::close_port() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (usb_fd>=0) { ioctl(usb_fd,TIOCNXCL); ::close(usb_fd); usb_fd=-1; ++generation_; }
}
void serial_interface::close_generation(uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation==generation_ && usb_fd>=0) {
        ioctl(usb_fd,TIOCNXCL); ::close(usb_fd); usb_fd=-1; ++generation_;
    }
}
serial_interface::~serial_interface() { close_port(); }
