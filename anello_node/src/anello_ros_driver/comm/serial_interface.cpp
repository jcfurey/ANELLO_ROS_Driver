#include "serial_interface.h"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
namespace {
struct FileDescriptor {
    int value;
    ~FileDescriptor() { if (value>=0) ::close(value); }
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
    termios options{};
    if (tcgetattr(fd.value,&options)!=0) throw std::runtime_error("tcgetattr: "+name);
    cfmakeraw(&options);
    options.c_cflag &= ~(PARENB|CSTOPB|CSIZE|CRTSCTS);
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
    const auto generation=generation_.load();
    FileDescriptor fd{duplicate_fd()};
    if (fd.value<0) return 0;
    pollfd pending{fd.value,POLLIN,0};
    const auto result=poll(&pending,1,std::max(0,timeout_ms));
    if (result<=0) return 0;
    const auto count=::read(fd.value,buf,size-1);
    if (count>0 && generation==generation_) { buf[count]=0; return static_cast<size_t>(count); }
    if ((pending.revents&(POLLHUP|POLLERR|POLLNVAL)) ||
        (count<0 && errno!=EINTR && errno!=EAGAIN && errno!=EWOULDBLOCK)) {
        close_generation(generation);
    }
    return 0;
}
bool serial_interface::write_data(const char *buf, size_t size) {
    if (size==0) return true;
    const auto generation=generation_.load();
    FileDescriptor fd{duplicate_fd()};
    if (fd.value<0) return false;
    // Duplicate ownership lets read/reopen continue independently while a
    // transmitter is backpressured. The old fd can never target a new port.
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(100);
    size_t sent=0;
    while (sent<size && generation==generation_) {
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
        close_generation(generation);
        return false;
    }
    return sent==size && generation==generation_;
}
std::string serial_interface::get_portname() const {
    std::lock_guard<std::mutex> lock(mutex_); return portname;
}
bool serial_interface::get_port_enabled() const {
    std::lock_guard<std::mutex> lock(mutex_); return usb_fd>=0;
}
void serial_interface::close_port() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (usb_fd>=0) { ::close(usb_fd); usb_fd=-1; ++generation_; }
}
void serial_interface::close_generation(uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation==generation_ && usb_fd>=0) {
        ::close(usb_fd); usb_fd=-1; ++generation_;
    }
}
serial_interface::~serial_interface() { close_port(); }
