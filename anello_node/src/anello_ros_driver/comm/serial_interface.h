#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H
#include "base_interface.h"
#include <atomic>
#include <mutex>
#include <vector>
#define PORT_DIR "/dev/"
#define PORT_PREFIX "ttyUSB"
#define MAX_PORT_PARSE_FAIL 5
#define MAX_CONFIRMED_PORT_FAIL 200
class serial_interface : public base_interface {
public:
    serial_interface()=default;
    ~serial_interface() override;
    serial_interface(const serial_interface &)=delete;
    serial_interface &operator=(const serial_interface &)=delete;
    void init(const std::string &name, uint32_t baud);
    size_t get_data(char *buf, size_t size) override { return get_data(buf,size,0); }
    size_t get_data(char *buf, size_t size, int timeout_ms);
    bool write_data(const char *buf, size_t size) override;
    std::string get_portname() const;
    bool get_port_enabled() const;
    uint64_t generation() const { return generation_; }
    void close_port();
private:
    int duplicate_fd() const;
    void close_generation(uint64_t generation);
    mutable std::mutex mutex_;
    int usb_fd=-1;
    std::string portname;
    std::atomic<uint64_t> generation_{0};
};
#endif
