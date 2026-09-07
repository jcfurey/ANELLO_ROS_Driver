#ifndef ANELLO_CONFIG_PORT_H
#define ANELLO_CONFIG_PORT_H
#include <atomic>
#include <chrono>
#include "serial_interface.h"
#include "ethernet_interface.h"
class anello_config_port {
public:
    explicit anello_config_port(const interface_config_t *config, std::string directory=PORT_DIR);
    void init();
    void poll();  // run in the config callback group
    size_t get_data(char *buf, size_t size) { return get_data(buf,size,0); }
    size_t get_data(char *buf, size_t size, int timeout_ms);
    bool write_data(const char *buf, size_t size);
    bool connected() const { return confirmed_; }
    std::string get_portname() const {
        return config_.type==ETH?ethernet_.get_remote_ip():uart_.get_portname();
    }
private:
    interface_config_t config_;
    serial_interface uart_;
    ethernet_interface ethernet_;
    std::string directory_, probe_response_;
    size_t scan_index_=0;
    bool probing_=false;
    std::atomic<bool> confirmed_{false};
    std::chrono::steady_clock::time_point deadline_{};
};
#endif
