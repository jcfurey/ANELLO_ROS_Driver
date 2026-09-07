#include "anello_config_port.h"
#include "../bit_tools.h"
#include <algorithm>
#include <filesystem>
#include <vector>
anello_config_port::anello_config_port(const interface_config_t *config, std::string directory)
    : config_(*config), ethernet_(config->remote_ip,2,config->local_config_port),
      directory_(std::move(directory)) {}
void anello_config_port::init() {
    if (config_.type==ETH) { ethernet_.init(); confirmed_=true; }
    else poll();
}
void anello_config_port::poll() {
    if (config_.type==ETH || config_.config_port_name=="OFF") return;
    const auto now=std::chrono::steady_clock::now();
    if (confirmed_ && uart_.get_port_enabled()) {
        // Detect a hangup even with no commands or odometer input. These
        // unsolicited config bytes have no waiting service consumer.
        char discard[512]; uart_.get_data(discard,sizeof(discard),0);
        if (uart_.get_port_enabled()) return;
    }
    confirmed_=false;
    if (probing_) {
        char response[256]; auto n=uart_.get_data(response,sizeof(response),0);
        probe_response_.append(response,n);
        auto end=probe_response_.find("\r\n");
        while (end!=std::string::npos) {
            auto line=probe_response_.substr(0,end+2);
            probe_response_.erase(0,end+2);
            if ((line.rfind("#APPNG,",0)==0 || line.rfind("#APPNG*",0)==0) && checksum(reinterpret_cast<const unsigned char *>(line.data()),line.size())) {
                confirmed_=true; probing_=false; return;
            }
            end=probe_response_.find("\r\n");
        }
        if (now<deadline_ && probe_response_.size()<1024 && uart_.get_port_enabled()) return;
        probing_=false; uart_.close_port();
    }
    if (now<deadline_) return;
    deadline_=now+std::chrono::milliseconds(500);
    std::string name=config_.config_port_name;
    if (name=="AUTO") {
        std::vector<std::string> ports;
        std::error_code error;
        for (const auto &entry:std::filesystem::directory_iterator(directory_,error))
            if (entry.path().filename().string().rfind(PORT_PREFIX,0)==0) ports.push_back(entry.path());
        std::sort(ports.rbegin(),ports.rend());
        if (ports.empty()) return;
        name=ports[scan_index_++%ports.size()];
    }
    try {
        uart_.init(name,config_.baud_rate);
        if (config_.config_port_name=="AUTO") {
            probe_response_.clear(); probing_=uart_.write_data("#APPNG*48\r\n",11);
            if (!probing_) uart_.close_port();
        } else confirmed_=true;
    } catch (const std::exception &) { uart_.close_port(); }
}
size_t anello_config_port::get_data(char *buf, size_t size, int timeout_ms) {
    if (!confirmed_) return 0;
    return config_.type==ETH?ethernet_.get_data(buf,size,timeout_ms):uart_.get_data(buf,size,timeout_ms);
}
bool anello_config_port::write_data(const char *buf, size_t size) {
    if (!confirmed_) poll();
    if (!confirmed_) return false;
    const bool sent=config_.type==ETH?ethernet_.write_data(buf,size):uart_.write_data(buf,size);
    if (!sent && config_.type==UART) confirmed_=false;
    return sent;
}
