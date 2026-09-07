/********************************************************************************
 * File Name:   anello_data_port.cpp
 * Description: Defines the anello_data_port class.
 *
 * Author:      Austin Johnson
 * Date:        7/12/24
 *
 * License:     MIT License
 ********************************************************************************/

#include "../logging.h"
#include "../bit_tools.h"

#include <fcntl.h>
#include <termios.h>
#include <algorithm>
#include <cstring>
#include <unistd.h>
#include <string>
#include <sys/ioctl.h>
#include <dirent.h>
#include <utility>
#include <vector>
#include <stdexcept>

#include "anello_data_port.h"

anello_data_port::anello_data_port(const interface_config_t *config,
                                   std::string port_directory)
    : port_dir(std::move(port_directory)),
      uart_port(),
      ethernet_port(config->remote_ip, 1, config->local_data_port)  // remote port 1 = ANELLO data channel
{
    this->config = *config;
    this->decode_success = false;
    this->port_index = 0;
    this->fail_count = 0;

    this->auto_detect = (this->config.data_port_name == "AUTO");
}

void anello_data_port::enumerate_ports()
{
    this->port_names.clear();
    DIR *dir = opendir(this->port_dir.c_str());
    if (dir == nullptr)
    {
        return;
    }
    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        if (strncmp(entry->d_name, PORT_PREFIX, strlen(PORT_PREFIX)) == 0)
        {
            this->port_names.push_back(this->port_dir + entry->d_name);
        }
    }
    closedir(dir);
    // Deterministic scan order (readdir order is arbitrary); on an EVK
    // the data port is the lowest-numbered tty of the FTDI block.
    std::sort(this->port_names.begin(), this->port_names.end());
}

anello_data_port::~anello_data_port()
{
    // Members are destroyed automatically — no manual destructor calls
}

void anello_data_port::init()
{
    if (this->config.type == ETH)
    {
        this->init_ethernet();
    }
    else {
        try { this->init_uart(); }
        catch (const std::exception &e) { WARNING_PRINT("Waiting for data port: %s",e.what()); }
    }
}

void anello_data_port::init_uart()
{
    next_open_=std::chrono::steady_clock::now()+std::chrono::milliseconds(500);
    if (this->auto_detect)
    {
        this->enumerate_ports();
        if (port_names.empty())
        {
            throw std::runtime_error("No serial ports found for auto-detection");
        }
        if (this->port_index >= this->port_names.size())
        {
            this->port_index = 0;
        }
        this->config.data_port_name = this->port_names[this->port_index];
    }

    this->uart_port.init(this->config.data_port_name, this->config.baud_rate);
}

void anello_data_port::init_ethernet()
{
    this->ethernet_port.init();
}

void anello_data_port::port_parse_fail()
{
    if (this->config.type == ETH)
        this->port_parse_fail_ethernet();
    else
        this->port_parse_fail_uart();
}

void anello_data_port::port_parse_fail_uart()
{
    // No port_enabled gate here: if a rotation's init() failed (port
    // unplugged, EBUSY), the port stays disabled and reads return 0 — the
    // fail count must keep advancing or the scan deadlocks on the dead port.
    this->fail_count++;

    if (this->decode_success)
    {
        // Confirmed link. A vanished device node (unit power-cycle -> USB
        // re-enumeration) shows up as a tty hangup, which closed the port
        // inside serial_interface — react immediately. An open port that
        // merely went silent (or streams only garbage) must instead cross
        // the generous sustained-failure threshold, so noise bursts and
        // GNSS-outage lulls cannot drop a live link.
        if (this->uart_port.get_port_enabled() &&
            this->fail_count <= MAX_CONFIRMED_PORT_FAIL)
        {
            return;
        }
        WARNING_PRINT("ANELLO data stream lost on %s — %s",
                      this->uart_port.get_portname().c_str(),
                      this->auto_detect ? "restarting port scan"
                                        : "reopening the port until it returns");
        this->decode_success = false;
        this->had_loss = true;
        this->reopen_warned = false;
        this->fail_count = 0;
        this->uart_port.close_port();
        confirmed_generation_=0;
        next_open_=std::chrono::steady_clock::now()+std::chrono::milliseconds(500);
        return;
    }

    // Unconfirmed: scanning (AUTO) or waiting for a fixed-name port to
    // come back. AUTO advances quickly; a fixed name is retried at the
    // slower confirmed cadence (~1 s) — udev recreates the node (or
    // re-points a /dev/serial/by-id symlink) when the device returns.
    const int fail_limit = this->auto_detect ? MAX_PORT_PARSE_FAIL
                                             : MAX_CONFIRMED_PORT_FAIL;
    if (fail_limit < this->fail_count)
    {
        const auto now=std::chrono::steady_clock::now();
        if (now<next_open_) return;
        next_open_=now+std::chrono::milliseconds(500);
        this->fail_count = 0;
        this->uart_port.close_port();

        if (this->auto_detect)
        {
            this->enumerate_ports();
            if (this->port_names.empty())
            {
                return;  // nothing to probe yet; keep polling
            }
            this->port_index++;
            if (this->port_index >= this->port_names.size())
            {
                this->port_index = 0;
            }
            this->config.data_port_name = this->port_names[this->port_index];
        }

        try {
            this->uart_port.init(this->config.data_port_name, this->config.baud_rate);
        } catch (const std::exception &e) {
            // One visible warning per loss episode; a dead unit would
            // otherwise flood the log at the scan rate.
            if (!this->reopen_warned)
            {
                this->reopen_warned = true;
                WARNING_PRINT("Failed to open port %s: %s",
                              this->config.data_port_name.c_str(), e.what());
            }
            else
            {
                DEBUG_PRINT("Failed to open port %s: %s",
                            this->config.data_port_name.c_str(), e.what());
            }
        }
    }
}

void anello_data_port::port_parse_fail_ethernet()
{
    /* Nothing to do */
}

void anello_data_port::port_confirm()
{
    if (this->config.type == ETH)
        this->port_confirm_ethernet();
    else
        this->port_confirm_uart();
}

void anello_data_port::port_confirm_uart()
{
    confirmed_generation_=uart_port.generation();
    last_ok_=std::chrono::steady_clock::now();
    this->fail_count = 0;
    if (this->decode_success) return;

    this->decode_success = true;
    this->reopen_warned = false;
    if (this->had_loss)
    {
        // The loss was logged at WARN; make the recovery just as visible.
        this->had_loss = false;
        WARNING_PRINT("ANELLO data stream re-acquired on %s",
                      this->uart_port.get_portname().c_str());
    }
    else
    {
        DEBUG_PRINT("Data port confirmed: %s", this->uart_port.get_portname().c_str());
    }
}

void anello_data_port::port_confirm_ethernet()
{
    /* Nothing to do */
}

size_t anello_data_port::get_data(char *buf, size_t buf_len)
{
    return this->get_data(buf, buf_len, 10);
}

size_t anello_data_port::get_data(char *buf, size_t buf_len, int timeout_ms)
{
    if (this->config.type == ETH)
        return this->get_data_ethernet(buf, buf_len);
    else
        return this->get_data_uart(buf, buf_len, timeout_ms);
}

size_t anello_data_port::get_data_uart(char *buf, size_t buf_len, int timeout_ms)
{
    size_t bytes_read = this->uart_port.get_data(buf, buf_len, timeout_ms);
    if (bytes_read == 0)
    {
        // Only a blocking read that came up empty counts as "no data" for
        // port rotation; a 0 ms drain poll returning empty is the normal
        // end of a drained tick.
        const auto now=std::chrono::steady_clock::now();
        if (timeout_ms>0 || !uart_port.get_port_enabled() || now-last_ok_>std::chrono::seconds(2)) {
            // One state-machine step per timer cadence, independent of the
            // number of empty nonblocking drain reads.
            if (now-last_retry_>=std::chrono::milliseconds(5)) {
                last_retry_=now; this->port_parse_fail();
            }
        }
        return 0;
    }
    return bytes_read;
}

size_t anello_data_port::get_data_ethernet(char *buf, size_t buf_len)
{
    return this->ethernet_port.get_data(buf, buf_len);
}

bool anello_data_port::write_data(const char *buf, size_t buf_len)
{
    if (this->config.type == ETH)
        return this->write_data_ethernet(buf, buf_len);
    else
        return this->write_data_uart(buf, buf_len);
}

bool anello_data_port::write_data_uart(const char *buf, size_t buf_len)
{
    const auto generation=confirmed_generation_.load();
    return generation!=0 && this->uart_port.write_data(buf,buf_len,generation);
}

bool anello_data_port::write_data_ethernet(const char *buf, size_t buf_len)
{
    return this->ethernet_port.write_data(buf, buf_len);
}
