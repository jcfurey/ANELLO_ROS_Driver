/********************************************************************************
 * File Name:   anello_data_port.cpp
 * Description: Defines the anello_data_port class.
 *
 * Author:      Austin Johnson
 * Date:        7/12/24
 *
 * License:     MIT License
 ********************************************************************************/

#include "../main_anello_ros_driver.h"
#include "../bit_tools.h"

#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <unistd.h>
#include <string>
#include <sys/ioctl.h>
#include <dirent.h>
#include <vector>
#include <stdexcept>

#include "anello_data_port.h"

anello_data_port::anello_data_port(const interface_config_t *config)
    : uart_port(),
      ethernet_port(config->remote_ip, 1, config->local_data_port)  // remote port 1 = ANELLO data channel
{
    this->config = *config;
    this->decode_success = false;
    this->port_index = 0;
    this->fail_count = 0;

    // Enumerate available serial ports
    DIR *dir = opendir(PORT_DIR);
    if (dir != nullptr)
    {
        struct dirent *entry;
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string temp_port_name = PORT_DIR;
            if (strncmp(entry->d_name, PORT_PREFIX, strlen(PORT_PREFIX)) == 0)
            {
                temp_port_name += entry->d_name;
                port_names.push_back(temp_port_name);
            }
        }
        closedir(dir);
    }

    this->auto_detect = (this->config.data_port_name == "AUTO");
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
    else if (this->auto_detect)
    {
        this->init_uart();
    }
    else
    {
        this->uart_port.init(this->config.data_port_name, this->config.baud_rate);
    }
}

void anello_data_port::init_uart()
{
    if (this->auto_detect)
    {
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
    if (this->decode_success || !this->auto_detect)
    {
        return;
    }

    this->fail_count++;

    if (MAX_PORT_PARSE_FAIL < this->fail_count)
    {
        this->fail_count = 0;
        this->port_index++;
        if (this->port_index >= this->port_names.size())
        {
            this->port_index = 0;
        }

        this->config.data_port_name = this->port_names[this->port_index];
        this->uart_port.close_port();

        try {
            this->uart_port.init(this->config.data_port_name, this->config.baud_rate);
        } catch (const std::exception &e) {
            WARNING_PRINT("Failed to open port %s: %s",
                          this->config.data_port_name.c_str(), e.what());
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
    this->fail_count = 0;
    if (this->decode_success) return;

    this->decode_success = true;
    DEBUG_PRINT("Data port confirmed: %s", this->uart_port.get_portname().c_str());
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
        if (timeout_ms > 0)
            this->port_parse_fail();
        return 0;
    }
    return bytes_read;
}

size_t anello_data_port::get_data_ethernet(char *buf, size_t buf_len)
{
    return this->ethernet_port.get_data(buf, buf_len);
}

void anello_data_port::write_data(const char *buf, size_t buf_len)
{
    if (this->config.type == ETH)
        this->write_data_ethernet(buf, buf_len);
    else
        this->write_data_uart(buf, buf_len);
}

void anello_data_port::write_data_uart(const char *buf, size_t buf_len)
{
    this->uart_port.write_data(buf, buf_len);
}

void anello_data_port::write_data_ethernet(const char *buf, size_t buf_len)
{
    this->ethernet_port.write_data(buf, buf_len);
}
