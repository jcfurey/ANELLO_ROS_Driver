/********************************************************************************
 * File Name:   anello_config_port.cpp
 * Description: Defines the anello_config_port class.
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

#include "anello_config_port.h"

anello_config_port::anello_config_port(const interface_config_t *config)
    : uart_port(),
      ethernet_port(config->remote_ip, 2, config->local_config_port)  // remote port 2 = ANELLO config channel
{
    this->config = *config;
}

anello_config_port::~anello_config_port()
{
    // Members are destroyed automatically — no manual destructor calls
}

void anello_config_port::init()
{
    if (this->config.type == ETH)
    {
        this->init_ethernet();
    }
    else
    {
        this->init_uart();
    }
}

void anello_config_port::init_uart()
{
    if (this->config.config_port_name == "AUTO")
    {
        DIR *dir = opendir(PORT_DIR);
        if (nullptr == dir)
        {
            throw std::runtime_error("Failed to open port directory: " +
                                     std::string(PORT_DIR));
        }

        struct dirent *entry;
        std::vector<std::string> port_names;
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string temp_port_name = PORT_DIR;
            if (strncmp(entry->d_name, PORT_PREFIX, strlen(PORT_PREFIX)) == 0)
            {
                temp_port_name += entry->d_name;
                port_names.insert(port_names.begin(), temp_port_name);
            }
        }
        closedir(dir);

        if (port_names.empty())
        {
            throw std::runtime_error("No serial ports found matching " +
                                     std::string(PORT_DIR) + std::string(PORT_PREFIX) + "*");
        }

        std::string command = "#APPNG*48\r\n";
        bool port_found = false;
        int max_attempts = 10;

        for (int attempt = 0; attempt < max_attempts && !port_found; ++attempt)
        {
            for (const auto &port_name : port_names)
            {
                try {
                    this->config.config_port_name = port_name;
                    this->uart_port.init(this->config.config_port_name, this->config.baud_rate);
                    char buf[100] = {0};

                    this->uart_port.get_data(buf, 100, 10);
                    this->uart_port.write_data(command.c_str(), command.length());
                    usleep(500 * 1000);
                    this->uart_port.get_data(buf, 100, 10);

                    if (strstr(buf, "#APPNG") != nullptr)
                    {
                        port_found = true;
                        DEBUG_PRINT("Config port found: %s", port_name.c_str());
                        break;
                    }
                    else
                    {
                        this->uart_port.close_port();
                    }
                } catch (const std::exception &e) {
                    // Port failed to open, try next
                    this->uart_port.close_port();
                }
            }

            if (!port_found)
            {
                WARNING_PRINT("Config port not found (attempt %d/%d), retrying...",
                              attempt + 1, max_attempts);
                usleep(1000 * 1000); // 1 second between retry rounds
            }
        }

        if (!port_found)
        {
            throw std::runtime_error("Config port auto-detection failed after " +
                                     std::to_string(max_attempts) + " attempts");
        }
    }
    else
    {
        this->uart_port.init(this->config.config_port_name, this->config.baud_rate);
    }
}

void anello_config_port::init_ethernet()
{
    this->ethernet_port.init();

    std::string command = "#APPNG*48\r\n";
    char buf[100] = {0};

    this->ethernet_port.write_data(command.c_str(), command.length());
    usleep(500 * 1000);
    this->ethernet_port.get_data(buf, 100);
}

size_t anello_config_port::get_data(char *buf, size_t buf_len)
{
    if (this->config.type == ETH)
        return this->get_data_ethernet(buf, buf_len);
    else
        return this->get_data_uart(buf, buf_len);
}

size_t anello_config_port::get_data(char *buf, size_t buf_len, int timeout_ms)
{
    if (this->config.type == ETH)
        return this->ethernet_port.get_data(buf, buf_len, timeout_ms);
    else
        return this->uart_port.get_data(buf, buf_len, timeout_ms);
}

size_t anello_config_port::get_data_uart(char *buf, size_t buf_len)
{
    return this->uart_port.get_data(buf, buf_len);
}

size_t anello_config_port::get_data_ethernet(char *buf, size_t buf_len)
{
    return this->ethernet_port.get_data(buf, buf_len);
}

void anello_config_port::write_data(const char *buf, size_t buf_len)
{
    if (this->config.type == ETH)
        this->write_data_ethernet(buf, buf_len);
    else
        this->write_data_uart(buf, buf_len);
}

void anello_config_port::write_data_uart(const char *buf, size_t buf_len)
{
    this->uart_port.write_data(buf, buf_len);
}

void anello_config_port::write_data_ethernet(const char *buf, size_t buf_len)
{
    this->ethernet_port.write_data(buf, buf_len);
}

double anello_config_port::get_baseline()
{
    std::string command = "#APVEH,R,bsl*65\r\n";
    char *field_array[MAXFIELD];
    char buf[100] = {0};

    this->write_data(command.c_str(), command.length());
    usleep(500 * 1000);
    this->get_data(buf, 100);

    int num_fields = parse_fields(buf, field_array);
    if (num_fields < 3)
    {
        return 0.0;
    }

    if ((strstr(field_array[0], "APVEH") == nullptr) ||
        (strstr(field_array[1], "bsl") == nullptr))
    {
        return 0.0;
    }

    return atof(field_array[2]);
}
