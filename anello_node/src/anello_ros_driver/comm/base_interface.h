/********************************************************************************
 * File Name:   base_interface.h
 * Description: Contains the base implementation used for all ANELLO device interfaces.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:        All interfaces to the ANELLO devices should inherit from this class.
 ********************************************************************************/

#ifndef BASE_INTERFACE_H
#define BASE_INTERFACE_H

#include <string>
#include <cstdint>

enum interface_type_t
{
    UART,
    ETH
};

struct interface_config_t
{
    interface_type_t type = UART;
    std::string data_port_name;
    std::string config_port_name;
    std::string remote_ip;
    int local_data_port = 1111;
    int local_config_port = 2222;
    int local_odometer_port = 3333;
    uint32_t baud_rate = 230400;
};

class base_interface
{
public:
    virtual ~base_interface() = default;

    virtual size_t get_data(char *buf, size_t buf_len) { (void)buf; (void)buf_len; return 0; }
    virtual void write_data(const char *buf, size_t buf_len) { (void)buf; (void)buf_len; }
};

#endif
