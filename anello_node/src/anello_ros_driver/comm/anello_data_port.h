/********************************************************************************
 * File Name:   anello_data_port.h
 * Description: header file for the anello_data_port class.
 *
 * Author:      Austin Johnson
 * Date:        7/12/24
 *
 * License:     MIT License
 *
 * Note:        The anello_data_port class is used to read/write data to the anello GNSS INS, EVK, and IMU+.
 *              This class is meant to abstract the UART/ethernet communication with the anello devices.
 *
 ********************************************************************************/

#ifndef ANELLO_DATA_PORT_H
#define ANELLO_DATA_PORT_H

#include "serial_interface.h"
#include "ethernet_interface.h"

class anello_data_port
{
private:
    bool decode_success = false;
    bool auto_detect = false;
    bool had_loss = false;      // a confirmed stream was lost (log recovery)
    bool reopen_warned = false; // one open-failure warning per loss episode
    std::vector<std::string> port_names;
    std::string port_dir;
    uint32_t port_index = 0;
    int fail_count = 0;

    interface_config_t config;
    serial_interface uart_port;
    ethernet_interface ethernet_port;

    void init_ethernet();
    void init_uart();

    /* Refresh port_names from port_dir (sorted for a deterministic scan
     * order). Re-run on every scan step: a power-cycled unit usually
     * re-enumerates under a different /dev name. */
    void enumerate_ports();

    void port_parse_fail_uart();
    void port_parse_fail_ethernet();
    void port_confirm_uart();
    void port_confirm_ethernet();

    size_t get_data_uart(char *buf, size_t buf_len, int timeout_ms);
    size_t get_data_ethernet(char *buf, size_t buf_len);

    void write_data_uart(const char *buf, size_t buf_len);
    void write_data_ethernet(const char *buf, size_t buf_len);
public:
    /*
     * Notes:
     * This constructor does not initialize its port.
     * port_directory overrides where AUTO mode scans for serial ports
     * (tests point it at a directory of pty symlinks).
     */
    explicit anello_data_port(const interface_config_t *config,
                              std::string port_directory = PORT_DIR);
    ~anello_data_port();

    /*
     * Notes:
     * This function initializes the serial port interface including configuring the port.
     */
    void init();
    size_t get_data(char *buf, size_t buf_len);
    /* timeout_ms applies to UART reads only (select() bound); ethernet
     * reads are always non-blocking. A 0 ms UART poll that returns empty
     * does not count toward port rotation. */
    size_t get_data(char *buf, size_t buf_len, int timeout_ms);
    void write_data(const char *buf, size_t buf_len);

    void port_parse_fail();
    void port_confirm();

    const std::string get_portname() const {
        if (this->config.type == ETH) {
            return this->ethernet_port.get_remote_ip(); // Assuming get_remote_ip() exists
        } else {
            return this->uart_port.get_portname();
        }
    }

};     

#endif // ANELLO_CONFIG_PORT_H