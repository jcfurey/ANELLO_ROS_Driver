#ifndef NTRIP_BUFFER_H
#define NTRIP_BUFFER_H
/********************************************************************************
 * File Name:   ntrip_buffer.h
 * Description: class declaration and header file for ntrip_buffer.cpp
 *
 * Author:      Austin Johnson
 * Date:        8/18/23
 *
 * License:     MIT License
 ********************************************************************************/

#include <cstdint>
#include <vector>

#ifndef NTRIP_BUFFER_SIZE
#define NTRIP_BUFFER_SIZE 4*180
#endif

class port_buffer {
private:
    bool read_ready = false;
    std::vector<uint8_t> buffer;
    uint32_t bytes_used = 0;
public:
    port_buffer();
    ~port_buffer() = default;
    void add_data_to_buffer(const uint8_t *buf, int len);
    void set_read_ready_false();
    void clear_buffer();
    int get_buffer_length();
    const uint8_t *get_buffer();
    bool is_read_ready();
};

#endif
