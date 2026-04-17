/********************************************************************************
 * File Name:   ntrip_buffer.cpp
 * Description: definition for ntrip_buffer.h
 *
 * Author:      Austin Johnson
 * Date:        8/18/23
 *
 * License:     MIT License
 ********************************************************************************/

#include "ntrip_buffer.h"
#include <cstring>

port_buffer::port_buffer()
    : buffer(NTRIP_BUFFER_SIZE, 0)
{
}

void port_buffer::add_data_to_buffer(const uint8_t *buf, int len)
{
    this->clear_buffer();

    if (len > static_cast<int>(buffer.size()))
        len = static_cast<int>(buffer.size());

    std::memcpy(this->buffer.data(), buf, len);

    this->read_ready = true;
    this->bytes_used = len;
}

void port_buffer::clear_buffer()
{
    std::memset(this->buffer.data(), 0, buffer.size());
    this->read_ready = false;
    this->bytes_used = 0;
}

int port_buffer::get_buffer_length()
{
    return this->bytes_used;
}

const uint8_t *port_buffer::get_buffer()
{
    return this->buffer.data();
}

bool port_buffer::is_read_ready()
{
    return this->read_ready;
}

void port_buffer::set_read_ready_false()
{
    this->read_ready = false;
}
