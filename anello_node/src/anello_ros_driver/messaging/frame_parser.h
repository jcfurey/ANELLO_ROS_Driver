/********************************************************************************
 * File Name:   frame_parser.h
 * Description: Byte-stream frame parser for ANELLO ASCII and RTCM messages.
 *
 * License:     MIT License
 ********************************************************************************/

#ifndef FRAME_PARSER_H
#define FRAME_PARSER_H

#include <cstdint>

#include "../bit_tools.h"  // MAXFIELD

#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN (1200)
#endif

// Message buffer structure
struct a1buff_t
{
    uint8_t buf[MAX_BUF_LEN] = {};
    int nseg = 0;
    int nbyte = 0;
    int nlen = 0;
    int type = 0;
    int subtype = 0;
    int crc = 0;
    int loc[MAXFIELD] = {};
};

int input_a1_data(a1buff_t *a1, uint8_t data);

#endif
