/********************************************************************************
 * File Name:   frame_parser.cpp
 * Description: Byte-stream frame parser for ANELLO ASCII and RTCM messages.
 *
 * License:     MIT License
 ********************************************************************************/

#include "frame_parser.h"

#include "../bit_tools.h"

/* State machine decoder for ASCII and RTCM messages
 *
 * Return:
 *   0 = not ready
 *   1 = ASCII message ready
 *   5 = RTCM message ready
 */
int input_a1_data(a1buff_t *a1, uint8_t data)
{
    int ret = 0;

    // Reset one byte early: buf is zeroed at frame start, so capping
    // nbyte at MAX_BUF_LEN - 1 keeps buf[MAX_BUF_LEN - 1] an untouched
    // NUL. A frame completing at exactly MAX_BUF_LEN bytes would
    // otherwise reach parse_fields()/"%s" logging unterminated.
    if (a1->nbyte >= MAX_BUF_LEN - 1)
        a1->nbyte = 0;

    // Detect correct start characters: #AP or 0xD3
    if (a1->nbyte == 0 && !(data == '#' || data == 0xD3))
    {
        a1->nbyte = 0;
        return 0;
    }
    // On a header mismatch, re-run the rejected byte through start
    // detection (single-level recursion: nbyte is 0 on re-entry) so a
    // '#' or 0xD3 that aborts a false header still opens a new frame —
    // otherwise "#A#APIMU,..." style streams drop the valid message.
    if (a1->nbyte == 1 && !((data == 'A' && a1->buf[0] == '#') || a1->buf[0] == 0xD3))
    {
        a1->nbyte = 0;
        return input_a1_data(a1, data);
    }
    if (a1->nbyte == 2 && !((data == 'P' && a1->buf[1] == 'A' && a1->buf[0] == '#') || a1->buf[0] == 0xD3))
    {
        a1->nbyte = 0;
        return input_a1_data(a1, data);
    }

    if (a1->nbyte == 0)
    {
        *a1 = a1buff_t{};
    }

    if (a1->nbyte < 3)
    {
        a1->buf[a1->nbyte++] = data;
        return 0;
    }

    if (a1->buf[0] != 0xD3)
    {
        // ASCII message
        if (data == ',')
        {
            if (a1->nseg < MAXFIELD)
                a1->loc[a1->nseg++] = a1->nbyte;
            if (a1->nseg == 2)
                a1->nlen = 0;
        }

        a1->buf[a1->nbyte++] = data;

        if (a1->nlen == 0)
        {
            if (data == '\r' || data == '\n')
            {
                if (a1->nbyte > 3 && a1->buf[a1->nbyte - 4] == '*')
                {
                    if (a1->nseg < MAXFIELD)
                        a1->loc[a1->nseg++] = a1->nbyte - 4;
                    ret = 1;
                }
            }
        }
    }
    else
    {
        // RTCM message
        a1->buf[a1->nbyte++] = data;
        a1->nlen = getbitu(a1->buf, 14, 10) + 3;
        if (a1->nbyte >= a1->nlen + 3)
        {
            int i = 24;
            a1->type = getbitu(a1->buf, i, 12);
            i += 12;

            if (crc24q(a1->buf, a1->nlen) != getbitu(a1->buf, a1->nlen * 8, 24))
            {
                a1->crc = 1;
            }
            else
            {
                a1->crc = 0;
                if (a1->type == 4058)
                    a1->subtype = getbitu(a1->buf, i, 4);
            }
            ret = 5;
        }
    }
    return ret;
}
