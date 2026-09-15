// Copyright (c) 2023 ANELLO Photonics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/********************************************************************************
 * File Name:   bit_tools.h
 * Description: Header file for bit_tools.cpp.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:
 ********************************************************************************/

#ifndef ANELLO_ROS_DRIVER__BIT_TOOLS_H_
#define ANELLO_ROS_DRIVER__BIT_TOOLS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef MAXFIELD
#define MAXFIELD 20
#endif

#include <stdint.h>
unsigned int crc24q(const unsigned char *buff, int len);

void setbitu(unsigned char *buff, int pos, int len, unsigned int data);
unsigned int getbitu(const unsigned char *buff, int pos, int len);
int getbits(const unsigned char *buff, int pos, int len);

    /*
     * Parameters:
     * unsigned char* buff : buffer containing full ASCII message
     * int len : length of the message
     *
     * Return:
     * int
     * non-zero : checksum at the end of the message matches the calculated checksum
     * zero : The checksum is not correct and the message is invalid
     *
     */
int checksum(const unsigned char *buff, int len);


#ifdef __cplusplus
}
#endif

#include <string>
extern std::string compute_checksum(const char *buff, int len);
extern int parse_fields(char *const buffer, char **val);
#endif  // ANELLO_ROS_DRIVER__BIT_TOOLS_H_
