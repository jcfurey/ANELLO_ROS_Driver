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
 * File Name:   rtcm_decoder.h
 * Description: header file for the rtcm_decoder.cpp.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:
 ********************************************************************************/

#ifndef ANELLO_ROS_DRIVER__MESSAGING__RTCM_DECODER_H_
#define ANELLO_ROS_DRIVER__MESSAGING__RTCM_DECODER_H_

#include "anello_ros_driver/protocol_types.h"
enum RECEIVER_ID
{
  GPS1 = 1,
  GPS2 = 2
};

/*
 * Parameters:
 * double imu[] : pointer to an array of size 'MAXFIELDS' which will be filled with the values of the message
 * a1buff_t a1buff : buffer variable where the rtcm message is buffered and information about it is stored
 *
 */
void decode_rtcm_imu_msg(double imu[], const a1buff_t & a1buff);

/*
 * Parameters:
 * double im1[] : pointer to an array of size 'MAXFIELDS' which will be filled with the values of the message
 * a1buff_t a1buff : buffer variable where the rtcm message is buffered and information about it is stored
 *
 */
void decode_rtcm_im1_msg(double im1[], const a1buff_t & a1buff);

/*
 * Parameters:
 * double ins[] : pointer to an array of size 'MAXFIELDS' which will be filled with the values of the message
 * a1buff_t a1buff : buffer variable where the rtcm message is buffered and information about it is stored
 *
 */
void decode_rtcm_ins_msg(double ins[], const a1buff_t & a1buff);

/*
 * Parameters:
 * double gps[] : pointer to an array of size 'MAXFIELDS' which will be filled with the values of the message
 * a1buff_t a1buff : buffer variable where the rtcm message is buffered and information about it is stored
 *
 * Return:
 * 1 if antenna 1
 * 2 if antenna 2
 */
int decode_rtcm_gps_msg(double gps[], const a1buff_t & a1buff);

/*
 * Parameters:
 * double hdg[] : pointer to an array of size 'MAXFIELDS' which will be filled with the values of the message
 * a1buff_t a1buff : buffer variable where the rtcm message is buffered and information about it is stored
 *
 */
void decode_rtcm_hdg_msg(double hdg[], const a1buff_t & a1buff);

/*
 * Parameters:
 * double cov[] : pointer to an array of size 'MAXFIELDS' which will be filled with the values of the message
 * a1buff_t a1buff : buffer variable where the rtcm message is buffered and information about it is stored
 *
 */
void decode_rtcm_cov_msg(double cov[], const a1buff_t & a1buff);

#endif  // ANELLO_ROS_DRIVER__MESSAGING__RTCM_DECODER_H_
