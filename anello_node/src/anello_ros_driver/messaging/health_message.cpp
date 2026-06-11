/********************************************************************************
 * File Name:   health_message.cpp
 * Description: Definition of the health_message class
 *
 * Author:      Austin Johnson
 * Date:        1/29/24
 *
 * License:     MIT License
 *
 * Note: This class is used to store the health status of the system. It is
 *       updated by the health_message class and read by the main thread. Finally,
 *       it is published by the main thread.       
 ********************************************************************************/

#include <stdio.h>
#include <cstdlib>
#include <cmath>
#include "health_message.h"

// Gross-fault gate on the MEMS-vs-FOG window-mean difference. The noise
// floor of the comparison is sigma_d = sqrt(ARW_mems^2 + ARW_fog^2 / N/Fs)
// ~ 5e-3 deg/s for a 1 s window, so 0.25 deg/s is a ~50-sigma gate: it
// cannot false-alarm on sensor noise, still covers a 1000 ppm
// scale-factor mismatch at 100 deg/s, and detects real failures 8x
// sooner than the previous 2 deg/s gate (which was ~400 sigma).
#ifndef GYRO_DISCREPANCY_THRESHOLD
#define GYRO_DISCREPANCY_THRESHOLD 0.25 // deg/s
#endif

#ifndef HEADING_STABILITY_THRESHOLD
#define HEADING_STABILITY_THRESHOLD 3  // deg
#endif

#ifndef GOOD_GPS_ACC_TRESHOLD
#define GOOD_GPS_ACC_TRESHOLD 1         // m
#endif

#ifndef BASELINE_ACC_THRESHOLD
#define BASELINE_ACC_THRESHOLD .03
#endif

#ifndef GPS_HEADING_ACC_GOOD_THRESHOLD
#define GPS_HEADING_ACC_GOOD_THRESHOLD 1
#endif

// GNSS course-over-ground accuracy degrades as atan(sigma_v/speed): with
// ~0.05 m/s Doppler accuracy the 3 deg heading threshold is only a
// 1-sigma test at 1 m/s (false streaks while maneuvering slowly) but
// ~2-3 sigma at 2 m/s and above. Receivers also freeze COG near
// standstill. Gate the GPS-vs-INS heading comparison on speed.
#ifndef GPS_HEADING_MIN_SPEED
#define GPS_HEADING_MIN_SPEED 2.0 // m/s
#endif

// Stuck-channel floors: a healthy channel's per-sample std is
// ARW * sqrt(Fs); these are 0.2x that at the slowest ODR (20 Hz), the
// worst case across 20-200 Hz (MEMS ARW 0.3 deg/sqrt-hr, FOG 0.05).
// P(std < 0.2 sigma | N=100) < 1e-40, so false alarms are negligible.
#ifndef STUCK_STD_FLOOR_MEMS
#define STUCK_STD_FLOOR_MEMS 4.5e-3 // deg/s
#endif
#ifndef STUCK_STD_FLOOR_FOG
#define STUCK_STD_FLOOR_FOG 7.4e-4 // deg/s
#endif

#ifndef HEADING_MISMATCH_COUNT_TH
#define HEADING_MISMATCH_COUNT_TH 4
#endif

// The ANELLO optical gyro range is 200 deg/s (vs 450 deg/s for the MEMS
// gyro), so near that rate OG_WZ saturates while WZ still tracks. A
// divergence there is a range limit, not a fault — suppress the
// discrepancy check above this guard.
#ifndef FOG_SATURATION_GUARD_DPS
#define FOG_SATURATION_GUARD_DPS 180.0
#endif

// APHDG status flags (mirrors u-blox RELPOSNED): heading is only
// meaningful when the GNSS fix is OK, the relative position is valid,
// and the heading itself is flagged valid.
#define HDG_FLAG_GNSS_FIX_OK (1 << 0)
#define HDG_FLAG_REL_POS_VALID (1 << 2)
#define HDG_FLAG_HEADING_VALID (1 << 8)

health_message::health_message()
{
    this->cur_imu_time = 0.0;
    this->buffer_full = false;
    this->wz_mems_current_sum = 0.0;
    this->wz_fog_current_sum = 0.0;
    this->wz_mems_moving_average = 0.0;
    this->wz_fog_moving_average = 0.0;
    this->wz_mems_std_dev = 1.0;
    this->wz_fog_std_dev = 1.0;
    this->circular_buffer_index = 0.0;

    for (int i = 0; i < IMU_MOVING_AVERAGE_SIZE; i++)
    {
        this->wz_mems_circular_buffer[i] = 0.0;
        this->wz_fog_circular_buffer[i] = 0.0;
    }

    this->ins_heading = 0.0;
    this->gps_heading = 0.0;
    this->hdg_heading = 0.0;

    this->hdg_baseline = 0;
    this->configured_baseline = 0.0;

    this->gps_hacc = 0.0;
    this->gps_heading_acc = 0.0;
    this->hdg_heading_acc = 0.0;
    this->rtk_status = 0;

    this->gps_read_flag = false;
    this->hdg_read_flag = false;
    this->gps_ins_mismatch_streak = 0;
    this->hdg_ins_mismatch_streak = 0;
}

void health_message::add_imu_message(double *imu_msg)
{
    this->cur_imu_time = imu_msg[0];
    double wz = imu_msg[6];
    double wz_fog = imu_msg[7];

    // add new data
    this->wz_mems_current_sum += wz;
    this->wz_fog_current_sum += wz_fog;

    // subtract old data
    this->wz_mems_current_sum -= this->wz_mems_circular_buffer[this->circular_buffer_index];
    this->wz_fog_current_sum -= this->wz_fog_circular_buffer[this->circular_buffer_index];

    // add to circular buffer
    this->wz_mems_circular_buffer[this->circular_buffer_index] = wz;
    this->wz_fog_circular_buffer[this->circular_buffer_index] = wz_fog;

    // increment circular buffer index
    this->circular_buffer_index++;
    if (this->circular_buffer_index >= IMU_MOVING_AVERAGE_SIZE)
    {
        this->circular_buffer_index = 0;
        this->buffer_full = true;
    }

    // update moving average when buffer is full
    if (this->buffer_full)
    {
        this->wz_mems_moving_average = this->wz_mems_current_sum / IMU_MOVING_AVERAGE_SIZE;
        this->wz_fog_moving_average = this->wz_fog_current_sum / IMU_MOVING_AVERAGE_SIZE;

        // calculate standard deviation
        double wz_mems_std_dev_sum = 0;
        double wz_fog_std_dev_sum = 0;
        for (int i = 0; i < IMU_MOVING_AVERAGE_SIZE; i++)
        {
            wz_mems_std_dev_sum += pow(this->wz_mems_circular_buffer[i] - this->wz_mems_moving_average, 2);
            wz_fog_std_dev_sum += pow(this->wz_fog_circular_buffer[i] - this->wz_fog_moving_average, 2);
        }
        
        this->wz_mems_std_dev = sqrt(wz_mems_std_dev_sum / IMU_MOVING_AVERAGE_SIZE);
        this->wz_fog_std_dev = sqrt(wz_fog_std_dev_sum / IMU_MOVING_AVERAGE_SIZE);
    }
}

void health_message::add_ins_message(double* ins_msg)
{ 
    this->ins_heading = ins_msg[11];
    double ins_status = ins_msg[2];

    double gps_ins_diff, hdg_ins_diff;
    this->get_current_diff(&gps_ins_diff, &hdg_ins_diff);

    // if gps has been read recently update mismatch streak accordingly
    if (this->gps_read_flag)
    {
        this->gps_read_flag = false;

        if (this->is_single_antenna_heading_valid())
        {
            this->gps_read_flag = false;
            //check backward direction as well
            bool match = ((HEADING_STABILITY_THRESHOLD > gps_ins_diff) || ((180 - HEADING_STABILITY_THRESHOLD) < gps_ins_diff));
            if (!match)
            {
                this->gps_ins_mismatch_streak++;
            }
            else
            {
                this->gps_ins_mismatch_streak = 0;
            }
        }
        else
        {
            // what to do in this case?
            this->gps_ins_mismatch_streak = 0;
        }
    }

    // If hdg has been read recently update mismatch streak accordingly
    if (this->hdg_read_flag)
    {
        this->hdg_read_flag = false;
        if (this->is_baseline_correct())
        {

            bool match = (HEADING_STABILITY_THRESHOLD > hdg_ins_diff);
            if (!match)
            {
                this->hdg_ins_mismatch_streak++;
            }
            else
            {
                this->hdg_ins_mismatch_streak = 0;
            }
        }
        else
        {
            // what to do in this case?
            this->hdg_ins_mismatch_streak = 0;
        }
    }

    //reset streak to 0 when INS is not initialized
    if (ins_status < 2)
    {
        this->gps_ins_mismatch_streak = 0;
        this->hdg_ins_mismatch_streak = 0;
    }
}

void health_message::add_gps_message(double *gps_msg)
{
    this->gps_heading = gps_msg[7];
    if (this->gps_heading > 180)
        this->gps_heading -= 360;
    this->gps_hacc = gps_msg[8];
    this->gps_heading_acc = gps_msg[14];
    this->rtk_status = gps_msg[15];

    // Only schedule the gps-vs-ins heading comparison when moving fast
    // enough for course-over-ground to be meaningful; below
    // GPS_HEADING_MIN_SPEED the streak counter must not accumulate.
    if (gps_msg[6] >= GPS_HEADING_MIN_SPEED)
    {
        this->gps_read_flag = true;
    }
}

void health_message::add_hdg_message(double *hdg_msg)
{
    // Ignore epochs where the receiver itself marks the heading invalid;
    // comparing INS heading against an invalid APHDG would accumulate
    // spurious mismatch streaks.
    uint16_t flags = static_cast<uint16_t>(hdg_msg[9]);
    uint16_t required = HDG_FLAG_GNSS_FIX_OK | HDG_FLAG_REL_POS_VALID |
                        HDG_FLAG_HEADING_VALID;
    if ((flags & required) != required)
        return;

    this->hdg_baseline = hdg_msg[5];
    this->hdg_heading = hdg_msg[6];
    this->hdg_heading_acc = hdg_msg[8];
    if (this->hdg_heading > 180)
        this->hdg_heading -= 360;

    // Tell the system to check hdg vs ins at next ins message
    this->hdg_read_flag = true;
}

void health_message::set_baseline(double baseline)
{
    this->configured_baseline = baseline;
}

void health_message::get_current_diff(double *gps_diff_out, double *hdg_diff_out)
{
    double hdg_ins_diff = std::fabs(this->ins_heading - this->hdg_heading);
    double gps_ins_diff = std::fabs(this->ins_heading - this->gps_heading);

    if (hdg_ins_diff > 180)
        hdg_ins_diff = 360 - hdg_ins_diff;
    if (gps_ins_diff > 180)
        gps_ins_diff = 360 - gps_ins_diff;

    *gps_diff_out = gps_ins_diff;
    *hdg_diff_out = hdg_ins_diff;
    return;
}

bool health_message::is_baseline_correct()
{
    // With no configured baseline the comparison would always fail and
    // permanently disable the HDG-vs-INS heading check, so skip it.
    if (this->configured_baseline <= 0.0)
        return (std::fabs(this->wz_fog_moving_average) < 5);

    return (std::fabs(this->hdg_baseline - this->configured_baseline) < BASELINE_ACC_THRESHOLD) && (std::fabs(this->wz_fog_moving_average) < 5);
}

bool health_message::is_single_antenna_heading_valid()
{
    return (this->gps_heading_acc < GPS_HEADING_ACC_GOOD_THRESHOLD) && (std::fabs(this->wz_fog_moving_average) < 5);
}

bool health_message::has_rtk_fix() const
{
    return (this->rtk_status >= 2);
}

bool health_message::has_gyro_discrepancy() const
{
    bool ret_val = false;

    // if moving average not ready yet, return false
    if (this->buffer_full)
    {
        // Above the optical gyro's range the channels legitimately diverge
        // (OG_WZ rails while the MEMS keeps tracking) — not a fault.
        if (fabs(this->wz_mems_moving_average) > FOG_SATURATION_GUARD_DPS)
        {
            return false;
        }

        // An exactly-zero FOG window means the optical gyro is disabled
        // (APCFG fog off), not stuck — nothing to compare against.
        if (this->wz_fog_moving_average == 0.0 && this->wz_fog_std_dev == 0.0)
        {
            return false;
        }

        double diff = fabs(this->wz_mems_moving_average - this->wz_fog_moving_average);
        if (diff > GYRO_DISCREPANCY_THRESHOLD)
        {
            ret_val = true;
        }

        if (wz_mems_std_dev < STUCK_STD_FLOOR_MEMS ||
            wz_fog_std_dev < STUCK_STD_FLOOR_FOG)
        {
            ret_val = true;
        }
    }

    return ret_val;
}

bool health_message::has_good_gps_accuracy() const
{
    return (this->gps_hacc < GOOD_GPS_ACC_TRESHOLD);
}

uint8_t health_message::get_position_status() const 
{
    uint8_t ret_val = GPS_ACC_POOR;

    if (this->has_rtk_fix())
    {
        ret_val = CM_LEVEL_ACCURACY;
    }
    else if (this->has_good_gps_accuracy())
    {
        ret_val = SUB_METER_LEVEL_ACCURACY;
    }
    else
    {
        ret_val = GPS_ACC_POOR;
    }

    return ret_val;
}

uint8_t health_message::get_heading_status() const 
{
    uint8_t ret_val = HEADING_STABLE;

    if (HEADING_MISMATCH_COUNT_TH <= this->gps_ins_mismatch_streak)
    {
        ret_val = HEADING_UNSTABLE;
    }

    if (HEADING_MISMATCH_COUNT_TH <= this->hdg_ins_mismatch_streak)
    {
        ret_val = HEADING_UNSTABLE;
    }

    return ret_val;
}

uint8_t health_message::get_gyro_status() const 
{
    uint8_t ret_val = GYRO_BAD;

    if (!this->has_gyro_discrepancy())
    {
        ret_val = GYRO_GOOD;
    }

    return ret_val;
}

const char* health_message::get_csv_header()
{
    return "cur_imu_time,lat,lon,alt,mems_avg,fog_avg,mems_std,fog_std,ins_heading,gps_heading,hdg_heading,ins_gps_heading_diff,ins_hdg_heading_diff,gps_streak,hdg_streak,hdg_baseline,gps_accuracy,gps_heading_acc,hdg_heading_acc,rtk,has_fix,gyro_disc,good_gps_acc,position_status,heading_status,gyro_status\n";
}

void health_message::get_csv_line(double *llh, char *buffer, int len)
{
    double ins_gps_heading_diff, ins_hdg_heading_diff;
    this->get_current_diff(&ins_gps_heading_diff, &ins_hdg_heading_diff);

    snprintf(buffer, len, "%10.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%i,%i,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%i,%i,%i,%i,%i,%i,\n",
                                                                                                    this->cur_imu_time,
                                                                                                    llh[0], llh[1], llh[2],
                                                                                                    this->wz_mems_moving_average, this->wz_fog_moving_average, 
                                                                                                    this->wz_mems_std_dev, this->wz_fog_std_dev,
                                                                                                    this->ins_heading, this->gps_heading, this->hdg_heading, 
                                                                                                    ins_gps_heading_diff, ins_hdg_heading_diff,
                                                                                                    this->gps_ins_mismatch_streak, this->hdg_ins_mismatch_streak,
                                                                                                    this->hdg_baseline, this->gps_hacc, this->gps_heading_acc, this->hdg_heading_acc, this->rtk_status,
                                                                                                    this->has_rtk_fix(), this->has_gyro_discrepancy(),this->has_good_gps_accuracy(),
                                                                                                    this->get_position_status(), this->get_heading_status(), this->get_gyro_status());

}