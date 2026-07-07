/********************************************************************************
 * File Name:   monotonic_stamp.h
 * Description: Strictly-increasing stamp sequencer for TF-consistent outputs.
 *
 * License:     MIT License
 ********************************************************************************/

#ifndef MONOTONIC_STAMP_H
#define MONOTONIC_STAMP_H

#include "rclcpp/rclcpp.hpp"

/* TF requires strictly-increasing stamps per frame. In 'arrival' mode
 * every message decoded from one port read shares the same host stamp,
 * so two INS frames in a single read would broadcast the TF frame pair
 * twice with an identical stamp (tf2 drops the second as
 * TF_REPEATED_DATA); in 'mcu' mode a settling clock offset can step a
 * stamp backward (TF_OLD_DATA). next() nudges a non-increasing stamp
 * forward by 1 ns so every consumer sharing it stays monotonic. */
class MonotonicStamp
{
public:
    rclcpp::Time next(rclcpp::Time stamp)
    {
        if (valid_ && stamp <= last_)
            last_ = last_ + rclcpp::Duration(0, 1);
        else
            last_ = stamp;  // first call also adopts the clock type
        valid_ = true;
        return last_;
    }

private:
    rclcpp::Time last_;
    bool valid_ = false;
};

#endif
