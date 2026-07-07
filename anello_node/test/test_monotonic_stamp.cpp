// Tests for the strictly-increasing stamp sequencer that protects TF
// consumers from TF_REPEATED_DATA (equal stamps) and TF_OLD_DATA
// (backwards stamps) drops. rclcpp::Time is a value type, so no
// rclcpp::init() is needed.

#include <gtest/gtest.h>

#include "../src/anello_ros_driver/monotonic_stamp.h"

TEST(MonotonicStamp, IncreasingStampsPassThroughUnchanged)
{
    // The sequencer must be a no-op on healthy input: any nudge here
    // would add artificial skew to every published stamp.
    MonotonicStamp ms;
    EXPECT_EQ(ms.next(rclcpp::Time(100, 0)), rclcpp::Time(100, 0));
    EXPECT_EQ(ms.next(rclcpp::Time(100, 500)), rclcpp::Time(100, 500));
    EXPECT_EQ(ms.next(rclcpp::Time(101, 0)), rclcpp::Time(101, 0));
}

TEST(MonotonicStamp, EqualStampIsNudgedForwardOneNs)
{
    // Two frames decoded from one port read share the same arrival
    // stamp; tf2 drops the second TF broadcast as TF_REPEATED_DATA
    // unless it is nudged forward.
    MonotonicStamp ms;
    EXPECT_EQ(ms.next(rclcpp::Time(100, 0)), rclcpp::Time(100, 0));
    EXPECT_EQ(ms.next(rclcpp::Time(100, 0)), rclcpp::Time(100, 1));
}

TEST(MonotonicStamp, BackwardsStampContinuesFromPreviousNotInput)
{
    // In 'mcu' mode a settling clock offset can step a stamp backwards;
    // the output must continue from the previous stamp + 1 ns, not
    // adopt the backwards value (which tf2 would reject as TF_OLD_DATA).
    MonotonicStamp ms;
    EXPECT_EQ(ms.next(rclcpp::Time(100, 0)), rclcpp::Time(100, 0));
    EXPECT_EQ(ms.next(rclcpp::Time(99, 0)), rclcpp::Time(100, 1));
    // And keeps counting up from there while the input stays behind.
    EXPECT_EQ(ms.next(rclcpp::Time(99, 500)), rclcpp::Time(100, 2));
}

TEST(MonotonicStamp, IdenticalRunYieldsStrictlyIncreasingOutputs)
{
    // A burst of N frames with one shared stamp must come out strictly
    // increasing so every consumer of the shared sequencer stays
    // monotonic, not just the first two.
    MonotonicStamp ms;
    const rclcpp::Time in(50, 0);
    rclcpp::Time prev = ms.next(in);
    EXPECT_EQ(prev, in);
    for (int i = 1; i <= 10; ++i) {
        const rclcpp::Time out = ms.next(in);
        EXPECT_GT(out, prev) << "output not strictly increasing at " << i;
        prev = out;
    }
    // Each nudge is exactly 1 ns, so drift stays negligible.
    EXPECT_EQ(prev, rclcpp::Time(50, 10));
}
