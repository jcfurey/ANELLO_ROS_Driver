// Tests for the header-only geodesy helpers: NED->ENU attitude
// conversion, the WGS-84 meters-per-degree scale, and antimeridian
// longitude-delta wrapping.

#include <gtest/gtest.h>

#include <cmath>

#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"

#include "../src/anello_ros_driver/geo_math.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

void get_rpy(const tf2::Quaternion &q, double &r, double &p, double &y)
{
    tf2::Matrix3x3(q).getRPY(r, p, y);
}
}  // namespace

TEST(NedToEnuQuat, NorthHeadingIsEnuYawHalfPi)
{
    // NED heading 0 = pointing north. In ENU (x=east, y=north) that is
    // yaw +90 deg — publishing raw NED yaw here would point consumers
    // (e.g. robot_localization) 90 deg off.
    tf2::Quaternion q = ned_rpy_deg_to_enu_quat(0.0, 0.0, 0.0);
    double r, p, y;
    get_rpy(q, r, p, y);
    EXPECT_NEAR(r, 0.0, 1e-9);
    EXPECT_NEAR(p, 0.0, 1e-9);
    EXPECT_NEAR(y, kPi / 2.0, 1e-9);

    // Rotating body-x (forward) must land on ENU north (+Y).
    tf2::Vector3 fwd = tf2::quatRotate(q, tf2::Vector3(1, 0, 0));
    EXPECT_NEAR(fwd.x(), 0.0, 1e-9);
    EXPECT_NEAR(fwd.y(), 1.0, 1e-9);
    EXPECT_NEAR(fwd.z(), 0.0, 1e-9);
}

TEST(NedToEnuQuat, EastHeadingIsEnuYawZero)
{
    tf2::Quaternion q = ned_rpy_deg_to_enu_quat(0.0, 0.0, 90.0);
    double r, p, y;
    get_rpy(q, r, p, y);
    EXPECT_NEAR(y, 0.0, 1e-9);

    // Forward points at ENU east (+X).
    tf2::Vector3 fwd = tf2::quatRotate(q, tf2::Vector3(1, 0, 0));
    EXPECT_NEAR(fwd.x(), 1.0, 1e-9);
    EXPECT_NEAR(fwd.y(), 0.0, 1e-9);
    EXPECT_NEAR(fwd.z(), 0.0, 1e-9);
}

TEST(NedToEnuQuat, PurePitchFlipsSign)
{
    // NED pitch +10 is nose-up; in ENU/FLU the pitch convention flips
    // (ENU pitch = -NED pitch). Getting this sign wrong inverts climbs
    // and descents for every downstream consumer.
    tf2::Quaternion q = ned_rpy_deg_to_enu_quat(0.0, 10.0, 0.0);
    double r, p, y;
    get_rpy(q, r, p, y);
    EXPECT_NEAR(r, 0.0, 1e-9);
    EXPECT_NEAR(p, -10.0 * kDeg2Rad, 1e-9);
    EXPECT_NEAR(y, kPi / 2.0, 1e-9);

    // Physical check: nose-up must raise body-forward above the
    // horizon in ENU, where +Z is up.
    tf2::Vector3 fwd = tf2::quatRotate(q, tf2::Vector3(1, 0, 0));
    EXPECT_NEAR(fwd.z(), std::sin(10.0 * kDeg2Rad), 1e-9);
}

TEST(NedToEnuQuat, PureRollKeepsSign)
{
    // Roll is about the shared forward axis, so its sign carries over
    // unchanged (ENU roll = NED roll).
    tf2::Quaternion q = ned_rpy_deg_to_enu_quat(10.0, 0.0, 0.0);
    double r, p, y;
    get_rpy(q, r, p, y);
    EXPECT_NEAR(r, 10.0 * kDeg2Rad, 1e-9);
    EXPECT_NEAR(p, 0.0, 1e-9);
    EXPECT_NEAR(y, kPi / 2.0, 1e-9);

    // Physical check: facing north, NED roll +10 drops the right side
    // (east), so body-up leans east (+X in ENU).
    tf2::Vector3 up = tf2::quatRotate(q, tf2::Vector3(0, 0, 1));
    EXPECT_NEAR(up.x(), std::sin(10.0 * kDeg2Rad), 1e-9);
    EXPECT_NEAR(up.z(), std::cos(10.0 * kDeg2Rad), 1e-9);
}

TEST(Wgs84MetersPerDegree, EquatorAndMidLatitudeScales)
{
    // Textbook WGS-84 values; the spherical approximation would be off
    // by ~700 m/deg at these latitudes, well outside the tolerance.
    double m_lat = 0.0, m_lon = 0.0;
    wgs84_meters_per_degree(0.0, m_lat, m_lon);
    EXPECT_NEAR(m_lat, 110574.0, 2.0);
    EXPECT_NEAR(m_lon, 111319.0, 2.0);

    wgs84_meters_per_degree(45.0, m_lat, m_lon);
    EXPECT_NEAR(m_lat, 111132.0, 2.0);
    EXPECT_NEAR(m_lon, 78847.0, 2.0);
}

TEST(WrapDlonDeg, SmallDeltasPassThrough)
{
    EXPECT_DOUBLE_EQ(wrap_dlon_deg(0.2), 0.2);
    EXPECT_DOUBLE_EQ(wrap_dlon_deg(-0.2), -0.2);
    EXPECT_DOUBLE_EQ(wrap_dlon_deg(0.0), 0.0);
}

TEST(WrapDlonDeg, AntimeridianCrossingWraps)
{
    // Anchor at lon +179.9 with a fix at -179.9 gives a raw delta of
    // -359.8: physically the vehicle moved +0.2 deg east, not 40000 km
    // west. Without wrapping the local-odometry output jumps by the
    // full Earth circumference when crossing the antimeridian.
    EXPECT_NEAR(wrap_dlon_deg(-359.8), 0.2, 1e-9);
    EXPECT_NEAR(wrap_dlon_deg(359.8), -0.2, 1e-9);
}

TEST(WrapDlonDeg, BoundaryIsInclusive)
{
    // The implementation wraps only strictly outside [-180, 180]:
    // exactly +/-180 passes through unchanged (both describe the same
    // meridian, so either representation is valid).
    EXPECT_DOUBLE_EQ(wrap_dlon_deg(180.0), 180.0);
    EXPECT_DOUBLE_EQ(wrap_dlon_deg(-180.0), -180.0);
    EXPECT_NEAR(wrap_dlon_deg(180.5), -179.5, 1e-9);
    EXPECT_NEAR(wrap_dlon_deg(-180.5), 179.5, 1e-9);
}
