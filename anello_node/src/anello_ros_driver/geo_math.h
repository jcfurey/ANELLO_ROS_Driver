/********************************************************************************
 * File Name:   geo_math.h
 * Description: Header-only geodesy / frame-conversion helpers.
 *
 * License:     MIT License
 ********************************************************************************/

#ifndef GEO_MATH_H
#define GEO_MATH_H

#include <cmath>

#include "tf2/LinearMath/Quaternion.hpp"

/* NED rpy -> ENU rpy:  roll' = roll, pitch' = -pitch, yaw' = pi/2 - heading */
inline tf2::Quaternion ned_rpy_deg_to_enu_quat(double roll_deg, double pitch_deg,
                                               double heading_deg)
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kDeg2Rad = kPi / 180.0;
    constexpr double kHalfPi = kPi / 2.0;

    tf2::Quaternion q;
    q.setRPY(roll_deg * kDeg2Rad,
             -pitch_deg * kDeg2Rad,
             kHalfPi - heading_deg * kDeg2Rad);
    return q;
}

/* WGS-84 local tangent plane scale at a given latitude: meters per degree
 * of latitude (meridional radius) and longitude (prime-vertical radius).
 * The spherical approximation (a·π/180 on both axes) is off by up to 0.7%
 * per axis — meters of systematic error within a km. */
inline void wgs84_meters_per_degree(double lat_deg, double &m_per_deg_lat,
                                    double &m_per_deg_lon)
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kDeg2Rad = kPi / 180.0;
    constexpr double kWgs84A = 6378137.0;
    constexpr double kWgs84E2 = 6.69437999014e-3;

    const double slat = std::sin(lat_deg * kDeg2Rad);
    const double denom = 1.0 - kWgs84E2 * slat * slat;
    m_per_deg_lat = kDeg2Rad * kWgs84A * (1.0 - kWgs84E2) /
                    (denom * std::sqrt(denom));
    m_per_deg_lon = kDeg2Rad * (kWgs84A / std::sqrt(denom)) *
                    std::cos(lat_deg * kDeg2Rad);
}

/* Wrap a longitude delta into [-180, 180] so an anchor near the
 * antimeridian doesn't produce a ±360° jump when crossing it. */
inline double wrap_dlon_deg(double dlon)
{
    if (dlon > 180.0)
        dlon -= 360.0;
    else if (dlon < -180.0)
        dlon += 360.0;
    return dlon;
}

#endif
