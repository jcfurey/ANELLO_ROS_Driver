#ifndef ANELLO_NAVIGATION_MATH_H
#define ANELLO_NAVIGATION_MATH_H
#include <cmath>
#include "tf2/LinearMath/Matrix3x3.hpp"
namespace anello {
constexpr double radians = 3.14159265358979323846 / 180.0;
inline tf2::Vector3 geodetic_ecef(double lat, double lon, double alt) {
    constexpr double a=6378137.0, e2=6.6943799901413165e-3;
    const double s=std::sin(lat*radians), c=std::cos(lat*radians);
    const double n=a/std::sqrt(1-e2*s*s);
    return {(n+alt)*c*std::cos(lon*radians), (n+alt)*c*std::sin(lon*radians),
        (n*(1-e2)+alt)*s};
}
inline tf2::Matrix3x3 ecef_to_enu(double lat, double lon) {
    const double s=std::sin(lat*radians), c=std::cos(lat*radians);
    const double sl=std::sin(lon*radians), cl=std::cos(lon*radians);
    return {-sl,cl,0, -s*cl,-s*sl,c, c*cl,c*sl,s};
}
class LocalCartesian {
public:
    bool initialized() const { return initialized_; }
    void set_origin(double lat, double lon, double alt) {
        origin_=geodetic_ecef(lat,lon,alt); rotation_=ecef_to_enu(lat,lon); initialized_=true;
    }
    tf2::Vector3 position(double lat, double lon, double alt) const {
        return rotation_*(geodetic_ecef(lat,lon,alt)-origin_);
    }
    // Current geodetic ENU axes -> the fixed ENU axes at the anchor.
    tf2::Matrix3x3 rotation(double lat, double lon) const {
        return rotation_*ecef_to_enu(lat,lon).transpose();
    }
private:
    bool initialized_=false;
    tf2::Vector3 origin_{0,0,0};
    tf2::Matrix3x3 rotation_{1,0,0,0,1,0,0,0,1};
};
// Euler-angle covariance is not a fixed-axis small-angle covariance away
// from level. Jacobian columns are the world axes of roll, pitch, yaw.
inline tf2::Matrix3x3 euler_covariance_to_fixed(const tf2::Matrix3x3 &cov,
                                              double pitch, double yaw) {
    const double c=std::cos(pitch), s=std::sin(pitch);
    const double cy=std::cos(yaw), sy=std::sin(yaw);
    const tf2::Matrix3x3 j(cy*c,-sy,0, sy*c,cy,0, -s,0,1);
    return j*cov*j.transpose();
}
}  // namespace anello
#endif
