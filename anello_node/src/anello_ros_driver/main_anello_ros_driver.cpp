/********************************************************************************
 * File Name:   main_anello_ros_driver.cpp
 * Description: ROS2 driver node for ANELLO Photonics GNSS/INS devices.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 ********************************************************************************/

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <vector>
#include <unistd.h>
#include <iomanip>
#include <memory>
#include <cmath>
#include <stdexcept>
#include <map>
#include <limits>
#include <atomic>
#include <filesystem>
#include <locale>
#include "rclcpp/version.h"
#include "navigation_math.h"
#include "sample_state.h"
#include "device_input.h"
#include "messaging/protocol_decoder.h"
#include "anello_interfaces/msg/apahrs.hpp"

#include "main_anello_ros_driver.h"
#include "comm/anello_config_port.h"
#include "comm/anello_data_port.h"
#include "messaging/publisher_types.h"

#include "anello_interfaces/msg/apimu.hpp"
#include "anello_interfaces/msg/apim1.hpp"
#include "anello_interfaces/msg/apins.hpp"
#include "anello_interfaces/msg/apgps.hpp"
#include "anello_interfaces/msg/aphdg.hpp"
#include "anello_interfaces/msg/aphealth.hpp"
#include "anello_interfaces/msg/apcov.hpp"
#include "anello_interfaces/msg/apodo.hpp"
#include "anello_interfaces/srv/cmd_and_rsp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "bit_tools.h"
#include "clock_translator.h"
#include "version.h"
#include "messaging/rtcm_decoder.h"
#include "messaging/message_publisher.h"
#include "messaging/health_message.h"

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kGAccel = 9.80665;
static constexpr double kDeg2Rad = kPi / 180.0;
static constexpr double kDeg2RadSq = kDeg2Rad * kDeg2Rad;
static constexpr double kHalfPi = kPi / 2.0;

/* Frame conventions
 *
 * The device reports attitude in NED (heading clockwise from north) and
 * body-frame vectors in FRD (x forward, y right, z down). The custom
 * "anello/..." topics carry these values unchanged, in the device-native
 * convention. Only the standard ROS interfaces (imu/data, gps/fix, TF)
 * are converted here to REP-103 ENU / FLU.
 *
 * NED rpy -> ENU rpy:  roll' = roll, pitch' = -pitch, yaw' = pi/2 - heading
 * FRD vec -> FLU vec:  x' = x, y' = -y, z' = -z
 */
static tf2::Quaternion ned_rpy_deg_to_enu_quat(double roll_deg, double pitch_deg,
                                               double heading_deg)
{
    tf2::Quaternion q;
    q.setRPY(roll_deg * kDeg2Rad,
             -pitch_deg * kDeg2Rad,
             kHalfPi - heading_deg * kDeg2Rad);
    return q;
}

/* Sign pattern for rotating a (roll, pitch, yaw) covariance through the
 * NED->ENU axis map D = diag(1, -1, -1): C' = D * C * D. */
static constexpr double kEnuCovSign[9] = {
     1.0, -1.0, -1.0,
    -1.0,  1.0,  1.0,
    -1.0,  1.0,  1.0,
};



namespace anello
{

struct ImuCache
{
    double ax = 0.0, ay = 0.0, az = 0.0;
    double wx = 0.0, wy = 0.0, wz = 0.0;
    double wz_fog = 0.0;
};

struct CovCache
{
    double orient[9] = {};   // 3x3 row-major: roll, pitch, heading
    double pos[9] = {};      // 3x3 row-major: lat, lon, alt
    double vel[9] = {};      // 3x3 row-major NED: vn, ve, vd
};

struct ReadBuffer
{
    int n_used = 0;
    int nbytes = 0;
    char buff[MAX_BUF_LEN] = {};
    rclcpp::Time stamp;  // host time captured at the port read
};

/* Sliding-window message/error rates for diagnostics: lifetime totals
 * plus a trailing window, so a device that stops streaming (or a link
 * that degrades) is visible on /diagnostics instead of showing the last
 * known health flags as "nominal". Single-threaded access only. */
struct RateMonitor
{
    static constexpr double kWindowSeconds = 5.0;

    uint64_t total_ok = 0;
    uint64_t total_checksum_fail = 0;
    uint64_t total_parse_fail = 0;

    void add_ok()            { total_ok++;            push(recent_ok_); }
    void add_checksum_fail() { total_checksum_fail++; push(recent_err_); }
    void add_parse_fail()    { total_parse_fail++;    push(recent_err_); }

    double rate_hz()
    {
        trim(recent_ok_);
        return static_cast<double>(recent_ok_.size()) / kWindowSeconds;
    }
    double error_percent()
    {
        trim(recent_ok_);
        trim(recent_err_);
        const size_t total = recent_ok_.size() + recent_err_.size();
        return total > 0
            ? 100.0 * static_cast<double>(recent_err_.size()) / static_cast<double>(total)
            : 0.0;
    }

private:
    std::deque<std::chrono::steady_clock::time_point> recent_ok_;
    std::deque<std::chrono::steady_clock::time_point> recent_err_;

    static void trim(std::deque<std::chrono::steady_clock::time_point> &q)
    {
        const auto cutoff = std::chrono::steady_clock::now() -
            std::chrono::milliseconds(static_cast<int64_t>(kWindowSeconds * 1000));
        while (!q.empty() && q.front() < cutoff)
            q.pop_front();
    }
    static void push(std::deque<std::chrono::steady_clock::time_point> &q)
    {
        trim(q);
        q.push_back(std::chrono::steady_clock::now());
    }
};

class AnelloRosDriver : public rclcpp::Node
{
public:
    explicit AnelloRosDriver(const rclcpp::NodeOptions &options)
        : Node("anello_ros_driver", options)
    {
        declare_all_parameters();
        read_parameters();
        // The send_cmd service and the APODO subscription both drive the
        // UART config port and can block up to ~500 ms waiting on the
        // device. Put them in their own mutually-exclusive group so, under
        // the MultiThreadedExecutor in main(), command waits can run alongside
        // data polling/publication. The default group owns decode/health state;
        // RTCM forwarding has another group. Shared serial handles are guarded
        // by locks/generations, and transmission counters are atomic.
        config_cb_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        setup_ports();
        setup_publishers();
        setup_subscribers();
        setup_services();
        setup_tf_broadcaster();
        setup_diagnostics();
        setup_timers();

        RCLCPP_INFO(get_logger(), "ANELLO ROS2 driver initialized (v%d.%d.%d)",
                    MAJOR_VERSION, MINOR_VERSION, PATCH_VERSION);
    }

    ~AnelloRosDriver() override = default;

private:
    // ── Parameter declaration ──────────────────────────────────────────
    void declare_all_parameters()
    {
        // Every parameter is read once in read_parameters() and never
        // re-read (there is no on_set_parameters callback), so mark them
        // read_only: a runtime `ros2 param set` then fails loudly instead
        // of silently having no effect.
        auto d = [](const std::string &desc) {
            rcl_interfaces::msg::ParameterDescriptor pd;
            pd.description = desc;
            pd.read_only = true;
            return pd;
        };

        declare_parameter("com_type", "UART",
            d("Communication type: UART or ETH"));
        declare_parameter("uart_data_port", "AUTO",
            d("UART data port path or AUTO for auto-detection. For a fixed "
              "path prefer a /dev/serial/by-id/ symlink: it survives the "
              "USB re-enumeration a unit power-cycle causes, so the "
              "driver's automatic reopen finds the device again"));
        declare_parameter("uart_config_port", "AUTO",
            d("UART config port path, AUTO, or OFF"));
        declare_parameter("baud_rate", 230400,
            d("Serial baud rate (230400 or 921600)"));
        declare_parameter("remote_ip", "192.168.1.111",
            d("Remote IP for ethernet mode"));
        declare_parameter("local_data_port", 1111,
            d("Local UDP port for data (ethernet mode)"));
        declare_parameter("local_config_port", 2222,
            d("Local UDP port for config (ethernet mode)"));
        declare_parameter("local_odometer_port", 3333,
            d("Local UDP port for odometer (ethernet mode)"));

        declare_parameter("frame_id.imu", "ins_link",
            d("Frame ID for IMU messages"));
        declare_parameter("frame_id.ins", "ins_link",
            d("Frame ID for INS messages"));
        declare_parameter("frame_id.gnss", "gnss_link",
            d("Frame ID for GNSS messages"));
        declare_parameter("frame_id.hdg", "gnss_link",
            d("Frame ID for dual-antenna heading messages"));

        declare_parameter("publish_tf", false,
            d("Publish the tf_parent_frame -> tf_child_frame transform"));
        declare_parameter("tf_parent_frame", "anello_local",
            d("Fixed first-fix ENU frame for the globally corrected INS solution"));
        declare_parameter("tf_child_frame", "ins_link",
            d("Must equal frame_id.ins: the firmware INS output point and axes"));
        declare_parameter("frame_id.gnss2", "gnss2_link", d("Secondary antenna frame"));
        declare_parameter("gps_utc_leap_seconds", 18, d("GPS minus UTC seconds for GGA; update from IERS announcements"));
        declare_parameter("gnss_service_mask", 0, d("Configured NavSatStatus constellation bitmask; 0 unknown"));
        declare_parameter("expected_streams", std::vector<std::string>{"imu", "ins", "gps"},
            d("Streams required for healthy diagnostics: imu, ins, gps, gps2, heading, covariance, ahrs"));
        declare_parameter("stream_timeout", 2.0, d("Maximum steady-clock stream silence in seconds"));
        declare_parameter("imu_max_age", 0.05, d("Maximum acquisition and arrival age for combining IMU and INS"));
        declare_parameter("covariance.max_age", 0.2, d("Maximum acquisition and arrival age of APCOV"));
        declare_parameter("covariance.device_convention", "unknown",
            d("unknown, or verified_m2_deg2_euler: position N/E/up in m², velocity NED in (m/s)², aerospace Euler attitude in deg²"));
        declare_parameter("covariance.unknown_variance", 1e6,
            d("Conservative odometry variance when uncertainty or a twist field is unavailable"));
        declare_parameter("imu_output_rate_hz", 100.0,
            d("Rate used to scale datasheet noise estimates; calibrate covariance for actual bandwidth"));
        declare_parameter("accel_sign_check_upright", false,
            d("Enable gravity-sign warning only when the IMU is known to be mounted upright at startup"));
        declare_parameter("publish_custom_messages", true, d("Publish device-native anello message topics"));
        declare_parameter("command_mode", "read_only",
            d("read_only permits queries/echo; unrestricted explicitly permits device configuration/reset commands"));
        declare_parameter("rtcm.max_bytes_per_second", 8192.0,
            d("RTCM admission budget, with a 4096-byte burst; UART is also capped at half the configured 8N1 byte rate"));
        declare_parameter("rtcm.max_frames_per_second", 100.0,
            d("RTCM frame admission rate, with a 16-frame burst; prevents floods of tiny frames"));
        declare_parameter("odometer.max_speed_mps", 100.0,
            d("Reject odometer speeds outside this absolute limit; firmware odo units must be m/s"));
        declare_parameter("odometer.max_rate_hz", 50.0,
            d("Maximum odometer send rate; excess samples are dropped without delayed replay"));

        declare_parameter("poll_interval_ms", 5,
            d("Main loop polling interval in milliseconds"));

        declare_parameter("timestamp_source", "mcu",
            d("Header stamp source: 'mcu' (default) = device MCU time "
              "translated to host time with a minimum-offset filter, "
              "eliminating serial/OS arrival jitter from inter-message "
              "timing (warms up on arrival stamps first); 'arrival' = host "
              "time captured once per port read, shared by every message in "
              "that read"));

        declare_parameter("heading_baseline", 0.0,
            d("Dual-antenna baseline length in meters, used to validate the "
              "APHDG heading in the health monitor (0.0 = skip the check)"));

        // The at-rest accelerometer sign convention is not stated in the
        // public manual. The FLU conversion below assumes the device
        // reports specific force in FRD (at rest: AZ = -1 g), but the
        // manual's example APIMU capture shows AZ = +1 g upright, which
        // would make every published axis inverted. Bench check: with the
        // vehicle stationary, imu/data linear_acceleration.z must read
        // +9.8; if it reads -9.8, set this parameter true. check_accel_sign()
        // performs this check automatically at startup and warns once if a
        // stationary, level unit is publishing inverted gravity.
        declare_parameter("flip_accel_sign", false,
            d("Negate all linear acceleration axes in imu/data and "
              "imu/data_raw. Use when a stationary unit reports -9.8 "
              "instead of +9.8 on linear_acceleration.z."));

        declare_parameter("use_fog_wz", true,
            d("Use the optical gyro (OG_WZ) for the z angular rate in the "
              "standard imu/data and imu/data_raw messages instead of the "
              "MEMS WZ. Set false if the FOG is disabled on the unit "
              "(APCFG fog off)."));

        // Defaults derived from the ANELLO GNSS INS datasheet noise specs at
        // 100 Hz: MEMS gyro ARW 0.3 deg/sqrt(hr), optical Z gyro ARW
        // 0.05 deg/sqrt(hr), accelerometer VRW 0.03 m/s/sqrt(hr).
        declare_parameter("covariance.angular_velocity",
            std::vector<double>{},
            d("Diagonal angular velocity covariance [x, y, z] in (rad/s)^2 "
              "for imu/data and imu/data_raw (REP-145 parameter override)"));
        declare_parameter("covariance.linear_acceleration",
            std::vector<double>{},
            d("Diagonal linear acceleration covariance [x, y, z] in "
              "(m/s^2)^2 for imu/data and imu/data_raw"));
    }

    void read_parameters()
    {
        std::string com_type = get_parameter("com_type").as_string();
        if (com_type == "ETH")
            config_.type = ETH;
        else if (com_type == "UART")
            config_.type = UART;
        else throw std::invalid_argument("com_type must be UART or ETH");

        const auto baud=get_parameter("baud_rate").as_int();
        if (baud!=115200 && baud!=230400 && baud!=460800 && baud!=921600)
            throw std::invalid_argument("Unsupported baud_rate");
        for (const auto name:{"local_data_port","local_config_port","local_odometer_port"}) {
            const auto port=get_parameter(name).as_int();
            if (port<=0 || port>65535) throw std::invalid_argument(std::string(name)+" must be 1..65535");
        }
        const auto baseline=get_parameter("heading_baseline").as_double();
        if (!std::isfinite(baseline) || baseline<0)
            throw std::invalid_argument("heading_baseline must be finite and nonnegative");
        config_.data_port_name = get_parameter("uart_data_port").as_string();
        config_.config_port_name = get_parameter("uart_config_port").as_string();
        if (config_.type==UART && config_.data_port_name!="AUTO" &&
            config_.config_port_name!="AUTO" && config_.config_port_name!="OFF") {
            std::error_code error;
            if (config_.data_port_name==config_.config_port_name ||
                std::filesystem::equivalent(config_.data_port_name,config_.config_port_name,error))
                throw std::invalid_argument("uart_data_port and uart_config_port must identify distinct serial devices");
        }
        config_.baud_rate = static_cast<uint32_t>(get_parameter("baud_rate").as_int());
        config_.remote_ip = get_parameter("remote_ip").as_string();
        config_.local_data_port = static_cast<int>(get_parameter("local_data_port").as_int());
        config_.local_config_port = static_cast<int>(get_parameter("local_config_port").as_int());
        config_.local_odometer_port = static_cast<int>(get_parameter("local_odometer_port").as_int());

        frame_imu_ = get_parameter("frame_id.imu").as_string();
        frame_ins_ = get_parameter("frame_id.ins").as_string();
        frame_gnss_ = get_parameter("frame_id.gnss").as_string();
        frame_gnss2_ = get_parameter("frame_id.gnss2").as_string();
        frame_hdg_ = get_parameter("frame_id.hdg").as_string();
        publish_tf_ = get_parameter("publish_tf").as_bool();
        tf_parent_ = get_parameter("tf_parent_frame").as_string();
        tf_child_ = get_parameter("tf_child_frame").as_string();
        poll_ms_ = get_parameter("poll_interval_ms").as_int();
        if (poll_ms_ < 1) {
            RCLCPP_WARN(get_logger(),
                "poll_interval_ms=%ld is invalid (0 ms busy-spins the executor, "
                "negative is rejected by the timer); clamping to 1 ms", poll_ms_);
            poll_ms_ = 1;
        }
        health_msg_.set_baseline(get_parameter("heading_baseline").as_double());
        use_fog_wz_ = get_parameter("use_fog_wz").as_bool();
        flip_accel_sign_ = get_parameter("flip_accel_sign").as_bool();

        timestamp_source_ = get_parameter("timestamp_source").as_string();
        if (timestamp_source_ != "arrival" && timestamp_source_ != "mcu") {
            RCLCPP_WARN(get_logger(),
                "Unknown timestamp_source '%s' — falling back to 'arrival'",
                timestamp_source_.c_str());
            timestamp_source_ = "arrival";
        }
        use_mcu_stamp_ = (timestamp_source_ == "mcu");

        if (tf_child_ != frame_ins_)
            throw std::invalid_argument("tf_child_frame must equal frame_id.ins; configure the firmware output point before changing that frame");
        if (tf_parent_ == "odom" || tf_parent_ == tf_child_)
            throw std::invalid_argument("tf_parent_frame must identify a distinct globally corrected frame, not REP-105 odom");
        for (const auto &frame : {frame_imu_,frame_ins_,frame_gnss_,frame_gnss2_,frame_hdg_,tf_parent_})
            if (frame.empty() || frame.front()=='/' || frame.find_first_of(" \t\r\n")!=std::string::npos)
                throw std::invalid_argument("Frame IDs must be nonempty, relative, and contain no whitespace");
        const auto leap_seconds=get_parameter("gps_utc_leap_seconds").as_int();
        if (leap_seconds<0 || leap_seconds>100) throw std::invalid_argument("gps_utc_leap_seconds must be 0..100");
        leap_seconds_=static_cast<int>(leap_seconds);
        gnss_service_mask_=get_parameter("gnss_service_mask").as_int();
        if (gnss_service_mask_<0 || gnss_service_mask_>15)
            throw std::invalid_argument("gnss_service_mask must be between 0 and 15");
        expected_streams_=get_parameter("expected_streams").as_string_array();
        for (const auto &stream: expected_streams_)
            if (stream!="imu" && stream!="ins" && stream!="gps" && stream!="gps2" &&
                stream!="heading" && stream!="covariance" && stream!="ahrs")
                throw std::invalid_argument("Unknown expected stream: "+stream);
        auto positive=[this](const char *name) {
            double value=get_parameter(name).as_double();
            if (!std::isfinite(value) || value<=0) throw std::invalid_argument(std::string(name)+" must be finite and positive");
            return value;
        };
        stream_timeout_=positive("stream_timeout"); imu_max_age_=positive("imu_max_age");
        command_mode_=get_parameter("command_mode").as_string();
        if (command_mode_!="read_only" && command_mode_!="unrestricted")
            throw std::invalid_argument("command_mode must be read_only or unrestricted");
        const double requested_rtcm_rate=positive("rtcm.max_bytes_per_second");
        if (requested_rtcm_rate>65536) throw std::invalid_argument("rtcm.max_bytes_per_second must be <=65536");
        rtcm_rate_=config_.type==UART?std::min(requested_rtcm_rate,config_.baud_rate/20.0):requested_rtcm_rate;
        rtcm_budget_=TrafficBudget(rtcm_rate_,4096);
        const double frame_rate=positive("rtcm.max_frames_per_second");
        if (frame_rate>1000) throw std::invalid_argument("rtcm.max_frames_per_second must be <=1000");
        rtcm_frame_budget_=TrafficBudget(frame_rate,16);
        odo_max_speed_=positive("odometer.max_speed_mps");
        if (odo_max_speed_>1000) throw std::invalid_argument("odometer.max_speed_mps must be <=1000");
        const double odo_rate=positive("odometer.max_rate_hz");
        if (odo_rate>100) throw std::invalid_argument("odometer.max_rate_hz must be <=100");
        odo_budget_=TrafficBudget(odo_rate,1);
        cov_max_age_=positive("covariance.max_age"); unknown_variance_=positive("covariance.unknown_variance");
        const double rate_scale=positive("imu_output_rate_hz")/100.0;
        const auto convention=get_parameter("covariance.device_convention").as_string();
        if (convention!="unknown" && convention!="verified_m2_deg2_euler")
            throw std::invalid_argument("Unsupported covariance.device_convention");
        use_device_cov_=convention=="verified_m2_deg2_euler";
        publish_custom_=get_parameter("publish_custom_messages").as_bool();
        health_msg_.set_fog_enabled(use_fog_wz_);
        auto read_cov3=[this,rate_scale](const char *name, double out[3], std::vector<double> defaults) {
            auto values=get_parameter(name).as_double_array();
            if (values.empty()) {
                values=defaults;
                for (double &v:values) v*=rate_scale;
            }
            if (values.size()!=3 || std::any_of(values.begin(),values.end(),
                [](double v){return !std::isfinite(v) || v<0;}))
                throw std::invalid_argument(std::string(name)+" needs three finite nonnegative variances, or [] for automatic defaults");
            std::copy(values.begin(),values.end(),out);
        };
        read_cov3("covariance.angular_velocity",ang_vel_cov_,{7.6e-7,7.6e-7,use_fog_wz_?2.1e-8:7.6e-7});
        read_cov3("covariance.linear_acceleration",lin_acc_cov_,{2.5e-5,2.5e-5,2.5e-5});

        RCLCPP_INFO(get_logger(), "com_type=%s baud=%u poll=%ldms",
                     com_type.c_str(), config_.baud_rate, poll_ms_);
    }


    // ── Port setup ─────────────────────────────────────────────────────
    void setup_ports()
    {
        try {
            data_port_ = std::make_unique<anello_data_port>(&config_);
            data_port_->init();
        } catch (const std::exception &e) {
            RCLCPP_FATAL(get_logger(), "Data port init failed: %s", e.what());
            throw;
        }

        // Claim the data candidate before AUTO config probing can open it.
        try {
            config_port_ = std::make_unique<anello_config_port>(&config_);
            config_port_->init();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Config port init failed: %s", e.what());
        }

        // Over ethernet the unit accepts odometer input only on its
        // dedicated channel (remote port 3); serial APODO goes to the
        // config port instead.
        if (config_.type == ETH) {
            try {
                odo_eth_port_ = std::make_unique<ethernet_interface>(
                    config_.remote_ip, 3, config_.local_odometer_port);
                odo_eth_port_->init();
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "Odometer port init failed: %s", e.what());
                odo_eth_port_.reset();
            }
        }
    }

    // ── Publishers ─────────────────────────────────────────────────────
    void setup_publishers()
    {
        auto sensor_qos = rclcpp::SensorDataQoS();

        pub_imu_ = create_publisher<anello_interfaces::msg::APIMU>("anello/imu_raw", sensor_qos);
        pub_im1_ = create_publisher<anello_interfaces::msg::APIM1>("anello/im1", sensor_qos);
        pub_ins_ = create_publisher<anello_interfaces::msg::APINS>("anello/ins", sensor_qos);
        pub_gps_ = create_publisher<anello_interfaces::msg::APGPS>("anello/gps", sensor_qos);
        pub_gp2_ = create_publisher<anello_interfaces::msg::APGPS>("anello/gps2", sensor_qos);
        pub_hdg_ = create_publisher<anello_interfaces::msg::APHDG>("anello/hdg", sensor_qos);
        pub_cov_ = create_publisher<anello_interfaces::msg::APCOV>("anello/cov", sensor_qos);
        // Reliable + transient_local (depth 1): a late-joining monitor
        // immediately latches the last published health instead of waiting
        // up to a second for the next 1 Hz tick.
        pub_health_ = create_publisher<anello_interfaces::msg::APHEALTH>(
            "anello/health", rclcpp::QoS(1).reliable().transient_local());
        pub_gga_ = create_publisher<nmea_msgs::msg::Sentence>("ntrip_client/nmea", 1);

        pub_ros_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", sensor_qos);
        pub_ros_imu_raw_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", sensor_qos);
        pub_navfix_ = create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", sensor_qos);
        pub_ins_fix_ = create_publisher<sensor_msgs::msg::NavSatFix>("ins/fix", sensor_qos);
        pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("ins/odometry", sensor_qos);
        pub_ahrs_=create_publisher<anello_interfaces::msg::APAHRS>("anello/ahrs",sensor_qos);
    }

    // ── Subscribers ────────────────────────────────────────────────────
    void setup_subscribers()
    {
        // RTCM corrections are low-rate and delivery-critical: a dropped
        // frame delays RTK reconvergence. Use RELIABLE (depth 10) so
        // transient DDS congestion or a busy executor cannot silently drop
        // corrections. The NTRIP client publishes RELIABLE to match; the
        // KEEP_LAST depth bounds memory.
        rtcm_cb_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions rtcm_options;
        rtcm_options.callback_group=rtcm_cb_group_;
        sub_rtcm_ = create_subscription<rtcm_msgs::msg::Message>(
            "ntrip_client/rtcm", rclcpp::QoS(10).reliable(),
            [this](const rtcm_msgs::msg::Message::SharedPtr msg) {
                size_t frame_count=0;
                if (!valid_rtcm_input(msg->message.data(),msg->message.size(),&frame_count)) {
                    ++tx_input_rejections_; return;
                }
                if (!rtcm_frame_budget_.take(frame_count) || !rtcm_budget_.take(msg->message.size())) {
                    ++tx_rate_drops_; return;
                }
                // Send one complete frame at a time. A maximum RTCM frame is
                // 1029 bytes, so bundles cannot cause ordinary Ethernet MTU
                // fragmentation or unnecessarily long individual UART writes.
                for (size_t offset=0;offset<msg->message.size();) {
                    const size_t length=6+((msg->message[offset+1]&3u)<<8)+msg->message[offset+2];
                    if (!data_port_ || !data_port_->write_data(
                            reinterpret_cast<const char *>(msg->message.data()+offset),length)) {
                        ++tx_failures_; break;
                    }
                    offset+=length;
                }
            },rtcm_options);

        // APODO drives the config port (serial mode), so it shares the
        // config callback group with the send_cmd service.
        rclcpp::SubscriptionOptions odo_opts;
        odo_opts.callback_group = config_cb_group_;
        sub_odo_ = create_subscription<anello_interfaces::msg::APODO>(
            "anello/odo", 1,
            [this](const anello_interfaces::msg::APODO::SharedPtr msg) {
                if (!std::isfinite(msg->odo_speed) || std::abs(msg->odo_speed)>odo_max_speed_) {
                    ++tx_input_rejections_; return;
                }
                if (!odo_budget_.take(1)) { ++tx_rate_drops_; return; }
                std::ostringstream body;
                body.imbue(std::locale::classic());
                body << "APODO," << std::fixed << std::setprecision(2) << msg->odo_speed;
                std::string body_str = body.str();
                std::string ck = compute_checksum(body_str.c_str(), body_str.length());
                std::string full = "#" + body_str + "*" + ck + "\r\n";
                bool sent=config_.type==ETH ? (odo_eth_port_ && odo_eth_port_->write_data(full.c_str(),full.size())) :
                    (config_port_ && config_port_->write_data(full.c_str(),full.size()));
                if (!sent) ++tx_failures_;
            },
            odo_opts);
    }

    // ── Services ───────────────────────────────────────────────────────
    void setup_services()
    {
        // Runs in the config callback group (see the constructor): the
        // ~500 ms device-response wait must not stall the data poll.
        srv_cmd_ = create_service<anello_interfaces::srv::CmdAndRsp>(
            "anello/send_cmd",
            [this](const std::shared_ptr<anello_interfaces::srv::CmdAndRsp::Request> req,
                   std::shared_ptr<anello_interfaces::srv::CmdAndRsp::Response> res) {
                send_command_callback(req, res);
            },
#if RCLCPP_VERSION_MAJOR < 17
            rclcpp::ServicesQoS().get_rmw_qos_profile(), config_cb_group_);
#else
            rclcpp::ServicesQoS(), config_cb_group_);
#endif
    }

    void send_command_callback(
        const std::shared_ptr<anello_interfaces::srv::CmdAndRsp::Request> req,
        std::shared_ptr<anello_interfaces::srv::CmdAndRsp::Response> res)
    {
        if (!config_port_) {
            res->response = "ERROR: config port not available";
            return;
        }
        constexpr int kMaxResp = 512;
        char read_buf[kMaxResp];
        std::string response;

        std::string body = req->command;
        if (!valid_command_body(body)) {
            ++tx_input_rejections_;
            res->response="ERROR: expected a 5..128-byte AP command body without checksum or line terminators";
            return;
        }
        if (command_mode_=="read_only" && !read_only_command(body)) {
            ++tx_input_rejections_;
            res->response="ERROR: command_mode=read_only blocks configuration, reset, and unknown commands";
            return;
        }
        if (!command_budget_.take(1)) {
            ++tx_rate_drops_;
            res->response="ERROR: command rate limit (2 per second); request was not transmitted";
            return;
        }
        const auto identifier=body.substr(0,body.find(','));
        std::string ck = compute_checksum(body.c_str(), body.size());
        std::string full = "#" + body + "*" + ck + "\r\n";

        // Drain stale bytes (a previous call's late response, unsolicited
        // APODO acks on the shared UART) so they can't be returned as
        // this command's response; bounded in case the port is streaming.
        for (int i = 0;
             i < 32 && config_port_->get_data(read_buf, kMaxResp - 1, 0) > 0;
             ++i)
        {
        }

        if (!config_port_->write_data(full.c_str(),full.size())) {
            ++tx_failures_; res->response="ERROR: command transmission failed"; return;
        }
        if (body=="APRST,0") {
            res->response="SENT: APRST,0; the device reset command has no acknowledgement";
            return;
        }

        auto start = std::chrono::steady_clock::now();
        auto timeout = std::chrono::milliseconds(500);

        // Bounded reads keep the response wait near its 500 ms budget.
        while (std::chrono::steady_clock::now() - start < timeout) {
            int n = static_cast<int>(config_port_->get_data(read_buf, kMaxResp - 1, 20));
            if (n > 0) {
                read_buf[n] = '\0';
                response.append(read_buf,n);
                auto end=response.find("\r\n");
                while (end!=std::string::npos) {
                    auto line=response.substr(0,end+2);
                    response.erase(0,end+2);
                    if ((line.rfind("#"+identifier+",",0)==0 || line.rfind("#"+identifier+"*",0)==0 ||
                         line.rfind("#APERR,",0)==0) &&
                        checksum(reinterpret_cast<const unsigned char *>(line.data()),line.size())) {
                        res->response=line; return;
                    }
                    end=response.find("\r\n");
                }
                if (response.size()>4096) response.clear();
            }
        }

        res->response = "ERROR: no valid matching response from device (timeout)";
    }


    // ── TF broadcaster ────────────────────────────────────────────────
    void setup_tf_broadcaster()
    {
        if (publish_tf_)
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    void broadcast_ins_tf(const nav_msgs::msg::Odometry &odom)
    {
        if (!tf_broadcaster_) return;
        geometry_msgs::msg::TransformStamped transform;
        transform.header=odom.header; transform.child_frame_id=odom.child_frame_id;
        transform.transform.translation.x=odom.pose.pose.position.x;
        transform.transform.translation.y=odom.pose.pose.position.y;
        transform.transform.translation.z=odom.pose.pose.position.z;
        transform.transform.rotation=odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(transform);
    }

    bool expects_stream(const std::string &name) const {
        return std::find(expected_streams_.begin(),expected_streams_.end(),name)!=expected_streams_.end();
    }
    std::string stale_streams() const {
        std::string stale;
        for (const auto &name:expected_streams_) {
            const auto it=streams_.find(name);
            if (it==streams_.end() || it->second.age()>stream_timeout_)
                stale+=(stale.empty()?"":", ")+name;
        }
        return stale;
    }

    // ── Diagnostics ───────────────────────────────────────────────────
    void setup_diagnostics()
    {
        diag_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
        diag_updater_->setHardwareID("anello_gnss_ins");

        diag_updater_->add(std::string(get_fully_qualified_name())+" Device Status", [this](diagnostic_updater::DiagnosticStatusWrapper &stat) {
            uint8_t pos = health_msg_.get_position_status();
            uint8_t hdg = health_msg_.get_heading_status();
            uint8_t gyro = health_msg_.get_gyro_status();
            const double rate_hz = rate_monitor_.rate_hz();
            const double err_pct = rate_monitor_.error_percent();

            // Data flow gates first: stale health flags must not report
            // "nominal" when the device has stopped streaming.
            if (rate_monitor_.total_ok == 0 && rate_hz <= 0.0)
                stat.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "No data received yet");
            else if (rate_hz <= 0.0)
                stat.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "No data from device");
            else if (!stale_streams().empty())
                stat.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Missing or stale streams: "+stale_streams());
            else if (expects_stream("imu") && gyro > 0)
                stat.summary(gyro==GYRO_BAD?diagnostic_updater::DiagnosticStatusWrapper::ERROR:diagnostic_updater::DiagnosticStatusWrapper::WARN, "Gyro unavailable or degraded");
            else if (err_pct >= 20.0)
                stat.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "High message error rate");
            else if ((!expects_stream("gps") || pos==0) && (!expects_stream("ins") || hdg==0))
                stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "All systems nominal");
            else
                stat.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Degraded accuracy");

            for (const auto &name:expected_streams_) {
                const auto it=streams_.find(name);
                stat.add(name+"_age_s",it==streams_.end()?-1.0:it->second.age());
            }
            stat.add("covariance_convention_verified",use_device_cov_);
            stat.add("clock_resets",static_cast<int64_t>(clock_resets_));
            stat.add("position_accuracy", pos == 0 ? "cm" : (pos == 1 ? "m" : (pos==POSITION_UNAVAILABLE?"unavailable":">1m")));
            stat.add("heading_health", hdg == 0 ? "stable" : (hdg==HEADING_UNAVAILABLE?"unavailable":"unstable"));
            stat.add("gyro_health", gyro == 0 ? "good" : (gyro==GYRO_UNAVAILABLE?"unavailable":"bad"));
            stat.add("message_rate_hz_recent", rate_hz);
            stat.add("error_rate_percent_recent", err_pct);
            stat.add("messages_total", static_cast<int64_t>(rate_monitor_.total_ok));
            stat.add("checksum_failures_total", static_cast<int64_t>(rate_monitor_.total_checksum_fail));
            stat.add("parse_failures_total", static_cast<int64_t>(rate_monitor_.total_parse_fail));
            stat.add("data_port", data_port_ ? data_port_->get_portname() : "N/A");
            stat.add("transmission_failures_total",static_cast<int64_t>(tx_failures_.load()));
            stat.add("device_input_rejections_total",static_cast<int64_t>(tx_input_rejections_.load()));
            stat.add("device_input_rate_drops_total",static_cast<int64_t>(tx_rate_drops_.load()));
            stat.add("rtcm_admission_bytes_per_second",rtcm_rate_);
            stat.add("command_mode",command_mode_);
            stat.add("truncated_datagrams_total",static_cast<int64_t>(data_port_?data_port_->truncated_datagrams():0));
            stat.add("config_connected",config_port_ && config_port_->connected());
            stat.add("config_port", config_port_ ? config_port_->get_portname() : "N/A");
        });
    }

    // ── Timers ────────────────────────────────────────────────────────
    void setup_timers()
    {
        config_timer_=create_wall_timer(std::chrono::milliseconds(100),
            [this] { if (config_port_) config_port_->poll(); },config_cb_group_);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(poll_ms_),
            std::bind(&AnelloRosDriver::mainloop_callback, this));
        health_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&AnelloRosDriver::health_callback, this));
    }

    void health_callback()
    {
        // Never received a message, or the stream has gone silent: the
        // flags describe a device that isn't talking, and restamping
        // them with now() would present them as fresh "all good".
        // /diagnostics carries the explicit no-data ERROR state.
        if (rate_monitor_.total_ok == 0 || rate_monitor_.rate_hz() <= 0.0)
            return;
        if (!stale_streams().empty()) return;
        publish_health(&health_msg_, pub_health_, now());
    }


    // ── Main decode loop ──────────────────────────────────────────────
    void mainloop_callback()
    {
        if (!data_port_) return;

        // Drain the port each tick instead of reading a single buffer:
        // one read per tick caps ethernet at one datagram per poll interval
        // and drops data at high message rates. Bounded to keep the
        // executor responsive.
        constexpr int kMaxReadsPerTick = 16;
        for (int reads = 0; reads < kMaxReadsPerTick; ++reads)
        {
            if (read_buf_.n_used >= read_buf_.nbytes)
            {
                // Poll without blocking; cap the callback at sixteen reads.
                read_buf_.nbytes = static_cast<int>(
                    data_port_->get_data(read_buf_.buff, MAX_BUF_LEN,
                                         0));
                read_buf_.n_used = 0;
                const auto generation=data_port_->generation();
                if (generation!=data_generation_) {
                    decoder_.reset(); reset_measurements(); clock_discontinuity_.reset(); data_generation_=generation;
                }
                if (read_buf_.nbytes <= 0)
                    break;
                // Arrival time captured at the read, not at parse —
                // messages decoded later from this buffer share it.
                read_buf_.stamp = now();
            }
            if (process_read_buffer() == 0 && !decoder_.pending() &&
                !frame_fail_reported_)
            {
                // Garbage outside a candidate frame advances UART discovery.
                data_port_->port_parse_fail();
            }
        }
    }

    int process_read_buffer()
    {
        int count=0;
        const auto crc_before=decoder_.checksum_failures, parse_before=decoder_.parse_failures;
        while (read_buf_.n_used<read_buf_.nbytes) {
            decoder_.feed(static_cast<uint8_t>(read_buf_.buff[read_buf_.n_used++]),
                [this,&count](const DecodedPacket &packet) {
                    ++count; rate_monitor_.add_ok(); data_port_->port_confirm();
                    dispatch(packet);
                });
        }
        for (auto n=crc_before;n<decoder_.checksum_failures;++n) rate_monitor_.add_checksum_fail();
        for (auto n=parse_before;n<decoder_.parse_failures;++n) rate_monitor_.add_parse_fail();
        frame_fail_reported_=crc_before!=decoder_.checksum_failures || parse_before!=decoder_.parse_failures;
        if (frame_fail_reported_) data_port_->port_parse_fail();
        return count;
    }

    void reset_measurements() {
        imu_time_={}; cov_time_={}; streams_.clear();
        health_msg_=health_message();
        health_msg_.set_fog_enabled(use_fog_wz_);
        health_msg_.set_baseline(get_parameter("heading_baseline").as_double());
        clock_translator_.reset(); ins_stamp_valid_=false; sim_stream_stamps_.clear();
        ++clock_resets_;
        // Keep the fixed geographic anchor across reconnects/reboots; changing
        // its definition without changing the frame ID would move the world.
    }

    rclcpp::Time stamp_from_mcu(double ms) {
        const auto steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
            SteadyClock::now().time_since_epoch()).count();
        const bool simulated=get_clock()->ros_time_is_active();
        if (clock_discontinuity_.update(ms,read_buf_.stamp.nanoseconds(),steady_ns,simulated))
            reset_measurements();
        if (use_mcu_stamp_ && !simulated) {
            clock_translator_.update_ns(ms*1e-3,read_buf_.stamp.nanoseconds());
            if (clock_translator_.ready()) {
                auto ns=clock_translator_.translate_ns(ms*1e-3);
                if (ns>=0) return rclcpp::Time(ns,read_buf_.stamp.get_clock_type());
            }
        }
        return read_buf_.stamp;
    }

    void dispatch(const DecodedPacket &packet) {
        auto values=packet.values; double *v=values.data();
        auto stamp=stamp_from_mcu(v[0]);
        // /clock=0 is uninitialized. During a simulated-time pause the
        // navigation stream is suppressed instead of inventing advancing TF stamps.
        if (get_clock()->ros_time_is_active()) {
            if (stamp.nanoseconds()==0) return;
            const auto previous=sim_stream_stamps_.find(packet.kind);
            if (previous!=sim_stream_stamps_.end() && stamp.nanoseconds()<=previous->second) return;
            sim_stream_stamps_[packet.kind]=stamp.nanoseconds();
        }
        switch (packet.kind) {
        case MessageKind::imu: case MessageKind::im1:
            streams_["imu"].set(v[0]);
            health_msg_.add_imu_message(v); store_last_imu(v);
            if (publish_custom_) {
                if (packet.kind==MessageKind::imu) publish_imu(v,pub_imu_,stamp,frame_imu_+"_frd");
                else publish_im1(v,pub_im1_,stamp,frame_imu_+"_frd");
            }
            publish_ros_imu_raw(v,stamp);
            break;
        case MessageKind::gps:
            streams_["gps"].set(v[0]); health_msg_.add_gps_message(v);
            if (publish_custom_) publish_gps(v,pub_gps_,stamp,frame_gnss_);
            publish_gga(v,pub_gga_,stamp,frame_gnss_,leap_seconds_);
            publish_navsat_from_gps(v,stamp);
            break;
        case MessageKind::gps2:
            streams_["gps2"].set(v[0]);
            if (publish_custom_) publish_gp2(v,pub_gp2_,stamp,frame_gnss2_);
            break;
        case MessageKind::heading:
            streams_["heading"].set(v[0]); health_msg_.add_hdg_message(v);
            if (publish_custom_) publish_hdr(v,pub_hdg_,stamp,frame_hdg_);
            break;
        case MessageKind::covariance:
            streams_["covariance"].set(v[0]); store_last_cov(v);
            if (publish_custom_) publish_cov(v,pub_cov_,stamp,frame_ins_+"_frd");
            break;
        case MessageKind::ins:
            streams_["ins"].set(v[0]); health_msg_.add_ins_message(v);
            if (ins_stamp_valid_) {
                if (v[0]<=last_ins_device_ms_) return;
                if (stamp<=last_ins_stamp_) {
                    if (!get_clock()->ros_time_is_active() && read_buf_.stamp==last_ins_arrival_stamp_)
                        stamp=last_ins_stamp_+rclcpp::Duration(0,1);
                    else return;
                }
            }
            last_ins_device_ms_=v[0]; last_ins_arrival_stamp_=read_buf_.stamp;
            last_ins_stamp_=stamp; ins_stamp_valid_=true;
            if (publish_custom_) publish_ins(v,pub_ins_,stamp,frame_ins_+"_frd");
            publish_ros_imu_and_nav(v,stamp);
            break;
        case MessageKind::ahrs: {
            streams_["ahrs"].set(v[0]);
            if (publish_custom_) {
                anello_interfaces::msg::APAHRS msg;
                msg.header.stamp=stamp; msg.header.frame_id=frame_imu_+"_frd";
                msg.time=v[0]; msg.sync_time=v[1]; msg.roll=v[2]; msg.pitch=v[3]; msg.yaw=v[4]; msg.zupt=v[5];
                pub_ahrs_->publish(msg);
            }
            // Relative AHRS yaw cannot claim a north-referenced INS attitude.
            // Publish its native angles; applications must establish heading
            // before using it as an ENU orientation measurement.
            break;
        }
        }
    }

    // ── Standard ROS message publishing ───────────────────────────────
    void store_last_imu(const double val[])
    {
        imu_time_.set(val[0]);
        imu_cache_.ax = val[1]; imu_cache_.ay = val[2]; imu_cache_.az = val[3];
        imu_cache_.wx = val[4]; imu_cache_.wy = val[5]; imu_cache_.wz = val[6];
        imu_cache_.wz_fog = val[7];
    }

    bool gyro_available(const ImuCache &imu) const {
        return health_msg_.get_gyro_status()!=GYRO_BAD &&
            (!use_fog_wz_ || (std::abs(imu.wz)<180.0 && std::abs(imu.wz_fog)<200.0));
    }

    // Body-frame measurements converted FRD -> FLU and to SI units, shared
    // by imu/data and imu/data_raw.
    void fill_imu_body_measurements(sensor_msgs::msg::Imu &msg,
                                    const ImuCache &imu) const
    {
        const double wz = use_fog_wz_ ? imu.wz_fog : imu.wz;
        msg.angular_velocity.x = imu.wx * kDeg2Rad;
        msg.angular_velocity.y = -imu.wy * kDeg2Rad;
        msg.angular_velocity.z = -wz * kDeg2Rad;

        const double accel_sign = flip_accel_sign_ ? -1.0 : 1.0;
        msg.linear_acceleration.x = accel_sign * imu.ax * kGAccel;
        msg.linear_acceleration.y = accel_sign * -imu.ay * kGAccel;
        msg.linear_acceleration.z = accel_sign * -imu.az * kGAccel;

        // Diagonal covariances from parameters; all-zero still means
        // "unknown" per REP-145. The FRD->FLU axis flips do not change a
        // diagonal covariance.
        for (int i = 0; i < 3; ++i) {
            msg.angular_velocity_covariance[4 * i] = ang_vel_cov_[i];
            msg.linear_acceleration_covariance[4 * i] = lin_acc_cov_[i];
        }
    }

    // REP-145 imu/data_raw: accelerometer + gyroscope only, published at
    // the sensor rate (every APIMU/APIM1). orientation_covariance[0] = -1
    // marks the orientation field as unreported.
    void publish_ros_imu_raw(const double val[], rclcpp::Time stamp)
    {
        ImuCache imu;
        imu.ax = val[1]; imu.ay = val[2]; imu.az = val[3];
        imu.wx = val[4]; imu.wy = val[5]; imu.wz = val[6];
        imu.wz_fog = val[7];

        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_imu_;
        msg.orientation.w = 1.0;
        msg.orientation_covariance[0] = -1.0;
        fill_imu_body_measurements(msg, imu);

        if (!gyro_available(imu)) msg.angular_velocity_covariance[0]=-1;
        check_accel_sign(imu);

        pub_ros_imu_raw_->publish(msg);
    }

    // One-shot startup sanity check for the accelerometer sign convention.
    // When a stationary, roughly level unit is detected, confirm the
    // published linear_acceleration.z points up (+g). If it comes out
    // negative the gravity vector is inverted (see the flip_accel_sign
    // parameter): warn loudly once so the misconfiguration is caught on
    // real hardware. Purely diagnostic — never alters the published data.
    void check_accel_sign(const ImuCache &imu)
    {
        if (!get_parameter("accel_sign_check_upright").as_bool() || accel_sign_checked_) return;

        // Bound the search: if the unit never settles (e.g. it powers up
        // already moving), give up quietly rather than warn spuriously.
        constexpr int kMaxSamples = 4000;      // ~20-40 s at 100-200 Hz
        constexpr int kRunNeeded = 50;         // consecutive stationary samples
        constexpr double kGyroStationaryDps = 1.0;
        if (++accel_check_samples_ > kMaxSamples) {
            accel_sign_checked_ = true;
            RCLCPP_DEBUG(get_logger(),
                "Accel-sign self-check skipped: unit not stationary/level "
                "at startup");
            return;
        }

        // Raw accel is in g, raw gyro in deg/s.
        const double gmag = std::sqrt(imu.ax * imu.ax + imu.ay * imu.ay +
                                      imu.az * imu.az);
        const double wmag = std::sqrt(imu.wx * imu.wx + imu.wy * imu.wy +
                                      imu.wz * imu.wz);
        const bool stationary =
            wmag < kGyroStationaryDps && gmag > 0.85 && gmag < 1.15;
        // Level enough that gravity lands mostly on the z axis, so its sign
        // is meaningful (within ~25 deg of level). A tilted stationary mount
        // is ambiguous for this simple test, so it resets the run.
        const bool level = std::fabs(imu.az) > 0.9 * gmag;
        if (!(stationary && level)) {
            accel_stationary_run_ = 0;
            accel_z_flu_sum_ = 0.0;
            return;
        }

        const double accel_sign = flip_accel_sign_ ? -1.0 : 1.0;
        accel_z_flu_sum_ += accel_sign * -imu.az * kGAccel;  // published FLU z
        if (++accel_stationary_run_ < kRunNeeded) return;

        accel_sign_checked_ = true;
        const double mean_z = accel_z_flu_sum_ / accel_stationary_run_;
        if (mean_z < 0.0) {
            RCLCPP_WARN(get_logger(),
                "imu/data gravity looks inverted: a stationary, level unit is "
                "publishing linear_acceleration.z = %.2f m/s^2 (a level IMU "
                "should read +%.2f per REP-145). Set the 'flip_accel_sign' "
                "parameter to %s to correct all three acceleration axes.",
                mean_z, kGAccel, flip_accel_sign_ ? "false" : "true");
        } else {
            RCLCPP_DEBUG(get_logger(),
                "Accel-sign self-check OK: stationary level unit reads "
                "linear_acceleration.z = +%.2f m/s^2", mean_z);
        }
    }

    void store_last_cov(const double val[])
    {
        cov_time_.set(val[0]);

        // Orientation covariance (deg^2) indices 13-18
        cov_cache_.orient[0] = val[13]; // roll-roll
        cov_cache_.orient[1] = val[16]; // roll-pitch
        cov_cache_.orient[2] = val[17]; // roll-heading
        cov_cache_.orient[3] = val[16]; // pitch-roll (symmetric)
        cov_cache_.orient[4] = val[14]; // pitch-pitch
        cov_cache_.orient[5] = val[18]; // pitch-heading
        cov_cache_.orient[6] = val[17]; // heading-roll (symmetric)
        cov_cache_.orient[7] = val[18]; // heading-pitch (symmetric)
        cov_cache_.orient[8] = val[15]; // heading-heading

        // Position covariance (m^2) indices 1-6
        cov_cache_.pos[0] = val[1]; // lat-lat
        cov_cache_.pos[1] = val[4]; // lat-lon
        cov_cache_.pos[2] = val[5]; // lat-alt
        cov_cache_.pos[3] = val[4]; // lon-lat
        cov_cache_.pos[4] = val[2]; // lon-lon
        cov_cache_.pos[5] = val[6]; // lon-alt
        cov_cache_.pos[6] = val[5]; // alt-lat
        cov_cache_.pos[7] = val[6]; // alt-lon
        cov_cache_.pos[8] = val[3]; // alt-alt

        // Velocity covariance ((m/s)^2) indices 7-12, NED order
        cov_cache_.vel[0] = val[7];  // vn-vn
        cov_cache_.vel[1] = val[10]; // vn-ve
        cov_cache_.vel[2] = val[11]; // vn-vd
        cov_cache_.vel[3] = val[10]; // ve-vn (symmetric)
        cov_cache_.vel[4] = val[8];  // ve-ve
        cov_cache_.vel[5] = val[12]; // ve-vd
        cov_cache_.vel[6] = val[11]; // vd-vn (symmetric)
        cov_cache_.vel[7] = val[12]; // vd-ve (symmetric)
        cov_cache_.vel[8] = val[9];  // vd-vd
    }

    bool covariance_fresh(double ms) const {
        return use_device_cov_ && cov_time_.matches(ms,cov_max_age_) &&
            cov_cache_.pos[0]+cov_cache_.pos[4]+cov_cache_.pos[8]>0 &&
            cov_cache_.orient[0]+cov_cache_.orient[4]+cov_cache_.orient[8]>0 &&
            cov_cache_.vel[0]+cov_cache_.vel[4]+cov_cache_.vel[8]>0;
    }
    tf2::Matrix3x3 orientation_covariance(const double ins[]) const {
        double c[9];
        for (int i=0;i<9;++i) c[i]=cov_cache_.orient[i]*kDeg2RadSq*kEnuCovSign[i];
        tf2::Matrix3x3 cov(c[0],c[1],c[2],c[3],c[4],c[5],c[6],c[7],c[8]);
        return euler_covariance_to_fixed(cov,-ins[10]*kDeg2Rad,kHalfPi-ins[11]*kDeg2Rad);
    }
    void publish_ros_imu_and_nav(const double ins[], rclcpp::Time stamp)
    {
        const bool fresh_imu=imu_time_.matches(ins[0],imu_max_age_) && frame_imu_==frame_ins_;
        const bool fresh_cov=covariance_fresh(ins[0]);
        const bool position_valid=ins_position_valid(ins);
        const auto q=ned_rpy_deg_to_enu_quat(ins[9],ins[10],ins[11]);
        sensor_msgs::msg::Imu imu;
        imu.header.stamp=stamp; imu.header.frame_id=frame_ins_;
        imu.orientation.x=q.x(); imu.orientation.y=q.y(); imu.orientation.z=q.z(); imu.orientation.w=q.w();
        if (fresh_imu) {
            fill_imu_body_measurements(imu,imu_cache_);
            if (!gyro_available(imu_cache_)) imu.angular_velocity_covariance[0]=-1;
        }
        else { imu.angular_velocity_covariance[0]=-1; imu.linear_acceleration_covariance[0]=-1; }
        if (fresh_cov) {
            const auto c=orientation_covariance(ins);
            for (int r=0;r<3;++r) for (int col=0;col<3;++col) imu.orientation_covariance[3*r+col]=c[r][col];
        }
        pub_ros_imu_->publish(imu);

        sensor_msgs::msg::NavSatFix nav;
        nav.header.stamp=stamp; nav.header.frame_id=frame_ins_;
        nav.status.status=position_valid ? ((ins[2]==3 || ins[2]==4) ?
            sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX:sensor_msgs::msg::NavSatStatus::STATUS_FIX) :
            sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        nav.status.service=gnss_service_mask_;
        nav.latitude=position_valid?ins[3]:NAN;
        nav.longitude=position_valid?ins[4]:NAN;
        nav.altitude=position_valid?ins[5]:NAN;
        if (fresh_cov && position_valid) {
            const double *p=cov_cache_.pos;
            nav.position_covariance={p[4],p[1],p[5],p[1],p[0],p[2],p[5],p[2],p[8]};
            nav.position_covariance_type=sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
        }
        pub_ins_fix_->publish(nav);
        if (position_valid) publish_odom(ins,stamp,q,fresh_imu,fresh_cov);
    }

    void publish_odom(const double ins[], rclcpp::Time stamp,
                      const tf2::Quaternion &q, bool fresh_imu, bool fresh_cov)
    {
        if (!local_cartesian_.initialized()) {
            local_cartesian_.set_origin(ins[3],ins[4],ins[5]);
            RCLCPP_INFO(get_logger(),"Local ENU origin: %.9f %.9f %.3f m",ins[3],ins[4],ins[5]);
        }
        nav_msgs::msg::Odometry odom;
        odom.header.stamp=stamp; odom.header.frame_id=tf_parent_; odom.child_frame_id=tf_child_;
        const auto pos=local_cartesian_.position(ins[3],ins[4],ins[5]);
        const auto world_rotation=local_cartesian_.rotation(ins[3],ins[4]);
        const tf2::Matrix3x3 body_to_current(q);
        const auto body_to_anchor=world_rotation*body_to_current;
        tf2::Quaternion projected_q; body_to_anchor.getRotation(projected_q); projected_q.normalize();
        odom.pose.pose.position.x=pos.x(); odom.pose.pose.position.y=pos.y(); odom.pose.pose.position.z=pos.z();
        odom.pose.pose.orientation.x=projected_q.x(); odom.pose.pose.orientation.y=projected_q.y();
        odom.pose.pose.orientation.z=projected_q.z(); odom.pose.pose.orientation.w=projected_q.w();
        for (int i=0;i<6;++i) {
            odom.pose.covariance[7*i]=unknown_variance_;
            odom.twist.covariance[7*i]=unknown_variance_;
        }
        const bool velocity_valid=std::isfinite(ins[6]) && std::isfinite(ins[7]) && std::isfinite(ins[8]);
        if (velocity_valid) {
            const auto velocity=body_to_current.transpose()*tf2::Vector3(ins[7],ins[6],-ins[8]);
            odom.twist.twist.linear.x=velocity.x(); odom.twist.twist.linear.y=velocity.y(); odom.twist.twist.linear.z=velocity.z();
        }
        if (fresh_imu && gyro_available(imu_cache_)) {
            odom.twist.twist.angular.x=imu_cache_.wx*kDeg2Rad;
            odom.twist.twist.angular.y=-imu_cache_.wy*kDeg2Rad;
            odom.twist.twist.angular.z=-(use_fog_wz_?imu_cache_.wz_fog:imu_cache_.wz)*kDeg2Rad;
            for (int i=0;i<3;++i) odom.twist.covariance[7*(i+3)]=ang_vel_cov_[i]>0?ang_vel_cov_[i]:unknown_variance_;
        }
        if (fresh_cov) {
            const double *p=cov_cache_.pos, *v=cov_cache_.vel;
            const tf2::Matrix3x3 cp(p[4],p[1],p[5],p[1],p[0],p[2],p[5],p[2],p[8]);
            const tf2::Matrix3x3 cv(v[4],v[1],-v[5],v[1],v[0],-v[2],-v[5],-v[2],v[8]);
            const auto pos_cov=world_rotation*cp*world_rotation.transpose();
            const auto att_cov=world_rotation*orientation_covariance(ins)*world_rotation.transpose();
            const auto vel_cov=body_to_current.transpose()*cv*body_to_current;
            for (int r=0;r<3;++r) for (int c=0;c<3;++c) {
                odom.pose.covariance[6*r+c]=pos_cov[r][c];
                odom.pose.covariance[6*(r+3)+c+3]=att_cov[r][c];
                if (velocity_valid) odom.twist.covariance[6*r+c]=vel_cov[r][c];
            }
        }
        pub_odom_->publish(odom);
        broadcast_ins_tf(odom);
    }

    // Raw GNSS solution on gps/fix: REP-145 consumers (notably
    // robot_localization's navsat_transform_node) expect an unfused GNSS
    // fix here — republishing the INS position would feed the IMU back
    // into the fusion. Covariance is approximated from the receiver's
    // horizontal/vertical accuracy estimates, as in the u-blox and NMEA
    // ROS drivers.
    void publish_navsat_from_gps(const double gps[], rclcpp::Time stamp)
    {
        auto nav = sensor_msgs::msg::NavSatFix();
        nav.header.stamp = stamp;
        nav.header.frame_id = frame_gnss_;

        // APGPS FixType {0 none, 2 2D, 3 3D, 5 time-only};
        // RTK status {0 SPP, 1 float, 2 fixed}.
        const int fix_type = static_cast<int>(gps[11]);
        const int rtk = static_cast<int>(gps[15]);
        if (fix_type != 2 && fix_type != 3)
            nav.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        else if (rtk == 1 || rtk == 2)
            nav.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        else
            nav.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        nav.status.service = gnss_service_mask_;

        nav.latitude  = gps[2];
        nav.longitude = gps[3];
        nav.altitude  = fix_type == 3 ? gps[4] : NAN;

        // navsat_transform copies this covariance verbatim into the EKF
        // measurement noise, and robot_localization replaces zeros with a
        // 1e-6 epsilon (wildly overtrusting the fix) — so claim
        // an approximate covariance only when the receiver reports real accuracies.
        const double hacc = gps[8];
        const double vacc = gps[9];
        if (fix_type == 3 && hacc > 0.0 && vacc > 0.0) {
            nav.position_covariance[0] = hacc * hacc;
            nav.position_covariance[4] = hacc * hacc;
            nav.position_covariance[8] = vacc * vacc;
            nav.position_covariance_type =
                sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        } else {
            nav.position_covariance_type =
                sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        }

        pub_navfix_->publish(nav);
    }


    // ── Member variables ──────────────────────────────────────────────
    interface_config_t config_;

    std::unique_ptr<anello_config_port> config_port_;
    std::unique_ptr<anello_data_port> data_port_;
    std::unique_ptr<ethernet_interface> odo_eth_port_;

    // Publishers
    imu_pub_t pub_imu_;
    im1_pub_t pub_im1_;
    ins_pub_t pub_ins_;
    gps_pub_t pub_gps_;
    gps_pub_t pub_gp2_;
    hdg_pub_t pub_hdg_;
    apcov_pub_t pub_cov_;
    health_pub_t pub_health_;
    gga_pub_t pub_gga_;
    ros_imu_pub_t pub_ros_imu_;
    ros_imu_pub_t pub_ros_imu_raw_;
    navfix_pub_t pub_navfix_;
    navfix_pub_t pub_ins_fix_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

    rclcpp::Publisher<anello_interfaces::msg::APAHRS>::SharedPtr pub_ahrs_;

    // Subscribers
    rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr sub_rtcm_;
    rclcpp::Subscription<anello_interfaces::msg::APODO>::SharedPtr sub_odo_;

    // Service
    rclcpp::Service<anello_interfaces::srv::CmdAndRsp>::SharedPtr srv_cmd_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Diagnostics
    std::unique_ptr<diagnostic_updater::Updater> diag_updater_;
    rclcpp::CallbackGroup::SharedPtr config_cb_group_, rtcm_cb_group_;
    rclcpp::TimerBase::SharedPtr config_timer_;
    std::atomic<uint64_t> tx_failures_{0};
    std::atomic<uint64_t> tx_input_rejections_{0},tx_rate_drops_{0};
    TrafficBudget rtcm_budget_,rtcm_frame_budget_,odo_budget_,command_budget_{2,1};
    std::string command_mode_="read_only";
    double rtcm_rate_=8192,odo_max_speed_=100;
    uint64_t data_generation_=0;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;

    // Frame IDs
    std::string frame_imu_, frame_ins_, frame_gnss_, frame_hdg_;
    bool publish_tf_ = false;
    std::string tf_parent_;
    std::string tf_child_;
    int64_t poll_ms_ = 5;
    bool use_fog_wz_ = true;
    bool flip_accel_sign_ = false;
    double ang_vel_cov_[3] = {};
    double lin_acc_cov_[3] = {};
    std::string timestamp_source_ = "mcu";
    bool use_mcu_stamp_ = false;
    ClockTranslator clock_translator_;

    // Decode state
    ReadBuffer read_buf_;
    StreamDecoder decoder_;
    bool frame_fail_reported_ = false;
    RateMonitor rate_monitor_;
    health_message health_msg_;
    ImuCache imu_cache_;
    CovCache cov_cache_;
    SampleTime imu_time_, cov_time_;
    std::map<std::string,SampleTime> streams_;
    std::vector<std::string> expected_streams_;
    double imu_max_age_=0.05, cov_max_age_=0.2, stream_timeout_=2, unknown_variance_=1e6;
    bool use_device_cov_=false, publish_custom_=true;
    int64_t gnss_service_mask_=0;
    int leap_seconds_=18;
    std::string frame_gnss2_;
    ClockDiscontinuity clock_discontinuity_;
    uint64_t clock_resets_=0;
    LocalCartesian local_cartesian_;

    // One-shot accelerometer-sign self-check state. The device's at-rest
    // accel sign is not in the public manual (see flip_accel_sign); this
    // watches a stationary, level unit at startup and warns once if the
    // published gravity looks inverted, without changing any output.
    bool accel_sign_checked_ = false;
    int accel_check_samples_ = 0;      // total IMU samples observed
    int accel_stationary_run_ = 0;     // consecutive stationary+level samples
    double accel_z_flu_sum_ = 0.0;     // sum of published FLU z over the run

    // Per-INS ordering within the current ROS/device clock epoch.
    rclcpp::Time last_ins_stamp_, last_ins_arrival_stamp_;
    bool ins_stamp_valid_ = false;
    double last_ins_device_ms_=0;
    std::map<MessageKind,int64_t> sim_stream_stamps_;


};

rclcpp::Node::SharedPtr make_anello_driver(const rclcpp::NodeOptions &options)
{
    return std::make_shared<AnelloRosDriver>(options);
}

} // namespace anello

RCLCPP_COMPONENTS_REGISTER_NODE(anello::AnelloRosDriver)
