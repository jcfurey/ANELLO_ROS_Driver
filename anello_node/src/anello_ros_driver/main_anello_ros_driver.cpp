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
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <unistd.h>
#include <iomanip>
#include <memory>
#include <cmath>
#include <stdexcept>

#include "main_anello_ros_driver.h"

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

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "bit_tools.h"
#include "messaging/rtcm_decoder.h"
#include "messaging/ascii_decoder.h"
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

static int input_a1_data(a1buff_t *a1, uint8_t data);

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
};

struct ReadBuffer
{
    int n_used = 0;
    int nbytes = 0;
    char buff[MAX_BUF_LEN] = {};
};


class AnelloRosDriver : public rclcpp::Node
{
public:
    explicit AnelloRosDriver(const rclcpp::NodeOptions &options)
        : Node("anello_ros_driver", options)
    {
        declare_all_parameters();
        read_parameters();
        setup_ports();
        setup_publishers();
        setup_subscribers();
        setup_services();
        setup_tf_broadcaster();
        setup_diagnostics();
        setup_timers();

        RCLCPP_INFO(get_logger(), "ANELLO ROS2 driver initialized (v3.1.0)");
    }

    ~AnelloRosDriver() override = default;

private:
    // ── Parameter declaration ──────────────────────────────────────────
    void declare_all_parameters()
    {
        auto d = [](const std::string &desc) {
            rcl_interfaces::msg::ParameterDescriptor pd;
            pd.description = desc;
            return pd;
        };

        declare_parameter("com_type", "UART",
            d("Communication type: UART or ETH"));
        declare_parameter("uart_data_port", "AUTO",
            d("UART data port path or AUTO for auto-detection"));
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

        declare_parameter("frame_id.imu", "imu_link",
            d("Frame ID for IMU messages"));
        declare_parameter("frame_id.ins", "ins_link",
            d("Frame ID for INS messages"));
        declare_parameter("frame_id.gnss", "gnss_link",
            d("Frame ID for GNSS messages"));
        declare_parameter("frame_id.hdg", "gnss_link",
            d("Frame ID for dual-antenna heading messages"));

        declare_parameter("publish_tf", true,
            d("Publish TF transform from odom to ins_link"));
        declare_parameter("tf_parent_frame", "odom",
            d("Parent frame for TF broadcast"));

        declare_parameter("poll_interval_ms", 5,
            d("Main loop polling interval in milliseconds"));

        declare_parameter("heading_baseline", 0.0,
            d("Dual-antenna baseline length in meters, used to validate the "
              "APHDG heading in the health monitor (0.0 = skip the check)"));

        declare_parameter("use_fog_wz", true,
            d("Use the optical gyro (OG_WZ) for the z angular rate in the "
              "standard imu/data and imu/data_raw messages instead of the "
              "MEMS WZ. Set false if the FOG is disabled on the unit "
              "(APCFG fog off)."));

        // Defaults derived from the ANELLO GNSS INS datasheet noise specs at
        // 100 Hz: MEMS gyro ARW 0.3 deg/sqrt(hr), optical Z gyro ARW
        // 0.05 deg/sqrt(hr), accelerometer VRW 0.03 m/s/sqrt(hr).
        declare_parameter("covariance.angular_velocity",
            std::vector<double>{7.6e-7, 7.6e-7, 2.1e-8},
            d("Diagonal angular velocity covariance [x, y, z] in (rad/s)^2 "
              "for imu/data and imu/data_raw (REP-145 parameter override)"));
        declare_parameter("covariance.linear_acceleration",
            std::vector<double>{2.5e-5, 2.5e-5, 2.5e-5},
            d("Diagonal linear acceleration covariance [x, y, z] in "
              "(m/s^2)^2 for imu/data and imu/data_raw"));
    }

    void read_parameters()
    {
        std::string com_type = get_parameter("com_type").as_string();
        if (com_type == "ETH")
            config_.type = ETH;
        else
            config_.type = UART;

        config_.data_port_name = get_parameter("uart_data_port").as_string();
        config_.config_port_name = get_parameter("uart_config_port").as_string();
        config_.baud_rate = static_cast<uint32_t>(get_parameter("baud_rate").as_int());
        config_.remote_ip = get_parameter("remote_ip").as_string();
        config_.local_data_port = static_cast<int>(get_parameter("local_data_port").as_int());
        config_.local_config_port = static_cast<int>(get_parameter("local_config_port").as_int());
        config_.local_odometer_port = static_cast<int>(get_parameter("local_odometer_port").as_int());

        frame_imu_ = get_parameter("frame_id.imu").as_string();
        frame_ins_ = get_parameter("frame_id.ins").as_string();
        frame_gnss_ = get_parameter("frame_id.gnss").as_string();
        frame_hdg_ = get_parameter("frame_id.hdg").as_string();
        publish_tf_ = get_parameter("publish_tf").as_bool();
        tf_parent_ = get_parameter("tf_parent_frame").as_string();
        poll_ms_ = get_parameter("poll_interval_ms").as_int();
        health_msg_.set_baseline(get_parameter("heading_baseline").as_double());
        use_fog_wz_ = get_parameter("use_fog_wz").as_bool();

        auto read_cov3 = [this](const char *name, double out[3]) {
            auto v = get_parameter(name).as_double_array();
            if (v.size() == 3) {
                std::copy(v.begin(), v.end(), out);
            } else {
                RCLCPP_WARN(get_logger(),
                    "%s must have exactly 3 elements, got %zu — using zeros "
                    "(covariance unknown)", name, v.size());
                std::fill(out, out + 3, 0.0);
            }
        };
        read_cov3("covariance.angular_velocity", ang_vel_cov_);
        read_cov3("covariance.linear_acceleration", lin_acc_cov_);

        RCLCPP_INFO(get_logger(), "com_type=%s baud=%u poll=%ldms",
                     com_type.c_str(), config_.baud_rate, poll_ms_);
    }


    // ── Port setup ─────────────────────────────────────────────────────
    void setup_ports()
    {
        try {
            config_port_ = std::make_unique<anello_config_port>(&config_);
            config_port_->init();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Config port init failed: %s", e.what());
            // Config port failure is not fatal — command service won't work
        }

        try {
            data_port_ = std::make_unique<anello_data_port>(&config_);
            data_port_->init();
        } catch (const std::exception &e) {
            RCLCPP_FATAL(get_logger(), "Data port init failed: %s", e.what());
            throw;
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
        pub_health_ = create_publisher<anello_interfaces::msg::APHEALTH>("anello/health", 1);
        pub_gga_ = create_publisher<nmea_msgs::msg::Sentence>("ntrip_client/nmea", 1);

        pub_ros_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", sensor_qos);
        pub_ros_imu_raw_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", sensor_qos);
        pub_navfix_ = create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", sensor_qos);
    }

    // ── Subscribers ────────────────────────────────────────────────────
    void setup_subscribers()
    {
        // SensorDataQoS (best-effort) to match the NTRIP client's publisher;
        // a reliable subscription would not connect to a best-effort publisher.
        sub_rtcm_ = create_subscription<rtcm_msgs::msg::Message>(
            "ntrip_client/rtcm", rclcpp::SensorDataQoS(),
            [this](const rtcm_msgs::msg::Message::SharedPtr msg) {
                if (data_port_)
                    data_port_->write_data(
                        reinterpret_cast<const char *>(msg->message.data()),
                        msg->message.size());
            });

        sub_odo_ = create_subscription<anello_interfaces::msg::APODO>(
            "anello/odo", 1,
            [this](const anello_interfaces::msg::APODO::SharedPtr msg) {
                std::ostringstream body;
                body << "APODO," << std::fixed << std::setprecision(2) << msg->odo_speed;
                std::string body_str = body.str();
                std::string ck = compute_checksum(body_str.c_str(), body_str.length());
                std::string full = "#" + body_str + "*" + ck + "\r\n";
                if (config_.type == ETH && odo_eth_port_)
                    odo_eth_port_->write_data(full.c_str(), full.length());
                else if (config_port_)
                    config_port_->write_data(full.c_str(), full.length());
            });
    }

    // ── Services ───────────────────────────────────────────────────────
    void setup_services()
    {
        srv_cmd_ = create_service<anello_interfaces::srv::CmdAndRsp>(
            "anello/send_cmd",
            [this](const std::shared_ptr<anello_interfaces::srv::CmdAndRsp::Request> req,
                   std::shared_ptr<anello_interfaces::srv::CmdAndRsp::Response> res) {
                send_command_callback(req, res);
            });
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
        std::string ck = compute_checksum(body.c_str(), body.size());
        std::string full = "#" + body + "*" + ck + "\r\n";

        config_port_->write_data(full.c_str(), static_cast<int>(full.size()));

        auto start = std::chrono::steady_clock::now();
        auto timeout = std::chrono::milliseconds(500);

        // Bounded 20 ms reads instead of unbounded reads + usleep: the plain
        // serial read can block for VTIME (0.5 s) per call, stalling the
        // executor well past the 500 ms response budget.
        while (std::chrono::steady_clock::now() - start < timeout) {
            int n = static_cast<int>(config_port_->get_data(read_buf, kMaxResp - 1, 20));
            if (n > 0) {
                read_buf[n] = '\0';
                response += read_buf;
                if (response.size() >= 2 &&
                    response.substr(response.size() - 2) == "\r\n")
                    break;
            }
        }

        if (response.empty()) {
            res->response = "ERROR: no response from device (timeout)";
        } else {
            res->response = response;
        }
        RCLCPP_DEBUG(get_logger(), "send_cmd: sent='%s' got='%s'",
                     full.c_str(), res->response.c_str());
    }


    // ── TF broadcaster ────────────────────────────────────────────────
    void setup_tf_broadcaster()
    {
        if (publish_tf_)
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    void broadcast_ins_tf(double roll_deg, double pitch_deg, double heading_deg)
    {
        if (!tf_broadcaster_) return;

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now();
        t.header.frame_id = tf_parent_;
        t.child_frame_id = frame_ins_;

        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q = ned_rpy_deg_to_enu_quat(roll_deg, pitch_deg, heading_deg);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }

    // ── Diagnostics ───────────────────────────────────────────────────
    void setup_diagnostics()
    {
        diag_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
        diag_updater_->setHardwareID("anello_gnss_ins");

        diag_updater_->add("ANELLO Device Status", [this](diagnostic_updater::DiagnosticStatusWrapper &stat) {
            uint8_t pos = health_msg_.get_position_status();
            uint8_t hdg = health_msg_.get_heading_status();
            uint8_t gyro = health_msg_.get_gyro_status();

            if (pos == 0 && hdg == 0 && gyro == 0)
                stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "All systems nominal");
            else if (gyro > 0)
                stat.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Gyro health degraded");
            else
                stat.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Degraded accuracy");

            stat.add("position_accuracy", pos == 0 ? "cm" : (pos == 1 ? "m" : ">1m"));
            stat.add("heading_health", hdg == 0 ? "stable" : "unstable");
            stat.add("gyro_health", gyro == 0 ? "good" : "bad");
            stat.add("data_port", data_port_ ? data_port_->get_portname() : "N/A");
            stat.add("config_port", config_port_ ? config_port_->get_portname() : "N/A");
        });
    }

    // ── Timers ────────────────────────────────────────────────────────
    void setup_timers()
    {
        timer_ = create_wall_timer(
            std::chrono::milliseconds(poll_ms_),
            std::bind(&AnelloRosDriver::mainloop_callback, this));
        health_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&AnelloRosDriver::health_callback, this));
    }

    void health_callback()
    {
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
                read_buf_.nbytes = static_cast<int>(
                    data_port_->get_data(read_buf_.buff, MAX_BUF_LEN));
                read_buf_.n_used = 0;
                if (read_buf_.nbytes <= 0)
                    break;
            }
            process_read_buffer();
        }
    }

    void process_read_buffer()
    {
        while (read_buf_.n_used < read_buf_.nbytes)
        {
            int ret = input_a1_data(&a1buff_,
                static_cast<uint8_t>(read_buf_.buff[read_buf_.n_used]));
            read_buf_.n_used++;

            if (!ret) continue;

            bool is_ok = false;
            int num = 0;
            char *val[MAXFIELD];
            double decoded_val[MAXFIELD] = {};

            if (ret == 1)
            {
                // ASCII message
                if (checksum(a1buff_.buf, a1buff_.nbyte))
                {
                    num = parse_fields(reinterpret_cast<char *>(a1buff_.buf), val);
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                        "Checksum fail: %s", a1buff_.buf);
                    num = 0;
                }

                auto stamp = now();

                if (num >= 17 && strstr(val[0], "APGPS") != nullptr)
                {
                    decode_ascii_gps(val, decoded_val);
                    publish_gps(decoded_val, pub_gps_, stamp, frame_gnss_);
                    publish_gga(decoded_val, pub_gga_, stamp, frame_gnss_);
                    health_msg_.add_gps_message(decoded_val);
                    is_ok = true;
                }
                else if (num >= 17 && strstr(val[0], "APGP2") != nullptr)
                {
                    decode_ascii_gps(val, decoded_val);
                    publish_gp2(decoded_val, pub_gp2_, stamp, frame_gnss_);
                    is_ok = true;
                }
                else if (num >= 12 && strstr(val[0], "APHDG") != nullptr)
                {
                    decode_ascii_hdr(val, decoded_val);
                    publish_hdr(decoded_val, pub_hdg_, stamp, frame_hdg_);
                    health_msg_.add_hdg_message(decoded_val);
                    is_ok = true;
                }
                else if (num >= 12 && strstr(val[0], "APIMU") != nullptr)
                {
                    decode_ascii_imu(val, num, decoded_val);
                    publish_imu(decoded_val, pub_imu_, stamp, frame_imu_);
                    publish_ros_imu_raw(decoded_val, stamp);
                    health_msg_.add_imu_message(decoded_val);
                    store_last_imu(decoded_val);
                    is_ok = true;
                }
                else if (num >= 11 && strstr(val[0], "APIM1") != nullptr)
                {
                    decode_ascii_im1(val, num, decoded_val);
                    publish_im1(decoded_val, pub_im1_, stamp, frame_imu_);
                    publish_ros_imu_raw(decoded_val, stamp);
                    is_ok = true;
                }
                else if (num >= 20 && strstr(val[0], "APCOV") != nullptr)
                {
                    decode_ascii_cov(val, decoded_val);
                    publish_cov(decoded_val, pub_cov_, stamp, frame_ins_);
                    store_last_cov(decoded_val);
                    is_ok = true;
                }
                else if (num >= 14 && strstr(val[0], "APINS") != nullptr)
                {
                    decode_ascii_ins(val, decoded_val);
                    publish_ins(decoded_val, pub_ins_, stamp, frame_ins_);
                    health_msg_.add_ins_message(decoded_val);
                    publish_ros_imu_and_nav(decoded_val, stamp);
                    broadcast_ins_tf(decoded_val[9], decoded_val[10], decoded_val[11]);
                    is_ok = true;
                }
            }
            else if (ret == 5)
            {
                is_ok = handle_rtcm_message(decoded_val);
            }

            if (is_ok) {
                data_port_->port_confirm();
            } else {
                data_port_->port_parse_fail();
            }
            a1buff_.nbyte = 0;
        }
    }


    // ── RTCM handler ──────────────────────────────────────────────────

    // The decoders memcpy a fixed-size packed struct from the payload at
    // buf+5 (3-byte frame header + 12-bit type + 4-bit subtype), so the
    // frame length (nlen = payload + 3) must cover struct size + 5.
    bool rtcm_payload_covers(size_t struct_size) const
    {
        return a1buff_.nlen >= static_cast<int>(struct_size) + 5;
    }

    bool handle_rtcm_message(double *decoded_val)
    {
        if (a1buff_.type != 4058 || a1buff_.crc)
            return false;

        auto stamp = now();

        switch (a1buff_.subtype) {
        case 1: // IMU
            if (!rtcm_payload_covers(sizeof(rtcm_old_apimu_t)))
                return false;
            decode_rtcm_imu_msg(decoded_val, a1buff_);
            publish_imu(decoded_val, pub_imu_, stamp, frame_imu_);
            publish_ros_imu_raw(decoded_val, stamp);
            health_msg_.add_imu_message(decoded_val);
            store_last_imu(decoded_val);
            return true;

        case 2: { // GPS PVT
            if (!rtcm_payload_covers(sizeof(rtcm_apgps_t)))
                return false;
            int ant_id = decode_rtcm_gps_msg(decoded_val, a1buff_);
            if (GPS1 == ant_id) {
                publish_gps(decoded_val, pub_gps_, stamp, frame_gnss_);
                publish_gga(decoded_val, pub_gga_, stamp, frame_gnss_);
                health_msg_.add_gps_message(decoded_val);
            } else {
                publish_gp2(decoded_val, pub_gp2_, stamp, frame_gnss_);
            }
            return true;
        }

        case 3: // Dual antenna heading
            if (!rtcm_payload_covers(sizeof(rtcm_aphdr_t)))
                return false;
            decode_rtcm_hdg_msg(decoded_val, a1buff_);
            publish_hdr(decoded_val, pub_hdg_, stamp, frame_hdg_);
            health_msg_.add_hdg_message(decoded_val);
            return true;

        case 4: // INS
            if (!rtcm_payload_covers(sizeof(rtcm_apins_t)))
                return false;
            decode_rtcm_ins_msg(decoded_val, a1buff_);
            publish_ins(decoded_val, pub_ins_, stamp, frame_ins_);
            health_msg_.add_ins_message(decoded_val);
            publish_ros_imu_and_nav(decoded_val, stamp);
            broadcast_ins_tf(decoded_val[9], decoded_val[10], decoded_val[11]);
            return true;

        case 6: // IM1
            if (!rtcm_payload_covers(sizeof(rtcm_apim1_t)))
                return false;
            decode_rtcm_im1_msg(decoded_val, a1buff_);
            publish_im1(decoded_val, pub_im1_, stamp, frame_imu_);
            publish_ros_imu_raw(decoded_val, stamp);
            return true;

        case 10: // APCOV
            if (!rtcm_payload_covers(sizeof(rtcm_apcov_t)))
                return false;
            decode_rtcm_cov_msg(decoded_val, a1buff_);
            publish_cov(decoded_val, pub_cov_, stamp, frame_ins_);
            store_last_cov(decoded_val);
            return true;

        default:
            return false;
        }
    }

    // ── Standard ROS message publishing ───────────────────────────────
    void store_last_imu(const double val[])
    {
        imu_cache_.ax = val[1]; imu_cache_.ay = val[2]; imu_cache_.az = val[3];
        imu_cache_.wx = val[4]; imu_cache_.wy = val[5]; imu_cache_.wz = val[6];
        imu_cache_.wz_fog = val[7];
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

        msg.linear_acceleration.x = imu.ax * kGAccel;
        msg.linear_acceleration.y = -imu.ay * kGAccel;
        msg.linear_acceleration.z = -imu.az * kGAccel;

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

        pub_ros_imu_raw_->publish(msg);
    }

    void store_last_cov(const double val[])
    {
        cov_received_ = true;

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
    }

    void publish_ros_imu_and_nav(const double ins[], rclcpp::Time stamp)
    {
        // ── sensor_msgs/Imu (REP-103: ENU orientation, FLU body axes) ──
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = frame_imu_;

        tf2::Quaternion q = ned_rpy_deg_to_enu_quat(ins[9], ins[10], ins[11]);
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();

        fill_imu_body_measurements(imu_msg, imu_cache_);

        // All-zero covariance means "unknown" per sensor_msgs convention,
        // which is what cov_cache_ holds until the first APCOV arrives.
        for (int i = 0; i < 9; ++i)
            imu_msg.orientation_covariance[i] =
                cov_cache_.orient[i] * kDeg2RadSq * kEnuCovSign[i];

        pub_ros_imu_->publish(imu_msg);

        // ── sensor_msgs/NavSatFix ──
        auto nav = sensor_msgs::msg::NavSatFix();
        nav.header.stamp = stamp;
        nav.header.frame_id = frame_gnss_;

        // APINS status: 0/8 = attitude only (8-10 are GPS-disabled variants),
        // 1/2/9/10 = position valid, 3/4 = RTK float/fix.
        const int ins_status = static_cast<int>(ins[2]);
        if (ins_status == 3 || ins_status == 4)
            nav.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        else if (ins_status == 0 || ins_status == 8)
            nav.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        else
            nav.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        nav.status.service =
            sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
            sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
            sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS |
            sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;

        nav.latitude  = ins[3];
        nav.longitude = ins[4];
        nav.altitude  = ins[5];

        // ENU covariance: [lon,lat,alt] -> [ee,en,eu; ne,nn,nu; ue,un,uu]
        nav.position_covariance[0] = cov_cache_.pos[4]; // lon-lon
        nav.position_covariance[1] = cov_cache_.pos[1]; // lat-lon
        nav.position_covariance[2] = cov_cache_.pos[5]; // lon-alt
        nav.position_covariance[3] = cov_cache_.pos[1]; // lat-lon
        nav.position_covariance[4] = cov_cache_.pos[0]; // lat-lat
        nav.position_covariance[5] = cov_cache_.pos[2]; // lat-alt
        nav.position_covariance[6] = cov_cache_.pos[5]; // lon-alt
        nav.position_covariance[7] = cov_cache_.pos[2]; // lat-alt
        nav.position_covariance[8] = cov_cache_.pos[8]; // alt-alt
        nav.position_covariance_type = cov_received_
            ? sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN
            : sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

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

    // Subscribers
    rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr sub_rtcm_;
    rclcpp::Subscription<anello_interfaces::msg::APODO>::SharedPtr sub_odo_;

    // Service
    rclcpp::Service<anello_interfaces::srv::CmdAndRsp>::SharedPtr srv_cmd_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Diagnostics
    std::unique_ptr<diagnostic_updater::Updater> diag_updater_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;

    // Frame IDs
    std::string frame_imu_, frame_ins_, frame_gnss_, frame_hdg_;
    bool publish_tf_ = true;
    std::string tf_parent_;
    int64_t poll_ms_ = 5;
    bool use_fog_wz_ = true;
    double ang_vel_cov_[3] = {};
    double lin_acc_cov_[3] = {};

    // Decode state
    ReadBuffer read_buf_;
    a1buff_t a1buff_;
    health_message health_msg_;
    ImuCache imu_cache_;
    CovCache cov_cache_;
    bool cov_received_ = false;
};

} // namespace anello

RCLCPP_COMPONENTS_REGISTER_NODE(anello::AnelloRosDriver)


/* State machine decoder for ASCII and RTCM messages
 *
 * Return:
 *   0 = not ready
 *   1 = ASCII message ready
 *   5 = RTCM message ready
 */
static int input_a1_data(a1buff_t *a1, uint8_t data)
{
    int ret = 0;

    if (a1->nbyte >= MAX_BUF_LEN)
        a1->nbyte = 0;

    // Detect correct start characters: #AP or 0xD3
    if (a1->nbyte == 0 && !(data == '#' || data == 0xD3))
    {
        a1->nbyte = 0;
        return 0;
    }
    if (a1->nbyte == 1 && !((data == 'A' && a1->buf[0] == '#') || a1->buf[0] == 0xD3))
    {
        a1->nbyte = 0;
        return 0;
    }
    if (a1->nbyte == 2 && !((data == 'P' && a1->buf[1] == 'A' && a1->buf[0] == '#') || a1->buf[0] == 0xD3))
    {
        a1->nbyte = 0;
        return 0;
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
