// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <iostream>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <auto_aim_interfaces/msg/firecontrol.hpp>
#include <auto_aim_interfaces/srv/set_mode.hpp>
#include <cstdint>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
namespace
{
constexpr double kQuaternionNorm2Min = 1e-12;

bool sanitizeQuaternion(
    tf2::Quaternion & q, const rclcpp::Logger & logger, rclcpp::Clock & clock,
    const char * context)
{
    const double x = q.x();
    const double y = q.y();
    const double z = q.z();
    const double w = q.w();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(w)) {
        RCLCPP_WARN_THROTTLE(
            logger, clock, 2000,
            "Skip invalid quaternion at %s (non-finite): [%.6f, %.6f, %.6f, %.6f]", context, x,
            y, z, w);
        return false;
    }

    const double norm2 = (x * x) + (y * y) + (z * z) + (w * w);
    if (!std::isfinite(norm2) || norm2 <= kQuaternionNorm2Min) {
        RCLCPP_WARN_THROTTLE(
            logger, clock, 2000,
            "Skip invalid quaternion at %s (bad norm2=%.6e): [%.6f, %.6f, %.6f, %.6f]", context,
            norm2, x, y, z, w);
        return false;
    }

    q.normalize();
    if (!std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
        !std::isfinite(q.w())) {
        RCLCPP_WARN_THROTTLE(
            logger, clock, 2000,
            "Skip invalid quaternion at %s (non-finite after normalize): [%.6f, %.6f, %.6f, "
            "%.6f]",
            context, q.x(), q.y(), q.z(), q.w());
        return false;
    }

    return true;
}

bool sendTransformIfQuaternionValid(
    tf2_ros::TransformBroadcaster & broadcaster, geometry_msgs::msg::TransformStamped & transform,
    const rclcpp::Logger & logger, rclcpp::Clock & clock, const char * context)
{
    tf2::Quaternion q(
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w);
    if (!sanitizeQuaternion(q, logger, clock, context)) {
        return false;
    }
    transform.transform.rotation = tf2::toMsg(q);
    broadcaster.sendTransform(transform);
    return true;
}
}  // namespace

void RMSerialDriver::appendBigYawSampleLocked(const tf2::Quaternion & q, const rclcpp::Time & stamp)
{
    big_yaw_history_.push_back(TimedQuaternion{q, stamp});

    while (big_yaw_history_.size() > big_yaw_buffer_max_size_) {
        big_yaw_history_.pop_front();
    }

    while (!big_yaw_history_.empty() &&
           (stamp - big_yaw_history_.front().stamp).seconds() > big_yaw_buffer_duration_sec_) {
        big_yaw_history_.pop_front();
    }
}

bool RMSerialDriver::findClosestBigYawSampleLocked(
    const rclcpp::Time & target_stamp, tf2::Quaternion & q, rclcpp::Time & stamp) const
{
    if (big_yaw_history_.empty()) {
        return false;
    }

    auto best_it = big_yaw_history_.begin();
    double best_abs_diff = std::abs((best_it->stamp - target_stamp).seconds());
    for (auto it = std::next(big_yaw_history_.begin()); it != big_yaw_history_.end(); ++it) {
        const double current_abs_diff = std::abs((it->stamp - target_stamp).seconds());
        if (current_abs_diff < best_abs_diff) {
            best_it = it;
            best_abs_diff = current_abs_diff;
        }
    }

    q = best_it->q;
    stamp = best_it->stamp;
    return true;
}

RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
    RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

    getParams();

    // TF broadcaster
    timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
    comp_alpha_yaw_aim_ = this->declare_parameter("comp_alpha_yaw_aim", 0.2);
    comp_alpha_lidar_yaw_ = this->declare_parameter("comp_alpha_lidar_yaw", 0.2);
    comp_alpha_motor_vs_imu_ = this->declare_parameter("comp_alpha_motor_vs_imu", 0.7);
    big_yaw_buffer_duration_sec_ =
        std::max(0.0, this->declare_parameter("big_yaw_buffer_duration_sec", 0.2));
    auto big_yaw_buffer_max_size = this->declare_parameter("big_yaw_buffer_max_size", 256);
    if (big_yaw_buffer_max_size < 1) {
        big_yaw_buffer_max_size = 1;
    }
    big_yaw_buffer_max_size_ = static_cast<std::size_t>(big_yaw_buffer_max_size);
    lidar_tf_max_stamp_diff_sec_ =
        std::max(0.0, this->declare_parameter("lidar_tf_max_stamp_diff_sec", 0.05));
    pitch_imu_enabled_ = this->declare_parameter("pitch_imu_enabled", true);
    use_dual_yaw_split_ = this->declare_parameter("use_dual_yaw_split", false);
    double dual_yaw_limit_deg = this->declare_parameter("dual_yaw_limit_deg", 60.0);
    dual_yaw_limit_deg = std::clamp(dual_yaw_limit_deg, 1.0, 179.0);
    dual_yaw_limit_rad_ = dual_yaw_limit_deg * M_PI / 180.0;
    dual_yaw_center_ratio_ = std::clamp(this->declare_parameter("dual_yaw_center_ratio", 0.3), 0.0, 1.0);
    follow_mark_timeout_sec_ = std::max(0.0, this->declare_parameter("follow_mark_timeout_sec", 0.5));
    nav_packet_version_ = this->declare_parameter("nav_packet_version", 1);
    if (nav_packet_version_ != 1 && nav_packet_version_ != 2) {
        RCLCPP_WARN(
            get_logger(), "Invalid nav_packet_version=%d, fallback to v1", nav_packet_version_);
        nav_packet_version_ = 1;
    }
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create Publisher
    latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
    sentry_health_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/ifhealth", 10);
    our_base_health_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/our_base_health", 10);
    enemy_base_health_pub_ =
        this->create_publisher<std_msgs::msg::UInt16>("/enemy_base_health", 10);
    our_outpost_health_pub_ =
        this->create_publisher<std_msgs::msg::UInt16>("/our_outpost_health", 10);
    enemy_outpost_health_pub_ =
        this->create_publisher<std_msgs::msg::UInt16>("/enemy_outpost_health", 10);
    remain_ammo_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/remain_ammo", 10);

    // Detect parameter client
    detector_param_client_ =
        std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

    // Tracker reset service client
    reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

    // set mode service client
    set_rune_detector_mode_client_ =
        this->create_client<auto_aim_interfaces::srv::SetMode>("/rune_detector/set_mode");
    set_rune_solver_mode_client_ =
        this->create_client<auto_aim_interfaces::srv::SetMode>("/rune_solver/set_mode");
    set_car_detector_mode_client_ =
        this->create_client<auto_aim_interfaces::srv::SetMode>("/armor_detector/set_mode");
    set_car_tracker_mode_client_ =
        this->create_client<auto_aim_interfaces::srv::SetMode>("/armor_tracker/set_mode");

    // set decision service server
    set_decision_service_server_ = this->create_service<std_srvs::srv::SetBool>(
        "/set_bool", [this](
                         const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            this->setDecisionCallback(request, response);
        });

    try {
        serial_driver_->init_port(device_name_, *device_config_);
        if (!serial_driver_->port()->is_open()) {
            serial_driver_->port()->open();
            receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
        }
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(
            get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
        throw ex;
    }

    aiming_point_.header.frame_id = "odom_aim";
    aiming_point_.ns = "aiming_point";
    aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
    aiming_point_.action = visualization_msgs::msg::Marker::ADD;
    aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
    aiming_point_.color.r = 1.0;
    aiming_point_.color.g = 1.0;
    aiming_point_.color.b = 1.0;
    aiming_point_.color.a = 1.0;
    aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

    // Create Subscription
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Firecontrol>(
        "/firecontrol", rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&RMSerialDriver::aimPointCallback, this, std::placeholders::_1));

    nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_chassis", rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&RMSerialDriver::navCallback, this, std::placeholders::_1));

    const auto follow_mark_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    follow_mark_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/chassis/follow_mark", follow_mark_qos,
        std::bind(&RMSerialDriver::followMarkCallback, this, std::placeholders::_1));
}

RMSerialDriver::~RMSerialDriver()
{
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    if (serial_driver_->port()->is_open()) {
        serial_driver_->port()->close();
    }

    if (owned_ctx_) {
        owned_ctx_->waitForExit();
    }
}

void RMSerialDriver::receiveData()
{
    std::vector<uint8_t> header(1);
    std::vector<uint8_t> data;
    data.reserve(sizeof(ReceivePacket));

    while (rclcpp::ok()) {
        try {
            serial_driver_->port()->receive(header);

            if (header[0] == 0x5A) {
                data.resize(sizeof(ReceivePacket) - 1);
                serial_driver_->port()->receive(data);

                data.insert(data.begin(), header[0]);
                ReceivePacket packet = fromVector(data);

                bool crc_ok = crc16::Verify_CRC16_Check_Sum(
                    reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
                if (crc_ok) {
                    const rclcpp::Time sample_stamp = this->now();
                    tf2::Quaternion yaw_q(
                        packet.yaw_imu_q[0], packet.yaw_imu_q[1], packet.yaw_imu_q[2],
                        packet.yaw_imu_q[3]);
                    if (!sanitizeQuaternion(
                            yaw_q, get_logger(), *get_clock(), "receiveData:packet.yaw_imu_q")) {
                        continue;
                    }
                    {
                        double roll, pitch, yaw;
                        tf2::Matrix3x3(yaw_q).getRPY(roll, pitch, yaw);
                        yaw_q.setRPY(-roll, -pitch, yaw);
                    }
                    if (!sanitizeQuaternion(
                            yaw_q, get_logger(), *get_clock(), "receiveData:yaw_q_converted")) {
                        continue;
                    }

                    tf2::Quaternion aim_q(0.0, 0.0, 0.0, 1.0);

                    float motor_yaw = packet.motor_yaw;
                    float motor_pitch =
                        -packet.motor_pitch;  // 电机编码器的正负和轴系的正负是相反的

                    if (pitch_imu_enabled_) {
                        aim_q = tf2::Quaternion(
                            packet.aim_imu_q[0], packet.aim_imu_q[1], packet.aim_imu_q[2],
                            packet.aim_imu_q[3]);
                        if (!sanitizeQuaternion(
                                aim_q, get_logger(), *get_clock(), "receiveData:packet.aim_imu_q")) {
                            continue;
                        }
                    } else {
                        // When pitch IMU is absent, derive aim orientation from yaw IMU plus motor feedback.
                        tf2::Quaternion q_mech;
                        q_mech.setRPY(
                            0.0, static_cast<double>(motor_pitch), static_cast<double>(motor_yaw));
                        if (!sanitizeQuaternion(
                                q_mech, get_logger(), *get_clock(), "receiveData:motor_feedback_q")) {
                            continue;
                        }
                        aim_q = yaw_q * q_mech;
                        if (!sanitizeQuaternion(
                                aim_q, get_logger(), *get_clock(), "receiveData:aim_q_from_motor")) {
                            continue;
                        }
                    }

                    {
                        std::lock_guard<std::mutex> lock(transform_mutex_);
                        yaw_imu_q_ = yaw_q;
                        aim_imu_q_ = aim_q;
                        yaw_imu_stamp_ = sample_stamp;
                        aim_imu_stamp_ = yaw_imu_stamp_;
                        has_yaw_imu_ = true;
                        has_aim_imu_ = pitch_imu_enabled_;
                        appendBigYawSampleLocked(yaw_q, sample_stamp);
                        motor_yaw_ = motor_yaw;
                        motor_pitch_ = motor_pitch;
                        motor_stamp_ = yaw_imu_stamp_;
                        has_motor_feedback_ = true;
                    }

                    updateOdomTransforms();

                    mode_ = 9;

                    // Broadcast odom_omni -> omni_gimbal_link using yaw IMU as the parent orientation.
                    tf2::Quaternion q_rot;
                    geometry_msgs::msg::TransformStamped t_omni;
                    timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
                    t_omni.header.stamp =
                        sample_stamp + rclcpp::Duration::from_seconds(timestamp_offset_);
                    t_omni.header.frame_id = "odom_omni";
                    t_omni.child_frame_id = "omni_gimbal_link";
                    q_rot.setRPY(0, 0, 0);
                    t_omni.transform.rotation = tf2::toMsg(yaw_q * q_rot);
                    t_omni.transform.translation.x = 0.0;
                    t_omni.transform.translation.y = 0.0;
                    t_omni.transform.translation.z = 0.0;
                    sendTransformIfQuaternionValid(
                        *tf_broadcaster_, t_omni, get_logger(), *get_clock(),
                        "receiveData:odom_omni->omni_gimbal_link");

                    geometry_msgs::msg::TransformStamped t;
                    timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
                    t.header.stamp =
                        sample_stamp + rclcpp::Duration::from_seconds(timestamp_offset_);
                    t.header.frame_id = "odom_aim";
                    t.child_frame_id = "gimbal_link";
                    q_rot.setRPY(0, 0, 0);
                    t.transform.rotation = tf2::toMsg(aim_q * q_rot);
                    t.transform.translation.x = 0.0;
                    t.transform.translation.y = 0.0;
                    t.transform.translation.z = 0.0;
                    sendTransformIfQuaternionValid(
                        *tf_broadcaster_, t, get_logger(), *get_clock(),
                        "receiveData:odom_aim->gimbal_link");

                    //publish game info
                    std_msgs::msg::UInt16 sentryHP;

                    sentryHP.data = packet.sentryHP;
                    //std::cout<<"sentHP: " << sentryHP.data << std::endl;
                    std_msgs::msg::UInt16 our_baseHP;
                    our_baseHP.data = packet.our_baseHP;
                    //std::cout<<"our_baseHP: " << our_baseHP.data << std::endl;
                    std_msgs::msg::UInt16 enemy_baseHP;
                    enemy_baseHP.data = packet.enemy_baseHP;
                    //std::cout<<"enemy_baseHP: " << enemy_baseHP.data << std::endl;
                    std_msgs::msg::UInt16 our_outpostHP;
                    our_outpostHP.data = packet.our_outpostHP;
                    //std::cout<<"our_outpostHP: " << our_outpostHP.data << std::endl;
                    std_msgs::msg::UInt16 enemy_outpostHP;
                    enemy_outpostHP.data = packet.enemy_outpostHP;
                    //std::cout<<"enemy_outpostHP: " << enemy_outpostHP.data << std::endl;
                    std_msgs::msg::UInt16 remain_ammo;
                    remain_ammo.data = packet.remain_ammo;

                    //std::cout<<"sentryHP: " << sentryHP.data << " our_baseHP: " << our_baseHP.data << " remain ammo: " << remain_ammo.data << " our_outpostHP: " << our_outpostHP.data << " enemy_outpostHP: " << enemy_outpostHP.data << std::endl;

                    sentry_health_pub_->publish(sentryHP);
                    our_base_health_pub_->publish(our_baseHP);
                    enemy_base_health_pub_->publish(enemy_baseHP);
                    our_outpost_health_pub_->publish(our_outpostHP);
                    enemy_outpost_health_pub_->publish(enemy_outpostHP);
                    remain_ammo_pub_->publish(remain_ammo);

                } else {
                    RCLCPP_ERROR(get_logger(), "CRC error!");
                }
            } else {
                std::cout << "invalid header" << std::endl;
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
            }
        } catch (const std::exception & ex) {
            std::cout << "Error while receiving data: " << ex.what() << std::endl;
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
            reopenPort();
        }
    }
}

void RMSerialDriver::updateOdomTransforms()
{
    const rclcpp::Time stamp = this->now();
    timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
    const rclcpp::Duration tf_stamp_offset = rclcpp::Duration::from_seconds(timestamp_offset_);
    geometry_msgs::msg::TransformStamped lidar_tf;
    bool has_lidar_tf = false;
    bool has_lidar_for_current_yaw = false;
    bool used_latest_lidar_tf_fallback = false;
    rclcpp::Time yaw_stamp(0, 0, this->get_clock()->get_clock_type());
    rclcpp::Time paired_yaw_stamp(0, 0, this->get_clock()->get_clock_type());
    tf2::Quaternion paired_yaw_q(0.0, 0.0, 0.0, 1.0);
    bool has_yaw_for_lookup = false;
    bool has_paired_yaw_for_lidar = false;
    {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        if (has_yaw_imu_) {
            yaw_stamp = yaw_imu_stamp_;
            has_yaw_for_lookup = true;
        }
    }

    if (has_yaw_for_lookup) {
        try {
            const auto latest_lidar_tf =
                tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            const rclcpp::Time latest_lidar_stamp(
                latest_lidar_tf.header.stamp, this->get_clock()->get_clock_type());
            if (yaw_stamp > latest_lidar_stamp) {
                lidar_tf = latest_lidar_tf;
                has_lidar_tf = true;
                used_latest_lidar_tf_fallback = true;
            } else {
                lidar_tf = tf_buffer_->lookupTransform("odom", "base_link", yaw_stamp);
                has_lidar_tf = true;
            }
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Failed to lookup odom->base_link for yaw stamp %.3f: %s",
                yaw_stamp.seconds(), ex.what());
        }
    }

    if (has_lidar_tf) {
        const rclcpp::Time lidar_stamp(lidar_tf.header.stamp, this->get_clock()->get_clock_type());
        {
            std::lock_guard<std::mutex> lock(transform_mutex_);
            if (used_latest_lidar_tf_fallback) {
                has_paired_yaw_for_lidar =
                    findClosestBigYawSampleLocked(lidar_stamp, paired_yaw_q, paired_yaw_stamp);
            } else if (has_yaw_imu_) {
                paired_yaw_q = yaw_imu_q_;
                paired_yaw_stamp = yaw_imu_stamp_;
                has_paired_yaw_for_lidar = true;
            }
        }

        if (!has_paired_yaw_for_lidar) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Skip odom->odom_omni update because big_yaw history is unavailable for lidar stamp %.3f",
                lidar_stamp.seconds());
        } else {
            const double stamp_diff_sec =
                std::abs((lidar_stamp - paired_yaw_stamp).seconds());
            if (used_latest_lidar_tf_fallback) {
                RCLCPP_DEBUG_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Using latest odom->base_link fallback for paired yaw stamp %.3f, lidar stamp %.3f, diff %.3f s",
                    paired_yaw_stamp.seconds(), lidar_stamp.seconds(), stamp_diff_sec);
            }
            if (stamp_diff_sec > lidar_tf_max_stamp_diff_sec_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Skip odom->odom_omni update because odom->base_link stamp diff is %.3f s "
                    "(limit %.3f s, paired_yaw=%.3f, lidar=%.3f)",
                    stamp_diff_sec, lidar_tf_max_stamp_diff_sec_, paired_yaw_stamp.seconds(),
                    lidar_stamp.seconds());
            } else {
                tf2::Quaternion lidar_q(
                    lidar_tf.transform.rotation.x, lidar_tf.transform.rotation.y,
                    lidar_tf.transform.rotation.z, lidar_tf.transform.rotation.w);
                if (sanitizeQuaternion(
                        paired_yaw_q, get_logger(), *get_clock(),
                        "updateOdomTransforms:paired_big_yaw_q") &&
                    sanitizeQuaternion(
                        lidar_q, get_logger(), *get_clock(), "updateOdomTransforms:odom->base_link")) {
                    std::lock_guard<std::mutex> lock(transform_mutex_);
                    lidar_imu_q_ = lidar_q;
                    lidar_imu_stamp_ = lidar_stamp;
                    has_lidar_imu_ = true;
                    has_lidar_for_current_yaw = true;
                }
            }
        }
    }

    tf2::Quaternion yaw_q;
    tf2::Quaternion aim_q;
    tf2::Quaternion lidar_q;
    float motor_yaw = 0.0F;
    float motor_pitch = 0.0F;
    bool has_yaw = false;
    bool has_aim = false;
    bool has_lidar = false;
    bool has_motor = false;

    {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        yaw_q = yaw_imu_q_;
        aim_q = aim_imu_q_;
        lidar_q = lidar_imu_q_;
        motor_yaw = motor_yaw_;
        motor_pitch = motor_pitch_;
        has_yaw = has_yaw_imu_;
        has_aim = has_aim_imu_;
        has_lidar = has_lidar_imu_;
        has_motor = has_motor_feedback_;
    }
    has_lidar = has_lidar && has_lidar_for_current_yaw;

    if (has_yaw &&
        !sanitizeQuaternion(yaw_q, get_logger(), *get_clock(), "updateOdomTransforms:yaw_imu_q_")) {
        has_yaw = false;
    }
    if (has_aim &&
        !sanitizeQuaternion(aim_q, get_logger(), *get_clock(), "updateOdomTransforms:aim_imu_q_")) {
        has_aim = false;
    }
    if (has_lidar && !sanitizeQuaternion(
                         lidar_q, get_logger(), *get_clock(), "updateOdomTransforms:lidar_imu_q_")) {
        has_lidar = false;
    }

    if (!pitch_imu_enabled_) {
        if (has_yaw) {
            tf2::Quaternion identity_q(0.0, 0.0, 0.0, 1.0);
            {
                std::lock_guard<std::mutex> lock(transform_mutex_);
                q_odom_omni_to_odom_aim_ = identity_q;
                q_odom_omni_to_odom_aim_fused_ = identity_q;
                has_odom_omni_to_odom_aim_ = true;
                has_fused_odom_omni_to_odom_aim_ = true;
            }

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = yaw_stamp + tf_stamp_offset;
            t.header.frame_id = "odom_omni";
            t.child_frame_id = "odom_aim";
            t.transform.rotation = tf2::toMsg(identity_q);
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;
            sendTransformIfQuaternionValid(
                *tf_broadcaster_, t, get_logger(), *get_clock(),
                "updateOdomTransforms:odom_omni->odom_aim(identity)");
        }
    } else if (has_yaw && has_aim) {
        tf2::Quaternion q_rel = yaw_q.inverse() * aim_q;
        if (sanitizeQuaternion(
                q_rel, get_logger(), *get_clock(),
                "updateOdomTransforms:q_odom_omni_to_odom_aim_raw")) {
            if (has_motor) {
                tf2::Quaternion q_mech;
                // Motor feedback defines relative yaw then pitch in odom_omni frame; assume extrinsic yaw (Z) then pitch (Y).
                q_mech.setRPY(
                    0.0, static_cast<double>(motor_pitch), static_cast<double>(motor_yaw));
                if (sanitizeQuaternion(
                        q_mech, get_logger(), *get_clock(),
                        "updateOdomTransforms:motor_feedback_relative_q")) {
                    q_rel = slerpSafe(q_mech, q_rel, comp_alpha_motor_vs_imu_);
                }
            }

            if (sanitizeQuaternion(
                    q_rel, get_logger(), *get_clock(),
                    "updateOdomTransforms:q_odom_omni_to_odom_aim_fused_input")) {
                {
                    std::lock_guard<std::mutex> lock(transform_mutex_);
                    q_odom_omni_to_odom_aim_ = q_rel;
                    has_odom_omni_to_odom_aim_ = true;
                    if (has_fused_odom_omni_to_odom_aim_) {
                        q_odom_omni_to_odom_aim_fused_ =
                            slerpSafe(q_odom_omni_to_odom_aim_fused_, q_rel, comp_alpha_yaw_aim_);
                    } else {
                        q_odom_omni_to_odom_aim_fused_ = q_rel;
                        has_fused_odom_omni_to_odom_aim_ = true;
                    }
                }

                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = yaw_stamp + tf_stamp_offset;
                t.header.frame_id = "odom_omni";
                t.child_frame_id = "odom_aim";
                t.transform.rotation = tf2::toMsg(q_odom_omni_to_odom_aim_fused_);
                t.transform.translation.x = 0.0;
                t.transform.translation.y = 0.0;
                t.transform.translation.z = 0.0;
                sendTransformIfQuaternionValid(
                    *tf_broadcaster_, t, get_logger(), *get_clock(),
                    "updateOdomTransforms:odom_omni->odom_aim");
            }
        }
    }

    if (has_lidar && has_paired_yaw_for_lidar) {
        // q(odom->odom_omni) = q(odom->base_link) * inv(q(odom_omni->omni_gimbal_link))
        tf2::Quaternion q_rel = lidar_q * paired_yaw_q.inverse();
        if (sanitizeQuaternion(
                q_rel, get_logger(), *get_clock(),
                "updateOdomTransforms:q_odom_to_odom_omni_raw")) {
            {
                std::lock_guard<std::mutex> lock(transform_mutex_);
                q_odom_to_odom_omni_ = q_rel;
                has_odom_to_odom_omni_ = true;
                if (has_fused_odom_to_odom_omni_) {
                    q_odom_to_odom_omni_fused_ =
                        slerpSafe(q_odom_to_odom_omni_fused_, q_rel, comp_alpha_lidar_yaw_);
                } else {
                    q_odom_to_odom_omni_fused_ = q_rel;
                    has_fused_odom_to_odom_omni_ = true;
                }
            }

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = paired_yaw_stamp + tf_stamp_offset;
            t.header.frame_id = "odom";
            t.child_frame_id = "odom_omni";
            t.transform.rotation = tf2::toMsg(q_odom_to_odom_omni_fused_);
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;
            sendTransformIfQuaternionValid(
                *tf_broadcaster_, t, get_logger(), *get_clock(),
                "updateOdomTransforms:odom->odom_omni");
        }
    }
}

void RMSerialDriver::aimPointCallback(const auto_aim_interfaces::msg::Firecontrol::SharedPtr msg)
{
    const static std::map<std::string, uint8_t> id_unit8_map{
        {"", 0},  {"outpost", 0}, {"1", 1},     {"1", 1},    {"2", 2},   {"3", 3},
        {"4", 4}, {"5", 5},       {"guard", 6}, {"base", 7}, {"rune", 8}};

    try {
        SendAimPacket packet;

        packet.tracking = msg->tracking;
        packet.id = id_unit8_map.at(msg->id);
        packet.iffire = msg->iffire;

        tf2::Quaternion target_in_aim;
        target_in_aim.setRPY(0.0, msg->pitch, msg->yaw);

        tf2::Quaternion target_in_omni = target_in_aim;
        bool has_transform = false;

        auto wrapAngle = [](double ang) {
            double v = std::fmod(ang + M_PI, 2.0 * M_PI);
            if (v < 0) v += 2.0 * M_PI;
            return v - M_PI;
        };

        double big_yaw_now = 0.0;
        double small_yaw_now = 0.0;
        bool has_big_yaw = false;
        bool has_small_yaw = false;

        {
            std::lock_guard<std::mutex> lock(transform_mutex_);
            has_transform = has_fused_odom_omni_to_odom_aim_;
            if (has_transform) {
                target_in_omni = q_odom_omni_to_odom_aim_fused_ * target_in_aim;
                target_in_omni.normalize();
            }

            if (has_yaw_imu_) {
                double r = 0.0, p = 0.0, y = 0.0;
                tf2::Matrix3x3(yaw_imu_q_).getRPY(r, p, y);
                big_yaw_now = wrapAngle(y);
                has_big_yaw = true;
            }

            if (has_motor_feedback_) {
                small_yaw_now = static_cast<double>(motor_yaw_);
                small_yaw_now = wrapAngle(small_yaw_now);
                has_small_yaw = true;
            }
        }

        if (!has_transform) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Transform odom_aim->odom_omni missing, forwarding aim command without frame "
                "conversion");
        }

        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf2::Matrix3x3(target_in_omni).getRPY(roll, pitch, yaw);
        packet.pitch = static_cast<float>(pitch);

        double yaw_cmd = yaw;
        double big_yaw_cmd = yaw;
        double small_yaw_cmd = yaw;

        if (use_dual_yaw_split_ && has_big_yaw) {
            double err = wrapAngle(yaw - big_yaw_now);

            if (std::abs(err) > dual_yaw_limit_rad_) {
                // Large error: let big yaw eat the portion beyond the limit, keep small yaw at the limit.
                const double sign = (err > 0.0) ? 1.0 : -1.0;
                small_yaw_cmd = sign * dual_yaw_limit_rad_;
                const double big_delta = err - small_yaw_cmd;
                big_yaw_cmd = wrapAngle(big_yaw_now + big_delta);
            } else {
                // Within limit: bleed a portion back to big yaw to recenter small yaw gradually.
                const double recenter_term = has_small_yaw ? small_yaw_now * dual_yaw_center_ratio_ : 0.0;
                small_yaw_cmd = wrapAngle(err - recenter_term);
                small_yaw_cmd = std::clamp(small_yaw_cmd, -dual_yaw_limit_rad_, dual_yaw_limit_rad_);
                big_yaw_cmd = wrapAngle(yaw - small_yaw_cmd);
            }

            yaw_cmd = small_yaw_cmd;
        } else {
            big_yaw_cmd = wrapAngle(yaw_cmd);
            small_yaw_cmd = wrapAngle(yaw_cmd - big_yaw_cmd);
        }

        packet.big_yaw = static_cast<float>(big_yaw_cmd);
        packet.small_yaw = static_cast<float>(yaw_cmd);

        if (use_dual_yaw_split_) {
            RCLCPP_DEBUG_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "dual-yaw split: target=%.3f big=%.3f small=%.3f err_limit=%.2fdeg",
                yaw, big_yaw_cmd, yaw_cmd, dual_yaw_limit_rad_ * 180.0 / M_PI);
        }

        crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

        std::vector<uint8_t> data = toVector(packet);

        std::lock_guard<std::mutex> lock(mutex_);
        serial_driver_->port()->send(data);

        std_msgs::msg::Float64 latency;
        latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
        RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
        latency_pub_->publish(latency);
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Error while sending auto-aim data: %s", ex.what());
        reopenPort();
    }
}

void RMSerialDriver::navCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    try {
        const float linear_x = -static_cast<float>(msg->linear.x);
        const float linear_y = -static_cast<float>(msg->linear.y);
        const float linear_z = static_cast<float>(msg->linear.z);
        const float angular_x = static_cast<float>(msg->angular.x);
        const float angular_y = static_cast<float>(msg->angular.y);
        const float angular_z = static_cast<float>(msg->angular.z);

        if (nav_packet_version_ == 2) {
            uint8_t follow_mark = 1;
            bool follow_mark_fresh = false;
            const rclcpp::Time now = this->now();
            {
                std::lock_guard<std::mutex> lock(follow_mark_mutex_);
                if (has_follow_mark_) {
                    const double age_sec = (now - latest_follow_mark_stamp_).seconds();
                    if (age_sec <= follow_mark_timeout_sec_) {
                        follow_mark = latest_follow_mark_;
                        follow_mark_fresh = true;
                    }
                }
            }

            if (!follow_mark_fresh) {
                follow_mark = 1;
                RCLCPP_DEBUG_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "follow_mark unavailable or timeout (%.3fs), fallback to 1",
                    follow_mark_timeout_sec_);
            }

            SendNavPacketV2 packet;
            packet.linear_x = linear_x;
            packet.linear_y = linear_y;
            packet.linear_z = linear_z;
            packet.angular_x = angular_x;
            packet.angular_y = angular_y;
            packet.angular_z = angular_z;
            packet.follow_mark = follow_mark;

            crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

            std::vector<uint8_t> data = toVector(packet);
            std::lock_guard<std::mutex> lock(mutex_);
            serial_driver_->port()->send(data);
        } else {
            SendNavPacket packet;
            packet.linear_x = -linear_x;
            packet.linear_y = -linear_y;
            packet.linear_z = linear_z;
            packet.angular_x = angular_x;
            packet.angular_y = angular_y;
            packet.angular_z = angular_z;

            crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

            std::vector<uint8_t> data = toVector(packet);
            std::lock_guard<std::mutex> lock(mutex_);
            serial_driver_->port()->send(data);
        }
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Error while sending nav data: %s", ex.what());
        reopenPort();
    }
}

void RMSerialDriver::followMarkCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(follow_mark_mutex_);
    if (msg) {
        if (msg->data <= 1U) {
            latest_follow_mark_ = msg->data;
        } else {
            latest_follow_mark_ = 1U;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "follow_mark=%u is invalid, fallback to 1", msg->data);
        }
        has_follow_mark_ = true;
        latest_follow_mark_stamp_ = this->now();
    } else {
        latest_follow_mark_ = 1;
        has_follow_mark_ = false;
    }
}

void RMSerialDriver::setDecisionCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    try {
        SendDecisionPacket packet;
        packet.ifreload = request->data;

        crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

        std::vector<uint8_t> data = toVector(packet);

        std::lock_guard<std::mutex> lock(mutex_);
        serial_driver_->port()->send(data);
        std::cout << "whether to enter outpost attack:" << packet.ifreload << std::endl;
        response->success = true;

    } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Error while sending decision data: %s", ex.what());
        response->success = false;
        reopenPort();
    }
}

tf2::Quaternion RMSerialDriver::slerpSafe(
    const tf2::Quaternion & from, const tf2::Quaternion & to, double alpha)
{
    double a = std::clamp(alpha, 0.0, 1.0);
    const auto logger = get_logger();
    auto clock = get_clock();

    tf2::Quaternion f = from;
    tf2::Quaternion t = to;
    const bool from_valid = sanitizeQuaternion(f, logger, *clock, "slerpSafe:from");
    const bool to_valid = sanitizeQuaternion(t, logger, *clock, "slerpSafe:to");

    if (!from_valid && !to_valid) {
        RCLCPP_WARN_THROTTLE(
            logger, *clock, 2000,
            "Both input quaternions are invalid in slerpSafe, fallback to identity");
        return tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
    }
    if (!from_valid) {
        return t;
    }
    if (!to_valid) {
        return f;
    }

    tf2::Quaternion r = tf2::slerp(f, t, a);
    if (!sanitizeQuaternion(r, logger, *clock, "slerpSafe:result")) {
        return t;
    }
    return r;
}

void RMSerialDriver::getParams()
{
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    uint32_t baud_rate{};
    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;

    try {
        device_name_ = declare_parameter<std::string>("device_name", "");
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
        throw ex;
    }

    try {
        baud_rate = declare_parameter<int>("baud_rate", 0);
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
        throw ex;
    }

    try {
        const auto fc_string = declare_parameter<std::string>("flow_control", "");

        if (fc_string == "none") {
            fc = FlowControl::NONE;
        } else if (fc_string == "hardware") {
            fc = FlowControl::HARDWARE;
        } else if (fc_string == "software") {
            fc = FlowControl::SOFTWARE;
        } else {
            throw std::invalid_argument{
                "The flow_control parameter must be one of: none, software, or hardware."};
        }
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
        throw ex;
    }

    try {
        const auto pt_string = declare_parameter<std::string>("parity", "");

        if (pt_string == "none") {
            pt = Parity::NONE;
        } else if (pt_string == "odd") {
            pt = Parity::ODD;
        } else if (pt_string == "even") {
            pt = Parity::EVEN;
        } else {
            throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
        }
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
        throw ex;
    }

    try {
        const auto sb_string = declare_parameter<std::string>("stop_bits", "");

        if (sb_string == "1" || sb_string == "1.0") {
            sb = StopBits::ONE;
        } else if (sb_string == "1.5") {
            sb = StopBits::ONE_POINT_FIVE;
        } else if (sb_string == "2" || sb_string == "2.0") {
            sb = StopBits::TWO;
        } else {
            throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
        }
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
        throw ex;
    }

    device_config_ =
        std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
    RCLCPP_WARN(get_logger(), "Attempting to reopen port");
    try {
        if (serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
        }
        serial_driver_->port()->open();
        RCLCPP_INFO(get_logger(), "Successfully reopened port");
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
        if (rclcpp::ok()) {
            rclcpp::sleep_for(std::chrono::seconds(1));
            reopenPort();
        }
    }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
    if (!detector_param_client_->service_is_ready()) {
        RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
        return;
    }

    if (!set_param_future_.valid() ||
        set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
        set_param_future_ = detector_param_client_->set_parameters(
            {param}, [this, param](const ResultFuturePtr & results) {
                for (const auto & result : results.get()) {
                    if (!result.successful) {
                        RCLCPP_ERROR(
                            get_logger(), "Failed to set parameter: %s", result.reason.c_str());
                        return;
                    }
                }
                RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
                initial_set_param_ = true;
            });
    }
}

void RMSerialDriver::resetTracker()
{
    if (!reset_tracker_client_->service_is_ready()) {
        RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    reset_tracker_client_->async_send_request(request);
    RCLCPP_INFO(get_logger(), "Reset tracker!");
}

bool RMSerialDriver::setRuneMode(uint8_t mode)
{
    if (!set_rune_solver_mode_client_->service_is_ready() ||
        !set_rune_detector_mode_client_->service_is_ready()) {
        RCLCPP_WARN(get_logger(), "Service not ready, skipping set rune mode");
        return 0;
    }

    auto request = std::make_shared<auto_aim_interfaces::srv::SetMode::Request>();
    request->mode = mode;

    auto result_tracker_future = set_rune_solver_mode_client_->async_send_request(request);
    auto result_detector_future = set_rune_detector_mode_client_->async_send_request(request);

    try {
        auto result1 = result_tracker_future.get();
        auto result2 = result_detector_future.get();
        if (result1->success && result2->success) {
            RCLCPP_INFO(get_logger(), "Successfully set rune mode to %d", mode);
            return true;
        } else {
            RCLCPP_ERROR(
                get_logger(), "Failed to set rune mode: %s and %s", result1->message.c_str(),
                result2->message.c_str());
        }
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Service call failed: %s", ex.what());
    }
    return false;
}

bool RMSerialDriver::setCarMode(uint8_t mode)
{
    if (!set_car_tracker_mode_client_->service_is_ready() ||
        !set_car_detector_mode_client_->service_is_ready()) {
        RCLCPP_WARN(get_logger(), "Service not ready, skipping set car mode");
        return 0;
    }

    auto request = std::make_shared<auto_aim_interfaces::srv::SetMode::Request>();
    request->mode = mode;

    auto result_tracker_future = set_car_tracker_mode_client_->async_send_request(request);
    auto result_detector_future = set_car_detector_mode_client_->async_send_request(request);

    try {
        auto result1 = result_tracker_future.get();
        auto result2 = result_detector_future.get();
        if (result1->success && result2->success) {
            RCLCPP_INFO(get_logger(), "Successfully set car mode to %d", mode);
            return true;
        } else {
            RCLCPP_ERROR(
                get_logger(), "Failed to set car mode: %s and %s", result1->message.c_str(),
                result2->message.c_str());
        }
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Service call failed: %s", ex.what());
    }
    return false;
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
