// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <cmath>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system
#include <tf2/LinearMath/Quaternion.h>

#include <auto_aim_interfaces/msg/firecontrol.hpp>
#include <auto_aim_interfaces/srv/set_mode.hpp>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <deque>
#include <vector>

#include "auto_aim_interfaces/msg/target.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
    explicit RMSerialDriver(const rclcpp::NodeOptions & options);

    ~RMSerialDriver() override;

private:
    void getParams();

    void receiveData();

    void aimPointCallback(const auto_aim_interfaces::msg::Firecontrol::SharedPtr msg);

    void navCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void followMarkCallback(const std_msgs::msg::UInt8::SharedPtr msg);

    void updateOdomTransforms();

    void setDecisionCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void reopenPort();

    void setParam(const rclcpp::Parameter & param);
    void setRuneParam(const rclcpp::Parameter & param);

    void resetTracker();

    bool setRuneMode(uint8_t mode);
    bool setCarMode(uint8_t mode);

    tf2::Quaternion slerpSafe(
        const tf2::Quaternion & from, const tf2::Quaternion & to, double alpha);
    void appendBigYawSampleLocked(const tf2::Quaternion & q, const rclcpp::Time & stamp);
    bool findClosestBigYawSampleLocked(
        const rclcpp::Time & target_stamp, tf2::Quaternion & q, rclcpp::Time & stamp) const;

    // Serial port
    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    std::mutex mutex_;

    // Param client to set detect_colr
    using ResultFuturePtr =
        std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
    bool initial_set_param_ = false;
    bool initial_set_rune_param_ = false;
    uint8_t previous_receive_color_ = 0;
    rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
    ResultFuturePtr set_param_future_;
    rclcpp::AsyncParametersClient::SharedPtr rune_detector_param_client_;
    ResultFuturePtr set_rune_param_future_;
    rclcpp::AsyncParametersClient::SharedPtr detector_param_client_wide_;
    ResultFuturePtr set_param_future_wide_;

    // Service client to reset tracker
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

    // Service client to set mode
    rclcpp::Client<auto_aim_interfaces::srv::SetMode>::SharedPtr set_rune_detector_mode_client_,
        set_rune_solver_mode_client_, set_car_detector_mode_client_, set_car_tracker_mode_client_,
        set_car_detector_mode_client_wide_;

    // Service server to deal with decision
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_decision_service_server_;

    // Aimimg point receiving from serial port for visualization
    visualization_msgs::msg::Marker aiming_point_;

    // Broadcast tf from odom_aim to gimbal_link
    double timestamp_offset_ = 0;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<auto_aim_interfaces::msg::Firecontrol>::SharedPtr target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr follow_mark_sub_;

    tf2::Quaternion yaw_imu_q_{0, 0, 0, 1};
    tf2::Quaternion aim_imu_q_{0, 0, 0, 1};
    tf2::Quaternion lidar_imu_q_{0, 0, 0, 1};
    struct TimedQuaternion
    {
        tf2::Quaternion q;
        rclcpp::Time stamp;
    };
    std::deque<TimedQuaternion> big_yaw_history_;
    rclcpp::Time yaw_imu_stamp_;
    rclcpp::Time aim_imu_stamp_;
    rclcpp::Time lidar_imu_stamp_;
    bool has_yaw_imu_ = false;
    bool has_aim_imu_ = false;
    bool has_lidar_imu_ = false;

    float motor_yaw_ = 0.0F;
    float motor_pitch_ = 0.0F;
    rclcpp::Time motor_stamp_;
    bool has_motor_feedback_ = false;

    tf2::Quaternion q_odom_omni_to_odom_aim_{0, 0, 0, 1};
    tf2::Quaternion q_odom_to_odom_omni_{0, 0, 0, 1};
    tf2::Quaternion q_odom_omni_to_odom_aim_fused_{0, 0, 0, 1};
    tf2::Quaternion q_odom_to_odom_omni_fused_{0, 0, 0, 1};
    bool has_odom_omni_to_odom_aim_ = false;
    bool has_odom_to_odom_omni_ = false;
    bool has_fused_odom_omni_to_odom_aim_ = false;
    bool has_fused_odom_to_odom_omni_ = false;

    double comp_alpha_yaw_aim_ = 0.2;
    double comp_alpha_lidar_yaw_ = 0.2;
    double comp_alpha_motor_vs_imu_ = 0.7;
    double big_yaw_buffer_duration_sec_ = 0.2;
    std::size_t big_yaw_buffer_max_size_ = 256;
    double lidar_tf_max_stamp_diff_sec_ = 0.05;
    bool pitch_imu_enabled_ = true;

    // Dual-yaw allocation (big yaw + small yaw)
    bool use_dual_yaw_split_ = false;
    double dual_yaw_limit_rad_ = 3.14159265358979323846 / 3.0;       // 60 deg default limit
    double dual_yaw_center_ratio_ = 0.3;           // aggressiveness of small-yaw recentering

    std::mutex transform_mutex_;

    std::mutex follow_mark_mutex_;
    uint8_t latest_follow_mark_ = 1;
    rclcpp::Time latest_follow_mark_stamp_;
    bool has_follow_mark_ = false;
    double follow_mark_timeout_sec_ = 0.5;
    int nav_packet_version_ = 1;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // For debug usage
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gimbal_vel_pub_;

    // For decision
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr remain_ammo_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr sentry_health_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr our_base_health_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr enemy_base_health_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr our_outpost_health_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr enemy_outpost_health_pub_;

    std::thread receive_thread_;

    // mode
    uint8_t mode_ = -1;
    bool has_wide_cam_ = false;
    float current_yaw_vel = 0.0;
    float current_pitch_vel = 0.0;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
