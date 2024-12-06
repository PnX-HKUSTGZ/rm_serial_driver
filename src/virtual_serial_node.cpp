// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.

// std
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <rclcpp/executors.hpp>
#include <thread>
// ros2
#include <Eigen/Eigen>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// project
#include "auto_aim_interfaces/srv/set_mode.hpp"

namespace rm_serial_driver {
class VirtualSerialNode : public rclcpp::Node {
  struct SetModeClient {
    SetModeClient(rclcpp::Client<auto_aim_interfaces::srv::SetMode>::SharedPtr p) : ptr(p) {}
    std::atomic<bool> on_waiting = false;
    std::atomic<int> mode = 0;
    rclcpp::Client<auto_aim_interfaces::srv::SetMode>::SharedPtr ptr;
  };

public:
  explicit VirtualSerialNode(const rclcpp::NodeOptions &options) : Node("serial_driver", options) {
    RCLCPP_INFO(this->get_logger(), "Start VirtualSerialNode!");


    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    this->declare_parameter("vision_mode", static_cast<int>(0));

    has_rune_ = this->declare_parameter("has_rune", true);

    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "gimbal_link";

    // Param client
    auto autoaim_set_mode_client_1 =
      this->create_client<auto_aim_interfaces::srv::SetMode>("armor_detector/set_mode");
    set_mode_clients_.emplace(autoaim_set_mode_client_1->get_service_name(),
                              autoaim_set_mode_client_1);
    auto autoaim_set_mode_client_2 =
      this->create_client<auto_aim_interfaces::srv::SetMode>("armor_solver/set_mode");
    set_mode_clients_.emplace(autoaim_set_mode_client_2->get_service_name(),
                              autoaim_set_mode_client_2);
    if (has_rune_) {
      auto client1 = this->create_client<auto_aim_interfaces::srv::SetMode>("rune_detector/set_mode");
      set_mode_clients_.emplace(client1->get_service_name(), client1);
      auto client2 = this->create_client<auto_aim_interfaces::srv::SetMode>("rune_solver/set_mode");
      set_mode_clients_.emplace(client2->get_service_name(), client2);
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(5), [this]() {
      int mode = this->get_parameter("vision_mode").as_int();
      double roll = this->get_parameter("roll").as_double();
      double pitch = this->get_parameter("pitch").as_double();
      double yaw = this->get_parameter("yaw").as_double();
      tf2::Quaternion q;
      q.setRPY(roll * M_PI / 180.0, -pitch * M_PI / 180.0, yaw * M_PI / 180.0);
      transform_stamped_.transform.rotation = tf2::toMsg(q);
      transform_stamped_.header.frame_id = "odom";
      transform_stamped_.child_frame_id = "gimbal_link";
      // serial_receive_data_msg.mode = mode;
      transform_stamped_.header.stamp = this->now();
      tf_broadcaster_->sendTransform(transform_stamped_);
      Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
      Eigen::Vector3d rpy  = getRPY(q_eigen.toRotationMatrix());
      q.setRPY(rpy[0], 0, 0);
      transform_stamped_.transform.rotation = tf2::toMsg(q);
      transform_stamped_.header.frame_id = "odom";
      transform_stamped_.child_frame_id = "odom_rectify";
      tf_broadcaster_->sendTransform(transform_stamped_);

      for (auto &[service_name, client] : set_mode_clients_) {
        if (client.mode.load() != mode && !client.on_waiting.load()) {
          setMode(client, mode);
        }
      }
    });
  }

  void setMode(SetModeClient &client, const uint8_t mode) {
    using namespace std::chrono_literals;

    std::string service_name = client.ptr->get_service_name();
    // Wait for service
    while (!client.ptr->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for %s service...", service_name.c_str());
    }
    if (!client.ptr->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "Service %s is not ready, skipping set mode", service_name.c_str());
      return;
    }
    // Send request
    auto req = std::make_shared<auto_aim_interfaces::srv::SetMode::Request>();
    req->mode = mode;
    client.on_waiting.store(true);
    auto result = client.ptr->async_send_request(
      req, [mode, &client](rclcpp::Client<auto_aim_interfaces::srv::SetMode>::SharedFuture result) {
        client.on_waiting.store(false);
        if (result.get()->success) {
          client.mode.store(mode);
        }
      });
  }

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TransformStamped transform_stamped_;

  bool has_rune_;

  std::unordered_map<std::string, SetModeClient> set_mode_clients_;
  inline Eigen::Vector3d getRPY(const Eigen::Matrix3d& rotation_matrix) {
    return rotation_matrix.eulerAngles(2, 1, 0).reverse();
  }
};
}  // namespace auto_aim_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::VirtualSerialNode)
