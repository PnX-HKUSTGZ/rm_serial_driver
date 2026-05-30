// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.

// std
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <rclcpp/executors.hpp>
#include <thread>
// ros2
#include <Eigen/Eigen>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
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

  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;

  struct DetectorParamClientEntry {
    std::string node_name;
    rclcpp::AsyncParametersClient::SharedPtr client;
    ResultFuturePtr future;
    std::string inflight_param_name;
    std::string inflight_param_value;
    std::unordered_map<std::string, bool> synced_params;
  };

public:
  explicit VirtualSerialNode(const rclcpp::NodeOptions &options) : Node("serial_driver", options) {
    RCLCPP_INFO(this->get_logger(), "Start VirtualSerialNode!");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    this->declare_parameter("vision_mode", static_cast<int>(0));
    this->declare_parameter("color", static_cast<int>(0));
    this->declare_parameter("has_rune", true);
    this->declare_parameter("roll", 0.0);
    this->declare_parameter("pitch", 0.0);
    this->declare_parameter("yaw", 0.0);

    transform_stamped_.header.frame_id = "odom_aim";
    transform_stamped_.child_frame_id = "gimbal_link";

    // Mode clients
    auto autoaim_set_mode_client_1 =
      this->create_client<auto_aim_interfaces::srv::SetMode>("/armor_detector_main/set_mode");
    auto autoaim_set_mode_client_2 =
      this->create_client<auto_aim_interfaces::srv::SetMode>("/armor_tracker/set_mode");
    set_mode_clients_.emplace(autoaim_set_mode_client_1->get_service_name(),
                              autoaim_set_mode_client_1);
    set_mode_clients_.emplace(autoaim_set_mode_client_2->get_service_name(),
                              autoaim_set_mode_client_2);
    has_rune_ = this->get_parameter("has_rune").as_bool();
    if (has_rune_) {
      auto client1 = this->create_client<auto_aim_interfaces::srv::SetMode>("/rune_detector/set_mode");
      set_mode_clients_.emplace(client1->get_service_name(), client1);
      auto client2 = this->create_client<auto_aim_interfaces::srv::SetMode>("/rune_solver/set_mode");
      set_mode_clients_.emplace(client2->get_service_name(), client2);
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(5), [this]() {
      const auto now = std::chrono::steady_clock::now();
      if (
        last_detector_discovery_time_ == std::chrono::steady_clock::time_point{} ||
        now - last_detector_discovery_time_ >= std::chrono::milliseconds(500)) {
        refreshDetectorParamClients();
        last_detector_discovery_time_ = now;
      }

      int mode = this->get_parameter("vision_mode").as_int();
      int color = this->get_parameter("color").as_int();
      double roll = this->get_parameter("roll").as_double();
      double pitch = this->get_parameter("pitch").as_double();
      double yaw = this->get_parameter("yaw").as_double();

      if (color != previous_receive_color_) {
        for (auto &entry : detector_param_clients_) {
          entry.synced_params["detect_color"] = false;
        }
        initial_set_param_["detect_color"] = false;
        previous_receive_color_ = color;
      }

      if (!initial_set_param_["detect_color"]) {
        setParam(rclcpp::Parameter("detect_color", color));
      }

      tf2::Quaternion q;
      q.setRPY(roll * M_PI / 180.0, -pitch * M_PI / 180.0, yaw * M_PI / 180.0);
      transform_stamped_.transform.rotation = tf2::toMsg(q);
      transform_stamped_.header.frame_id = "odom_aim";
      transform_stamped_.child_frame_id = "gimbal_link";
      // serial_receive_data_msg.mode = mode;
      transform_stamped_.header.stamp = this->now();
      tf_broadcaster_->sendTransform(transform_stamped_);
      Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
      Eigen::Vector3d rpy  = getRPY(q_eigen.toRotationMatrix());
      q.setRPY(rpy[0], 0, 0);
      transform_stamped_.transform.rotation = tf2::toMsg(q);
      transform_stamped_.header.frame_id = "odom_aim";
      transform_stamped_.child_frame_id = "odom_rectify";
      tf_broadcaster_->sendTransform(transform_stamped_);

      for (auto &entry : set_mode_clients_) {
        auto &client = entry.second;
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
    auto future = client.ptr->async_send_request(
      req, [mode, &client](rclcpp::Client<auto_aim_interfaces::srv::SetMode>::SharedFuture result) {
        client.on_waiting.store(false);
        if (result.get()->success) {
          client.mode.store(mode);
        }
      });
    (void)future;
  }

  void setParam(const rclcpp::Parameter & param)
  {
    if (detector_param_clients_.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No armor_detector_* node found, skipping parameter set");
      return;
    }

    bool any_service_ready = false;
    bool all_discovered_clients_synced = true;
    for (auto &entry : detector_param_clients_) {
      if (
        entry.future.valid() &&
        entry.future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        bool request_success = true;
        for (const auto &result : entry.future.get()) {
          if (!result.successful) {
            request_success = false;
            RCLCPP_ERROR(
              get_logger(), "Failed to set parameter %s on %s: %s",
              entry.inflight_param_name.c_str(), entry.node_name.c_str(), result.reason.c_str());
            break;
          }
        }
        const bool matches_current_param =
          entry.inflight_param_name == param.get_name() &&
          entry.inflight_param_value == param.value_to_string();
        if (request_success && !entry.inflight_param_name.empty() && matches_current_param) {
          entry.synced_params[entry.inflight_param_name] = true;
          RCLCPP_INFO(
            get_logger(), "Successfully set %s on %s!", entry.inflight_param_name.c_str(),
            entry.node_name.c_str());
        }
        entry.future = ResultFuturePtr{};
        entry.inflight_param_name.clear();
        entry.inflight_param_value.clear();
      }

      if (!entry.client->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Service %s/set_parameters not ready, skipping parameter set",
          entry.node_name.c_str());
        all_discovered_clients_synced = false;
        continue;
      }
      any_service_ready = true;

      if (entry.synced_params[param.get_name()]) {
        continue;
      }

      all_discovered_clients_synced = false;
      if (!entry.future.valid()) {
        RCLCPP_INFO(
          get_logger(), "Setting %s to %s on %s...", param.get_name().c_str(),
          param.value_to_string().c_str(), entry.node_name.c_str());
        entry.inflight_param_name = param.get_name();
        entry.inflight_param_value = param.value_to_string();
        entry.future = entry.client->set_parameters({param});
      }
    }

    initial_set_param_[param.get_name()] = any_service_ready && all_discovered_clients_synced;
  }

  void refreshDetectorParamClients()
  {
    std::vector<std::string> detector_nodes;
    const auto service_names_and_types = this->get_service_names_and_types();
    constexpr const char * kSetParametersSuffix = "/set_parameters";
    const std::size_t suffix_len = std::char_traits<char>::length(kSetParametersSuffix);

    for (const auto &service_entry : service_names_and_types) {
      const auto &service_name = service_entry.first;
      if (service_name.size() <= suffix_len) {
        continue;
      }
      if (
        service_name.compare(service_name.size() - suffix_len, suffix_len, kSetParametersSuffix) !=
        0) {
        continue;
      }

      const std::string detector_node = service_name.substr(0, service_name.size() - suffix_len);
      const std::size_t leaf_pos = detector_node.find_last_of('/');
      const std::string detector_leaf_name =
        (leaf_pos == std::string::npos) ? detector_node : detector_node.substr(leaf_pos + 1);
      if (detector_leaf_name.rfind("armor_detector_", 0) == 0) {
        detector_nodes.push_back(detector_node);
      }
    }

    std::sort(detector_nodes.begin(), detector_nodes.end());
    detector_nodes.erase(std::unique(detector_nodes.begin(), detector_nodes.end()), detector_nodes.end());

    for (const auto &detector_node : detector_nodes) {
      const auto existed = std::find_if(
        detector_param_clients_.begin(), detector_param_clients_.end(),
        [&detector_node](const DetectorParamClientEntry &entry) {
          return entry.node_name == detector_node;
        });
      if (existed != detector_param_clients_.end()) {
        continue;
      }

      detector_param_clients_.push_back(DetectorParamClientEntry{
        detector_node,
        std::make_shared<rclcpp::AsyncParametersClient>(this, detector_node),
        ResultFuturePtr{},
        std::string{},
        std::string{},
        std::unordered_map<std::string, bool>{}});
      initial_set_param_["detect_color"] = false;
      RCLCPP_INFO(get_logger(), "Discovered detector parameter client: %s", detector_node.c_str());
    }
  }

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TransformStamped transform_stamped_;
  
  // Param clients to set detect_color on armor_detector_* nodes
  std::unordered_map<std::string, bool> initial_set_param_;
  int previous_receive_color_ = -1;
  std::vector<DetectorParamClientEntry> detector_param_clients_;
  std::chrono::steady_clock::time_point last_detector_discovery_time_;

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
