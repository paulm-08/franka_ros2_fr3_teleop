// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <Eigen/Dense>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_example_controllers/robot_utils.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The cartesian pose example controller
 */
class CartesianPoseExampleController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
 private:
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;
  Eigen::Vector3d target_position_;
  Eigen::Quaterniond target_orientation_;
  Eigen::Vector3d initial_position_;
  Eigen::Quaterniond initial_orientation_;
  Eigen::Vector3d pose_offset_;
  Eigen::Quaterniond orientation_offset_;
  bool offset_computed_ = false;
  Eigen::Vector3d filtered_position_;
  Eigen::Quaterniond filtered_orientation_;
  Eigen::Vector3d filtered_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d filtered_acceleration_{Eigen::Vector3d::Zero()};

  realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped> pose_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  const bool k_elbow_activated_{false};
  bool initialization_flag_{true};

  double elapsed_time_{0.0};
  double initial_robot_time_{0.0};
  double robot_time_{0.0};
  std::string robot_description_;
  std::string arm_id_;
};

bool is_valid_pose(const geometry_msgs::msg::PoseStamped& msg);

}  // namespace franka_example_controllers
