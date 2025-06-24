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

#include <franka_example_controllers/cartesian_pose_example_controller.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();

  return config;
}

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  // add the robot time interface
  config.names.push_back(arm_id_ + "/robot_time");
  return config;
}

controller_interface::return_type CartesianPoseExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) 
{
  if (initialization_flag_) {
    // Get initial orientation and translation
    std::tie(orientation_, position_) =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    initial_robot_time_ = state_interfaces_.back().get_value();
    
    elapsed_time_ = 0.0;

    // Store the initial pose
    initial_position_ = position_;
    initial_orientation_ = orientation_;
    filtered_position_ = position_;
    filtered_orientation_ = orientation_;

    initialization_flag_ = false;
  } else {
    robot_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = robot_time_ - initial_robot_time_;
  }

  // double radius = 0.1;
  // double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_));

  // double delta_x = radius * std::sin(angle);
  // double delta_z = radius * (std::cos(angle) - 1);

  // Eigen::Quaterniond new_orientation;
  // Eigen::Vector3d new_position;

  // new_position = position_;
  // new_orientation = orientation_;

  // new_position(0) -= delta_x;
  // new_position(2) -= delta_z;

  // if (franka_cartesian_pose_->setCommand(new_orientation, new_position)) {
  //   return controller_interface::return_type::OK;
  // } else {
  //   RCLCPP_FATAL(get_node()->get_logger(),
  //                "Set command failed. Did you activate the elbow command interface?");
  //   return controller_interface::return_type::ERROR;
  // }

  double alpha = 0.005; // Smoothing factor, tune as needed


  // Read the latest pose message from the buffer
  auto msg = pose_buffer_.readFromRT();
  if (!msg || !is_valid_pose(*msg)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                        "Waiting for the first valid target pose...");
    target_position_ = initial_position_;
    target_orientation_ = initial_orientation_;
    return controller_interface::return_type::OK;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Received target pose message.");
  RCLCPP_INFO(get_node()->get_logger(), "Target pose: [%f, %f, %f]",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  RCLCPP_INFO(get_node()->get_logger(), "Target orientation: [%f, %f, %f, %f]",
              msg->pose.orientation.w, msg->pose.orientation.x,
              msg->pose.orientation.y, msg->pose.orientation.z);

  Eigen::Vector3d msg_position(
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z);
  Eigen::Quaterniond msg_orientation(
    msg->pose.orientation.w,
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z);

  // If message received, update target
  if (!offset_computed_) {
    // Compute the offset between robot and first message
    pose_offset_ = initial_position_ - msg_position;
    orientation_offset_ = initial_orientation_ * msg_orientation.inverse();
    offset_computed_ = true;
  }

  // Apply the offset to every message
  target_position_ = msg_position + pose_offset_;
  target_orientation_ = orientation_offset_ * msg_orientation;

  // // After computing target_position_ and target_orientation_:
  // filtered_position = (1 - alpha) * filtered_position + alpha * target_position_;
  // filtered_orientation = filtered_orientation.slerp(alpha, target_orientation_);

  // --- Rate limiting parameters ---
  const double dt = 0.001; // 1000Hz
  const double max_velocity = 0.3;      // m/s
  const double max_acceleration = 0.9;  // m/s^2
  const double max_jerk = 450.0;       // m/s^3

  // // --- Rate-limited orientation ---
  // double angle = filtered_orientation_.angularDistance(target_orientation_);
  // if (angle > max_angle_delta) {
  //   filtered_orientation_ = filtered_orientation_.slerp(
  //       max_angle_delta / angle, target_orientation_);
  // } else {
  //   filtered_orientation_ = target_orientation_;
  // }

  // --- Compute desired velocity and acceleration ---
  Eigen::Vector3d desired_velocity = (target_position_ - filtered_position_) / dt;
  Eigen::Vector3d velocity_delta = desired_velocity - filtered_velocity_;

  // --- Jerk limiting ---
  Eigen::Vector3d desired_acceleration = velocity_delta / dt;
  Eigen::Vector3d jerk = (desired_acceleration - filtered_acceleration_) / dt;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(jerk(i)) > max_jerk) {
      jerk(i) = std::copysign(max_jerk, jerk(i));
    }
  }

  // --- Acceleration limiting ---
  Eigen::Vector3d new_acceleration = filtered_acceleration_ + jerk * dt;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(new_acceleration(i)) > max_acceleration) {
      new_acceleration(i) = std::copysign(max_acceleration, new_acceleration(i));
    }
  }

  // --- Velocity limiting ---
  Eigen::Vector3d new_velocity = filtered_velocity_ + new_acceleration * dt;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(new_velocity(i)) > max_velocity) {
      new_velocity(i) = std::copysign(max_velocity, new_velocity(i));
    }
  }

  // --- Position update ---
  filtered_position_ += new_velocity * dt;

  // --- Save for next cycle ---
  filtered_velocity_ = new_velocity;
  filtered_acceleration_ = new_acceleration;

  RCLCPP_INFO(get_node()->get_logger(), "Current robot position: [%f, %f, %f]",
              position_(0), position_(1), position_(2));
  RCLCPP_INFO(get_node()->get_logger(), "Target robot position: [%f, %f, %f]",
              target_position_(0), target_position_(1), target_position_(2));
  RCLCPP_INFO(get_node()->get_logger(), "Filtered robot position: [%f, %f, %f]",
              filtered_position_(0), filtered_position_(1), filtered_position_(2));

  RCLCPP_INFO(get_node()->get_logger(), "Current robot orientation: [%f, %f, %f, %f]",
              orientation_.w(), orientation_.x(), orientation_.y(), orientation_.z());
  RCLCPP_INFO(get_node()->get_logger(), "Target robot orientation: [%f, %f, %f, %f]",
              target_orientation_.w(), target_orientation_.x(), target_orientation_.y(),
              target_orientation_.z());
  RCLCPP_INFO(get_node()->get_logger(), "Filtered robot orientation: [%f, %f, %f, %f]",
              filtered_orientation_.w(), filtered_orientation_.x(),
              filtered_orientation_.y(), filtered_orientation_.z());


  // // Interpolate smoothly toward target (low-pass filter)
  // // double alpha = std::min(1.0, period.seconds() * 15.0);  // adjust filter rate
  // current_position_ = current_position_ + alpha * (target_position_ - current_position_);
  // current_orientation_ = current_orientation_.slerp(alpha, target_orientation_);


if (franka_cartesian_pose_->setCommand(target_orientation_, filtered_position_)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

bool is_valid_pose(const geometry_msgs::msg::PoseStamped& msg) {
  // Check if position is not all zeros
  if (msg.pose.position.x == 0.0 &&
      msg.pose.position.y == 0.0 &&
      msg.pose.position.z == 0.0) {
    return false;
  }
  // Check if orientation is not all zeros (invalid quaternion)
  if (msg.pose.orientation.w == 0.0 &&
      msg.pose.orientation.x == 0.0 &&
      msg.pose.orientation.y == 0.0 &&
      msg.pose.orientation.z == 0.0) {
    return false;
  }
  return true;
}

CallbackReturn CartesianPoseExampleController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);
  future_result.wait_for(robot_utils::time_out);

  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());
  
  pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/target_pose", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      pose_buffer_.writeFromNonRT(*msg);
    });

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerInterface)
