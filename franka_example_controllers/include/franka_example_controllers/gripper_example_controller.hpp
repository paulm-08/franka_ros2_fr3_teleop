// Copyright (c) 2025 Franka Robotics GmbH
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

#include <string>

#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include "controller_interface/controller_interface.hpp"
#include "franka_example_controllers/robot_utils.hpp"
#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/move.hpp"
#include "std_srvs/srv/trigger.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The Gripper example controller
 */
class GripperExampleController : public controller_interface::ControllerInterface {
  void toggleGripperState();
  bool openGripper();
  void graspGripper();

 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_init() override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  pinocchio::Model robot_model_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

  std::shared_ptr<rclcpp_action::Client<franka_msgs::action::Grasp>> gripper_grasp_action_client_;
  std::shared_ptr<rclcpp_action::Client<franka_msgs::action::Move>> gripper_move_action_client_;
  std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> gripper_stop_client_;

  // poorly named struct SendGoalOptions - should be SendGoalCallbacks (or something similar)
  rclcpp_action::Client<franka_msgs::action::Move>::SendGoalOptions move_goal_options_;
  rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions grasp_goal_options_;
  void assignMoveGoalOptionsCallbacks();
  void assignGraspGoalOptionsCallbacks();
  std::shared_ptr<pinocchio::Data> robot_data_;

  std::string arm_id_;
  const int num_joints = 7;
};

}  // namespace franka_example_controllers
