import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- This file starts the robot drivers AND the policy node ---
    
    # --- Part 1: Robot Drivers (from your old bringup.launch.py) ---
    # --- 1. Declare the arguments that will be passed to this launch file ---
    model_path_arg = DeclareLaunchArgument(
        'model_path', 
        description='Path to the policy model (.pt), relative to workspace root.'
    )
    goal_state_path_arg = DeclareLaunchArgument(
        'goal_state_path', 
        description='Path to the goal state file (.pkl), relative to workspace root.'
    )

    # --- The Autonomous Rollout ---
    policy_rollout_node = Node(
        package='model_pipeline',
        executable='policy_rollout',
        name='policy_rollout_node',
        output='screen',
        parameters=[{
            # Pass the launch arguments to the node's parameters
            'model_path': LaunchConfiguration('model_path'),
            'goal_state_path': LaunchConfiguration('goal_state_path')
        }]
    )

    return LaunchDescription([
        model_path_arg,
        goal_state_path_arg,
        policy_rollout_node, # Launch the policy node at the same time
    ])