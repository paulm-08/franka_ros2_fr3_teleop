#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    Shutdown,
    RegisterEventHandler
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch.event_handlers import OnProcessStart, OnShutdown

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'mock_sensor_commands'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    

    mode_param_name = 'mode'
    mode = LaunchConfiguration(mode_param_name)
    mode_parameter = DeclareLaunchArgument(
        mode_param_name,
        default_value='None',
        description='Mode for the pose tracking node: "record" to record poses, "replay" to replay recorded poses'
    )

    hand_parameter_name = 'hand'
    hand = 'true'
    # hand = LaunchConfiguration(hand_parameter_name)
    hand_parameter = DeclareLaunchArgument(
        hand_parameter_name,
        default_value='true',
        description='true if the robot has a hand, false if it is a bare robot arm'
    )
    ee_id_parameter_name = 'ee_id'
    ee_id = 'leap_hand'  # Default end-effector ID
    # ee_id = LaunchConfiguration(ee_id_parameter_name)
    ee_id_parameter = DeclareLaunchArgument(
        ee_id_parameter_name,
        default_value='leap_hand',
        description='End-effector hand type (e.g., leap_hand or franka_gripper)'
    )

    # Load pose tracking PID settings
    pose_tracking_settings = load_yaml(
        "franka_fr3_moveit_config", "config/pose_tracking_settings.yaml"
    )
    # Load your main servo config (with pose_tracking block included)
    servo_yaml = load_yaml(
        "franka_fr3_moveit_config", "config/servo_fr3.yaml"
    )
    if servo_yaml is None or pose_tracking_settings is None:
        raise RuntimeError("Could not load servo_fr3.yaml or pose_tracking_settings.yaml!")
    
    # Combine parameters for pose tracking node
    servo_params = {
        "moveit_servo": {
            **servo_yaml["moveit_servo"],
            **pose_tracking_settings
        }
    }

    # Command-line arguments
    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )

    # planning_context
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file,
         ' hand:=', hand,
         ' ee_id:=', ee_id,
         ' robot_ip:=', robot_ip,
         ' use_fake_hardware:=', use_fake_hardware,
         ' mock_sensor_commands:=', fake_sensor_commands,
         ' ros2_control:=true'])

    robot_description = {'robot_description': ParameterValue(
        robot_description_config, value_type=str)}

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'srdf',
        'fr3_arm.srdf.xacro'
    )

    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ',
         franka_semantic_xacro_file, ' hand:=',hand,' ee_id:=',ee_id]
    )

    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}

    kinematics_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/kinematics.yaml'
    )

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # # Trajectory Execution Functionality
    # moveit_simple_controllers_yaml = load_yaml(
    #     'franka_fr3_moveit_config', 'config/fr3_controllers.yaml'
    # )
    # moveit_controllers = {
    #     'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
    #     'moveit_controller_manager': 'moveit_simple_controller_manager'
    #                                  '/MoveItSimpleControllerManager',
    # }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # # Start the actual move_group node/action server
    # run_move_group_node = Node(
    #     package='moveit_ros_move_group',
    #     executable='move_group',
    #     output='screen',
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         kinematics_yaml,
    #         ompl_planning_pipeline_config,
    #         trajectory_execution,
    #         # moveit_controllers,
    #         planning_scene_monitor_parameters,
    #     ],
    # )

    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'franka_fr3_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'config',
        'fr3_ros_controllers.yaml',
    )
    
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    # Load controllers
    load_controllers = []
    for controller in ['fr3_arm_controller', 'joint_state_broadcaster']:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {}'.format(
                    controller)],
                shell=True,
                output='screen',
            )
        ]

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            # {'source_list': ['franka/joint_states', 'fr3_gripper/joint_states'], 'rate': 30}],
            {'source_list': ['franka/joint_states'], 'rate': 30}],
    )

    franka_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka_robot_state_broadcaster'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    robot_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        description='Hostname or IP address of the robot.',
        default_value='192.168.1.11'  # Default IP address of the robot
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='false',
        description='Use fake hardware')
    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value='false',
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name))
    # gripper_launch_file = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution(
    #         [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
    #     launch_arguments={'robot_ip': robot_ip,
    #                       use_fake_hardware_parameter_name: use_fake_hardware}.items(),
    # )

    # Servo node
    # This is the main node for pose tracking, not needed if using the pose tracking executable
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
        output="screen",
    )

    # Pose tracking node
    pose_tracking_node = Node(
        package="moveit_servo",
        executable="servo_pose_tracking",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )
    
    # Process killer to ensure the port 5005 is free (camera)
    process_killer = ExecuteProcess(
        cmd=['fuser', '-k', '5005/udp'],
        output='screen'
    )

    viualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='true',
        description='Enable visualization'
    )

    optimizer_arg = DeclareLaunchArgument(
        'optimizer',
        default_value='adapted',
        description='Type of optimizer to use: "original" for DexPilot, "adapted" for DexPilotAdapted'
    )

    # Vive pose publisher
    # This node publishes the pose of the Vive trackers to the /target_pose topic and starts the retargeting process
    vive_pose_publisher = ExecuteProcess(
        cmd=['ros2', 'run', 'fr3_leap_teleop', 'teleop_vive_leap_ros2',
            #  '--hand', LaunchConfiguration('hand'),
            #  '--ee_id', LaunchConfiguration('ee_id'),
             '--optimizer', LaunchConfiguration('optimizer'),
            #  '--camera', 'generic',
             '--visualize', LaunchConfiguration('visualize'),],
        output='screen',
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'replay'"]))
    )

    # Bag recorder: record the /target_pose topic to a ROS2 bag
    remove_old_bag = ExecuteProcess(
        cmd=['rm', '-rf', '/tmp/pose_tracking_bag'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'record'"]))
    )
    bag_recorder = TimerAction(
        period=5.0,  # seconds
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record', '-o', '/tmp/pose_tracking_bag',
                    '/target_pose'],
                output='screen',
                condition=IfCondition(PythonExpression(["'", mode, "' == 'record'"]))
            )
        ]
    )

    # Bag replayer: play the bag only
    bag_replayer = TimerAction(
        period=5.0,  # seconds
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', '/tmp/pose_tracking_bag'],
                output='screen',
                condition=IfCondition(PythonExpression(["'", mode, "' == 'replay'"]))
            )
        ]
    )

    # Reset tracking origin trigger when pose tracking starts
    # This is needed to reset the tracking origin to the current pose of the robot
    # The trigger was moved to the pose tracking executable to ensure it is called right before the pose tracking starts, so it is not needed here
    origin_reset_trigger = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=pose_tracking_node,
            on_start=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/reset_tracking_origin',
                        'std_srvs/srv/Trigger', '{}'
                    ],
                    output='screen'
                )
            ]
        )
    )

    # Trigger servo node to start servoing after 5 seconds
    # This is not needed if using the pose tracking executable
    servo_node_trigger = TimerAction(
        period=5.0,  # seconds
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                     '/servo_node/start_servo', 'std_srvs/srv/Trigger', '{}'],
                output='screen'
            )
        ]
    )

    # Realsense camera node to publish frames to /tf and the images to camera topics
    # Realsense cameras are started in the recorder process to avoid using ROS2 topics
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch",
                "rs_launch.py"
            ])
        ),
        launch_arguments={
            "serial_no": "_151422254571"  # Change this to your camera's serial number
        }.items(),
    )

    # Aruco marker pose publisher
    # This node publishes the pose of the ArUco marker to the /aruco_marker_frame topic
    # It is used for hand-eye calibration, so it is not needed if you are not calibrating the camera
    # If using an Aruco marker for the task, do it in data processing to avoid using ROS2 topics
    aruco_tf_publisher = Node(
        package='easy_handeye2',
        executable='aruco_tf_publisher',
        name='aruco_tf_publisher',
        output='screen',
        parameters=[
            {'marker_id': 0,  # Change this to your marker ID
             'marker_size': 0.10,  # Size of the marker in meters
             'camera_frame': 'camera_link',  # Frame of the camera
             'tracking_marker_frame': 'aruco_marker_frame',  # Frame for the marker
            #  'camera_matrix': [600, 0, 320, 0, 600, 240, 0, 0, 1],  # Example camera matrix
            #  'dist_coeffs': [0.1, -0.25, 0, 0]  # Example distortion coefficients
            }
        ]
    )

    # easy_handeye2 hand-eye calibration publisher
    # This node publishes the transform between the camera and the robot base frame to /tf
    handeye_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("easy_handeye2"), "launch", "publish.launch.py"])
        ),
        launch_arguments={
            'name': 'my_handeye_calib2',
        }.items()
    )

    # 9DTact sensor node
    # This node publishes the sensor data to the /rectify_crop_image, /deformation_representation, and /height_map topics
    # The recording script directly gets the image from the sensor to avoid using ROS2 topics
    sensor_node = Node(
        package='9dtact',
        executable='sensor_node',
        name='sensor_node',
        output='screen',
    )

    out_directory_arg = DeclareLaunchArgument(
        'out_directory',
        default_value='/home/user/recorded_data/test',
        description='Output directory for recorded data'
    )

    haptic_arg = DeclareLaunchArgument(
        'haptic',
        default_value='true',
        description='Enable haptic feedback'
    )

    # Recorder script
    # This script records the FR3 and LEAP hand joint states, RGB and depth images from the Realsense cameras and 9DTact sensor image 
    recorder = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'fr3_leap_recorder', 'fr3_leap_recorder',
            '--out_directory', LaunchConfiguration('out_directory'),
            '--haptic', LaunchConfiguration('haptic'),
            '--visualize', LaunchConfiguration('visualize'),
        ],
        output='screen',
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'replay'"]))
    )

    return LaunchDescription([
        robot_arg,
        mode_parameter,
        hand_parameter,
        ee_id_parameter,
        use_fake_hardware_arg,
        fake_sensor_commands_arg,
        db_arg,
        optimizer_arg,
        viualize_arg,
        out_directory_arg,
        haptic_arg,
        # rviz_node,
        robot_state_publisher,
        # run_move_group_node,
        ros2_control_node,
        joint_state_publisher,
        franka_robot_state_broadcaster,
        # gripper_launch_file,
        # servo_node,
        pose_tracking_node,
        process_killer,
        vive_pose_publisher,
        # origin_reset_trigger,
        # servo_node_trigger,
        # remove_old_bag,
        # bag_recorder,
        # bag_replayer,
        # realsense_node,
        # aruco_tf_publisher,
        # handeye_node,
        # sensor_node
        recorder,
    ] + load_controllers
    )