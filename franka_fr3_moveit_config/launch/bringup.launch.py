import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution # <-- Added PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
    # --- This file starts the robot drivers  ---
    
    # --- Robot Drivers (from your old bringup.launch.py) ---
    # --- 1. Declare the arguments that will be passed to this launch file ---
    robot_ip = LaunchConfiguration('robot_ip')
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.1.11')
    
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_fake_hardware_arg = DeclareLaunchArgument('use_fake_hardware', default_value='false')

    # NEW ARGUMENT: Allows selection of the controller config file
    controllers_yaml_file = LaunchConfiguration('controllers_yaml_file')
    controllers_yaml_file_arg = DeclareLaunchArgument(
        'controllers_yaml_file', 
        default_value='fr3_ros_controllers_rollout.yaml', # Default to the original file
        description='Name of the ROS 2 controllers YAML file (e.g., fr3_ros_controllers_rollout_fixed.yaml)'
    )
    
    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots', 'fr3', 'fr3.urdf.xacro')
    robot_description_config = Command([
        FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=true', ' ee_id:=leap_hand',
        ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware, ' ros2_control:=true'
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    # THE FIX IS HERE: Use PathJoinSubstitution instead of os.path.join()
    ros2_controllers_path = PathJoinSubstitution([
        get_package_share_directory('franka_fr3_moveit_config'), 
        'config', 
        controllers_yaml_file
    ])
    
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output='screen',
        on_exit=Shutdown(),
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='both', parameters=[robot_description]
    )

    load_controllers = []
    for controller in ['fr3_arm_controller', 'joint_state_broadcaster']:
        load_controllers += [
            ExecuteProcess(cmd=[f'ros2 run controller_manager spawner {controller}'], shell=True, output='screen')
        ]

    # SERVO
    hand = 'true'
    ee_id = 'leap_hand'  # Default end-effector ID

    fake_sensor_commands_parameter_name = 'mock_sensor_commands'
    use_fake_hardware_parameter_name = 'use_fake_hardware'

    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='false',
        description='Use fake hardware')
    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value='false',
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name))


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

    return LaunchDescription([
        robot_ip_arg,
        use_fake_hardware_arg,
        fake_sensor_commands_arg,
        controllers_yaml_file_arg, # Declare the new argument
        ros2_control_node,
        robot_state_publisher,
        # servo_node,
        # servo_node_trigger,
        # pose_tracking_node
    ] + load_controllers)