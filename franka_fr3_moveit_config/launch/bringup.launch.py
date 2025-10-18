import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- This file starts the robot drivers  ---
    
    # --- Robot Drivers (from your old bringup.launch.py) ---
    # --- 1. Declare the arguments that will be passed to this launch file ---
    robot_ip = LaunchConfiguration('robot_ip')
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.1.11')
    
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_fake_hardware_arg = DeclareLaunchArgument('use_fake_hardware', default_value='false')

    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots', 'fr3', 'fr3.urdf.xacro')
    robot_description_config = Command([
        FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=true', ' ee_id:=leap_hand',
        ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware, ' ros2_control:=true'
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    ros2_controllers_path = os.path.join(get_package_share_directory('franka_fr3_moveit_config'), 'config', 'fr3_ros_controllers.yaml')
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

    return LaunchDescription([
        robot_ip_arg,
        use_fake_hardware_arg,
        ros2_control_node,
        robot_state_publisher,
    ] + load_controllers)