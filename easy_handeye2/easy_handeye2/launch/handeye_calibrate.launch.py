from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    easy_handeye_calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("easy_handeye2"), "launch", "calibrate.launch.py"])
        ),
        launch_arguments={
            "calibration_type": "eye_on_base",
            "name": "my_handeye_calib3",
            "robot_base_frame": "base",
            "robot_effector_frame": "fr3_link8",
            "tracking_base_frame": "camera_link",            # static RealSense base frame
            "tracking_marker_frame": "aruco_marker_frame",   # published by ArUco/AprilTag node
            "freehand_robot_movement": "true"
        }.items()
    )

    return LaunchDescription([easy_handeye_calibrate])
