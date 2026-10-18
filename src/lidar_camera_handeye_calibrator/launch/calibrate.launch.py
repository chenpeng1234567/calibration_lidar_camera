from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("lidar_camera_handeye_calibrator"),
        "config",
        "calibrator.param.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("param_file", default_value=default_params),
            Node(
                package="lidar_camera_handeye_calibrator",
                executable="handeye_calibrator_node",
                name="lidar_camera_handeye_calibrator",
                output="screen",
                parameters=[LaunchConfiguration("param_file")],
            ),
        ]
    )
