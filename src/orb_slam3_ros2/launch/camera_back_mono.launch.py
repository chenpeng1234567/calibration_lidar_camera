from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    orb_root = LaunchConfiguration("orb_slam3_root")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "orb_slam3_root",
                default_value="/root/autoware/src/ys/calibration/ORB_SLAM3",
            ),
            DeclareLaunchArgument(
                "image_topic", default_value="/rect/camera_back/image_gray"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/rect/camera_back/camera_info"
            ),
            DeclareLaunchArgument("use_viewer", default_value="false"),
            DeclareLaunchArgument("camera_fps", default_value="15"),
            Node(
                package="orb_slam3_ros2",
                executable="mono_node",
                name="orb_slam3_camera_back",
                namespace="orb_slam3/camera_back",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "vocabulary_file": PathJoinSubstitution(
                            [orb_root, "Vocabulary", "ORBvoc.txt"]
                        ),
                        "settings_file": "",
                        "generated_settings_file": "/tmp/orb_slam3_camera_back.yaml",
                        "world_frame": "orb_world",
                        "camera_frame": "camera_back/camera_optical_link",
                        "assume_rectified": True,
                        "use_viewer": ParameterValue(
                            LaunchConfiguration("use_viewer"), value_type=bool
                        ),
                        "publish_tf": False,
                        "camera_fps": ParameterValue(
                            LaunchConfiguration("camera_fps"), value_type=int
                        ),
                        "orb.n_features": 2500,
                        "orb.ini_th_fast": 15,
                        "orb.min_th_fast": 5,
                        "orb.scale_factor": 1.2,
                        "orb.n_levels": 8,
                    }
                ],
            ),
        ]
    )
