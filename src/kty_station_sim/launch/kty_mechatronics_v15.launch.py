from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("kty_station_sim"))

    auto_repeat = LaunchConfiguration("auto_repeat")
    spawn_interval = LaunchConfiguration("product_spawn_interval_s")
    fill_threshold = LaunchConfiguration("fill_ratio_threshold")
    height_threshold = LaunchConfiguration("max_height_threshold_m")
    show_dashboard = LaunchConfiguration("show_dashboard")
    output_directory = LaunchConfiguration("polygon_output_directory")

    # The contact-surface Gazebo system publishes a compact, explicitly named
    # JSON registry.  StringMsg avoids the Pose_V -> TFMessage frame-name issue
    # observed on the target Jazzy installation.
    pose_registry_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="kty_model_pose_registry_bridge",
        output="screen",
        arguments=[
            "/kty/mech/model_pose_registry_json"
            "@std_msgs/msg/String[gz.msgs.StringMsg",
        ],
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / "kty_mechatronics_v13.launch.py")
        ),
        launch_arguments={
            "auto_repeat": auto_repeat,
            "product_spawn_interval_s": spawn_interval,
            "fill_ratio_threshold": fill_threshold,
            "max_height_threshold_m": height_threshold,
            "show_dashboard": show_dashboard,
            "polygon_output_directory": output_directory,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_repeat", default_value="true"),
            DeclareLaunchArgument(
                "product_spawn_interval_s",
                default_value="1.90",
            ),
            DeclareLaunchArgument("fill_ratio_threshold", default_value="0.70"),
            DeclareLaunchArgument(
                "max_height_threshold_m",
                default_value="0.280",
            ),
            DeclareLaunchArgument("show_dashboard", default_value="false"),
            DeclareLaunchArgument(
                "polygon_output_directory",
                default_value="~/.ros/kty_vision",
            ),
            pose_registry_bridge,
            base_launch,
        ]
    )
