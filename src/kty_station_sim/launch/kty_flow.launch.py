from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("kty_station_sim"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = package_share / "worlds" / "kty_flow.sdf"

    product_count = LaunchConfiguration("product_count")
    auto_repeat = LaunchConfiguration("auto_repeat")
    approach_duration = LaunchConfiguration("approach_duration_s")
    spawn_interval = LaunchConfiguration("product_spawn_interval_s")
    outfeed_duration = LaunchConfiguration("outfeed_duration_s")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 3 {world}"}.items(),
    )

    controller = Node(
        package="kty_station_sim",
        executable="flow_cycle",
        name="kty_flow_cycle",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "world_name": "kty_flow",
                "product_count": ParameterValue(product_count, value_type=int),
                "auto_repeat": ParameterValue(auto_repeat, value_type=bool),
                "approach_duration_s": ParameterValue(
                    approach_duration,
                    value_type=float,
                ),
                "product_spawn_interval_s": ParameterValue(
                    spawn_interval,
                    value_type=float,
                ),
                "outfeed_duration_s": ParameterValue(
                    outfeed_duration,
                    value_type=float,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("product_count", default_value="6"),
            DeclareLaunchArgument("auto_repeat", default_value="false"),
            DeclareLaunchArgument("approach_duration_s", default_value="3.0"),
            DeclareLaunchArgument(
                "product_spawn_interval_s",
                default_value="0.9",
            ),
            DeclareLaunchArgument("outfeed_duration_s", default_value="3.0"),
            gazebo,
            controller,
        ]
    )
