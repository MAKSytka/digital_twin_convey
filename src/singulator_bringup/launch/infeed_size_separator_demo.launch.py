from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    gazebo_share = Path(get_package_share_directory("singulator_gazebo"))
    bringup_share = Path(get_package_share_directory("singulator_bringup"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))

    world = gazebo_share / "worlds" / "infeed_size_separator_demo.sdf"

    spawn_mode = LaunchConfiguration("spawn_mode")
    target_rate = LaunchConfiguration("target_rate_boxes_per_sec")
    maximum_items = LaunchConfiguration("maximum_items")
    small_probability = LaunchConfiguration("small_item_probability")
    seed = LaunchConfiguration("seed")
    conveyor_speed = LaunchConfiguration("conveyor_speed_mps")
    screen_surface_speed = LaunchConfiguration(
        "screen_surface_speed_mps"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 3 {world}"}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="separator_demo_bridge",
        output="screen",
        parameters=[
            {
                "config_file": str(
                    bringup_share
                    / "config"
                    / "bridge_separator_demo.yaml"
                )
            }
        ],
    )

    controller = Node(
        package="singulator_control",
        executable="separator_demo_controller",
        name="separator_demo_controller",
        output="screen",
        parameters=[
            {
                "conveyor_speed_mps": ParameterValue(
                    conveyor_speed,
                    value_type=float,
                ),
                "screen_surface_speed_mps": ParameterValue(
                    screen_surface_speed,
                    value_type=float,
                ),
                "shaft_collision_radius_m": 0.025,
                "publish_rate_hz": 30.0,
                "use_sim_time": True,
            }
        ],
    )

    cleanup = Node(
        package="singulator_sim",
        executable="separator_demo_cleanup",
        name="separator_demo_cleanup",
        output="screen",
        parameters=[
            {
                "world_name": "infeed_size_separator_demo",
                "route_decision_x_m": 0.85,
                "upper_exit_x_m": 3.45,
                "lower_exit_x_m": 3.52,
                "upper_route_min_z_m": -0.05,
                "lower_route_max_z_m": -0.08,
                "fallen_z_m": -0.85,
                "lateral_limit_m": 1.80,
                "maximum_lifetime_s": 30.0,
                "statistics_period_s": 2.0,
                "use_sim_time": True,
            }
        ],
    )

    spawner = Node(
        package="singulator_sim",
        executable="separator_demo_spawner",
        name="separator_demo_spawner",
        output="screen",
        parameters=[
            {
                "world_name": "infeed_size_separator_demo",
                "spawn_x": -3.20,
                "belt_top_z": 0.08,
                "conveyor_half_width_m": 1.25,
                "target_rate_boxes_per_sec": ParameterValue(
                    target_rate,
                    value_type=float,
                ),
                "spawn_mode": spawn_mode,
                "maximum_items": ParameterValue(
                    maximum_items,
                    value_type=int,
                ),
                "small_item_probability": ParameterValue(
                    small_probability,
                    value_type=float,
                ),
                "cutoff_m": 0.070,
                "seed": ParameterValue(seed, value_type=int),
                "statistics_every_items": 20,
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "spawn_mode",
                default_value="continuous",
            ),
            DeclareLaunchArgument(
                "target_rate_boxes_per_sec",
                default_value="4.0",
            ),
            DeclareLaunchArgument(
                "maximum_items",
                default_value="100",
            ),
            DeclareLaunchArgument(
                "small_item_probability",
                default_value="0.20",
            ),
            DeclareLaunchArgument("seed", default_value="42"),
            DeclareLaunchArgument(
                "conveyor_speed_mps",
                default_value="2.0",
            ),
            DeclareLaunchArgument(
                "screen_surface_speed_mps",
                default_value="2.0",
            ),
            gazebo,
            bridge,
            controller,
            TimerAction(period=2.5, actions=[cleanup]),
            TimerAction(period=4.0, actions=[spawner]),
        ]
    )
