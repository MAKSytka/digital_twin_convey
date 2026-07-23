from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    gazebo_share = Path(get_package_share_directory("singulator_gazebo"))
    bringup_share = Path(get_package_share_directory("singulator_bringup"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))

    world = gazebo_share / "worlds" / "infeed_size_separator_demo.sdf"

    spawn_period = LaunchConfiguration("spawn_period_s")
    cycles = LaunchConfiguration("cycles")

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
                    bringup_share / "config" / "bridge_separator_demo.yaml"
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
                "infeed_speed_mps": 0.65,
                "screen_speed_mps": 0.55,
                "accepted_speed_mps": 0.70,
                "reject_speed_mps": 0.50,
                "publish_rate_hz": 20.0,
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
                "spawn_x": -1.60,
                "belt_top_z": 0.08,
                "spawn_period_s": ParameterValue(
                    spawn_period,
                    value_type=float,
                ),
                "cycles": ParameterValue(cycles, value_type=int),
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
                "exit_x_m": 1.82,
                "fallen_z_m": -0.80,
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_period_s", default_value="1.60"),
            DeclareLaunchArgument("cycles", default_value="3"),
            gazebo,
            bridge,
            controller,
            TimerAction(period=3.0, actions=[cleanup]),
            TimerAction(period=4.0, actions=[spawner]),
        ]
    )
