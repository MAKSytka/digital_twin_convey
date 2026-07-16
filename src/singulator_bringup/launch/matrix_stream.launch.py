from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    gazebo_share = Path(get_package_share_directory("singulator_gazebo"))
    bringup_share = Path(get_package_share_directory("singulator_bringup"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))

    world = gazebo_share / "worlds" / "matrix_14x4_stream.sdf"

    start_spawner = LaunchConfiguration("start_spawner")
    start_demo_controller = LaunchConfiguration("start_demo_controller")
    start_cleanup = LaunchConfiguration("start_cleanup")
    infeed_speed = LaunchConfiguration("infeed_speed_mps")
    outfeed_speed = LaunchConfiguration("outfeed_speed_mps")
    demo_speed = LaunchConfiguration("demo_speed_mps")
    target_rate = LaunchConfiguration("target_rate_boxes_per_sec")
    seed = LaunchConfiguration("seed")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 3 {world}"}.items(),
    )

    bridge_files = (
        "bridge_rows_00_03.yaml",
        "bridge_rows_04_07.yaml",
        "bridge_rows_08_11.yaml",
        "bridge_rows_12_13.yaml",
        "bridge_aux.yaml",
    )

    bridges = [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"singulator_bridge_{index}",
            output="screen",
            parameters=[
                {
                    "config_file": str(
                        bringup_share / "config" / filename
                    )
                }
            ],
        )
        for index, filename in enumerate(bridge_files)
    ]

    fanout = Node(
        package="singulator_sim",
        executable="matrix_command_fanout",
        name="matrix_command_fanout",
        output="screen",
        parameters=[{"rows": 14, "cols": 4, "use_sim_time": True}],
    )

    auxiliary_conveyors = Node(
        package="singulator_control",
        executable="aux_conveyor_controller",
        name="aux_conveyor_controller",
        output="screen",
        parameters=[
            {
                "infeed_speed_mps": ParameterValue(infeed_speed, value_type=float),
                "outfeed_speed_mps": ParameterValue(outfeed_speed, value_type=float),
                "publish_rate_hz": 10.0,
                "use_sim_time": True,
            }
        ],
    )

    spawner = Node(
        package="singulator_sim",
        executable="singulation_row_spawner",
        name="singulation_row_spawner",
        output="screen",
        condition=IfCondition(start_spawner),
        parameters=[
            {
                "world_name": "matrix_14x4_stream",
                "spawn_x": -3.60,
                "x_jitter_m": 0.08,
                "belt_top_z": 0.08,
                "infeed_speed_mps": ParameterValue(infeed_speed, value_type=float),
                "target_rate_boxes_per_sec": ParameterValue(target_rate, value_type=float),
                "maximum_box_length_m": 0.40,
                "safety_gap_m": 0.10,
                "seed": ParameterValue(seed, value_type=int),
                "use_sim_time": True,
            }
        ],
    )

    demo_controller = Node(
        package="singulator_control",
        executable="uniform_matrix_controller",
        name="uniform_matrix_controller",
        output="screen",
        condition=IfCondition(start_demo_controller),
        parameters=[
            {
                "rows": 14,
                "cols": 4,
                "speed_mps": ParameterValue(demo_speed, value_type=float),
                "publish_rate_hz": 20.0,
                "use_sim_time": True,
            }
        ],
    )

    cleanup = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "singulator_sim",
            "cleanup_passed_boxes",
            "--delete-x",
            "3.60",
            "--fallen-z",
            "-0.50",
        ],
        output="screen",
        condition=IfCondition(start_cleanup),
    )

    # The matrix controller publishes continuously from startup.  The spawner is
    # delayed so the bridge is ready and all belts have already ramped to the
    # requested 2 m/s before the first box reaches the matrix.
    delayed_box_nodes = TimerAction(
        period=4.0,
        actions=[spawner, cleanup],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_spawner", default_value="false"),
            DeclareLaunchArgument(
                "start_demo_controller", default_value="false"
            ),
            DeclareLaunchArgument("start_cleanup", default_value="true"),
            DeclareLaunchArgument("infeed_speed_mps", default_value="2.0"),
            DeclareLaunchArgument("outfeed_speed_mps", default_value="2.0"),
            DeclareLaunchArgument("demo_speed_mps", default_value="2.0"),
            DeclareLaunchArgument(
                "target_rate_boxes_per_sec", default_value="4.0"
            ),
            DeclareLaunchArgument("seed", default_value="42"),
            gazebo,
            *bridges,
            fanout,
            auxiliary_conveyors,
            demo_controller,
            delayed_box_nodes,
        ]
    )
