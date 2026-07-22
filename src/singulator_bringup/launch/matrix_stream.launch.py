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
    description_share = Path(
        get_package_share_directory("singulator_description")
    )

    world = gazebo_share / "worlds" / "matrix_14x4_stream.sdf"

    start_spawner = LaunchConfiguration("start_spawner")
    start_demo_controller = LaunchConfiguration("start_demo_controller")
    start_cleanup = LaunchConfiguration("start_cleanup")
    start_perception = LaunchConfiguration("start_perception")
    start_singulation_controller = LaunchConfiguration(
        "start_singulation_controller"
    )
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

    camera_model = description_share / "models" / "vision_station" / "model.sdf"

    camera_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        name="vision_station_spawner",
        output="screen",
        condition=IfCondition(start_perception),
        arguments=[
            "-world",
            "matrix_14x4_stream",
            "-file",
            str(camera_model),
            "-name",
            "vision_station",
        ],
    )

    camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="singulator_camera_bridge",
        output="screen",
        condition=IfCondition(start_perception),
        arguments=["/singulator/camera/image_raw"],
    )

    perception = Node(
        package="singulator_perception",
        executable="vision_stream_node",
        name="vision_stream_node",
        output="screen",
        condition=IfCondition(start_perception),
        parameters=[
            {
                "image_topic": "/singulator/camera/image_raw",
                "boxes_topic": "/singulator/boxes",
                "field_length_m": 7.70,
                "field_width_m": 0.90,
                "field_min_x_m": -3.95,
                "field_max_y_m": 0.45,
                "calibration_frames": 15,
                "background_threshold": 18,
                "use_background_subtraction": True,
                "inner_erode_px": 2,
                "minimum_contour_area_px": 18.0,
                "morphology_open_px": 3,
                "morphology_close_px": 3,
                "dilate_iterations": 0,
                "enable_touching_split": True,
                "split_min_contour_area_px": 180.0,
                "split_peak_ratio": 0.42,
                "split_min_area_fraction": 0.12,
                "track_max_distance_m": 0.55,
                "track_max_lateral_distance_m": 0.28,
                "track_max_backward_distance_m": 0.16,
                "track_max_misses": 12,
                "use_sim_time": True,
            }
        ],
    )

    delayed_vision_station = TimerAction(
        period=1.5,
        actions=[camera_spawner],
    )

    fanout = Node(
        package="singulator_sim",
        executable="matrix_command_fanout",
        name="matrix_command_fanout",
        output="screen",
        parameters=[{"rows": 14, "cols": 4, "use_sim_time": True}],
    )

    singulation_controller = Node(
        package="singulator_control",
        executable="singulation_controller",
        name="singulation_controller",
        output="screen",
        condition=IfCondition(start_singulation_controller),
        parameters=[
            {
                "rows": 14,
                "cols": 4,
                "cell_length_m": 0.360,
                "cell_width_m": 0.175,
                "gap_x_m": 0.020,
                "gap_y_m": 0.020,
                "base_speed_mps": 1.20,
                "minimum_speed_mps": 0.15,
                "maximum_speed_mps": 2.00,
                "leader_speed_mps": 1.90,
                "target_gap_m": 0.16,
                "hard_gap_m": 0.04,
                "gap_gain": 1.80,
                "yaw_gain": 1.10,
                "maximum_yaw_delta_mps": 0.55,
                "publish_rate_hz": 20.0,
                "use_sim_time": True,
            }
        ],
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
            "5.20",
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
        # Leave an empty-camera interval for field/background calibration.
        period=6.0,
        actions=[spawner, cleanup],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_spawner", default_value="false"),
            DeclareLaunchArgument(
                "start_demo_controller", default_value="false"
            ),
            DeclareLaunchArgument("start_cleanup", default_value="true"),
            DeclareLaunchArgument("start_perception", default_value="true"),
            DeclareLaunchArgument(
                "start_singulation_controller",
                default_value="false",
            ),
            DeclareLaunchArgument("infeed_speed_mps", default_value="2.0"),
            DeclareLaunchArgument("outfeed_speed_mps", default_value="2.0"),
            DeclareLaunchArgument("demo_speed_mps", default_value="2.0"),
            DeclareLaunchArgument(
                "target_rate_boxes_per_sec", default_value="4.0"
            ),
            DeclareLaunchArgument("seed", default_value="42"),
            gazebo,
            *bridges,
            camera_bridge,
            perception,
            delayed_vision_station,
            fanout,
            singulation_controller,
            auxiliary_conveyors,
            demo_controller,
            delayed_box_nodes,
        ]
    )
