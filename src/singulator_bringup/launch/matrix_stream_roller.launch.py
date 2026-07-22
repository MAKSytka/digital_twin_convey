"""Roller-throat launch using the V7 immutable global queue."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _load_base_launch():
    bringup_share = Path(get_package_share_directory("singulator_bringup"))
    base_path = bringup_share / "launch" / "matrix_stream.launch.py"
    spec = spec_from_file_location("singulator_matrix_stream_base", base_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load base launch: {base_path}")
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def generate_launch_description() -> LaunchDescription:
    base_launch = _load_base_launch()
    gazebo_share = Path(get_package_share_directory("singulator_gazebo"))
    description_share = Path(
        get_package_share_directory("singulator_description")
    )
    bringup_share = Path(get_package_share_directory("singulator_bringup"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))

    world = gazebo_share / "worlds" / "matrix_14x4_stream_v2.sdf"
    throat_model = description_share / "models" / "roller_throat" / "model.sdf"

    upgraded_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 3 {world}"}.items(),
    )

    throat_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="singulator_throat_bridge",
        output="screen",
        parameters=[
            {
                "config_file": str(
                    bringup_share / "config" / "bridge_throat.yaml"
                )
            }
        ],
    )

    extension_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="singulator_extension_bridge",
        output="screen",
        parameters=[
            {
                "config_file": str(
                    bringup_share / "config" / "bridge_rows_14_17.yaml"
                )
            }
        ],
    )

    extension_fanout = Node(
        package="singulator_sim",
        executable="matrix_command_fanout",
        name="matrix_command_fanout_18x4",
        output="screen",
        parameters=[{"rows": 18, "cols": 4, "use_sim_time": True}],
    )

    throat_controller = Node(
        package="singulator_control",
        executable="roller_throat_controller",
        name="roller_throat_controller",
        output="screen",
        parameters=[
            {
                "speed_mps": 2.50,
                "publish_rate_hz": 30.0,
                "use_sim_time": True,
            }
        ],
    )

    advanced_controller = Node(
        package="singulator_control",
        executable="singulation_controller",
        name="singulation_controller",
        output="screen",
        parameters=[
            {
                "rows": 18,
                "cols": 4,
                "cell_length_m": 0.360,
                "cell_width_m": 0.175,
                "gap_x_m": 0.020,
                "gap_y_m": 0.020,
                "matrix_center_x_m": 0.760,
                "minimum_speed_mps": 1.00,
                "maximum_speed_mps": 3.00,
                "idle_speed_mps": 2.20,
                "transport_speed_mps": 2.50,
                "maximum_acceleration_mps2": 6.0,
                "target_gap_m": 0.18,
                "inter_wave_target_gap_m": 0.28,
                "gap_gain": 3.00,
                "relative_velocity_gain": 0.50,
                "maximum_relative_speed_mps": 2.00,
                "order_inversion_margin_m": 0.03,
                "exit_gap_check_margin_m": 0.80,
                "deadline_separation_distance_m": 1.80,
                "deadline_gap_margin_m": 0.04,
                "deadline_recovery_gain": 1.35,
                "deadline_min_time_s": 0.10,
                "entry_gate_offset_m": 0.30,
                "entry_capture_window_s": 0.18,
                "entry_wave_dx_m": 0.36,
                "entry_wave_max_size": 6,
                "entry_capture_back_margin_m": 1.35,
                "entry_capture_front_margin_m": 0.18,
                "normal_track_timeout_s": 0.60,
                "merged_track_timeout_s": 2.80,
                "logical_item_expiration_s": 3.50,
                "prediction_max_horizon_s": 3.00,
                "reid_max_distance_m": 0.70,
                "reid_max_lateral_distance_m": 0.30,
                "reid_max_backward_distance_m": 0.22,
                "merge_area_ratio": 1.35,
                "merge_padding_x_m": 0.08,
                "merge_padding_y_m": 0.05,
                "exit_remove_margin_m": 0.65,
                "prediction_horizon_s": 0.12,
                "longitudinal_control_margin_m": 0.10,
                "yaw_gain": 0.65,
                "yaw_rate_gain": 0.05,
                "maximum_yaw_delta_mps": 0.25,
                "dense_pair_yaw_gap_m": 0.10,
                "yaw_control_exit_margin_m": 0.65,
                "allocation_urgency_gain": 1.50,
                "allocation_idle_regularization": 0.03,
                "allocation_iterations": 12,
                "uncontrollable_similarity": 0.97,
                "minimum_confidence": 0.12,
                "observation_timeout_s": 0.70,
                "publish_rate_hz": 30.0,
                "use_sim_time": True,
            }
        ],
    )

    throat_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        name="roller_throat_spawner",
        output="screen",
        arguments=[
            "-world",
            "matrix_14x4_stream",
            "-file",
            str(throat_model),
            "-name",
            "roller_throat",
            "-x",
            "4.590",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
    )

    entities = []
    gazebo_replaced = False
    for entity in base_launch.entities:
        if isinstance(entity, IncludeLaunchDescription) and not gazebo_replaced:
            entities.append(upgraded_gazebo)
            gazebo_replaced = True
        else:
            entities.append(entity)

    if not gazebo_replaced:
        raise RuntimeError("Base launch does not contain a Gazebo include")

    entities.extend(
        [
            throat_bridge,
            extension_bridge,
            extension_fanout,
            throat_controller,
            advanced_controller,
            TimerAction(period=1.5, actions=[throat_spawner]),
        ]
    )
    return LaunchDescription(entities)
