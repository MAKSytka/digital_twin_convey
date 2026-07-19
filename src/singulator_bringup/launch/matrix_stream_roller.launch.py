"""Upgraded stream launch with ±3 m/s matrix and angled roller throat.

The file reuses all nodes and arguments from matrix_stream.launch.py, but swaps
its Gazebo world for matrix_14x4_stream_v2.sdf and adds the roller throat.
This keeps the perception and singulation patches as the single source of truth.
"""

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

    throat_controller = Node(
        package="singulator_control",
        executable="roller_throat_controller",
        name="roller_throat_controller",
        output="screen",
        parameters=[
            {
                "speed_mps": 2.00,
                "publish_rate_hz": 20.0,
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
                "rows": 14,
                "cols": 4,
                "cell_length_m": 0.360,
                "cell_width_m": 0.175,
                "gap_x_m": 0.020,
                "gap_y_m": 0.020,
                "base_speed_mps": 2.00,
                "minimum_speed_mps": 0.35,
                "maximum_speed_mps": 3.00,
                "leader_speed_mps": 2.80,
                "target_gap_m": 0.18,
                "hard_gap_m": 0.035,
                "gap_gain": 2.20,
                "relative_velocity_gain": 0.45,
                "leader_boost_gain": 1.20,
                "nominal_transport_speed_mps": 2.00,
                "maximum_longitudinal_lag_m": 0.30,
                "lag_guard_horizon_s": 0.20,
                "lag_recovery_gain": 2.00,
                "prediction_horizon_s": 0.18,
                "longitudinal_control_margin_m": 0.16,
                "yaw_gain": 1.35,
                "yaw_rate_gain": 0.10,
                "maximum_yaw_delta_mps": 0.90,
                "maximum_acceleration_mps2": 3.0,
                "publish_rate_hz": 20.0,
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
            "3.070",
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

    # Spawn the throat before the delayed box generator starts producing waves.
    entities.extend(
        [
            throat_bridge,
            throat_controller,
            advanced_controller,
            TimerAction(period=1.5, actions=[throat_spawner]),
        ]
    )
    return LaunchDescription(entities)
