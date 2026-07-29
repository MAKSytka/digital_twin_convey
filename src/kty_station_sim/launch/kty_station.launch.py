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
    package_share = Path(get_package_share_directory("kty_station_sim"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = package_share / "worlds" / "kty_station.sdf"
    bridge_config = package_share / "config" / "bridge.yaml"
    station_config = package_share / "config" / "station.yaml"

    frequency = LaunchConfiguration("vibration_frequency_hz")
    amplitude = LaunchConfiguration("vibration_amplitude_m")
    product_rate = LaunchConfiguration("product_rate_products_per_s")
    seed = LaunchConfiguration("seed")
    vision_gui_enabled = LaunchConfiguration("vision_gui")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 3 {world}"}.items(),
    )

    # Gazebo can expose the world before its control service is ready.  Retry
    # with wall time so use_sim_time nodes are not left waiting on a paused
    # simulation clock.
    unpause_world = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            (
                "for attempt in $(seq 1 40); do "
                "output=$(gz service "
                "-s /world/kty_station/control "
                "--reqtype gz.msgs.WorldControl "
                "--reptype gz.msgs.Boolean "
                "--timeout 1000 "
                "--req 'pause: false' 2>&1); "
                "printf '%s\n' \"$output\"; "
                "if printf '%s' \"$output\" | grep -qi 'data: true'; then "
                "echo '[kty-startup] Gazebo world is running'; exit 0; fi; "
                "sleep 0.25; "
                "done; "
                "echo '[kty-startup] failed to unpause Gazebo world' >&2; "
                "exit 1"
            ),
        ],
        output="screen",
    )

    delayed_unpause = TimerAction(period=0.5, actions=[unpause_world])

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="kty_parameter_bridge",
        output="screen",
        parameters=[{"config_file": str(bridge_config)}],
    )

    rgb_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="kty_rgb_bridge",
        output="screen",
        arguments=["/kty/camera/image"],
    )
    depth_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="kty_depth_bridge",
        output="screen",
        arguments=["/kty/camera/depth_image"],
    )

    perception = Node(
        package="kty_station_sim",
        executable="depth_perception",
        name="kty_depth_perception",
        output="screen",
        parameters=[str(station_config)],
    )
    product_spawner = Node(
        package="kty_station_sim",
        executable="product_spawner",
        name="product_spawner",
        output="screen",
        parameters=[
            str(station_config),
            {
                "rate_products_per_s": ParameterValue(product_rate, value_type=float),
                "seed": ParameterValue(seed, value_type=int),
            },
        ],
    )
    controller = Node(
        package="kty_station_sim",
        executable="station_controller",
        name="station_controller",
        output="screen",
        parameters=[
            str(station_config),
            {
                "vibration_frequency_hz": ParameterValue(
                    frequency, value_type=float
                ),
                "vibration_amplitude_m": ParameterValue(
                    amplitude, value_type=float
                ),
            },
        ],
    )
    safety = Node(
        package="kty_station_sim",
        executable="safety_monitor",
        name="kty_safety_monitor",
        output="screen",
        parameters=[str(station_config)],
    )
    metrics = Node(
        package="kty_station_sim",
        executable="metrics_node",
        name="kty_metrics",
        output="screen",
        parameters=[str(station_config)],
    )
    registry_json_mirror = Node(
        package="kty_station_sim",
        executable="registry_json_mirror",
        name="kty_registry_json_mirror",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    vision_gui = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="kty_vision_view",
        output="screen",
        arguments=["/kty/perception/debug_image"],
        condition=IfCondition(vision_gui_enabled),
    )

    # The control nodes start after Gazebo and transport bridges have had time
    # to advertise the create/remove and sensor endpoints.
    delayed_nodes = TimerAction(
        period=2.0,
        actions=[
            registry_json_mirror,
            perception,
            product_spawner,
            safety,
            metrics,
            controller,
            vision_gui,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("vibration_frequency_hz", default_value="25.0"),
            DeclareLaunchArgument("vibration_amplitude_m", default_value="0.001"),
            DeclareLaunchArgument("product_rate_products_per_s", default_value="1.0"),
            DeclareLaunchArgument("seed", default_value="42"),
            DeclareLaunchArgument(
                "vision_gui",
                default_value="false",
                description="Open rqt_image_view with the machine-vision debug feed",
            ),
            gazebo,
            delayed_unpause,
            bridge,
            rgb_bridge,
            depth_bridge,
            delayed_nodes,
        ]
    )
