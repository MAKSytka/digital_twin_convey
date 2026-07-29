import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from kty_station_sim.world_patch_v3 import build_surface_world


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("kty_station_sim"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    plugin_lib = Path(get_package_prefix("kty_conveyor_surface")) / "lib"
    source_world = package_share / "worlds" / "kty_mechatronics.sdf"
    generated_world = Path(tempfile.gettempdir()) / "kty_mechatronics_surface.sdf"
    build_surface_world(source_world, generated_world)

    auto_repeat = LaunchConfiguration("auto_repeat")
    spawn_interval = LaunchConfiguration("product_spawn_interval_s")
    fill_threshold = LaunchConfiguration("fill_ratio_threshold")
    height_threshold = LaunchConfiguration("max_height_threshold_m")
    show_dashboard = LaunchConfiguration("show_dashboard")
    output_directory = LaunchConfiguration("polygon_output_directory")

    plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            str(plugin_lib),
            os.pathsep,
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 3 {generated_world}"}.items(),
    )

    command_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="kty_mechatronics_command_bridge",
        output="screen",
        arguments=[
            "/kty/mech/infeed_surface/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
            "/kty/mech/active_surface/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
            "/kty/mech/outfeed_surface/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
            "/kty/mech/pusher/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/kty/mech/clamps/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/kty/mech/vibration/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/kty/mech/locator_stop/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
        ],
    )

    rgb_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="kty_vision_rgb_bridge",
        output="screen",
        arguments=["/kty/vision/image"],
    )
    depth_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="kty_vision_depth_bridge",
        output="screen",
        arguments=["/kty/vision/depth_image"],
    )

    fill_estimator = Node(
        package="kty_station_sim",
        executable="fill_estimator_v2",
        name="kty_fill_estimator_v2",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "depth_topic": "/kty/vision/depth_image",
                "camera_info_topic": "/kty/vision/camera_info",
                "output_topic": "/kty/fill/state",
                "camera_to_bottom_m": 1.25,
                "internal_length_m": 0.60,
                "internal_width_m": 0.40,
                "internal_height_m": 0.40,
                "wall_exclusion_margin_m": 0.040,
                "maximum_product_height_m": 0.360,
                "processing_hz": 4.0,
            }
        ],
    )

    perception = Node(
        package="kty_station_sim",
        executable="depth_perception_3d_v2",
        name="kty_classical_3d_perception_v2",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "rgb_topic": "/kty/vision/image",
                "depth_topic": "/kty/vision/depth_image",
                "camera_info_topic": "/kty/vision/camera_info",
                "output_topic": "/kty/perception/contours",
                "debug_topic": "/kty/perception/debug_image",
                "camera_to_kty_bottom_m": 1.25,
                "internal_length_m": 0.60,
                "internal_width_m": 0.40,
                "internal_height_m": 0.40,
                "minimum_product_height_m": 0.008,
                "minimum_contour_area_px": 42.0,
                "depth_edge_threshold_m": 0.010,
                "normal_edge_threshold": 0.12,
                "seed_min_distance_px": 14,
                "seed_height_prominence_m": 0.010,
                "track_max_distance_m": 0.14,
                "track_max_height_delta_m": 0.18,
                "track_max_misses": 12,
                "top_normal_min_z": 0.80,
                "top_occlusion_max": 0.48,
                "simulated_depth_noise_std_m": 0.0008,
                "simulated_dropout_probability": 0.001,
                "processing_hz": 4.0,
            }
        ],
    )

    recorder = Node(
        package="kty_station_sim",
        executable="contour_recorder_3d",
        name="kty_contour_recorder_3d",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "input_topic": "/kty/perception/contours",
                "json_topic": "/kty/vision/polygons_json",
                "output_directory": output_directory,
                "save_empty_frames": False,
            }
        ],
    )

    dashboard = Node(
        package="kty_station_sim",
        executable="vision_dashboard_3d",
        name="kty_vision_dashboard_3d",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "rgb_topic": "/kty/vision/image",
                "depth_topic": "/kty/vision/depth_image",
                "debug_topic": "/kty/perception/debug_image",
                "contours_topic": "/kty/perception/contours",
                "flow_state_topic": "/kty/flow/state",
                "dashboard_topic": "/kty/vision/dashboard",
                "show_window": ParameterValue(show_dashboard, value_type=bool),
                "refresh_hz": 5.0,
            }
        ],
    )

    controller = Node(
        package="kty_station_sim",
        executable="mechatronics_cycle_v3",
        name="kty_mechatronics_cycle_v3",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "world_name": "kty_mechatronics_surface",
                "auto_repeat": ParameterValue(auto_repeat, value_type=bool),
                "product_spawn_interval_s": ParameterValue(
                    spawn_interval,
                    value_type=float,
                ),
                "fill_ratio_threshold": ParameterValue(
                    fill_threshold,
                    value_type=float,
                ),
                "max_height_threshold_m": ParameterValue(
                    height_threshold,
                    value_type=float,
                ),
                "roller_linear_speed_mps": 0.34,
                "slow_roller_linear_speed_mps": 0.12,
                "weak_vibration_frequency_hz": 8.0,
                "weak_vibration_amplitude_m": 0.0005,
                "strong_vibration_frequency_hz": 18.0,
                "strong_vibration_amplitude_m": 0.0030,
                "strong_vibration_duration_s": 8.0,
                "strong_vibration_ramp_s": 1.0,
            }
        ],
    )

    delayed_vision = TimerAction(
        period=1.0,
        actions=[fill_estimator, perception, recorder, dashboard],
    )
    delayed_controller = TimerAction(period=2.0, actions=[controller])

    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_repeat", default_value="true"),
            DeclareLaunchArgument(
                "product_spawn_interval_s",
                default_value="1.15",
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
            plugin_path,
            gazebo,
            command_bridge,
            rgb_bridge,
            depth_bridge,
            delayed_vision,
            delayed_controller,
        ]
    )
