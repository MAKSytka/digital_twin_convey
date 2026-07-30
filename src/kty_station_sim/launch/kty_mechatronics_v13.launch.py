import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from kty_station_sim.world_patch_v4 import build_runtime_v13_world


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("kty_station_sim"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    plugin_lib = Path(get_package_prefix("kty_conveyor_surface")) / "lib"
    source_world = package_share / "worlds" / "kty_mechatronics.sdf"
    generated_world = Path(tempfile.gettempdir()) / "kty_mechatronics_v13.sdf"
    build_runtime_v13_world(source_world, generated_world)

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
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
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

    # One persistent bridge replaces hundreds of short-lived `gz topic -e`
    # subprocesses. PosePublisher limits the source to model poses at 20 Hz.
    pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="kty_model_pose_bridge",
        output="screen",
        arguments=[
            "/world/kty_mechatronics_surface/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        remappings=[
            ("/world/kty_mechatronics_surface/pose", "/kty/mech/model_poses"),
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
        parameters=[{
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
            "processing_hz": 2.5,
        }],
    )

    perception = Node(
        package="kty_station_sim",
        executable="depth_perception_3d_v2",
        name="kty_classical_3d_perception_v2",
        output="screen",
        parameters=[{
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
            "minimum_contour_area_px": 24.0,
            "depth_edge_threshold_m": 0.010,
            "normal_edge_threshold": 0.12,
            "seed_min_distance_px": 10,
            "seed_height_prominence_m": 0.009,
            "track_max_distance_m": 0.14,
            "track_max_height_delta_m": 0.20,
            "track_max_misses": 12,
            "top_normal_min_z": 0.80,
            "top_occlusion_max": 0.48,
            "simulated_depth_noise_std_m": 0.0008,
            "simulated_dropout_probability": 0.001,
            "processing_hz": 2.5,
        }],
    )

    recorder = Node(
        package="kty_station_sim",
        executable="contour_recorder_3d",
        name="kty_contour_recorder_3d",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "input_topic": "/kty/perception/contours",
            "json_topic": "/kty/vision/polygons_json",
            "output_directory": output_directory,
            "save_empty_frames": False,
        }],
    )

    dashboard = Node(
        package="kty_station_sim",
        executable="vision_dashboard_3d",
        name="kty_vision_dashboard_3d",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "rgb_topic": "/kty/vision/image",
            "depth_topic": "/kty/vision/depth_image",
            "debug_topic": "/kty/perception/debug_image",
            "contours_topic": "/kty/perception/contours",
            "flow_state_topic": "/kty/flow/state",
            "dashboard_topic": "/kty/vision/dashboard",
            "show_window": ParameterValue(show_dashboard, value_type=bool),
            "refresh_hz": 3.0,
        }],
    )

    controller = Node(
        package="kty_station_sim",
        executable="mechatronics_cycle_v3",
        name="kty_mechatronics_cycle_v3",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "world_name": "kty_mechatronics_surface",
            "auto_repeat": ParameterValue(auto_repeat, value_type=bool),
            "product_spawn_interval_s": ParameterValue(spawn_interval, value_type=float),
            "fill_ratio_threshold": ParameterValue(fill_threshold, value_type=float),
            "max_height_threshold_m": ParameterValue(height_threshold, value_type=float),
            "roller_linear_speed_mps": 0.80,
            "slow_roller_linear_speed_mps": 0.22,
            "weak_vibration_frequency_hz": 5.0,
            "weak_vibration_amplitude_m": 0.0018,
            "strong_vibration_frequency_hz": 7.75,
            "strong_vibration_sweep_hz": 1.25,
            "strong_vibration_modulation_hz": 0.22,
            "strong_vibration_amplitude_m": 0.0080,
            "strong_vibration_duration_s": 15.0,
            "strong_vibration_ramp_s": 2.0,
            "vibration_settle_s": 1.2,
            "closed_gate_spawn_interval_s": 3.0,
            "closed_gate_max_products": 5,
            "prefeed_target_x_m": -0.50,
            "prefeed_timeout_s": 24.0,
            "position_next_timeout_s": 60.0,
        }],
    )

    delayed_vision = TimerAction(
        period=1.0,
        actions=[fill_estimator, perception, recorder, dashboard],
    )
    delayed_controller = TimerAction(period=2.5, actions=[controller])

    return LaunchDescription([
        DeclareLaunchArgument("auto_repeat", default_value="true"),
        DeclareLaunchArgument("product_spawn_interval_s", default_value="1.90"),
        DeclareLaunchArgument("fill_ratio_threshold", default_value="0.70"),
        DeclareLaunchArgument("max_height_threshold_m", default_value="0.280"),
        DeclareLaunchArgument("show_dashboard", default_value="false"),
        DeclareLaunchArgument(
            "polygon_output_directory",
            default_value="~/.ros/kty_vision",
        ),
        plugin_path,
        gazebo,
        command_bridge,
        pose_bridge,
        rgb_bridge,
        depth_bridge,
        delayed_vision,
        delayed_controller,
    ])
