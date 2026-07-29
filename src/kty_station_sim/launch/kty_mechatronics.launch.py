from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("kty_station_sim"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = package_share / "worlds" / "kty_mechatronics.sdf"

    auto_repeat = LaunchConfiguration("auto_repeat")
    spawn_interval = LaunchConfiguration("product_spawn_interval_s")
    fill_threshold = LaunchConfiguration("fill_ratio_threshold")
    height_threshold = LaunchConfiguration("max_height_threshold_m")
    show_dashboard = LaunchConfiguration("show_dashboard")
    output_directory = LaunchConfiguration("polygon_output_directory")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 3 {world}"}.items(),
    )

    command_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="kty_mechatronics_command_bridge",
        output="screen",
        arguments=[
            "/kty/mech/infeed_rollers/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
            "/kty/mech/active_rollers/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
            "/kty/mech/outfeed_rollers/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double",
            "/kty/mech/pusher/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/kty/mech/clamps/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/kty/mech/gate/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
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
        executable="fill_estimator",
        name="kty_fill_estimator",
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
                "processing_hz": 10.0,
            }
        ],
    )

    perception = Node(
        package="kty_station_sim",
        executable="depth_perception",
        name="kty_depth_perception",
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
                "minimum_contour_area_px": 70.0,
                "track_max_distance_m": 0.12,
                "track_max_misses": 10,
                "simulated_depth_noise_std_m": 0.001,
                "simulated_dropout_probability": 0.002,
            }
        ],
    )

    recorder = Node(
        package="kty_station_sim",
        executable="contour_recorder",
        name="kty_contour_recorder",
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
        executable="vision_dashboard",
        name="kty_vision_dashboard",
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
                "refresh_hz": 10.0,
            }
        ],
    )

    controller = Node(
        package="kty_station_sim",
        executable="mechatronics_cycle",
        name="kty_mechatronics_cycle",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "world_name": "kty_mechatronics",
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
                default_value="0.65",
            ),
            DeclareLaunchArgument(
                "fill_ratio_threshold",
                default_value="0.70",
            ),
            DeclareLaunchArgument(
                "max_height_threshold_m",
                default_value="0.280",
            ),
            DeclareLaunchArgument("show_dashboard", default_value="true"),
            DeclareLaunchArgument(
                "polygon_output_directory",
                default_value="~/.ros/kty_vision",
            ),
            gazebo,
            command_bridge,
            rgb_bridge,
            depth_bridge,
            delayed_vision,
            delayed_controller,
        ]
    )
