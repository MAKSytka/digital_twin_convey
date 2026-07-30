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
    flow_launch = package_share / "launch" / "kty_flow.launch.py"

    product_count = LaunchConfiguration("product_count")
    auto_repeat = LaunchConfiguration("auto_repeat")
    approach_duration = LaunchConfiguration("approach_duration_s")
    spawn_interval = LaunchConfiguration("product_spawn_interval_s")
    outfeed_duration = LaunchConfiguration("outfeed_duration_s")
    pose_update_hz = LaunchConfiguration("pose_update_hz")
    show_dashboard = LaunchConfiguration("show_dashboard")
    output_directory = LaunchConfiguration("polygon_output_directory")

    flow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(flow_launch)),
        launch_arguments={
            "product_count": product_count,
            "auto_repeat": auto_repeat,
            "approach_duration_s": approach_duration,
            "product_spawn_interval_s": spawn_interval,
            "outfeed_duration_s": outfeed_duration,
            "pose_update_hz": pose_update_hz,
        }.items(),
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

    delayed_vision = TimerAction(
        period=1.5,
        actions=[perception, recorder, dashboard],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("product_count", default_value="6"),
            DeclareLaunchArgument("auto_repeat", default_value="true"),
            DeclareLaunchArgument("approach_duration_s", default_value="3.0"),
            DeclareLaunchArgument(
                "product_spawn_interval_s",
                default_value="0.9",
            ),
            DeclareLaunchArgument("outfeed_duration_s", default_value="3.0"),
            DeclareLaunchArgument("pose_update_hz", default_value="20.0"),
            DeclareLaunchArgument("show_dashboard", default_value="true"),
            DeclareLaunchArgument(
                "polygon_output_directory",
                default_value="~/.ros/kty_vision",
            ),
            flow,
            rgb_bridge,
            depth_bridge,
            delayed_vision,
        ]
    )
