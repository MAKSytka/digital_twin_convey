from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    gazebo_share = Path(get_package_share_directory("singulator_gazebo"))
    bringup_share = Path(get_package_share_directory("singulator_bringup"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = gazebo_share / "worlds" / "single_cell.sdf"
    bridge_config = bringup_share / "config" / "single_cell_bridge.yaml"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v 3 {world}"}.items(),
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="singulator_bridge", output="screen",
        parameters=[{"config_file": str(bridge_config)}],
    )
    return LaunchDescription([gazebo, bridge])
