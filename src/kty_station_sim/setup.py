from glob import glob
import os

from setuptools import setup

package_name = "kty_station_sim"

setup(
    name=package_name,
    version="0.5.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob("worlds/*.sdf"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Singulator Team",
    maintainer_email="team@example.com",
    description="KTY contact-surface transport, physical vibration and RGB-D planning",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "smoke_heartbeat = kty_station_sim.smoke_heartbeat:main",
            "flow_cycle = kty_station_sim.flow_cycle_smooth:main",
            "mechatronics_cycle = kty_station_sim.mechatronics_cycle:main",
            "mechatronics_cycle_v2 = kty_station_sim.mechatronics_cycle_v2:main",
            # Preserve the accepted executable name while routing it to the
            # deadlock-free runtime-v14 startup implementation.
            "mechatronics_cycle_v3 = kty_station_sim.mechatronics_cycle_v14:main",
            "mechatronics_cycle_v10 = kty_station_sim.mechatronics_cycle_v10:main",
            "mechatronics_cycle_v11 = kty_station_sim.mechatronics_cycle_v11:main",
            "mechatronics_cycle_v12 = kty_station_sim.mechatronics_cycle_v12:main",
            "mechatronics_cycle_v13 = kty_station_sim.mechatronics_cycle_v13:main",
            "mechatronics_cycle_v14 = kty_station_sim.mechatronics_cycle_v14:main",
            "fill_estimator = kty_station_sim.fill_estimator:main",
            "fill_estimator_v2 = kty_station_sim.fill_estimator_v2:main",
            "depth_perception = kty_station_sim.depth_perception:main",
            "depth_perception_3d = kty_station_sim.depth_perception_3d:main",
            "depth_perception_3d_v2 = kty_station_sim.depth_perception_3d_v2:main",
            "contour_recorder = kty_station_sim.contour_recorder:main",
            "contour_recorder_3d = kty_station_sim.contour_recorder_3d:main",
            "vision_dashboard = kty_station_sim.vision_dashboard:main",
            "vision_dashboard_3d = kty_station_sim.vision_dashboard_3d:main",
            "station_controller = kty_station_sim.station_controller:main",
            "product_spawner = kty_station_sim.product_spawner:main",
            "safety_monitor = kty_station_sim.safety_monitor:main",
            "metrics_node = kty_station_sim.metrics_node:main",
        ]
    },
)
