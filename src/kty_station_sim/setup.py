from glob import glob
import os

from setuptools import setup

package_name = "kty_station_sim"

setup(
    name=package_name,
    version="0.1.0",
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
    description="KTY station simulation, control, perception and metrics",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "station_controller = kty_station_sim.station_controller:main",
            "product_spawner = kty_station_sim.product_spawner:main",
            "depth_perception = kty_station_sim.depth_perception:main",
            "safety_monitor = kty_station_sim.safety_monitor:main",
            "metrics_node = kty_station_sim.metrics_node:main",
        ]
    },
)
