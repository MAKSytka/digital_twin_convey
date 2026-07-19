from setuptools import setup

package_name = "singulator_perception"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Singulator Team",
    maintainer_email="team@example.com",
    description="Continuous camera-frame processing for box observations",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vision_stream_node = singulator_perception.vision_stream_node:main",
        ]
    },
)
