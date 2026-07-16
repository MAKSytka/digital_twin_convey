from setuptools import setup

package_name = "singulator_sim"

setup(
    name=package_name,
    version="0.2.0",
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
    description="Simulation adapters and scenario tools",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cleanup_passed_boxes = singulator_sim.cleanup_passed_boxes:main",
            "singulation_row_spawner = singulator_sim.singulation_row_spawner:main",
            "matrix_command_fanout = singulator_sim.matrix_command_fanout:main",
        ]
    },
)
