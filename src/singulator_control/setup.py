from setuptools import setup

package_name = "singulator_control"

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
    description="Baseline and auxiliary controllers",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "aux_conveyor_controller = singulator_control.aux_conveyor_controller:main",
            "singulation_controller = singulator_control.singulation_controller:main",
            "matrix_test_controller = singulator_control.matrix_test_controller:main",
            "single_cell_commander = singulator_control.single_cell_commander:main",
            "uniform_matrix_controller = singulator_control.uniform_matrix_controller:main",
            "row_1x4_controller = singulator_control.row_1x4_controller:main",
            "roller_throat_controller = singulator_control.roller_throat_controller:main",
            "separator_demo_controller = singulator_control.separator_demo_controller:main",
        ]
    },
)
