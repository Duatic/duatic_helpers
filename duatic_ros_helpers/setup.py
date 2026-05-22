from setuptools import find_packages, setup

package_name = "duatic_ros_helpers"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "e_stop_node = duatic_ros_helpers.e_stop_node:main",
            "joint_parking_node = duatic_ros_helpers.joint_parking_node:main",
        ],
    },
)
