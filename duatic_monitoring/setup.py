from setuptools import find_packages, setup

package_name = "duatic_monitoring"

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
            "gamepad_battery_node = duatic_monitoring.gamepad_battery_node:main",
            "litime_battery_node = duatic_monitoring.litime_battery_node:main",
            "mock_battery_node = duatic_monitoring.mock_battery_node:main",
            "battery_aggregator_node = duatic_monitoring.battery_aggregator_node:main",
        ],
    },
)
