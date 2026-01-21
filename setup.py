from setuptools import find_packages, setup

package_name = "duatic_helpers"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sven Kähler",
    maintainer_email="skaehler@duatic.com",
    description="Reusable helpers.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
