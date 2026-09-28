from setuptools import find_packages, setup

package_name = "hexapod_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hexapod takimi",
    maintainer_email="sametoruc74@gmail.com",
    description="VL53L0X mesafe sensoru ve BNO055 IMU surucusu (dry-run destekli)",
    license="Apache-2.0",
    entry_points={"console_scripts": ["sensors = hexapod_sensors.node:main"]},
)
