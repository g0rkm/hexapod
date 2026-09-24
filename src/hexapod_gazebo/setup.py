from pathlib import Path

from setuptools import find_packages, setup

package_name = "hexapod_gazebo"

worlds = sorted(str(p) for p in Path("worlds").glob("*.sdf"))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sim.launch.py"]),
        ("share/" + package_name + "/worlds", worlds),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hexapod takimi",
    maintainer_email="gorkemmutlu227@gmail.com",
    description="Hexapod Gazebo simulasyonu",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["stand = hexapod_gazebo.stand:main"]},
)
