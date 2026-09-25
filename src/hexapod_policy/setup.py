from setuptools import find_packages, setup

package_name = "hexapod_policy"

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
    maintainer_email="gorkemmutlu227@gmail.com",
    description="Egitilmis RL politikasini calistiran ROS 2 dugumu",
    license="Apache-2.0",
    entry_points={"console_scripts": ["policy = hexapod_policy.node:main"]},
)
