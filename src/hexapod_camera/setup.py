from setuptools import find_packages, setup

package_name = "hexapod_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/kamera.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hexapod takimi",
    maintainer_email="gorkemmutlu227@gmail.com",
    description="Pi kamerasindan tarayicida canli goruntu (yalniz izleme)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["stream = hexapod_camera.node:main"]},
)
