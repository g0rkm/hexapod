from pathlib import Path

from setuptools import find_packages, setup

package_name = "hexapod_description"

# STL'ler git'te değil (boyut); tools/cad_sim_model.py --copy-meshes ile
# meshes/ altına kopyalanır. Varsa kurulur, yoksa model mesh'siz çalışır.
meshes = sorted(str(p) for p in Path("meshes").glob("*.stl"))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    package_data={package_name: ["meshes.yaml"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/meshes", meshes),
        ("share/" + package_name + "/launch", ["launch/display.launch.py"]),
        ("share/" + package_name + "/rviz", ["rviz/display.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hexapod takimi",
    maintainer_email="gorkemmutlu227@gmail.com",
    description="Hexapod simulasyon modeli (URDF girdisi)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["make_urdf = hexapod_description.__main__:main"]},
)
