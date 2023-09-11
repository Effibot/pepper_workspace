from setuptools import setup
from glob import glob
import os

package_name = "my_turtle_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*.urdf")),
        ),
        (
            os.path.join("share", package_name, "sdf"),
            glob(os.path.join("sdf", "*.sdf")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
        (
            os.path.join("share", package_name, "params"),
            glob(os.path.join("params", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "meshes"),
            glob(os.path.join("meshes", "*.dae")),
        ),
        (
            os.path.join("share", package_name, "model/turtlebot3_house"),
            glob(os.path.join("model/turtlebot3_house", "*")),
        ),
        (
            os.path.join("share", package_name, "world"),
            glob(os.path.join("world", "*.world")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="andrea.efficace1@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
