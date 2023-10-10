import os
from glob import glob

from setuptools import setup

package_name = "pepper_nav"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "params"),
            glob(os.path.join("params", "*")),
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
        "console_scripts": [
            "navigator = pepper_nav.navigator:main",
            "laserconverter = pepper_nav.laserscan_to_pointcloud:main",
            "headlocker = pepper_nav.head_locker:main",
        ],
    },
)
