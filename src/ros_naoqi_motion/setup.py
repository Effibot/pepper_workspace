import os
from glob import glob

from setuptools import setup

package_name = "ros_naoqi_motion"

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
            os.path.join("share", package_name, "script"),
            glob(os.path.join("script", "*.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="andrea efficace",
    maintainer_email="andrea.efficace1@gmail.com",
    description="NAOqi motion server",
    license="Apache License 2",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["naoqi_motion = ros_naoqi_motion.naoqi_motion:main"],
    },
)
