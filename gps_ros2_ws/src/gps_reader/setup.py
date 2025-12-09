from setuptools import setup
from glob import glob
import os

package_name = "gps_reader"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pynmea2"],
    zip_safe=True,
    maintainer="orin",
    maintainer_email="you@example.com",
    description="ROS 2 GPS + compass nodes for Jetson",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # ros2 run gps_reader gps_node
            "gps_node = gps_reader.gps_node:main",
        ],
    },
)
