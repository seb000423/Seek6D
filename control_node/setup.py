from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'control_node'

setup(
    name=package_name,
    version="1.6.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "scipy", "pymodbus"],
    zip_safe=True,
    maintainer="eon",
    maintainer_email="seb000423@gmail.com",
    description="MoveIt 2 motion backend for the Any6D M0609 task controller",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "robot_control_any6d_moveit = control_node.main:main",
        ],
    },
)
