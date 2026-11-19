import os
from glob import glob

from setuptools import find_packages, setup

package_name = "kinova_lfd_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="iras",
    maintainer_email="soloww@oregonstate.edu",
    description="Package to support collecting demonstration data",
    license="BSD-3-Clause",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "tabletop_rgbd = kinova_lfd_ros.services.tabletop_rgbd:main",
            "teleop = kinova_lfd_ros.teleop.teleop:main",
            "tool_publisher = kinova_lfd_ros.info.tool_publisher:main",
        ],
    },
)
