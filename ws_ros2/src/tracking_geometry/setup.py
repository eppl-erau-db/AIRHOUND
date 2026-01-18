from setuptools import find_packages, setup
import os
from glob import glob

package_name = "tracking_geometry"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="castej",
    maintainer_email="castej@todo.todo",
    description="Tracking and geometry node - converts detections to yaw commands",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "tracking_node = tracking_geometry.tracking:main",
        ],
    },
)
