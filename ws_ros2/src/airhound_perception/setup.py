from setuptools import setup

package_name = "airhound_perception"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/perception.launch.py",
                "launch/system.launch.py",
                "launch/synthetic_test.launch.py",
                "launch/sim_test.launch.py",
                "launch/record_bag.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/env_hook",
            [
                "env_hook/airhound_perception_env.sh",
            ],
        ),
        ("share/" + package_name + "/config", ["config/perception.yaml"]),
    ],
    install_requires=["setuptools", "numpy>=1.24,<2.0"],
    extras_require={
        "yolo": ["ultralytics>=8.0.0"],  # Only needed for real YOLO detection
    },
    zip_safe=True,
    maintainer="Rylan",
    maintainer_email="rylan@example.com",
    description="ROS 2 perception node for AIRHOUND.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector_node = airhound_perception.detector_node:main",
            "synthetic_camera = airhound_perception.synthetic_camera:main",
            "mock_detector = airhound_perception.mock_detector:main",
        ],
    },
)
