from glob import glob
from setuptools import find_packages, setup


package_name = "robot_decision"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="boreas",
    maintainer_email="boreas@example.com",
    description="Decision nodes for localization initialization and Nav2 waypoint cruising.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "init_pose_node = robot_decision.init_pose_node:main",
            "cruise_node = robot_decision.cruise_node:main",
            "point3_fine_tune_node = robot_decision.point3_fine_tune_node:main",
        ],
    },
)
