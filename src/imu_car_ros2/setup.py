from setuptools import find_packages, setup


package_name = "imu_car_ros2"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}", ["README.md"]),
        (f"share/{package_name}/launch", ["launch/car_with_imu.launch.py"]),
        (f"share/{package_name}/config", ["config/car_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="baotianyi",
    maintainer_email="baotianyi@example.com",
    description="ROS2 package for car control and IMU fused odometry.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "car_controller = imu_car_ros2.car_controller:main",
            "car_odometry = imu_car_ros2.car_odometry:main",
            "imu_driver = imu_car_ros2.imu_driver:main",
        ],
    },
)
