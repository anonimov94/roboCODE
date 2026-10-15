from setuptools import find_packages, setup

package_name = "robocode_ros"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="anonimov94",
    maintainer_email="noreply@example.com",
    description="ROS 2 nodes for roboCODE",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "trajectory_player = robocode_ros.trajectory_player:main",
            "joint_points_demo = robocode_ros.joint_points_demo:main",
        ],
    },
)
