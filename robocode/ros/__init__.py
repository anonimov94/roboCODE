"""Bridge to ROS 2 / Gazebo (docs/SPECYFIKACJA.md D9).

roboCODE itself does not import ROS. It writes a validated joint trajectory
(`<name>.ros_trajectory.json`) that the `robocode_ros` package in `ros2/` plays
back on a ros2_control JointTrajectoryController (Gazebo, mock hardware or MoveIt).
"""
from .trajectory import TRAJECTORY_FORMAT, build_trajectory, ros_mapping_problem

__all__ = ["TRAJECTORY_FORMAT", "build_trajectory", "ros_mapping_problem"]
