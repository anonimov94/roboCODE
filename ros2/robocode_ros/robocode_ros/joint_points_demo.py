"""Five-point joint trajectory demo (port of ros_milling/joint_points_act_service.py, ClaBirk, GPL-3.0).

Changes vs. the original: joint names joint_1..joint_6 and the controller action of this cell
(the original sent joint1..joint6 to /joint_trajectory_controller, which does not exist in the cell).

  ros2 run robocode_ros joint_points_demo
"""
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
POINTS = [  # rad, original values
    [-0.56, -0.72, 1.89, -2.11, 1.32, 2.81],
    [-0.31, -0.99, 1.89, -1.97, 1.28, 2.48],
    [-0.56, -0.99, 1.89, -1.97, 1.28, 2.48],
    [-0.56, -0.72, 1.89, -2.11, 1.32, 2.81],
    [-0.56, -0.48, 1.66, -2.60, 1.32, 2.94],
]


class JointPointsDemo(Node):
    def __init__(self):
        super().__init__("robocode_joint_points_demo")
        self.declare_parameter("controller_action", "/kr10_arm_controller/follow_joint_trajectory")
        self.declare_parameter("seconds_per_point", 2.0)
        action = self.get_parameter("controller_action").value
        self.client = ActionClient(self, FollowJointTrajectory, action)

    def send_goal(self):
        step = float(self.get_parameter("seconds_per_point").value)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for i, q in enumerate(POINTS, start=1):
            goal.trajectory.points.append(
                JointTrajectoryPoint(positions=q, time_from_start=Duration(seconds=step * i).to_msg()))
        goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        self.client.wait_for_server()
        self.client.send_goal_async(goal).add_done_callback(self._on_goal)

    def _on_goal(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal rejected")
            rclpy.shutdown()
            return
        self.get_logger().info("Goal accepted")
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self.get_logger().info(f"Result: {future.result().result}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = JointPointsDemo()
    node.send_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
