"""Play a roboCODE trajectory (<name>.ros_trajectory.json) on a JointTrajectoryController.

  ros2 run robocode_ros trajectory_player --ros-args -p trajectory_file:=/path/Machining.ros_trajectory.json

The trajectory was validated by roboCODE before it was written (roboCODE never writes it otherwise).
Before the program starts, the robot is moved slowly from its current position to the first point.
Also publishes the TCP path as markers on /robocode/path_markers (RViz).
Intended for simulation (Gazebo / mock hardware); it is not approved for controlling a real robot.
"""
from __future__ import annotations

import time

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from .trajectory_file import TrajectoryFileError, approach_duration, load

DEFAULT_ACTION = "/kr10_arm_controller/follow_joint_trajectory"


def _duration(seconds: float) -> DurationMsg:
    sec = int(seconds)
    return DurationMsg(sec=sec, nanosec=int(round((seconds - sec) * 1e9)) % 1_000_000_000)


class TrajectoryPlayer(Node):
    def __init__(self):
        super().__init__("robocode_trajectory_player")
        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("controller_action", "")
        self.declare_parameter("time_scale", 1.0)
        self.declare_parameter("approach_velocity", 0.2)   # rad/s, move to the first point
        self.declare_parameter("joint_state_timeout", 30.0)  # s
        self.declare_parameter("publish_path", True)

        path = self.get_parameter("trajectory_file").value
        if not path:
            raise TrajectoryFileError("Podaj parametr trajectory_file")
        self.traj = load(path)
        self.time_scale = float(self.get_parameter("time_scale").value)
        self.times = self.traj.scaled(self.time_scale)
        action = self.get_parameter("controller_action").value or self.traj.controller_action or DEFAULT_ACTION
        self.get_logger().info(
            f"Program '{self.traj.program}': {len(self.times)} punktów, {self.times[-1]:.1f} s "
            f"(estymata roboCODE {self.traj.cycle_time_s_estimate:.1f} s × {self.time_scale}), akcja {action}")

        self.current: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.client = ActionClient(self, FollowJointTrajectory, action)
        self.goal_handle = None
        self.done = False
        self.success = False

        if self.get_parameter("publish_path").value:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.marker_pub = self.create_publisher(MarkerArray, "/robocode/path_markers", qos)
            self._publish_path()

    # ------------------------------------------------------------------ path markers
    def _publish_path(self):
        if not self.traj.tcp_path:
            return
        m = Marker()
        m.header.frame_id = self.traj.frame_id
        m.ns = "robocode_tcp_path"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.002
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 1.0, 1.0
        m.pose.orientation.w = 1.0
        m.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self.traj.tcp_path]
        self.marker_pub.publish(MarkerArray(markers=[m]))

    # ------------------------------------------------------------------ playback
    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.current[name] = pos

    def has_joint_state(self) -> bool:
        return all(n in self.current for n in self.traj.joint_names)

    def send(self) -> bool:
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Brak serwera akcji kontrolera – czy kontroler jest uruchomiony?")
            return False
        current = [self.current[n] for n in self.traj.joint_names]
        first = self.traj.positions[0]
        offset = approach_duration(current, first, float(self.get_parameter("approach_velocity").value))
        self.get_logger().info(f"Dojazd do punktu startowego: {offset:.1f} s")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.traj.joint_names)
        for t, q in zip(self.times, self.traj.positions):
            goal.trajectory.points.append(JointTrajectoryPoint(positions=list(q), time_from_start=_duration(offset + t)))
        goal.goal_time_tolerance = _duration(1.0)
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Kontroler odrzucił trajektorię")
            self.done = True
            return
        self.get_logger().info("Trajektoria przyjęta – odtwarzanie")
        self.goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        self.success = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        if self.success:
            self.get_logger().info("Program zakończony")
        else:
            self.get_logger().error(f"Kontroler przerwał trajektorię: kod {result.error_code} {result.error_string}")
        self.done = True

    def cancel(self):
        if self.goal_handle is not None and not self.done:
            self.goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TrajectoryPlayer()
    except TrajectoryFileError as exc:
        rclpy.logging.get_logger("robocode_trajectory_player").error(str(exc))
        rclpy.shutdown()
        return 2
    code = 1
    try:
        timeout = float(node.get_parameter("joint_state_timeout").value)
        start = time.monotonic()  # wall time: the sim clock may not run yet
        while rclpy.ok() and not node.has_joint_state():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - start > timeout:
                node.get_logger().error(f"Brak /joint_states dla osi {node.traj.joint_names}")
                return 2
        if node.send():
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.1)
            code = 0 if node.success else 1
        return code
    except KeyboardInterrupt:
        node.cancel()
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
