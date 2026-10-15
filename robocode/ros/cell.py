"""roboCODE station matching the ROS 2 / Gazebo KR10 milling cell (ros2/robocode_kr10_description).

Robot base = URDF `base_link`. All values are nominal values taken from the cell URDF; the real
TCP, workpiece base and tool load must be measured (docs §12) before any use on a robot.
"""
from __future__ import annotations

import numpy as np

from ..core import pose as P
from ..core.scene import Frame, Object, Robot, Station, Tool
from ..robots import load_robot

MODEL = "kuka_kr10_r1100_sixx"
# URDF: link_6 -> flange -> tool0 (ros2/robocode_kr10_description/urdf/kr10_cell.urdf.xacro), mm
LINK6_TO_FLANGE = P.transl(10, 40, -53) @ P.rotz(-90)
FLANGE_TO_TOOL0 = P.transl(40, 54, 37) @ P.rotz(90)
SPINDLE_TIP_IN_TOOL0_MM = -76.0  # lowest point of spindel_v1.stl along tool0 Z (estimate, measure!)
# workpiece zero relative to the robot base as used by ClaBirk's ros_milling_cpp (Wobj_*_base_mm)
WORKPIECE_IN_BASE_MM = (500.0, 250.0, 300.0)
START_JOINTS_DEG = np.array([0.0, -90.0, 90.0, 0.0, 30.0, 0.0])


def spindle_tcp() -> np.ndarray:
    """Spindle tip in the roboCODE flange frame (= URDF link_6 rotated +90° about Y).

    Tool Z points down along the spindle (into the material), tool X = link_6 X.
    """
    link6_tip = LINK6_TO_FLANGE @ FLANGE_TO_TOOL0 @ P.transl(0, 0, SPINDLE_TIP_IN_TOOL0_MM)
    link6_tip = link6_tip.copy()
    link6_tip[:3, :3] = P.rotx(180)[:3, :3]  # spindle axis pointing down when link_6 is level
    rc_flange_in_link6 = P.from_xyzabc(0, 0, 0, 0, 90, 0)
    return P.inv(rc_flange_in_link6) @ link6_tip


def build_kr10_station(post: str = "kuka") -> tuple[Station, Robot, Frame, Tool]:
    st = Station(name="KR10 ROS 2 cell")
    robot = st.add(Robot(name="KR10", model=load_robot(MODEL), joints=START_JOINTS_DEG.copy(),
                         post_processor=post))
    spindle = robot.add(Tool(name="Wrzeciono (nominalne)", pose=spindle_tcp()))
    robot.active_tool = spindle
    frame = st.add(Frame(name="Detal", pose=P.transl(*WORKPIECE_IN_BASE_MM)))
    robot.active_frame = frame
    frame.add(Object(name="Surowiec", pose=P.transl(-10, -10, -30), box_size=(80, 80, 30)))
    return st, robot, frame, spindle
