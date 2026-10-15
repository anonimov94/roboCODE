"""Gotowa stacja demonstracyjna + projekt obróbki (używane przez CLI, GUI i testy)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .core import pose as P
from .core.scene import Frame, Object, Robot, Station, Tool
from .kinematics.robot_model import example_6r
from .machining.nc_parser import parse_gcode
from .machining.solver import MachiningResult, MachiningSettings, solve
from .machining.toolpath import Toolpath

DEMO_NC = """(kwadrat 60x60 mm, glebokosc 2 mm)
G90 G17
T1 M6
S12000 M3
G0 X0 Y0 Z10
G1 Z-2 F600
G1 X60
G1 Y60
G1 X0
G1 Y0
G2 X20 Y0 I10 J0
G0 Z10
M5
"""


def build_demo_station(post: str = "kuka"):
    st = Station(name="Demo")
    r = st.add(Robot(name="Robot", model=example_6r(), joints=np.array([0, 40, 15, 0, 35, 0.0]),
                     post_processor=post))
    spindle = r.add(Tool(name="Wrzeciono", pose=P.transl(0, 0, 150)))
    r.active_tool = spindle
    fr = st.add(Frame(name="Detal", pose=P.transl(550, -30, 150)))
    r.active_frame = fr
    # surowiec 80x80x30 mm, górna powierzchnia na Z=0 układu detalu
    fr.add(Object(name="Surowiec", pose=P.transl(-10, -10, -30), box_size=(80, 80, 30)))
    return st, r, fr, spindle


@dataclass
class MachiningProject:
    """Projekt obróbki powiązany ze stacją (M6 rozbuduje go o panel GUI)."""
    robot: Robot
    frame: Frame
    tool: Tool
    toolpath: Toolpath
    settings: MachiningSettings
    result: MachiningResult | None = None

    def update(self) -> MachiningResult:
        self.result = solve(self.robot, self.robot.frame_rel_base(self.frame), self.tool.tcp(),
                            self.toolpath, self.settings, name="Machining")
        if self.result.ok:
            self.robot.joints = self.result.joints[0].copy()
        return self.result

    def overlay(self, trace=None):
        from .gui.render import Overlay
        tp = self.toolpath.with_approach_retract(self.settings.approach_mm) \
            if self.settings.approach_mm > 0 else self.toolpath
        F = self.frame.pose_abs()
        pts = np.array([(F @ np.append(pp.p, 1))[:3] for pp in tp.points])
        nrm = np.array([F[:3, :3] @ pp.n for pp in tp.points])
        reach = self.result.reachable if self.result else None
        return Overlay(path_points=pts, path_normals=nrm, reachable=reach, trace=trace)


def demo_project(nc_text: str = DEMO_NC, settings: MachiningSettings | None = None):
    st, r, fr, tool = build_demo_station()
    proj = MachiningProject(r, fr, tool, parse_gcode(nc_text), settings or MachiningSettings())
    proj.update()
    return st, proj
