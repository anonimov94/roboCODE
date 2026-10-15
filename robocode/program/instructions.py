"""Instrukcje programu robota (niezależne od marki kontrolera)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..core.scene import Item


@dataclass
class Instruction:
    kind = "instr"


@dataclass
class SetFrame(Instruction):
    pose: np.ndarray
    name: str = "Frame"
    kind = "set_frame"


@dataclass
class SetTool(Instruction):
    pose: np.ndarray
    name: str = "Tool"
    kind = "set_tool"


@dataclass
class MoveJ(Instruction):
    joints: np.ndarray
    pose: Optional[np.ndarray] = None  # TCP względem układu (jeśli kartezjański)
    kind = "movej"


@dataclass
class MoveL(Instruction):
    pose: np.ndarray
    joints: Optional[np.ndarray] = None
    kind = "movel"


@dataclass
class MoveC(Instruction):
    pose_mid: np.ndarray
    pose_end: np.ndarray
    joints_mid: Optional[np.ndarray] = None
    joints_end: Optional[np.ndarray] = None
    kind = "movec"


@dataclass
class SetSpeed(Instruction):
    linear_mm_s: Optional[float] = None
    joint_deg_s: Optional[float] = None
    linear_acc: Optional[float] = None
    joint_acc: Optional[float] = None
    kind = "speed"


@dataclass
class SetRounding(Instruction):
    mm: float = -1  # -1 = ruch dokładny (fine)
    kind = "rounding"


@dataclass
class Pause(Instruction):
    ms: float = -1  # -1 = stop do wznowienia
    kind = "pause"


@dataclass
class Message(Instruction):
    text: str
    kind = "message"


@dataclass
class Call(Instruction):
    name: str
    mode: str = "call"  # call | insert_code | start_thread
    kind = "call"


@dataclass
class SetIO(Instruction):
    io: str
    value: float | str
    analog: bool = False
    kind = "set_io"


@dataclass
class WaitIO(Instruction):
    io: str
    value: float | str
    timeout_ms: Optional[float] = None
    kind = "wait_io"


@dataclass
class SimEvent(Instruction):
    """Tylko symulacja: attach/detach/show/hide/set_abs/set_rel/home/conveyor/create/delete/wait."""
    action: str
    items: list = field(default_factory=list)
    params: dict = field(default_factory=dict)
    kind = "sim_event"


@dataclass(eq=False)
class Program(Item):
    robot: object = None
    instructions: list[Instruction] = field(default_factory=list)
    kind = "program"

    def append(self, ins: Instruction) -> Instruction:
        # pierwszy ruch -> automatycznie SetFrame + SetTool
        if isinstance(ins, (MoveJ, MoveL, MoveC)) and not any(
                isinstance(i, (MoveJ, MoveL, MoveC)) for i in self.instructions) and self.robot is not None:
            r = self.robot
            if not any(isinstance(i, SetFrame) for i in self.instructions):
                self.instructions.append(SetFrame(r.frame_rel_base(r.active_frame),
                                                  r.active_frame.name if r.active_frame else "Base"))
            if not any(isinstance(i, SetTool) for i in self.instructions):
                self.instructions.append(SetTool(r.active_tool.tcp() if r.active_tool else np.eye(4),
                                                 r.active_tool.name if r.active_tool else "Flange"))
        self.instructions.append(ins)
        return ins

    def toggle_move_type(self, index: int) -> None:
        """MoveJ <-> MoveL jednym kliknięciem."""
        ins = self.instructions[index]
        if isinstance(ins, MoveL):
            self.instructions[index] = MoveJ(joints=ins.joints, pose=ins.pose)
        elif isinstance(ins, MoveJ) and ins.pose is not None:
            self.instructions[index] = MoveL(pose=ins.pose, joints=ins.joints)
