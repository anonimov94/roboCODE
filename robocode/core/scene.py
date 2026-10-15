"""Drzewo stacji: Station / Frame / Robot / Tool / Object / Target / Program / Curve."""
from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import pose as P

_ids = itertools.count(1)


@dataclass(eq=False)
class Item:
    name: str
    pose: np.ndarray = field(default_factory=P.eye)  # względem rodzica
    parent: Optional["Item"] = None
    children: list["Item"] = field(default_factory=list)
    visible: bool = True
    id: int = field(default_factory=lambda: next(_ids))

    kind = "item"

    # --- hierarchia ---
    def add(self, child: "Item") -> "Item":
        if child.parent is not None:
            child.parent.children.remove(child)
        child.parent = self
        self.children.append(child)
        return child

    def set_parent(self, new_parent: "Item", keep_absolute: bool = True) -> None:
        absT = self.pose_abs()
        new_parent.add(self)
        if keep_absolute:
            self.pose = P.inv(new_parent.pose_abs()) @ absT

    def pose_abs(self) -> np.ndarray:
        T = self.pose.copy()
        p = self.parent
        while p is not None:
            T = p.child_base_pose() @ T
            p = p.parent
        return T

    def child_base_pose(self) -> np.ndarray:
        """Poza, do której doczepiane są dzieci (dla robota narzędzia idą na kołnierz)."""
        return self.pose

    def walk(self):
        yield self
        for c in self.children:
            yield from c.walk()

    def find(self, name: str) -> Optional["Item"]:
        return next((i for i in self.walk() if i.name == name), None)

    def __repr__(self) -> str:
        return f"<{self.kind} '{self.name}'>"


class Station(Item):
    kind = "station"


class Frame(Item):
    kind = "frame"


@dataclass(eq=False)
class Object(Item):
    mesh_path: Optional[str] = None
    color: tuple = (0.7, 0.7, 0.7)
    box_size: Optional[tuple] = None  # prymityw: prostopadłościan (dx, dy, dz) od początku układu, mm
    kind = "object"


@dataclass(eq=False)
class Tool(Item):
    """pose = TCP względem kołnierza (lub względem innego TCP przez parent_tcp)."""
    mesh_path: Optional[str] = None
    kind = "tool"

    def tcp(self) -> np.ndarray:
        T = self.pose.copy()
        p = self.parent
        while isinstance(p, Tool):  # TCP względny do innego TCP
            T = p.pose @ T
            p = p.parent
        return T


@dataclass(eq=False)
class Target(Item):
    joints: Optional[np.ndarray] = None
    joint_target: bool = False  # True = cel przegubowy
    kind = "target"


@dataclass(eq=False)
class Curve(Item):
    """Lista punktów (Nx3) + opcjonalne normalne (Nx3) w układzie rodzica."""
    points: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    normals: Optional[np.ndarray] = None
    kind = "curve"


@dataclass(eq=False)
class Robot(Item):
    model: object = None  # kinematics.robot_model.RobotModel
    joints: np.ndarray = field(default_factory=lambda: np.zeros(6))
    active_tool: Optional[Tool] = None
    active_frame: Optional[Frame] = None
    post_processor: str = "kuka"
    kind = "robot"

    def child_base_pose(self) -> np.ndarray:
        return self.pose  # baza; narzędzia liczone osobno przez flange()

    def flange_abs(self) -> np.ndarray:
        return self.pose_abs() @ self.model.fk(self.joints)

    def tcp_abs(self) -> np.ndarray:
        tcp = self.active_tool.tcp() if self.active_tool else P.eye()
        return self.flange_abs() @ tcp

    def tools(self) -> list[Tool]:
        return [c for c in self.walk() if isinstance(c, Tool)]

    def set_tool(self, tool: Tool) -> None:
        self.active_tool = tool

    def set_frame(self, frame: Frame) -> None:
        self.active_frame = frame

    def frame_rel_base(self, frame: Optional[Item]) -> np.ndarray:
        """Układ odniesienia względem bazy robota."""
        if frame is None:
            return P.eye()
        return P.inv(self.pose_abs()) @ frame.pose_abs()

    def teach_target(self, name: str = "Target") -> Target:
        """Ctrl+T: zapisuje bieżącą pozę TCP względem aktywnego układu."""
        fr = self.active_frame or self
        tcp = self.active_tool.tcp() if self.active_tool else P.eye()
        T_ref = P.inv(self.frame_rel_base(self.active_frame)) @ self.model.fk(self.joints) @ tcp
        t = Target(name=name, pose=T_ref, joints=self.joints.copy())
        fr.add(t)
        return t
