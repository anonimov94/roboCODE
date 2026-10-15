"""Zapis/odczyt stacji (.rcs.json) – własny, otwarty format."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .scene import Curve, Frame, Item, Object, Robot, Station, Target, Tool

FORMAT = "robocode-station/1"


def _ser(item: Item) -> dict:
    d = {"type": item.kind, "name": item.name, "pose": item.pose.tolist(), "visible": item.visible,
         "children": [_ser(c) for c in item.children if c.kind != "program"]}
    if isinstance(item, Robot):
        d["model"] = json.loads(item.model.to_json())
        d["joints"] = item.joints.tolist()
        d["post"] = item.post_processor
        d["active_tool"] = item.active_tool.name if item.active_tool else None
        d["active_frame"] = item.active_frame.name if item.active_frame else None
    if isinstance(item, Target):
        d["joints"] = None if item.joints is None else item.joints.tolist()
        d["joint_target"] = item.joint_target
    if isinstance(item, Curve):
        d["points"] = item.points.tolist()
        d["normals"] = None if item.normals is None else item.normals.tolist()
    if isinstance(item, (Object, Tool)):
        d["mesh"] = item.mesh_path
    return d


def save_station(st: Station, path: str | Path) -> None:
    Path(path).write_text(json.dumps({"format": FORMAT, "root": _ser(st)}, indent=1), encoding="utf-8")


def _de(d: dict, root: Station | None = None) -> Item:
    from ..kinematics.robot_model import RobotModel
    t = d["type"]
    common = dict(name=d["name"], pose=np.array(d["pose"]), visible=d.get("visible", True))
    if t == "station":
        it = Station(**common)
    elif t == "frame":
        it = Frame(**common)
    elif t == "robot":
        it = Robot(**common, model=RobotModel.from_json(json.dumps(d["model"])),
                   joints=np.array(d["joints"]), post_processor=d.get("post", "kuka"))
    elif t == "tool":
        it = Tool(**common, mesh_path=d.get("mesh"))
    elif t == "object":
        it = Object(**common, mesh_path=d.get("mesh"))
    elif t == "target":
        it = Target(**common, joints=None if d["joints"] is None else np.array(d["joints"]),
                    joint_target=d.get("joint_target", False))
    elif t == "curve":
        it = Curve(**common, points=np.array(d["points"]),
                   normals=None if d.get("normals") is None else np.array(d["normals"]))
    else:
        it = Item(**common)
    for c in d.get("children", []):
        it.add(_de(c))
    return it


def load_station(path: str | Path) -> Station:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("format") != FORMAT:
        raise ValueError("To nie jest plik stacji roboCODE")
    st = _de(data["root"])
    for r in st.walk():  # przywróć aktywne narzędzie/układ
        if isinstance(r, Robot):
            src = _find_dict(data["root"], r.name)
            if src and src.get("active_tool"):
                r.active_tool = r.find(src["active_tool"])
            if src and src.get("active_frame"):
                r.active_frame = st.find(src["active_frame"])
    return st


def _find_dict(d, name):
    if d["name"] == name and d["type"] == "robot":
        return d
    for c in d.get("children", []):
        f = _find_dict(c, name)
        if f:
            return f
    return None
