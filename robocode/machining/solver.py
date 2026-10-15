"""Projekt obróbki: Toolpath -> program robota.

Robot 6-osiowy ma 1 nadmiarowy stopień swobody względem 5-osiowej ścieżki: obrót wokół osi Z narzędzia.
Solver dla każdego punktu przeszukuje dozwolony obrót (±range, krok step) i wybiera rozwiązanie
osiągalne, z dala od osobliwości i najbliższe poprzedniemu (ciągłość przegubów).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..core import pose as P
from ..program.instructions import MoveJ, MoveL, Program, SetRounding, SetSpeed, Call
from .toolpath import Toolpath


@dataclass
class MachiningSettings:
    path_to_tool: np.ndarray = field(default_factory=P.eye)   # rotz(θ) itd.
    algorithm: str = "min_orientation_change"  # | "follow_path" | "robot_holds_object"
    z_range_deg: float = 180.0
    z_step_deg: float = 20.0
    approach_mm: float = 100.0
    rounding_mm: float = 1.0
    speed_mm_s: float = 50.0
    rapid_speed_mm_s: float = 250.0
    start_joints: np.ndarray | None = None
    tool_flip: bool = True  # narzędzie skierowane PRZECIWNIE do normalnej ścieżki (frez wchodzi w materiał)


@dataclass
class MachiningResult:
    ok: bool
    program: Program | None
    joints: list[np.ndarray]
    reachable: list[bool]
    message: str


def _target_pose(pt, prev_x, s: MachiningSettings) -> np.ndarray:
    n = -pt.n if s.tool_flip else pt.n
    T = P.pose_from_point_normal(pt.p, n, xdir=prev_x)
    return T @ s.path_to_tool


def _cost(model, q, q_prev) -> float:
    c = float(np.abs(q - q_prev).sum()) if q_prev is not None else 0.0
    m = model.manipulability(q)
    c += 50.0 / (m + 1e-3)
    if abs(q[4]) < 10:
        c += 500.0
    return c


def solve(robot, frame_rel_base: np.ndarray, tool_tcp: np.ndarray, toolpath: Toolpath,
          s: MachiningSettings | None = None, name: str = "Machining") -> MachiningResult:
    s = s or MachiningSettings()
    model = robot.model
    tp = toolpath.with_approach_retract(s.approach_mm) if s.approach_mm > 0 else toolpath
    zs = np.arange(-s.z_range_deg, s.z_range_deg + 1e-9, max(s.z_step_deg, 1e-3)) if s.z_range_deg > 0 else [0.0]
    zs = sorted(zs, key=abs)

    q_prev = np.array(s.start_joints if s.start_joints is not None else robot.joints, float)
    prev_x = None
    joints, reach, poses = [], [], []
    tool_inv = P.inv(tool_tcp)
    for i, pt in enumerate(tp.points):
        base_T = _target_pose(pt, prev_x if s.algorithm == "min_orientation_change" else None, s)
        best = None
        for z in zs:
            T = frame_rel_base @ base_T @ P.rotz(z) @ tool_inv
            q = model.ik(T, q_prev, iters=80)
            if q is None:
                continue
            if i > 0 and np.abs(q - q_prev).max() > 45:
                continue
            c = _cost(model, q, q_prev) + abs(z) * 0.05
            if best is None or c < best[0]:
                best = (c, q, z)
            if s.algorithm != "min_orientation_change" or z == 0:
                break  # wystarczy pierwsze dobre
        if best is None:
            reach.append(False)
            joints.append(q_prev)
            poses.append(base_T)
            continue
        _, q, z = best
        reach.append(True)
        joints.append(q)
        Tz = base_T @ P.rotz(z)
        poses.append(Tz)
        prev_x = Tz[:3, 0]
        q_prev = q

    if not all(reach):
        bad = [i for i, r in enumerate(reach) if not r]
        return MachiningResult(False, None, joints, reach,
                               f"✗ {len(bad)} z {len(reach)} punktów nieosiągalnych (pierwszy: {bad[0]}). "
                               "Przesuń układ detalu, zwiększ zakres obrotu Z lub zmień konfigurację startową.")

    prog = Program(name=name, robot=robot)
    prog.append(SetRounding(s.rounding_mm))
    prog.append(MoveJ(joints=joints[0], pose=poses[0]))
    cur_speed = None
    for pt, q, T in zip(tp.points[1:], joints[1:], poses[1:]):
        for ev in pt.events:
            prog.append(Call(ev))
        sp = s.rapid_speed_mm_s if pt.rapid else (pt.feed / 60.0 if pt.feed else s.speed_mm_s)
        if sp != cur_speed:
            prog.append(SetSpeed(linear_mm_s=sp))
            cur_speed = sp
        prog.append(MoveL(pose=T, joints=q))
    return MachiningResult(True, prog, joints, reach, f"✓ Program gotowy: {len(tp)} punktów")
