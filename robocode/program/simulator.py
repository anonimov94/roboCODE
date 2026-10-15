"""Symulacja programu: interpolacja, walidacja MoveL, czas cyklu, ślad TCP."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..core import pose as P
from ..kinematics.configs import near_singularity
from .instructions import MoveC, MoveJ, MoveL, Pause, Program, SetFrame, SetSpeed, SetTool


@dataclass
class SimResult:
    ok: bool
    cycle_time_s: float
    trace: list[np.ndarray] = field(default_factory=list)  # pozycje TCP (abs, mm)
    joints: list[np.ndarray] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


def _interp_pose(A, B, t):
    """Liniowo pozycja + slerp orientacji (przez oś-kąt)."""
    T = np.eye(4)
    T[:3, 3] = (1 - t) * A[:3, 3] + t * B[:3, 3]
    Rd = A[:3, :3].T @ B[:3, :3]
    ang = np.arccos(np.clip((np.trace(Rd) - 1) / 2, -1, 1))
    if ang < 1e-9:
        T[:3, :3] = A[:3, :3]
    else:
        axis = np.array([Rd[2, 1] - Rd[1, 2], Rd[0, 2] - Rd[2, 0], Rd[1, 0] - Rd[0, 1]]) / (2 * np.sin(ang))
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        a = ang * t
        R = np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
        T[:3, :3] = A[:3, :3] @ R
    return T


def simulate(program: Program, step_mm: float = 5.0) -> SimResult:
    r = program.robot
    m = r.model
    q = r.joints.copy()
    frame = P.eye()
    tool = P.eye()
    v_lin, v_j = 100.0, 50.0  # mm/s, deg/s
    res = SimResult(ok=True, cycle_time_s=0.0)

    def tcp_abs(qq):
        return r.pose_abs() @ m.fk(qq) @ tool

    for idx, ins in enumerate(program.instructions):
        if isinstance(ins, SetFrame):
            frame = ins.pose
        elif isinstance(ins, SetTool):
            tool = ins.pose
        elif isinstance(ins, SetSpeed):
            v_lin = ins.linear_mm_s or v_lin
            v_j = ins.joint_deg_s or v_j
        elif isinstance(ins, Pause) and ins.ms > 0:
            res.cycle_time_s += ins.ms / 1000
        elif isinstance(ins, MoveJ):
            target = ins.joints
            if target is None and ins.pose is not None:
                target = m.ik(frame @ ins.pose @ P.inv(tool), q)
            if target is None or not m.within_limits(target):
                res.ok = False
                res.errors.append(f"[{idx}] MoveJ: cel nieosiągalny")
                continue
            dq = np.abs(np.asarray(target) - q).max()
            res.cycle_time_s += dq / v_j
            for t in np.linspace(0, 1, max(2, int(dq / 2))):
                qq = q + t * (np.asarray(target) - q)
                res.joints.append(qq)
                res.trace.append(tcp_abs(qq)[:3, 3])
            q = np.asarray(target, float)
        elif isinstance(ins, (MoveL, MoveC)):
            if isinstance(ins, MoveC):  # uproszczenie: łuk jako dwa odcinki (TODO prawdziwy łuk)
                segs = [ins.pose_mid, ins.pose_end]
            else:
                segs = [ins.pose]
            for goal in segs:
                A = m.fk(q) @ tool
                B = frame @ goal
                dist = np.linalg.norm(B[:3, 3] - A[:3, 3])
                n = max(2, int(dist / step_mm))
                res.cycle_time_s += dist / v_lin
                for t in np.linspace(0, 1, n)[1:]:
                    qn = m.ik(_interp_pose(A, B, t) @ P.inv(tool), q, iters=60)
                    if qn is None:
                        res.ok = False
                        res.errors.append(f"[{idx}] MoveL: punkt nieosiągalny / poza limitem osi")
                        break
                    if np.abs(qn - q).max() > 30:
                        res.ok = False
                        res.errors.append(f"[{idx}] MoveL: skok konfiguracji – użyj MoveJ")
                        break
                    w = near_singularity(m, qn)
                    if w and w not in res.warnings:
                        res.warnings.append(f"[{idx}] {w}")
                    q = qn
                    res.joints.append(q)
                    res.trace.append(tcp_abs(q)[:3, 3])
    r.joints = q
    return res
