"""Projekt obróbki: Toolpath -> program robota.

Robot 6-osiowy ma 1 nadmiarowy stopień swobody względem 5-osiowej ścieżki: obrót wokół osi Z narzędzia.
Solver dla każdego punktu przeszukuje dozwolony obrót (±range, krok step) i wybiera rozwiązanie
osiągalne, z dala od osobliwości i najbliższe poprzedniemu (ciągłość przegubów).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..core import pose as P
from ..kinematics.configs import config_flags
from ..program.instructions import Call, MoveJ, MoveL, Pause, Program, SetRounding, SetSpeed
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
    start_configuration_index: int | None = None  # 1-based index from enumerate_start_configurations()
    max_orientation_step_deg: float = 5.0  # densify big tool-axis changes (0 = off)
    tool_flip: bool = True  # narzędzie skierowane PRZECIWNIE do normalnej ścieżki (frez wchodzi w materiał)


@dataclass(frozen=True)
class StartConfiguration:
    orientation_z_deg: float
    joints: np.ndarray
    cost: float
    label: str


@dataclass
class MachiningResult:
    ok: bool
    program: Program | None
    joints: list[np.ndarray]
    reachable: list[bool]
    message: str
    toolpath: Toolpath | None = None  # the exact path solved (after approach/retract + densification)
    start_configurations: list[StartConfiguration] = field(default_factory=list)


def _tag(ins, line):
    """Remember the NC source line of an instruction (for validation reports)."""
    ins.src_line = line
    return ins


def _event_instruction(ev: str):
    """Map a parser event to an instruction. 'Pause(ms)' -> Pause, everything else -> Call."""
    if ev.startswith("Pause(") and ev.endswith(")"):
        return Pause(float(ev[6:-1]))
    return Call(ev)


def _target_pose(pt, prev_x, s: MachiningSettings) -> np.ndarray:
    n = -pt.n if s.tool_flip else pt.n
    T = P.pose_from_point_normal(pt.p, n, xdir=prev_x)
    return T @ s.path_to_tool


def _prepared_toolpath(toolpath: Toolpath, settings: MachiningSettings) -> Toolpath:
    tp = toolpath.with_approach_retract(settings.approach_mm) if settings.approach_mm > 0 else toolpath
    return tp.densify_orientation(settings.max_orientation_step_deg)


def enumerate_start_configurations(robot, frame_rel_base: np.ndarray, tool_tcp: np.ndarray, toolpath: Toolpath,
                                   settings: MachiningSettings | None = None) -> list[StartConfiguration]:
    """List reachable first-point IK branches, ranked by seed proximity and manipulability."""
    s = settings or MachiningSettings()
    tp = _prepared_toolpath(toolpath, s)
    if not tp.points:
        return []
    model = robot.model
    seed = np.asarray(s.start_joints if s.start_joints is not None else robot.joints, float)
    zs = np.arange(-s.z_range_deg, s.z_range_deg + 1e-9, max(s.z_step_deg, 1e-3)) \
        if s.z_range_deg > 0 else [0.0]
    if s.algorithm != "min_orientation_change":
        zs = [0.0]
    prev_x = None
    base_T = _target_pose(tp.points[0], prev_x, s)
    tool_inv = P.inv(tool_tcp)
    candidates: list[StartConfiguration] = []
    for z in sorted(zs, key=abs):
        target = frame_rel_base @ base_T @ P.rotz(z) @ tool_inv
        for q in model.ik_all(target, seeds=[seed]):
            q = np.asarray(q, float)
            if q.shape != (model.n,) or not np.all(np.isfinite(q)) or not model.within_limits(q):
                continue
            if any(abs(z - old.orientation_z_deg) < 1e-6 and np.allclose(q, old.joints, atol=0.5)
                   for old in candidates):
                continue
            cost = _cost(model, q, seed) + abs(z) * 0.05
            label = config_flags(model, q)["label"]
            candidates.append(StartConfiguration(float(z), q.copy(), cost, label))
    return sorted(candidates, key=lambda item: item.cost)


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
    tp = _prepared_toolpath(toolpath, s)
    zs = np.arange(-s.z_range_deg, s.z_range_deg + 1e-9, max(s.z_step_deg, 1e-3)) if s.z_range_deg > 0 else [0.0]
    zs = sorted(zs, key=abs)

    q_prev = np.array(s.start_joints if s.start_joints is not None else robot.joints, float)
    initial_configurations: list[StartConfiguration] = []
    selected_start: StartConfiguration | None = None
    if s.start_configuration_index is not None:
        initial_configurations = enumerate_start_configurations(robot, frame_rel_base, tool_tcp, toolpath, s)
        if not isinstance(s.start_configuration_index, int) or isinstance(s.start_configuration_index, bool) \
                or not 1 <= s.start_configuration_index <= len(initial_configurations):
            return MachiningResult(
                False, None, [], [], f"✗ Nieprawidłowy wybór konfiguracji początkowej "
                f"{s.start_configuration_index}; dostępny zakres: 1–{len(initial_configurations)}",
                toolpath=tp, start_configurations=initial_configurations)
        selected_start = initial_configurations[s.start_configuration_index - 1]
    prev_x = None
    joints, reach, poses = [], [], []
    reasons: list[str] = []  # per unreachable point: "ik" (no solution) | "jump" (only with a config jump)
    tool_inv = P.inv(tool_tcp)
    for i, pt in enumerate(tp.points):
        base_T = _target_pose(pt, prev_x if s.algorithm == "min_orientation_change" else None, s)
        best = None
        jump_rejected = False
        if i == 0 and selected_start is not None:
            best = (selected_start.cost, selected_start.joints, selected_start.orientation_z_deg)
        else:
            for z in zs:
                T = frame_rel_base @ base_T @ P.rotz(z) @ tool_inv
                q = model.ik(T, q_prev, iters=80)
                if q is None:
                    continue
                if i > 0 and np.abs(q - q_prev).max() > 45:
                    jump_rejected = True
                    continue
                c = _cost(model, q, q_prev) + abs(z) * 0.05
                if best is None or c < best[0]:
                    best = (c, q, z)
                if s.algorithm != "min_orientation_change" or z == 0:
                    break  # wystarczy pierwsze dobre
        if best is None:
            reasons.append("jump" if jump_rejected else "ik")
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
        if reasons[0] == "jump":
            msg = (f"✗ {len(bad)} z {len(reach)} punktów wymaga skoku konfiguracji robota (pierwszy: {bad[0]}). "
                   "Zmniejsz max_orientation_step_deg, zwiększ zakres obrotu Z lub zmień konfigurację startową.")
        else:
            msg = (f"✗ {len(bad)} z {len(reach)} punktów nieosiągalnych (pierwszy: {bad[0]}). "
                   "Przesuń układ detalu, zwiększ zakres obrotu Z lub zmień konfigurację startową.")
        return MachiningResult(False, None, joints, reach, msg, toolpath=tp,
                               start_configurations=initial_configurations)

    prog = Program(name=name, robot=robot)
    prog.append(SetRounding(s.rounding_mm))
    for ev in tp.points[0].events:  # e.g. spindle on before the first move (approach_mm == 0)
        _tag(prog.append(_event_instruction(ev)), tp.points[0].line)
    _tag(prog.append(MoveJ(joints=joints[0], pose=poses[0])), tp.points[0].line)
    cur_speed = None
    for pt, q, T in zip(tp.points[1:], joints[1:], poses[1:]):
        for ev in pt.events:
            _tag(prog.append(_event_instruction(ev)), pt.line)
        sp = s.rapid_speed_mm_s if pt.rapid else (pt.feed / 60.0 if pt.feed else s.speed_mm_s)
        if sp != cur_speed:
            _tag(prog.append(SetSpeed(linear_mm_s=sp)), pt.line)
            cur_speed = sp
        _tag(prog.append(MoveL(pose=T, joints=q)), pt.line)
    for ev in tp.end_events:
        prog.append(_event_instruction(ev))
    return MachiningResult(True, prog, joints, reach, f"✓ Program gotowy: {len(tp)} punktów", toolpath=tp,
                           start_configurations=initial_configurations)
