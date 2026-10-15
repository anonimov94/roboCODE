"""Program simulation: interpolation, full-trajectory checks, cycle time, TCP trace.

Every problem is reported as a structured Issue (see program/report.py) with the instruction index
and the NC source line, so the validator and the GUI can point the operator at the exact place.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..core import pose as P
from ..collision.checker import CollisionChecker
from ..kinematics.configs import near_singularity
from .instructions import MoveC, MoveJ, MoveL, Pause, Program, SetFrame, SetSpeed, SetTool
from .report import Severity, ValidationReport

E, W, INFO = Severity.ERROR, Severity.WARNING, Severity.INFO


@dataclass
class SimTolerances:
    """Explicit, unit-carrying tolerances (never one epsilon for everything)."""
    step_mm: float = 5.0             # Cartesian sampling of MoveL
    step_deg: float = 2.0            # joint sampling of MoveJ
    step_ori_deg: float = 2.0        # orientation sampling of MoveL (pure tool rotations too)
    ori_speed_deg_s: float = 90.0    # TCP orientation speed used for cycle time (controller-like)
    linear_acc_mm_s2: float = 500.0  # estimate only; overridden by SetSpeed.linear_acc
    joint_acc_deg_s2: float = 300.0  # estimate only; overridden by SetSpeed.joint_acc
    max_joint_jump_deg: float = 30.0  # between two MoveL samples -> configuration change
    limit_warn_margin_deg: float = 2.0
    wrist_singular_deg: float = 5.0
    end_pos_tol_mm: float = 1e-3     # accepting the solver's stored joints at a segment end
    end_ori_tol_deg: float = 1e-3

    def __post_init__(self) -> None:  # review (ChatGPT): invalid tolerances must not disable checks
        for k, v in vars(self).items():
            if not (isinstance(v, (int, float)) and np.isfinite(v) and v > 0):
                raise ValueError(f"SimTolerances.{k} musi być skończoną liczbą > 0 (jest {v!r})")


@dataclass
class SimResult:
    ok: bool
    cycle_time_s: float
    trace: list[np.ndarray] = field(default_factory=list)  # TCP positions (abs, mm)
    joints: list[np.ndarray] = field(default_factory=list)
    report: ValidationReport = field(default_factory=ValidationReport)
    max_contour_deviation_mm: float = 0.0
    mean_contour_deviation_mm: float = 0.0
    contour_samples: int = 0

    # backwards-compatible string views
    @property
    def errors(self) -> list[str]:
        return [i.text() for i in self.report.errors]

    @property
    def warnings(self) -> list[str]:
        return [i.text() for i in self.report.warnings]


def _interp_pose(A, B, t):
    """Linear position + slerp of orientation (axis-angle)."""
    T = np.eye(4)
    T[:3, 3] = (1 - t) * A[:3, 3] + t * B[:3, 3]
    Rd = A[:3, :3].T @ B[:3, :3]
    ang = np.arccos(np.clip((np.trace(Rd) - 1) / 2, -1, 1))
    if ang < 1e-9:
        T[:3, :3] = A[:3, :3]
    elif np.pi - ang < 1e-6:
        # 180 deg: axis from the symmetric part
        M = (Rd + np.eye(3)) / 2
        axis = M[:, int(np.argmax(np.diag(M)))]
        axis = axis / np.linalg.norm(axis)
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        a = ang * t
        T[:3, :3] = A[:3, :3] @ (np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K)
    else:
        axis = np.array([Rd[2, 1] - Rd[1, 2], Rd[0, 2] - Rd[2, 0], Rd[1, 0] - Rd[0, 1]]) / (2 * np.sin(ang))
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        a = ang * t
        R = np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
        T[:3, :3] = A[:3, :3] @ R
    return T


def _circle_through(pA, pM, pE):
    """Circle through 3 points -> (centre, unit normal, radius, total angle A->E passing M) or None."""
    a, b = pM - pA, pE - pA
    nrm = np.cross(a, b)
    nn = np.linalg.norm(nrm)
    la, lb = np.linalg.norm(a), np.linalg.norm(b)
    # relative test: IK noise (~1e-3 mm) on collinear points must not create a 'circle' of km radius
    if la < 1e-3 or lb < 1e-3 or nn / (la * lb) < 1e-3:
        return None
    nrm = nrm / nn
    # centre = pA + alpha*a + beta*b solving |c-pA|=|c-pM|=|c-pE| in the plane
    M = np.array([[a @ a, a @ b], [a @ b, b @ b]])
    rhs = 0.5 * np.array([a @ a, b @ b])
    alpha, beta = np.linalg.solve(M, rhs)
    c = pA + alpha * a + beta * b
    r = float(np.linalg.norm(pA - c))
    e1 = (pA - c) / r
    e2 = np.cross(nrm, e1)

    def ang(p):
        v = p - c
        return float(np.arctan2(v @ e2, v @ e1)) % (2 * np.pi)
    total = ang(pE) or 2 * np.pi  # nrm = a x b makes A->M->E counter-clockwise, so M lies inside (0, total)
    return c, e1, e2, r, total


def _trapezoidal_time(distance: float, speed: float, acceleration: float) -> float:
    """Shortest rest-to-rest time for a symmetric trapezoidal (or triangular) profile."""
    if distance <= 0:
        return 0.0
    ramp_distance = speed * speed / acceleration
    if distance <= ramp_distance:
        return 2.0 * np.sqrt(distance / acceleration)
    return 2.0 * speed / acceleration + (distance - ramp_distance) / speed


def simulate(program: Program, step_mm: float | None = None, tol: SimTolerances | None = None,
             restore_joints: bool = False, collision_checker: CollisionChecker | None = None) -> SimResult:
    tol = tol or SimTolerances()
    if step_mm is not None:
        tol.step_mm = step_mm
    r = program.robot
    m = r.model
    collision_checker = collision_checker or CollisionChecker()
    root = r
    while root.parent is not None:
        root = root.parent
    q0 = r.joints.copy()
    q = r.joints.copy()
    frame = P.eye()
    tool = P.eye()
    v_lin, v_j = 100.0, 50.0  # mm/s, deg/s (defaults until SetSpeed)
    a_lin, a_j = tol.linear_acc_mm_s2, tol.joint_acc_deg_s2
    linear_acceleration_is_assumed = True
    joint_acceleration_is_assumed = True
    assumed_accelerations: set[str] = set()
    vmax = np.asarray(m.vmax_deg_s, float)
    rep = ValidationReport()
    res = SimResult(ok=True, cycle_time_s=0.0, report=rep)
    warned: set[tuple] = set()
    contour_deviation_sum = 0.0

    def tcp_abs(qq):
        return r.pose_abs() @ m.fk(qq) @ tool

    def warn_once(key, *a, **kw):
        if key not in warned:
            warned.add(key)
            rep.add(*a, **kw)

    def check_sample(idx, line, qq):
        for j, (v, (lo, hi)) in enumerate(zip(qq, m.limits_deg)):
            if v < lo + tol.limit_warn_margin_deg or v > hi - tol.limit_warn_margin_deg:
                warn_once(("lim", idx, j), W, "NEAR_JOINT_LIMIT",
                          f"Oś {j + 1} = {v:.1f}° blisko limitu [{lo:g}, {hi:g}]",
                          instruction=idx, source_line=line, hint="zmień obrót wokół Z lub konfigurację startową")
        s = near_singularity(m, qq, wrist_deg=tol.wrist_singular_deg)
        if s:
            warn_once(("sing", idx), W, "SINGULARITY", s, instruction=idx, source_line=line)
        for hit in collision_checker.check(root, r, qq):
            warn_once(("collision", idx, hit.pair), W, "COLLISION_PROXY",
                      f"Możliwa kolizja {hit.component} z obiektem {hit.obstacle} "
                      "(test uproszczonych brył)",
                      instruction=idx, source_line=line,
                      hint="sprawdź dokładną geometrię w symulatorze producenta")

    for idx, ins in enumerate(program.instructions):
        line = getattr(ins, "src_line", None)
        if isinstance(ins, SetFrame):
            frame = ins.pose
        elif isinstance(ins, SetTool):
            tool = ins.pose
        elif isinstance(ins, SetSpeed):
            # non-positive / non-finite speeds are blocking static errors; never silently keep the old value
            if ins.linear_mm_s is not None and np.isfinite(ins.linear_mm_s) and ins.linear_mm_s > 0:
                v_lin = ins.linear_mm_s
            if ins.joint_deg_s is not None and np.isfinite(ins.joint_deg_s) and ins.joint_deg_s > 0:
                v_j = ins.joint_deg_s
            if ins.linear_acc is not None and np.isfinite(ins.linear_acc) and ins.linear_acc > 0:
                a_lin = ins.linear_acc
                linear_acceleration_is_assumed = False
            if ins.joint_acc is not None and np.isfinite(ins.joint_acc) and ins.joint_acc > 0:
                a_j = ins.joint_acc
                joint_acceleration_is_assumed = False
        elif isinstance(ins, Pause) and ins.ms > 0:
            res.cycle_time_s += ins.ms / 1000
        elif isinstance(ins, MoveJ):
            target = ins.joints
            if target is None and ins.pose is not None:
                target = m.ik(frame @ ins.pose @ P.inv(tool), q)
            if target is None:
                rep.add(E, "UNREACHABLE", "MoveJ: cel nieosiągalny", instruction=idx, source_line=line,
                        hint="przesuń układ detalu bliżej robota")
                continue
            target = np.asarray(target, float)
            if not m.within_limits(target):
                rep.add(E, "JOINT_LIMIT", f"MoveJ: cel poza limitami osi {np.round(target, 1).tolist()}",
                        instruction=idx, source_line=line)
                continue
            dq = np.abs(target - q)
            if dq.max() > 0 and joint_acceleration_is_assumed:
                assumed_accelerations.add(f"{a_j:g}°/s² dla ruchów osiowych")
            joint_speeds = np.minimum(v_j, vmax)
            axis_times = [_trapezoidal_time(float(distance), float(speed), a_j)
                          for distance, speed in zip(dq, joint_speeds)]
            t_move = max(axis_times, default=0.0)
            res.cycle_time_s += t_move
            for t in np.linspace(0, 1, max(2, int(dq.max() / tol.step_deg) + 1)):
                qq = q + t * (target - q)
                res.joints.append(qq)
                res.trace.append(tcp_abs(qq)[:3, 3])
                check_sample(idx, line, qq)
                rep.samples_checked += 1
            q = target
        elif isinstance(ins, (MoveL, MoveC)):
            goals = [ins.pose_end] if isinstance(ins, MoveC) else [ins.pose]
            failed = False
            for goal in goals:
                if failed:
                    break
                A = m.fk(q) @ tool
                B = frame @ goal
                if not np.all(np.isfinite(B)):
                    rep.add(E, "BAD_POSE", "MoveL: poza celu zawiera NaN/inf", instruction=idx, source_line=line)
                    failed = True
                    break
                arc = None
                if isinstance(ins, MoveC):
                    # real circular path through the auxiliary point (review: ChatGPT); orientation A->B like #VAR
                    pm = (frame @ ins.pose_mid)[:3, 3]
                    arc = _circle_through(A[:3, 3], pm, B[:3, 3])
                    if arc is None:
                        rep.add(E, "MOVEC_DEGENERATE", "MoveC: punkty współliniowe lub zbyt blisko – łuk nieokreślony",
                                instruction=idx, source_line=line, hint="użyj MoveL")
                        failed = True
                        break
                    dist = arc[3] * arc[4]
                else:
                    dist = float(np.linalg.norm(B[:3, 3] - A[:3, 3]))
                ang = P.pose_distance(A, B)[1]
                if dist > 0 and linear_acceleration_is_assumed:
                    assumed_accelerations.add(f"{a_lin:g} mm/s² dla ruchów liniowych")
                n = max(2, int(np.ceil(max(dist / tol.step_mm, ang / tol.step_ori_deg))) + 1)
                # Rest-to-rest translation estimate; orientation remains speed-limited.
                seg_time = max(_trapezoidal_time(dist, v_lin, a_lin), ang / tol.ori_speed_deg_s)
                res.cycle_time_s += seg_time
                dt = seg_time / (n - 1) if n > 1 else 0.0
                known = ins.joints_end if isinstance(ins, MoveC) else ins.joints
                for t in np.linspace(0, 1, n)[1:]:
                    Ti = _interp_pose(A, B, t)
                    if arc is not None:
                        c, e1, e2, r_arc, total = arc
                        phi = t * total
                        Ti[:3, 3] = c + r_arc * (np.cos(phi) * e1 + np.sin(phi) * e2)
                    qn = m.ik(Ti @ P.inv(tool), q, iters=100)
                    if qn is None and t == 1.0 and known is not None and m.within_limits(known):
                        dp_k, da_k = P.pose_distance(m.fk(known) @ tool, B)
                        if dp_k < tol.end_pos_tol_mm and da_k < tol.end_ori_tol_deg:  # position AND orientation
                            qn = np.asarray(known, float)  # the solver's verified solution for the target
                    if qn is None:
                        rep.add(E, "UNREACHABLE", f"MoveL: punkt na {t * 100:.0f}% odcinka nieosiągalny lub poza "
                                "limitem osi", instruction=idx, source_line=line,
                                hint="przesuń układ detalu lub zmień obrót wokół Z narzędzia")
                        failed = True
                        break
                    jump = np.abs(qn - q)
                    if jump.max() > tol.max_joint_jump_deg:
                        rep.add(E, "CONFIG_JUMP", f"MoveL: skok osi {int(np.argmax(jump)) + 1} o {jump.max():.0f}° "
                                "(zmiana konfiguracji)", instruction=idx, source_line=line, hint="użyj MoveJ")
                        failed = True
                        break
                    if dt > 0:
                        ratio = (jump / dt) / vmax
                        if ratio.max() > 1.0:
                            j = int(np.argmax(ratio))
                            warn_once(("vel", idx), W, "JOINT_SPEED",
                                      f"Oś {j + 1} potrzebuje {jump[j] / dt:.0f}°/s > max {vmax[j]:g}°/s – robot "
                                      "nie utrzyma zadanego posuwu", instruction=idx, source_line=line,
                                      hint="zmniejsz posuw lub odsuń ścieżkę od osobliwości")
                    q = qn
                    if t == 1.0 and known is not None and np.abs(q - np.asarray(known, float)).max() > 5.0:
                        warn_once(("cfg", idx), W, "CONFIG_DIFFERS",
                                  "Robot dojdzie do celu w innej konfiguracji osi niż wyliczył solver "
                                  f"(różnica {np.abs(q - np.asarray(known, float)).max():.0f}°)",
                                  instruction=idx, source_line=line,
                                  hint="sprawdź kolejny MoveJ / konfigurację startową")
                    res.joints.append(q)
                    res.trace.append(tcp_abs(q)[:3, 3])
                    contour_deviation = float(np.linalg.norm(
                        res.trace[-1] - (r.pose_abs() @ Ti)[:3, 3]
                    ))
                    res.max_contour_deviation_mm = max(res.max_contour_deviation_mm, contour_deviation)
                    contour_deviation_sum += contour_deviation
                    res.contour_samples += 1
                    check_sample(idx, line, q)
                    rep.samples_checked += 1
    if res.contour_samples:
        res.mean_contour_deviation_mm = contour_deviation_sum / res.contour_samples
    if assumed_accelerations:
        rep.add(INFO, "ACCELERATION_ASSUMED",
                "Czas cyklu używa przyjętego przyspieszenia symulatora: "
                + ", ".join(sorted(assumed_accelerations))
                + "; podaj SetSpeed(linear_acc/joint_acc), aby je zastąpić. "
                "Wartości nie pochodzą z karty konkretnego robota.",
                stage="trajectory")
    rep.cycle_time_s = res.cycle_time_s
    res.ok = rep.ok
    r.joints = q0 if restore_joints else q
    return res
