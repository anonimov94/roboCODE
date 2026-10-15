"""Central pre-export validator (docs/SPECYFIKACJA.md §7).

    validate_program(program)            static + full-trajectory checks -> ValidationReport
    verify_post_output(post, text, prog) parses the generated KRL/RAPID back and compares poses

Export is allowed only when report.ok (no ERROR issues). Warnings are shown to the operator and
stored in the run manifest.
"""
from __future__ import annotations

import re
from dataclasses import dataclass

import numpy as np

from ..core import pose as P
from .instructions import Call, MoveC, MoveJ, MoveL, Pause, Program, SetFrame, SetRounding, SetSpeed, SetTool
from .report import Severity, ValidationReport
from .simulator import SimResult, SimTolerances, simulate

E, W, INFO = Severity.ERROR, Severity.WARNING, Severity.INFO

KNOWN_CONTROLLER_CALLS = ("SetTool", "Spindle", "SpindleOn", "SpindleOnCCW", "SpindleOff", "M_RunCode", "Extruder")


@dataclass
class ValidationLimits:
    max_linear_speed_mm_s: float = 1000.0  # anything above is almost certainly a units bug
    min_linear_speed_mm_s: float = 0.1
    max_rounding_mm: float = 50.0
    max_joint_speed_deg_s: float = 500.0
    target_pos_tol_mm: float = 0.05        # FK(joints) vs stored pose of each target
    target_ori_tol_deg: float = 0.05
    post_pos_tol_mm: float = 0.01          # parsed controller text vs program pose
    post_ori_tol_deg: float = 0.01
    post_joint_tol_deg: float = 0.001


def _static(program: Program, lim: ValidationLimits, mapped_events: set[str] | None = None) -> ValidationReport:
    rep = ValidationReport()
    m = program.robot.model if program.robot is not None else None
    if program.robot is None or m is None:
        rep.add(E, "NO_ROBOT", "Program nie ma przypisanego robota", stage="static")
        return rep
    moves = [i for i, ins in enumerate(program.instructions) if isinstance(ins, (MoveJ, MoveL, MoveC))]
    if not moves:
        rep.add(E, "EMPTY", "Program nie zawiera ruchów", stage="static")
        return rep
    first = moves[0]
    for kind, name in ((SetFrame, "SetFrame"), (SetTool, "SetTool")):
        if not any(isinstance(ins, kind) for ins in program.instructions[:first]):
            rep.add(E, "MISSING_" + name.upper(), f"Brak {name} przed pierwszym ruchem", stage="static",
                    instruction=first)
    frame = tool = None
    calls: set[str] = set()
    for idx, ins in enumerate(program.instructions):
        line = getattr(ins, "src_line", None)
        for attr in ("pose", "pose_mid", "pose_end"):
            T = getattr(ins, attr, None)
            if T is not None and not P.is_valid_pose(T):
                rep.add(E, "BAD_POSE", f"Niepoprawna macierz pozy ({attr}): NaN/inf lub obrót nieortonormalny",
                        stage="static", instruction=idx, source_line=line)
        for attr in ("joints", "joints_mid", "joints_end"):
            q = getattr(ins, attr, None)
            if q is None:
                continue
            q = np.asarray(q, float)
            if q.shape != (m.n,) or not np.all(np.isfinite(q)):
                rep.add(E, "BAD_JOINTS", f"Niepoprawny wektor osi ({attr})", stage="static", instruction=idx,
                        source_line=line)
            elif not m.within_limits(q):
                bad = [j + 1 for j, (v, (lo, hi)) in enumerate(zip(q, m.limits_deg)) if not lo <= v <= hi]
                rep.add(E, "JOINT_LIMIT", f"Osie {bad} poza limitami", stage="static", instruction=idx,
                        source_line=line)
        if isinstance(ins, SetFrame):
            frame = ins.pose
        elif isinstance(ins, SetTool):
            tool = ins.pose
        elif isinstance(ins, SetSpeed):
            v = ins.linear_mm_s
            if v is not None and (not np.isfinite(v) or v < lim.min_linear_speed_mm_s
                                  or v > lim.max_linear_speed_mm_s):
                rep.add(E, "SPEED_RANGE", f"Prędkość {v:g} mm/s poza zakresem "
                        f"[{lim.min_linear_speed_mm_s:g}, {lim.max_linear_speed_mm_s:g}]", stage="static",
                        instruction=idx, source_line=line, hint="sprawdź jednostki posuwu (mm/min vs mm/s, cale)")
            vj = ins.joint_deg_s  # review (ChatGPT): joint speed was not range-checked
            if vj is not None and (not np.isfinite(vj) or vj <= 0 or vj > lim.max_joint_speed_deg_s):
                rep.add(E, "JOINT_SPEED_RANGE", f"Prędkość osi {vj:g}°/s poza zakresem (0, "
                        f"{lim.max_joint_speed_deg_s:g}]", stage="static", instruction=idx, source_line=line)
        elif isinstance(ins, SetRounding):
            if ins.mm != -1 and not 0 <= ins.mm <= lim.max_rounding_mm:
                rep.add(E, "ROUNDING_RANGE", f"Zaokrąglenie {ins.mm:g} mm poza zakresem", stage="static",
                        instruction=idx)
        elif isinstance(ins, Pause) and ins.ms < 0:
            rep.add(INFO, "STOP", "Program zatrzyma się i czeka na operatora (M0/M1)", stage="static",
                    instruction=idx, source_line=line)
        elif isinstance(ins, Call):
            calls.add(ins.name.split("(")[0])
        # stored joints must actually produce the stored pose (catches solver / frame mix-ups)
        if isinstance(ins, (MoveJ, MoveL)) and ins.pose is not None and ins.joints is not None \
                and frame is not None and tool is not None:
            T_fk = P.inv(frame) @ m.fk(ins.joints) @ tool
            dp, da = P.pose_distance(T_fk, ins.pose)
            if dp > lim.target_pos_tol_mm or da > lim.target_ori_tol_deg:
                rep.add(E, "TARGET_MISMATCH", f"Osie celu nie dają zapisanej pozy (Δ={dp:.3f} mm, {da:.3f}°)",
                        stage="static", instruction=idx, source_line=line)
    calls -= set(mapped_events or ())
    if calls:
        rep.add(W, "CONTROLLER_SUBPROGRAMS", "Program wywołuje podprogramy, które muszą istnieć na sterowniku: "
                + ", ".join(sorted(calls)), stage="static",
                hint="przygotuj je na sterowniku albo zamapuj na I/O: --events examples/events_kuka.json")
    return rep


def validate_program(program: Program, limits: ValidationLimits | None = None,
                     tol: SimTolerances | None = None, parser_warnings: list[str] | None = None,
                     mapped_events: set[str] | None = None) -> tuple[ValidationReport, SimResult | None]:
    """Static checks, then the whole trajectory through the simulator. Robot joints are restored."""
    lim = limits or ValidationLimits()
    rep = ValidationReport()
    for wtxt in parser_warnings or []:
        rep.add(W, "PARSER", wtxt, stage="parse")
    st = _static(program, lim, mapped_events)
    rep.extend(st)
    if any(i.code in ("NO_ROBOT", "EMPTY", "BAD_POSE", "BAD_JOINTS") for i in st.errors):
        rep.add(E, "SIM_SKIPPED", "Symulacja trajektorii pominięta – najpierw usuń błędy danych", stage="trajectory")
        return rep, None
    sim = simulate(program, tol=tol, restore_joints=True)
    rep.extend(sim.report)
    # fail-safe wording (review: ChatGPT): never let a clean report look like a collision check
    rep.add(W, "COLLISION_NOT_CHECKED", "Kolizje (robot, narzędzie, detal, mocowania) NIE są jeszcze sprawdzane",
            stage="trajectory",
            hint="sprawdź przebieg w symulacji producenta i w trybie T1 przy zredukowanej prędkości")
    return rep, sim


# ---------------------------------------------------------------- post round trip
_NUM = r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?"
_KRL_FRAME = re.compile(r"\{X (%s),Y (%s),Z (%s),A (%s),B (%s),C (%s)\}" % ((_NUM,) * 6))
_KRL_PTP = re.compile(r"^\s*PTP \{A1 (%s),A2 (%s),A3 (%s),A4 (%s),A5 (%s),A6 (%s)\}" % ((_NUM,) * 6))
_ABB_ABSJ = re.compile(r"^\s*MoveAbsJ \[\[(%s),(%s),(%s),(%s),(%s),(%s)\]" % ((_NUM,) * 6))
_ABB_POSE = re.compile(r"\[\[(%s),(%s),(%s)\],\[(%s),(%s),(%s),(%s)\]" % ((_NUM,) * 7))  # [[x,y,z],[q1..q4]


def _program_cart_poses(program: Program) -> list[tuple[int, str, np.ndarray]]:
    out = []
    for idx, ins in enumerate(program.instructions):
        if isinstance(ins, SetFrame):
            out.append((idx, "frame", ins.pose))
        elif isinstance(ins, SetTool):
            out.append((idx, "tool", ins.pose))
        elif isinstance(ins, MoveL):
            out.append((idx, "movel", ins.pose))
        elif isinstance(ins, MoveC):
            out += [(idx, "movec", ins.pose_mid), (idx, "movec", ins.pose_end)]
    return out


def verify_post_output(post_name: str, text: str, program: Program,
                       limits: ValidationLimits | None = None) -> ValidationReport:
    """Re-read Cartesian poses from generated controller text and compare with the program.

    Catches Euler/quaternion convention and formatting bugs. It does NOT replace a check on the
    real controller (syntax, configuration data) – see docs §9.
    """
    lim = limits or ValidationLimits()
    rep = ValidationReport()
    if post_name == "kuka":
        parsed = [P.from_xyzabc(*map(float, mt.groups()), convention="KUKA_ABC")
                  for line in text.splitlines() for mt in _KRL_FRAME.finditer(line)]
    elif post_name == "abb":
        body = [ln for ln in text.splitlines()
                if ln.strip().startswith(("MoveL ", "MoveC ", "rcWobj.uframe", "rcTool.tframe"))]
        parsed = [P.from_quaternion(*map(float, mt.groups())) for ln in body for mt in _ABB_POSE.finditer(ln)]
    else:
        rep.add(INFO, "POST_NOT_VERIFIED", f"Post-procesor '{post_name}' nie ma weryfikacji zwrotnej", stage="post")
        return rep
    # joint-space commands (review: ChatGPT) – PTP / MoveAbsJ must carry exactly the validated joints
    jre = _KRL_PTP if post_name == "kuka" else _ABB_ABSJ
    got_j = [np.array(list(map(float, mt.groups()))) for line in text.splitlines() if (mt := jre.match(line))]
    exp_j = [(i, np.asarray(ins.joints, float)) for i, ins in enumerate(program.instructions)
             if isinstance(ins, MoveJ) and ins.joints is not None]
    if len(got_j) != len(exp_j):
        rep.add(E, "POST_JOINT_COUNT", f"Liczba ruchów osiowych w pliku ({len(got_j)}) ≠ w programie ({len(exp_j)})",
                stage="post")
    else:
        for (i, qe), qg in zip(exp_j, got_j):
            if np.abs(qe - qg).max() > lim.post_joint_tol_deg:
                rep.add(E, "POST_JOINT_MISMATCH", f"Osie w pliku różnią się od programu o {np.abs(qe - qg).max():.4f}°",
                        stage="post", instruction=i)
    expected = _program_cart_poses(program)
    if len(parsed) != len(expected):
        rep.add(E, "POST_COUNT", f"Liczba poz w pliku ({len(parsed)}) ≠ w programie ({len(expected)})", stage="post")
        return rep
    worst = (0.0, 0.0)
    for (idx, kind, T), Tp in zip(expected, parsed):
        dp, da = P.pose_distance(T, Tp)
        worst = (max(worst[0], dp), max(worst[1], da))
        if dp > lim.post_pos_tol_mm or da > lim.post_ori_tol_deg:
            rep.add(E, "POST_MISMATCH", f"{kind}: poza w pliku różni się od programu (Δ={dp:.4f} mm, {da:.4f}°)",
                    stage="post", instruction=idx, source_line=getattr(program.instructions[idx], "src_line", None))
    rep.add(INFO, "POST_VERIFIED", f"Zweryfikowano {len(parsed)} poz; maks. odchyłka {worst[0]:.4f} mm / "
            f"{worst[1]:.4f}°", stage="post")
    return rep
