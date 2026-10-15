"""Frezowanie kooperatywne: robot A trzyma detal, robot B trzyma wrzeciono.

Model: T_A_B = poza bazy B w bazie A (z kalibracji).
Dla punktu ścieżki (poza narzędzia względem detalu T_part_tool) i orientacji detalu
(poza detalu względem bazy A, T_A_part):
    flansza A:  T_A_part @ inv(grip)                         (grip = detal względem kołnierza A)
    TCP B (w bazie B): inv(T_A_B) @ T_A_part @ T_part_tool
Strategia v0.1: detal ustawiany w "pozycji roboczej" (stała poza) + obrót wokół osi narzędzia
przeszukiwany tak, by OBA roboty miały rozwiązanie i dobrą manipulowalność.
TODO M7: optymalizacja ciągła pozycji detalu (A rusza się razem z B), synchronizacja programów.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..core import pose as P
from ..machining.toolpath import Toolpath


@dataclass
class DualArmSetup:
    robot_a: object  # trzyma detal
    robot_b: object  # trzyma wrzeciono
    T_A_B: np.ndarray
    grip: np.ndarray = field(default_factory=P.eye)       # detal względem kołnierza A
    tool_b: np.ndarray = field(default_factory=P.eye)     # TCP wrzeciona względem kołnierza B
    work_pose_A: np.ndarray = field(default_factory=lambda: P.transl(600, 0, 500))  # domyślna poza detalu


@dataclass
class DualArmResult:
    ok: bool
    joints_a: list[np.ndarray]
    joints_b: list[np.ndarray]
    message: str


def solve_dual(setup: DualArmSetup, toolpath: Toolpath, part_rot_candidates_deg=(0, 30, -30, 60, -60, 90, -90),
               z_candidates_deg=(0, 45, -45, 90, -90, 135, -135, 180)) -> DualArmResult:
    A, B = setup.robot_a.model, setup.robot_b.model
    qa_prev = setup.robot_a.joints.copy()
    qb_prev = setup.robot_b.joints.copy()
    ja, jb = [], []
    inv_AB = P.inv(setup.T_A_B)
    inv_tool = P.inv(setup.tool_b)
    inv_grip = P.inv(setup.grip)
    for i, pt in enumerate(toolpath.points):
        best = None
        for rp in part_rot_candidates_deg:
            T_A_part = setup.work_pose_A @ P.rotz(rp)
            qa = A.ik(T_A_part @ inv_grip, qa_prev, iters=80)
            if qa is None:
                continue
            for z in z_candidates_deg:
                T_part_tool = P.pose_from_point_normal(pt.p, -pt.n) @ P.rotz(z)
                Tb = inv_AB @ T_A_part @ T_part_tool @ inv_tool
                qb = B.ik(Tb, qb_prev, iters=80)
                if qb is None:
                    continue
                cost = (np.abs(qa - qa_prev).sum() + np.abs(qb - qb_prev).sum()
                        + 50 / (B.manipulability(qb) + 1e-3))
                if best is None or cost < best[0]:
                    best = (cost, qa, qb)
        if best is None:
            return DualArmResult(False, ja, jb, f"✗ Punkt {i} nieosiągalny dla pary robotów")
        _, qa_prev, qb_prev = best
        ja.append(qa_prev)
        jb.append(qb_prev)
    return DualArmResult(True, ja, jb, f"✓ {len(ja)} punktów, oba roboty osiągają ścieżkę")
