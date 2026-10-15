"""Kalibracja baz dwóch robotów i układów odniesienia."""
from __future__ import annotations

import numpy as np

from ..core import pose as P


def calibrate_bases(points_in_A, points_in_B) -> np.ndarray:
    """Te same fizyczne punkty zmierzone w bazie A i w bazie B (≥3, niewspółliniowe).
    Zwraca T_A_B: poza bazy B w układzie bazy A (p_A = T_A_B @ p_B)."""
    pa, pb = np.asarray(points_in_A, float), np.asarray(points_in_B, float)
    if len(pa) < 3:
        raise ValueError("Potrzeba co najmniej 3 punktów")
    return P.fit_rigid(pb, pa)


def frame_from_3_points(origin, x_point, xy_point) -> np.ndarray:
    o, px, pxy = (np.asarray(v, float) for v in (origin, x_point, xy_point))
    x = px - o
    x /= np.linalg.norm(x)
    z = np.cross(x, pxy - o)
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, o
    return T


def calibrate_tcp(flange_poses: list[np.ndarray]) -> tuple[np.ndarray, float]:
    """TCP (xyz względem kołnierza) z N poz kołnierza dotykających tego samego punktu.
    Rozwiązuje R_i t + p_i = c (najmniejsze kwadraty). Zwraca (t, błąd RMS mm)."""
    A, b = [], []
    for T in flange_poses:
        A.append(np.hstack([T[:3, :3], -np.eye(3)]))
        b.append(-T[:3, 3])
    A, b = np.vstack(A), np.concatenate(b)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    rms = float(np.sqrt(np.mean((A @ x - b) ** 2)))
    return x[:3], rms
