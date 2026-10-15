"""Kalibracja baz dwóch robotów i układów odniesienia."""
from __future__ import annotations

import numpy as np

from ..core import pose as P


def calibrate_bases(points_in_A, points_in_B) -> np.ndarray:
    """Te same fizyczne punkty zmierzone w bazie A i w bazie B (≥3, niewspółliniowe).
    Zwraca T_A_B: poza bazy B w układzie bazy A (p_A = T_A_B @ p_B)."""
    pa, pb = np.asarray(points_in_A, float), np.asarray(points_in_B, float)
    if pa.ndim != 2 or pb.ndim != 2 or pa.shape != pb.shape or pa.shape[1:] != (3,):
        raise ValueError("Obie listy punktów muszą mieć zgodny kształt N×3")
    if len(pa) < 3:
        raise ValueError("Potrzeba co najmniej 3 punktów")
    if not np.all(np.isfinite(pa)) or not np.all(np.isfinite(pb)):
        raise ValueError("Punkty muszą być skończone")
    if np.linalg.matrix_rank(pa - pa.mean(axis=0), tol=1e-9) < 2 \
            or np.linalg.matrix_rank(pb - pb.mean(axis=0), tol=1e-9) < 2:
        raise ValueError("Punkty muszą zawierać co najmniej trzy punkty niewspółliniowe")
    return P.fit_rigid(pb, pa)


def frame_from_3_points(origin, x_point, xy_point) -> np.ndarray:
    o, px, pxy = (np.asarray(v, float) for v in (origin, x_point, xy_point))
    if any(v.shape != (3,) or not np.all(np.isfinite(v)) for v in (o, px, pxy)):
        raise ValueError("Każdy punkt musi zawierać dokładnie 3 współrzędne skończone")
    x = px - o
    x_length = np.linalg.norm(x)
    if x_length < 1e-9:
        raise ValueError("Punkt na osi X nie może pokrywać się z początkiem układu")
    x /= x_length
    z = np.cross(x, pxy - o)
    z_length = np.linalg.norm(z)
    if z_length < 1e-9:
        raise ValueError("Punkt XY nie może leżeć na osi X")
    z /= z_length
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, o
    return T


def calibrate_tcp(flange_poses: list[np.ndarray]) -> tuple[np.ndarray, float]:
    """TCP (xyz względem kołnierza) z N poz kołnierza dotykających tego samego punktu.
    Rozwiązuje R_i t + p_i = c (najmniejsze kwadraty). Zwraca (t, błąd RMS mm)."""
    if len(flange_poses) < 3:
        raise ValueError("Kalibracja TCP wymaga co najmniej 3 póz kołnierza")
    A, b = [], []
    for pose in flange_poses:
        T = np.asarray(pose, dtype=float)
        if not P.is_valid_pose(T):
            raise ValueError("Każda poza kołnierza musi być poprawną macierzą 4×4")
        A.append(np.hstack([T[:3, :3], -np.eye(3)]))
        b.append(-T[:3, 3])
    A, b = np.vstack(A), np.concatenate(b)
    if np.linalg.matrix_rank(A) < 6:
        raise ValueError("Pózy nie zawierają wystarczającej różnorodności obrotów do wyznaczenia TCP")
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    rms = float(np.sqrt(np.mean((A @ x - b) ** 2)))
    return x[:3], rms
