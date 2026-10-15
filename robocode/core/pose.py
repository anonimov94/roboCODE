"""Pozy 4x4 i konwencje orientacji. API w mm i stopniach."""
from __future__ import annotations

import numpy as np

D2R = np.pi / 180.0


def eye() -> np.ndarray:
    return np.eye(4)


def transl(x: float = 0, y: float = 0, z: float = 0) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def _rot(axis: str, deg: float) -> np.ndarray:
    a = deg * D2R
    c, s = np.cos(a), np.sin(a)
    T = np.eye(4)
    if axis == "x":
        T[1:3, 1:3] = [[c, -s], [s, c]]
    elif axis == "y":
        T[0, 0], T[0, 2], T[2, 0], T[2, 2] = c, s, -s, c
    else:
        T[0:2, 0:2] = [[c, -s], [s, c]]
    return T


def rotx(deg: float) -> np.ndarray:
    return _rot("x", deg)


def roty(deg: float) -> np.ndarray:
    return _rot("y", deg)


def rotz(deg: float) -> np.ndarray:
    return _rot("z", deg)


def inv(T: np.ndarray) -> np.ndarray:
    R, p = T[:3, :3], T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ p
    return Ti


# --- konwencje Eulera ------------------------------------------------------
# "XYZ_static": obrót wokół X, potem stałej Y, potem stałej Z  (Fanuc/Motoman)  -> R = Rz*Ry*Rx
# "ZYX_moving": Z, Y', X''  (KUKA A,B,C)                                         -> R = Rz*Ry*Rx (te same macierze, inna kolejność parametrów)

def from_xyzabc(x, y, z, a, b, c, convention: str = "XYZ_static") -> np.ndarray:
    """XYZ_static: a=rx, b=ry, c=rz. KUKA_ABC: a=rz, b=ry, c=rx."""
    if convention == "XYZ_static":
        R = rotz(c) @ roty(b) @ rotx(a)
    elif convention == "KUKA_ABC":
        R = rotz(a) @ roty(b) @ rotx(c)
    else:
        raise ValueError(f"Nieznana konwencja: {convention}")
    R[:3, 3] = [x, y, z]
    return R


def _zyx_angles(R: np.ndarray) -> tuple[float, float, float]:
    """Zwraca (rz, ry, rx) w stopniach dla R = Rz*Ry*Rx."""
    sy = -R[2, 0]
    sy = np.clip(sy, -1.0, 1.0)
    ry = np.arcsin(sy)
    if abs(np.cos(ry)) > 1e-9:
        rx = np.arctan2(R[2, 1], R[2, 2])
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:  # gimbal lock
        rx = 0.0
        rz = np.arctan2(-R[0, 1], R[1, 1])
    return rz / D2R, ry / D2R, rx / D2R


def to_xyzabc(T: np.ndarray, convention: str = "XYZ_static") -> list[float]:
    x, y, z = T[:3, 3]
    rz, ry, rx = _zyx_angles(T)
    if convention == "XYZ_static":
        return [x, y, z, rx, ry, rz]
    if convention == "KUKA_ABC":
        return [x, y, z, rz, ry, rx]
    raise ValueError(convention)


def to_quaternion(T: np.ndarray) -> list[float]:
    """[q1(w), q2, q3, q4] – konwencja ABB."""
    R = T[:3, :3]
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    if q[0] < 0:
        q = -q
    return list(q / np.linalg.norm(q))


def pose_from_point_normal(p, n, xdir=None) -> np.ndarray:
    """Poza z osią Z narzędzia = -n? Nie: Z = n (konwencja: oś Z narzędzia wzdłuż wektora narzędzia)."""
    z = np.asarray(n, float)
    z = z / np.linalg.norm(z)
    ref = np.array([1.0, 0, 0]) if xdir is None else np.asarray(xdir, float)
    if abs(np.dot(ref, z)) > 0.99:
        ref = np.array([0, 1.0, 0])
    x = ref - np.dot(ref, z) * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, p
    return T


def pose_distance(A: np.ndarray, B: np.ndarray) -> tuple[float, float]:
    """(odległość mm, kąt deg) między pozami."""
    dp = float(np.linalg.norm(A[:3, 3] - B[:3, 3]))
    Rd = A[:3, :3].T @ B[:3, :3]
    ang = np.arccos(np.clip((np.trace(Rd) - 1) / 2, -1, 1)) / D2R
    return dp, float(ang)


def fit_rigid(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Kabsch: T taka, że dst ≈ T*src (punkty Nx3)."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = cd - R @ cs
    return T
