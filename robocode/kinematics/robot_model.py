"""Model kinematyczny 6R (DH zmodyfikowane = standard Craiga nie; tu klasyczne DH)."""
from __future__ import annotations

import json
from dataclasses import dataclass, field

import numpy as np

from ..core import pose as P

D2R = np.pi / 180


@dataclass
class DH:
    a: float
    alpha: float  # deg
    d: float
    theta_offset: float = 0.0  # deg


@dataclass
class IKResult:
    q: np.ndarray | None
    status: str  # "ok" | "no_convergence" | "joint_limits" | "invalid_input"
    message: str

    @property
    def ok(self) -> bool:
        return self.status == "ok"


@dataclass
class RobotModel:
    name: str
    dh: list[DH]
    limits_deg: list[tuple[float, float]]
    joint_sign: list[int] = field(default_factory=lambda: [1] * 6)  # zgodność z kontrolerem
    flange: np.ndarray = field(default_factory=P.eye)
    vmax_deg_s: list[float] = field(default_factory=lambda: [150] * 6)
    source: str = "example"
    license: str = "CC0"

    @property
    def n(self) -> int:
        return len(self.dh)

    # ---------------- FK ----------------
    def fk_all(self, q_deg) -> list[np.ndarray]:
        q = np.asarray(q_deg, float) * np.asarray(self.joint_sign)
        T = np.eye(4)
        frames = [T.copy()]
        for i, p in enumerate(self.dh):
            th = (q[i] + p.theta_offset) * D2R
            al = p.alpha * D2R
            ct, st, ca, sa = np.cos(th), np.sin(th), np.cos(al), np.sin(al)
            A = np.array([[ct, -st * ca, st * sa, p.a * ct],
                          [st, ct * ca, -ct * sa, p.a * st],
                          [0, sa, ca, p.d],
                          [0, 0, 0, 1]])
            T = T @ A
            frames.append(T.copy())
        frames[-1] = frames[-1] @ self.flange
        return frames

    def fk(self, q_deg) -> np.ndarray:
        return self.fk_all(q_deg)[-1]

    # ---------------- Jakobian / IK ----------------
    def jacobian(self, q_deg) -> np.ndarray:
        fr = self.fk_all(q_deg)
        pe = fr[-1][:3, 3]
        J = np.zeros((6, self.n))
        for i in range(self.n):
            z = fr[i][:3, 2] * self.joint_sign[i]
            o = fr[i][:3, 3]
            J[:3, i] = np.cross(z, pe - o)
            J[3:, i] = z
        return J * D2R  # na stopień

    def manipulability(self, q_deg) -> float:
        J = self.jacobian(q_deg) / D2R
        J[:3] /= 1000.0  # mm -> m, wyrównanie skali
        return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))

    def within_limits(self, q_deg, margin: float = 0.0) -> bool:
        return all(lo + margin <= v <= hi - margin for v, (lo, hi) in zip(q_deg, self.limits_deg))

    def ik(self, T_target: np.ndarray, q_seed, tol_mm=1e-3, tol_deg=1e-3, iters=200):
        """IK numeryczne (Levenberg-Marquardt). Zwraca q lub None."""
        q = np.array(q_seed, float)
        lam = 0.5
        for _ in range(iters):
            T = self.fk(q)
            dp = T_target[:3, 3] - T[:3, 3]
            Re = T_target[:3, :3] @ T[:3, :3].T
            ang = np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))
            if ang < 1e-9:
                w = np.zeros(3)
            else:
                w = ang / (2 * np.sin(ang)) * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
            if np.linalg.norm(dp) < tol_mm and np.degrees(np.linalg.norm(w)) < tol_deg:
                return q if self.within_limits(q) else None
            e = np.concatenate([dp, w * 500.0])  # waga orientacji (mm/rad)
            J = self.jacobian(q)
            J[3:] *= 500.0
            dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), e)
            step = np.clip(dq, -20, 20)
            q = q + step
        return None

    def ik_solve(self, T_target: np.ndarray, q_seed, **kw) -> "IKResult":
        """IK with an explicit status instead of a bare None (docs §6.3)."""
        if not (np.asarray(T_target).shape == (4, 4) and np.all(np.isfinite(T_target))):
            return IKResult(None, "invalid_input", "Poza celu zawiera NaN/inf")
        q = self.ik_unbounded(T_target, q_seed, **kw)
        if q is None:
            reach = float(np.linalg.norm(T_target[:3, 3]))
            return IKResult(None, "no_convergence",
                            f"Brak rozwiązania IK (odległość od bazy {reach:.0f} mm) – cel poza zasięgiem "
                            "lub w osobliwości")
        if not self.within_limits(q):
            bad = [j + 1 for j, (v, (lo, hi)) in enumerate(zip(q, self.limits_deg)) if not lo <= v <= hi]
            return IKResult(q, "joint_limits", f"Rozwiązanie istnieje, ale osie {bad} poza limitami")
        return IKResult(q, "ok", "")

    def ik_unbounded(self, T_target, q_seed, tol_mm=1e-3, tol_deg=1e-3, iters=200):
        """Numeric IK ignoring joint limits (for diagnostics)."""
        lim = self.limits_deg
        self.limits_deg = [(-1e9, 1e9)] * self.n
        try:
            return self.ik(T_target, q_seed, tol_mm, tol_deg, iters)
        finally:
            self.limits_deg = lim

    def ik_all(self, T_target, seeds=None, include_turns: bool = True) -> list[np.ndarray]:
        """All IK solutions found (approximation of the configuration list, docs §6.3).

        1. numeric IK from a grid of seeds (shoulder/elbow/wrist combinations),
        2. for every solution also try the wrist flip (q4±180, -q5, q6±180) – exact for spherical wrists,
        3. optionally add ±360° variants of joints whose range exceeds 360° (axis-6 "winding").
        Solutions are unique (0.5° tolerance) and within joint limits.
        """
        if seeds is None:
            seeds = [[a1, a2, a3, a4, a5, 0]
                     for a1 in (0, 90, -90, 180, -180) for a2 in (-60, 0, 60) for a3 in (60, -60, 120)
                     for a4 in (0, 90, -90) for a5 in (45, -45)]
        sols: list[np.ndarray] = []

        def add(q):
            if q is not None and self.within_limits(q) and not any(np.allclose(q, x, atol=0.5) for x in sols):
                sols.append(np.asarray(q, float))
                return True
            return False

        for sd in seeds:
            add(self.ik(T_target, sd, iters=150))
        for q in list(sols):
            for s4 in (180.0, -180.0):
                for s6 in (180.0, -180.0):
                    flip = q.copy()
                    flip[3] += s4
                    flip[4] = -flip[4]
                    flip[5] += s6
                    if self.within_limits(flip):
                        add(self.ik(T_target, flip, iters=50))
        if include_turns:
            for q in list(sols):
                for j, (lo, hi) in enumerate(self.limits_deg):
                    if hi - lo > 360:
                        for turn in (360.0, -360.0):
                            v = q.copy()
                            v[j] += turn
                            add(v)
        return sols

    # ---------------- I/O ----------------
    def to_json(self) -> str:
        return json.dumps({
            "name": self.name, "source": self.source, "license": self.license,
            "dh": [vars(d) for d in self.dh], "limits_deg": self.limits_deg,
            "joint_sign": self.joint_sign, "vmax_deg_s": self.vmax_deg_s,
        }, indent=2)

    @classmethod
    def from_json(cls, s: str) -> "RobotModel":
        d = json.loads(s)
        return cls(name=d["name"], dh=[DH(**x) for x in d["dh"]],
                   limits_deg=[tuple(x) for x in d["limits_deg"]],
                   joint_sign=d.get("joint_sign", [1] * 6),
                   vmax_deg_s=d.get("vmax_deg_s", [150] * 6),
                   source=d.get("source", ""), license=d.get("license", ""))


def example_6r(name: str = "Generic6R-900") -> RobotModel:
    """Przykładowy robot 6R ~900 mm zasięgu. Parametry WYMYŚLONE do testów – nie jest to model żadnego produktu."""
    return RobotModel(
        name=name,
        dh=[DH(25, -90, 400), DH(455, 0, 0, -90), DH(35, -90, 0),
            DH(0, 90, 420), DH(0, -90, 0), DH(0, 0, 80)],
        limits_deg=[(-170, 170), (-100, 135), (-150, 150), (-185, 185), (-125, 125), (-350, 350)],
        source="roboCODE example (synthetic)", license="CC0",
    )
