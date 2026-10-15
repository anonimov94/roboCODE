"""Model kinematyczny 6R (DH zmodyfikowane = standard Craiga nie; tu klasyczne DH)."""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from itertools import product

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

    def supports_analytic_ik(self) -> bool:
        """Whether this DH chain matches the closed-form spherical-wrist solver."""
        if self.n != 6:
            return False

        def close(value: float, expected: float) -> bool:
            return abs(value - expected) < 1e-7

        a = [link.a for link in self.dh]
        alpha = [link.alpha for link in self.dh]
        d = [link.d for link in self.dh]
        return (
            len(self.joint_sign) == 6 and all(sign in (-1, 1) for sign in self.joint_sign)
            and abs(abs(alpha[0]) - 90.0) < 1e-7
            and close(alpha[1], 0.0)
            and close(a[3], 0.0) and close(a[4], 0.0) and close(d[4], 0.0)
            and close(a[5], 0.0) and close(alpha[5], 0.0)
            and abs(abs(alpha[3]) - 90.0) < 1e-7
            and abs(alpha[3] + alpha[4]) < 1e-7
            and abs(a[1]) > 1e-9
        )

    def ik_analytic(self, T_target: np.ndarray, include_turns: bool = True) -> list[np.ndarray]:
        """Return closed-form IK branches for supported 6R spherical-wrist DH models.

        The arm is solved as two planar links after the shoulder branch; the wrist
        is decomposed into Z-Y-Z rotations. Unsupported DH layouts raise ValueError.
        """
        if not self.supports_analytic_ik():
            raise ValueError("Analityczne IK obsługuje tylko model 6R z nadgarstkiem sferycznym i zgodną geometrią DH")
        target = np.asarray(T_target, dtype=float)
        if target.shape != (4, 4) or not np.all(np.isfinite(target)) or not P.is_valid_pose(target):
            return []

        target_flange = target @ P.inv(self.flange)
        p_wc = target_flange[:3, 3] - self.dh[5].d * target_flange[:3, 2]
        alpha1 = np.deg2rad(self.dh[0].alpha)
        alpha3 = np.deg2rad(self.dh[2].alpha)
        a2 = self.dh[1].a
        a3 = self.dh[2].a
        d4 = self.dh[3].d
        link3_length = float(np.hypot(a3, d4 * np.sin(alpha3)))
        if abs(a2) < 1e-9 or link3_length < 1e-9:
            return []
        link3_angle = float(np.arctan2(-d4 * np.sin(alpha3), a3))
        z_offset = self.dh[1].d + self.dh[2].d + d4 * np.cos(alpha3)
        position_scale = max(1.0, float(np.linalg.norm(p_wc)))
        tolerance = position_scale * 1e-9
        solutions: list[np.ndarray] = []

        def joint_values(theta_deg: float, index: int) -> list[float]:
            q0 = (theta_deg - self.dh[index].theta_offset) / self.joint_sign[index]
            lo, hi = self.limits_deg[index]
            first = int(np.ceil((lo - q0 - 1e-9) / 360.0))
            last = int(np.floor((hi - q0 + 1e-9) / 360.0))
            candidates = [q0 + 360.0 * turn for turn in range(first, last + 1)]
            if not include_turns and candidates:
                return [min(candidates, key=abs)]
            return candidates

        shoulder_a = np.sin(alpha1) * p_wc[0]
        shoulder_b = -np.sin(alpha1) * p_wc[1]
        shoulder_c = np.cos(alpha1) * (p_wc[2] - self.dh[0].d)
        shoulder_radius = float(np.hypot(shoulder_a, shoulder_b))
        if shoulder_radius < 1e-12:
            if abs(shoulder_c) > tolerance:
                return []
            shoulder_angles = [0.0]
        else:
            shoulder_value = -shoulder_c / shoulder_radius
            if abs(shoulder_value) > 1.0 + 1e-9:
                return []
            shoulder_value = float(np.clip(shoulder_value, -1.0, 1.0))
            shoulder_phase = float(np.arctan2(shoulder_b, shoulder_a))
            shoulder_asin = float(np.arcsin(shoulder_value))
            shoulder_angles = [
                shoulder_asin - shoulder_phase,
                np.pi - shoulder_asin - shoulder_phase,
            ]

        for theta1 in shoulder_angles:
            theta1_deg = np.rad2deg(theta1)
            q1_values = joint_values(theta1_deg, 0)
            if not q1_values:
                continue
            for q1 in q1_values:
                T01 = self.fk_all([q1, 0, 0, 0, 0, 0])[1]
                local = T01[:3, :3].T @ (p_wc - T01[:3, 3])
                if abs(local[2] - z_offset) > tolerance:
                    continue
                x, y = local[0], local[1]
                elbow_cos = (x * x + y * y - a2 * a2 - link3_length * link3_length) / (
                    2.0 * a2 * link3_length
                )
                if abs(elbow_cos) > 1.0 + 1e-9:
                    continue
                elbow_cos = float(np.clip(elbow_cos, -1.0, 1.0))
                position_angle = float(np.arctan2(y, x))
                elbow_angle = float(np.arccos(elbow_cos))
                for delta in (elbow_angle, -elbow_angle):
                    theta2 = position_angle - np.arctan2(
                        link3_length * np.sin(delta), a2 + link3_length * np.cos(delta)
                    )
                    theta3 = delta - link3_angle
                    q2_values = joint_values(np.rad2deg(theta2), 1)
                    q3_values = joint_values(np.rad2deg(theta3), 2)
                    if not q2_values or not q3_values:
                        continue
                    for q2, q3 in product(q2_values, q3_values):
                        arm = [q1, q2, q3, 0.0, 0.0, 0.0]
                        R03 = self.fk_all(arm)[3][:3, :3]
                        R36 = R03.T @ target_flange[:3, :3]
                        middle = float(np.arccos(np.clip(R36[2, 2], -1.0, 1.0)))
                        wrist_betas = (middle, -middle)
                        for beta in wrist_betas:
                            sin_beta = np.sin(beta)
                            if abs(sin_beta) < 1e-9:
                                theta4 = 0.0
                                Rz6 = np.array([
                                    [np.cos(beta), 0.0, -np.sin(beta)],
                                    [0.0, 1.0, 0.0],
                                    [np.sin(beta), 0.0, np.cos(beta)],
                                ]) @ R36
                                theta6 = float(np.arctan2(Rz6[1, 0], Rz6[0, 0]))
                            else:
                                direction = np.sign(sin_beta)
                                theta4 = float(np.arctan2(direction * R36[1, 2], direction * R36[0, 2]))
                                theta6 = float(np.arctan2(direction * R36[2, 1], -direction * R36[2, 0]))
                            wrist_sign = np.sign(np.sin(np.deg2rad(self.dh[3].alpha)))
                            theta5 = -beta / wrist_sign
                            wrist_values = [
                                joint_values(np.rad2deg(theta), index)
                                for theta, index in ((theta4, 3), (theta5, 4), (theta6, 5))
                            ]
                            if any(not values for values in wrist_values):
                                continue
                            for q4, q5, q6 in product(*wrist_values):
                                q = np.array([q1, q2, q3, q4, q5, q6], dtype=float)
                                if self.within_limits(q) and not any(np.allclose(q, old, atol=0.5) for old in solutions):
                                    solutions.append(q)
        return solutions

    def ik_all(self, T_target, seeds=None, include_turns: bool = True) -> list[np.ndarray]:
        """Return all IK solutions in limits, preferring analytic branches when available.

        For other DH layouts, retain the numeric seed search and wrist-flip fallback.
        """
        if self.supports_analytic_ik():
            return self.ik_analytic(T_target, include_turns=include_turns)
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
