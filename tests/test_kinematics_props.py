"""Property-style kinematics tests (random but reproducible; no extra dependencies). docs §6.3."""
import numpy as np
import pytest

from robocode.core import pose as P
from robocode.kinematics.configs import config_flags
from robocode.kinematics.robot_model import example_6r
from robocode.robots import available, load_robot

RNG = np.random.default_rng(42)
M = example_6r()


def random_q(n=40, margin=15.0):
    lo = np.array([a for a, _ in M.limits_deg]) + margin
    hi = np.array([b for _, b in M.limits_deg]) - margin
    qs = RNG.uniform(lo, hi, (n, 6))
    return [q for q in qs if abs(q[4]) > 10 and M.manipulability(q) > 1e-3]


def test_fk_ik_fk_roundtrip_random():
    """FK -> IK -> FK must recover the pose; joint values may differ (other configuration)."""
    ok = 0
    for q in random_q():
        T = M.fk(q)
        r = M.ik_solve(T, q + RNG.uniform(-8, 8, 6))
        assert r.ok, r.message
        dp, da = P.pose_distance(M.fk(r.q), T)
        assert dp < 1e-2 and da < 1e-2
        ok += 1
    assert ok >= 15


def test_ik_all_solutions_valid_unique_and_within_limits():
    q = np.array([20, 10, 30, 15, 50, -20.0])
    T = M.fk(q)
    sols = M.ik_all(T)
    assert len(sols) >= 2
    for i, s in enumerate(sols):
        assert M.within_limits(s)
        assert P.pose_distance(M.fk(s), T)[0] < 1e-2
        for t in sols[i + 1:]:
            assert not np.allclose(s, t, atol=0.5)


def test_analytic_ik_finds_eight_spherical_wrist_configurations():
    model = example_6r()
    model.limits_deg = [(-360.0, 360.0)] * 6
    q = np.array([20, 10, 30, 15, 50, -20.0])
    target = model.fk(q)

    assert model.supports_analytic_ik()
    solutions = model.ik_analytic(target, include_turns=False)
    assert len(solutions) == 8
    for i, solution in enumerate(solutions):
        assert model.within_limits(solution)
        assert P.pose_distance(model.fk(solution), target)[0] < 1e-5
        assert P.pose_distance(model.fk(solution), target)[1] < 1e-5
        for other in solutions[i + 1:]:
            assert not np.allclose(solution, other, atol=0.5)


def test_analytic_ik_configuration_flags_preserve_all_eight_branches():
    expected = {shoulder + elbow + wrist
                for shoulder in ("F", "R")
                for elbow in ("U", "D")
                for wrist in ("F", "N")}
    for signs, offsets in (([1] * 6, (0.0, 0.0)), ([-1, 1, -1, 1, -1, 1], (17.0, -23.0))):
        model = example_6r()
        model.limits_deg = [(-360.0, 360.0)] * 6
        model.joint_sign = signs
        model.dh[2].theta_offset, model.dh[4].theta_offset = offsets
        target = model.fk([20, 10, 30, 15, 50, -20])
        labels = {config_flags(model, q)["label"] for q in model.ik_analytic(target, include_turns=False)}

        assert labels == expected


def test_analytic_ik_roundtrips_random_reachable_poses():
    model = example_6r()
    for q in random_q(20):
        target = model.fk(q)
        solutions = model.ik_analytic(target)
        assert solutions
        assert all(P.pose_distance(model.fk(solution), target)[0] < 1e-5 for solution in solutions)
        assert all(P.pose_distance(model.fk(solution), target)[1] < 1e-5 for solution in solutions)


def test_analytic_ik_handles_singular_wrist_with_flange_offset():
    model = example_6r()
    model.limits_deg = [(-360.0, 360.0)] * 6
    model.flange = P.transl(12, -7, 35) @ P.rotx(13)
    for wrist_angle in (0.0, 180.0):
        target = model.fk([20, 10, 30, 15, wrist_angle, -20])
        solutions = model.ik_analytic(target, include_turns=False)
        assert solutions
        for solution in solutions:
            dp, da = P.pose_distance(model.fk(solution), target)
            assert dp < 1e-5 and da < 1e-5


def test_ik_unreachable_has_explicit_status():
    r = M.ik_solve(P.transl(5000, 0, 0), np.zeros(6))
    assert not r.ok and r.status == "no_convergence" and r.q is None


def test_ik_invalid_input():
    T = np.eye(4)
    T[0, 3] = np.nan
    assert M.ik_solve(T, np.zeros(6)).status == "invalid_input"


def test_ik_joint_limit_status():
    lim = list(M.limits_deg)
    q = np.array([10, 30, 20, 5, 40, -15.0])
    T = M.fk(q)
    m2 = example_6r()
    m2.limits_deg = lim[:5] + [(0, 350)]  # axis 6 must be >= 0; the pose needs -15
    r = m2.ik_solve(T, q)
    assert r.status == "joint_limits" and r.q is not None


def test_ik_continuity_small_steps():
    """Neighbouring targets seeded with the previous solution must not jump (no 360 deg flips)."""
    q = np.array([0, 30, 20, 0, 40, 0.0])
    T0 = M.fk(q)
    prev = q
    for k in range(1, 30):
        T = T0 @ P.transl(k * 2.0, 0, 0) @ P.rotz(k * 3.0)
        r = M.ik_solve(T, prev)
        assert r.ok
        assert np.abs(r.q - prev).max() < 15
        prev = r.q


def test_composition_and_inverse():
    for _ in range(20):
        A = P.from_xyzabc(*RNG.uniform(-500, 500, 3), *RNG.uniform(-180, 180, 3))
        B = P.from_xyzabc(*RNG.uniform(-500, 500, 3), *RNG.uniform(-180, 180, 3))
        assert np.allclose(A @ P.inv(A), np.eye(4), atol=1e-9)
        assert np.allclose(P.inv(A @ B), P.inv(B) @ P.inv(A), atol=1e-9)
        assert P.is_valid_pose(A @ B)


@pytest.mark.parametrize("conv", ["XYZ_static", "KUKA_ABC"])
def test_euler_roundtrip_including_gimbal_lock(conv):
    for b in (-90, -89.999, 0, 45, 89.999, 90):
        T = P.from_xyzabc(1, 2, 3, 30, b, -40, conv)
        T2 = P.from_xyzabc(*P.to_xyzabc(T, conv), conv)
        assert np.allclose(T, T2, atol=1e-6), (conv, b)


def test_quaternion_roundtrip():
    for _ in range(30):
        T = P.from_xyzabc(*RNG.uniform(-500, 500, 3), *RNG.uniform(-180, 180, 3))
        q = P.to_quaternion(T)
        assert np.allclose(P.from_quaternion(*T[:3, 3], *q), T, atol=1e-9)


def test_is_valid_pose_rejects_bad():
    T = np.eye(4)
    T[0, 0] = 2
    assert not P.is_valid_pose(T)
    T = np.eye(4)
    T[:3, :3] = np.diag([1, 1, -1])  # left-handed
    assert not P.is_valid_pose(T)


def test_robot_library():
    assert "generic6r_900" in available()
    m = load_robot("generic6r_900")
    assert m.license and m.source
    assert np.allclose(m.fk([1, 2, 3, 4, 5, 6]), M.fk([1, 2, 3, 4, 5, 6]))
    with pytest.raises(FileNotFoundError):
        load_robot("does_not_exist")


def test_ur5e_model_matches_ros_industrial_urdf_chain():
    model = load_robot("ur5e")
    assert "ur5e" in available()
    assert "ros-industrial/universal_robot" in model.source
    assert "BSD-3-Clause" in model.license
    assert model.limits_deg == [(-360.0, 360.0), (-360.0, 360.0), (-180.0, 180.0),
                                (-360.0, 360.0), (-360.0, 360.0), (-360.0, 360.0)]
    assert model.vmax_deg_s == [180.0] * 6

    urdf_origins = [
        ((0, 0, 162.5), (0, 0, 0)),
        ((0, 0, 0), (90, 0, 0)),
        ((-425, 0, 0), (0, 0, 0)),
        ((-392.2, 0, 133.3), (0, 0, 0)),
        ((0, -99.7, 0), (90, 0, 0)),
        ((0, 99.6, 0), (90, 180, 180)),
    ]
    for joints in ([0, 0, 0, 0, 0, 0], [10, 20, 30, 40, 50, 60],
                   [-45, 30, -60, 90, -30, 15]):
        urdf_pose = np.eye(4)
        for (xyz, rpy), angle in zip(urdf_origins, joints):
            urdf_pose = urdf_pose @ P.transl(*xyz) @ P.from_xyzabc(0, 0, 0, *rpy) @ P.rotz(angle)
        assert np.allclose(model.fk(joints), urdf_pose, atol=1e-8)
