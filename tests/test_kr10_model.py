"""KUKA KR 10 R1100 sixx model must match the URDF used by the ROS 2 / Gazebo cell (ros2/)."""
import numpy as np

from robocode.core import pose as P
from robocode.robots import available, load_robot

# joint origins [mm] and axes copied from ros2/robocode_kr10_description/urdf/kr10_arm.xacro
URDF_JOINTS = [
    ((0, 0, 400), (0, 0, -1)),
    ((25, 0, 0), (0, 1, 0)),
    ((560, 0, 0), (0, 1, 0)),
    ((0, 0, 35), (-1, 0, 0)),
    ((515, 0, 0), (0, 1, 0)),
    ((80, 0, 0), (-1, 0, 0)),
]


def _axis_rot(axis, deg):
    k = np.asarray(axis, float)
    th = np.radians(deg)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
    return T


def urdf_tool0(q_deg):
    T = np.eye(4)
    for (xyz, axis), angle in zip(URDF_JOINTS, q_deg):
        T = T @ P.transl(*xyz) @ _axis_rot(axis, angle)
    return T @ _axis_rot((0, 1, 0), 90)  # tool0: Z out of the flange


def test_kr10_in_library_with_source_and_license():
    assert "kuka_kr10_r1100_sixx" in available()
    m = load_robot("kuka_kr10_r1100_sixx")
    assert "ClaBirk/ROS2_Gazebo_Moveit_Simulation" in m.source
    assert "GPL-3.0" in m.license
    assert m.vmax_deg_s == [300.0, 225.0, 225.0, 381.0, 311.0, 492.0]


def test_kr10_fk_matches_urdf_joint_values():
    m = load_robot("kuka_kr10_r1100_sixx")
    rng = np.random.default_rng(7)
    samples = [np.zeros(6), np.array([10, -90, 90, 0, 45, 0])] + [rng.uniform(-120, 120, 6) for _ in range(20)]
    for q in samples:
        assert np.allclose(m.fk(q), urdf_tool0(q), atol=1e-8), q


def test_kr10_analytic_ik_roundtrip():
    m = load_robot("kuka_kr10_r1100_sixx")
    assert m.supports_analytic_ik()
    for q in ([0, -90, 90, 0, 30, 0], [20, -60, 70, 30, -40, 10], [-45, -100, 110, -60, 60, 90]):
        target = m.fk(q)
        sols = m.ik_analytic(target, include_turns=False)
        assert sols, q
        assert all(np.allclose(m.fk(s), target, atol=1e-6) for s in sols)
        assert any(np.allclose(s, q, atol=1e-6) for s in sols)
