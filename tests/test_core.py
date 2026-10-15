import numpy as np
import pytest

from robocode.core import pose as P
from robocode.core import commands as C
from robocode.core.scene import Frame, Robot, Station, Tool
from robocode.core.io import save_station, load_station
from robocode.kinematics.robot_model import example_6r, RobotModel


def test_pose_roundtrip_conventions():
    for conv in ("XYZ_static", "KUKA_ABC"):
        T = P.from_xyzabc(10, 20, 30, 15, -25, 40, conv)
        assert np.allclose(P.from_xyzabc(*P.to_xyzabc(T, conv), conv), T)


def test_quaternion_identity_and_inverse():
    assert np.allclose(P.to_quaternion(P.eye()), [1, 0, 0, 0])
    T = P.transl(1, 2, 3) @ P.rotz(30) @ P.rotx(10)
    assert np.allclose(T @ P.inv(T), np.eye(4))


def test_fit_rigid_recovers_transform():
    T = P.transl(100, -50, 20) @ P.rotz(33) @ P.roty(12)
    pts = np.random.default_rng(1).uniform(-200, 200, (6, 3))
    moved = (T[:3, :3] @ pts.T).T + T[:3, 3]
    assert np.allclose(P.fit_rigid(pts, moved), T, atol=1e-9)


def test_fk_ik_consistency():
    m = example_6r()
    q = np.array([10, 30, 20, 5, 40, -15.0])
    T = m.fk(q)
    s = m.ik(T, q + 5)
    assert s is not None
    assert np.allclose(m.fk(s), T, atol=1e-2)


def test_robot_model_json_roundtrip():
    m = example_6r()
    m2 = RobotModel.from_json(m.to_json())
    q = [5, 20, 10, 0, 30, 0]
    assert np.allclose(m.fk(q), m2.fk(q))


def test_undo_redo_and_bundling():
    st = Station(name="S")
    stack = C.UndoStack()
    f = Frame(name="F")
    stack.push(C.add_item(st, f))
    assert f in st.children
    stack.undo()
    assert f not in st.children
    stack.redo()
    assert f in st.children
    r = Robot(name="R", model=example_6r())
    for v in range(5):  # jog = jedna akcja w historii
        stack.push(C.set_attr(r, "joints", np.full(6, v), merge_key="jog"))
    assert stack.history().count("Zmień joints") == 1
    stack.undo()
    assert np.allclose(r.joints, 0)


def test_set_parent_keeps_absolute():
    st = Station(name="S")
    a = st.add(Frame(name="A", pose=P.transl(100, 0, 0)))
    b = st.add(Frame(name="B", pose=P.transl(0, 50, 0) @ P.rotz(20)))
    before = b.pose_abs()
    b.set_parent(a)
    assert np.allclose(b.pose_abs(), before)


def test_station_save_load(tmp_path):
    st = Station(name="S")
    r = st.add(Robot(name="R", model=example_6r(), joints=np.array([1, 2, 3, 4, 5, 6.0])))
    t = r.add(Tool(name="T", pose=P.transl(0, 0, 100)))
    r.active_tool = t
    f = st.add(Frame(name="F", pose=P.transl(500, 0, 0)))
    r.active_frame = f
    path = tmp_path / "s.rcs.json"
    save_station(st, path)
    st2 = load_station(path)
    r2 = st2.find("R")
    assert np.allclose(r2.joints, r.joints)
    assert r2.active_tool.name == "T" and r2.active_frame.name == "F"
    assert np.allclose(r2.tcp_abs(), r.tcp_abs())
