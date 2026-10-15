import numpy as np

from robocode.demo import DEMO_NC, build_demo_station
from robocode.coop.calibration import calibrate_bases, calibrate_tcp, frame_from_3_points
from robocode.core import pose as P
from robocode.machining.nc_parser import parse_gcode
from robocode.machining.solver import MachiningSettings, solve
from robocode.machining.toolpath import Toolpath
from robocode.post.base import get_post
from robocode.program.simulator import simulate


def test_gcode_parser_basic_and_events():
    tp = parse_gcode(DEMO_NC)
    assert np.allclose(tp.points[0].p, [0, 0, 10])
    assert tp.points[0].rapid
    assert any("SetTool(1)" in pt.events for pt in tp.points)
    assert any("Spindle(12000)" in pt.events for pt in tp.points)
    arc = [pt.p for pt in tp.points if pt.p[1] > 9.9 and pt.p[0] < 15]  # G2 z lewej strony idzie górą (CW)
    assert arc, "łuk G2 powinien przejść przez Y≈10"


def test_gcode_ab_axes_tool_vector():
    tp = parse_gcode("G1 X0 Y0 Z0 B90 F100")
    assert np.allclose(tp.points[0].n, [1, 0, 0], atol=1e-9)


def test_csv_import():
    tp = Toolpath.from_csv("0,0,0\n10,0,0,0,1,1")
    assert len(tp) == 2 and np.isclose(np.linalg.norm(tp.points[1].n), 1)


def test_machining_solve_simulate_and_posts(tmp_path):
    st, r, fr, tool = build_demo_station()
    res = solve(r, r.frame_rel_base(fr), tool.tcp(), parse_gcode(DEMO_NC), MachiningSettings())
    assert res.ok, res.message
    # każdy punkt: TCP w układzie detalu zgodny ze ścieżką
    tp = parse_gcode(DEMO_NC).with_approach_retract(100)
    for pt, q in zip(tp.points, res.joints):
        tcp = P.inv(r.frame_rel_base(fr)) @ r.model.fk(q) @ tool.tcp()
        assert np.allclose(tcp[:3, 3], pt.p, atol=0.05)
        assert np.allclose(tcp[:3, 2], -pt.n, atol=1e-3)
    sim = simulate(res.program)
    assert sim.ok, sim.errors
    for post in ("kuka", "abb", "generic"):
        f = get_post(post).save(res.program, tmp_path)[0]
        txt = f.read_text()
        assert txt.count("\n") > 20
    krl = (tmp_path / "Machining.src").read_text()
    assert krl.startswith("DEF Machining") and "LIN {X 60.000" in krl and krl.strip().endswith("END")


def test_unreachable_reports_failure():
    st, r, fr, tool = build_demo_station()
    far = P.transl(5000, 0, 0)
    res = solve(r, far, tool.tcp(), parse_gcode("G1 X0 Y0 Z0 F100"), MachiningSettings(z_range_deg=0))
    assert not res.ok and "nieosiągalnych" in res.message


def test_calibrations():
    T = P.transl(1500, 200, 0) @ P.rotz(180)
    pb = np.array([[500, 0, 0], [600, 100, 0], [550, -80, 300], [400, 50, 100.0]])
    pa = (T[:3, :3] @ pb.T).T + T[:3, 3]
    assert np.allclose(calibrate_bases(pa, pb), T, atol=1e-9)
    F = frame_from_3_points([0, 0, 0], [10, 0, 0], [3, 5, 0])
    assert np.allclose(F, np.eye(4))
    tcp_true = np.array([5, -3, 120.0])
    c = np.array([600, 50, 200.0])
    poses = []
    for ang in [(0, 0), (20, 0), (0, 25), (-15, 10)]:
        R = P.rotx(ang[0]) @ P.roty(ang[1])
        R[:3, 3] = c - R[:3, :3] @ tcp_true
        poses.append(R)
    t, rms = calibrate_tcp(poses)
    assert np.allclose(t, tcp_true, atol=1e-6) and rms < 1e-6
