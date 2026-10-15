"""Regression tests for defects reported by the external review (Gemini, ChatGPT, Grok) – see docs §13.2."""
import json

import numpy as np
import pytest

from robocode.core import pose as P
from robocode.demo import DEMO_NC, build_demo_station
from robocode.machining.nc_parser import parse_gcode
from robocode.machining.solver import MachiningSettings, solve
from robocode.machining.toolpath import PathPoint, Toolpath
from robocode.pipeline import run_job
from robocode.post.abb_rapid import _conf, _zone
from robocode.post.base import get_post
from robocode.post.tool_load import ToolLoad
from robocode.program.instructions import MoveC, MoveJ, MoveL, SetSpeed
from robocode.program.simulator import SimTolerances, _circle_through, _trapezoidal_time, simulate
from robocode.program.validator import validate_program, verify_post_output


def demo_program():
    st, r, fr, tool = build_demo_station()
    res = solve(r, r.frame_rel_base(fr), tool.tcp(), parse_gcode(DEMO_NC), MachiningSettings())
    assert res.ok
    return res.program


def test_validator_reports_cartesian_contour_deviation_in_millimeters():
    report, sim = validate_program(demo_program())

    assert report.ok
    assert sim is not None and sim.contour_samples > 0
    assert 0 <= sim.mean_contour_deviation_mm <= sim.max_contour_deviation_mm < 0.01
    metric = next(issue for issue in report.issues if issue.code == "CONTOUR_DEVIATION")
    assert "mm" in metric.message
    assert f"{sim.max_contour_deviation_mm:.4f} mm" in metric.message
    assert f"{sim.contour_samples} próbek MoveL/MoveC" in metric.message
    assert "CONTOUR_DEVIATION" in report.summary()


def codes(rep, sev="errors"):
    return {i.code for i in getattr(rep, sev)}


# ---- Gemini (parser / toolpath) ----
def test_densify_handles_opposite_tool_vectors():
    a = PathPoint(np.zeros(3), np.array([0, 0, 1.0]))
    b = PathPoint(np.array([10.0, 0, 0]), np.array([0, 0, -1.0]))
    tp = Toolpath([a, b]).densify_orientation(5.0)
    assert len(tp) == 37
    for p, q in zip(tp.points, tp.points[1:]):
        assert np.degrees(np.arccos(np.clip(p.n @ q.n, -1, 1))) <= 5.0 + 1e-6
        assert np.all(np.isfinite(q.n)) and np.isclose(np.linalg.norm(q.n), 1)


# ---- ChatGPT (simulator / validator) ----
def test_invalid_tolerances_rejected():
    with pytest.raises(ValueError):
        SimTolerances(step_mm=0)
    with pytest.raises(ValueError):
        SimTolerances(step_ori_deg=float("nan"))


def test_joint_speed_range_checked():
    prog = demo_program()
    prog.instructions.insert(3, SetSpeed(joint_deg_s=-50))
    rep, _ = validate_program(prog)
    assert "JOINT_SPEED_RANGE" in codes(rep)


@pytest.mark.parametrize("value", [0, -1, float("nan")])
def test_invalid_acceleration_blocks_validation(value):
    prog = demo_program()
    prog.instructions.insert(3, SetSpeed(linear_acc=value))

    report, _ = validate_program(prog)

    assert "ACCELERATION_RANGE" in codes(report)


def test_trapezoidal_time_handles_triangular_and_speed_limited_profiles():
    assert np.isclose(_trapezoidal_time(25, 10, 2), 2 * np.sqrt(12.5))
    assert np.isclose(_trapezoidal_time(100, 10, 2), 15)


def test_cycle_time_uses_programmed_acceleration_and_reports_defaults():
    prog = demo_program()
    default_sim = simulate(prog, restore_joints=True)
    first_move = next(i for i, ins in enumerate(prog.instructions) if isinstance(ins, (MoveJ, MoveL)))
    prog.instructions.insert(first_move, SetSpeed(linear_acc=100_000, joint_acc=100_000))
    accelerated_sim = simulate(prog, restore_joints=True)

    assert accelerated_sim.cycle_time_s < default_sim.cycle_time_s
    assert "ACCELERATION_ASSUMED" in codes(default_sim.report, "issues")
    assert "ACCELERATION_ASSUMED" not in codes(accelerated_sim.report, "issues")


def test_movej_cycle_time_uses_joint_acceleration():
    prog = demo_program()
    frame = next(ins for ins in prog.instructions if type(ins).__name__ == "SetFrame")
    tool = next(ins for ins in prog.instructions if type(ins).__name__ == "SetTool")
    start = prog.robot.joints.copy()
    target = start.copy()
    target[0] += 10
    prog.instructions = [frame, tool, SetSpeed(joint_deg_s=10, joint_acc=20), MoveJ(joints=target)]

    sim = simulate(prog, restore_joints=True)
    expected = _trapezoidal_time(10, min(10, prog.robot.model.vmax_deg_s[0]), 20)

    assert sim.ok, sim.errors
    assert np.isclose(sim.cycle_time_s, expected)
    assert "ACCELERATION_ASSUMED" not in codes(sim.report, "issues")


def test_report_always_states_collisions_not_checked():
    rep, _ = validate_program(demo_program())
    assert rep.ok and "COLLISION_NOT_CHECKED" in codes(rep, "warnings")


def test_circle_through_three_points():
    c, e1, e2, r, total = _circle_through(np.array([10.0, 0, 0]), np.array([0, 10.0, 0]), np.array([-10.0, 0, 0]))
    assert np.allclose(c, 0) and np.isclose(r, 10) and np.isclose(total, np.pi)
    _, _, _, _, t2 = _circle_through(np.array([10.0, 0, 0]), np.array([0, -10.0, 0]), np.array([-10.0, 0, 0]))
    assert np.isclose(t2, np.pi)  # the other half, through M
    assert _circle_through(np.zeros(3), np.array([1.0, 0, 0]), np.array([2.0, 0, 0])) is None


def test_movec_is_simulated_as_real_arc():
    """The arc passes through the auxiliary point; two straight lines would not sample its far side."""
    prog = demo_program()
    m = [i for i in prog.instructions if isinstance(i, MoveL)]
    start = m[2].pose  # (0,0,-2) in the part frame
    mid = start @ P.transl(10, 10, 0)
    end = start @ P.transl(20, 0, 0)
    idx = next(k for k, x in enumerate(prog.instructions) if x is m[2]) + 1
    prog.instructions.insert(idx, MoveC(pose_mid=mid, pose_end=end))
    del prog.instructions[idx + 1:]
    sim = simulate(prog, restore_joints=True)
    assert sim.ok, sim.errors
    frame = next(i for i in prog.instructions if type(i).__name__ == "SetFrame").pose
    st, r, fr, tool = build_demo_station()
    centre = (r.pose_abs() @ frame @ start @ P.transl(10, 0, 0))[:3, 3]  # trace is absolute
    tail = np.array(sim.trace[-5:])  # only samples of the arc itself
    assert np.allclose(np.linalg.norm(tail - centre, axis=1), 10.0, atol=0.05)


def test_movec_degenerate_blocks():
    prog = demo_program()
    m = [i for i in prog.instructions if isinstance(i, MoveL)]
    start = m[2].pose
    idx = next(k for k, x in enumerate(prog.instructions) if x is m[2]) + 1
    prog.instructions.insert(idx, MoveC(pose_mid=start @ P.transl(5, 0, 0), pose_end=start @ P.transl(10, 0, 0)))
    rep, _ = validate_program(prog)
    assert "MOVEC_DEGENERATE" in codes(rep)


def test_post_joint_read_back_catches_corruption():
    prog = demo_program()
    for post, old, new in (("kuka", "PTP {A1 -", "PTP {A1 -1"), ("abb", "MoveAbsJ [[-", "MoveAbsJ [[-1")):
        text = get_post(post).generate(prog)
        assert verify_post_output(post, text, prog).ok
        assert "POST_JOINT_MISMATCH" in codes(verify_post_output(post, text.replace(old, new, 1), prog))


def test_end_fallback_checks_orientation():
    tol = SimTolerances()
    assert tol.end_ori_tol_deg <= 1e-3 and tol.end_pos_tol_mm <= 1e-3


# ---- Grok (controller code) ----
def test_abb_zones_valid():
    assert _zone(-1) == "fine" and _zone(1) == "z1" and _zone(10) == "z10"
    assert _zone(2).startswith("[FALSE,2,")  # z2 does not exist in RAPID -> inline zonedata
    assert _zone(1.5).startswith("[FALSE,1.5,")


def test_abb_confdata_from_joints():
    assert _conf([-3, 30, 20, 0, 40, -3]) == "[-1,0,-1,0]"
    assert _conf([100, 0, 0, 185, 0, 270]) == "[1,2,3,0]"
    text = get_post("abb").generate(demo_program())
    assert "[0.000000,1.000000,0.000000,0.000000],[-1,0,-1,0],[9E9" in text  # demo joints q1, q6 < 0


def test_kuka_header_and_ptp():
    text = get_post("kuka").generate(demo_program())
    head = text.split("\nPTP", 1)[0]
    assert "BAS (#VEL_PTP,20)" in head and "$ORI_TYPE = #VAR" in head and "$CIRC_TYPE = #BASE" in head
    assert "C_PTP" not in text  # exact positioning for joint moves


def test_kuka_joint_speed_is_percent():
    prog = demo_program()
    prog.instructions.insert(3, SetSpeed(joint_deg_s=75))
    text = get_post("kuka").generate(prog)
    assert "BAS (#VEL_PTP,50) ; 75 deg/s" in text  # 75 / 150 deg/s


def test_event_map(tmp_path):
    em = {"SpindleOn": "$OUT[1] = TRUE", "SpindleOff": "$OUT[1] = FALSE", "Spindle": "$ANOUT[1] = {0}/24000.0",
          "SetTool": "; tool {0} already mounted"}
    job = run_job(DEMO_NC, out_dir=tmp_path, event_map=em)
    assert job.ok, job.report.summary()
    text = (tmp_path / "Machining.src").read_text()
    assert "$OUT[1] = TRUE" in text and "$ANOUT[1] = 12000/24000.0" in text and "SpindleOn()" not in text
    assert "CONTROLLER_SUBPROGRAMS" not in codes(job.report, "warnings")
    assert json.loads(job.manifest_path.read_text(encoding="utf-8"))["event_map"] == em


def test_cli_events_file(tmp_path):
    from robocode import cli
    ev = tmp_path / "ev.json"
    ev.write_text(json.dumps({"SpindleOn": "SetDO doSpindle,1;"}), encoding="utf-8")
    nc = tmp_path / "a.nc"
    nc.write_text(DEMO_NC, encoding="utf-8")
    assert cli.main(["machine", str(nc), "--post", "abb", "--events", str(ev), "--out", str(tmp_path / "o")]) == 0
    assert "SetDO doSpindle,1;" in (tmp_path / "o" / "Machining.mod").read_text()
    bad = tmp_path / "bad.json"
    bad.write_text("[1,2]", encoding="utf-8")
    assert cli.main(["machine", str(nc), "--events", str(bad), "--out", str(tmp_path / "p")]) == 2


def test_movej_known_joint_target_untouched():
    prog = demo_program()
    assert any(isinstance(i, MoveJ) for i in prog.instructions)


def test_abb_tool_load_warning_and_absj_without_wobj(tmp_path):
    job = run_job(DEMO_NC, post="abb", out_dir=tmp_path)
    assert job.ok and "TOOL_LOAD_DEFAULT" in codes(job.report, "warnings")
    text = (tmp_path / "Machining.mod").read_text()
    assert "UWAGA: dane obciazenia" in text
    absj = [ln for ln in text.splitlines() if "MoveAbsJ" in ln]
    assert absj and all(ln.rstrip().endswith(",rcTool;") for ln in absj)


@pytest.mark.parametrize("kwargs", [
    {"mass_kg": 0, "cog_mm": [0, 0, 50], "inertia_kg_m2": [0, 0, 0]},
    {"mass_kg": 1, "cog_mm": [0, 0], "inertia_kg_m2": [0, 0, 0]},
    {"mass_kg": 1, "cog_mm": [0, 0, 50], "inertia_kg_m2": [0, -1, 0]},
    {"mass_kg": 1, "cog_mm": [0, 0, float("inf")], "inertia_kg_m2": [0, 0, 0]},
])
def test_tool_load_rejects_invalid_values(kwargs):
    with pytest.raises(ValueError):
        ToolLoad(**kwargs)


@pytest.mark.parametrize("post", ["abb", "kuka"])
def test_pipeline_uses_configured_tool_load_and_records_it(tmp_path, post):
    load = ToolLoad(2.5, (10, -20, 80), (0.01, 0.02, 0.03))
    job = run_job(DEMO_NC, post=post, out_dir=tmp_path, tool_load=load)

    assert job.ok, job.report.summary()
    assert "TOOL_LOAD_DEFAULT" not in codes(job.report, "warnings")
    output = job.outputs[0].read_text(encoding="utf-8")
    manifest = json.loads(job.manifest_path.read_text(encoding="utf-8"))
    assert ("2.500" in output if post == "abb" else "$LOAD = {M 2.500" in output)
    assert manifest["post"]["tool_load"] == {
        "mass_kg": 2.5, "cog_mm": [10.0, -20.0, 80.0], "inertia_kg_m2": [0.01, 0.02, 0.03]
    }


def test_cli_tool_load_file(tmp_path):
    from robocode import cli

    nc = tmp_path / "a.nc"
    nc.write_text(DEMO_NC, encoding="utf-8")
    load = tmp_path / "load.json"
    load.write_text(json.dumps({
        "mass_kg": 2.5,
        "cog_mm": [10, -20, 80],
        "inertia_kg_m2": [0.01, 0.02, 0.03],
    }), encoding="utf-8")
    out = tmp_path / "out"
    assert cli.main(["machine", str(nc), "--post", "kuka", "--tool-load", str(load), "--out", str(out)]) == 0
    assert "$LOAD = {M 2.500" in (out / "Machining.src").read_text(encoding="utf-8")

    load.write_text('{"mass_kg":0,"cog_mm":[0,0,0],"inertia_kg_m2":[0,0,0]}', encoding="utf-8")
    assert cli.main(["machine", str(nc), "--tool-load", str(load), "--out", str(tmp_path / "bad")]) == 2


def test_cli_max_lines_per_file_splits_export(tmp_path):
    from robocode import cli

    nc = tmp_path / "a.nc"
    nc.write_text(DEMO_NC, encoding="utf-8")
    out = tmp_path / "out"
    status = cli.main(["machine", str(nc), "--post", "abb", "--max-lines-per-file", "20",
                       "--out", str(out)])

    parts = sorted(out.glob("Machining_*.mod"))
    assert status == 0 and len(parts) > 1
    assert all(len(path.read_text(encoding="utf-8").splitlines()) <= 20 for path in parts)


def test_cli_lists_and_selects_start_configuration(tmp_path, capsys):
    from robocode import cli

    nc = tmp_path / "a.nc"
    nc.write_text(DEMO_NC, encoding="utf-8")
    status = cli.main(["machine", str(nc), "--list-start-configurations"])
    listing = capsys.readouterr().out
    assert status == 0 and "1: obrót Z=" in listing

    out = tmp_path / "selected"
    status = cli.main(["machine", str(nc), "--start-config", "1", "--out", str(out)])
    assert status == 0
    assert (out / "Machining.src").exists()
