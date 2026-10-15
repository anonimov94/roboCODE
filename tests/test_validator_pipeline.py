"""Central validator, post read-back check and the end-to-end pipeline (docs §7, §8, §10)."""
import json

import numpy as np
import pytest

from robocode.core import pose as P
from robocode.core.fileio import atomic_write_text, sha256_file
from robocode.demo import DEMO_NC, build_demo_station
from robocode.errors import ExportBlockedError
from robocode.machining.nc_parser import parse_gcode
from robocode.machining.solver import MachiningSettings, solve
from robocode.pipeline import export_or_raise, export_program, run_job
from robocode.post.base import get_post
from robocode.program.instructions import MoveJ, MoveL, Program, SetSpeed
from robocode.program.validator import validate_program, verify_post_output


def demo_program(nc=DEMO_NC):
    st, r, fr, tool = build_demo_station()
    res = solve(r, r.frame_rel_base(fr), tool.tcp(), parse_gcode(nc), MachiningSettings())
    assert res.ok, res.message
    return res.program


def codes(rep):
    return {i.code for i in rep.errors}


def test_demo_program_validates_and_keeps_robot_joints():
    prog = demo_program()
    q0 = prog.robot.joints.copy()
    rep, sim = validate_program(prog)
    assert rep.ok, rep.summary()
    assert rep.samples_checked > 50 and rep.cycle_time_s > 0
    assert np.allclose(prog.robot.joints, q0)  # validation must not move the robot


def test_missing_frame_tool_and_empty_program():
    st, r, fr, tool = build_demo_station()
    p = Program(name="P", robot=None)
    p.instructions.append(MoveJ(joints=np.zeros(6)))
    p.robot = r
    rep, _ = validate_program(p)
    assert {"MISSING_SETFRAME", "MISSING_SETTOOL"} <= codes(rep)
    rep, _ = validate_program(Program(name="E", robot=r))
    assert "EMPTY" in codes(rep)


def test_speed_out_of_range_is_blocking():
    prog = demo_program()
    prog.instructions.insert(3, SetSpeed(linear_mm_s=12000))  # e.g. mm/min mistaken for mm/s
    rep, _ = validate_program(prog)
    assert "SPEED_RANGE" in codes(rep)


def test_joint_limit_and_nan_detected():
    prog = demo_program()
    mj = next(i for i in prog.instructions if isinstance(i, MoveJ))
    mj.joints = np.array([0, 0, 0, 0, 0, 400.0])
    ml = next(i for i in prog.instructions if isinstance(i, MoveL))
    ml.pose = ml.pose.copy()
    ml.pose[0, 3] = np.nan
    rep, _ = validate_program(prog)
    assert {"JOINT_LIMIT", "BAD_POSE"} <= codes(rep)


def test_target_mismatch_detected():
    """Stored joints that do not produce the stored pose = solver/frame bug -> blocking."""
    prog = demo_program()
    ml = [i for i in prog.instructions if isinstance(i, MoveL)][3]
    ml.pose = ml.pose @ P.transl(0.5, 0, 0)
    rep, _ = validate_program(prog)
    assert "TARGET_MISMATCH" in codes(rep)
    assert any(i.source_line is not None for i in rep.errors)  # located in the NC file


def test_unreachable_segment_on_full_trajectory():
    """Endpoints fine, but the straight line between them leaves the workspace."""
    prog = demo_program()
    moves = [i for i in prog.instructions if isinstance(i, MoveL)]
    far = moves[5]
    far.pose = far.pose @ P.transl(0, 0, -2000)
    far.joints = None
    rep, _ = validate_program(prog)
    assert "UNREACHABLE" in codes(rep)


@pytest.mark.parametrize("post", ["kuka", "abb"])
def test_post_read_back_ok(post):
    prog = demo_program()
    text = get_post(post).generate(prog)
    rep = verify_post_output(post, text, prog)
    assert rep.ok, rep.summary()
    assert any(i.code == "POST_VERIFIED" for i in rep.issues)


def test_post_read_back_catches_corruption():
    prog = demo_program()
    text = get_post("kuka").generate(prog).replace("LIN {X 60.000", "LIN {X 61.000", 1)
    assert "POST_MISMATCH" in codes(verify_post_output("kuka", text, prog))


def test_pipeline_exports_with_manifest(tmp_path):
    job = run_job(DEMO_NC, post="kuka", out_dir=tmp_path, nc_name="demo.nc")
    assert job.ok, job.report.summary()
    out = tmp_path / "Machining.src"
    man = json.loads(job.manifest_path.read_text(encoding="utf-8"))
    assert man["exported"] is True and man["format"] == "robocode-manifest/1"
    assert man["outputs"][0]["sha256"] == sha256_file(out)
    assert man["cell"]["robot"]["model_sha256"] and man["post"]["version"]
    assert man["input"]["sha256"] and man["validation"]["ok"] is True
    assert export_or_raise(job) == [out]


def test_pipeline_splits_output_files_and_records_each_hash(tmp_path):
    job = run_job(DEMO_NC, post="kuka", out_dir=tmp_path, max_lines_per_file=20)

    assert job.ok, job.report.summary()
    assert len(job.outputs) > 1
    assert all(len(path.read_text(encoding="utf-8").splitlines()) <= 20 for path in job.outputs)
    manifest = json.loads(job.manifest_path.read_text(encoding="utf-8"))
    assert manifest["post"]["max_lines_per_file"] == 20
    assert [entry["file"] for entry in manifest["outputs"]] == [path.name for path in job.outputs]
    assert all(entry["sha256"] == sha256_file(path) for entry, path in zip(manifest["outputs"], job.outputs))
    assert any(issue.code == "PROGRAM_SPLIT" for issue in job.report.issues)


def test_pipeline_rejects_nonpositive_file_line_limit(tmp_path):
    job = run_job(DEMO_NC, post="kuka", out_dir=tmp_path, max_lines_per_file=0)

    assert not job.ok
    assert "PROGRAM_LINE_LIMIT" in codes(job.report)
    assert not list(tmp_path.glob("*.src"))


def test_pipeline_reports_invalid_start_configuration_and_exports_selection(tmp_path):
    bad = run_job(DEMO_NC, post="kuka", out_dir=tmp_path / "bad",
                  settings=MachiningSettings(start_configuration_index=10000))
    assert not bad.ok
    assert "START_CONFIGURATION" in codes(bad.report)
    assert not list((tmp_path / "bad").glob("*.src"))

    good = run_job(DEMO_NC, post="kuka", out_dir=tmp_path / "good",
                   settings=MachiningSettings(start_configuration_index=1))
    assert good.ok, good.report.summary()
    manifest = json.loads(good.manifest_path.read_text(encoding="utf-8"))
    assert manifest["start_configuration"]["index"] == 1
    assert len(manifest["start_configuration"]["joints_deg"]) == 6


def test_pipeline_warns_about_leftover_split_parts(tmp_path):
    (tmp_path / "Machining_099.src").write_text("stale", encoding="utf-8")

    job = run_job(DEMO_NC, post="kuka", out_dir=tmp_path)

    assert job.ok
    assert any(issue.code == "STALE_OUTPUT" and "Machining_099.src" in issue.message
               for issue in job.report.warnings)


def test_pipeline_blocks_and_writes_no_program(tmp_path):
    job = run_job("G21\nG41 D1\nG1 X1 F100", post="kuka", out_dir=tmp_path)
    assert not job.ok and "NC_PARSE" in codes(job.report)
    assert not list(tmp_path.glob("*.src"))
    assert json.loads(job.manifest_path.read_text(encoding="utf-8"))["exported"] is False
    with pytest.raises(ExportBlockedError):
        export_or_raise(job)


def test_pipeline_unreachable_points_located(tmp_path):
    job = run_job("G21\nG1 X0 Y0 Z0 F100\nG1 X3000 F100", out_dir=tmp_path)
    assert not job.ok
    assert any(i.code == "UNREACHABLE" and i.source_line == 3 for i in job.report.errors)


def test_pipeline_warns_about_stale_output(tmp_path):
    assert run_job(DEMO_NC, out_dir=tmp_path).ok
    job = run_job("G21\nG41\nG1 X1 F1", out_dir=tmp_path)
    assert any(i.code == "STALE_OUTPUT" for i in job.report.warnings)


def test_export_program_blocks_invalid(tmp_path):
    prog = demo_program()
    prog.instructions.insert(3, SetSpeed(linear_mm_s=-5))
    job = export_program(prog, "abb", tmp_path)
    assert not job.ok and not list(tmp_path.glob("*.mod"))


def test_inch_program_end_to_end(tmp_path):
    nc_mm = "G21 G90\nG0 X0 Y0 Z10\nG1 Z-2 F600\nG1 X25.4\nG1 Y25.4\nG0 Z10\n"
    nc_in = "G20 G90\nG0 X0 Y0 Z0.3937007874\nG1 Z-0.0787401575 F23.62204724\nG1 X1\nG1 Y1\nG0 Z0.3937007874\n"
    a = run_job(nc_mm, out_dir=tmp_path / "mm")
    b = run_job(nc_in, out_dir=tmp_path / "in")
    assert a.ok and b.ok
    ta = (tmp_path / "mm" / "Machining.src").read_text()
    tb = (tmp_path / "in" / "Machining.src").read_text()
    assert ta == tb  # same physical program regardless of input units


def test_atomic_write_leaves_no_temp_files(tmp_path):
    p = atomic_write_text(tmp_path / "a" / "x.txt", "hello")
    assert p.read_text() == "hello" and [f.name for f in p.parent.iterdir()] == ["x.txt"]
