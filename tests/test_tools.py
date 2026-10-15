"""Bot-drafted helpers (sample NC generator, HTML report) + fixes found while testing them."""
import json

import numpy as np
import pytest

from robocode.demo import build_demo_station
from robocode.machining.nc_parser import parse_gcode
from robocode.machining.solver import MachiningSettings, solve
from robocode.machining.toolpath import PathPoint, Toolpath
from robocode.pipeline import run_job
from robocode.program.instructions import MoveL
from robocode.program.simulator import simulate
from robocode.tools import report_html, sample_nc


@pytest.mark.parametrize("name", sorted(sample_nc.ALL_SAMPLES))
def test_every_sample_passes_full_pipeline(name, tmp_path):
    """Each generated program must parse strictly and export on the demo cell."""
    job = run_job(sample_nc.ALL_SAMPLES[name](), out_dir=tmp_path, nc_name=f"{name}.nc")
    assert job.ok, job.report.summary()


def test_samples_dialect_and_format():
    for name, gen in sample_nc.ALL_SAMPLES.items():
        txt = gen()
        lines = txt.strip().splitlines()
        assert lines[0].startswith("(") and lines[-2:] == ["M5", "M30"], name
        assert "-0 " not in txt + " " and "-0\n" not in txt, name
        parse_gcode(txt, strict=True)  # raises on anything outside the dialect
    assert sample_nc.fmt(-0.00001) == "0" and sample_nc.fmt(1.23456) == "1.2346" and sample_nc.fmt(2.0) == "2"


def test_samples_validate_arguments_and_write(tmp_path):
    with pytest.raises(ValueError):
        sample_nc.circle(radius=0)
    with pytest.raises(ValueError):
        sample_nc._header("nested (comment)")
    paths = sample_nc.write_samples(tmp_path)
    assert len(paths) == len(sample_nc.ALL_SAMPLES) and all(p.read_text() for p in paths)


def test_report_from_real_manifest(tmp_path):
    job = run_job(sample_nc.circle(), out_dir=tmp_path, nc_name="circle.nc")
    out = report_html.write_report(job.manifest_path)
    page = out.read_text(encoding="utf-8")
    assert out.suffix == ".html" and "WYEKSPORTOWANO" in page and "circle.nc" in page
    assert job.outputs[0].name in page


def test_report_blocked_and_missing_keys():
    assert "EKSPORT ZABLOKOWANY" in report_html.render_manifest_html({})
    assert "Brak uwag" in report_html.render_manifest_html({"validation": None, "cell": "garbage"})
    page = report_html.render_manifest_html({"cell": {"tool": {"tcp": [[1, 2], "x"]}}})
    assert "n/d" in page


def test_report_escapes_hostile_input():
    evil = "<script>alert(1)</script>"
    m = {"input": {"name": evil}, "validation": {"issues": [{"severity": "error", "message": evil}]},
         "settings": {evil: evil}, "outputs": [{"file": evil, "sha256": evil}]}
    page = report_html.render_manifest_html(m)
    assert "<script>" not in page and "&lt;script&gt;" in page


def test_report_issue_order_error_warning_info():
    m = {"validation": {"issues": [{"severity": "info", "code": "C_INFO"}, {"severity": "warning", "code": "B_WARN"},
                                   {"severity": "error", "code": "A_ERR"}]}}
    page = report_html.render_manifest_html(m)
    assert page.index("A_ERR") < page.index("B_WARN") < page.index("C_INFO")


def test_report_cli(tmp_path, capsys):
    assert report_html.main([str(tmp_path / "nope.json")]) == 1
    bad = tmp_path / "bad.json"
    bad.write_text("{oops", encoding="utf-8")
    assert report_html.main([str(bad)]) == 1
    good = tmp_path / "m.manifest.json"
    good.write_text(json.dumps({"exported": True}), encoding="utf-8")
    assert report_html.main([str(good), str(tmp_path / "r.html")]) == 0
    assert (tmp_path / "r.html").exists()


# ---- fixes discovered while running the bot samples -------------------------------------------

def test_densify_orientation_limits_step():
    a = PathPoint(np.zeros(3), np.array([0, 0, 1.0]), line=1)
    b = PathPoint(np.array([10.0, 0, 0]), np.array([np.sin(np.radians(40)), 0, np.cos(np.radians(40))]), line=2)
    tp = Toolpath([a, b]).densify_orientation(5.0)
    assert len(tp) == 9
    for p, q in zip(tp.points, tp.points[1:]):
        assert np.degrees(np.arccos(np.clip(p.n @ q.n, -1, 1))) <= 5.0 + 1e-9
        assert np.isclose(np.linalg.norm(q.n), 1)
    assert all(p.line == 2 for p in tp.points[1:])


def test_large_reorientation_in_one_block_is_solved():
    st, r, fr, tool = build_demo_station()
    nc = "G21\nG1 X0 Y0 Z-1 B20 F500\nG1 X50 B-20"
    assert solve(r, r.frame_rel_base(fr), tool.tcp(), parse_gcode(nc), MachiningSettings()).ok
    off = MachiningSettings(max_orientation_step_deg=0)
    res = solve(r, r.frame_rel_base(fr), tool.tcp(), parse_gcode(nc), off)
    assert not res.ok and "skoku konfiguracji" in res.message  # honest reason, not "unreachable"


def test_pure_rotation_move_takes_time_and_is_sampled():
    st, r, fr, tool = build_demo_station()
    nc = "G21\nG1 X0 Y0 Z-1 B15 F500\nG1 B-15"
    res = solve(r, r.frame_rel_base(fr), tool.tcp(), parse_gcode(nc), MachiningSettings(approach_mm=0,
                max_orientation_step_deg=0, z_range_deg=0))
    assert res.ok, res.message
    n_moves = sum(isinstance(i, MoveL) for i in res.program.instructions)
    sim = simulate(res.program, restore_joints=True)
    assert sim.cycle_time_s >= 30 / 90 - 1e-6  # 30 deg at 90 deg/s
    assert sim.report.samples_checked > n_moves + 5
