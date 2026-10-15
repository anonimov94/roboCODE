from pathlib import Path

import numpy as np
import pytest

from robocode.core import pose as P
from robocode.post.base import get_post
from robocode.program.instructions import (
    Call,
    Message,
    MoveC,
    MoveJ,
    MoveL,
    Pause,
    Program,
    SetFrame,
    SetIO,
    SetRounding,
    SetSpeed,
    SetTool,
    WaitIO,
)
from robocode.post.tool_load import ToolLoad
from robocode.program.validator import verify_post_output


def golden_program():
    program = Program(name="Golden")
    program.instructions = [
        SetFrame(P.transl(100, 200, 300) @ P.rotz(90), "Fixture"),
        SetTool(P.transl(0, 0, 150), "Spindle"),
        SetRounding(-1),
        SetSpeed(linear_mm_s=50, joint_deg_s=30),
        MoveJ(np.array([0, 10, -20, 30, 40, -50.0]), P.eye()),
        MoveL(P.transl(10, 20, 30) @ P.rotz(45), np.array([1, 2, 3, 4, 5, 6.0])),
        MoveC(
            P.transl(15, 25, 35),
            P.transl(20, 30, 40),
            np.array([2, 3, 4, 5, 6, 7.0]),
            np.array([3, 4, 5, 6, 7, 8.0]),
        ),
        SetRounding(5),
        Message("Kontrola"),
        SetIO("do1", 1),
        WaitIO("di1", 1, timeout_ms=2500),
        Pause(500),
        Call("CycleDone"),
    ]
    return program


@pytest.mark.parametrize(
    ("post_name", "golden_file"),
    [("kuka", "controller_golden.src"), ("abb", "controller_golden.mod")],
)
def test_post_output_matches_golden_file(post_name, golden_file):
    golden_path = Path(__file__).parent / "golden" / golden_file
    expected = golden_path.read_text(encoding="utf-8")
    assert get_post(post_name).generate(golden_program()) == expected


@pytest.mark.parametrize(
    ("post_name", "expected"),
    [
        ("abb", "],[2.500,[10.000,-20.000,80.000],[1,0,0,0],0.010000,0.020000,0.030000]];"),
        ("kuka", "$LOAD = {M 2.500, MX 0.025000, MY -0.050000, MZ 0.200000, "
                  "JX 0.010000, JY 0.020000, JZ 0.030000}"),
    ],
)
def test_posts_emit_configured_tool_load(post_name, expected):
    load = ToolLoad(2.5, (10, -20, 80), (0.01, 0.02, 0.03))
    text = get_post(post_name, tool_load=load).generate(golden_program())

    assert expected in text


@pytest.mark.parametrize("post_name", ["kuka", "abb"])
def test_post_split_has_safe_restart_sequence(post_name):
    """Review of the Copilot change: parts 2+ must not start with LIN inside the material, nor without
    the spindle; parts 1..n-1 must end with a retract. Every part reads back cleanly."""
    from robocode.demo import DEMO_NC, build_demo_station
    from robocode.machining.nc_parser import parse_gcode
    from robocode.machining.solver import MachiningSettings, solve
    st, r, fr, tool = build_demo_station()
    prog = solve(r, r.frame_rel_base(fr), tool.tcp(), parse_gcode(DEMO_NC), MachiningSettings()).program
    post = get_post(post_name)
    parts = post.generate_chunks(prog, max_lines=30)
    assert len(parts) > 1
    assert all(len(text.splitlines()) <= 30 for _, text in parts)
    assert all(part.name.endswith(f"_{i:03d}") for i, (part, _) in enumerate(parts, 1))
    ptp, lin = ("PTP ", "LIN ") if post_name == "kuka" else ("MoveAbsJ ", "MoveL ")
    spindle = "SpindleOn" if post_name == "kuka" else "SpindleOn;"
    for k, (part, text) in enumerate(parts):
        assert verify_post_output(post_name, text, part).ok
        lines = [ln.strip() for ln in text.splitlines()]
        moves = [ln for ln in lines if ln.startswith((ptp, lin))]
        if k > 0:
            assert moves[0].startswith(ptp), "part must start with a joint move above the path"
            first_lin = next(i for i, ln in enumerate(lines) if ln.startswith(lin))
            assert any(spindle in ln for ln in lines[:first_lin]), "spindle must be re-enabled before cutting"
        if k < len(parts) - 1:
            assert moves[-1].startswith(lin)
            assert getattr(part.instructions[-1], "inserted_by_split", False), "part must end with a retract"


def test_post_split_without_robot_is_refused():
    with pytest.raises(ValueError):
        get_post("kuka").generate_chunks(golden_program(), max_lines=14)