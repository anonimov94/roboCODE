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
def test_post_splits_program_at_instruction_boundaries_and_carries_state(post_name):
    post = get_post(post_name)
    parts = post.generate_chunks(golden_program(), max_lines=14)

    assert len(parts) > 1
    assert all(len(text.splitlines()) <= 14 for _, text in parts)
    assert all(part.name.endswith(f"_{i:03d}") for i, (part, _) in enumerate(parts, 1))
    for part, text in parts:
        assert verify_post_output(post_name, text, part).ok
    combined = "\n".join(text for _, text in parts)
    move_prefixes = ("PTP ", "LIN ", "CIRC ") if post_name == "kuka" else ("MoveAbsJ ", "MoveL ", "MoveC ")
    assert sum(line.lstrip().startswith(move_prefixes) for line in combined.splitlines()) == 3
    for _, text in parts[1:]:
        if post_name == "kuka":
            assert "$BASE =" in text and "$TOOL =" in text and "$VEL.CP =" in text
        else:
            assert "rcWobj.uframe :=" in text and "rcTool.tframe :=" in text
