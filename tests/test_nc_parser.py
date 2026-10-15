"""NC parser: modal state, units, planes, arcs, strictness (docs §6.4)."""
import numpy as np
import pytest

from robocode.errors import NCParseError, UnsupportedCommandError
from robocode.machining.nc_parser import parse_gcode


def pts(tp):
    return np.array([p.p for p in tp.points])


def test_absolute_and_incremental():
    tp = parse_gcode("G21 G90\nG1 X10 Y0 Z0 F100\nG91\nG1 X5\nG1 Y5\nG90\nG1 X0")
    assert np.allclose(pts(tp), [[10, 0, 0], [15, 0, 0], [15, 5, 0], [0, 5, 0]])


def test_inch_units_converted_to_mm():
    tp = parse_gcode("G20 G90\nG1 X1 Y2 Z0 F10")
    assert np.allclose(tp.points[0].p, [25.4, 50.8, 0])
    assert np.isclose(tp.points[0].feed, 254.0)  # in/min -> mm/min
    assert tp.source_units == "inch"


def test_missing_units_is_warning_not_silent():
    tp = parse_gcode("G1 X1 F10")
    assert any("G20/G21" in w for w in tp.warnings)


def test_g1_without_feed_is_error_in_strict_mode():
    with pytest.raises(UnsupportedCommandError):
        parse_gcode("G21\nG1 X1")
    with pytest.raises(UnsupportedCommandError):
        parse_gcode("G21\nG0 X10 Y0\nG2 X0 Y10 I-10 J0")
    tp = parse_gcode("G21\nG1 X1", strict=False)
    assert any("posuwu F" in w for w in tp.warnings)


@pytest.mark.parametrize("code", ["G41 D1", "G42", "G43 H1", "G81 Z-5 R1", "G93", "G28", "G55"])
def test_meaning_changing_codes_rejected(code):
    with pytest.raises(UnsupportedCommandError) as e:
        parse_gcode(f"G21\n{code}\nG1 X1 F100")
    assert e.value.line_no == 2
    assert code.split()[0] in str(e.value)  # blame the G code, not its D/H argument


def test_non_strict_turns_unsupported_into_warning():
    tp = parse_gcode("G21\nG41 D1\nG1 X1 F100", strict=False)
    assert len(tp) == 1 and any("G41" in w for w in tp.warnings)


def test_c_axis_rejected():
    with pytest.raises(UnsupportedCommandError):
        parse_gcode("G21\nG1 X1 C30 F100")


def test_garbage_line_rejected_with_line_number():
    with pytest.raises(NCParseError) as e:
        parse_gcode("G21\nG1 X1 F100\nG1 X$2")
    assert e.value.line_no == 3


def test_comments_and_percent_ignored():
    tp = parse_gcode("%\n(header)\nG21 ; mm\nN10 G1 X1 F100 (move)\n%")
    assert len(tp) == 1


def _all_on_circle(P, c, r, u, v):
    return np.allclose(np.hypot(P[:, u] - c[0], P[:, v] - c[1]), r, atol=1e-6)


def test_g2_cw_ijk_xy_plane():
    tp = parse_gcode("G21 G17\nG1 X10 Y0 Z0 F100\nG2 X0 Y10 I-10 J0", arc_seg_mm=0.5)
    P = pts(tp)[1:]
    assert _all_on_circle(P, (0, 0), 10, 0, 1)
    assert np.allclose(P[-1], [0, 10, 0])
    # CW from (10,0) to (0,10) goes the long way (through -Y): some point must have y < 0
    assert P[:, 1].min() < -9


def test_g3_ccw_short_way():
    tp = parse_gcode("G21\nG1 X10 Y0 F100\nG3 X0 Y10 I-10 J0")
    P = pts(tp)[1:]
    assert P[:, 1].min() >= -1e-9 and P[:, 0].min() >= -1e-9


def test_full_circle():
    tp = parse_gcode("G21\nG1 X10 Y0 F100\nG2 X10 Y0 I-10 J0", arc_seg_mm=1)
    P = pts(tp)[1:]
    assert len(P) > 50 and _all_on_circle(P, (0, 0), 10, 0, 1)


@pytest.mark.parametrize("r,cw,expect_long", [(10, False, False), (-10, False, True), (10, True, False),
                                              (-10, True, True)])
def test_r_arcs_choose_correct_centre(r, cw, expect_long):
    g = "G2" if cw else "G3"
    tp = parse_gcode(f"G21\nG1 X10 Y0 F100\n{g} X0 Y10 R{r}")
    P = pts(tp)[1:]
    assert np.allclose(P[-1], [0, 10, 0])
    # R>0 = short arc (<=180 deg), R<0 = long arc; the long one leaves the 0..10 square
    long_way = (P[:, 1].min() < -1) or (P[:, 0].max() > 11) or (P[:, 0].min() < -1) or (P[:, 1].max() > 11)
    assert long_way == expect_long


def test_r_too_small_rejected():
    with pytest.raises(NCParseError):
        parse_gcode("G21\nG1 X0 Y0 F100\nG2 X20 Y0 R5")


def test_inconsistent_arc_rejected():
    with pytest.raises(NCParseError, match="niespójny"):
        parse_gcode("G21\nG1 X10 Y0 F100\nG2 X0 Y12 I-10 J0")


def test_g18_arc_in_zx_plane_and_helix_along_y():
    tp = parse_gcode("G21 G18\nG1 X10 Y0 Z0 F100\nG3 X0 Z10 Y5 I-10 K0", arc_seg_mm=0.5)
    P = pts(tp)[1:]
    assert _all_on_circle(P, (0, 0), 10, 2, 0)  # (u,v) = (Z,X)
    assert np.allclose(P[-1], [0, 5, 10])
    assert np.all(np.diff(P[:, 1]) >= -1e-9)  # helix component monotonic


def test_g19_rejects_wrong_plane_letter():
    with pytest.raises(NCParseError):
        parse_gcode("G21 G19\nG1 X0 Y10 Z0 F100\nG2 Y0 Z10 I5 J-10")


def test_tool_vector_ijk_and_ab_conflict():
    tp = parse_gcode("G21\nG1 X0 Y0 Z0 I0 J0 K2 F100")
    assert np.allclose(tp.points[0].n, [0, 0, 1])
    with pytest.raises(NCParseError):
        parse_gcode("G21\nG1 X0 B10 I0 J0 K1 F100")
    with pytest.raises(NCParseError):
        parse_gcode("G21\nG1 X0 I0 J0 K0 F100")


def test_events_order_and_end_events():
    tp = parse_gcode("G21\nT2 M6\nS9000 M3\nG1 X1 F100\nM0\nG1 X2\nM5\nM30\nG1 X99")
    ev = tp.points[0].events
    assert ev == ["SetTool(2)", "Spindle(9000)", "SpindleOn"]  # S before M3 in the same block
    assert tp.points[1].events == ["Pause(-1)"]
    assert tp.end_events == ["SpindleOff"]
    assert len(tp) == 2 and any("M2/M30" in w for w in tp.warnings)


def test_dwell():
    tp = parse_gcode("G21\nG4 P1.5\nG1 X1 F100")
    assert tp.points[0].events == ["Pause(1500)"]
    with pytest.raises(NCParseError):
        parse_gcode("G21\nG4\nG1 X1 F100")


def test_fuzz_parser_never_crashes_unexpectedly():
    """Random garbage must raise NCParseError or parse – never IndexError/ZeroDivision/etc."""
    rng = np.random.default_rng(0)
    alphabet = list("GXYZIJKRFMSTABE0123456789.- \n()%;") + ["G2", "G3", "G91", "G20", "R0", "I0J0"]
    for _ in range(500):
        txt = "".join(rng.choice(alphabet, size=rng.integers(1, 60)))
        try:
            tp = parse_gcode(txt)
        except NCParseError:
            continue
        for p in tp.points:
            assert np.all(np.isfinite(p.p)) and np.isclose(np.linalg.norm(p.n), 1)


def test_r_arc_centres_explicit():
    cw = pts(parse_gcode("G21\nG1 X10 Y0 F100\nG2 X0 Y10 R10"))[1:]
    ccw = pts(parse_gcode("G21\nG1 X10 Y0 F100\nG3 X0 Y10 R10"))[1:]
    assert _all_on_circle(cw, (10, 10), 10, 0, 1)
    assert _all_on_circle(ccw, (0, 0), 10, 0, 1)
