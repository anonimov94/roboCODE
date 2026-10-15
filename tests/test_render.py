"""Testy widoku 3D (offscreen). Pomijane, gdy brak pyvista."""
import numpy as np
import pytest

pv = pytest.importorskip("pyvista")

from robocode.demo import demo_project
from robocode.gui.render import (Overlay, RenderOptions, build_scene, path_bounds, render_png,
                                 scene_bounds)
from robocode.gui.handlers import AppState, dispatch


def test_build_scene_contains_robot_tool_frames_and_path():
    st, proj = demo_project()
    actors = build_scene(st, proj.overlay())
    names = {a.name for a in actors}
    assert any(n.startswith("Robot:link") for n in names)
    assert "Wrzeciono:body" in names
    assert "Detal:x" in names and "Surowiec" in names
    assert "toolpath" in names and "path:ok" in names and "path:bad" not in names


def test_unreachable_points_drawn_red():
    ov = Overlay(path_points=np.array([[0, 0, 0], [10, 0, 0], [20, 0, 0.0]]),
                 path_normals=np.tile([0, 0, 1.0], (3, 1)), reachable=[True, False, True])
    names = {a.name for a in build_scene(demo_project()[0], ov)}
    assert "path:bad" in names


def test_tcp_actor_matches_kinematics():
    st, proj = demo_project()
    r = proj.robot
    actors = {a.name: a for a in build_scene(st, None, RenderOptions(frame_size=50))}
    tcp = r.tcp_abs()
    arrow = actors["Wrzeciono:tcp:z"].mesh
    # strzałka osi Z TCP zaczyna się w punkcie TCP
    d = np.linalg.norm(arrow.points - tcp[:3, 3], axis=1).min()
    assert d < 2.0


def test_bounds_ignore_floor_and_focus_on_path():
    st, proj = demo_project()
    ov = proj.overlay()
    b = scene_bounds(build_scene(st, ov))
    assert b[1] - b[0] < 1800  # podłoga 2 m nie wchodzi w kadr
    pb = path_bounds(ov)
    assert pb[1] - pb[0] < 200


def test_render_png_offscreen(tmp_path):
    st, proj = demo_project()
    out = render_png(st, str(tmp_path / "s.png"), proj.overlay(), size=(400, 300), focus="path")
    data = (tmp_path / "s.png").read_bytes()
    assert out.endswith("s.png") and data[:4] == b"\x89PNG" and len(data) > 5000


def test_view_handlers_and_machining_from_file(tmp_path):
    s = AppState()
    assert "Demo" in dispatch(s, "open_demo")
    assert s.overlay is not None and s.view_request == "iso"
    assert "Symulacja OK" in dispatch(s, "simulate_program")
    assert s.overlay.trace is not None and len(s.overlay.trace) > 10
    dispatch(s, "view_path")
    assert s.render_opt.frame_size < 80
    dispatch(s, "frames_bigger")
    nc = tmp_path / "a.nc"
    nc.write_text("G0 X0 Y0 Z5\nG1 Z-1 F300\nG1 X20\nG1 Y20\nG0 Z5\n")
    s2 = AppState()
    msg = dispatch(s2, "new_machining_project", str(nc))
    assert msg.startswith("✓"), msg
    assert s2.view_request == "path"
