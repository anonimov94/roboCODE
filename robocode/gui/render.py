"""Budowanie siatek 3D sceny (pyvista) – bez Qt, testowalne offscreen.

build_scene(station, overlay) -> list[Actor]   (czysta funkcja)
Kolory i styl są własne roboCODE (nie kopiujemy cudzego wyglądu).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..core.scene import Curve, Frame, Object, Robot, Station, Target, Tool

# --- paleta roboCODE ---
COL = {
    "link": "#e8a33d",       # ogniwa robota (bursztyn)
    "joint": "#3a3f4b",      # przeguby (grafit)
    "tool": "#8fa3b8",       # narzędzie
    "x": "#e5484d", "y": "#30a46c", "z": "#3e63dd",
    "path_ok": "#2fbf71", "path_bad": "#e5484d", "normal": "#f2f2f2",
    "trace": "#ffcc33", "target": "#b07cf0", "object": "#a7b0bb", "curve": "#2fbf71",
    "floor": "#2a2e36",
}


@dataclass
class Actor:
    name: str
    mesh: object                 # pyvista.PolyData
    color: str
    opacity: float = 1.0
    line_width: float | None = None
    point_size: float | None = None
    kind: str = "surface"        # surface | lines | points


@dataclass
class Overlay:
    """Dane tymczasowe do narysowania na scenie (projekt obróbki, ślad)."""
    path_points: np.ndarray | None = None   # Nx3, abs mm
    path_normals: np.ndarray | None = None  # Nx3
    reachable: list[bool] | None = None
    trace: np.ndarray | None = None         # Mx3
    show_normals: bool = True
    normal_len: float = 15.0


@dataclass
class RenderOptions:
    frame_size: float = 80.0      # długość osi układów (+/- zmienia)
    show_workspace: bool = False
    show_floor: bool = True
    link_radius: float = 28.0


def _pv():
    import pyvista as pv
    return pv


def axes_actors(T: np.ndarray, size: float, name: str) -> list[Actor]:
    pv = _pv()
    o = T[:3, 3]
    out = []
    for i, ax in enumerate("xyz"):
        d = T[:3, i]
        arrow = pv.Arrow(start=o, direction=d, scale=size, tip_length=0.25, tip_radius=0.08, shaft_radius=0.03)
        out.append(Actor(f"{name}:{ax}", arrow, COL[ax]))
    return out


def robot_actors(r: Robot, opt: RenderOptions) -> list[Actor]:
    pv = _pv()
    base = r.pose_abs()
    frames = [base @ F for F in r.model.fk_all(r.joints)]
    pts = [F[:3, 3] for F in frames]
    out: list[Actor] = []
    # podstawa
    out.append(Actor(f"{r.name}:base", pv.Cylinder(center=base[:3, 3] + base[:3, 2] * 40, direction=base[:3, 2],
                                                   radius=opt.link_radius * 3, height=80), COL["joint"]))
    rad = opt.link_radius
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        L = np.linalg.norm(b - a)
        if L > 1e-6:
            out.append(Actor(f"{r.name}:link{i}", pv.Cylinder(center=(a + b) / 2, direction=b - a,
                                                              radius=rad, height=L), COL["link"]))
        out.append(Actor(f"{r.name}:j{i}", pv.Sphere(radius=rad * 1.25, center=a), COL["joint"]))
        rad = max(rad * 0.85, 10)
    flange = frames[-1]
    out.append(Actor(f"{r.name}:flange", pv.Cylinder(center=flange[:3, 3], direction=flange[:3, 2],
                                                     radius=rad * 1.3, height=8), COL["joint"]))
    # narzędzia
    for t in r.tools():
        tcp = flange @ t.tcp()
        p0, p1 = flange[:3, 3], tcp[:3, 3]
        L = np.linalg.norm(p1 - p0)
        if L > 1e-6:
            out.append(Actor(f"{t.name}:body", pv.Cylinder(center=(p0 + p1) / 2, direction=p1 - p0,
                                                           radius=18 if t is r.active_tool else 10, height=L),
                             COL["tool"], 0.9 if t is r.active_tool else 0.4))
        out += axes_actors(tcp, opt.frame_size * 0.6, f"{t.name}:tcp")
    if opt.show_workspace:
        reach = sum(abs(d.a) + abs(d.d) for d in r.model.dh[1:])
        out.append(Actor(f"{r.name}:workspace", pv.Sphere(radius=reach, center=pts[1], theta_resolution=40,
                                                          phi_resolution=40), COL["path_ok"], 0.06))
    return out


def object_actor(o: Object) -> Actor | None:
    pv = _pv()
    if o.box_size:
        dx, dy, dz = o.box_size
        m = pv.Box(bounds=(0, dx, 0, dy, 0, dz)).transform(o.pose_abs(), inplace=False)
        return Actor(o.name, m, COL["object"], 0.22)
    if not o.mesh_path or not Path(o.mesh_path).exists():
        return None
    m = pv.read(o.mesh_path).transform(o.pose_abs(), inplace=False)
    return Actor(o.name, m, COL["object"], 0.95)


def curve_actor(c: Curve) -> Actor | None:
    pv = _pv()
    if len(c.points) < 2:
        return None
    T = c.pose_abs()
    P = (T[:3, :3] @ c.points.T).T + T[:3, 3]
    return Actor(c.name, pv.lines_from_points(P), COL["curve"], line_width=3, kind="lines")


def overlay_actors(ov: Overlay) -> list[Actor]:
    pv = _pv()
    out: list[Actor] = []
    if ov.path_points is not None and len(ov.path_points) > 1:
        P = np.asarray(ov.path_points, float)
        out.append(Actor("toolpath", pv.lines_from_points(P), COL["path_ok"], line_width=4, kind="lines"))
        if ov.reachable is not None:
            ok = np.asarray(ov.reachable, bool)
            if ok.any():
                out.append(Actor("path:ok", pv.PolyData(P[ok]), COL["path_ok"], point_size=7, kind="points"))
            if (~ok).any():
                out.append(Actor("path:bad", pv.PolyData(P[~ok]), COL["path_bad"], point_size=10, kind="points"))
        if ov.show_normals and ov.path_normals is not None:
            N = np.asarray(ov.path_normals, float)
            segs = np.empty((2 * len(P), 3))
            segs[0::2], segs[1::2] = P, P + N * ov.normal_len
            out.append(Actor("path:normals", pv.line_segments_from_points(segs), COL["normal"],
                             0.6, line_width=1, kind="lines"))
    if ov.trace is not None and len(ov.trace) > 1:
        out.append(Actor("trace", pv.lines_from_points(np.asarray(ov.trace, float)), COL["trace"],
                         line_width=2, kind="lines"))
    return out


def build_scene(station: Station, overlay: Overlay | None = None,
                opt: RenderOptions | None = None) -> list[Actor]:
    pv = _pv()
    opt = opt or RenderOptions()
    out: list[Actor] = []
    if opt.show_floor:
        out.append(Actor("floor", pv.Plane(center=(0, 0, -1), i_size=2000, j_size=2000, i_resolution=20,
                                           j_resolution=20), COL["floor"], 0.35, kind="wire"))
    out += axes_actors(station.pose_abs(), opt.frame_size * 1.5, "station")
    for it in station.walk():
        if not it.visible:
            continue
        if isinstance(it, Robot):
            out += robot_actors(it, opt)
        elif isinstance(it, Frame):
            out += axes_actors(it.pose_abs(), opt.frame_size, it.name)
        elif isinstance(it, Target):
            out.append(Actor(it.name, pv.Sphere(radius=6, center=it.pose_abs()[:3, 3]), COL["target"]))
        elif isinstance(it, Object):
            a = object_actor(it)
            if a:
                out.append(a)
        elif isinstance(it, Curve):
            a = curve_actor(it)
            if a:
                out.append(a)
        elif isinstance(it, Tool):
            pass  # rysowane przy robocie
    if overlay:
        out += overlay_actors(overlay)
    return out


def add_to_plotter(plotter, actors: list[Actor]) -> None:
    for a in actors:
        kw = dict(color=a.color, opacity=a.opacity, name=a.name, pickable=a.kind == "surface")
        if a.kind == "lines":
            plotter.add_mesh(a.mesh, line_width=a.line_width or 2, render_lines_as_tubes=False, **kw)
        elif a.kind == "points":
            plotter.add_mesh(a.mesh, point_size=a.point_size or 6, render_points_as_spheres=True, **kw)
        elif a.kind == "wire":
            plotter.add_mesh(a.mesh, style="wireframe", line_width=1, **kw)
        else:
            plotter.add_mesh(a.mesh, smooth_shading=True, specular=0.3, **kw)


def setup_plotter(plotter) -> None:
    plotter.set_background("#1b1e24", top="#323844")
    plotter.enable_anti_aliasing("ssaa") if hasattr(plotter, "enable_anti_aliasing") else None
    plotter.add_axes(line_width=2, labels_off=False)


def scene_bounds(actors: list[Actor], pad: float = 0.15):
    """Granice sceny bez podłogi i osi stacji – do kadrowania kamery."""
    bs = [a.mesh.bounds for a in actors if a.name != "floor" and not a.name.startswith("station:")]
    if not bs:
        return None
    b = np.array(bs)
    lo, hi = b[:, 0::2].min(0), b[:, 1::2].max(0)
    d = (hi - lo).max() * pad
    lo, hi = lo - d, hi + d
    return (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])


VIEW_DIRS = {  # kierunek od środka sceny do kamery, wektor "w górę"
    "iso": ((1.0, -1.0, 0.75), (0, 0, 1)),
    "top": ((0, 0, 1.0), (0, 1, 0)),
    "front": ((0, -1.0, 0), (0, 0, 1)),
    "side": ((1.0, 0, 0), (0, 0, 1)),
}


def set_view(plotter, which: str, bounds=None) -> None:
    """Ustawia kamerę tak, by bounds wypełniały kadr (bez wpływu podłogi)."""
    if bounds is None:
        bounds = plotter.bounds
    b = np.asarray(bounds, float)
    c = np.array([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2])
    diag = float(np.linalg.norm([b[1] - b[0], b[3] - b[2], b[5] - b[4]])) or 1000.0
    d, up = VIEW_DIRS.get(which, VIEW_DIRS["iso"])
    d = np.asarray(d) / np.linalg.norm(d)
    cam = plotter.camera
    cam.focal_point = tuple(c)
    cam.position = tuple(c + d * diag * 1.9)
    cam.up = up
    cam.view_angle = 30
    plotter.reset_camera_clipping_range()


def path_bounds(overlay, pad_mm: float = 40.0):
    if overlay is None or overlay.path_points is None or len(overlay.path_points) == 0:
        return None
    P = np.asarray(overlay.path_points)
    lo, hi = P.min(0) - pad_mm, P.max(0) + pad_mm
    return (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])


def render_png(station: Station, path: str, overlay: Overlay | None = None,
               opt: RenderOptions | None = None, view: str = "iso", size=(1200, 800),
               focus: str = "scene") -> str:
    """Zrzut sceny do PNG (offscreen) – do testów, raportów, CLI. focus: scene | path."""
    pv = _pv()
    p = pv.Plotter(off_screen=True, window_size=size)
    setup_plotter(p)
    if focus == "path" and opt is None:
        opt = RenderOptions(frame_size=20.0)   # osie w skali detalu
    actors = build_scene(station, overlay, opt)
    add_to_plotter(p, actors)
    b = path_bounds(overlay) if focus == "path" else None
    set_view(p, view, b or scene_bounds(actors))
    p.screenshot(path)
    p.close()
    return path
