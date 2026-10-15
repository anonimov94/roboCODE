"""CLI: robocode demo | machine <plik.nc> --post kuka --out out/ | gui"""
from __future__ import annotations

import argparse
from pathlib import Path

from .demo import DEMO_NC, build_demo_station, demo_project
from .machining.nc_parser import parse_gcode
from .machining.solver import MachiningSettings, solve
from .post.base import get_post
from .program.simulator import simulate

def run_machining(nc_text: str, post: str, out: str, settings: MachiningSettings | None = None):
    st, r, fr, tool = build_demo_station(post)
    tp = parse_gcode(nc_text)
    res = solve(r, r.frame_rel_base(fr), tool.tcp(), tp, settings or MachiningSettings(), name="Machining")
    print(res.message)
    if not res.ok:
        return 1
    sim = simulate(res.program)
    print(f"Symulacja: {'OK' if sim.ok else 'BŁĘDY'} | czas cyklu ≈ {sim.cycle_time_s:.1f} s | punkty śladu: {len(sim.trace)}")
    for w in sim.warnings[:5]:
        print("  ⚠", w)
    for e in sim.errors[:5]:
        print("  ✗", e)
    files = get_post(post).save(res.program, out)
    print("Zapisano:", ", ".join(str(f) for f in files))
    return 0 if sim.ok else 2


def main(argv=None):
    ap = argparse.ArgumentParser(prog="robocode")
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("demo", help="G-code demo -> robot -> program")
    d.add_argument("--post", default="kuka", choices=["kuka", "abb", "generic"])
    d.add_argument("--out", default="out")
    m = sub.add_parser("machine", help="Plik NC -> program robota")
    m.add_argument("nc")
    m.add_argument("--post", default="kuka", choices=["kuka", "abb", "generic"])
    m.add_argument("--out", default="out")
    m.add_argument("--zrange", type=float, default=180)
    m.add_argument("--zstep", type=float, default=20)
    rd = sub.add_parser("render", help="Zrzut sceny demo z ścieżką do PNG (offscreen)")
    rd.add_argument("--nc", default=None)
    rd.add_argument("--out", default="scene.png")
    rd.add_argument("--view", default="iso", choices=["iso", "top", "front", "side"])
    rd.add_argument("--focus", default="scene", choices=["scene", "path"])
    sub.add_parser("gui", help="Uruchom GUI (wymaga PySide6)")
    a = ap.parse_args(argv)
    if a.cmd == "demo":
        return run_machining(DEMO_NC, a.post, a.out)
    if a.cmd == "machine":
        s = MachiningSettings(z_range_deg=a.zrange, z_step_deg=a.zstep)
        return run_machining(Path(a.nc).read_text(), a.post, a.out, s)
    if a.cmd == "render":
        from .gui.render import render_png
        from .program.simulator import simulate as _sim
        st, proj = demo_project(Path(a.nc).read_text() if a.nc else DEMO_NC)
        print(proj.result.message)
        trace = None
        if proj.result.ok:
            q0 = proj.robot.joints.copy()
            sim = _sim(proj.result.program)
            trace = sim.trace
            proj.robot.joints = q0
        print("Zapisano", render_png(st, a.out, proj.overlay(trace), view=a.view, focus=a.focus))
        return 0
    if a.cmd == "gui":
        from .gui.main_window import main as gui_main
        gui_main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
