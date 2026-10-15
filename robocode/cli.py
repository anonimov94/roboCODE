"""CLI: robocode demo | machine <plik.nc> --post kuka --out out/ | gui"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .core import pose as P
from .core.scene import Frame, Robot, Station, Tool
from .kinematics.robot_model import example_6r
from .machining.nc_parser import parse_gcode
from .machining.solver import MachiningSettings, solve
from .post.base import get_post
from .program.simulator import simulate

DEMO_NC = """(kwadrat 60x60 mm, glebokosc 2 mm)
G90 G17
T1 M6
S12000 M3
G0 X0 Y0 Z10
G1 Z-2 F600
G1 X60
G1 Y60
G1 X0
G1 Y0
G2 X20 Y0 I10 J0
G0 Z10
M5
"""


def build_demo_station(post: str = "kuka"):
    st = Station(name="Demo")
    r = st.add(Robot(name="Robot", model=example_6r(), joints=np.array([0, 40, 15, 0, 35, 0.0]), post_processor=post))
    spindle = r.add(Tool(name="Wrzeciono", pose=P.transl(0, 0, 150)))
    r.active_tool = spindle
    fr = st.add(Frame(name="Detal", pose=P.transl(550, -30, 150)))
    r.active_frame = fr
    return st, r, fr, spindle


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
    sub.add_parser("gui", help="Uruchom GUI (wymaga PySide6)")
    a = ap.parse_args(argv)
    if a.cmd == "demo":
        return run_machining(DEMO_NC, a.post, a.out)
    if a.cmd == "machine":
        s = MachiningSettings(z_range_deg=a.zrange, z_step_deg=a.zstep)
        return run_machining(Path(a.nc).read_text(), a.post, a.out, s)
    if a.cmd == "gui":
        from .gui.main_window import main as gui_main
        gui_main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
