"""CLI: robocode demo | machine <plik.nc> --post kuka --out out/ | gui"""
from __future__ import annotations

if __name__ == "__main__" and not __package__:
    # Run directly as a file: make the package importable and use absolute imports.
    import pathlib as _pl
    import sys as _sys
    _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[1]))
    from robocode.cli import main as _main
    raise SystemExit(_main())

import argparse
from pathlib import Path

from .demo import DEMO_NC, demo_project
from .machining.solver import MachiningSettings
from .pipeline import run_job

def run_machining(nc_text: str, post: str, out: str, settings: MachiningSettings | None = None,
                  station: str | None = None, nc_name: str = "<demo>", strict: bool = True,
                  event_map: dict | None = None) -> int:
    """Pipeline: parse -> solve -> validate -> post -> verify -> manifest. Exit 0 = exported, 2 = blocked."""
    job = run_job(nc_text, post=post, out_dir=out, settings=settings or MachiningSettings(),
                  station_path=station, nc_name=nc_name, strict=strict, event_map=event_map)
    print(job.message)
    print(job.report.summary())
    print("Manifest:", job.manifest_path)
    return 0 if job.ok else 2


def main(argv=None):
    from ._console import setup_console
    setup_console()
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
    m.add_argument("--station", default=None, help="plik stacji .rcs.json (domyślnie komórka demo)")
    m.add_argument("--approach", type=float, default=100.0, help="najazd/odjazd wzdłuż osi narzędzia, mm")
    m.add_argument("--events", default=None,
                   help='JSON: zdarzenie -> kod sterownika, np. {"SpindleOn": "$OUT[1] = TRUE"}')
    m.add_argument("--no-strict", action="store_true",
                   help="nieobsługiwane kody NC jako ostrzeżenia zamiast błędów (NIEZALECANE)")
    rd = sub.add_parser("render", help="Zrzut sceny demo z ścieżką do PNG (offscreen)")
    rd.add_argument("--nc", default=None)
    rd.add_argument("--out", default="scene.png")
    rd.add_argument("--view", default="iso", choices=["iso", "top", "front", "side"])
    rd.add_argument("--focus", default="scene", choices=["scene", "path"])
    rp = sub.add_parser("report", help="Raport HTML z manifestu (<nazwa>.manifest.json)")
    rp.add_argument("manifest")
    rp.add_argument("out", nargs="?", default=None)
    sp = sub.add_parser("samples", help="Zapisz przykładowe pliki NC do folderu")
    sp.add_argument("folder", nargs="?", default="examples/samples")
    sub.add_parser("gui", help="Uruchom GUI (wymaga PySide6)")
    a = ap.parse_args(argv)
    if a.cmd == "demo":
        return run_machining(DEMO_NC, a.post, a.out)
    if a.cmd == "machine":
        s = MachiningSettings(z_range_deg=a.zrange, z_step_deg=a.zstep, approach_mm=a.approach)
        events = None
        if a.events:
            import json
            events = json.loads(Path(a.events).read_text(encoding="utf-8"))
            if not isinstance(events, dict) or not all(isinstance(v, str) for v in events.values()):
                print("✗ --events: oczekiwano obiektu JSON {nazwa_zdarzenia: kod_sterownika}")
                return 2
        return run_machining(Path(a.nc).read_text(encoding="utf-8"), a.post, a.out, s, station=a.station,
                             nc_name=Path(a.nc).name, strict=not a.no_strict, event_map=events)
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
    if a.cmd == "report":
        from .tools.report_html import main as report_main
        return report_main([a.manifest] + ([a.out] if a.out else []))
    if a.cmd == "samples":
        from .tools.sample_nc import main as samples_main
        return samples_main([a.folder])
    if a.cmd == "gui":
        from .gui.main_window import main as gui_main
        gui_main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
