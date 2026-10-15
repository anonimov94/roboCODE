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
import json
from pathlib import Path

import numpy as np

from .coop.calibration import calibrate_bases, calibrate_tcp, frame_from_3_points
from .core.fileio import atomic_write_text
from .demo import DEMO_NC, demo_project
from .errors import RoboCodeError
from .machining.solver import MachiningSettings
from .pipeline import list_start_configurations, run_job
from .post.tool_load import ToolLoad

def run_machining(nc_text: str, post: str, out: str, settings: MachiningSettings | None = None,
                  station: str | None = None, nc_name: str = "<demo>", strict: bool = True,
                  event_map: dict | None = None, tool_load: ToolLoad | None = None,
                  max_lines_per_file: int | None = None, ros: bool = False) -> int:
    """Pipeline: parse -> solve -> validate -> post -> verify -> manifest. Exit 0 = exported, 2 = blocked."""
    job = run_job(nc_text, post=post, out_dir=out, settings=settings or MachiningSettings(),
                  station_path=station, nc_name=nc_name, strict=strict, event_map=event_map,
                  tool_load=tool_load, max_lines_per_file=max_lines_per_file, ros_trajectory=ros)
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
    m.add_argument("--tool-load", default=None,
                   help="JSON obciążenia narzędzia: mass_kg, cog_mm [x,y,z], inertia_kg_m2 [Ix,Iy,Iz]")
    m.add_argument("--max-lines-per-file", type=int, default=None,
                   help="podziel program na części po max N linii (z bezpiecznym wznowieniem); domyślnie bez podziału")
    m.add_argument("--list-start-configurations", action="store_true",
                   help="wypisz konfiguracje IK (etykiety F/R, U/D, F/N) dla pierwszego punktu i zakończ")
    m.add_argument("--start-config", type=int, default=None,
                   help="wybór konfiguracji początkowej 1-based z --list-start-configurations")
    m.add_argument("--ros", action="store_true",
                   help="zapisz też <nazwa>.ros_trajectory.json do odtworzenia w ROS 2 / Gazebo (ros2/)")
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
    cal = sub.add_parser("calibrate", help="Kalibracja TCP, relacji baz lub układu z pliku JSON")
    cal.add_argument("kind", choices=["tcp", "bases", "frame3"])
    cal.add_argument("input", help="plik JSON z pomiarami w milimetrach")
    cal.add_argument("--out", default="calibration-result.json", help="plik wynikowy JSON")
    sub.add_parser("gui", help="Uruchom GUI (wymaga PySide6)")
    a = ap.parse_args(argv)
    if a.cmd == "demo":
        return run_machining(DEMO_NC, a.post, a.out)
    if a.cmd == "machine":
        s = MachiningSettings(z_range_deg=a.zrange, z_step_deg=a.zstep, approach_mm=a.approach,
                              start_configuration_index=a.start_config)
        nc_text = Path(a.nc).read_text(encoding="utf-8")
        if a.list_start_configurations:
            try:
                configs = list_start_configurations(nc_text, a.station, s, strict=not a.no_strict)
            except (OSError, RoboCodeError, ValueError) as exc:
                print(f"✗ Nie można wyznaczyć konfiguracji początkowych: {exc}")
                return 2
            for index, config in enumerate(configs, start=1):
                print(f"{index}: {config.label}, obrót Z={config.orientation_z_deg:.1f}°, osie="
                      f"{[round(float(value), 3) for value in config.joints]}, koszt={config.cost:.3f}")
            if not configs:
                print("✗ Brak osiągalnych konfiguracji początkowych")
                return 2
            return 0
        events = None
        if a.events:
            events = json.loads(Path(a.events).read_text(encoding="utf-8"))
            if not isinstance(events, dict) or not all(isinstance(v, str) for v in events.values()):
                print("✗ --events: oczekiwano obiektu JSON {nazwa_zdarzenia: kod_sterownika}")
                return 2
        tool_load = None
        if a.tool_load:
            try:
                load_data = json.loads(Path(a.tool_load).read_text(encoding="utf-8"))
                if not isinstance(load_data, dict):
                    raise ValueError("oczekiwano obiektu JSON")
                tool_load = ToolLoad(**load_data)
            except (OSError, json.JSONDecodeError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
                print(f"✗ --tool-load: niepoprawny plik lub dane obciążenia: {exc}")
                return 2
        return run_machining(nc_text, a.post, a.out, s, station=a.station,
                             nc_name=Path(a.nc).name, strict=not a.no_strict, event_map=events,
                             tool_load=tool_load, max_lines_per_file=a.max_lines_per_file, ros=a.ros)
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
    if a.cmd == "calibrate":
        try:
            data = json.loads(Path(a.input).read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                raise ValueError("główny element JSON musi być obiektem")
            if a.kind == "tcp":
                if set(data) != {"flange_poses"}:
                    raise ValueError("oczekiwane pole flange_poses")
                tcp_mm, rms_mm = calibrate_tcp(data["flange_poses"])
                result = {"kind": "tcp", "tcp_mm": tcp_mm.tolist(), "rms_mm": rms_mm,
                          "samples": len(data["flange_poses"])}
            elif a.kind == "bases":
                if set(data) != {"points_in_A", "points_in_B"}:
                    raise ValueError("oczekiwane pola points_in_A i points_in_B")
                points_a = np.asarray(data["points_in_A"], dtype=float)
                points_b = np.asarray(data["points_in_B"], dtype=float)
                transform = calibrate_bases(points_a, points_b)
                predicted = (transform[:3, :3] @ points_b.T).T + transform[:3, 3]
                rms_mm = float(np.sqrt(np.mean(np.sum((predicted - points_a) ** 2, axis=1))))
                result = {"kind": "bases", "T_A_B": transform.tolist(), "rms_mm": rms_mm,
                          "samples": len(points_a)}
            else:
                if set(data) != {"origin", "x_point", "xy_point"}:
                    raise ValueError("oczekiwane pola origin, x_point i xy_point")
                transform = frame_from_3_points(data["origin"], data["x_point"], data["xy_point"])
                result = {"kind": "frame3", "T_base_frame": transform.tolist()}
            output_path = atomic_write_text(a.out, json.dumps(result, indent=2, ensure_ascii=False) + "\n")
        except (OSError, json.JSONDecodeError, TypeError, ValueError) as exc:
            print(f"✗ Kalibracja {a.kind}: {exc}")
            return 2
        print(f"✓ Zapisano wynik kalibracji: {output_path}")
        if "rms_mm" in result:
            print(f"RMS: {result['rms_mm']:.6f} mm ({result['samples']} próbek)")
        return 0
    if a.cmd == "gui":
        from .gui.main_window import main as gui_main
        gui_main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
