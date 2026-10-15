"""Handlery akcji GUI – logika bez Qt. Każdy zwraca komunikat dla paska stanu."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import json
import numpy as np

from ..core import commands as C
from ..core.io import load_station, save_station
from ..core.scene import Frame, Item, Robot, Station, Tool
from ..coop.calibration import calibrate_bases, calibrate_tcp, frame_from_3_points
from ..program.instructions import MoveJ, MoveL, Program
from ..program.report import Severity, ValidationReport


@dataclass
class AppState:
    station: Station = field(default_factory=lambda: Station(name="Stacja 1"))
    undo: C.UndoStack = field(default_factory=C.UndoStack)
    selection: list[Item] = field(default_factory=list)
    path: Path | None = None
    fast_sim: bool = False
    overlay: object = None            # render.Overlay – ścieżka obróbki, ślad
    render_opt: object = None         # render.RenderOptions
    project: object = None            # demo.MachiningProject
    view_request: str | None = None   # GUI ustawia kamerę po akcji ("iso", "top", "path"…)
    trace_on: bool = True
    last_sim: object = None
    last_report: object = None  # ValidationReport z ostatniego eksportu (F6)
    pending_calibration: tuple[Item, np.ndarray, str] | None = None

    def __post_init__(self):
        from .render import RenderOptions
        if self.render_opt is None:
            self.render_opt = RenderOptions()

    def robot(self) -> Robot | None:
        return next((i for i in self.station.walk() if isinstance(i, Robot)), None)


def new_station(s: AppState) -> str:
    s.station, s.undo, s.selection, s.path = Station(name="Stacja 1"), C.UndoStack(), [], None
    s.project, s.last_report, s.overlay, s.pending_calibration = None, None, None, None
    return "Nowa stacja"


def save_station_as(s: AppState, path: str | None = None) -> str:
    if path:
        s.path = Path(path)
    if not s.path:
        return "Podaj ścieżkę pliku"
    save_station(s.station, s.path)
    return f"Zapisano {s.path.name}"


def save_station_(s: AppState) -> str:
    return save_station_as(s)


def open_file(s: AppState, path: str | None = None) -> str:
    if not path:
        return "Wybierz plik"
    s.station, s.path = load_station(path), Path(path)
    s.undo = C.UndoStack()
    s.project, s.last_report, s.overlay, s.pending_calibration = None, None, None, None
    return f"Otwarto {Path(path).name}"


def undo(s: AppState) -> str:
    return "Cofnięto" if s.undo.undo() else "Nic do cofnięcia"


def redo(s: AppState) -> str:
    return "Ponowiono" if s.undo.redo() else "Nic do ponowienia"


def add_frame(s: AppState) -> str:
    parent = s.selection[0] if s.selection and isinstance(s.selection[0], (Frame, Station, Robot)) else s.station
    f = Frame(name=f"Układ {sum(isinstance(i, Frame) for i in s.station.walk()) + 1}")
    s.undo.push(C.add_item(parent, f, "Dodaj układ"))
    return f"Dodano {f.name}"


def add_tool(s: AppState) -> str:
    r = s.robot()
    if not r:
        return "Brak robota w stacji"
    t = Tool(name=f"Narzędzie {len(r.tools()) + 1}")
    s.undo.push(C.add_item(r, t, "Dodaj narzędzie"))
    r.active_tool = r.active_tool or t
    return f"Dodano {t.name}"


def teach_target(s: AppState) -> str:
    r = s.robot()
    if not r:
        return "Brak robota w stacji"
    t = r.teach_target(f"Cel {sum(1 for i in s.station.walk() if i.kind == 'target') + 1}")
    return f"Nauczono {t.name}"


def add_program(s: AppState) -> str:
    r = s.robot()
    p = Program(name=f"Prog{sum(1 for i in s.station.walk() if i.kind == 'program') + 1}", robot=r)
    s.undo.push(C.add_item(s.station, p, "Dodaj program"))
    s.selection = [p]
    return f"Dodano {p.name}"


def _current_program(s: AppState) -> Program | None:
    return next((i for i in s.selection if isinstance(i, Program)), None) or \
        next((i for i in s.station.walk() if isinstance(i, Program)), None)


def add_movej(s: AppState) -> str:
    p = _current_program(s) or (add_program(s) and _current_program(s))
    r = p.robot
    p.append(MoveJ(joints=r.joints.copy()))
    return "Dodano MoveJ"


def add_movel(s: AppState) -> str:
    p = _current_program(s) or (add_program(s) and _current_program(s))
    r = p.robot
    tcp = r.active_tool.tcp() if r.active_tool else None
    import numpy as np
    pose = np.linalg.inv(r.frame_rel_base(r.active_frame)) @ r.model.fk(r.joints) @ (tcp if tcp is not None else np.eye(4))
    p.append(MoveL(pose=pose, joints=r.joints.copy()))
    return "Dodano MoveL"


def rename_selected(s: AppState, name: str | None = None) -> str:
    if not s.selection or not name:
        return "Zaznacz element"
    s.undo.push(C.rename(s.selection[0], name))
    return f"Zmieniono nazwę na {name}"


def toggle_visible(s: AppState) -> str:
    for it in s.selection:
        s.undo.push(C.set_attr(it, "visible", not it.visible, "Pokaż/ukryj"))
    return "Przełączono widoczność"


def generate_program(s: AppState, folder: str = "programs") -> str:
    """F6: always through validation + read-back check; never writes an unvalidated file."""
    from ..pipeline import export_program
    p = s.project.result.program if s.project and s.project.result and s.project.result.ok \
        else _current_program(s)
    if not p:
        return "Brak programu"
    job = export_program(p, p.robot.post_processor, folder,
                         parser_warnings=s.project.toolpath.warnings if s.project else None)
    s.last_report = job.report
    if not job.ok:
        first = job.report.errors[0].text() if job.report.errors else ""
        return f"{job.message}: {first}"
    return f"{job.message} – {job.outputs[0]}"


def _update_machining_report(s: AppState) -> None:
    if s.project is None or s.project.result is None:
        s.last_report = None
        return
    result = s.project.result
    if result.ok and result.program is not None:
        from ..program.validator import validate_program
        s.last_report, _ = validate_program(result.program, parser_warnings=s.project.toolpath.warnings)
        return

    report = ValidationReport()
    if "Nieprawidłowy wybór konfiguracji początkowej" in result.message:
        code = "START_CONFIGURATION"
    else:
        code = "CONFIG_JUMP" if "skoku konfiguracji" in result.message else "UNREACHABLE"
    unreachable = [index for index, reachable in enumerate(result.reachable) if not reachable]
    if unreachable and result.toolpath is not None:
        for index in unreachable[:20]:
            point = result.toolpath.points[index]
            report.add(Severity.ERROR, code, f"Punkt ścieżki {index}: nieosiągalny",
                       stage="solve", source_line=point.line,
                       hint="przesuń układ detalu lub zmień konfigurację początkową")
    else:
        report.add(Severity.ERROR, code, result.message, stage="solve")
    s.last_report = report


def _set_machining_error(s: AppState, code: str, message: str, stage: str = "solve",
                         source_line: int | None = None) -> None:
    report = ValidationReport()
    report.add(Severity.ERROR, code, message, stage=stage, source_line=source_line)
    s.last_report = report


def _owning_robot(item: Item | None) -> Robot | None:
    parent = item
    while parent is not None:
        if isinstance(parent, Robot):
            return parent
        parent = parent.parent
    return None


def calibrate_tcp_gui(s: AppState, path: str | None = None) -> str:
    s.pending_calibration = None
    if not path:
        return "Wybierz plik pomiarów TCP"
    selected_tool = next((item for item in s.selection if isinstance(item, Tool)), None)
    robot = _owning_robot(selected_tool) if selected_tool else s.robot()
    tool = selected_tool or (robot.active_tool if robot else None)
    if robot is None or tool is None:
        return "Kalibracja TCP wymaga robota i wybranego lub aktywnego narzędzia"
    if tool.parent is not robot:
        return "Kalibracja TCP wymaga narzędzia bezpośrednio pod kołnierzem robota"
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(data, dict) or set(data) != {"flange_poses"}:
            raise ValueError("oczekiwano obiektu JSON z polem flange_poses")
        tcp_mm, rms_mm = calibrate_tcp(data["flange_poses"])
    except (OSError, json.JSONDecodeError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
        return f"✗ Kalibracja TCP: {exc}"
    new_pose = tool.pose.copy()
    new_pose[:3, 3] = tcp_mm
    s.pending_calibration = (tool, new_pose, f"TCP {tool.name}: RMS {rms_mm:.4f} mm")
    return f"Kalibracja TCP obliczona dla {tool.name}; RMS {rms_mm:.4f} mm"


def calibrate_frame_gui(s: AppState, path: str | None = None) -> str:
    s.pending_calibration = None
    if not path:
        return "Wybierz plik pomiarów układu"
    selected_frame = next((item for item in s.selection if isinstance(item, Frame)), None)
    if selected_frame is not None:
        frame = selected_frame
        robot = _owning_robot(frame) or next(
            (item for item in s.station.walk()
             if isinstance(item, Robot) and item.active_frame is frame), None
        )
    else:
        robot = s.robot()
        frame = robot.active_frame if robot else None
    if robot is None or frame is None:
        return "Kalibracja układu wymaga robota i wybranego lub aktywnego układu"
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("główny element JSON musi być obiektem")
        if set(data) == {"origin", "x_point", "xy_point"}:
            frame_in_base = frame_from_3_points(data["origin"], data["x_point"], data["xy_point"])
            quality = "metoda 3 punktów"
        elif set(data) == {"points_in_A", "points_in_B"}:
            frame_in_base = calibrate_bases(data["points_in_A"], data["points_in_B"])
            points_a = np.asarray(data["points_in_A"], dtype=float)
            points_b = np.asarray(data["points_in_B"], dtype=float)
            predicted = (frame_in_base[:3, :3] @ points_b.T).T + frame_in_base[:3, 3]
            rms_mm = float(np.sqrt(np.mean(np.sum((predicted - points_a) ** 2, axis=1))))
            quality = f"RMS {rms_mm:.4f} mm"
        else:
            raise ValueError("oczekiwano pól origin/x_point/xy_point albo points_in_A/points_in_B")
        if frame.parent is None:
            raise ValueError("układ musi należeć do drzewa stacji")
        parent_in_base = robot.frame_rel_base(frame.parent)
        new_pose = np.linalg.inv(parent_in_base) @ frame_in_base
    except (OSError, json.JSONDecodeError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
        return f"✗ Kalibracja układu: {exc}"
    s.pending_calibration = (frame, new_pose, f"Układ {frame.name} ({quality})")
    return f"Kalibracja układu obliczona dla {frame.name} ({quality})"


def apply_pending_calibration(s: AppState) -> str:
    if s.pending_calibration is None:
        return "Brak oczekującej kalibracji"
    item, pose, description = s.pending_calibration
    if not any(candidate is item for candidate in s.station.walk()):
        s.pending_calibration = None
        return "Element kalibracji nie należy już do bieżącej stacji"
    s.undo.push(C.set_attr(item, "pose", pose, f"Kalibracja: {item.name}"))
    s.pending_calibration = None
    if s.project is not None:
        s.project.result = None
        s.overlay = None
        s.last_report = None
    return f"Zastosowano {description}; wynik projektu obróbki unieważniony, użyj Ctrl+U"


def cancel_pending_calibration(s: AppState) -> None:
    s.pending_calibration = None


def toggle_fast_sim(s: AppState) -> str:
    s.fast_sim = not s.fast_sim
    return f"Szybka symulacja: {'wł.' if s.fast_sim else 'wył.'}"


# ---------------- widok 3D (M3) ----------------

def _auto_frame_size(s: AppState, which: str) -> None:
    """Zbliżenie na ścieżkę -> osie w skali detalu; widok ogólny -> rozmiar domyślny."""
    import numpy as np
    if which == "path" and s.overlay is not None and s.overlay.path_points is not None \
            and len(s.overlay.path_points):
        P = np.asarray(s.overlay.path_points)
        ext = float((P.max(0) - P.min(0)).max())
        s.render_opt.frame_size = float(np.clip(ext * 0.25, 10.0, 80.0))
    elif which in ("iso", "fit", "top", "front", "side"):
        s.render_opt.frame_size = 80.0


def _view(which: str):
    def h(s: AppState) -> str:
        _auto_frame_size(s, which)
        s.view_request = which
        return {"iso": "Widok izometryczny", "top": "Widok z góry", "front": "Widok z przodu",
                "side": "Widok z boku", "fit": "Dopasowano widok", "path": "Widok na ścieżkę"}[which]
    return h


def frames_bigger(s: AppState) -> str:
    s.render_opt.frame_size *= 1.5
    return f"Rozmiar układów: {s.render_opt.frame_size:.0f} mm"


def frames_smaller(s: AppState) -> str:
    s.render_opt.frame_size = max(5.0, s.render_opt.frame_size / 1.5)
    return f"Rozmiar układów: {s.render_opt.frame_size:.0f} mm"


def toggle_workspace(s: AppState) -> str:
    s.render_opt.show_workspace = not s.render_opt.show_workspace
    return f"Przestrzeń robocza: {'wł.' if s.render_opt.show_workspace else 'wył.'}"


def toggle_trace(s: AppState) -> str:
    s.trace_on = not s.trace_on
    if s.overlay is not None and not s.trace_on:
        s.overlay.trace = None
    return f"Ślad TCP: {'wł.' if s.trace_on else 'wył.'}"


# ---------------- demo / obróbka / symulacja ----------------

def open_demo(s: AppState) -> str:
    from ..demo import demo_project
    st, proj = demo_project()
    s.station, s.project, s.undo, s.selection, s.path = st, proj, C.UndoStack(), [], None
    s.overlay = proj.overlay()
    s.view_request = "iso"
    _update_machining_report(s)
    return f"Demo obróbki: {proj.result.message}"


def new_machining_project(s: AppState, nc_path: str | None = None) -> str:
    """Ctrl+M: plik NC -> projekt obróbki na pierwszym robocie stacji."""
    from ..demo import MachiningProject, build_demo_station
    from ..machining.nc_parser import parse_gcode
    from ..machining.solver import MachiningSettings
    if not nc_path:
        return "Wybierz plik NC (G-code)"
    s.project, s.last_report, s.overlay = None, None, None
    r = s.robot()
    if r is None:  # pusta stacja -> użyj stacji demonstracyjnej jako komórki
        s.station, r, *_ = build_demo_station()
    fr = r.active_frame or next((i for i in s.station.walk() if isinstance(i, Frame)), None)
    tool = r.active_tool or (r.tools()[0] if r.tools() else None)
    if fr is None or tool is None:
        _set_machining_error(s, "PROJECT_SETUP", "Projekt obróbki wymaga układu detalu i narzędzia na robocie",
                             stage="static")
        return "Projekt obróbki wymaga układu detalu i narzędzia na robocie"
    from ..errors import RoboCodeError
    try:
        tp = parse_gcode(Path(nc_path).read_text(encoding="utf-8", errors="replace"))
    except RoboCodeError as e:
        _set_machining_error(s, "NC_PARSE", str(e), stage="parse", source_line=getattr(e, "line_no", None))
        return f"✗ Plik NC: {e}"
    if len(tp) == 0:
        _set_machining_error(s, "EMPTY", "Plik NC nie zawiera ruchów", stage="parse")
        return "Plik NC nie zawiera ruchów"
    s.project = MachiningProject(r, fr, tool, tp, MachiningSettings())
    res = s.project.update()
    s.overlay = s.project.overlay()
    _update_machining_report(s)
    s.view_request = "path"
    _auto_frame_size(s, "path")
    return res.message


def update_machining(s: AppState) -> str:
    if s.project is None:
        return "Brak projektu obróbki (Ctrl+M)"
    res = s.project.update()
    s.overlay = s.project.overlay()
    _update_machining_report(s)
    return res.message


def simulate_program(s: AppState) -> str:
    """Symulacja (bez animacji – M4): liczy przebieg, ślad TCP i czas cyklu."""
    from ..program.simulator import simulate
    prog = s.project.result.program if s.project and s.project.result and s.project.result.ok \
        else _current_program(s)
    if prog is None:
        return "Brak programu do symulacji"
    q0 = prog.robot.joints.copy()
    sim = simulate(prog)
    s.last_sim = sim
    if s.trace_on:
        if s.overlay is None:
            from .render import Overlay
            s.overlay = Overlay()
        s.overlay.trace = sim.trace
    if not sim.ok:
        prog.robot.joints = q0
        return f"✗ Symulacja: {sim.errors[0]}"
    w = f", ostrzeżenia: {len(sim.warnings)}" if sim.warnings else ""
    return f"✓ Symulacja OK – czas cyklu ≈ {sim.cycle_time_s:.1f} s{w}"


HANDLERS = {
    "new_station": new_station, "save_station": save_station_, "save_station_as": save_station_as,
    "open_file": open_file, "undo": undo, "redo": redo, "add_frame": add_frame, "add_tool": add_tool,
    "teach_target": teach_target, "add_program": add_program, "add_movej": add_movej, "add_movel": add_movel,
    "rename_selected": rename_selected, "toggle_visible": toggle_visible, "generate_program": generate_program,
    "toggle_fast_sim": toggle_fast_sim,
    "view_iso": _view("iso"), "view_top": _view("top"), "view_front": _view("front"), "view_side": _view("side"),
    "view_fit": _view("fit"), "view_path": _view("path"), "frames_bigger": frames_bigger,
    "frames_smaller": frames_smaller, "toggle_workspace": toggle_workspace, "toggle_trace": toggle_trace,
    "open_demo": open_demo, "new_machining_project": new_machining_project, "update_machining": update_machining,
    "simulate_program": simulate_program,
    "calibrate_tcp_gui": calibrate_tcp_gui, "calibrate_frame_gui": calibrate_frame_gui,
    "apply_pending_calibration": apply_pending_calibration,
}

# komendy, które w GUI wymagają wyboru pliku: (tytuł, filtr, tryb)
NEEDS_PATH = {
    "open_file": ("Otwórz stację", "Stacja roboCODE (*.rcs.json)", "open"),
    "save_station_as": ("Zapisz stację", "Stacja roboCODE (*.rcs.json)", "save"),
    "new_machining_project": ("Plik NC", "G-code (*.nc *.gcode *.ngc *.tap *.txt);;Wszystkie (*)", "open"),
    "calibrate_tcp_gui": ("Pomiary TCP", "JSON (*.json);;Wszystkie (*)", "open"),
    "calibrate_frame_gui": ("Pomiary układu", "JSON (*.json);;Wszystkie (*)", "open"),
}


def dispatch(state: AppState, command: str | None, *args) -> str:
    if command is None or command not in HANDLERS:
        return "Funkcja jeszcze niezaimplementowana (zob. README, milestones)"
    return HANDLERS[command](state, *args)
