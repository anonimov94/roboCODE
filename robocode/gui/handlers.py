"""Handlery akcji GUI – logika bez Qt. Każdy zwraca komunikat dla paska stanu."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from ..core import commands as C
from ..core.io import load_station, save_station
from ..core.scene import Frame, Item, Robot, Station, Tool
from ..program.instructions import MoveJ, MoveL, Program


@dataclass
class AppState:
    station: Station = field(default_factory=lambda: Station(name="Stacja 1"))
    undo: C.UndoStack = field(default_factory=C.UndoStack)
    selection: list[Item] = field(default_factory=list)
    path: Path | None = None
    fast_sim: bool = False

    def robot(self) -> Robot | None:
        return next((i for i in self.station.walk() if isinstance(i, Robot)), None)


def new_station(s: AppState) -> str:
    s.station, s.undo, s.selection, s.path = Station(name="Stacja 1"), C.UndoStack(), [], None
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
    from ..post.base import get_post
    p = _current_program(s)
    if not p:
        return "Brak programu"
    files = get_post(p.robot.post_processor).save(p, folder)
    return f"Wygenerowano {files[0]}"


def toggle_fast_sim(s: AppState) -> str:
    s.fast_sim = not s.fast_sim
    return f"Szybka symulacja: {'wł.' if s.fast_sim else 'wył.'}"


HANDLERS = {
    "new_station": new_station, "save_station": save_station_, "save_station_as": save_station_as,
    "open_file": open_file, "undo": undo, "redo": redo, "add_frame": add_frame, "add_tool": add_tool,
    "teach_target": teach_target, "add_program": add_program, "add_movej": add_movej, "add_movel": add_movel,
    "rename_selected": rename_selected, "toggle_visible": toggle_visible, "generate_program": generate_program,
    "toggle_fast_sim": toggle_fast_sim,
}


def dispatch(state: AppState, command: str | None, *args) -> str:
    if command is None or command not in HANDLERS:
        return "Funkcja jeszcze niezaimplementowana (zob. README, milestones)"
    return HANDLERS[command](state, *args)
