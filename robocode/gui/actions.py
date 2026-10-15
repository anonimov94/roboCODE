"""DEKLARATYWNA definicja GUI: akcje, menu, pasek narzędzi, skróty.

Bez zależności od Qt – testowalne headless. main_window.py tylko to renderuje.
Ikony: nazwy z zestawu open source (Lucide, licencja ISC). Żadnych cudzych ikon.
`command` = nazwa handlera w robocode.gui.handlers (lub None = TODO).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Action:
    id: str
    label_pl: str
    label_en: str
    shortcut: str | None = None
    icon: str | None = None
    command: str | None = None
    checkable: bool = False
    milestone: str = "M2"


A = Action
ACTIONS: list[Action] = [
    # Plik
    A("file.new", "Nowa stacja", "New station", "Ctrl+N", "file-plus", "new_station"),
    A("file.open", "Otwórz…", "Open…", "Ctrl+O", "folder-open", "open_file"),
    A("file.library", "Biblioteka robotów", "Robot library", None, "library", "open_library"),
    A("file.demo", "Otwórz demo obróbki", "Open machining demo", None, "sparkles", "open_demo", milestone="M3"),
    A("file.recent", "Ostatni plik", "Load last file", "Ctrl+1", "history", "open_recent"),
    A("file.save", "Zapisz", "Save", "Ctrl+S", "save", "save_station"),
    A("file.save_as", "Zapisz jako…", "Save as…", "Ctrl+Shift+S", "save-all", "save_station_as"),
    A("file.export_sim", "Eksport symulacji (HTML)", "Export simulation (HTML)", None, "share-2", None, milestone="M4"),
    A("file.quit", "Zamknij", "Quit", "Ctrl+Q", None, "quit"),
    # Edycja
    A("edit.undo", "Cofnij", "Undo", "Ctrl+Z", "undo-2", "undo"),
    A("edit.redo", "Ponów", "Redo", "Ctrl+Y", "redo-2", "redo"),
    A("edit.history", "Historia zmian", "Undo history", None, "list", "show_history"),
    A("edit.cut", "Wytnij", "Cut", "Ctrl+X", "scissors", None),
    A("edit.copy", "Kopiuj", "Copy", "Ctrl+C", "copy", None),
    A("edit.paste", "Wklej", "Paste", "Ctrl+V", "clipboard-paste", None),
    A("edit.delete", "Usuń", "Delete", "Del", "trash-2", "delete_selected"),
    A("edit.rename", "Zmień nazwę", "Rename", "F2", "pencil", "rename_selected"),
    # Program
    A("prog.add_frame", "Dodaj układ odniesienia", "Add reference frame", None, "axis-3d", "add_frame"),
    A("prog.add_tool", "Dodaj puste narzędzie", "Add empty tool", None, "wrench", "add_tool"),
    A("prog.teach", "Naucz cel", "Teach target", "Ctrl+T", "crosshair", "teach_target"),
    A("prog.teach_surface", "Cele na powierzchni", "Teach targets on surface", "Ctrl+Shift+T", "scan", None, milestone="M3"),
    A("prog.add_program", "Dodaj program", "Add program", None, "list-plus", "add_program"),
    A("prog.add_macro", "Dodaj makro Python", "Add Python macro", None, "file-code", None),
    A("prog.movej", "Ruch przegubowy (MoveJ)", "Joint move", None, "spline", "add_movej"),
    A("prog.movel", "Ruch liniowy (MoveL)", "Linear move", None, "move-diagonal", "add_movel"),
    A("prog.movec", "Ruch kołowy (MoveC)", "Circular move", None, "circle-dashed", None),
    A("prog.set_frame", "Ustaw układ", "Set reference frame", None, "frame", None),
    A("prog.set_tool", "Ustaw narzędzie", "Set tool frame", None, "pen-tool", None),
    A("prog.speed", "Ustaw prędkość", "Set speed", None, "gauge", None),
    A("prog.rounding", "Ustaw zaokrąglenie", "Set rounding", None, "corner-down-right", None),
    A("prog.pause", "Pauza", "Pause", None, "pause-circle", None),
    A("prog.message", "Komunikat", "Show message", None, "message-square", None),
    A("prog.call", "Wywołanie programu", "Program call", None, "corner-right-down", None),
    A("prog.io", "Ustaw/czekaj na I/O", "Set/Wait I/O", None, "toggle-right", None),
    A("prog.sim_event", "Zdarzenie symulacji", "Simulation event", None, "sparkles", None, milestone="M4"),
    A("prog.generate", "Generuj program", "Generate program", "F6", "file-output", "generate_program"),
    A("prog.generate_as", "Generuj program jako…", "Generate program as…", "Shift+F6", "file-down", "generate_program_as"),
    A("prog.send", "Wyślij do robota", "Send program to robot", "Ctrl+F6", "send", None, milestone="M8"),
    A("prog.simulate", "Symuluj program", "Simulate program", "Ctrl+R", "play", "simulate_program", milestone="M3"),
    A("prog.fast_sim", "Szybka symulacja", "Fast simulation", "Space", "fast-forward", "toggle_fast_sim", True, "M4"),
    A("prog.pause_sim", "Pauza symulacji", "Pause simulation", "Backspace", "pause", "pause_sim", milestone="M4"),
    A("prog.stop_sim", "Stop", "Stop", "Esc", "square", "stop_sim", milestone="M4"),
    A("prog.posts", "Edytuj post-procesory", "Edit post processors", None, "settings-2", None),
    # Widok
    A("view.fit", "Dopasuj wszystko", "Fit all", "Alt+0", "maximize", "view_fit", milestone="M3"),
    A("view.path", "Pokaż ścieżkę z bliska", "Zoom to toolpath", "Alt+P", "scan-search", "view_path", milestone="M3"),
    A("view.iso", "Widok izometryczny", "Isometric view", None, "box", "view_iso", milestone="M3"),
    A("view.top", "Widok z góry", "Top view", None, "arrow-down-to-dot", "view_top", milestone="M3"),
    A("view.front", "Widok z przodu", "Front view", None, "panel-bottom", "view_front", milestone="M3"),
    A("view.side", "Widok z boku", "Side view", None, "panel-right", "view_side", milestone="M3"),
    A("view.align", "Wyrównaj obrót", "Align rotation", None, "lock", None, True, "M3"),
    A("view.text", "Pokaż/ukryj tekst", "Show/hide text", "/", "type", None, True, "M3"),
    A("view.frames_bigger", "Układy większe", "Frames bigger", "+", "zoom-in", "frames_bigger", milestone="M3"),
    A("view.frames_smaller", "Układy mniejsze", "Frames smaller", "-", "zoom-out", "frames_smaller", milestone="M3"),
    A("view.workspace", "Przestrzeń robocza", "Robot workspace", "*", "globe", "toggle_workspace", True, "M3"),
    A("view.toggle_visible", "Pokaż/ukryj zaznaczone", "Show/hide selected", "F7", "eye", "toggle_visible"),
    A("view.move_frame", "Przesuń układ (Alt)", "Move frame (Alt)", None, "move", None, True, "M3"),
    A("view.move_tcp", "Przesuń TCP (Alt+Shift)", "Move TCP (Alt+Shift)", None, "move-3d", None, True, "M3"),
    # Narzędzia
    A("tools.trace", "Ślad TCP", "Trace", None, "route", "toggle_trace", True, "M3"),
    A("tools.collisions", "Sprawdzaj kolizje", "Check collisions", None, "shield-alert",
      "toggle_collisions", True, "M5"),
    A("tools.collision_map", "Mapa kolizji", "Collision map", None, "grid-3x3", None, milestone="M5"),
    A("tools.color", "Kolor", "Change color", None, "palette", None, milestone="M3"),
    A("tools.measure", "Pomiar", "Measure", None, "ruler", None, milestone="M3"),
    A("tools.create_curves", "Twórz krzywe z geometrii", "Create curves", None, "pen-line", None, milestone="M6"),
    A("tools.language", "Język", "Language", None, "languages", "set_language"),
    A("tools.options", "Opcje…", "Options…", None, "settings", "open_options"),
    # Narzędzia CNC (Utilities)
    A("cnc.calib_tcp", "Kalibruj TCP…", "Calibrate TCP…", None, "target", "calibrate_tcp_gui", milestone="M6"),
    A("cnc.calib_frame", "Kalibruj układ…", "Calibrate reference frame…", None, "locate-fixed",
      "calibrate_frame_gui", milestone="M6"),
    A("cnc.sync_axes", "Synchronizuj osie zewnętrzne", "Synchronize external axes", None, "link", None, milestone="M7"),
    A("cnc.machining", "Projekt obróbki", "Machining project", "Ctrl+M", "drill", "new_machining_project"),
    A("cnc.update", "Aktualizuj projekt obróbki", "Update machining project", "Ctrl+U", "refresh-cw",
      "update_machining", milestone="M3"),
    A("cnc.curve_follow", "Podążanie za krzywą", "Curve follow project", None, "waypoints", None, milestone="M6"),
    A("cnc.point_follow", "Podążanie za punktami", "Point follow project", None, "map-pin", None, milestone="M6"),
    A("cnc.print3d", "Projekt druku 3D", "3D print project", None, "printer", None, milestone="M6"),
    A("cnc.import_curve", "Importuj krzywą/punkty (CSV)", "Import curve/points", None, "file-input", "import_curve"),
    A("cnc.dual_arm", "Projekt kooperatywny 2 ramion", "Dual-arm cooperative project", None, "git-merge", None, milestone="M7"),
    # Połącz
    A("conn.connect", "Połącz z robotem…", "Connect to robot…", None, "plug", None, milestone="M8"),
    A("conn.disconnect", "Rozłącz", "Disconnect", None, "unplug", None, milestone="M8"),
    A("conn.run_on_robot", "Uruchamiaj na robocie", "Run on robot", None, "play", None, True, "M8"),
    # Pomoc
    A("help.docs", "Dokumentacja", "Documentation", "F1", "help-circle", "open_help"),
    A("help.shortcuts", "Skróty klawiszowe", "Shortcuts", None, "keyboard", "show_shortcuts"),
    A("help.about", "O programie", "About", None, "info", "about"),
]

BY_ID = {a.id: a for a in ACTIONS}

MENUS: list[tuple[str, str, list[str | None]]] = [  # (id, etykieta PL, lista akcji; None = separator)
    ("file", "Plik", ["file.new", "file.open", "file.demo", "file.library", "file.recent", None, "file.save", "file.save_as",
                      None, "file.export_sim", None, "file.quit"]),
    ("edit", "Edycja", ["edit.undo", "edit.redo", "edit.history", None, "edit.cut", "edit.copy", "edit.paste",
                        "edit.delete", "edit.rename"]),
    ("program", "Program", ["prog.add_frame", "prog.add_tool", "prog.teach", "prog.teach_surface", None,
                            "prog.add_program", "prog.add_macro", None, "prog.movej", "prog.movel", "prog.movec",
                            "prog.set_frame", "prog.set_tool", "prog.speed", "prog.rounding", "prog.pause",
                            "prog.message", "prog.call", "prog.io", "prog.sim_event", None, "prog.generate",
                            "prog.generate_as", "prog.send", None, "prog.simulate", "prog.fast_sim", "prog.pause_sim",
                            "prog.stop_sim", None, "prog.posts"]),
    ("view", "Widok", ["view.fit", "view.path", "view.iso", "view.top", "view.front", "view.side", None, "view.align", "view.text",
                       "view.frames_bigger", "view.frames_smaller", "view.workspace", "view.toggle_visible"]),
    ("tools", "Narzędzia", ["tools.trace", "tools.collisions", "tools.collision_map", None, "tools.color",
                            "tools.measure", "tools.create_curves", None, "tools.language", "tools.options"]),
    ("cnc", "Narzędzia CNC", ["cnc.calib_tcp", "cnc.calib_frame", "cnc.sync_axes", None, "cnc.machining",
                              "cnc.update", "cnc.curve_follow", "cnc.point_follow", "cnc.print3d", "cnc.import_curve", None,
                              "cnc.dual_arm"]),
    ("connect", "Połącz", ["conn.connect", "conn.disconnect", "conn.run_on_robot"]),
    ("help", "Pomoc", ["help.docs", "help.shortcuts", "help.about"]),
]

TOOLBAR: list[str | None] = [
    "file.open", "file.library", "file.save", None, "edit.undo", "edit.redo", None,
    "prog.add_frame", "prog.teach", None, "view.fit", "view.iso", "view.move_frame", "view.move_tcp", None,
    "tools.collisions", "prog.simulate", "prog.fast_sim", "prog.pause_sim", None, "cnc.machining", "cnc.update",
    None,
    "prog.add_program", "prog.add_macro", "prog.movej", "prog.movel", None, "file.export_sim",
]

# Panele dokowane (M2/M6)
PANELS = {
    "station_tree": {"area": "left", "title": "Stacja"},
    "properties": {"area": "right", "title": "Właściwości"},
    "robot": {"area": "right", "title": "Robot", "sections": ["jog_joints", "limits", "tcp_vs_flange",
                                                                "frame_vs_base", "tcp_vs_frame", "configurations",
                                                                "parameters"]},
    "machining": {"area": "right", "title": "Projekt obróbki",
                  "sections": ["robot_frame_tool", "path_input", "program_events", "approach_retract",
                               "path_to_tool_offset", "algorithm", "optimization", "start_config",
                               "update_simulate"]},
    "sim_bar": {"area": "bottom", "title": "Symulacja"},
    "status": {"area": "bottom", "title": "Status"},
}


def validate() -> list[str]:
    """Spójność definicji GUI (używane w testach)."""
    errs = []
    ids = [a.id for a in ACTIONS]
    if len(ids) != len(set(ids)):
        errs.append("Zduplikowane id akcji")
    shortcuts = [a.shortcut for a in ACTIONS if a.shortcut]
    dup = {s for s in shortcuts if shortcuts.count(s) > 1}
    if dup:
        errs.append(f"Zduplikowane skróty: {dup}")
    for _, _, items in MENUS:
        errs += [f"Menu: nieznana akcja {i}" for i in items if i and i not in BY_ID]
    errs += [f"Toolbar: nieznana akcja {i}" for i in TOOLBAR if i and i not in BY_ID]
    return errs
