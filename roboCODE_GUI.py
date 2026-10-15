"""Entry point for roboCODE-GUI.exe (okno, bez konsoli) – cel PyInstallera w build_exe.bat.

Dwuklik roboCODE-GUI.exe otwiera od razu okno GUI. Gdy coś pójdzie nie tak, błąd trafia do
`out/gui_error.log` obok exe i jest pokazany w okienku (exe nie ma konsoli, więc inaczej byłby niewidoczny).
`--smoke` buduje okno i zamyka je po chwili (test budowania exe, bez interakcji).
"""
from __future__ import annotations

import os
import sys
import traceback
from pathlib import Path

os.environ.setdefault("QT_API", "pyside6")  # pyvistaqt -> qtpy: zawsze PySide6 z paczki exe


def _base_dir() -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent.parent  # roboCODE-GUI\roboCODE-GUI.exe -> projekt
    return Path(__file__).resolve().parent


def _report_crash(exc: BaseException, show: bool = True) -> None:
    text = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
    log = _base_dir() / "out" / "gui_error.log"
    try:
        log.parent.mkdir(parents=True, exist_ok=True)
        log.write_text(text, encoding="utf-8")
    except OSError:
        log = None
    if sys.stderr:
        sys.stderr.write(text)
    if not show:  # --smoke: bez okienka (platforma offscreen – nikt by go nie zamknął)
        return
    try:
        from PySide6.QtWidgets import QApplication, QMessageBox
        app = QApplication.instance() or QApplication(sys.argv)  # noqa: F841 - wymagane przez QMessageBox
        QMessageBox.critical(None, "roboCODE – błąd GUI",
                             f"{type(exc).__name__}: {exc}\n\nSzczegóły: {log or text[-1500:]}")
    except Exception:  # noqa: BLE001 - nie da się nic więcej pokazać
        pass


def preflight_3d() -> None:
    """Importuje widok 3D jawnie (to samo, co robi pyvistaqt), żeby błąd pokazał prawdziwą przyczynę.

    pyvista ładuje `pyvista.plotting` leniwie przez `__getattr__`; gdy w paczce exe czegoś brakuje,
    Python zgłasza mylące „cannot import name 'global_theme'” zamiast właściwego błędu VTK.
    """
    import pyvista.plotting  # noqa: F401
    from pyvista.plotting import global_theme  # noqa: F401
    from pyvistaqt import QtInteractor  # noqa: F401


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    try:
        preflight_3d()
    except Exception as e:  # noqa: BLE001 - pokazujemy prawdziwą przyczynę; okno i tak się otworzy
        _report_crash(e, show="--smoke" not in argv)
        if "--smoke" in argv:
            return 1
    try:
        from robocode.gui.main_window import build_window
        app, win = build_window()
        win.show()
        if "--smoke" in argv:
            from PySide6.QtCore import QTimer
            QTimer.singleShot(1500, app.quit)
        code = app.exec()
        win.viewport.close()
        if "--smoke" in argv:
            print(f"SMOKE OK viewport3d={win.viewport.available}")
        return int(code)
    except SystemExit as e:
        return int(e.code or 0)
    except BaseException as e:  # noqa: BLE001 - każdy błąd pokazujemy użytkownikowi
        _report_crash(e, show="--smoke" not in argv)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
