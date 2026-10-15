"""roboCODE launcher: double-click entry point (also the target of build_exe.bat -> roboCODE.exe).

Console menu in Polish. NC files are picked with a standard file dialog (tkinter) when available,
otherwise typed. All results go to an `out` folder next to the executable (or the project root).
"""
from __future__ import annotations

if __name__ == "__main__" and not __package__:
    import pathlib as _pl
    import sys as _sys
    _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[1]))
    from robocode.launcher import main as _main
    raise SystemExit(_main())

import sys
import webbrowser
from pathlib import Path

from . import __version__


def base_dir() -> Path:
    """Folder of the .exe when frozen (PyInstaller), else the project root."""
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parents[1]


def out_dir() -> Path:
    d = base_dir() / "out"
    d.mkdir(parents=True, exist_ok=True)
    return d


def pick_nc_file() -> str | None:
    """File dialog for an NC file; falls back to typed input (e.g. no display)."""
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        path = filedialog.askopenfilename(
            title="Wybierz plik NC (G-code)",
            filetypes=[("Pliki NC", "*.nc *.ngc *.gcode *.tap *.cnc *.txt"), ("Wszystkie pliki", "*.*")])
        root.destroy()
        return path or None
    except Exception:  # noqa: BLE001 - any GUI problem -> plain prompt
        p = input("Ścieżka do pliku NC: ").strip().strip('"')
        return p or None


def ask(prompt: str, choices: list[str], default: str) -> str:
    while True:
        v = input(f"{prompt} [{'/'.join(choices)}] (Enter = {default}): ").strip().lower() or default
        if v in choices:
            return v
        print("  Nieprawidłowy wybór.")


def run_demo(post: str = "kuka") -> int:
    from .cli import run_machining
    from .demo import DEMO_NC
    return run_machining(DEMO_NC, post, str(out_dir()), nc_name="demo")


def run_nc(path: str, post: str) -> int:
    from .cli import run_machining
    p = Path(path)
    if not p.is_file():
        print(f"✗ Nie ma takiego pliku: {p}")
        return 1
    try:
        text = p.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        text = p.read_text(encoding="cp1250")  # common for Polish Windows CAM exports
    return run_machining(text, post, str(out_dir()), nc_name=p.name)


def open_report() -> int:
    from .tools.report_html import write_report
    manifest = out_dir() / "Machining.manifest.json"
    if not manifest.is_file():
        print("✗ Brak manifestu – najpierw uruchom demo albo obróbkę pliku NC.")
        return 1
    html = write_report(manifest)
    print(f"Raport: {html}")
    webbrowser.open(html.as_uri())
    return 0


def write_samples() -> int:
    from .tools.sample_nc import write_samples as ws
    folder = base_dir() / "przyklady_nc"
    for p in ws(folder):
        print("  ", p.name)
    print(f"Zapisano w: {folder}")
    return 0


def venv_python() -> Path | None:
    """Python of the project's .venv (Windows or POSIX layout), if present."""
    root = base_dir() / ".venv"
    for cand in (root / "Scripts" / "python.exe", root / "bin" / "python"):
        if cand.is_file():
            return cand
    return None


def _gui_libs_available() -> bool:
    try:
        import PySide6  # noqa: F401
        import pyvistaqt  # noqa: F401
    except ImportError:
        return False
    return True


def gui_exe() -> Path | None:
    """roboCODE-GUI.exe zbudowany przez build_exe.bat (folder roboCODE-GUI obok roboCODE.exe)."""
    for cand in (base_dir() / "roboCODE-GUI" / "roboCODE-GUI.exe", base_dir() / "roboCODE-GUI" / "roboCODE-GUI"):
        if cand.is_file():
            return cand
    return None


def _venv_has_gui(py: Path) -> bool:
    import subprocess
    check = subprocess.run([str(py), "-c", "import PySide6, pyvistaqt, robocode"], cwd=base_dir(),
                           capture_output=True, timeout=120)
    return check.returncode == 0


def _install_gui_into_venv(py: Path) -> bool:
    """Doinstalowuje PySide6/pyvista do .venv projektu (jednorazowo, bez udziału użytkownika)."""
    import subprocess
    if not (base_dir() / "pyproject.toml").is_file():
        return False
    print("Instaluję biblioteki GUI (PySide6, pyvista) – jednorazowo, może potrwać kilka minut…")
    r = subprocess.run([str(py), "-m", "pip", "install", "-e", ".[gui]"], cwd=base_dir())
    return r.returncode == 0


def _start_detached(args: list[str]) -> None:
    import subprocess
    flags = getattr(subprocess, "DETACHED_PROCESS", 0) | getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
    subprocess.Popen(args, cwd=base_dir(), creationflags=flags)


def run_gui() -> int:
    """Otwiera okno GUI: roboCODE-GUI.exe → biblioteki w tym procesie → .venv projektu (doinstaluje GUI)."""
    exe = gui_exe()
    if exe is not None:
        _start_detached([str(exe)])
        print("✓ GUI uruchomione w osobnym oknie")
        return 0
    if _gui_libs_available():
        from .gui.main_window import main as gui_main
        try:
            gui_main()
        except SystemExit:
            pass  # closing the window must not close the launcher menu
        return 0
    py = venv_python()
    if py is not None and Path(sys.executable).resolve() != py.resolve():
        if _venv_has_gui(py) or (_install_gui_into_venv(py) and _venv_has_gui(py)):
            pyw = py.with_name("pythonw.exe")
            _start_detached([str(pyw if pyw.is_file() else py), "-m", "robocode.cli", "gui"])
            print(f"✓ GUI uruchomione w osobnym oknie (Python z {py.parent.parent})")
            return 0
    print("✗ Nie znaleziono GUI (roboCODE-GUI.exe) ani bibliotek PySide6/pyvista.")
    print("  Uruchom dwuklikiem build_exe.bat – zbuduje roboCODE.exe i roboCODE-GUI.exe.")
    return 1


MENU = [
    ("1", "Demo: G-code → program robota (KUKA/ABB)"),
    ("2", "Obróbka własnego pliku NC…"),
    ("3", "Otwórz raport HTML ostatniego przebiegu"),
    ("4", "Zapisz przykładowe pliki NC"),
    ("5", "Uruchom GUI"),
    ("6", "Otwórz folder wyników"),
    ("0", "Wyjście"),
]


def menu_once(choice: str) -> int | None:
    """Execute one menu entry. Returns None for 'exit'."""
    if choice == "0":
        return None
    if choice == "1":
        return run_demo(ask("Post-procesor", ["kuka", "abb"], "kuka"))
    if choice == "2":
        path = pick_nc_file()
        if not path:
            print("Anulowano.")
            return 1
        return run_nc(path, ask("Post-procesor", ["kuka", "abb"], "kuka"))
    if choice == "3":
        return open_report()
    if choice == "4":
        return write_samples()
    if choice == "5":
        return run_gui()
    if choice == "6":
        webbrowser.open(out_dir().as_uri())
        return 0
    print("  Nieprawidłowy wybór.")
    return 1


def main(argv: list[str] | None = None) -> int:
    from ._console import setup_console
    setup_console()
    argv = sys.argv[1:] if argv is None else argv
    if argv:  # roboCODE.exe machine plik.nc --post abb  -> same as the CLI
        from .cli import main as cli_main
        return cli_main(argv)
    print(f"\n=== roboCODE {__version__} – programowanie robotów offline ===")
    print(f"Wyniki: {out_dir()}")
    while True:
        print()
        for k, label in MENU:
            print(f"  {k}. {label}")
        try:
            choice = input("Wybór: ").strip()
        except (EOFError, KeyboardInterrupt):
            return 0
        try:
            if menu_once(choice) is None:
                return 0
        except KeyboardInterrupt:
            print("\nPrzerwano.")
        except Exception as e:  # noqa: BLE001 - never close the window on an unexpected error
            print(f"✗ Nieoczekiwany błąd: {type(e).__name__}: {e}")


if __name__ == "__main__":
    raise SystemExit(main())
