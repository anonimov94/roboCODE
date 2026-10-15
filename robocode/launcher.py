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


def run_gui() -> int:
    try:
        from .gui.main_window import main as gui_main
    except ImportError as e:
        print(f"✗ GUI niedostępne ({e}). Zainstaluj: pip install -e .[gui]")
        return 1
    gui_main()
    return 0


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
