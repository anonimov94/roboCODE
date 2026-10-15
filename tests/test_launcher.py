"""Launcher menu (the code behind roboCODE.exe) – non-interactive checks."""
import builtins

from robocode import launcher


def test_menu_demo_report_samples_exit(tmp_path, monkeypatch):
    monkeypatch.setattr(launcher, "base_dir", lambda: tmp_path)
    monkeypatch.setattr(launcher.webbrowser, "open", lambda *a, **k: True)
    answers = iter(["abb"])
    monkeypatch.setattr(builtins, "input", lambda *_: next(answers))
    assert launcher.menu_once("1") == 0
    assert (tmp_path / "out" / "Machining.mod").exists()
    assert launcher.menu_once("3") == 0 and (tmp_path / "out" / "Machining.manifest.html").exists()
    assert launcher.menu_once("4") == 0 and len(list((tmp_path / "przyklady_nc").glob("*.nc"))) == 7
    assert launcher.menu_once("0") is None
    assert launcher.menu_once("x") == 1


def test_run_nc_missing_and_cp1250(tmp_path, monkeypatch):
    monkeypatch.setattr(launcher, "base_dir", lambda: tmp_path)
    assert launcher.run_nc(str(tmp_path / "nope.nc"), "kuka") == 1
    f = tmp_path / "pl.nc"
    f.write_bytes("(frezowanie żółć)\nG21\nG1 X0 Y0 Z0 F300\nG1 X10\n".encode("cp1250"))
    assert launcher.run_nc(str(f), "kuka") == 0


def test_cli_passthrough(tmp_path):
    assert launcher.main(["demo", "--out", str(tmp_path)]) == 0
    assert (tmp_path / "Machining.src").exists()


def test_redirected_cp1250_output_does_not_crash(tmp_path, monkeypatch):
    """Regression (found on Windows): exe output redirected to a file used cp1250 -> crash on ✓."""
    import io
    import sys

    from robocode import cli
    buf = io.BytesIO()
    fake = io.TextIOWrapper(buf, encoding="cp1250")
    monkeypatch.setattr(sys, "stdout", fake)
    assert cli.main(["demo", "--out", str(tmp_path)]) == 0
    fake.flush()
    assert "✓ Zapisano".encode("utf-8") in buf.getvalue()
    bad = tmp_path / "bad.nc"
    bad.write_text("G21\nG41 D1\nG1 X10 F100\n")
    assert cli.main(["machine", str(bad), "--out", str(tmp_path / "b")]) == 2
