"""Console output that never crashes on Windows code pages (cp1250/cp852).

Redirected output (file, pipe, .bat `>> log`) on Windows defaults to the ANSI code page, which cannot
encode ✓ ✗ ⚠ → UnicodeEncodeError. Found by running roboCODE.exe on Windows (test_exe.bat).
"""
from __future__ import annotations

import sys


def setup_console() -> None:
    """Redirected streams -> UTF-8; interactive ones keep their encoding but never raise on a character."""
    for stream in (sys.stdout, sys.stderr):
        reconf = getattr(stream, "reconfigure", None)
        if reconf is None:
            continue
        try:
            if not stream.isatty():
                reconf(encoding="utf-8", errors="replace")
            else:
                reconf(errors="replace")
        except (ValueError, OSError):
            pass
