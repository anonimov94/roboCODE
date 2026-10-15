"""Poprawki zgodności Qt/PySide6 stosowane przed zbudowaniem okna (exe i uruchomienie z .venv)."""
from __future__ import annotations

import sys


def _patch() -> None:
    """Python 3.12.0 + PySide6 w exe: hak importu shibokena wywraca import `six.moves` (dateutil → matplotlib →
    pyvista) błędem „_SixMetaPathImporter has no attribute _path”. Skutek w GUI: mylące „cannot import name
    'global_theme' from 'pyvista'” i brak widoku 3D. Hak tylko sprawdza, czy moduł używa PySide, więc
    błąd przy tym sprawdzeniu traktujemy jako „nie używa”.
    """
    import shiboken6  # noqa: F401 - instaluje shibokensupport
    feature = sys.modules.get("shibokensupport.feature")
    if feature is None:
        return
    original = getattr(feature, "_mod_uses_pyside", None)
    if original is None or getattr(original, "_robocode_safe", False):
        return

    def safe(module):
        try:
            return original(module)
        except Exception:  # noqa: BLE001
            return False

    safe._robocode_safe = True
    feature._mod_uses_pyside = safe


def patch_shiboken_import_hook() -> None:
    try:
        _patch()
    except Exception:  # noqa: BLE001 - brak PySide6 lub inna wersja: poprawka zbędna
        pass
