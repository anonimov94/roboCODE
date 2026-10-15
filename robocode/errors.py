"""Domain exceptions for roboCODE.

Rule (docs/SPECYFIKACJA.md §2): never silently "fix" bad input. Raise a typed error that says
WHAT failed, WHERE (file line / path point / instruction) and HOW to fix it. Messages shown to the
operator are in Polish.
"""
from __future__ import annotations


class RoboCodeError(Exception):
    """Base class for all roboCODE domain errors."""


class NCParseError(RoboCodeError):
    """Invalid or unsupported NC (G-code) input."""

    def __init__(self, message: str, line_no: int | None = None, line: str | None = None):
        self.line_no = line_no
        self.line = line
        where = f"linia {line_no}: " if line_no is not None else ""
        src = f"  [{line.strip()}]" if line else ""
        super().__init__(f"{where}{message}{src}")


class UnsupportedCommandError(NCParseError):
    """NC word that could change the meaning of the program but is not supported."""


class InvalidGeometryError(RoboCodeError):
    """NaN/inf, zero-length vectors, non-orthonormal rotations, etc."""


class KinematicsError(RoboCodeError):
    """No IK solution / outside joint limits."""


class ExportBlockedError(RoboCodeError):
    """Export refused because validation reported blocking errors."""

    def __init__(self, report):
        self.report = report
        super().__init__(f"Eksport zablokowany: {len(report.errors)} błędów krytycznych walidacji.")
