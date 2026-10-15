"""Ścieżka narzędzia: punkty + wektory narzędzia + posuw + zdarzenia."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class PathPoint:
    p: np.ndarray            # mm, w układzie detalu
    n: np.ndarray            # wektor narzędzia (oś Z narzędzia), jednostkowy
    feed: float | None = None  # mm/min
    rapid: bool = False
    events: list[str] = field(default_factory=list)  # np. "SetTool(2)", "Spindle(12000)", "M_RunCode(8)"


@dataclass
class Toolpath:
    points: list[PathPoint] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.points)

    def positions(self) -> np.ndarray:
        return np.array([pp.p for pp in self.points])

    @classmethod
    def from_csv(cls, text: str, default_n=(0, 0, 1)) -> "Toolpath":
        """Linie 'x,y,z[,i,j,k]' – import krzywej/punktów."""
        tp = cls()
        for line in text.strip().splitlines():
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            v = [float(s) for s in line.replace(";", ",").split(",") if s.strip()]
            n = np.array(v[3:6]) if len(v) >= 6 else np.array(default_n, float)
            tp.points.append(PathPoint(np.array(v[:3]), n / np.linalg.norm(n)))
        return tp

    def with_approach_retract(self, dist_mm: float = 100.0) -> "Toolpath":
        """Najazd/odjazd wzdłuż wektora narzędzia (domyślnie 100 mm)."""
        if not self.points:
            return self
        first, last = self.points[0], self.points[-1]
        a = PathPoint(first.p + first.n * dist_mm, first.n, rapid=True)
        r = PathPoint(last.p + last.n * dist_mm, last.n, rapid=True)
        return Toolpath([a, *self.points, r])
