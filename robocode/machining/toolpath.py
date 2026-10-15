"""Ścieżka narzędzia: punkty + wektory narzędzia + posuw + zdarzenia."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..errors import InvalidGeometryError


@dataclass
class PathPoint:
    p: np.ndarray            # mm, w układzie detalu
    n: np.ndarray            # wektor narzędzia (oś Z narzędzia), jednostkowy
    feed: float | None = None  # mm/min
    rapid: bool = False
    events: list[str] = field(default_factory=list)  # np. "SetTool(2)", "Spindle(12000)", "M_RunCode(8)"
    line: int | None = None  # numer linii w pliku źródłowym (diagnostyka)


@dataclass
class Toolpath:
    points: list[PathPoint] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)  # ostrzeżenia parsera (nie blokują)
    source_units: str = "mm"
    end_events: list[str] = field(default_factory=list)  # zdarzenia po ostatnim ruchu (np. M5)

    def __len__(self) -> int:
        return len(self.points)

    def positions(self) -> np.ndarray:
        return np.array([pp.p for pp in self.points])

    @classmethod
    def from_csv(cls, text: str, default_n=(0, 0, 1)) -> "Toolpath":
        """Linie 'x,y,z[,i,j,k]' – import krzywej/punktów."""
        tp = cls()
        for ln, line in enumerate(text.strip().splitlines(), 1):
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            v = [float(s) for s in line.replace(";", ",").split(",") if s.strip()]
            if len(v) not in (3, 6) or not np.all(np.isfinite(v)):
                raise InvalidGeometryError(f"CSV linia {ln}: oczekiwano 'x,y,z' lub 'x,y,z,i,j,k' (skończone liczby)")
            n = np.array(v[3:6]) if len(v) >= 6 else np.array(default_n, float)
            if np.linalg.norm(n) < 1e-9:
                raise InvalidGeometryError(f"CSV linia {ln}: zerowy wektor narzędzia")
            tp.points.append(PathPoint(np.array(v[:3]), n / np.linalg.norm(n), line=ln))
        return tp

    def with_approach_retract(self, dist_mm: float = 100.0) -> "Toolpath":
        """Najazd/odjazd wzdłuż wektora narzędzia (domyślnie 100 mm)."""
        if not self.points:
            return self
        first, last = self.points[0], self.points[-1]
        a = PathPoint(first.p + first.n * dist_mm, first.n, rapid=True)
        r = PathPoint(last.p + last.n * dist_mm, last.n, rapid=True)
        return Toolpath([a, *self.points, r], warnings=list(self.warnings), source_units=self.source_units,
                        end_events=list(self.end_events))

    def densify_orientation(self, max_step_deg: float = 5.0) -> "Toolpath":
        """Insert intermediate points where the tool vector turns more than max_step_deg between points.

        Position is interpolated linearly, the tool vector by slerp – the same thing a controller does
        in a LIN move. Inserted points inherit line/feed/rapid of the target point but carry no events.
        """
        if max_step_deg <= 0 or len(self.points) < 2:
            return self
        out = [self.points[0]]
        for a, b in zip(self.points, self.points[1:]):
            ang = float(np.degrees(np.arccos(np.clip(np.dot(a.n, b.n), -1.0, 1.0))))
            k = int(np.ceil(ang / max_step_deg - 1e-9))
            if k > 1:
                # rotation axis; for (nearly) opposite vectors pick any axis perpendicular to a.n (review: Gemini)
                axis = np.cross(a.n, b.n)
                if np.linalg.norm(axis) < 1e-6:
                    ref = np.array([1.0, 0, 0]) if abs(a.n[0]) < 0.9 else np.array([0, 1.0, 0])
                    axis = np.cross(a.n, ref)
                axis = axis / np.linalg.norm(axis)
                th = np.radians(ang)
                for i in range(1, k):
                    t = i / k
                    phi = t * th  # Rodrigues rotation of a.n about axis
                    n = (a.n * np.cos(phi) + np.cross(axis, a.n) * np.sin(phi)
                         + axis * np.dot(axis, a.n) * (1 - np.cos(phi)))
                    out.append(PathPoint(a.p + t * (b.p - a.p), n / np.linalg.norm(n), b.feed, b.rapid, [], b.line))
            out.append(b)
        return Toolpath(out, warnings=list(self.warnings), source_units=self.source_units,
                        end_events=list(self.end_events))
