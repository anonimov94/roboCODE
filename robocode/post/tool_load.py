"""Validated payload data used by controller post-processors."""
from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class ToolLoad:
    mass_kg: float
    cog_mm: tuple[float, float, float]
    inertia_kg_m2: tuple[float, float, float]

    def __post_init__(self) -> None:
        try:
            mass = float(self.mass_kg)
        except (OverflowError, TypeError, ValueError) as exc:
            raise ValueError("mass_kg musi być skończoną liczbą > 0") from exc
        if not isfinite(mass) or mass <= 0:
            raise ValueError("mass_kg musi być skończone i > 0")
        object.__setattr__(self, "mass_kg", mass)
        for name in ("cog_mm", "inertia_kg_m2"):
            try:
                values = tuple(float(value) for value in getattr(self, name))
            except (OverflowError, TypeError, ValueError) as exc:
                raise ValueError(f"{name} musi zawierać dokładnie 3 skończone liczby") from exc
            if len(values) != 3 or not all(isfinite(value) for value in values):
                raise ValueError(f"{name} musi zawierać dokładnie 3 skończone liczby")
            if name == "inertia_kg_m2" and any(value < 0 for value in values):
                raise ValueError("inertia_kg_m2 nie może zawierać wartości ujemnych")
            object.__setattr__(self, name, values)
