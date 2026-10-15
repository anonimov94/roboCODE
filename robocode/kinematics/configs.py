"""Flagi konfiguracji 6R: Front/Rear, Up/Down, Flip/NoFlip (heurystyka geometryczna)."""
from __future__ import annotations

import numpy as np


def config_flags(model, q) -> dict:
    fr = model.fk_all(q)
    wrist = fr[4][:3, 3]          # środek nadgarstka (przybliżenie)
    shoulder = fr[1][:3, 3]
    base_x = fr[1][:3, 0]
    front = float(np.dot(wrist[:2] - shoulder[:2], base_x[:2])) >= 0
    up = q[2] >= 0  # uproszczenie: znak osi 3 (do dopracowania per model)
    flip = q[4] < 0
    return {"front": bool(front), "up": bool(up), "flip": bool(flip),
            "label": ("F" if front else "R") + ("U" if up else "D") + ("F" if flip else "N")}


def near_singularity(model, q, wrist_deg: float = 5.0, min_manip: float = 1e-4) -> str | None:
    if abs(q[4]) < wrist_deg:
        return f"Oś 5 blisko osobliwości ({q[4]:.1f}°). Rozważ ruch MoveJ albo zmianę obrotu wokół Z narzędzia."
    if model.manipulability(q) < min_manip:
        return "Robot blisko osobliwości (niska manipulowalność)."
    return None
