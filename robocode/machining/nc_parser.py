"""Parser G-code (podzbiór): G0/G1/G2/G3 (łuki w XY jako odcinki), G90/G91, G17,
osie obrotowe A/B (konwencja głowicy: B wokół Y, A wokół X), wektory narzędzia I/J/K (nie dla łuków),
F, S, T, M-kody, E (ekstruder)."""
from __future__ import annotations

import re

import numpy as np

from .toolpath import PathPoint, Toolpath

_word = re.compile(r"([A-Z])\s*([-+]?\d*\.?\d+)")


def _tool_vec_from_ab(a_deg: float, b_deg: float) -> np.ndarray:
    a, b = np.radians(a_deg), np.radians(b_deg)
    # Rx(a) * Ry(b) * [0,0,1]
    v = np.array([np.sin(b), 0.0, np.cos(b)])
    v = np.array([v[0], v[1] * np.cos(a) - v[2] * np.sin(a), v[1] * np.sin(a) + v[2] * np.cos(a)])
    return v / np.linalg.norm(v)


def parse_gcode(text: str, arc_seg_mm: float = 1.0) -> Toolpath:
    tp = Toolpath()
    pos = np.zeros(3)
    a = b = 0.0
    feed = None
    absolute = True
    mode = 0
    pending: list[str] = []
    for raw in text.splitlines():
        line = re.sub(r"\(.*?\)|;.*", "", raw).upper().strip()
        if not line:
            continue
        words = _word.findall(line)
        w: dict[str, float] = {}
        gs = []
        for k, v in words:
            if k == "G":
                gs.append(int(float(v)))
            elif k == "M":
                m = int(float(v))
                if m in (3, 4):
                    pending.append("SpindleOn")
                elif m == 5:
                    pending.append("SpindleOff")
                elif m not in (0, 1, 2, 30, 6):
                    pending.append(f"M_RunCode({m})")
            else:
                w[k] = float(v)
        for g in gs:
            if g in (0, 1, 2, 3):
                mode = g
            elif g == 90:
                absolute = True
            elif g == 91:
                absolute = False
        if "T" in w:
            pending.append(f"SetTool({int(w['T'])})")
        if "S" in w:
            pending.append(f"Spindle({w['S']:g})")
        if "F" in w:
            feed = w["F"]
        if "E" in w:
            pending.append(f"Extruder({w['E']:g})")
        if not any(k in w for k in "XYZAB") and "I" not in w:
            continue
        new = np.array([w.get(ax, pos[i] if absolute else 0.0) for i, ax in enumerate("XYZ")])
        if not absolute:
            new = pos + new
        a = w.get("A", a)
        b = w.get("B", b)
        n = _tool_vec_from_ab(a, b)
        if mode in (0, 1) and all(k in w for k in "IJK"):
            n = np.array([w["I"], w["J"], w["K"]])
            n = n / np.linalg.norm(n)
        if mode in (2, 3):
            c = pos + np.array([w.get("I", 0.0), w.get("J", 0.0), 0.0])
            r = np.linalg.norm(pos[:2] - c[:2])
            a0 = np.arctan2(*(pos[:2] - c[:2])[::-1])
            a1 = np.arctan2(*(new[:2] - c[:2])[::-1])
            if mode == 2 and a1 >= a0:
                a1 -= 2 * np.pi
            if mode == 3 and a1 <= a0:
                a1 += 2 * np.pi
            nseg = max(2, int(abs(a1 - a0) * r / arc_seg_mm))
            for t in np.linspace(0, 1, nseg + 1)[1:]:
                ang = a0 + t * (a1 - a0)
                p = np.array([c[0] + r * np.cos(ang), c[1] + r * np.sin(ang), pos[2] + t * (new[2] - pos[2])])
                tp.points.append(PathPoint(p, n, feed, False, pending))
                pending = []
        else:
            tp.points.append(PathPoint(new, n, feed, mode == 0, pending))
            pending = []
        pos = new
    return tp
