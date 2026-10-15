"""G-code parser with explicit modal state (supported dialect: docs/SPECYFIKACJA.md §6.4).

Supported:
  motion   G0 G1 G2 G3, G4 (dwell, P seconds)
  plane    G17 (XY) G18 (ZX) G19 (YZ)          - arcs in any plane, helical motion allowed
  units    G20 (inch) G21 (mm)                 - everything is converted to mm internally
  distance G90 / G91 (XYZ and A/B; arc centres I/J/K are always incremental)
  no-ops   G40 G49 G80 G94 (cancel modes / feed per minute = already our semantics)
  arcs     I/J/K centre offsets or R radius (R<0 = arc > 180 deg)
  tool     A/B head angles (B about Y, then A about X) or IJK tool vector on G0/G1
  other    F (feed, unit/min), S, T, M3/M4/M5/M6, M0/M1 (stop), M2/M30 (end), other M -> M_RunCode(n), E

Anything else that could change the meaning of the program (cutter compensation, canned
cycles, work offsets other than G54, inverse-time feed, C axis...) raises
UnsupportedCommandError in strict mode (default) or becomes a parser warning when strict=False.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field

import numpy as np

from ..errors import NCParseError, UnsupportedCommandError
from .toolpath import PathPoint, Toolpath

_WORD = re.compile(r"([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")
_LINE_OK = re.compile(r"^(?:\s*[A-Z]\s*[-+]?(?:\d+\.?\d*|\.\d+))*\s*$")

_NOOP_G = {40, 49, 54, 80, 94}  # G54 = identity work offset (the frame comes from the station)
_SUPPORTED_G = {0, 1, 2, 3, 4, 17, 18, 19, 20, 21, 90, 91} | _NOOP_G
_KNOWN_LETTERS = set("GMNOXYZABIJKRFSTEP")
# arc planes: (u, v, normal) axis indices; normal = u x v, CW (G2) = negative angle in (u, v)
_PLANES = {17: (0, 1, 2), 18: (2, 0, 1), 19: (1, 2, 0)}
_ARC_RADIUS_TOL_MM = 0.01


def _tool_vec_from_ab(a_deg: float, b_deg: float) -> np.ndarray:
    """Tool axis for a head with B about Y, then A about X: Rx(a) * Ry(b) * [0,0,1]."""
    a, b = np.radians(a_deg), np.radians(b_deg)
    v = np.array([np.sin(b), 0.0, np.cos(b)])
    v = np.array([v[0], v[1] * np.cos(a) - v[2] * np.sin(a), v[1] * np.sin(a) + v[2] * np.cos(a)])
    return v / np.linalg.norm(v)


@dataclass
class ModalState:
    """Explicit modal state of the NC interpreter (all lengths in mm)."""
    motion: int = 0
    plane: int = 17
    unit_scale: float = 1.0  # 1.0 = mm (G21), 25.4 = inch (G20)
    absolute: bool = True
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    a: float = 0.0
    b: float = 0.0
    feed_mm_min: float | None = None
    units_seen: bool = False


def _arc_center_from_r(p0, p1, r, cw, u, v, line_no, raw):
    du, dv = p1[u] - p0[u], p1[v] - p0[v]
    d = float(np.hypot(du, dv))
    if d < 1e-9:
        raise NCParseError("pełny okrąg nie może być zadany przez R – użyj I/J/K", line_no, raw)
    h2 = r * r - (d / 2) ** 2
    if h2 < -(_ARC_RADIUS_TOL_MM ** 2) and abs(r) < d / 2 - _ARC_RADIUS_TOL_MM:
        raise NCParseError(f"promień R={r:g} mm za mały dla cięciwy {d:.3f} mm", line_no, raw)
    h = np.sqrt(max(h2, 0.0))
    # left normal of the chord; CCW with R>0 -> centre on the left
    left = np.array([-dv, du]) / d
    side = 1.0 if not cw else -1.0
    if r < 0:
        side = -side
    mid = np.array([(p0[u] + p1[u]) / 2, (p0[v] + p1[v]) / 2])
    return mid + side * h * left


def parse_gcode(text: str, arc_seg_mm: float = 1.0, strict: bool = True) -> Toolpath:
    """Parse G-code into a Toolpath (positions in mm, part frame). See module docstring."""
    tp = Toolpath()
    st = ModalState()
    pending: list[str] = []
    ended = False

    def problem(msg: str, line_no: int, raw: str) -> None:
        if strict:
            raise UnsupportedCommandError(msg, line_no, raw)
        tp.warnings.append(f"linia {line_no}: {msg} – POMINIĘTO")

    for line_no, raw in enumerate(text.splitlines(), 1):
        line = re.sub(r"\(.*?\)|;.*", "", raw).upper().strip()
        if not line or line == "%":
            continue
        if ended:
            tp.warnings.append(f"linia {line_no}: treść po M2/M30 zignorowana")
            continue
        if "(" in line or ")" in line:
            raise NCParseError("niezamknięty komentarz", line_no, raw)
        if not _LINE_OK.match(line):
            raise NCParseError("nieczytelna składnia", line_no, raw)

        gs: list[int] = []
        w: dict[str, float] = {}
        m_events: list[str] = []  # executed after T/S/F in the same block (standard order of execution)
        unknown: list[str] = []
        for k, v in _WORD.findall(line):
            val = float(v)
            if k not in _KNOWN_LETTERS:
                unknown.append(f"{k}{v}")  # reported after G codes: "G41 D1" should blame G41, not D1
                continue
            if k == "G":
                if val != int(val):
                    problem(f"nieobsługiwany kod G{v}", line_no, raw)
                    continue
                gs.append(int(val))
            elif k == "M":
                m = int(val)
                if m == 3:
                    m_events.append("SpindleOn")
                elif m == 4:
                    m_events.append("SpindleOnCCW")
                elif m == 5:
                    m_events.append("SpindleOff")
                elif m in (0, 1):
                    m_events.append("Pause(-1)")
                elif m in (2, 30):
                    ended = True
                elif m == 6:
                    pass  # tool change: the T word carries the number
                else:
                    m_events.append(f"M_RunCode({m})")
            elif k in ("N", "O"):
                continue
            else:
                if k in w:
                    raise NCParseError(f"słowo {k} powtórzone w jednej linii", line_no, raw)
                w[k] = val

        dwell = False
        for g in gs:
            if g not in _SUPPORTED_G:
                problem(f"nieobsługiwany kod G{g} (zmienia znaczenie programu)", line_no, raw)
            elif g in (0, 1, 2, 3):
                st.motion = g
            elif g == 4:
                dwell = True
            elif g in _PLANES:
                st.plane = g
            elif g == 20:
                st.unit_scale, st.units_seen = 25.4, True
                tp.source_units = "inch"
            elif g == 21:
                st.unit_scale, st.units_seen = 1.0, True
            elif g == 90:
                st.absolute = True
            elif g == 91:
                st.absolute = False

        for word in unknown:
            problem(f"nieobsługiwane słowo '{word}'", line_no, raw)

        if dwell:
            sec = w.pop("P", None)
            if sec is None or sec < 0:
                raise NCParseError("G4 wymaga P (czas w sekundach, ≥0)", line_no, raw)
            pending.append(f"Pause({sec * 1000:g})")
        if "P" in w:
            problem("słowo P poza G4", line_no, raw)
            w.pop("P")

        sc = st.unit_scale
        if "T" in w:
            pending.append(f"SetTool({int(w['T'])})")
        if "S" in w:
            pending.append(f"Spindle({w['S']:g})")
        if "F" in w:
            if w["F"] <= 0:
                raise NCParseError("posuw F musi być > 0", line_no, raw)
            st.feed_mm_min = w["F"] * sc
        if "E" in w:
            pending.append(f"Extruder({w['E']:g})")
        pending.extend(m_events)

        has_xyz = any(k in w for k in "XYZ")
        has_ab = any(k in w for k in "AB")
        has_ijk = any(k in w for k in "IJK")
        if not (has_xyz or has_ab or has_ijk or "R" in w):
            continue
        if dwell:
            raise NCParseError("G4 nie może zawierać ruchu", line_no, raw)
        if not st.units_seen:
            tp.warnings.append(f"linia {line_no}: brak G20/G21 przed pierwszym ruchem – przyjęto mm (G21)")
            st.units_seen = True

        # --- target position ---
        new = st.pos.copy()
        for i, ax in enumerate("XYZ"):
            if ax in w:
                new[i] = w[ax] * sc if st.absolute else st.pos[i] + w[ax] * sc
        if "A" in w:
            st.a = w["A"] if st.absolute else st.a + w["A"]
        if "B" in w:
            st.b = w["B"] if st.absolute else st.b + w["B"]
        n = _tool_vec_from_ab(st.a, st.b)

        if st.motion in (0, 1):
            if "R" in w:
                raise NCParseError("R w ruchu liniowym", line_no, raw)
            if has_ijk:
                if not all(k in w for k in "IJK"):
                    raise NCParseError("wektor narzędzia wymaga wszystkich I, J, K", line_no, raw)
                if has_ab:
                    raise NCParseError("jednocześnie A/B i wektor IJK – niejednoznaczna orientacja", line_no, raw)
                vec = np.array([w["I"], w["J"], w["K"]])
                if np.linalg.norm(vec) < 1e-9:
                    raise NCParseError("zerowy wektor narzędzia IJK", line_no, raw)
                n = vec / np.linalg.norm(vec)
            if st.motion == 1 and st.feed_mm_min is None:
                # unknown cutting speed = risk (review: Gemini) -> error in strict mode
                problem("G1 bez wcześniejszego posuwu F", line_no, raw)
            tp.points.append(PathPoint(new, n, st.feed_mm_min, st.motion == 0, pending, line=line_no))
            pending = []
        else:
            u, v, wn = _PLANES[st.plane]
            cw = st.motion == 2
            p0 = st.pos
            if "R" in w:
                if has_ijk:
                    raise NCParseError("łuk zadany jednocześnie przez R i I/J/K", line_no, raw)
                c_uv = _arc_center_from_r(p0, new, w["R"] * sc, cw, u, v, line_no, raw)
            else:
                letters = "IJK"
                off = {0: w.get("I", 0.0), 1: w.get("J", 0.0), 2: w.get("K", 0.0)}
                if letters[wn] in w:
                    raise NCParseError(f"{letters[wn]} nie należy do płaszczyzny G{st.plane}", line_no, raw)
                if off[u] == 0 and off[v] == 0:
                    raise NCParseError("łuk bez środka (brak I/J/K lub R)", line_no, raw)
                c_uv = np.array([p0[u] + off[u] * sc, p0[v] + off[v] * sc])
            r0 = float(np.hypot(p0[u] - c_uv[0], p0[v] - c_uv[1]))
            r1 = float(np.hypot(new[u] - c_uv[0], new[v] - c_uv[1]))
            if abs(r0 - r1) > max(_ARC_RADIUS_TOL_MM, 1e-4 * r0):
                raise NCParseError(f"niespójny łuk: promień początku {r0:.4f} ≠ końca {r1:.4f} mm", line_no, raw)
            if st.feed_mm_min is None:
                problem(f"G{st.motion} bez wcześniejszego posuwu F", line_no, raw)
            a0 = np.arctan2(p0[v] - c_uv[1], p0[u] - c_uv[0])
            a1 = np.arctan2(new[v] - c_uv[1], new[u] - c_uv[0])
            if cw and a1 >= a0 - 1e-12:
                a1 -= 2 * np.pi
            if not cw and a1 <= a0 + 1e-12:
                a1 += 2 * np.pi
            nseg = max(2, int(np.ceil(abs(a1 - a0) * r0 / max(arc_seg_mm, 1e-3))))
            for t in np.linspace(0, 1, nseg + 1)[1:]:
                ang = a0 + t * (a1 - a0)
                p = p0 + t * (new - p0)  # helical component along the plane normal
                p[u] = c_uv[0] + r0 * np.cos(ang)
                p[v] = c_uv[1] + r0 * np.sin(ang)
                if t == 1.0:
                    p = new.copy()
                tp.points.append(PathPoint(p, n, st.feed_mm_min, False, pending, line=line_no))
                pending = []
        st.pos = new

    tp.end_events = pending  # e.g. final M5 – emitted after the last move
    return tp
