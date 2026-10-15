"""ABB RAPID (.mod). Orientacja jako kwaternion."""
from __future__ import annotations

import numpy as np

from ..core import pose as P
from .base import PostProcessor


_PREDEFINED_ZONES = {0, 1, 5, 10, 15, 20, 30, 40, 50, 60, 80, 100, 150, 200}


def _zone(mm: float) -> str:
    """fine / predefined zN / inline zonedata for any other radius (z2, z3 do not exist in RAPID)."""
    if mm < 0:
        return "fine"
    n = round(mm)
    if abs(mm - n) < 1e-9 and n in _PREDEFINED_ZONES:
        return f"z{n}"
    return f"[FALSE,{mm:g},{1.5 * mm:g},{1.5 * mm:g},{0.15 * mm:g},{1.5 * mm:g},{0.15 * mm:g}]"


def _conf(joints) -> str:
    """confdata from the solver's joints: quadrant numbers of axes 1, 4, 6 (cfx left 0, ConfL\\Off)."""
    if joints is None:
        return "[0,0,0,0]"
    q = np.round(np.asarray(joints, float), 6) + 0.0  # -2e-14 must not land in quadrant -1
    return f"[{int(np.floor(q[0] / 90))},{int(np.floor(q[3] / 90))},{int(np.floor(q[5] / 90))},0]"


def _n(v: float, d: int) -> str:
    """Fixed decimals without '-0.000'."""
    t = f"{v:.{d}f}"
    return t[1:] if t.startswith("-") and float(t) == 0 else t


def _pose(T) -> str:
    x, y, z = T[:3, 3]
    q = P.to_quaternion(T)
    return f"[{_n(x, 3)},{_n(y, 3)},{_n(z, 3)}],[{_n(q[0], 6)},{_n(q[1], 6)},{_n(q[2], 6)},{_n(q[3], 6)}]"


class AbbRapid(PostProcessor):
    ext = ".mod"
    name = "abb"
    version = "0.2"

    def __init__(self, event_map=None):
        super().__init__(event_map)
        self.speed = "v100"
        self.zone = "fine"
        self.tool = "tool0"
        self.wobj = "wobj0"
        self.decl: list[str] = []

    def ProgStart(self, name):
        self.addline(f"MODULE MOD_{name}")
        self.addline("  ! UWAGA: dane obciazenia narzedzia (1 kg, CoG Z50) sa domyslne - wpisz rzeczywiste")
        self.addline("  ! (masa/srodek ciezkosci/bezwladnosc wrzeciona, np. z LoadIdentify) przed praca produkcyjna")
        self.addline("  PERS tooldata rcTool := [TRUE,[[0,0,0],[1,0,0,0]],[1,[0,0,50],[1,0,0,0],0,0,0]];")
        self.addline("  TASK PERS wobjdata rcWobj := [FALSE,TRUE,\"\",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];")
        self.addline(f"  PROC {name}()")
        self.addline("    ConfJ \\Off; ConfL \\Off;")

    def ProgFinish(self, name):
        self.addline("  ENDPROC")
        self.addline("ENDMODULE")

    def MoveJ(self, pose, joints):
        j = ",".join(_n(v, 4) for v in joints)
        self.addline(f"    MoveAbsJ [[{j}],[9E9,9E9,9E9,9E9,9E9,9E9]],{self.speed},{self.zone},rcTool;")

    def MoveL(self, pose, joints):
        ext = "[9E9,9E9,9E9,9E9,9E9,9E9]"
        self.addline(f"    MoveL [{_pose(pose)},{_conf(joints)},{ext}],{self.speed},{self.zone},rcTool\\WObj:=rcWobj;")

    def MoveC(self, p1, j1, p2, j2):
        ext = "[9E9,9E9,9E9,9E9,9E9,9E9]"
        self.addline(f"    MoveC [{_pose(p1)},{_conf(j1)},{ext}],[{_pose(p2)},{_conf(j2)},{ext}],"
                     f"{self.speed},{self.zone},rcTool\\WObj:=rcWobj;")

    def setFrame(self, pose, name):
        self.addline(f"    rcWobj.uframe := [{_pose(pose)}]; ! {name}")

    def setTool(self, pose, name):
        self.addline(f"    rcTool.tframe := [{_pose(pose)}]; ! {name}")

    def setSpeed(self, mm_s):
        self.addline(f"    ! speed {mm_s:.1f} mm/s")
        self.speed = f"[{mm_s:.1f},500,5000,1000]"

    def setZoneData(self, mm):
        self.zone = _zone(mm)

    def setSpeedJoints(self, deg_s):
        self.addline(f"    ! joint speed {deg_s:g} deg/s (MoveAbsJ is limited by speeddata)")

    def Pause(self, ms):
        self.addline("    Stop;" if ms < 0 else f"    WaitTime {ms/1000:.3f};")

    def RunMessage(self, text):
        self.addline(f"    TPWrite \"{text[:80]}\";")

    def RunCode(self, code, is_function_call=True):
        if not is_function_call:
            self.addline(f"    {code}")              # wstawiony kod
        elif "(" in code:
            name, args = code.split("(", 1)
            self.addline(f"    {name} {args.rstrip(')')};")  # SetTool(2) -> SetTool 2;
        else:
            self.addline(f"    {code};")

    def setDO(self, io, value):
        self.addline(f"    SetDO {io},{value};")

    def waitDI(self, io, value, timeout_ms=None):
        t = f"\\MaxTime:={timeout_ms/1000:.1f}" if timeout_ms else ""
        self.addline(f"    WaitDI {io},{value}{t};")

