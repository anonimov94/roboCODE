"""ABB RAPID (.mod). Orientacja jako kwaternion."""
from __future__ import annotations

import numpy as np

from ..core import pose as P
from .base import PostProcessor


def _pose(T) -> str:
    x, y, z = T[:3, 3]
    q = P.to_quaternion(T)
    return f"[{x:.3f},{y:.3f},{z:.3f}],[{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}]"


class AbbRapid(PostProcessor):
    ext = ".mod"

    def __init__(self):
        super().__init__()
        self.speed = "v100"
        self.zone = "fine"
        self.tool = "tool0"
        self.wobj = "wobj0"
        self.decl: list[str] = []

    def ProgStart(self, name):
        self.addline(f"MODULE MOD_{name}")
        self.addline("  PERS tooldata rcTool := [TRUE,[[0,0,0],[1,0,0,0]],[1,[0,0,50],[1,0,0,0],0,0,0]];")
        self.addline("  TASK PERS wobjdata rcWobj := [FALSE,TRUE,\"\",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];")
        self.addline(f"  PROC {name}()")
        self.addline("    ConfJ \\Off; ConfL \\Off;")

    def ProgFinish(self, name):
        self.addline("  ENDPROC")
        self.addline("ENDMODULE")

    def MoveJ(self, pose, joints):
        j = ",".join(f"{v:.4f}" for v in joints)
        self.addline(f"    MoveAbsJ [[{j}],[9E9,9E9,9E9,9E9,9E9,9E9]],{self.speed},{self.zone},rcTool\\WObj:=rcWobj;")

    def MoveL(self, pose, joints):
        self.addline(f"    MoveL [{_pose(pose)},[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],{self.speed},{self.zone},rcTool\\WObj:=rcWobj;")

    def MoveC(self, p1, j1, p2, j2):
        r = "[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]"
        self.addline(f"    MoveC [{_pose(p1)},{r}],[{_pose(p2)},{r}],{self.speed},{self.zone},rcTool\\WObj:=rcWobj;")

    def setFrame(self, pose, name):
        self.addline(f"    rcWobj.uframe := [{_pose(pose)}]; ! {name}")

    def setTool(self, pose, name):
        self.addline(f"    rcTool.tframe := [{_pose(pose)}]; ! {name}")

    def setSpeed(self, mm_s):
        self.addline(f"    ! speed {mm_s:.1f} mm/s")
        self.speed = f"[{mm_s:.1f},500,5000,1000]"

    def setZoneData(self, mm):
        self.zone = "fine" if mm < 0 else f"z{int(round(mm))}"

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


_ = np  # (numpy zostaje do przyszłych rozszerzeń)
