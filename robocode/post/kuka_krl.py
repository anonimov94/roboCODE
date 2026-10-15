"""KUKA KRL (.src). Orientacja A,B,C = Z, Y', X''."""
from __future__ import annotations

from ..core import pose as P
from .base import PostProcessor


def _frame(T) -> str:
    x, y, z, a, b, c = P.to_xyzabc(T, "KUKA_ABC")
    return f"{{X {x:.3f},Y {y:.3f},Z {z:.3f},A {a:.4f},B {b:.4f},C {c:.4f}}}"


class KukaKrl(PostProcessor):
    ext = ".src"

    def ProgStart(self, name):
        self.addline(f"DEF {name} ( )")
        self.addline(";FOLD INI")
        self.addline("  BAS (#INITMOV,0 )")
        self.addline(";ENDFOLD")

    def ProgFinish(self, name):
        self.addline("END")

    def MoveJ(self, pose, joints):
        j = [f"A{i+1} {v:.4f}" for i, v in enumerate(joints)]
        self.addline("PTP {" + ",".join(j) + "} C_PTP")

    def MoveL(self, pose, joints):
        self.addline(f"LIN {_frame(pose)} C_DIS")

    def MoveC(self, p1, j1, p2, j2):
        self.addline(f"CIRC {_frame(p1)},{_frame(p2)} C_DIS")

    def setFrame(self, pose, name):
        self.addline(f"$BASE = {_frame(pose)} ; {name}")

    def setTool(self, pose, name):
        self.addline(f"$TOOL = {_frame(pose)} ; {name}")

    def setSpeed(self, mm_s):
        self.addline(f"$VEL.CP = {mm_s/1000:.5f}")

    def setSpeedJoints(self, deg_s):
        self.addline(f"BAS(#VEL_PTP,{min(100, deg_s):.1f})")

    def setZoneData(self, mm):
        self.addline(f"$APO.CDIS = {max(mm, 0):.3f}")

    def Pause(self, ms):
        self.addline("HALT" if ms < 0 else f"WAIT SEC {ms/1000:.3f}")

    def RunMessage(self, text):
        self.addline(f"; MSG: {text}")

    def RunCode(self, code, is_function_call=True):
        self.addline(code if not is_function_call or "(" in code else code + "()")

    def setDO(self, io, value):
        self.addline(f"$OUT[{io}] = {'TRUE' if str(value) in ('1', 'True', 'TRUE') else 'FALSE'}")

    def waitDI(self, io, value, timeout_ms=None):
        self.addline(f"WAIT FOR $IN[{io}] == {'TRUE' if str(value) in ('1', 'True', 'TRUE') else 'FALSE'}")
