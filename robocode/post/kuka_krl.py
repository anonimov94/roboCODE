"""KUKA KRL (.src). Orientacja A,B,C = Z, Y', X''."""
from __future__ import annotations

from ..core import pose as P
from .base import PostProcessor


def _n(v: float, d: int) -> str:
    """Fixed decimals without '-0.000'."""
    t = f"{v:.{d}f}"
    return t[1:] if t.startswith("-") and float(t) == 0 else t


def _frame(T) -> str:
    x, y, z, a, b, c = P.to_xyzabc(T, "KUKA_ABC")
    return f"{{X {_n(x, 3)},Y {_n(y, 3)},Z {_n(z, 3)},A {_n(a, 4)},B {_n(b, 4)},C {_n(c, 4)}}}"


class KukaKrl(PostProcessor):
    ext = ".src"
    name = "kuka"
    version = "0.2"

    PTP_DEFAULT_PERCENT = 20.0  # first PTP must not run at the INITMOV default (review: Grok)

    def ProgStart(self, name):
        self.addline(f"DEF {name} ( )")
        self.addline(";FOLD INI")
        self.addline("  BAS (#INITMOV,0 )")
        self.addline(";ENDFOLD")
        self.addline(f"BAS (#VEL_PTP,{self.PTP_DEFAULT_PERCENT:.0f})")
        self.addline("$ORI_TYPE = #VAR")
        self.addline("$CIRC_TYPE = #BASE")
        if self.tool_load is not None:
            load = self.tool_load
            mx, my, mz = (load.mass_kg * value / 1000 for value in load.cog_mm)
            self.addline("; tool load: first moments in kg*m, inertia in kg*m^2")
            self.addline(f"$LOAD = {{M {_n(load.mass_kg, 3)}, MX {_n(mx, 6)}, MY {_n(my, 6)}, "
                         f"MZ {_n(mz, 6)}, JX {_n(load.inertia_kg_m2[0], 6)}, "
                         f"JY {_n(load.inertia_kg_m2[1], 6)}, JZ {_n(load.inertia_kg_m2[2], 6)}}}")

    def ProgFinish(self, name):
        self.addline("END")

    def MoveJ(self, pose, joints):
        j = [f"A{i+1} {_n(v, 4)}" for i, v in enumerate(joints)]
        self.addline("PTP {" + ",".join(j) + "}")  # exact stop: joint moves are approach/positioning only

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
        # BAS(#VEL_PTP, x) takes PERCENT of the axis maximum, not deg/s (bug found in review)
        vmax = min(getattr(getattr(self.robot, "model", None), "vmax_deg_s", None) or [150.0])
        pct = max(1.0, min(100.0, 100.0 * deg_s / vmax))
        self.addline(f"BAS (#VEL_PTP,{pct:.0f}) ; {deg_s:g} deg/s")

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
