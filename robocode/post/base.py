"""Bazowy post-procesor: Program -> tekst dla kontrolera."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from ..core.fileio import atomic_write_text

from ..program.instructions import (Call, Message, MoveC, MoveJ, MoveL, Pause, Program, SetFrame,
                                    SetIO, SetRounding, SetSpeed, SetTool, WaitIO)


class PostProcessor:
    ext = ".txt"
    name = "generic"
    version = "0.2"
    MAX_LINES_X_PROG = 5000

    def __init__(self, event_map: dict[str, str] | None = None) -> None:
        self.lines: list[str] = []
        self.prog_name = "Prog"
        self.robot = None
        # event name -> controller code template, e.g. {"SpindleOn": "$OUT[1] = TRUE", "Spindle": "S_SET({0})"}
        # {0}, {1}... are the event arguments. Unmapped events are emitted as subprogram calls (they must
        # exist on the controller – the validator warns about them). Review: Grok.
        self.event_map: dict[str, str] = dict(event_map or {})

    @staticmethod
    def split_call(code: str) -> tuple[str, list[str]]:
        """'Spindle(12000)' -> ('Spindle', ['12000'])."""
        if "(" in code and code.endswith(")"):
            name, args = code[:-1].split("(", 1)
            return name.strip(), [a.strip() for a in args.split(",") if a.strip()]
        return code.strip(), []

    def mapped_event(self, code: str) -> str | None:
        name, args = self.split_call(code)
        tpl = self.event_map.get(name)
        return tpl.format(*args) if tpl is not None else None

    # --- do nadpisania ---
    def ProgStart(self, name: str): self.addline(f"PROGRAM {name}")
    def ProgFinish(self, name: str): self.addline("END")
    def MoveJ(self, pose, joints): self.addline(f"MOVEJ {np.round(joints, 4).tolist()}")
    def MoveL(self, pose, joints): self.addline(f"MOVEL {np.round(pose[:3, 3], 3).tolist()}")
    def MoveC(self, pose1, joints1, pose2, joints2): self.addline("MOVEC")
    def setFrame(self, pose, name): self.addline(f"FRAME {name}")
    def setTool(self, pose, name): self.addline(f"TOOL {name}")
    def setSpeed(self, mm_s): self.addline(f"SPEED {mm_s}")
    def setSpeedJoints(self, deg_s): self.addline(f"SPEEDJ {deg_s}")
    def setZoneData(self, mm): self.addline(f"ZONE {mm}")
    def Pause(self, ms): self.addline(f"PAUSE {ms}")
    def RunMessage(self, text): self.addline(f"MSG {text}")
    def RunCode(self, code, is_function_call=True): self.addline(f"{code}" + ("" if "(" in code else "()"))
    def setDO(self, io, value): self.addline(f"SET {io}={value}")
    def waitDI(self, io, value, timeout_ms=None): self.addline(f"WAIT {io}=={value}")

    # --- wspólne ---
    def addline(self, s: str) -> None:
        self.lines.append(s)

    def generate(self, prog: Program) -> str:
        self.lines = []
        self.prog_name = prog.name
        self.robot = prog.robot
        self.ProgStart(prog.name)
        for ins in prog.instructions:
            if isinstance(ins, SetFrame):
                self.setFrame(ins.pose, ins.name)
            elif isinstance(ins, SetTool):
                self.setTool(ins.pose, ins.name)
            elif isinstance(ins, MoveJ):
                pose = ins.pose if ins.pose is not None else None
                self.MoveJ(pose, ins.joints)
            elif isinstance(ins, MoveL):
                self.MoveL(ins.pose, ins.joints)
            elif isinstance(ins, MoveC):
                self.MoveC(ins.pose_mid, ins.joints_mid, ins.pose_end, ins.joints_end)
            elif isinstance(ins, SetSpeed):
                if ins.linear_mm_s:
                    self.setSpeed(ins.linear_mm_s)
                if ins.joint_deg_s:
                    self.setSpeedJoints(ins.joint_deg_s)
            elif isinstance(ins, SetRounding):
                self.setZoneData(ins.mm)
            elif isinstance(ins, Pause):
                self.Pause(ins.ms)
            elif isinstance(ins, Message):
                self.RunMessage(ins.text)
            elif isinstance(ins, Call):
                mapped = self.mapped_event(ins.name) if ins.mode == "call" else None
                if mapped is not None:
                    self.RunCode(mapped, is_function_call=False)
                else:
                    self.RunCode(ins.name, ins.mode == "call")
            elif isinstance(ins, SetIO):
                self.setDO(ins.io, ins.value)
            elif isinstance(ins, WaitIO):
                self.waitDI(ins.io, ins.value, ins.timeout_ms)
        self.ProgFinish(prog.name)
        return "\n".join(self.lines) + "\n"

    def save(self, prog: Program, folder: str | Path) -> list[Path]:
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        out = folder / f"{prog.name}{self.ext}"
        atomic_write_text(out, self.generate(prog))
        return [out]


def get_post(name: str, event_map: dict[str, str] | None = None) -> PostProcessor:
    from .abb_rapid import AbbRapid
    from .kuka_krl import KukaKrl
    posts = {"kuka": KukaKrl, "abb": AbbRapid, "generic": PostProcessor}
    if name not in posts:
        raise ValueError(f"Nieznany post-procesor '{name}'. Dostępne: {', '.join(posts)}")
    return posts[name](event_map=event_map)
