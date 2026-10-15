"""Bazowy post-procesor: Program -> tekst dla kontrolera."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from ..core.fileio import atomic_write_text

from ..program.instructions import (Call, Instruction, Message, MoveC, MoveJ, MoveL, Pause, Program, SetFrame,
                                    SetIO, SetRounding, SetSpeed, SetTool, WaitIO)
from .tool_load import ToolLoad


class PostProcessor:
    ext = ".txt"
    name = "generic"
    version = "0.2"
    MAX_LINES_X_PROG = 5000

    def __init__(self, event_map: dict[str, str] | None = None, tool_load: ToolLoad | None = None) -> None:
        self.lines: list[str] = []
        self.prog_name = "Prog"
        self.robot = None
        # event name -> controller code template, e.g. {"SpindleOn": "$OUT[1] = TRUE", "Spindle": "S_SET({0})"}
        # {0}, {1}... are the event arguments. Unmapped events are emitted as subprogram calls (they must
        # exist on the controller – the validator warns about them). Review: Grok.
        self.event_map: dict[str, str] = dict(event_map or {})
        self.tool_load = tool_load

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

    def _emit_instruction(self, ins) -> None:
        if isinstance(ins, SetFrame):
            self.setFrame(ins.pose, ins.name)
        elif isinstance(ins, SetTool):
            self.setTool(ins.pose, ins.name)
        elif isinstance(ins, MoveJ):
            self.MoveJ(ins.pose if ins.pose is not None else None, ins.joints)
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

    def generate(self, prog: Program) -> str:
        self.lines = []
        self.prog_name = prog.name
        self.robot = prog.robot
        self.ProgStart(prog.name)
        for ins in prog.instructions:
            self._emit_instruction(ins)
        self.ProgFinish(prog.name)
        return "\n".join(self.lines) + "\n"

    def generate_chunks(self, prog: Program, max_lines: int) -> list[tuple[Program, str]]:
        """Render standalone, instruction-boundary parts without exceeding the line limit."""
        if not isinstance(max_lines, int) or isinstance(max_lines, bool) or max_lines < 1:
            raise ValueError("max_lines musi być dodatnią liczbą całkowitą")

        frame: SetFrame | None = None
        tool: SetTool | None = None
        rounding: SetRounding | None = None
        speed_values: dict[str, float] = {}
        prefix: list[Instruction] = []
        body: list[Instruction] = []
        chunks: list[tuple[list[Instruction], list[Instruction]]] = []

        def state_prefix() -> list[Instruction]:
            items: list[Instruction] = [item for item in (frame, tool) if item is not None]
            if speed_values:
                items.append(SetSpeed(**speed_values))
            if rounding is not None:
                items.append(rounding)
            return items

        def rendered_line_count(candidate_instructions: list[Instruction], chunk_index: int = 1) -> int:
            candidate = Program(name=f"{prog.name}_{chunk_index:03d}", robot=prog.robot,
                                instructions=list(candidate_instructions))
            candidate_post = self.__class__(event_map=self.event_map, tool_load=self.tool_load)
            return len(candidate_post.generate(candidate).splitlines())

        for ins in prog.instructions:
            candidate_instructions = prefix + body + [ins]
            if rendered_line_count(candidate_instructions) > max_lines:
                if not body:
                    raise ValueError(f"Instrukcja {type(ins).__name__} przekracza limit {max_lines} linii")
                chunks.append((prefix, body))
                prefix, body = state_prefix(), []
                candidate_instructions = prefix + [ins]
                if rendered_line_count(candidate_instructions, len(chunks) + 1) > max_lines:
                    raise ValueError(f"Instrukcja {type(ins).__name__} wraz ze stanem wymaganym "
                                     f"do kontynuacji przekracza limit {max_lines} linii")
            body.append(ins)
            if isinstance(ins, SetFrame):
                frame = ins
            elif isinstance(ins, SetTool):
                tool = ins
            elif isinstance(ins, SetSpeed):
                for key in ("linear_mm_s", "joint_deg_s", "linear_acc", "joint_acc"):
                    value = getattr(ins, key)
                    if value is not None:
                        speed_values[key] = value
            elif isinstance(ins, SetRounding):
                rounding = ins

        if body:
            chunks.append((prefix, body))

        if len(chunks) == 1:
            return [(prog, self.generate(prog))]

        rendered = []
        for index, (state, instructions) in enumerate(chunks, start=1):
            name = f"{prog.name}_{index:03d}"
            part = Program(name=name, robot=prog.robot, instructions=state + instructions)
            post = self.__class__(event_map=self.event_map, tool_load=self.tool_load)
            text = post.generate(part)
            if len(text.splitlines()) > max_lines:
                raise RuntimeError("Wewnętrzny błąd dzielenia: wygenerowana część przekracza limit linii")
            rendered.append((part, text))
        return rendered

    def save(self, prog: Program, folder: str | Path) -> list[Path]:
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        out = folder / f"{prog.name}{self.ext}"
        atomic_write_text(out, self.generate(prog))
        return [out]


def get_post(name: str, event_map: dict[str, str] | None = None,
             tool_load: ToolLoad | None = None) -> PostProcessor:
    from .abb_rapid import AbbRapid
    from .kuka_krl import KukaKrl
    posts = {"kuka": KukaKrl, "abb": AbbRapid, "generic": PostProcessor}
    if name not in posts:
        raise ValueError(f"Nieznany post-procesor '{name}'. Dostępne: {', '.join(posts)}")
    return posts[name](event_map=event_map, tool_load=tool_load)
