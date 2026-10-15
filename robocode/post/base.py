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

    SPLIT_RETRACT_MM = 50.0  # safe restart distance along the tool axis between program parts

    def _instruction_line_count(self, ins, robot) -> int:
        probe = self.__class__(event_map=self.event_map, tool_load=self.tool_load)
        probe.robot = robot
        probe.lines = []
        probe._emit_instruction(ins)
        return len(probe.lines)

    def generate_chunks(self, prog: Program, max_lines: int,
                        retract_mm: float | None = None) -> list[tuple[Program, str]]:
        """Split a program into standalone parts with a SAFE restart sequence (review of Copilot change).

        Every part after the first starts with: frame/tool/speed/rounding state, the active tool and
        spindle events, a joint move (PTP) to a point `retract_mm` above its first target along the tool
        axis and a linear plunge. Every part except the last ends with a linear retract along the tool
        axis. Without this a part would start with LIN inside the material (KUKA BCO run) and without
        the spindle running. Line counting is O(n) (was O(n^2)). The added moves are validated by the
        pipeline. Raises ValueError when a safe split is impossible.
        """
        if not isinstance(max_lines, int) or isinstance(max_lines, bool) or max_lines < 1:
            raise ValueError("max_lines musi być dodatnią liczbą całkowitą")
        retract = self.SPLIT_RETRACT_MM if retract_mm is None else float(retract_mm)
        robot = prog.robot
        header = len(self.__class__(event_map=self.event_map, tool_load=self.tool_load)
                     .generate(Program(name=prog.name, robot=robot)).splitlines())
        counts = [self._instruction_line_count(ins, robot) for ins in prog.instructions]
        if header + sum(counts) <= max_lines:
            return [(prog, self.generate(prog))]
        if robot is None or getattr(robot, "model", None) is None:
            raise ValueError("Dzielenie programu wymaga robota (ruchy bezpiecznego wznowienia)")
        model = robot.model

        frame = tool = rounding = None
        speed_values: dict[str, float] = {}
        ev_tool = ev_speed = ev_spindle = None
        last_q = None
        last_pose = None
        parts: list[list[Instruction]] = []
        cur: list[Instruction] = []
        cur_lines = header

        def motion_target(ins):
            if isinstance(ins, MoveL):
                return ins.pose, ins.joints
            if isinstance(ins, MoveC):
                return ins.pose_end, ins.joints_end
            if isinstance(ins, MoveJ):
                return ins.pose, ins.joints
            return None, None

        def offset_move(pose, seed, kind):
            if pose is None or seed is None or frame is None or tool is None:
                raise ValueError("Nie można bezpiecznie podzielić programu: brak pozy/osi przy granicy części")
            target = pose @ np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -retract], [0, 0, 0, 1.0]])
            base_T = frame.pose @ target @ np.linalg.inv(tool.pose)
            q = model.ik(base_T, np.asarray(seed, float))
            if q is None or np.abs(q - np.asarray(seed, float)).max() > 45.0:
                raise ValueError(f"Nie można bezpiecznie podzielić programu: punkt {kind} "
                                 f"{retract:g} mm nad ścieżką jest nieosiągalny")
            ins = (MoveL(pose=target, joints=q) if kind == "odjazdu" else MoveJ(joints=q, pose=target))
            ins.inserted_by_split = True
            return ins

        def restart_prefix(first_motion):
            items: list[Instruction] = [x for x in (frame, tool) if x is not None]
            if speed_values:
                items.append(SetSpeed(**speed_values))
            if rounding is not None:
                items.append(rounding)
            items += [Call(c) for c in (ev_tool, ev_speed, ev_spindle) if c]
            pose, q = motion_target(first_motion)
            items.append(offset_move(pose, q, "najazdu"))
            for x in items:
                x.inserted_by_split = True
            return items

        def lines_of(items):
            return sum(self._instruction_line_count(x, robot) for x in items)

        for k, ins in enumerate(prog.instructions):
            is_motion = isinstance(ins, (MoveJ, MoveL, MoveC))
            # reserve room for the retract that closes this part
            if is_motion and cur and last_pose is not None and cur_lines + counts[k] + 2 > max_lines:
                retract_ins = offset_move(last_pose, last_q, "odjazdu")
                cur.append(retract_ins)
                parts.append(cur)
                cur = restart_prefix(ins)
                cur_lines = header + lines_of(cur)
                if cur_lines + counts[k] + 2 > max_lines:
                    raise ValueError(f"Limit {max_lines} linii jest za mały na sekwencję bezpiecznego wznowienia")
            cur.append(ins)
            cur_lines += counts[k]
            if isinstance(ins, SetFrame):
                frame = ins
            elif isinstance(ins, SetTool):
                tool = ins
            elif isinstance(ins, SetRounding):
                rounding = ins
            elif isinstance(ins, SetSpeed):
                for key in ("linear_mm_s", "joint_deg_s", "linear_acc", "joint_acc"):
                    value = getattr(ins, key)
                    if value is not None:
                        speed_values[key] = value
            elif isinstance(ins, Call):
                name = self.split_call(ins.name)[0]
                if name == "SetTool":
                    ev_tool = ins.name
                elif name == "Spindle":
                    ev_speed = ins.name
                elif name in ("SpindleOn", "SpindleOnCCW"):
                    ev_spindle = ins.name
                elif name == "SpindleOff":
                    ev_spindle = None
            if is_motion:
                pose, q = motion_target(ins)
                if pose is not None and q is not None:
                    last_pose, last_q = pose, q
        if cur:
            parts.append(cur)

        rendered = []
        for index, instructions in enumerate(parts, start=1):
            part = Program(name=f"{prog.name}_{index:03d}", robot=robot, instructions=instructions)
            text = self.__class__(event_map=self.event_map, tool_load=self.tool_load).generate(part)
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
