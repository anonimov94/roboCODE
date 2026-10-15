"""End-to-end job: NC file -> toolpath -> robot program -> validation -> post -> manifest.

This is the PoC "single robot, full flow" required by docs/SPECYFIKACJA.md §10 (v0.3/v0.4).
Export happens ONLY when validation has no blocking errors. Every run writes a manifest that ties
the generated file to the exact configuration it was validated for (§8).
"""
from __future__ import annotations

import dataclasses
import datetime as _dt
import json
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import __version__
from .core.fileio import atomic_write_text, sha256_text
from .errors import ExportBlockedError, RoboCodeError
from .machining.nc_parser import parse_gcode
from .machining.solver import MachiningSettings, solve
from .post.base import get_post
from .program.instructions import Program, SetFrame, SetTool
from .program.report import Severity, ValidationReport
from .program.validator import ValidationLimits, validate_program, verify_post_output

MANIFEST_FORMAT = "robocode-manifest/1"


@dataclass
class JobResult:
    ok: bool
    report: ValidationReport
    outputs: list[Path] = field(default_factory=list)
    manifest_path: Path | None = None
    message: str = ""


def _git_commit() -> str | None:
    try:
        out = subprocess.run(["git", "rev-parse", "HEAD"], cwd=Path(__file__).parent, capture_output=True,
                             text=True, timeout=3)
        return (out.stdout.strip() or None) if out.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


def _jsonable(v):
    if isinstance(v, np.ndarray):
        return np.round(v, 9).tolist()
    if isinstance(v, (np.floating, np.integer)):
        return v.item()
    if dataclasses.is_dataclass(v) and not isinstance(v, type):
        return {k: _jsonable(x) for k, x in dataclasses.asdict(v).items()}
    if isinstance(v, dict):
        return {k: _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    return v


def _load_cell(station_path: str | None, post: str):
    """(robot, frame, tool) from a .rcs.json station or the built-in demo cell."""
    if station_path:
        from .core.io import load_station
        from .core.scene import Frame, Robot
        st = load_station(station_path)
        robot = next((i for i in st.walk() if isinstance(i, Robot)), None)
        if robot is None:
            raise RoboCodeError("Stacja nie zawiera robota")
        frame = robot.active_frame or next((i for i in st.walk() if isinstance(i, Frame)), None)
        tool = robot.active_tool or (robot.tools()[0] if robot.tools() else None)
        if frame is None or tool is None:
            raise RoboCodeError("Robot w stacji musi mieć aktywny układ detalu i narzędzie")
        robot.post_processor = post
        return robot, frame, tool, station_path
    from .demo import build_demo_station
    _, robot, frame, tool = build_demo_station(post)
    return robot, frame, tool, "builtin:demo"


def _new_manifest(post: str, limits: ValidationLimits | None) -> dict:
    return {"format": MANIFEST_FORMAT, "robocode_version": __version__, "git_commit": _git_commit(),
            "created_utc": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"),
            "post": {"name": post}, "limits": _jsonable(limits or ValidationLimits())}


def _cell_info(robot, frame_name: str, frame_rel_base, tool_name: str, tcp, source: str) -> dict:
    m = robot.model
    return {"source": source,
            "robot": {"name": m.name, "model_source": m.source, "model_license": m.license,
                      "model_sha256": sha256_text(m.to_json()), "base_pose": _jsonable(robot.pose_abs()),
                      "start_joints": _jsonable(robot.joints)},
            "frame": {"name": frame_name, "pose_rel_base": _jsonable(frame_rel_base)},
            "tool": {"name": tool_name, "tcp": _jsonable(tcp)}}


def _finish(rep: ValidationReport, manifest: dict, out_dir: Path, name: str, ok: bool,
            outputs: list[Path], msg: str) -> JobResult:
    if not ok:
        for p in sorted(out_dir.glob(f"{name}.*")):
            if p.suffix in (".src", ".mod", ".txt"):
                rep.add(Severity.WARNING, "STALE_OUTPUT", f"W folderze leży starszy plik {p.name} z innego "
                        "przebiegu – NIE odpowiada temu wejściu", stage="post", hint="usuń go albo nie wysyłaj")
    manifest["validation"] = rep.to_dict()
    manifest["exported"] = ok
    manifest["outputs"] = [{"file": p.name, "sha256": sha256_text(p.read_text(encoding="utf-8"))}
                           for p in outputs]
    mp = atomic_write_text(out_dir / f"{name}.manifest.json", json.dumps(manifest, indent=1, ensure_ascii=False))
    return JobResult(ok, rep, outputs, mp, msg)


def export_program(program: Program, post: str, out_dir: str | Path, limits: ValidationLimits | None = None,
                   parser_warnings: list[str] | None = None, manifest: dict | None = None,
                   report: ValidationReport | None = None, event_map: dict[str, str] | None = None) -> JobResult:
    """The ONLY sanctioned way to write a controller file: validate -> post -> read-back -> atomic write.

    Used by the CLI pipeline and by the GUI (F6). Always writes <name>.manifest.json.
    """
    out_dir = Path(out_dir)
    rep = report if report is not None else ValidationReport()
    if manifest is None:
        manifest = _new_manifest(post, limits)
        fr = next((i for i in program.instructions if isinstance(i, SetFrame)), None)
        tl = next((i for i in program.instructions if isinstance(i, SetTool)), None)
        if program.robot is not None:
            manifest["cell"] = _cell_info(program.robot, fr.name if fr else None, fr.pose if fr else None,
                                          tl.name if tl else None, tl.pose if tl else None, "program")
    name = program.name

    vrep, _ = validate_program(program, limits, parser_warnings=parser_warnings,
                               mapped_events=set(event_map or ()))
    manifest["event_map"] = dict(event_map or {})
    rep.extend(vrep)
    if not rep.ok:
        return _finish(rep, manifest, out_dir, name, False, [], "✗ Walidacja nie przeszła – eksport zablokowany")

    pp = get_post(post, event_map)
    if post == "abb":  # review (Grok): default tool load data must not go unnoticed
        rep.add(Severity.WARNING, "TOOL_LOAD_DEFAULT", "Dane obciążenia narzędzia w RAPID są domyślne (1 kg, CoG Z50)",
                stage="post", hint="wpisz rzeczywistą masę i środek ciężkości wrzeciona w rcTool przed produkcją")
    manifest["post"].update({"class": type(pp).__name__, "version": getattr(pp, "version", "?")})
    text = pp.generate(program)  # in memory first; written only after the read-back check
    rep.extend(verify_post_output(post, text, program, limits))
    if not rep.ok:
        return _finish(rep, manifest, out_dir, name, False, [],
                       "✗ Weryfikacja pliku sterownika nie przeszła – eksport zablokowany")
    out = atomic_write_text(out_dir / f"{name}{pp.ext}", text)
    msg = f"✓ Zapisano {out.name} (ostrzeżenia: {len(rep.warnings)})"
    return _finish(rep, manifest, out_dir, name, True, [out], msg)


def run_job(nc_text: str, post: str = "kuka", out_dir: str | Path = "out", name: str = "Machining",
            settings: MachiningSettings | None = None, limits: ValidationLimits | None = None,
            station_path: str | None = None, nc_name: str = "<text>", strict: bool = True,
            event_map: dict[str, str] | None = None) -> JobResult:
    """Run the whole pipeline. Never raises for operator-level problems: they land in the report."""
    out_dir = Path(out_dir)
    settings = settings or MachiningSettings()
    rep = ValidationReport()
    manifest = _new_manifest(post, limits)
    manifest["input"] = {"name": nc_name, "sha256": sha256_text(nc_text), "strict_parser": strict}
    manifest["settings"] = _jsonable(settings)

    def fail(msg: str) -> JobResult:
        return _finish(rep, manifest, out_dir, name, False, [], msg)

    # 1. parse
    try:
        tp = parse_gcode(nc_text, strict=strict)
    except RoboCodeError as e:
        rep.add(Severity.ERROR, "NC_PARSE", str(e), stage="parse", source_line=getattr(e, "line_no", None))
        return fail(f"✗ Błąd pliku NC: {e}")
    manifest["input"]["units"] = tp.source_units
    if len(tp) == 0:
        rep.add(Severity.ERROR, "EMPTY", "Plik NC nie zawiera ruchów", stage="parse")
        return fail("✗ Plik NC nie zawiera ruchów")

    # 2. cell + solve
    try:
        robot, frame, tool, cell_src = _load_cell(station_path, post)
    except (RoboCodeError, OSError, ValueError, KeyError) as e:
        rep.add(Severity.ERROR, "STATION", f"Nie można wczytać stacji: {e}", stage="static")
        return fail(f"✗ Stacja: {e}")
    manifest["cell"] = _cell_info(robot, frame.name, robot.frame_rel_base(frame), tool.name, tool.tcp(), cell_src)
    res = solve(robot, robot.frame_rel_base(frame), tool.tcp(), tp, settings, name=name)
    if not res.ok:
        tpa = res.toolpath
        code = "CONFIG_JUMP" if "skoku konfiguracji" in res.message else "UNREACHABLE"
        for i in [i for i, r in enumerate(res.reachable) if not r][:20]:
            rep.add(Severity.ERROR, code, f"Punkt ścieżki {i}: " + ("wymaga skoku konfiguracji" if code ==
                    "CONFIG_JUMP" else "nieosiągalny"), stage="solve",
                    source_line=tpa.points[i].line,
                    hint="przesuń układ detalu, zwiększ zakres obrotu Z lub zmień konfigurację startową")
        return fail(res.message)

    # 3+4. validate whole trajectory, post, read back, write
    return export_program(res.program, post, out_dir, limits, tp.warnings, manifest, rep, event_map)


def export_or_raise(job: JobResult) -> list[Path]:
    """For API users who prefer exceptions."""
    if not job.ok:
        raise ExportBlockedError(job.report)
    return job.outputs
