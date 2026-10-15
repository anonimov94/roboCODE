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
from .collision.checker import CollisionChecker
from .core.fileio import atomic_write_text, sha256_text
from .errors import ExportBlockedError, RoboCodeError
from .machining.nc_parser import parse_gcode
from .machining.solver import MachiningSettings, StartConfiguration, enumerate_start_configurations, solve
from .post.base import get_post
from .program.instructions import Program, SetFrame, SetTool
from .program.report import Severity, ValidationReport
from .program.validator import ValidationLimits, validate_program, verify_post_output
from .post.tool_load import ToolLoad

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


def list_start_configurations(nc_text: str, station_path: str | None = None,
                              settings: MachiningSettings | None = None, strict: bool = True
                              ) -> list[StartConfiguration]:
    """Return ordered IK branches for the first solved CAM point, without exporting a program."""
    toolpath = parse_gcode(nc_text, strict=strict)
    if not toolpath.points:
        raise RoboCodeError("Plik NC nie zawiera punktów ruchu")
    robot, frame, tool, _ = _load_cell(station_path, "kuka")
    return enumerate_start_configurations(robot, robot.frame_rel_base(frame), tool.tcp(), toolpath,
                                          settings or MachiningSettings())


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


def _collision_check_info(checker: CollisionChecker) -> dict:
    return {
        "method": "link_capsules_vs_axis_aligned_boxes",
        "link_radius_mm": checker.link_radius_mm,
        "tool_radius_mm": checker.tool_radius_mm,
        "pairs": None if checker.pairs is None else [
            list(pair) for pair in sorted(checker.pairs, key=lambda pair: (pair[0], pair[1]))
        ],
        "tool_proxy_enabled": checker.pairs is not None
        and any(component.rsplit("/", 1)[-1].startswith("tool:") for component, _ in checker.pairs),
        "limitations": ["approximate_link_geometry", "no_self_collision", "no_mesh_collision"],
    }


def _finish(rep: ValidationReport, manifest: dict, out_dir: Path, name: str, ok: bool,
            outputs: list[Path], msg: str) -> JobResult:
    if not ok:
        old_outputs = set(out_dir.glob(f"{name}.*")) | set(out_dir.glob(f"{name}_*.*"))
        for p in sorted(old_outputs):
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
                   report: ValidationReport | None = None, event_map: dict[str, str] | None = None,
                   tool_load: ToolLoad | None = None, max_lines_per_file: int | None = None,
                   collision_checker: CollisionChecker | None = None) -> JobResult:
    """The ONLY sanctioned way to write a controller file: validate -> post -> read-back -> atomic write.

    Used by the CLI pipeline and by the GUI (F6). Always writes <name>.manifest.json.
    """
    out_dir = Path(out_dir)
    rep = report if report is not None else ValidationReport()
    collision_checker = collision_checker or CollisionChecker()
    if manifest is None:
        manifest = _new_manifest(post, limits)
        fr = next((i for i in program.instructions if isinstance(i, SetFrame)), None)
        tl = next((i for i in program.instructions if isinstance(i, SetTool)), None)
        if program.robot is not None:
            manifest["cell"] = _cell_info(program.robot, fr.name if fr else None, fr.pose if fr else None,
                                          tl.name if tl else None, tl.pose if tl else None, "program")
    manifest["collision_check"] = _collision_check_info(collision_checker)
    name = program.name

    vrep, _ = validate_program(program, limits, parser_warnings=parser_warnings,
                               mapped_events=set(event_map or ()), collision_checker=collision_checker)
    manifest["event_map"] = dict(event_map or {})
    rep.extend(vrep)
    if not rep.ok:
        return _finish(rep, manifest, out_dir, name, False, [], "✗ Walidacja nie przeszła – eksport zablokowany")

    pp = get_post(post, event_map, tool_load)
    # splitting is OPT-IN (review of Copilot change): a split part needs a restart sequence, and the
    # default must never silently cut a CAM program into pieces
    line_limit = max_lines_per_file
    if line_limit is not None and (not isinstance(line_limit, int) or isinstance(line_limit, bool)
                                   or line_limit < 1):
        rep.add(Severity.ERROR, "PROGRAM_LINE_LIMIT", "Limit linii na plik musi być dodatnią liczbą całkowitą",
                stage="post")
        return _finish(rep, manifest, out_dir, name, False, [], "✗ Nieprawidłowy limit linii na plik")
    manifest["post"]["tool_load"] = _jsonable(tool_load)
    manifest["post"]["max_lines_per_file"] = line_limit
    if tool_load is None:
        if post == "abb":
            message = "Dane obciążenia narzędzia w RAPID są domyślne (1 kg, CoG Z50 mm)"
            hint = "podaj rzeczywistą masę, środek ciężkości i bezwładności przez --tool-load"
        elif post == "kuka":
            message = "Nie podano danych obciążenia narzędzia; KRL nie ustawia $LOAD"
            hint = "podaj masę, środek ciężkości i bezwładności przez --tool-load"
        else:
            message = "Nie podano danych obciążenia narzędzia"
            hint = "podaj dane obciążenia przez --tool-load, jeśli post ich wymaga"
        rep.add(Severity.WARNING, "TOOL_LOAD_DEFAULT", message, stage="post", hint=hint)
    manifest["post"].update({"class": type(pp).__name__, "version": getattr(pp, "version", "?")})
    try:
        if line_limit is None:
            chunks = [(program, pp.generate(program))]
            n_lines = len(chunks[0][1].splitlines())
            if n_lines > pp.MAX_LINES_X_PROG:
                rep.add(Severity.WARNING, "PROGRAM_LONG", f"Program ma {n_lines} linii (> {pp.MAX_LINES_X_PROG}); "
                        "sprawdź limit pamięci sterownika", stage="post",
                        hint="w razie potrzeby podziel: --max-lines-per-file N (z bezpiecznym wznowieniem części)")
        else:
            chunks = pp.generate_chunks(program, line_limit)
    except (ValueError, RuntimeError) as exc:
        rep.add(Severity.ERROR, "PROGRAM_SPLIT", str(exc), stage="post")
        return _finish(rep, manifest, out_dir, name, False, [], "✗ Nie można bezpiecznie podzielić programu")
    if len(chunks) > 1:
        # every part (incl. the inserted retract / approach / plunge moves) is validated on its own
        for part, _ in chunks:
            prep, _ = validate_program(part, limits, mapped_events=set(event_map or ()),
                                       collision_checker=collision_checker)
            for issue in prep.errors:
                issue.message = f"[{part.name}] {issue.message}"
                issue.stage = "split"
                rep.issues.append(issue)
        if not rep.ok:
            return _finish(rep, manifest, out_dir, name, False, [],
                           "✗ Część programu po podziale nie przeszła walidacji – eksport zablokowany")
    for part, text in chunks:
        rep.extend(verify_post_output(post, text, part, limits))
    if not rep.ok:
        return _finish(rep, manifest, out_dir, name, False, [],
                       "✗ Weryfikacja pliku sterownika nie przeszła – eksport zablokowany")
    expected_names = {f"{part.name}{pp.ext}" for part, _ in chunks}
    stale_parts = [path for path in out_dir.glob(f"{name}_*{pp.ext}") if path.name not in expected_names]
    for path in stale_parts:
        rep.add(Severity.WARNING, "STALE_OUTPUT", f"W folderze leży starsza część programu {path.name} "
                "z poprzedniego przebiegu – NIE należy jej uruchamiać",
                stage="post", hint="usuń plik po ręcznym sprawdzeniu folderu eksportu")
    outputs = [atomic_write_text(out_dir / f"{part.name}{pp.ext}", text) for part, text in chunks]
    if len(outputs) > 1:
        rep.add(Severity.INFO, "PROGRAM_SPLIT", f"Program podzielono na {len(outputs)} pliki "
                f"(maks. {line_limit} linii każdy); każda część od 2. zaczyna od PTP nad ścieżką, "
                "włączenia wrzeciona i zagłębienia; każda poza ostatnią kończy odjazdem.", stage="post",
                hint="uruchamiaj części kolejno; pierwszy przebieg w T1")
    msg = f"✓ Zapisano {len(outputs)} plik(ów) (ostrzeżenia: {len(rep.warnings)})"
    return _finish(rep, manifest, out_dir, name, True, outputs, msg)


def run_job(nc_text: str, post: str = "kuka", out_dir: str | Path = "out", name: str = "Machining",
            settings: MachiningSettings | None = None, limits: ValidationLimits | None = None,
            station_path: str | None = None, nc_name: str = "<text>", strict: bool = True,
            event_map: dict[str, str] | None = None, tool_load: ToolLoad | None = None,
            max_lines_per_file: int | None = None,
            collision_checker: CollisionChecker | None = None) -> JobResult:
    """Run the whole pipeline. Never raises for operator-level problems: they land in the report."""
    out_dir = Path(out_dir)
    settings = settings or MachiningSettings()
    rep = ValidationReport()
    manifest = _new_manifest(post, limits)
    manifest["input"] = {"name": nc_name, "sha256": sha256_text(nc_text), "strict_parser": strict}
    manifest["settings"] = _jsonable(settings)
    collision_checker = collision_checker or CollisionChecker()
    manifest["collision_check"] = _collision_check_info(collision_checker)

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
        if "Nieprawidłowy wybór konfiguracji początkowej" in res.message:
            code = "START_CONFIGURATION"
        else:
            code = "CONFIG_JUMP" if "skoku konfiguracji" in res.message else "UNREACHABLE"
        for i in [i for i, r in enumerate(res.reachable) if not r][:20]:
            rep.add(Severity.ERROR, code, f"Punkt ścieżki {i}: " + ("wymaga skoku konfiguracji" if code ==
                    "CONFIG_JUMP" else "nieosiągalny"), stage="solve",
                    source_line=tpa.points[i].line,
                    hint="przesuń układ detalu, zwiększ zakres obrotu Z lub zmień konfigurację startową")
        if code == "START_CONFIGURATION":
            rep.add(Severity.ERROR, code, res.message, stage="solve",
                    hint=f"wybierz indeks z zakresu 1–{len(res.start_configurations)}")
        return fail(res.message)

    # 3+4. validate whole trajectory, post, read back, write
    if settings.start_configuration_index is not None:
        selected = res.start_configurations[settings.start_configuration_index - 1]
        manifest["start_configuration"] = {
            "index": settings.start_configuration_index,
            "orientation_z_deg": selected.orientation_z_deg,
            "joints_deg": _jsonable(selected.joints),
            "label": selected.label,
        }
    return export_program(res.program, post, out_dir, limits, tp.warnings, manifest, rep, event_map, tool_load,
                          max_lines_per_file, collision_checker)


def export_or_raise(job: JobResult) -> list[Path]:
    """For API users who prefer exceptions."""
    if not job.ok:
        raise ExportBlockedError(job.report)
    return job.outputs
