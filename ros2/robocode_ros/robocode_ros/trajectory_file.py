"""Reading roboCODE trajectory files (`robocode-ros-trajectory/1`) – pure Python, no ROS imports.

Kept free of rclpy so it can be tested on any machine (also on Windows next to roboCODE).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

FORMAT = "robocode-ros-trajectory/1"


class TrajectoryFileError(ValueError):
    """The file is not a usable roboCODE trajectory."""


@dataclass
class Trajectory:
    program: str
    joint_names: list[str]
    times: list[float]                 # s, strictly increasing, times[0] == 0
    positions: list[list[float]]       # rad
    controller_action: str | None
    frame_id: str
    tcp_path: list[list[float]]        # m, in frame_id
    cycle_time_s_estimate: float

    def scaled(self, time_scale: float) -> list[float]:
        """Point times multiplied by time_scale (> 0; 2.0 = half speed)."""
        if not (math.isfinite(time_scale) and time_scale > 0):
            raise TrajectoryFileError("time_scale musi być dodatni")
        return [t * time_scale for t in self.times]


def parse(data: dict) -> Trajectory:
    if not isinstance(data, dict) or data.get("format") != FORMAT:
        raise TrajectoryFileError(f"To nie jest plik {FORMAT}")
    names = data.get("joint_names")
    if not isinstance(names, list) or not names or not all(isinstance(n, str) and n for n in names):
        raise TrajectoryFileError("Brak listy joint_names")
    if len(set(names)) != len(names):
        raise TrajectoryFileError("joint_names zawiera powtórzenia")
    points = data.get("points")
    if not isinstance(points, list) or len(points) < 2:
        raise TrajectoryFileError("Trajektoria musi mieć co najmniej 2 punkty")
    times: list[float] = []
    positions: list[list[float]] = []
    for i, p in enumerate(points):
        try:
            t = float(p["t"])
            q = [float(v) for v in p["positions"]]
        except (KeyError, TypeError, ValueError) as exc:
            raise TrajectoryFileError(f"Punkt {i}: niepoprawne dane ({exc})") from exc
        if len(q) != len(names):
            raise TrajectoryFileError(f"Punkt {i}: {len(q)} pozycji, oczekiwano {len(names)}")
        if not (math.isfinite(t) and all(math.isfinite(v) for v in q)):
            raise TrajectoryFileError(f"Punkt {i}: NaN/inf")
        if times and t <= times[-1]:
            raise TrajectoryFileError(f"Punkt {i}: czas {t} nie rośnie")
        times.append(t)
        positions.append(q)
    if abs(times[0]) > 1e-9:
        raise TrajectoryFileError("Pierwszy punkt musi mieć t = 0")
    tcp = data.get("tcp_path") or {}
    return Trajectory(
        program=str(data.get("program", "")),
        joint_names=list(names),
        times=times,
        positions=positions,
        controller_action=data.get("controller_action"),
        frame_id=str(tcp.get("frame_id", "base_link")),
        tcp_path=[[float(v) for v in p] for p in tcp.get("points", [])],
        cycle_time_s_estimate=float(data.get("cycle_time_s_estimate", times[-1])),
    )


def load(path: str | Path) -> Trajectory:
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise TrajectoryFileError(f"Nie można wczytać {path}: {exc}") from exc
    return parse(data)


def approach_duration(current: list[float], target: list[float], max_velocity: float) -> float:
    """Time [s] for a joint move current -> target when no axis exceeds max_velocity [rad/s].

    Used to bring the robot slowly to the first trajectory point before the program starts.
    """
    if len(current) != len(target):
        raise TrajectoryFileError("Różna liczba osi w pozycji bieżącej i docelowej")
    if not (math.isfinite(max_velocity) and max_velocity > 0):
        raise TrajectoryFileError("max_velocity musi być dodatnie")
    delta = max((abs(a - b) for a, b in zip(current, target)), default=0.0)
    return max(delta / max_velocity, 0.5)  # never faster than 0.5 s, even for a tiny correction
