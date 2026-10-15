"""Validated joint trajectory -> JSON for the ROS 2 player (ros2/robocode_ros).

Format `robocode-ros-trajectory/1` (all angles in radians, times in seconds, lengths in metres):

{
  "format": "robocode-ros-trajectory/1",
  "program": "<name>",
  "robot": {"name": ..., "model_sha256": ..., "description_package": ..., "base_link": ...},
  "controller_action": "/kr10_arm_controller/follow_joint_trajectory",
  "joint_names": ["joint_1", ...],
  "points": [{"t": 0.0, "positions": [...]}, ...],     # strictly increasing t
  "tcp_path": {"frame_id": "base_link", "points": [[x, y, z], ...]},
  "cycle_time_s_estimate": 31.5,
  "notes": [...]
}

Timing is the roboCODE estimate (docs §6.6), not the controller's real timing.
"""
from __future__ import annotations

import math

import numpy as np

from ..core import pose as P
from ..core.fileio import sha256_text

TRAJECTORY_FORMAT = "robocode-ros-trajectory/1"
_MIN_DT = 1e-4  # s; samples closer than this are the same instant (MoveJ start repeats the previous end)


def ros_mapping_problem(model) -> str | None:
    """Why this robot model cannot be exported to ROS 2, or None when it can."""
    ros = getattr(model, "ros", None)
    if not ros:
        return (f"Model robota '{model.name}' nie ma powiązania z ROS 2 (pole 'ros' w .robot.json); "
                "użyj np. kuka_kr10_r1100_sixx")
    names = ros.get("joint_names")
    if not isinstance(names, list) or len(names) != model.n or not all(isinstance(n, str) and n for n in names):
        return f"Model '{model.name}': ros.joint_names musi zawierać {model.n} nazw osi"
    if len(set(names)) != len(names):
        return f"Model '{model.name}': ros.joint_names zawiera powtórzenia"
    return None


def build_trajectory(program, sim) -> dict:
    """Build the JSON-able trajectory from a successful simulation of `program`.

    Raises ValueError when the data cannot form a valid trajectory; callers turn that into a
    blocking report entry (no file is written).
    """
    robot = program.robot
    model = robot.model
    problem = ros_mapping_problem(model)
    if problem:
        raise ValueError(problem)
    if sim is None or not sim.ok:
        raise ValueError("Brak poprawnej symulacji – trajektoria ROS nie powstanie")
    if len(sim.times) != len(sim.joints) or not sim.joints:
        raise ValueError("Symulacja nie zawiera próbek z czasem")

    points: list[dict] = []
    last_t = -math.inf
    for t, q in zip(sim.times, sim.joints):
        q = np.asarray(q, float)
        if not (math.isfinite(t) and np.all(np.isfinite(q))):
            raise ValueError("Próbka trajektorii zawiera NaN/inf")
        if not model.within_limits(q, margin=-1e-6):
            raise ValueError(f"Próbka poza limitami osi: {np.round(q, 3).tolist()}")
        if t - last_t < _MIN_DT:
            if points and np.abs(np.radians(q) - np.asarray(points[-1]["positions"])).max() > 1e-6:
                # same instant but a different position would be an infinite velocity
                raise ValueError(f"Dwie różne pozycje osi w tej samej chwili t={t:.4f} s")
            continue
        points.append({"t": round(float(t), 6), "positions": [round(float(v), 9) for v in np.radians(q)]})
        last_t = t

    base_inv = P.inv(robot.pose_abs())
    tcp = [[round(float(v) / 1000.0, 6) for v in (base_inv @ np.append(p, 1.0))[:3]] for p in sim.trace]
    ros = model.ros
    return {
        "format": TRAJECTORY_FORMAT,
        "program": program.name,
        "robot": {"name": model.name, "model_sha256": sha256_text(model.to_json()),
                  "description_package": ros.get("description_package"),
                  "base_link": ros.get("base_link", "base_link")},
        "controller_action": ros.get("controller_action"),
        "joint_names": list(ros["joint_names"]),
        "points": points,
        "tcp_path": {"frame_id": ros.get("base_link", "base_link"), "points": tcp},
        "cycle_time_s_estimate": round(float(sim.cycle_time_s), 3),
        "notes": [
            "Czasy to estymata roboCODE (profil trapezowy), nie czas sterownika.",
            "Trajektoria do symulacji (Gazebo / mock). Nie jest dopuszczona do sterowania robotem.",
        ],
    }
