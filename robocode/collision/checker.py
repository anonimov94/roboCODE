"""Conservative capsule-vs-box collision proxies for sampled robot configurations.

Robot links are represented by capsules between DH frame origins. This is useful for early
clearance checks, but is not a replacement for OEM geometry or a safety-rated collision monitor.
"""
from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np

from ..core import pose as P
from ..core.scene import Item, Object, Robot, Station
from ..kinematics.robot_model import RobotModel


@dataclass(frozen=True)
class CollisionHit:
    robot: str
    component: str
    obstacle: str

    @property
    def pair(self) -> tuple[str, str]:
        return f"{self.robot}/{self.component}", self.obstacle


def _segment_box_distance_sq(start: np.ndarray, end: np.ndarray,
                             lower: np.ndarray, upper: np.ndarray) -> float:
    """Exact minimum squared distance from a segment to an axis-aligned box."""
    direction = end - start
    breaks = [0.0, 1.0]
    for axis in range(3):
        if abs(direction[axis]) > 1e-15:
            for bound in (lower[axis], upper[axis]):
                t = (bound - start[axis]) / direction[axis]
                if 0.0 < t < 1.0:
                    breaks.append(float(t))
    breaks = sorted(set(breaks))
    best = float("inf")

    def distance_sq(t: float) -> float:
        point = start + t * direction
        delta = np.maximum(np.maximum(lower - point, point - upper), 0.0)
        return float(delta @ delta)

    for left, right in zip(breaks, breaks[1:]):
        middle = (left + right) * 0.5
        sample = start + middle * direction
        active = (sample < lower) | (sample > upper)
        if not np.any(active):
            return 0.0
        bounds = np.where(sample < lower, lower, upper)
        offset = start[active] - bounds[active]
        slope = direction[active]
        denom = float(slope @ slope)
        optimum = left if denom == 0 else float(np.clip(-(offset @ slope) / denom, left, right))
        best = min(best, distance_sq(left), distance_sq(right), distance_sq(optimum))
    if len(breaks) == 1:
        return distance_sq(breaks[0])
    return best


class CollisionChecker:
    """Check robot link capsules against box objects in the same station tree.

    `pairs` is an optional allow-list of `(robot/component, object)` names, such as
    `("Robot/link_2", "Fixture")`. With no allow-list, every link/object pair is checked.
    Mesh objects are deliberately not treated as boxes; callers should report them unchecked.
    """

    def __init__(self, pairs: Iterable[tuple[str, str]] | None = None,
                 link_radius_mm: float = 28.0, tool_radius_mm: float = 10.0):
        link_radius_mm = self._positive_finite(link_radius_mm, "link_radius_mm")
        tool_radius_mm = self._positive_finite(tool_radius_mm, "tool_radius_mm")
        self.pairs: set[tuple[str, str]] | None = None if pairs is None else self._normalize_pairs(pairs)
        self.link_radius_mm = link_radius_mm
        self.tool_radius_mm = tool_radius_mm

    @staticmethod
    def _positive_finite(value: float, name: str) -> float:
        if isinstance(value, (bool, np.bool_)):
            raise ValueError(f"{name} musi być skończone i > 0")
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} musi być skończone i > 0") from exc
        if not np.isfinite(number) or number <= 0:
            raise ValueError(f"{name} musi być skończone i > 0")
        return number

    @staticmethod
    def _normalize_pairs(pairs: Iterable[tuple[str, str]]) -> set[tuple[str, str]]:
        if isinstance(pairs, (str, bytes)):
            raise ValueError("Pary kolizji muszą zawierać nazwy komponentu i przeszkody")
        try:
            normalized = set()
            for pair in pairs:
                if isinstance(pair, (str, bytes)):
                    raise ValueError("Pary kolizji muszą zawierać nazwy komponentu i przeszkody")
                normalized.add(tuple(pair))
        except TypeError as exc:
            raise ValueError("Pary kolizji muszą zawierać nazwy komponentu i przeszkody") from exc
        if any(len(pair) != 2 or not all(isinstance(value, str) and value for value in pair)
               for pair in normalized):
            raise ValueError("Pary kolizji muszą zawierać nazwy komponentu i przeszkody")
        return normalized

    def set_pairs(self, pairs: Iterable[tuple[str, str]]) -> None:
        self.pairs = self._normalize_pairs(pairs)

    def _enabled(self, robot: Robot, component: str, obstacle: Object) -> bool:
        if component.startswith("tool:") and self.pairs is None:
            return False
        return self.pairs is None or (f"{robot.name}/{component}", obstacle.name) in self.pairs

    @staticmethod
    def _objects(root: Item) -> list[Object]:
        return [item for item in root.walk() if isinstance(item, Object) and item.visible]

    def check(self, station: Station, robot: Robot | None = None, joints=None) -> list[CollisionHit]:
        """Return proxy collisions for a robot configuration (or current station configuration)."""
        root: Item = station
        parent = root.parent
        while parent is not None:
            root = parent
            parent = root.parent
        if robot is None:
            robot = next((item for item in root.walk() if isinstance(item, Robot)), None)
        if robot is None or not isinstance(robot.model, RobotModel):
            return []
        model = robot.model
        if not any(item is robot for item in root.walk()):
            raise ValueError("Robot musi należeć do sprawdzanej stacji")
        q = robot.joints if joints is None else np.asarray(joints, dtype=float)
        if q.shape != (model.n,) or not np.all(np.isfinite(q)):
            raise ValueError("Konfiguracja osi do testu kolizji jest nieprawidłowa")

        frames = [robot.pose_abs() @ frame for frame in model.fk_all(q)]
        links = [(f"link_{index + 1}", frames[index][:3, 3], frames[index + 1][:3, 3],
                  self.link_radius_mm) for index in range(len(frames) - 1)]
        flange = frames[-1]
        tool = robot.active_tool
        if tool is not None:
            tcp = flange @ tool.tcp()
            links.append((f"tool:{tool.name}", flange[:3, 3], tcp[:3, 3], self.tool_radius_mm))

        hits = []
        for obstacle in self._objects(root):
            if obstacle.box_size is None:
                continue
            size = np.asarray(obstacle.box_size, dtype=float)
            if size.shape != (3,) or not np.all(np.isfinite(size)) or np.any(size <= 0):
                raise ValueError(f"Nieprawidłowy rozmiar prostopadłościanu: {obstacle.name}")
            obstacle_pose = obstacle.pose_abs()
            inverse = P.inv(obstacle_pose)
            for component, start, end, radius in links:
                if not self._enabled(robot, component, obstacle):
                    continue
                start_local = (inverse @ np.append(start, 1.0))[:3]
                end_local = (inverse @ np.append(end, 1.0))[:3]
                distance_sq = _segment_box_distance_sq(start_local, end_local, np.zeros(3), size)
                if distance_sq <= radius * radius:
                    hits.append(CollisionHit(robot.name, component, obstacle.name))
        return hits
