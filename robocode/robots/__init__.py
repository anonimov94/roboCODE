"""Robot library: *.robot.json files (DH + limits + source + license). docs §6.2."""
from __future__ import annotations

from pathlib import Path

from ..kinematics.robot_model import RobotModel

LIB = Path(__file__).parent


def available() -> list[str]:
    return sorted(p.name[: -len(".robot.json")] for p in LIB.glob("*.robot.json"))


def load_robot(name_or_path: str) -> RobotModel:
    p = Path(name_or_path)
    if not p.exists():
        p = LIB / f"{name_or_path}.robot.json"
    if not p.exists():
        raise FileNotFoundError(f"Brak modelu robota '{name_or_path}'. Dostępne: {', '.join(available())}")
    m = RobotModel.from_json(p.read_text(encoding="utf-8"))
    if not m.source or not m.license:
        raise ValueError(f"{p.name}: model robota musi mieć pola 'source' i 'license'")
    return m
