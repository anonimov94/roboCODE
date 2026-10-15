"""Pure-Python tests (no ROS needed): python -m pytest ros2/robocode_ros/test"""
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robocode_ros.trajectory_file import (FORMAT, TrajectoryFileError, approach_duration,  # noqa: E402
                                          load, parse)


def _data(**over):
    d = {"format": FORMAT, "program": "P", "joint_names": ["a", "b"],
         "points": [{"t": 0.0, "positions": [0.0, 0.0]}, {"t": 1.0, "positions": [0.1, 0.2]}],
         "tcp_path": {"frame_id": "base_link", "points": [[0.5, 0.0, 0.3]]},
         "controller_action": "/c/follow_joint_trajectory", "cycle_time_s_estimate": 1.0}
    d.update(over)
    return d


def test_parse_ok_and_scale():
    t = parse(_data())
    assert t.joint_names == ["a", "b"] and t.frame_id == "base_link"
    assert t.scaled(2.0) == [0.0, 2.0]
    with pytest.raises(TrajectoryFileError):
        t.scaled(0.0)


@pytest.mark.parametrize("bad", [
    {"format": "x"},
    {"joint_names": ["a", "a"]},
    {"points": [{"t": 0.0, "positions": [0.0, 0.0]}]},
    {"points": [{"t": 0.0, "positions": [0.0, 0.0]}, {"t": 0.0, "positions": [0.1, 0.1]}]},
    {"points": [{"t": 0.5, "positions": [0.0, 0.0]}, {"t": 1.0, "positions": [0.1, 0.1]}]},
    {"points": [{"t": 0.0, "positions": [0.0]}, {"t": 1.0, "positions": [0.1, 0.1]}]},
    {"points": [{"t": 0.0, "positions": [0.0, math.nan]}, {"t": 1.0, "positions": [0.1, 0.1]}]},
])
def test_parse_rejects_bad_files(bad):
    with pytest.raises(TrajectoryFileError):
        parse(_data(**bad))


def test_load_roundtrip(tmp_path):
    p = tmp_path / "x.ros_trajectory.json"
    p.write_text(json.dumps(_data()), encoding="utf-8")
    assert load(p).program == "P"
    with pytest.raises(TrajectoryFileError):
        load(tmp_path / "missing.json")


def test_approach_duration():
    assert approach_duration([0, 0], [0.4, -1.0], 0.5) == pytest.approx(2.0)
    assert approach_duration([0, 0], [0, 0], 0.5) == 0.5
    with pytest.raises(TrajectoryFileError):
        approach_duration([0], [0, 0], 0.5)
