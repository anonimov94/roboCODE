"""ROS 2 trajectory export (docs D9): only after validation, consistent with the URDF joint values."""
import itertools
import json

import numpy as np
import pytest

from robocode.core.io import load_station, save_station
from robocode.demo import DEMO_NC
from robocode.pipeline import run_job
from robocode.robots import load_robot
from robocode.ros import TRAJECTORY_FORMAT, ros_mapping_problem
from robocode.ros.cell import build_kr10_station


@pytest.fixture()
def kr10_station(tmp_path):
    st, *_ = build_kr10_station()
    path = tmp_path / "kr10.rcs.json"
    save_station(st, path)
    return path


def test_example_station_file_matches_builder(tmp_path):
    st = load_station("examples/kr10_cell.rcs.json")
    robot = next(i for i in st.walk() if i.kind == "robot")
    assert robot.model.ros["joint_names"] == [f"joint_{i}" for i in range(1, 7)]
    _, rb, *_ = build_kr10_station()
    assert np.allclose(robot.active_tool.pose, rb.active_tool.pose)


def test_kr10_job_writes_validated_ros_trajectory(kr10_station, tmp_path):
    out = tmp_path / "out"
    job = run_job(DEMO_NC, station_path=str(kr10_station), out_dir=out, ros_trajectory=True)
    assert job.ok, job.report.summary()
    assert not any(i.code == "PROGRAM_SPLIT" for i in job.report.issues)
    traj_path = out / "Machining.ros_trajectory.json"
    assert traj_path in job.outputs
    data = json.loads(traj_path.read_text(encoding="utf-8"))
    assert data["format"] == TRAJECTORY_FORMAT
    assert data["joint_names"] == [f"joint_{i}" for i in range(1, 7)]
    assert data["controller_action"] == "/kr10_arm_controller/follow_joint_trajectory"
    times = [p["t"] for p in data["points"]]
    assert times[0] == 0.0 and all(b > a for a, b in itertools.pairwise(times))
    assert times[-1] == pytest.approx(data["cycle_time_s_estimate"], abs=1e-3)
    model = load_robot("kuka_kr10_r1100_sixx")
    for p in data["points"]:
        q = np.degrees(p["positions"])
        assert model.within_limits(q, margin=-1e-6)
    manifest = json.loads(job.manifest_path.read_text(encoding="utf-8"))
    assert manifest["ros_trajectory"] is True
    assert "Machining.ros_trajectory.json" in {o["file"] for o in manifest["outputs"]}


def test_ros_export_blocked_for_model_without_ros_mapping(tmp_path):
    job = run_job(DEMO_NC, out_dir=tmp_path, ros_trajectory=True)  # built-in demo: synthetic robot
    assert not job.ok
    assert any(i.code == "ROS_TRAJECTORY" for i in job.report.errors)
    assert not list(tmp_path.glob("*.src")) and not list(tmp_path.glob("*.ros_trajectory.json"))


def test_ros_mapping_problem_checks_joint_names():
    m = load_robot("kuka_kr10_r1100_sixx")
    assert ros_mapping_problem(m) is None
    m.ros = dict(m.ros, joint_names=["a"] * 6)
    assert "powtórzenia" in ros_mapping_problem(m)
    m.ros = None
    assert ros_mapping_problem(m)


def test_simulation_timestamps_are_monotonic(kr10_station):
    from robocode.machining.nc_parser import parse_gcode
    from robocode.machining.solver import MachiningSettings, solve
    from robocode.program.simulator import simulate
    st = load_station(kr10_station)
    robot = next(i for i in st.walk() if i.kind == "robot")
    res = solve(robot, robot.frame_rel_base(robot.active_frame), robot.active_tool.tcp(),
                parse_gcode(DEMO_NC), MachiningSettings())
    sim = simulate(res.program, restore_joints=True)
    assert len(sim.times) == len(sim.joints)
    assert all(b >= a for a, b in itertools.pairwise(sim.times))
    assert sim.times[-1] == pytest.approx(sim.cycle_time_s)


def test_exported_file_is_accepted_by_ros_player_parser(kr10_station, tmp_path):
    """Contract between roboCODE (writer) and ros2/robocode_ros (reader)."""
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2" / "robocode_ros"))
    from robocode_ros.trajectory_file import load

    job = run_job(DEMO_NC, station_path=str(kr10_station), out_dir=tmp_path, ros_trajectory=True)
    assert job.ok
    traj = load(tmp_path / "Machining.ros_trajectory.json")
    assert traj.joint_names == [f"joint_{i}" for i in range(1, 7)]
    assert traj.frame_id == "base_link" and len(traj.tcp_path) > 10
