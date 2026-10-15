import json

import numpy as np
import pytest

from robocode.collision import CollisionChecker
from robocode.core import pose as P
from robocode.core.scene import Object, Robot, Station
from robocode.kinematics.robot_model import example_6r
from robocode.program.instructions import MoveJ, Program
from robocode.program.simulator import simulate
from robocode.demo import DEMO_NC
from robocode.pipeline import run_job


def station_with_box(visible=True):
    station = Station("Station")
    model = example_6r()
    robot = Robot(name="Robot", model=model)
    station.add(robot)
    joint = model.fk_all(robot.joints)[1][:3, 3]
    obstacle = station.add(Object(
        name="Fixture", pose=P.transl(*(joint - 5)), box_size=(10, 10, 10), visible=visible,
    ))
    return station, robot, obstacle


def test_collision_checker_detects_link_box_and_honors_pair_allowlist():
    station, robot, _ = station_with_box()

    hits = CollisionChecker(link_radius_mm=1).check(station, robot)
    assert any(hit.obstacle == "Fixture" for hit in hits)
    assert CollisionChecker(pairs=[], link_radius_mm=1).check(station, robot) == []
    assert CollisionChecker(pairs=[("Robot/link_1", "Fixture")], link_radius_mm=1).check(station, robot)


def test_collision_checker_ignores_hidden_objects():
    station, robot, _ = station_with_box(visible=False)
    assert CollisionChecker(link_radius_mm=1).check(station, robot) == []


@pytest.mark.parametrize("pairs", ["Robot/link_1", [("Robot/link_1",)], [("Robot/link_1", "")]])
def test_collision_checker_rejects_malformed_pairs(pairs):
    with pytest.raises(ValueError):
        CollisionChecker(pairs=pairs)  # type: ignore[arg-type]


@pytest.mark.parametrize("radius", [0, -1, np.inf, "invalid", True])
def test_collision_checker_rejects_invalid_proxy_radius(radius):
    with pytest.raises(ValueError, match="link_radius_mm"):
        CollisionChecker(link_radius_mm=radius)  # type: ignore[arg-type]


def test_simulator_reports_proxy_collision_at_sampled_joint_configuration():
    station, robot, _ = station_with_box()
    program = Program(name="Collision test", robot=robot)
    program.append(MoveJ(joints=robot.joints.copy()))

    result = simulate(program, collision_checker=CollisionChecker(link_radius_mm=1))

    assert any(issue.code == "COLLISION_PROXY" for issue in result.report.errors)  # blocking
    assert not result.ok


def test_collision_checker_rejects_invalid_obstacle_size():
    station = Station("Station")
    robot = Robot(name="Robot", model=example_6r())
    station.add(robot)
    station.add(Object(name="Broken fixture", box_size=(0, 10, 10)))
    with pytest.raises(ValueError, match="Broken fixture"):
        CollisionChecker().check(station, robot)


def test_pipeline_records_collision_map_in_manifest(tmp_path):
    checker = CollisionChecker(
        pairs=[("Robot/link_2", "Fixture"), ("Robot/tool:Tool", "Fixture")], link_radius_mm=18,
    )
    job = run_job(DEMO_NC, out_dir=tmp_path, collision_checker=checker)

    assert job.ok, job.report.summary()
    assert job.manifest_path is not None
    config = json.loads(job.manifest_path.read_text(encoding="utf-8"))["collision_check"]
    assert config["method"] == "link_capsules_vs_axis_aligned_boxes"
    assert config["link_radius_mm"] == 18
    assert config["pairs"] == [["Robot/link_2", "Fixture"], ["Robot/tool:Tool", "Fixture"]]
    assert config["tool_proxy_enabled"]


def test_detected_collision_blocks_export(tmp_path):
    """Review of the Copilot change: a detected collision must block the controller file."""
    from robocode.core.scene import Object
    from robocode.demo import build_demo_station
    from robocode.pipeline import export_program
    from robocode.machining.nc_parser import parse_gcode
    from robocode.machining.solver import MachiningSettings, solve
    st, r, fr, tool = build_demo_station()
    prog = solve(r, r.frame_rel_base(fr), tool.tcp(), parse_gcode(DEMO_NC), MachiningSettings()).program
    # a big block around the robot's upper arm
    st.add(Object(name="Wall", pose=P.transl(-2000, -2000, 300), box_size=(4000, 4000, 2000)))
    job = export_program(prog, "kuka", tmp_path)
    assert not job.ok and any(i.code == "COLLISION_PROXY" for i in job.report.errors)
    assert not list(tmp_path.glob("*.src"))
