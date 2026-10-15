import json

import numpy as np
import pytest

from robocode import cli
from robocode.coop.calibration import calibrate_bases, calibrate_tcp
from robocode.core import pose as P


def test_calibrate_cli_writes_tcp_result(tmp_path):
    tcp = np.array([5.0, -3.0, 120.0])
    point = np.array([600.0, 50.0, 200.0])
    poses = []
    for angles in ((0, 0), (20, 0), (0, 25), (-15, 10)):
        pose = P.rotx(angles[0]) @ P.roty(angles[1])
        pose[:3, 3] = point - pose[:3, :3] @ tcp
        poses.append(pose.tolist())

    source = tmp_path / "tcp.json"
    output = tmp_path / "result.json"
    source.write_text(json.dumps({"flange_poses": poses}), encoding="utf-8")

    assert cli.main(["calibrate", "tcp", str(source), "--out", str(output)]) == 0
    result = json.loads(output.read_text(encoding="utf-8"))
    assert result["kind"] == "tcp"
    assert np.allclose(result["tcp_mm"], tcp)
    assert result["rms_mm"] < 1e-6
    assert result["samples"] == 4


def test_calibrate_cli_supports_bases_and_three_point_frame(tmp_path):
    transform = P.transl(150, 20, 30) @ P.rotz(45)
    points_b = [[0, 0, 0], [100, 0, 0], [0, 100, 0], [0, 0, 100]]
    points_a = (transform[:3, :3] @ np.asarray(points_b).T).T + transform[:3, 3]
    bases_source = tmp_path / "bases.json"
    bases_source.write_text(json.dumps({
        "points_in_A": points_a.tolist(),
        "points_in_B": points_b,
    }), encoding="utf-8")
    bases_output = tmp_path / "bases-result.json"

    assert cli.main(["calibrate", "bases", str(bases_source), "--out", str(bases_output)]) == 0
    bases = json.loads(bases_output.read_text(encoding="utf-8"))
    assert np.allclose(bases["T_A_B"], transform)
    assert bases["rms_mm"] < 1e-6

    frame_source = tmp_path / "frame.json"
    frame_source.write_text(json.dumps({
        "origin": [10, 20, 30], "x_point": [20, 20, 30], "xy_point": [13, 25, 30],
    }), encoding="utf-8")
    frame_output = tmp_path / "frame-result.json"
    assert cli.main(["calibrate", "frame3", str(frame_source), "--out", str(frame_output)]) == 0
    frame = json.loads(frame_output.read_text(encoding="utf-8"))
    assert np.allclose(frame["T_base_frame"], P.transl(10, 20, 30))


def test_calibrate_cli_rejects_invalid_data_without_output(tmp_path):
    source = tmp_path / "degenerate.json"
    output = tmp_path / "must-not-exist.json"
    source.write_text(json.dumps({
        "origin": [0, 0, 0], "x_point": [1, 0, 0], "xy_point": [2, 0, 0],
    }), encoding="utf-8")

    assert cli.main(["calibrate", "frame3", str(source), "--out", str(output)]) == 2
    assert not output.exists()

    source.write_text("{ invalid json", encoding="utf-8")
    assert cli.main(["calibrate", "frame3", str(source), "--out", str(output)]) == 2
    assert not output.exists()


def test_calibration_rejects_degenerate_measurements():
    collinear = [[0, 0, 0], [1, 0, 0], [2, 0, 0]]
    with pytest.raises(ValueError, match="niewspółliniowe"):
        calibrate_bases(collinear, collinear)
    with pytest.raises(ValueError, match="różnorodności obrotów"):
        calibrate_tcp([np.eye(4), np.eye(4), np.eye(4)])
