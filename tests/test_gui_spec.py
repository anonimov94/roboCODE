import json

import numpy as np

from robocode.gui import actions as A
from robocode.gui.handlers import AppState, cancel_pending_calibration, dispatch
from robocode.demo import build_demo_station
from robocode.gui.main_window import _reachability_text, _report_text
from robocode.core import pose as P


def test_gui_definition_consistent():
    assert A.validate() == []


def test_required_shortcuts_present():
    sc = {a.shortcut for a in A.ACTIONS}
    for s in ["F1", "F2", "F6", "Shift+F6", "Ctrl+F6", "F7", "Alt+0", "Ctrl+1", "Ctrl+T", "Ctrl+Shift+T",
              "Ctrl+M", "Ctrl+Z", "Ctrl+Y", "Space", "Backspace", "Esc"]:
        assert s in sc, s
    assert A.BY_ID["cnc.calib_tcp"].command == "calibrate_tcp_gui"
    assert A.BY_ID["cnc.calib_frame"].command == "calibrate_frame_gui"


def test_handlers_flow(tmp_path):
    s = AppState()
    s.station, *_ = build_demo_station()
    assert "Dodano" in dispatch(s, "add_frame")
    assert "Cofnięto" == dispatch(s, "undo")
    dispatch(s, "add_program")
    dispatch(s, "add_movej")
    dispatch(s, "add_movel")
    msg = dispatch(s, "generate_program", str(tmp_path))
    assert "Zapisano" in msg and (tmp_path / "Prog1.manifest.json").exists()
    assert "niezaimplementowana" in dispatch(s, None)
    assert "Zapisano" in dispatch(s, "save_station_as", str(tmp_path / "x.rcs.json"))


def test_machining_project_updates_validation_report_and_panel_text():
    state = AppState()
    assert "Demo obróbki" in dispatch(state, "open_demo")
    report = state.last_report
    assert report is not None and report.ok
    text = _report_text(report)
    assert "Walidacja: OK" in text
    assert "PARSER" in text
    assert "COLLISION_NOT_CHECKED" in text
    assert "czas cyklu" in text

    assert "Program gotowy" in dispatch(state, "update_machining")
    assert state.last_report is not report
    assert "próbek trajektorii" in _report_text(state.last_report)
    assert "Brak raportu walidacji" in _report_text(None)


def test_reachability_summary_reports_all_and_unreachable_samples():
    from types import SimpleNamespace

    assert _reachability_text(None) == "Osiągalność: brak obliczonego projektu obróbki."
    project = SimpleNamespace(result=SimpleNamespace(reachable=[True, True, True]))
    assert "3/3 próbek" in _reachability_text(project)
    assert "wszystkie osiągalne" in _reachability_text(project)
    project.result.reachable = [True, False, False, True, False, False, False]
    summary = _reachability_text(project)
    assert "2/7 próbek" in summary
    assert "2, 3, 5, 6, 7" in summary
    assert "czerwone punkty" in summary


def test_gui_collision_toggle_controls_simulation_checker(monkeypatch):
    from robocode.program import simulator

    state = AppState()
    dispatch(state, "open_demo")
    real_simulate = simulator.simulate
    seen_checkers = []

    def capture_checker(program, **kwargs):
        checker = kwargs["collision_checker"]
        seen_checkers.append(checker)
        return real_simulate(program, **kwargs)

    monkeypatch.setattr(simulator, "simulate", capture_checker)
    assert state.collisions_enabled
    dispatch(state, "simulate_program")
    assert seen_checkers[-1].pairs is None

    assert "wył." in dispatch(state, "toggle_collisions")
    dispatch(state, "simulate_program")
    assert seen_checkers[-1].pairs == set()
    assert "wł." in dispatch(state, "toggle_collisions")


def test_gui_collision_toggle_is_passed_to_exporter(monkeypatch, tmp_path):
    from types import SimpleNamespace
    from robocode import pipeline
    from robocode.program.report import ValidationReport

    state = AppState()
    dispatch(state, "open_demo")
    captured = {}

    def capture_export(program, post, out_dir, **kwargs):
        captured.update(kwargs)
        return SimpleNamespace(
            ok=True, report=ValidationReport(), message="Zapisano",
            outputs=[tmp_path / "Machining.src"],
        )

    monkeypatch.setattr(pipeline, "export_program", capture_export)
    state.collisions_enabled = False
    dispatch(state, "generate_program", str(tmp_path))
    assert captured["collision_checker"].pairs == set()


def test_simulation_timeline_seek_and_stop_restore_initial_robot_pose():
    state = AppState()
    dispatch(state, "open_demo")
    robot = state.robot()
    initial_joints = robot.joints.copy()

    assert "Symulacja OK" in dispatch(state, "simulate_program")
    assert state.last_sim.ok and len(state.last_sim.joints) > 1
    assert np.allclose(robot.joints, state.last_sim.joints[0])

    last_index = len(state.last_sim.joints) - 1
    assert "symulacji" in dispatch(state, "seek_simulation", last_index)
    assert state.simulation_index == last_index
    assert np.allclose(robot.joints, state.last_sim.joints[last_index])
    assert dispatch(state, "pause_sim") == "Odtwarzanie symulacji"
    assert state.simulation_playing and state.simulation_index == 0
    assert dispatch(state, "pause_sim") == "Symulacja wstrzymana"
    assert not state.simulation_playing
    assert "pozycję początkową" in dispatch(state, "stop_simulation")
    assert state.simulation_index == 0 and not state.simulation_playing
    assert np.allclose(robot.joints, initial_joints)
    assert "poza zakresem" in dispatch(state, "seek_simulation", len(state.last_sim.joints))


def test_simulation_timeline_widget_seeks_and_plays(monkeypatch):
    import pytest

    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtTest import QTest
    from robocode.gui.main_window import build_window

    state = AppState()
    dispatch(state, "open_demo")
    app, window = build_window(state)
    try:
        collision_action = window.qactions["tools.collisions"]
        assert collision_action.isChecked()
        collision_action.trigger()
        assert not state.collisions_enabled and not collision_action.isChecked()
        collision_action.trigger()
        assert state.collisions_enabled and collision_action.isChecked()

        assert "Symulacja OK" in dispatch(state, "simulate_program")
        window.refresh_all()
        assert window.simulation_slider.isEnabled()
        assert window.simulation_slider.maximum() == len(state.last_sim.joints) - 1
        interval_at_5x = window.simulation_timer.interval()
        window.simulation_speed_combo.setCurrentIndex(3)
        assert window.simulation_speed_combo.currentData() == 100
        assert window.simulation_timer.interval() < interval_at_5x
        expected_100x_interval = max(
            1, round(state.last_sim.cycle_time_s * 1000 / (len(state.last_sim.joints) - 1) / 100)
        )
        assert window.simulation_timer.interval() == expected_100x_interval
        window.simulation_speed_combo.setCurrentIndex(1)
        window.qactions["prog.fast_sim"].trigger()
        assert window.simulation_speed_combo.currentData() == 100
        window.qactions["prog.fast_sim"].trigger()
        assert window.simulation_speed_combo.currentData() == 5

        window.simulation_slider.setValue(1)
        assert state.simulation_index == 1
        assert np.allclose(state.robot().joints, state.last_sim.joints[1])

        window.simulation_play_button.click()
        assert state.simulation_playing and window.simulation_timer.isActive()
        QTest.qWait(window.simulation_timer.interval() + 30)
        assert state.simulation_index > 1
        window.qactions["prog.pause_sim"].trigger()
        assert not state.simulation_playing and not window.simulation_timer.isActive()
        window.qactions["prog.pause_sim"].trigger()
        assert state.simulation_playing and window.simulation_timer.isActive()
        window.qactions["prog.stop_sim"].trigger()
        assert not state.simulation_playing and not window.simulation_timer.isActive()
        assert np.allclose(state.robot().joints, state.simulation_start_joints)
    finally:
        window.viewport.close()
        window.close()
        app.quit()


def test_changing_station_clears_stale_machining_report():
    state = AppState()
    dispatch(state, "open_demo")
    assert state.project is not None and state.last_report is not None
    assert "Symulacja OK" in dispatch(state, "simulate_program")

    dispatch(state, "new_station")

    assert state.project is None and state.last_report is None and state.overlay is None
    assert state.last_sim is None and state.simulation_robot is None


def test_invalid_machining_input_is_shown_as_validation_error(tmp_path):
    nc = tmp_path / "invalid.nc"
    nc.write_text("G21\nG41 D1\n", encoding="utf-8")
    state = AppState()

    assert "Plik NC" in dispatch(state, "new_machining_project", str(nc))
    assert state.last_report is not None and not state.last_report.ok
    text = _report_text(state.last_report)
    assert "NC_PARSE" in text
    assert "BŁĘDY" in text


def test_edit_item_properties_is_atomic_undoable_and_invalidates_project():
    state = AppState()
    state.station, robot, frame, _ = build_demo_station()
    old_name, old_pose = frame.name, frame.pose.copy()
    new_pose = P.transl(123, 456, 789) @ P.rotz(30)
    state.selection = [frame]

    message = dispatch(state, "edit_item_properties", "Fixture frame", new_pose)

    assert "unieważniony" in message
    assert frame.name == "Fixture frame" and np.allclose(frame.pose, new_pose)
    assert dispatch(state, "undo") == "Cofnięto"
    assert frame.name == old_name and np.allclose(frame.pose, old_pose)
    assert dispatch(state, "redo") == "Ponowiono"
    assert frame.name == "Fixture frame" and np.allclose(frame.pose, new_pose)


def test_edit_item_properties_rejects_empty_duplicate_names_and_invalid_poses():
    state = AppState()
    state.station, robot, frame, _ = build_demo_station()
    old_pose = frame.pose.copy()
    state.selection = [frame]

    assert "nie może być pusta" in dispatch(state, "edit_item_properties", "  ", old_pose)
    assert "już istnieje" in dispatch(state, "edit_item_properties", robot.name, old_pose)
    assert "poprawną macierzą" in dispatch(state, "edit_item_properties", "Fixture frame", np.zeros((4, 4)))
    assert frame.name == "Detal" and np.allclose(frame.pose, old_pose)


def test_property_editor_button_updates_scene_and_supports_undo(monkeypatch):
    import pytest

    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QDialog, QDoubleSpinBox, QLineEdit
    from robocode.gui.main_window import build_window

    state = AppState()
    state.station, _, frame, _ = build_demo_station()
    app, window = build_window(state)

    def accept_editor():
        dialog = app.activeModalWidget()
        assert isinstance(dialog, QDialog)
        dialog.findChild(QLineEdit).setText("Edited frame")
        spinboxes = dialog.findChildren(QDoubleSpinBox)
        spinboxes[0].setValue(123.0)
        spinboxes[1].setValue(0.0)
        spinboxes[2].setValue(0.0)
        spinboxes[5].setValue(30.0)
        dialog.accept()

    try:
        frame_widget = window.station_item_widgets[frame.id]
        window.station_tree.setCurrentItem(frame_widget)
        assert window.edit_properties_button.isEnabled()
        QTimer.singleShot(0, accept_editor)
        window.edit_properties_button.click()

        assert frame.name == "Edited frame"
        assert np.allclose(frame.pose, P.from_xyzabc(123, 0, 0, 0, 0, 30))
        assert dispatch(state, "undo") == "Cofnięto"
        assert frame.name == "Detal"
    finally:
        window.viewport.close()
        window.close()
        app.quit()


def test_3d_pick_selects_item_in_tree_and_properties_panel(monkeypatch):
    import pytest

    pytest.importorskip("PySide6")
    try:
        from pyvistaqt import QtInteractor  # noqa: F401
    except ImportError as exc:
        pytest.skip(f"pyvistaqt renderer unavailable: {exc}")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from robocode.gui.main_window import build_window

    state = AppState()
    state.station, _, frame, _ = build_demo_station()
    app, window = build_window(state)

    try:
        actor = next(
            actor for name, actor in window.viewport.plotter.renderer.actors.items()
            if name.startswith(f"robocode-item-{frame.id}-")
        )
        window.viewport._on_mesh_picked(actor)

        assert state.selection == [frame]
        assert window.station_tree.currentItem() is window.station_item_widgets[frame.id]
        assert window.edit_properties_button.isEnabled()
        assert "Detal" in window.properties_text.text()
    finally:
        window.viewport.close()
        window.close()
        app.quit()


def test_main_window_displays_machining_validation_report(monkeypatch):
    import pytest

    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from robocode.gui.main_window import build_window

    state = AppState()
    app, window = build_window(state)
    try:
        dispatch(state, "open_demo")
        window.refresh_all()
        assert window.machining_report_panel.toPlainText() == _report_text(state.last_report)
        assert "wszystkie osiągalne" in window.machining_reachability_label.text()
        assert "COLLISION_NOT_CHECKED" in window.machining_report_panel.toPlainText()
        assert window.machining_report_dock.windowTitle() == "Projekt obróbki"
    finally:
        window.viewport.close()
        window.close()
        app.quit()


def test_gui_calibration_action_confirms_and_applies_result(monkeypatch, tmp_path):
    import pytest

    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QFileDialog, QMessageBox
    from robocode.gui.main_window import build_window

    state = AppState()
    state.station, robot, frame, _ = build_demo_station()
    expected = P.transl(100, 200, 300)
    source = tmp_path / "frame.json"
    source.write_text(json.dumps({
        "origin": [100, 200, 300], "x_point": [110, 200, 300], "xy_point": [103, 205, 300],
    }), encoding="utf-8")
    monkeypatch.setattr(QFileDialog, "getOpenFileName", lambda *args: (str(source), "JSON"))
    questions = []

    def confirm(*args):
        questions.append(args[2])
        return QMessageBox.Yes

    monkeypatch.setattr(QMessageBox, "question", confirm)
    app, window = build_window(state)
    try:
        window.qactions["cnc.calib_frame"].trigger()
        assert len(questions) == 1 and "Układ Detal" in questions[0]
        assert np.allclose(robot.frame_rel_base(frame), expected)
        assert "Zastosowano" in window.statusBar().currentMessage()
        assert dispatch(state, "undo") == "Cofnięto"
        assert not np.allclose(robot.frame_rel_base(frame), expected)
    finally:
        window.viewport.close()
        window.close()
        app.quit()


def test_gui_tcp_calibration_requires_confirmation_and_supports_undo(tmp_path):
    state = AppState()
    state.station, robot, _, tool = build_demo_station()
    tcp = np.array([5.0, -3.0, 120.0])
    point = np.array([600.0, 50.0, 200.0])
    poses = []
    for angles in ((0, 0), (20, 0), (0, 25), (-15, 10)):
        flange = P.rotx(angles[0]) @ P.roty(angles[1])
        flange[:3, 3] = point - flange[:3, :3] @ tcp
        poses.append(flange.tolist())
    source = tmp_path / "tcp.json"
    source.write_text(json.dumps({"flange_poses": poses}), encoding="utf-8")
    old_pose = tool.pose.copy()

    message = dispatch(state, "calibrate_tcp_gui", str(source))
    assert "RMS" in message and state.pending_calibration is not None
    assert np.allclose(tool.pose, old_pose)

    message = dispatch(state, "apply_pending_calibration")
    assert "Zastosowano" in message
    assert np.allclose(tool.pose[:3, 3], tcp)
    assert dispatch(state, "undo") == "Cofnięto"
    assert np.allclose(tool.pose, old_pose)
    assert dispatch(state, "redo") == "Ponowiono"
    assert np.allclose(tool.pose[:3, 3], tcp)


def test_gui_frame_calibration_updates_relative_pose_and_is_undoable(tmp_path):
    state = AppState()
    state.station, robot, frame, _ = build_demo_station()
    robot.pose = P.transl(100, 0, 0)
    frame_in_base = P.transl(500, 200, 300) @ P.rotz(90)
    source = tmp_path / "frame.json"
    source.write_text(json.dumps({
        "origin": frame_in_base[:3, 3].tolist(),
        "x_point": (frame_in_base[:3, 3] + frame_in_base[:3, 0] * 100).tolist(),
        "xy_point": (frame_in_base[:3, 3] + frame_in_base[:3, 0] * 30
                     + frame_in_base[:3, 1] * 50).tolist(),
    }), encoding="utf-8")
    old_pose = frame.pose.copy()

    assert "3 punktów" in dispatch(state, "calibrate_frame_gui", str(source))
    assert dispatch(state, "apply_pending_calibration").startswith("Zastosowano")
    assert np.allclose(robot.frame_rel_base(frame), frame_in_base)
    assert dispatch(state, "undo") == "Cofnięto"
    assert np.allclose(frame.pose, old_pose)
    assert dispatch(state, "redo") == "Ponowiono"
    assert np.allclose(robot.frame_rel_base(frame), frame_in_base)


def test_gui_frame_calibration_supports_corresponding_point_sets(tmp_path):
    state = AppState()
    state.station, robot, frame, _ = build_demo_station()
    transform = P.transl(150, 20, 30) @ P.rotz(45)
    points_b = np.array([[0, 0, 0], [100, 0, 0], [0, 100, 0], [0, 0, 100]], dtype=float)
    points_a = (transform[:3, :3] @ points_b.T).T + transform[:3, 3]
    source = tmp_path / "bases.json"
    source.write_text(json.dumps({
        "points_in_A": points_a.tolist(), "points_in_B": points_b.tolist(),
    }), encoding="utf-8")

    assert "RMS" in dispatch(state, "calibrate_frame_gui", str(source))
    assert dispatch(state, "apply_pending_calibration").startswith("Zastosowano")
    assert np.allclose(robot.frame_rel_base(frame), transform, atol=1e-9)


def test_gui_calibration_rejects_invalid_input_and_cancel_is_non_mutating(tmp_path):
    state = AppState()
    state.station, _, frame, tool = build_demo_station()
    old_pose = tool.pose.copy()
    source = tmp_path / "bad.json"
    source.write_text(json.dumps({"flange_poses": [[[1, 0], [0, 1]]]}), encoding="utf-8")

    assert dispatch(state, "calibrate_tcp_gui", str(source)).startswith("✗")
    assert state.pending_calibration is None
    source.write_text(json.dumps({
        "origin": [0, 0, 0], "x_point": [1, 0, 0], "xy_point": [2, 0, 0],
    }), encoding="utf-8")
    assert dispatch(state, "calibrate_frame_gui", str(source)).startswith("✗")
    assert state.pending_calibration is None

    old_frame_pose = frame.pose.copy()
    source.write_text(json.dumps({
        "origin": [10, 20, 30], "x_point": [20, 20, 30], "xy_point": [13, 25, 30],
    }), encoding="utf-8")
    assert "3 punktów" in dispatch(state, "calibrate_frame_gui", str(source))
    cancel_pending_calibration(state)
    assert np.allclose(tool.pose, old_pose)
    assert np.allclose(frame.pose, old_frame_pose)
