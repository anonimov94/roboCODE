from robocode.gui import actions as A
from robocode.gui.handlers import AppState, dispatch
from robocode.demo import build_demo_station


def test_gui_definition_consistent():
    assert A.validate() == []


def test_required_shortcuts_present():
    sc = {a.shortcut for a in A.ACTIONS}
    for s in ["F1", "F2", "F6", "Shift+F6", "Ctrl+F6", "F7", "Alt+0", "Ctrl+1", "Ctrl+T", "Ctrl+Shift+T",
              "Ctrl+M", "Ctrl+Z", "Ctrl+Y", "Space", "Backspace", "Esc"]:
        assert s in sc, s


def test_handlers_flow(tmp_path):
    s = AppState()
    s.station, *_ = build_demo_station()
    assert "Dodano" in dispatch(s, "add_frame")
    assert "Cofnięto" == dispatch(s, "undo")
    dispatch(s, "add_program")
    dispatch(s, "add_movej")
    dispatch(s, "add_movel")
    msg = dispatch(s, "generate_program", str(tmp_path))
    assert "Wygenerowano" in msg
    assert "niezaimplementowana" in dispatch(s, None)
    assert "Zapisano" in dispatch(s, "save_station_as", str(tmp_path / "x.rcs.json"))
