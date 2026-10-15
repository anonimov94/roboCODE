"""Widok 3D w oknie Qt (pyvistaqt). Cała geometria pochodzi z render.build_scene."""
from __future__ import annotations

from .render import add_to_plotter, build_scene, path_bounds, scene_bounds, set_view, setup_plotter


class Viewport:
    """Opakowanie QtInteractor. Gdy pyvistaqt nie jest zainstalowane – widget zastępczy."""

    def __init__(self, parent=None):
        self.available = False
        self.widget = None
        self.plotter = None
        self._actors = []
        try:
            from pyvistaqt import QtInteractor
            self.plotter = QtInteractor(parent)
            setup_plotter(self.plotter)
            self.widget = self.plotter.interactor
            self.available = True
        except Exception as e:  # brak pyvistaqt / OpenGL
            from PySide6.QtWidgets import QLabel
            from PySide6.QtCore import Qt
            self.widget = QLabel(f"Widok 3D niedostępny: {e}\npip install -e .[gui]")
            self.widget.setAlignment(Qt.AlignCenter)

    def refresh(self, state, keep_camera: bool = True) -> None:
        if not self.available:
            return
        cam = self.plotter.camera_position if keep_camera and self._actors else None
        self.plotter.clear_actors()
        self._actors = build_scene(state.station, state.overlay, state.render_opt)
        add_to_plotter(self.plotter, self._actors)
        if cam is not None:
            self.plotter.camera_position = cam
        else:
            self.apply_view("iso", state)
        self.plotter.render()

    def apply_view(self, which: str, state=None) -> None:
        if not self.available:
            return
        b = None
        if which == "path" and state is not None:
            b = path_bounds(state.overlay)
            which = "iso"
        set_view(self.plotter, which, b or scene_bounds(self._actors))
        self.plotter.render()

    def close(self) -> None:
        if self.plotter is not None:
            self.plotter.close()
