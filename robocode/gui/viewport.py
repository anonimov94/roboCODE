"""Widok 3D w oknie Qt (pyvistaqt). Cała geometria pochodzi z render.build_scene."""
from __future__ import annotations

import numpy as np

from .render import add_to_plotter, build_scene, path_bounds, scene_bounds, set_view, setup_plotter


class Viewport:
    """Opakowanie QtInteractor. Gdy pyvistaqt nie jest zainstalowane – widget zastępczy."""

    def __init__(self, parent=None):
        self.available = False
        self.widget = None
        self.plotter = None
        self._actors = []
        self._pickable_items: dict[str, int] = {}
        self._selection_callback = None
        self._transform_callback = None
        self._transform_widget = None
        self.supports_transform_gizmo = False
        try:
            from pyvistaqt import QtInteractor
            self.plotter = QtInteractor(parent)
            setup_plotter(self.plotter)
            self.supports_transform_gizmo = hasattr(self.plotter, "add_affine_transform_widget")
            self.widget = self.plotter.interactor
            self.plotter.enable_mesh_picking(
                callback=self._on_mesh_picked, show=False, show_message=False,
                left_clicking=True, use_actor=True,
            )
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
        if self._transform_widget is not None:
            self._transform_widget.remove()
            self._transform_widget = None
        self.plotter.clear_actors()
        self._actors = build_scene(state.station, state.overlay, state.render_opt)
        self._pickable_items = add_to_plotter(self.plotter, self._actors)
        if not self.plotter.picking._picker_in_use:
            self._enable_picking()
        if not state.simulation_playing:
            self._add_transform_gizmo(state)
        if cam is not None:
            self.plotter.camera_position = cam
        else:
            self.apply_view("iso", state)
        self.plotter.render()

    def set_selection_callback(self, callback) -> None:
        self._selection_callback = callback

    def set_transform_callback(self, callback) -> None:
        self._transform_callback = callback

    def _enable_picking(self) -> None:
        self.plotter.enable_mesh_picking(
            callback=self._on_mesh_picked, show=False, show_message=False,
            left_clicking=True, use_actor=True,
        )

    def _add_transform_gizmo(self, state) -> None:
        if (not self.supports_transform_gizmo or state.gizmo_mode is None
                or len(state.selection) != 1 or self._transform_callback is None):
            return
        item = state.selection[0]
        if state.gizmo_mode == "frame" and item.kind in {"frame", "object", "target", "curve"}:
            world_pose = item.pose_abs()
        elif state.gizmo_mode == "tcp" and item.kind == "tool":
            robot = item.parent
            while robot is not None and robot.kind != "robot":
                robot = robot.parent
            if robot is None:
                return
            world_pose = robot.flange_abs() @ item.tcp()
        else:
            return

        import pyvista as pv
        origin = tuple(float(value) for value in world_pose[:3, 3])
        anchor = self.plotter.add_mesh(
            pv.Sphere(radius=35.0, center=origin), color="white", opacity=0.01,
            name="transform-gizmo-anchor", pickable=False, reset_camera=False,
        )

        def apply_transform(matrix):
            new_world_pose = np.asarray(matrix, dtype=float) @ world_pose
            self._transform_callback(item.id, new_world_pose)

        self._transform_widget = self.plotter.add_affine_transform_widget(
            anchor, origin=origin, scale=0.5, release_callback=apply_transform,
        )

    def _on_mesh_picked(self, actor) -> None:
        if self._selection_callback is None or actor is None:
            return
        item_id = self._pickable_items.get(getattr(actor, "name", ""))
        if item_id is not None:
            self._selection_callback(item_id)

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
