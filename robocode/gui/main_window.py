"""Okno główne PySide6 budowane z actions.py.
Uruchom: python -m robocode.cli gui   (wymaga: pip install -e .[gui])"""
from __future__ import annotations

import sys

from . import actions as ACT
from .handlers import NEEDS_PATH, AppState, dispatch

KIND_PL = {"station": "stacja", "robot": "robot", "tool": "narzędzie", "frame": "układ", "object": "obiekt",
           "target": "cel", "program": "program", "curve": "krzywa", "item": "element"}


def build_window(state: AppState | None = None):
    from PySide6.QtCore import Qt
    from PySide6.QtGui import QAction, QKeySequence
    from PySide6.QtWidgets import (QApplication, QDockWidget, QFileDialog, QLabel, QMainWindow, QToolBar,
                                   QTreeWidget, QTreeWidgetItem)

    from .viewport import Viewport

    app = QApplication.instance() or QApplication(sys.argv)
    state = state or AppState()
    win = QMainWindow()
    win.setWindowTitle("roboCODE")
    win.resize(1400, 860)

    # --- widok 3D ---
    view = Viewport(win)
    win.setCentralWidget(view.widget)
    win.viewport = view

    # --- drzewo stacji ---
    tree = QTreeWidget()
    tree.setHeaderHidden(True)
    id_map: dict[int, object] = {}

    def refresh_tree():
        tree.blockSignals(True)
        tree.clear()
        id_map.clear()

        def add(item, parent_widget):
            w = QTreeWidgetItem([f"{item.name}  ·  {KIND_PL.get(item.kind, item.kind)}"])
            w.setData(0, Qt.UserRole, item.id)
            if not item.visible:
                w.setForeground(0, Qt.gray)
            id_map[item.id] = item
            (parent_widget.addChild(w) if parent_widget else tree.addTopLevelItem(w))
            for c in item.children:
                add(c, w)
        add(state.station, None)
        tree.expandAll()
        tree.blockSignals(False)

    def on_select():
        state.selection = [id_map[w.data(0, Qt.UserRole)] for w in tree.selectedItems()
                           if w.data(0, Qt.UserRole) in id_map]
        props.setText(_props_text(state.selection[0]) if state.selection else "Zaznacz element w drzewie")

    tree.itemSelectionChanged.connect(on_select)

    def refresh_all(first: bool = False):
        refresh_tree()
        view.refresh(state, keep_camera=not first and state.view_request is None)
        if state.view_request:
            view.apply_view("iso" if state.view_request == "fit" else state.view_request, state)
            state.view_request = None

    # --- akcje ---
    def run_command(cmd):
        args = []
        if cmd in NEEDS_PATH:
            title, filt, mode = NEEDS_PATH[cmd]
            fn = (QFileDialog.getSaveFileName if mode == "save" else QFileDialog.getOpenFileName)(win, title, "", filt)[0]
            if not fn:
                return
            args = [fn]
        try:
            msg = dispatch(state, cmd, *args)
        except Exception as e:  # błąd nie może zabić GUI
            msg = f"Błąd: {e}"
        win.statusBar().showMessage(msg, 8000)
        refresh_all()

    qactions = {}
    for a in ACT.ACTIONS:
        qa = QAction(a.label_pl, win)
        if a.shortcut:
            qa.setShortcut(QKeySequence(a.shortcut))
        qa.setCheckable(a.checkable)
        tip = f"{a.label_pl} ({a.shortcut})" if a.shortcut else a.label_pl
        if a.command is None:
            qa.setEnabled(False)
            tip += f" – planowane w {a.milestone}"
        qa.setToolTip(tip)
        qa.setStatusTip(tip)
        qa.triggered.connect(lambda _c=False, cmd=a.command: run_command(cmd))
        qactions[a.id] = qa
    win.qactions = qactions

    for _, label, items in ACT.MENUS:
        m = win.menuBar().addMenu(label)
        for i in items:
            m.addSeparator() if i is None else m.addAction(qactions[i])

    tb = QToolBar("Główny")
    tb.setObjectName("main_toolbar")
    win.addToolBar(tb)
    for i in ACT.TOOLBAR:
        tb.addSeparator() if i is None else tb.addAction(qactions[i])

    dock = QDockWidget("Stacja", win)
    dock.setWidget(tree)
    win.addDockWidget(Qt.LeftDockWidgetArea, dock)
    props = QLabel("Zaznacz element w drzewie")
    props.setAlignment(Qt.AlignTop | Qt.AlignLeft)
    props.setWordWrap(True)
    props.setMargin(8)
    pdock = QDockWidget("Właściwości", win)
    pdock.setWidget(props)
    win.addDockWidget(Qt.RightDockWidgetArea, pdock)

    win.statusBar().showMessage("Gotowy. Plik → Otwórz demo obróbki, albo Ctrl+M z własnym plikiem NC.")
    refresh_all(first=True)
    win.refresh_all = refresh_all
    win.state = state
    return app, win


def _props_text(item) -> str:
    import numpy as np
    from ..core import pose as P
    lines = [f"<b>{item.name}</b> ({KIND_PL.get(item.kind, item.kind)})"]
    T = item.pose
    x, y, z, rx, ry, rz = P.to_xyzabc(T)
    lines.append(f"Poza wzgl. rodzica:<br>X {x:.2f} Y {y:.2f} Z {z:.2f} mm<br>Rx {rx:.2f} Ry {ry:.2f} Rz {rz:.2f}°")
    if item.kind == "robot":
        lines.append("Osie: " + ", ".join(f"J{i+1} {v:.2f}°" for i, v in enumerate(item.joints)))
        tcp = item.tcp_abs()
        lines.append("TCP (abs): " + ", ".join(f"{v:.1f}" for v in tcp[:3, 3]) + " mm")
        lines.append(f"Post-procesor: {item.post_processor}")
    if item.kind == "tool":
        lines.append("TCP wzgl. kołnierza: " + ", ".join(f"{v:.1f}" for v in item.tcp()[:3, 3]) + " mm")
    if item.kind == "object" and getattr(item, "box_size", None):
        lines.append("Prostopadłościan: " + " × ".join(f"{v:g}" for v in item.box_size) + " mm")
    _ = np
    return "<br>".join(lines)


def main():
    app, win = build_window()
    win.show()
    code = app.exec()
    win.viewport.close()
    sys.exit(code)


if __name__ == "__main__":
    main()
