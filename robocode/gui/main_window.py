"""Okno główne PySide6 budowane z actions.py. Uruchom: python -m robocode.gui.main_window
Wymaga: pip install -e .[gui]"""
from __future__ import annotations

import sys

from . import actions as ACT
from .handlers import AppState, dispatch


def build_window():
    from PySide6.QtGui import QAction, QKeySequence
    from PySide6.QtWidgets import (QApplication, QDockWidget, QLabel, QMainWindow, QToolBar, QTreeWidget,
                                   QTreeWidgetItem)
    from PySide6.QtCore import Qt

    app = QApplication.instance() or QApplication(sys.argv)
    state = AppState()
    win = QMainWindow()
    win.setWindowTitle("roboCODE")
    win.resize(1400, 860)

    tree = QTreeWidget()
    tree.setHeaderHidden(True)

    def refresh_tree():
        tree.clear()

        def add(item, parent_widget):
            w = QTreeWidgetItem([f"{item.name}  ·  {item.kind}"])
            w.setData(0, Qt.UserRole, item.id)
            (parent_widget.addChild(w) if parent_widget else tree.addTopLevelItem(w))
            for c in item.children:
                add(c, w)
        add(state.station, None)
        tree.expandAll()

    qactions = {}
    for a in ACT.ACTIONS:
        qa = QAction(a.label_pl, win)
        if a.shortcut:
            qa.setShortcut(QKeySequence(a.shortcut))
        qa.setCheckable(a.checkable)
        qa.setToolTip(f"{a.label_pl} ({a.shortcut})" if a.shortcut else a.label_pl)
        if a.command is None:
            qa.setEnabled(False)
            qa.setToolTip(qa.toolTip() + f" – planowane w {a.milestone}")

        def run(_checked=False, cmd=a.command):
            msg = dispatch(state, cmd)
            win.statusBar().showMessage(msg, 5000)
            refresh_tree()
        qa.triggered.connect(run)
        qactions[a.id] = qa

    for _, label, items in ACT.MENUS:
        m = win.menuBar().addMenu(label)
        for i in items:
            m.addSeparator() if i is None else m.addAction(qactions[i])

    tb = QToolBar("Główny")
    win.addToolBar(tb)
    for i in ACT.TOOLBAR:
        tb.addSeparator() if i is None else tb.addAction(qactions[i])

    dock = QDockWidget("Stacja", win)
    dock.setWidget(tree)
    win.addDockWidget(Qt.LeftDockWidgetArea, dock)
    props = QDockWidget("Właściwości", win)
    props.setWidget(QLabel("Zaznacz element w drzewie (M2)"))
    win.addDockWidget(Qt.RightDockWidgetArea, props)

    view = QLabel("Widok 3D – M3 (pyvista)")
    view.setAlignment(Qt.AlignCenter)
    win.setCentralWidget(view)
    win.statusBar().showMessage("Gotowy")
    refresh_tree()
    return app, win


def main():
    app, win = build_window()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
