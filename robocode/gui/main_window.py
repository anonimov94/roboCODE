"""Okno główne PySide6 budowane z actions.py.
Uruchom: python -m robocode.cli gui   (wymaga: pip install -e .[gui])"""
from __future__ import annotations

import sys

from . import actions as ACT
from .handlers import NEEDS_PATH, AppState, dispatch

KIND_PL = {"station": "stacja", "robot": "robot", "tool": "narzędzie", "frame": "układ", "object": "obiekt",
           "target": "cel", "program": "program", "curve": "krzywa", "item": "element"}


def _report_text(report) -> str:
    if report is None:
        return "Brak raportu walidacji. Otwórz projekt obróbki lub wygeneruj program (F6)."
    return report.summary(max_items=max(20, len(report.issues)))


def build_window(state: AppState | None = None):
    from PySide6.QtCore import Qt
    from PySide6.QtGui import QAction, QKeySequence
    from PySide6.QtWidgets import (QApplication, QDialog, QDialogButtonBox, QDockWidget, QFileDialog, QFormLayout,
                                   QLabel, QLineEdit, QMainWindow, QPushButton, QDoubleSpinBox,
                                   QToolBar, QMessageBox, QTextBrowser, QTreeWidget, QTreeWidgetItem, QVBoxLayout,
                                   QWidget)

    from ..core import pose as P
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
    props = QLabel("Zaznacz element w drzewie")
    props.setAlignment(Qt.AlignTop | Qt.AlignLeft)
    props.setWordWrap(True)
    props.setMargin(8)
    edit_properties_button = QPushButton("Edytuj właściwości…")
    edit_properties_button.setEnabled(False)
    widget_map: dict[int, QTreeWidgetItem] = {}

    def refresh_tree():
        selected_ids = {item.id for item in state.selection}
        tree.blockSignals(True)
        tree.clear()
        id_map.clear()
        widget_map.clear()

        def add(item, parent_widget):
            w = QTreeWidgetItem([f"{item.name}  ·  {KIND_PL.get(item.kind, item.kind)}"])
            w.setData(0, Qt.UserRole, item.id)
            if not item.visible:
                w.setForeground(0, Qt.gray)
            id_map[item.id] = item
            widget_map[item.id] = w
            (parent_widget.addChild(w) if parent_widget else tree.addTopLevelItem(w))
            for c in item.children:
                add(c, w)
        add(state.station, None)
        tree.expandAll()
        for item_id in selected_ids & widget_map.keys():
            widget_map[item_id].setSelected(True)
        tree.blockSignals(False)
        on_select()

    def on_select():
        state.selection = [id_map[w.data(0, Qt.UserRole)] for w in tree.selectedItems()
                           if w.data(0, Qt.UserRole) in id_map]
        props.setText(_props_text(state.selection[0]) if state.selection else "Zaznacz element w drzewie")
        edit_properties_button.setEnabled(len(state.selection) == 1)

    tree.itemSelectionChanged.connect(on_select)

    def refresh_all(first: bool = False):
        refresh_tree()
        view.refresh(state, keep_camera=not first and state.view_request is None)
        machining_report.setPlainText(_report_text(state.last_report))
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
            if state.pending_calibration is not None:
                _, _, details = state.pending_calibration
                answer = QMessageBox.question(
                    win, "Zastosować kalibrację?",
                    f"{details}\n\nZastosować wynik do stacji? Zmiana będzie dostępna w Cofnij.",
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
                )
                if answer == QMessageBox.Yes:
                    msg = dispatch(state, "apply_pending_calibration")
                else:
                    from .handlers import cancel_pending_calibration
                    cancel_pending_calibration(state)
                    msg = "Kalibracja obliczona, zmiany nie zastosowano"
        except Exception as e:  # błąd nie może zabić GUI
            msg = f"Błąd: {e}"
        win.statusBar().showMessage(msg, 8000)
        refresh_all()
        if cmd in {"open_demo", "new_machining_project", "update_machining",
                   "generate_program", "generate_program_as"}:
            report_dock.raise_()

    def edit_selected_properties():
        if len(state.selection) != 1:
            return
        item = state.selection[0]
        dialog = QDialog(win)
        dialog.setWindowTitle(f"Właściwości: {item.name}")
        form = QFormLayout(dialog)
        name_field = QLineEdit(item.name, dialog)
        form.addRow("Nazwa", name_field)
        current = P.to_xyzabc(item.pose)
        fields = []
        for label, value, suffix, limit in (
            ("X", current[0], " mm", 1e9), ("Y", current[1], " mm", 1e9),
            ("Z", current[2], " mm", 1e9), ("Rx", current[3], "°", 360000),
            ("Ry", current[4], "°", 360000), ("Rz", current[5], "°", 360000),
        ):
            field = QDoubleSpinBox(dialog)
            field.setDecimals(4)
            field.setRange(-limit, limit)
            field.setSuffix(suffix)
            field.setValue(float(value))
            form.addRow(label, field)
            fields.append(field)
        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=dialog,
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)
        if dialog.exec() != QDialog.Accepted:
            return
        pose = P.from_xyzabc(*(field.value() for field in fields))
        message = dispatch(state, "edit_item_properties", name_field.text(), pose)
        win.statusBar().showMessage(message, 8000)
        refresh_all()

    edit_properties_button.clicked.connect(edit_selected_properties)
    tree.itemDoubleClicked.connect(lambda _item, _column: edit_selected_properties())

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
    properties_widget = QWidget()
    properties_layout = QVBoxLayout(properties_widget)
    properties_layout.addWidget(props)
    properties_layout.addWidget(edit_properties_button)
    properties_layout.addStretch(1)
    pdock = QDockWidget("Właściwości", win)
    pdock.setWidget(properties_widget)
    win.addDockWidget(Qt.RightDockWidgetArea, pdock)
    machining_report = QTextBrowser()
    machining_report.setObjectName("machining_validation_report")
    machining_report.setOpenExternalLinks(False)
    machining_report.setPlainText("Brak raportu walidacji.")
    report_dock = QDockWidget("Projekt obróbki", win)
    report_dock.setObjectName("machining_project_dock")
    report_dock.setWidget(machining_report)
    win.addDockWidget(Qt.RightDockWidgetArea, report_dock)
    win.tabifyDockWidget(pdock, report_dock)
    win.machining_report_panel = machining_report
    win.machining_report_dock = report_dock
    win.edit_properties_button = edit_properties_button
    win.station_tree = tree
    win.station_item_widgets = widget_map
    win.properties_text = props

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
