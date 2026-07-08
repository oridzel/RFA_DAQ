"""Simple PySide6 GUI for the RBD 9103 monitor.

Install optional dependencies:
    pip install PySide6 pyqtgraph

Run:
    python -m rfa_python.apps.rbd_monitor_gui --config rfa_python/config/rbd_channels.yaml
    python -m rfa_python.apps.rbd_monitor_gui --simulate
"""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime
from pathlib import Path
import sys
import time

from PySide6 import QtCore, QtWidgets
import pyqtgraph as pg

from rfa_python.instruments.rbd_manager import RBDManager, RBDCsvLogger


def _format_current(value) -> str:
    if value is None:
        return ""
    av = abs(value)
    if av >= 1e-3:
        return f"{value*1e3:.6g} mA"
    if av >= 1e-6:
        return f"{value*1e6:.6g} uA"
    if av >= 1e-9:
        return f"{value*1e9:.6g} nA"
    if av >= 1e-12:
        return f"{value*1e12:.6g} pA"
    return f"{value:.3e} A"


class RBDMonitorWindow(QtWidgets.QMainWindow):
    def __init__(self, manager: RBDManager, output_dir: str = "data") -> None:
        super().__init__()
        self.manager = manager
        self.output_dir = Path(output_dir)
        self.logger: RBDCsvLogger | None = None
        self.t0 = time.time()
        self.history: dict[str, deque[tuple[float, float]]] = {name: deque(maxlen=1000) for name in manager.devices}
        self.setWindowTitle("RFA Python DAQ - RBD 9103 Monitor")
        self.resize(1320, 780)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)
        self.connect_btn = QtWidgets.QPushButton("Connect All")
        self.init_btn = QtWidgets.QPushButton("Initialize All")
        self.start_btn = QtWidgets.QPushButton("Start Monitor")
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.once_btn = QtWidgets.QPushButton("Read Once")
        self.save_check = QtWidgets.QCheckBox("Save CSV")
        self.save_check.setChecked(True)
        self.period_spin = QtWidgets.QDoubleSpinBox()
        self.period_spin.setRange(0.1, 10.0)
        self.period_spin.setValue(1.0)
        self.period_spin.setSuffix(" s")
        for w in [self.connect_btn, self.init_btn, self.once_btn, self.start_btn, self.stop_btn]:
            controls.addWidget(w)
        controls.addWidget(QtWidgets.QLabel("Update:"))
        controls.addWidget(self.period_spin)
        controls.addWidget(self.save_check)
        controls.addStretch()

        splitter = QtWidgets.QSplitter()
        layout.addWidget(splitter, stretch=1)

        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        splitter.addWidget(left)

        self.table = QtWidgets.QTableWidget()
        headers = [
            "Electrode", "RBD nickname", "Port", "Status", "Current", "RBD status", "Range", "Raw response", "Error"
        ]
        self.table.setColumnCount(len(headers))
        self.table.setHorizontalHeaderLabels(headers)
        self.table.setRowCount(len(self.manager.devices))
        self.name_to_row = {}
        for row, (name, dev) in enumerate(self.manager.devices.items()):
            self.name_to_row[name] = row
            label = self.manager.labels.get(name, name)
            vals = [label, name, dev.port_name, "closed", "", "", "", "", ""]
            for col, val in enumerate(vals):
                item = QtWidgets.QTableWidgetItem(val)
                item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                self.table.setItem(row, col, item)
        self.table.horizontalHeader().setStretchLastSection(True)
        left_layout.addWidget(self.table)

        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(500)
        left_layout.addWidget(self.log_box, stretch=1)

        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        splitter.addWidget(right)
        splitter.setSizes([650, 670])

        self.plot = pg.PlotWidget(title="Live RBD currents")
        self.plot.setLabel("bottom", "Time", units="s")
        self.plot.setLabel("left", "Current", units="A")
        self.plot.addLegend()
        self.curves = {}
        for name in self.manager.devices:
            self.curves[name] = self.plot.plot([], [], pen=pg.mkPen(width=2), name=self.manager.labels.get(name, name))
        right_layout.addWidget(self.plot)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_readings)

        self.connect_btn.clicked.connect(self.connect_all)
        self.init_btn.clicked.connect(self.initialize_all)
        self.once_btn.clicked.connect(self.read_once)
        self.start_btn.clicked.connect(self.start_monitor)
        self.stop_btn.clicked.connect(self.stop_monitor)
        self.stop_btn.setEnabled(False)

    def append_log(self, text: str) -> None:
        self.log_box.appendPlainText(f"{datetime.now().strftime('%H:%M:%S')}  {text}")

    def connect_all(self) -> None:
        results = self.manager.open_all()
        for name, status in results.items():
            row = self.name_to_row[name]
            self.table.item(row, 3).setText(status)
            self.append_log(f"{name}: {status}")

    def initialize_all(self) -> None:
        results = self.manager.initialize_all()
        for name, status in results.items():
            self.append_log(f"{name}: {status}")

    def read_once(self) -> None:
        readings = self.manager.sample_all_once()
        self.handle_readings(readings)

    def start_monitor(self) -> None:
        results = self.manager.start_all()
        for name, status in results.items():
            self.append_log(f"{name}: {status}")
        if self.save_check.isChecked() and self.logger is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.logger = RBDCsvLogger(self.output_dir / f"rbd_monitor_{ts}.csv")
            self.append_log(f"Saving CSV: {self.logger.output_path}")
        self.timer.start(int(self.period_spin.value() * 1000))
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

    def stop_monitor(self) -> None:
        self.timer.stop()
        self.manager.stop_all()
        if self.logger:
            path = self.logger.output_path
            self.logger.close()
            self.logger = None
            self.append_log(f"Saved CSV: {path}")
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

    def closeEvent(self, event) -> None:  # noqa: N802
        self.stop_monitor()
        self.manager.close_all()
        event.accept()

    def update_readings(self) -> None:
        readings = self.manager.read_all_once()
        self.handle_readings(readings)

    def handle_readings(self, readings) -> None:
        t = time.time() - self.t0
        for r in readings:
            row = self.name_to_row.get(r.name)
            if row is None:
                continue
            raw_current_text = _format_current(r.current_A)
            self.table.item(row, 4).setText(raw_current_text)
            self.table.item(row, 5).setText(str(r.status_code or ""))
            self.table.item(row, 6).setText(str(r.range_code or ""))
            self.table.item(row, 7).setText(r.raw_response or "")
            self.table.item(row, 8).setText(r.error or "")
            y = r.current_A
            if y is not None:
                self.history[r.name].append((t, y))
            if r.error:
                self.append_log(f"{r.name}: {r.error}; raw={r.raw_response!r}")
        if self.logger:
            self.logger.write_many(readings)
        self.update_plot()

    def update_plot(self) -> None:
        for name, curve in self.curves.items():
            hist = self.history[name]
            if not hist:
                continue
            xs = [p[0] for p in hist]
            ys = [p[1] for p in hist]
            curve.setData(xs, ys)


def main() -> None:
    parser = argparse.ArgumentParser(description="RBD 9103 GUI monitor")
    parser.add_argument("--config", default="rfa_python/config/rbd_channels.yaml")
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--output-dir", default="data")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    manager = RBDManager.from_yaml(args.config, simulate=args.simulate)
    win = RBDMonitorWindow(manager, output_dir=args.output_dir)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
