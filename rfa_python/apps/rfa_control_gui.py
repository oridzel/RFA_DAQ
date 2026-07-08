"""Integrated RFA DAQ control GUI.

v0.16 scope:
- RBD 9103 picoammeter monitor with real electrode labels.
- Four TDK/FUG PHV supply control using set-and-wait / zero-and-wait logic.
- SRS DC205 collector supply control.
- Conservative bias-scan measurement tab with named RBD acquisition profiles,
  stale-sample discard, cleaner CSV columns, companion metadata logging,
  and collector preflight to actively hold the SRS output at ground/bias.
- Safe NI-DAQ/Kimball tab: device/channel discovery, meter readback, zero-all AO outputs, LUT preview, and conservative electrostatic-control apply.

Stepper motor motion is included only as a conservative MDrive bench-test tab.
"""

from __future__ import annotations

import argparse
import csv
from collections import deque
from datetime import datetime
from pathlib import Path
import sys
import time
from typing import Any

import yaml

try:
    import numpy as np
except Exception:  # pragma: no cover
    np = None

try:
    from PySide6 import QtCore, QtWidgets
    import pyqtgraph as pg
except ImportError as exc:  # pragma: no cover
    raise SystemExit("Install GUI dependencies first: pip install PySide6 pyqtgraph") from exc

from rfa_python.instruments.rbd_manager import RBDManager, RBDCsvLogger
from rfa_python.instruments.srs_dc205 import DC205, SimulatedDC205
from rfa_python.instruments.tdk_manager import TDKManager, HV_TOKEN
from rfa_python.instruments.ni_kimball_daq import KimballDAQ, GUN_DAQ_TOKEN
from rfa_python.instruments.mdrive import MDrive, SimulatedMDrive, MOTION_TOKEN, AngleStepLUT


TDK_ELECTRODE_LABELS = {
    "PN": "Space-charge grid",
    "N": "N supply / negative-only",
    "PNR": "Retarding grids",
    "R": "Sample and rod",
}

RBD_TO_ELECTRODE_FOR_TOTAL = {
    "Sneezy": "Sample",
    "Sleepy": "Retarding Grid 1",
    "Dopey": "Collector",
    "Happy": "Retarding Grid 2",
    "Grumpy": "Space-charge Grid",
    "Bashful": "Drift Tube",
    "Doc": "Rod",
}

# Explicit, stable plot colors. Pyqtgraph's default pens can look too similar,
# especially on dark backgrounds and with many nearly-overlapping traces.
RBD_PLOT_COLORS = {
    "Sneezy": "#1f77b4",   # Sample
    "Sleepy": "#ff7f0e",   # RG1
    "Dopey": "#2ca02c",    # Collector
    "Happy": "#d62728",    # RG2
    "Grumpy": "#9467bd",   # SCG
    "Bashful": "#8c564b",  # Drift tube
    "Doc": "#e377c2",      # Rod
}

TDK_PLOT_COLORS = {
    "PN": "#17becf",
    "N": "#7f7f7f",
    "PNR": "#bcbd22",
    "R": "#1f77b4",
}


def default_config_path(filename: str) -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / filename)


def fmt_float(value: Any, suffix: str = "", precision: int = 6) -> str:
    if value is None:
        return ""
    try:
        return f"{float(value):.{precision}g}{suffix}"
    except Exception:
        return str(value)


def fmt_current(value: Any) -> str:
    if value is None:
        return ""
    try:
        value = float(value)
    except Exception:
        return str(value)
    av = abs(value)
    if av >= 1e-3:
        return f"{value*1e3:.6g} mA"
    if av >= 1e-6:
        return f"{value*1e6:.6g} µA"
    if av >= 1e-9:
        return f"{value*1e9:.6g} nA"
    if av >= 1e-12:
        return f"{value*1e12:.6g} pA"
    return f"{value:.3e} A"


def load_srs_from_yaml(path: str | Path, *, simulate_override: bool | None = None):
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    simulate = bool(cfg.get("simulate", False)) if simulate_override is None else bool(simulate_override)
    serial_cfg = cfg.get("serial", {}) or {}
    safety_cfg = cfg.get("safety", {}) or {}
    kwargs = dict(
        name=str(cfg.get("name", "SRS DC205")),
        port=str(serial_cfg.get("port", "COM1")),
        baudrate=int(serial_cfg.get("baudrate", 115200)),
        timeout_s=float(serial_cfg.get("timeout_s", 1.0)),
        max_abs_voltage_V=float(safety_cfg.get("max_abs_voltage_V", 10.0)),
        allow_100V_range=bool(safety_cfg.get("allow_100V_range", False)),
    )
    if simulate:
        dev = SimulatedDC205(kwargs["name"])
        dev.port_name = kwargs["port"]  # for GUI display only
        return dev
    dev = DC205(**kwargs)
    dev.port_name = kwargs["port"]
    return dev


def load_yaml_file(path: str | Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def make_mdrive_axis(root_cfg: dict[str, Any], axis: str, *, simulate_override: bool | None):
    axes = root_cfg.get("axes", {}) or {}
    if axis not in axes:
        raise KeyError(f"Unknown MDrive axis {axis!r}")
    axis_cfg = dict(axes[axis] or {})
    defaults = root_cfg.get("serial_defaults", {}) or {}
    simulate = bool(root_cfg.get("simulate", False)) if simulate_override is None else bool(simulate_override)
    axis_cfg.setdefault("baudrate", defaults.get("baudrate", 9600))
    axis_cfg.setdefault("timeout_s", defaults.get("timeout_s", 1.0))
    if simulate:
        return SimulatedMDrive(name=axis), axis_cfg
    return MDrive(
        port=str(axis_cfg.get("port", "COM1")),
        name=axis,
        baudrate=int(axis_cfg.get("baudrate", 9600)),
        timeout_s=float(axis_cfg.get("timeout_s", 1.0)),
    ), axis_cfg


def motion_requires_token(root_cfg: dict[str, Any]) -> bool:
    return bool((root_cfg.get("motion_safety", {}) or {}).get("require_enable_token", True))


def motion_limit_steps(root_cfg: dict[str, Any], axis_cfg: dict[str, Any]) -> int:
    safety = root_cfg.get("motion_safety", {}) or {}
    return int(axis_cfg.get("max_single_move_steps", safety.get("max_single_move_steps", 10000)))


def motion_user_units_to_microsteps(axis: str, value: int | float, axis_cfg: dict[str, Any]) -> tuple[int, dict[str, Any]]:
    scale = int(axis_cfg.get("manual_move_scale", axis_cfg.get("steps_per_micron", 1)) or 1)
    units = str(axis_cfg.get("manual_move_units_label", "microsteps"))
    motor_steps = int(round(float(value) * scale))
    return motor_steps, {"entered": value, "scale": scale, "motor_microsteps": motor_steps, "units": units}


def motion_manual_relative_steps(axis: str, requested_steps: int, axis_cfg: dict[str, Any]) -> tuple[int, dict[str, Any]]:
    return motion_user_units_to_microsteps(axis, requested_steps, axis_cfg)


def motion_lift_target_microsteps(axis_cfg: dict[str, Any], which: str) -> tuple[int, dict[str, Any]]:
    micron_key = "open_position_microns" if which == "open" else "closed_position_microns"
    step_key = "open_position_steps" if which == "open" else "closed_position_steps"
    if micron_key in axis_cfg:
        target, info = motion_user_units_to_microsteps("lift", axis_cfg[micron_key], axis_cfg)
        info["source"] = micron_key
        return target, info
    target = int(axis_cfg.get(step_key, -1200000 if which == "open" else 0))
    return target, {"entered": target, "scale": 1, "motor_microsteps": target, "units": "microsteps", "source": step_key}


def motion_limit_slew(root_cfg: dict[str, Any], axis_cfg: dict[str, Any]) -> int:
    safety = root_cfg.get("motion_safety", {}) or {}
    return int(axis_cfg.get("max_slew_steps_per_s", safety.get("max_slew_steps_per_s", 10000)))


def default_rbd_profiles() -> dict[str, Any]:
    return {
        "default_profile": "yield_standard",
        "profiles": {
            "yield_standard": {
                "description": "Standard low-noise yield measurement.",
                "range": "auto",
                "filter": 32,
                "sample_interval_ms": 50,
                "default_n_samples": 5,
                "default_sample_period_s": 0.05,
                "discard_first": 2,
                "mode": "interval",
                "implemented": True,
            }
        },
    }


def load_rbd_acquisition_profiles(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    if not p.exists():
        return default_rbd_profiles()
    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if "profiles" not in data:
        # Also accept a plain mapping of profile_name -> profile.
        data = {"default_profile": next(iter(data), "yield_standard"), "profiles": data}
    return data


MEASUREMENT_MODES = {
    "retarding_grid_bias": {
        "label": "Retarding-grid bias (old PM)",
        "scanned_supply": "PNR",
        "description": "Scan retarding-grid supply PNR; other supplies stay at their present values.",
    },
    "sample_bias_bsey": {
        "label": "Sample-bias BSEY (old PM2)",
        "scanned_supply": "R",
        "description": "Scan sample/rod supply R; other supplies stay at their present values.",
    },
}


def make_scan_values(start: float, end: float, step: float) -> list[float]:
    """Generate inclusive scan values with LabVIEW-like start/end/step behavior."""
    start = float(start)
    end = float(end)
    step = float(step)
    if step == 0:
        raise ValueError("Scan step cannot be zero.")
    if start < end and step < 0:
        raise ValueError("Step must be positive when start < end.")
    if start > end and step > 0:
        raise ValueError("Step must be negative when start > end.")
    values: list[float] = []
    v = start
    eps = abs(step) * 1e-9 + 1e-12
    if step > 0:
        while v <= end + eps:
            values.append(round(v, 12))
            v += step
    else:
        while v >= end - eps:
            values.append(round(v, 12))
            v += step
    return values


class BiasScanWorker(QtCore.QObject):
    """Runs a conservative voltage-bias scan in a background Qt thread."""

    log = QtCore.Signal(str)
    row_ready = QtCore.Signal(dict)
    finished = QtCore.Signal(str)
    failed = QtCore.Signal(str)

    def __init__(
        self,
        *,
        rbd: RBDManager,
        tdk: TDKManager,
        srs,
        mode: str,
        energy_kev: float,
        scan_values_V: list[float],
        n_samples: int,
        n_repeats: int,
        sample_period_s: float,
        post_settle_s: float,
        initial_settle_s: float,
        voltage_tolerance_V: float,
        voltage_timeout_s: float,
        current_limit_A: float,
        output_path: Path,
        event_log_path: Path,
        metadata_path: Path,
        rbd_profile_name: str,
        rbd_profile: dict[str, Any],
        discard_first: int,
        collector_voltage_V: float,
        collector_range: str,
        ensure_srs_output_on: bool,
        zero_after_scan: bool,
        output_off_after_zero: bool,
        ramp_mode: int | None,
    ) -> None:
        super().__init__()
        self.rbd = rbd
        self.tdk = tdk
        self.srs = srs
        self.mode = mode
        self.energy_kev = float(energy_kev)
        self.scan_values_V = list(scan_values_V)
        self.n_samples = int(n_samples)
        self.n_repeats = int(n_repeats)
        self.sample_period_s = float(sample_period_s)
        self.post_settle_s = float(post_settle_s)
        self.initial_settle_s = float(initial_settle_s)
        self.voltage_tolerance_V = float(voltage_tolerance_V)
        self.voltage_timeout_s = float(voltage_timeout_s)
        self.current_limit_A = float(current_limit_A)
        self.output_path = Path(output_path)
        self.event_log_path = Path(event_log_path)
        self.metadata_path = Path(metadata_path)
        self.rbd_profile_name = str(rbd_profile_name)
        self.rbd_profile = dict(rbd_profile)
        self.discard_first = int(discard_first)
        self.collector_voltage_V = float(collector_voltage_V)
        self.collector_range = str(collector_range)
        self.ensure_srs_output_on = bool(ensure_srs_output_on)
        self.srs_start_status: dict[str, Any] | None = None
        self.zero_after_scan = bool(zero_after_scan)
        self.output_off_after_zero = bool(output_off_after_zero)
        self.ramp_mode = ramp_mode
        self._stop_requested = False

    @QtCore.Slot()
    def request_stop(self) -> None:
        self._stop_requested = True
        self.log.emit("Stop requested; scan will stop after current operation finishes.")

    def _write_event(self, text: str) -> None:
        self.event_log_path.parent.mkdir(parents=True, exist_ok=True)
        line = f"{datetime.now().isoformat(timespec='seconds')}  {text}\n"
        with self.event_log_path.open("a", encoding="utf-8") as f:
            f.write(line)
        self.log.emit(text)

    def _write_metadata(self, *, scanned_supply: str) -> None:
        metadata = {
            "created_local": datetime.now().isoformat(timespec="seconds"),
            "software_version": "v0.17",
            "data_csv": str(self.output_path),
            "event_log": str(self.event_log_path),
            "mode": self.mode,
            "energy_kev": self.energy_kev,
            "scanned_supply": scanned_supply,
            "scan_values_V": self.scan_values_V,
            "n_repeats": self.n_repeats,
            "n_samples_per_point": self.n_samples,
            "sample_period_s": self.sample_period_s,
            "post_settle_s": self.post_settle_s,
            "initial_settle_s": self.initial_settle_s,
            "voltage_tolerance_V": self.voltage_tolerance_V,
            "voltage_timeout_s": self.voltage_timeout_s,
            "current_limit_A": self.current_limit_A,
            "rbd_acquisition_profile_name": self.rbd_profile_name,
            "rbd_acquisition_profile": self.rbd_profile,
            "discard_first_samples_after_settle": self.discard_first,
            "collector_voltage_V": self.collector_voltage_V,
            "collector_range": self.collector_range,
            "ensure_srs_output_on": self.ensure_srs_output_on,
            "srs_start_status": self.srs_start_status,
            "rbd_current_used_for_averaging": "raw current_A; GUI baseline correction is not applied to measurement CSV",
            "rbd_channels": {name: self.rbd.labels.get(name, name) for name in self.rbd.devices},
            "tdk_supplies": {name: TDK_ELECTRODE_LABELS.get(name, name) for name in self.tdk.names()},
        }
        self.metadata_path.parent.mkdir(parents=True, exist_ok=True)
        with self.metadata_path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(metadata, f, sort_keys=False, allow_unicode=True)

    def _ensure_srs_collector_preflight(self) -> None:
        """Actively hold the collector with the SRS DC205 before measuring.

        Important for RFA measurements: even at 0 V the DC205 output should be
        ON so the collector is actively grounded rather than left floating.
        """
        if not self.ensure_srs_output_on:
            self._write_event("SRS collector preflight skipped by user setting.")
            return
        self.srs.open()
        target_range = self.collector_range
        try:
            current_range = self.srs.query_range()
        except Exception:
            current_range = None
        if current_range != target_range:
            try:
                self.srs.set_output(False)
            except Exception:
                pass
            self.srs.set_range(target_range)
            self._write_event(f"SRS collector range set to {target_range}")
        self.srs.set_voltage(self.collector_voltage_V)
        self.srs.set_output(True)
        st = self.srs.status().to_dict()
        self.srs_start_status = st
        if not st.get("output_on"):
            raise RuntimeError("SRS collector output did not turn ON during measurement preflight.")
        v = st.get("voltage_setpoint_V")
        v_text = "unknown" if v is None else f"{float(v):g}"
        self._write_event(f"SRS collector output ON at setpoint {v_text} V, range={st.get('range')}")

    @QtCore.Slot()
    def run(self) -> None:
        try:
            mode_cfg = MEASUREMENT_MODES[self.mode]
            scanned_supply = mode_cfg["scanned_supply"]
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            self._write_event(f"Bias scan started: mode={self.mode}, supply={scanned_supply}, file={self.output_path}")
            self._write_event(
                f"Values={self.scan_values_V}, repeats={self.n_repeats}, samples/point={self.n_samples}, "
                f"tolerance={self.voltage_tolerance_V} V"
            )
            self._ensure_srs_collector_preflight()

            self._write_metadata(scanned_supply=scanned_supply)
            self._write_event(f"Metadata written: {self.metadata_path}")
            self._write_event(
                f"RBD profile={self.rbd_profile_name}, range={self.rbd_profile.get('range')}, "
                f"filter={self.rbd_profile.get('filter')}, interval={self.rbd_profile.get('sample_interval_ms')} ms, "
                f"discard_first={self.discard_first}; measurement CSV uses raw current_A"
            )

            fieldnames = [
                "timestamp_local",
                "mode",
                "energy_kev",
                "point_index",
                "repeat_index",
                "scanned_supply",
                "commanded_voltage_V",
                "reached_voltage_V",
                "final_scanned_supply_voltage_V",
                "n_samples_requested",
                "sample_period_s",
                "post_settle_s",
                "initial_settle_s",
                "settle_s_used",
                "collector_voltage_V",
                "collector_range",
                "SRS_output_on_at_start",
                "SRS_voltage_setpoint_at_start_V",
                "discard_first_samples",
                "voltage_tolerance_V",
                "voltage_timeout_s",
                "rbd_profile",
            ]
            for name in self.rbd.devices:
                fieldnames.extend([
                    f"{name}_mean_A",
                    f"{name}_std_A",
                    f"{name}_n_valid",
                ])
            for supply in self.tdk.names():
                fieldnames.append(f"TDK_{supply}_actual_V")

            with self.output_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
                writer.writeheader()

                point_index = 0
                for repeat_index in range(1, self.n_repeats + 1):
                    for target_v in self.scan_values_V:
                        if self._stop_requested:
                            raise InterruptedError("Measurement scan stopped by user.")

                        point_index += 1
                        dev = self.tdk.get(scanned_supply)
                        reached_v = dev.set_signed_voltage_and_wait(
                            target_v,
                            current_limit_A=self.current_limit_A,
                            ramp_mode=self.ramp_mode,
                            tolerance_V=self.voltage_tolerance_V,
                            timeout_s=self.voltage_timeout_s,
                        )
                        self._write_event(
                            f"Point {point_index}: {scanned_supply} commanded {target_v:g} V, reached {reached_v:g} V"
                        )
                        settle_s_used = self.initial_settle_s if point_index == 1 else self.post_settle_s
                        if settle_s_used > 0:
                            if point_index == 1 and self.initial_settle_s != self.post_settle_s:
                                self._write_event(f"Initial settle before first logged point: {settle_s_used:g} s")
                            time.sleep(settle_s_used)

                        # Remove stale samples queued while the voltage was moving/settling,
                        # then discard a small number of fresh samples before averaging.
                        self.rbd.flush_input_all()
                        if self.discard_first > 0:
                            self.rbd.discard_samples(self.discard_first, sample_period_s=self.sample_period_s)

                        samples_by_name: dict[str, list[float]] = {name: [] for name in self.rbd.devices}
                        for _ in range(self.n_samples):
                            if self._stop_requested:
                                raise InterruptedError("Measurement scan stopped by user.")
                            readings = self.rbd.read_all_once(latest=False)
                            for r in readings:
                                # Measurement CSV intentionally uses raw current_A.
                                # GUI baseline/offset correction is monitor-only.
                                if r.current_A is not None:
                                    samples_by_name.setdefault(r.name, []).append(float(r.current_A))
                            if self.sample_period_s > 0:
                                time.sleep(self.sample_period_s)

                        tdk_actuals: dict[str, float | None] = {}
                        for supply in self.tdk.names():
                            try:
                                st = self.tdk.get(supply).status().to_dict()
                                tdk_actuals[supply] = st.get("signed_measured_voltage_V")
                            except Exception:
                                tdk_actuals[supply] = None
                        final_scanned_v = tdk_actuals.get(scanned_supply)

                        row: dict[str, Any] = {
                            "timestamp_local": datetime.now().isoformat(timespec="milliseconds"),
                            "mode": self.mode,
                            "energy_kev": self.energy_kev,
                            "point_index": point_index,
                            "repeat_index": repeat_index,
                            "scanned_supply": scanned_supply,
                            "commanded_voltage_V": target_v,
                            "reached_voltage_V": reached_v,
                            "final_scanned_supply_voltage_V": final_scanned_v,
                            "n_samples_requested": self.n_samples,
                            "sample_period_s": self.sample_period_s,
                            "post_settle_s": self.post_settle_s,
                            "initial_settle_s": self.initial_settle_s,
                            "settle_s_used": settle_s_used,
                            "collector_voltage_V": self.collector_voltage_V,
                            "collector_range": self.collector_range,
                            "SRS_output_on_at_start": None if self.srs_start_status is None else self.srs_start_status.get("output_on"),
                            "SRS_voltage_setpoint_at_start_V": None if self.srs_start_status is None else self.srs_start_status.get("voltage_setpoint_V"),
                            "discard_first_samples": self.discard_first,
                            "voltage_tolerance_V": self.voltage_tolerance_V,
                            "voltage_timeout_s": self.voltage_timeout_s,
                            "rbd_profile": self.rbd_profile_name,
                        }
                        for name, vals in samples_by_name.items():
                            avg = sum(vals) / len(vals) if vals else None
                            std = None
                            if vals and len(vals) > 1:
                                m = avg
                                std = (sum((x - m) ** 2 for x in vals) / (len(vals) - 1)) ** 0.5
                            row[f"{name}_mean_A"] = avg
                            row[f"{name}_std_A"] = std
                            row[f"{name}_n_valid"] = len(vals)
                        for supply, actual in tdk_actuals.items():
                            row[f"TDK_{supply}_actual_V"] = actual

                        writer.writerow(row)
                        f.flush()
                        self.row_ready.emit(row)

            if self.zero_after_scan:
                self._write_event("Scan complete; zeroing scanned supply.")
                dev = self.tdk.get(MEASUREMENT_MODES[self.mode]["scanned_supply"])
                actual = dev.zero_and_wait(
                    current_limit_A=self.current_limit_A,
                    tolerance_V=self.voltage_tolerance_V,
                    timeout_s=self.voltage_timeout_s,
                    output_off_after_zero=self.output_off_after_zero,
                )
                self._write_event(f"Scanned supply zeroed; actual {actual:g} V")

            self._write_event("Bias scan finished normally.")
            self.finished.emit(str(self.output_path))
        except InterruptedError as exc:
            self._write_event(str(exc))
            self.finished.emit(str(self.output_path))
        except Exception as exc:
            self._write_event(f"ERROR: {exc}")
            self.failed.emit(str(exc))


class ImagingScanWorker(QtCore.QObject):
    """Background XY deflection scan for total-yield imaging."""

    log = QtCore.Signal(str)
    pixel_ready = QtCore.Signal(dict)
    finished = QtCore.Signal(str)
    failed = QtCore.Signal(str)

    def __init__(
        self,
        *,
        rbd: RBDManager,
        tdk: TDKManager,
        srs,
        kimball: KimballDAQ,
        x_values: list[float],
        y_values: list[float],
        n_samples: int,
        sample_period_s: float,
        settle_s: float,
        collector_voltage_V: float,
        collector_range: str,
        output_path: Path,
        event_log_path: Path,
        metadata_path: Path,
        zero_tdk_first: bool,
        serpentine: bool,
        rbd_profile_name: str,
        rbd_profile: dict[str, Any],
        discard_first: int,
        use_abs_currents: bool,
    ) -> None:
        super().__init__()
        self.rbd = rbd
        self.tdk = tdk
        self.srs = srs
        self.kimball = kimball
        self.x_values = list(x_values)
        self.y_values = list(y_values)
        self.n_samples = int(n_samples)
        self.sample_period_s = float(sample_period_s)
        self.settle_s = float(settle_s)
        self.collector_voltage_V = float(collector_voltage_V)
        self.collector_range = str(collector_range)
        self.output_path = Path(output_path)
        self.event_log_path = Path(event_log_path)
        self.metadata_path = Path(metadata_path)
        self.zero_tdk_first = bool(zero_tdk_first)
        self.serpentine = bool(serpentine)
        self.rbd_profile_name = str(rbd_profile_name)
        self.rbd_profile = dict(rbd_profile)
        self.discard_first = int(discard_first)
        self.use_abs_currents = bool(use_abs_currents)
        self._stop_requested = False

    @QtCore.Slot()
    def request_stop(self) -> None:
        self._stop_requested = True
        self.log.emit("Stop requested; imaging scan will stop after current pixel finishes.")

    def _write_event(self, text: str) -> None:
        self.event_log_path.parent.mkdir(parents=True, exist_ok=True)
        with self.event_log_path.open("a", encoding="utf-8") as f:
            f.write(f"{datetime.now().isoformat(timespec='seconds')}  {text}\n")
        self.log.emit(text)

    def _safe_col(self, label: str) -> str:
        return label.replace(" ", "_").replace("-", "_").replace("/", "_")

    def _ensure_collector(self) -> dict[str, Any]:
        self.srs.open()
        try:
            if self.srs.query_range() != self.collector_range:
                try:
                    self.srs.set_output(False)
                except Exception:
                    pass
                self.srs.set_range(self.collector_range)
                self._write_event(f"SRS collector range set to {self.collector_range}")
        except Exception:
            self.srs.set_range(self.collector_range)
            self._write_event(f"SRS collector range set to {self.collector_range}")
        self.srs.set_voltage(self.collector_voltage_V)
        self.srs.set_output(True)
        st = self.srs.status().to_dict()
        if not st.get("output_on"):
            raise RuntimeError("SRS collector output did not turn ON for imaging.")
        self._write_event(f"Collector held by SRS at {self.collector_voltage_V:g} V, range={self.collector_range}")
        return st

    def _zero_tdk_supplies(self) -> None:
        if not self.zero_tdk_first:
            self._write_event("TDK zeroing skipped by user setting.")
            return
        self._write_event("Zeroing TDK supplies before imaging so non-collector electrodes are grounded.")
        for name in self.tdk.names():
            dev = self.tdk.get(name)
            if hasattr(dev, "arm_hv_changes"):
                dev.arm_hv_changes(HV_TOKEN)
            try:
                actual = dev.zero_and_wait(current_limit_A=0.0001, tolerance_V=0.5, timeout_s=30.0, output_off_after_zero=False)
                self._write_event(f"TDK {name} zeroed; actual={actual:g} V")
            except Exception as exc:
                raise RuntimeError(f"Could not zero TDK {name}: {exc}") from exc

    def _calculate_yield(self, currents_by_label: dict[str, float]) -> tuple[float | None, float, float]:
        def cur(label: str) -> float:
            v = float(currents_by_label.get(label, 0.0))
            return abs(v) if self.use_abs_currents else v
        numerator_labels = [
            "Collector",
            "Retarding Grid 1",
            "Retarding Grid 2",
            "Space-charge Grid",
            "Rod",
            "Drift Tube",
        ]
        numerator = sum(cur(label) for label in numerator_labels)
        denominator = numerator + cur("Sample")
        if abs(denominator) < 1e-30:
            return None, numerator, denominator
        return numerator / denominator, numerator, denominator

    def _write_metadata(self, srs_status: dict[str, Any]) -> None:
        metadata = {
            "created_local": datetime.now().isoformat(timespec="seconds"),
            "software_version": "v0.19.19",
            "scan_type": "xy_deflection_total_yield_image",
            "data_csv": str(self.output_path),
            "event_log": str(self.event_log_path),
            "x_values_V": self.x_values,
            "y_values_V": self.y_values,
            "n_samples_per_pixel": self.n_samples,
            "sample_period_s": self.sample_period_s,
            "settle_s": self.settle_s,
            "collector_voltage_V": self.collector_voltage_V,
            "collector_range": self.collector_range,
            "zero_tdk_first": self.zero_tdk_first,
            "serpentine": self.serpentine,
            "use_abs_currents": self.use_abs_currents,
            "formula": "TEY = (Collector + Retarding Grid 1 + Retarding Grid 2 + Space-charge Grid + Rod + Drift Tube)/(same + Sample)",
            "rbd_channels": {name: self.rbd.labels.get(name, name) for name in self.rbd.devices},
            "srs_start_status": srs_status,
        }
        self.metadata_path.parent.mkdir(parents=True, exist_ok=True)
        with self.metadata_path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(metadata, f, sort_keys=False, allow_unicode=True)

    @QtCore.Slot()
    def run(self) -> None:
        try:
            token = GUN_DAQ_TOKEN
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            self._write_event(f"XY imaging started: {len(self.x_values)} x {len(self.y_values)} pixels, file={self.output_path}")
            self._zero_tdk_supplies()
            srs_status = self._ensure_collector()
            deflection_status = self.kimball.ensure_deflection_enabled(token=token)
            self._write_event(f"Kimball deflection switch ON before XY imaging: {deflection_status.get('daqmx_line')}")
            self._write_metadata({**srs_status, "kimball_deflection_switch": deflection_status})
            self._write_event(f"Metadata written: {self.metadata_path}")

            label_by_name = {name: self.rbd.labels.get(name, name) for name in self.rbd.devices}
            label_cols = {name: self._safe_col(label) for name, label in label_by_name.items()}
            fieldnames = [
                "timestamp_local", "pixel_index", "ix", "iy", "x_deflection_V", "y_deflection_V",
                "n_samples_requested", "settle_s", "sample_period_s", "collector_voltage_V",
                "total_yield", "yield_numerator_A", "yield_denominator_A", "use_abs_currents",
            ]
            for name in self.rbd.devices:
                fieldnames.extend([f"{label_cols[name]}_mean_A", f"{label_cols[name]}_std_A", f"{label_cols[name]}_n_valid", f"{name}_internal_id"])

            with self.output_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
                writer.writeheader()
                pixel = 0
                for iy, y in enumerate(self.y_values):
                    xs = list(enumerate(self.x_values))
                    if self.serpentine and (iy % 2 == 1):
                        xs = list(reversed(xs))
                    # Set Y once per line, then X for each pixel.
                    self.kimball.write_control_engineering("y_center", y, token=token, allow_large=True)
                    for ix, x in xs:
                        if self._stop_requested:
                            raise InterruptedError("XY imaging stopped by user.")
                        pixel += 1
                        self.kimball.write_control_engineering("x_center", x, token=token, allow_large=True)
                        if self.settle_s > 0:
                            time.sleep(self.settle_s)
                        self.rbd.flush_input_all()
                        if self.discard_first > 0:
                            self.rbd.discard_samples(self.discard_first, sample_period_s=self.sample_period_s)
                        samples_by_name: dict[str, list[float]] = {name: [] for name in self.rbd.devices}
                        for _ in range(self.n_samples):
                            if self._stop_requested:
                                raise InterruptedError("XY imaging stopped by user.")
                            readings = self.rbd.read_all_once(latest=False)
                            for r in readings:
                                if r.current_A is not None:
                                    samples_by_name.setdefault(r.name, []).append(float(r.current_A))
                            if self.sample_period_s > 0:
                                time.sleep(self.sample_period_s)
                        currents_by_label: dict[str, float] = {}
                        row: dict[str, Any] = {
                            "timestamp_local": datetime.now().isoformat(timespec="milliseconds"),
                            "pixel_index": pixel,
                            "ix": ix,
                            "iy": iy,
                            "x_deflection_V": x,
                            "y_deflection_V": y,
                            "n_samples_requested": self.n_samples,
                            "settle_s": self.settle_s,
                            "sample_period_s": self.sample_period_s,
                            "collector_voltage_V": self.collector_voltage_V,
                            "use_abs_currents": self.use_abs_currents,
                        }
                        for name, vals in samples_by_name.items():
                            avg = sum(vals) / len(vals) if vals else None
                            std = None
                            if vals and len(vals) > 1:
                                m = avg
                                std = (sum((v - m) ** 2 for v in vals) / (len(vals) - 1)) ** 0.5
                            label = label_by_name[name]
                            col = label_cols[name]
                            if avg is not None:
                                currents_by_label[label] = avg
                            row[f"{col}_mean_A"] = avg
                            row[f"{col}_std_A"] = std
                            row[f"{col}_n_valid"] = len(vals)
                            row[f"{name}_internal_id"] = name
                        yld, numerator, denominator = self._calculate_yield(currents_by_label)
                        row["total_yield"] = yld
                        row["yield_numerator_A"] = numerator
                        row["yield_denominator_A"] = denominator
                        writer.writerow(row)
                        f.flush()
                        self.pixel_ready.emit(row)
                        self._write_event(f"Pixel {pixel}: X={x:g} V, Y={y:g} V, TEY={yld}")
            self._write_event("XY imaging finished normally.")
            self.finished.emit(str(self.output_path))
        except InterruptedError as exc:
            self._write_event(str(exc))
            self.finished.emit(str(self.output_path))
        except Exception as exc:
            self._write_event(f"ERROR: {exc}")
            self.failed.emit(str(exc))


class RFAControlWindow(QtWidgets.QMainWindow):
    def __init__(
        self,
        *,
        rbd: RBDManager,
        tdk: TDKManager,
        srs,
        kimball: KimballDAQ,
        simulate: bool,
        output_dir: str = "data",
        rbd_profiles: dict[str, Any] | None = None,
    ) -> None:
        super().__init__()
        self.rbd = rbd
        self.tdk = tdk
        self.srs = srs
        self.kimball = kimball
        self.simulate = simulate
        self.output_dir = Path(output_dir)
        self.rbd_profiles = rbd_profiles or default_rbd_profiles()
        self.rbd_logger: RBDCsvLogger | None = None
        self.t0 = time.time()
        self.rbd_history: dict[str, deque[tuple[float, float]]] = {name: deque(maxlen=1500) for name in self.rbd.devices}
        self.tdk_history: dict[str, deque[tuple[float, float]]] = {name: deque(maxlen=1500) for name in self.tdk.names()}
        self.monitor_ticks = 0
        self.rbd_sampling_active = False
        self.kimball_source_abort_requested = False
        self.kimball_last_source_target_V = 0.0
        self.mdrive_config_path = default_config_path("mdrive.yaml")
        self.mdrive_config = load_yaml_file(self.mdrive_config_path)

        self.setWindowTitle(f"RFA Python DAQ v0.19.17 ({'SIMULATED' if simulate else 'REAL HARDWARE'})")
        self.resize(1500, 900)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        top = QtWidgets.QHBoxLayout()
        root.addLayout(top)
        self.mode_label = QtWidgets.QLabel("SIMULATED" if simulate else "REAL HARDWARE")
        self.mode_label.setStyleSheet(
            "font-weight: bold; padding: 4px; color: #006400;" if simulate
            else "font-weight: bold; padding: 4px; color: #8b0000;"
        )
        top.addWidget(self.mode_label)
        self.status_label = QtWidgets.QLabel("Ready")
        top.addWidget(self.status_label, stretch=1)

        self.tabs = QtWidgets.QTabWidget()
        root.addWidget(self.tabs, stretch=1)
        self.monitor_tab = QtWidgets.QWidget()
        self.pico_tab = QtWidgets.QWidget()
        self.power_tab = QtWidgets.QWidget()
        self.kimball_tab = QtWidgets.QWidget()
        self.motion_tab = QtWidgets.QWidget()
        self.measure_tab = QtWidgets.QWidget()
        self.imaging_tab = QtWidgets.QWidget()
        self.tabs.addTab(self.monitor_tab, "Monitor")
        self.tabs.addTab(self.pico_tab, "Picoammeters")
        self.tabs.addTab(self.power_tab, "Power Supplies")
        self.tabs.addTab(self.kimball_tab, "Kimball DAQ")
        self.tabs.addTab(self.motion_tab, "MDrive Motion")
        self.tabs.addTab(self.measure_tab, "Bias Scan")
        self.tabs.addTab(self.imaging_tab, "XY Imaging")

        self.scan_thread = None
        self.scan_worker = None
        self.imaging_thread = None
        self.imaging_worker = None
        self.imaging_image_array = None
        self.imaging_x_values: list[float] = []
        self.imaging_y_values: list[float] = []
        self.imaging_pixel_rows: dict[tuple[int, int], dict[str, Any]] = {}
        self.imaging_selected_ix: int | None = None
        self.imaging_selected_iy: int | None = None

        self._build_monitor_tab()
        self._build_pico_tab()
        self._build_power_tab()
        self._build_kimball_tab()
        self._build_motion_tab()
        self._build_measure_tab()
        self._build_imaging_tab()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.monitor_tick)

        self.kimball_timer = QtCore.QTimer(self)
        self.kimball_timer.timeout.connect(self.kimball_read_meters)


    # ------------------------------------------------------------------
    # Shared helpers
    # ------------------------------------------------------------------
    def append_log(self, text: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        line = f"{stamp}  {text}"
        self.status_label.setText(text)
        for box_name in ("monitor_log", "pico_log", "power_log", "kimball_log", "motion_log", "imaging_log"):
            box = getattr(self, box_name, None)
            if box is not None:
                box.appendPlainText(line)

    def error_box(self, title: str, exc: Exception | str) -> None:
        text = str(exc)
        self.append_log(f"ERROR: {text}")
        QtWidgets.QMessageBox.critical(self, title, text)

    def ensure_rbd_sampling_started(self) -> None:
        """Open, initialize, and start interval sampling on all RBDs.

        Read Once works by starting/stopping each RBD internally. Continuous GUI
        monitoring needs the RBDs to be left in interval-sampling mode first;
        otherwise read_latest_sample() correctly times out waiting for data.
        """
        if self.rbd_sampling_active:
            return

        open_results = self.rbd.open_all()
        bad = {k: v for k, v in open_results.items() if str(v).startswith("ERROR")}
        if bad:
            raise RuntimeError("RBD open failed: " + "; ".join(f"{k}: {v}" for k, v in bad.items()))

        init_results = self.rbd.initialize_all()
        bad = {k: v for k, v in init_results.items() if str(v).startswith("ERROR")}
        if bad:
            raise RuntimeError("RBD initialize failed: " + "; ".join(f"{k}: {v}" for k, v in bad.items()))

        start_results = self.rbd.start_all()
        bad = {k: v for k, v in start_results.items() if str(v).startswith("ERROR")}
        if bad:
            raise RuntimeError("RBD start failed: " + "; ".join(f"{k}: {v}" for k, v in bad.items()))

        self.rbd_sampling_active = True
        self.append_log("RBD interval sampling started")

    def hv_armed(self) -> bool:
        return self.hv_enable_check.isChecked()

    def arm_tdk_if_allowed(self) -> None:
        if not self.hv_armed():
            raise RuntimeError("Check 'Enable HV-changing buttons' first.")
        for name in self.tdk.names():
            dev = self.tdk.get(name)
            if hasattr(dev, "arm_hv_changes"):
                dev.arm_hv_changes(HV_TOKEN)

    # ------------------------------------------------------------------
    # Monitor tab
    # ------------------------------------------------------------------
    def _build_monitor_tab(self) -> None:
        layout = QtWidgets.QVBoxLayout(self.monitor_tab)

        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)
        self.monitor_start_btn = QtWidgets.QPushButton("Start Monitor")
        self.monitor_stop_btn = QtWidgets.QPushButton("Stop")
        self.monitor_read_once_btn = QtWidgets.QPushButton("Read Once")
        self.monitor_tdk_check = QtWidgets.QCheckBox("Update TDK every 5 ticks")
        self.monitor_srs_check = QtWidgets.QCheckBox("Update SRS every 5 ticks")
        self.monitor_rbd_period = QtWidgets.QDoubleSpinBox()
        self.monitor_rbd_period.setRange(0.2, 30.0)
        self.monitor_rbd_period.setValue(1.0)
        self.monitor_rbd_period.setSuffix(" s")
        controls.addWidget(self.monitor_start_btn)
        controls.addWidget(self.monitor_stop_btn)
        controls.addWidget(self.monitor_read_once_btn)
        controls.addWidget(QtWidgets.QLabel("Period:"))
        controls.addWidget(self.monitor_rbd_period)
        controls.addWidget(self.monitor_tdk_check)
        controls.addWidget(self.monitor_srs_check)
        controls.addStretch()
        self.monitor_stop_btn.setEnabled(False)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        layout.addWidget(splitter, stretch=1)

        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        splitter.addWidget(left)

        self.monitor_rbd_table = self._make_rbd_table()
        left_layout.addWidget(QtWidgets.QLabel("Picoammeter currents"))
        left_layout.addWidget(self.monitor_rbd_table, stretch=2)

        self.monitor_tdk_table = self._make_tdk_table()
        left_layout.addWidget(QtWidgets.QLabel("TDK actual voltages"))
        left_layout.addWidget(self.monitor_tdk_table, stretch=1)

        self.monitor_srs_table = QtWidgets.QTableWidget(1, 7)
        self.monitor_srs_table.setHorizontalHeaderLabels(["Port", "Range", "Setpoint", "Output", "Interlock", "Overload", "IDN"])
        self.monitor_srs_table.horizontalHeader().setStretchLastSection(True)
        left_layout.addWidget(QtWidgets.QLabel("SRS DC205 collector supply"))
        left_layout.addWidget(self.monitor_srs_table)

        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        splitter.addWidget(right)
        splitter.setSizes([800, 650])

        self.current_plot = pg.PlotWidget(title="RBD currents")
        self.current_plot.setLabel("bottom", "Time", units="s")
        self.current_plot.setLabel("left", "Current", units="A")
        self.current_plot.addLegend()
        self.rbd_curves = {}
        for name in self.rbd.devices:
            label = self.rbd.labels.get(name, name)
            pen = pg.mkPen(RBD_PLOT_COLORS.get(name, None), width=2.5)
            self.rbd_curves[name] = self.current_plot.plot([], [], pen=pen, name=label)
        right_layout.addWidget(self.current_plot, stretch=2)

        self.voltage_plot = pg.PlotWidget(title="TDK signed actual voltages")
        self.voltage_plot.setLabel("bottom", "Time", units="s")
        self.voltage_plot.setLabel("left", "Voltage", units="V")
        self.voltage_plot.addLegend()
        self.tdk_curves = {}
        for name in self.tdk.names():
            pen = pg.mkPen(TDK_PLOT_COLORS.get(name, None), width=2.5)
            self.tdk_curves[name] = self.voltage_plot.plot([], [], pen=pen, name=f"{name}: {TDK_ELECTRODE_LABELS.get(name, name)}")
        right_layout.addWidget(self.voltage_plot, stretch=1)

        self.monitor_log = QtWidgets.QPlainTextEdit()
        self.monitor_log.setReadOnly(True)
        self.monitor_log.setMaximumBlockCount(1000)
        right_layout.addWidget(self.monitor_log, stretch=1)

        self.monitor_start_btn.clicked.connect(self.start_monitor)
        self.monitor_stop_btn.clicked.connect(self.stop_monitor)
        self.monitor_read_once_btn.clicked.connect(self.read_all_once)

    # ------------------------------------------------------------------
    # Picoammeter tab
    # ------------------------------------------------------------------
    def _build_pico_tab(self) -> None:
        layout = QtWidgets.QVBoxLayout(self.pico_tab)
        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)
        self.rbd_connect_btn = QtWidgets.QPushButton("Connect All")
        self.rbd_init_btn = QtWidgets.QPushButton("Initialize All")
        self.rbd_start_btn = QtWidgets.QPushButton("Start Sampling")
        self.rbd_stop_btn = QtWidgets.QPushButton("Stop Sampling")
        self.rbd_once_btn = QtWidgets.QPushButton("Read Once")
        self.rbd_save_check = QtWidgets.QCheckBox("Save CSV")
        self.rbd_save_check.setChecked(True)
        self.rbd_period_spin = QtWidgets.QDoubleSpinBox()
        self.rbd_period_spin.setRange(0.2, 30.0)
        self.rbd_period_spin.setValue(1.0)
        self.rbd_period_spin.setSuffix(" s")
        for w in [self.rbd_connect_btn, self.rbd_init_btn, self.rbd_once_btn, self.rbd_start_btn, self.rbd_stop_btn]:
            controls.addWidget(w)
        controls.addWidget(QtWidgets.QLabel("Read period:"))
        controls.addWidget(self.rbd_period_spin)
        controls.addWidget(self.rbd_save_check)
        controls.addStretch()
        self.rbd_stop_btn.setEnabled(False)

        self.pico_table = self._make_rbd_table()
        layout.addWidget(self.pico_table, stretch=2)

        self.pico_log = QtWidgets.QPlainTextEdit()
        self.pico_log.setReadOnly(True)
        self.pico_log.setMaximumBlockCount(1000)
        layout.addWidget(self.pico_log, stretch=1)

        self.rbd_connect_btn.clicked.connect(self.connect_rbd)
        self.rbd_init_btn.clicked.connect(self.initialize_rbd)
        self.rbd_start_btn.clicked.connect(self.start_rbd_sampling)
        self.rbd_stop_btn.clicked.connect(self.stop_rbd_sampling)
        self.rbd_once_btn.clicked.connect(self.read_rbd_once)

    def _make_rbd_table(self) -> QtWidgets.QTableWidget:
        headers = ["Electrode", "Internal ID", "Port", "Current", "RBD status", "Range", "Raw response", "Error"]
        table = QtWidgets.QTableWidget(len(self.rbd.devices), len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.horizontalHeader().setStretchLastSection(True)
        table.setAlternatingRowColors(True)
        table.name_to_row = {}  # type: ignore[attr-defined]
        for row, (name, dev) in enumerate(self.rbd.devices.items()):
            table.name_to_row[name] = row  # type: ignore[attr-defined]
            label = self.rbd.labels.get(name, RBD_TO_ELECTRODE_FOR_TOTAL.get(name, name))
            values = [label, name, dev.port_name, "", "", "", "", ""]
            for col, val in enumerate(values):
                item = QtWidgets.QTableWidgetItem(str(val))
                item.setFlags(item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
                table.setItem(row, col, item)
        table.setColumnHidden(1, True)  # keep historical dwarf nickname internal only
        table.resizeColumnsToContents()
        return table

    def connect_rbd(self) -> None:
        try:
            for name, status in self.rbd.open_all().items():
                self.append_log(f"RBD {self.rbd.labels.get(name, name)}: {status}")
        except Exception as exc:
            self.error_box("RBD connect failed", exc)

    def initialize_rbd(self) -> None:
        try:
            for name, status in self.rbd.initialize_all().items():
                self.append_log(f"RBD {self.rbd.labels.get(name, name)}: {status}")
        except Exception as exc:
            self.error_box("RBD initialize failed", exc)

    def start_rbd_sampling(self) -> None:
        try:
            self.ensure_rbd_sampling_started()
            if self.rbd_save_check.isChecked() and self.rbd_logger is None:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.rbd_logger = RBDCsvLogger(self.output_dir / f"rbd_monitor_{ts}.csv")
                self.append_log(f"Saving RBD CSV: {self.rbd_logger.output_path}")
            # Start the same GUI timer used by the Monitor tab so the Picoammeters
            # tab updates continuously too.
            self.timer.start(int(float(self.rbd_period_spin.value()) * 1000))
            self.rbd_start_btn.setEnabled(False)
            self.rbd_stop_btn.setEnabled(True)
            self.monitor_start_btn.setEnabled(False)
            self.monitor_stop_btn.setEnabled(True)
            self.append_log("RBD GUI sampling display started")
        except Exception as exc:
            self.error_box("RBD start failed", exc)

    def stop_rbd_sampling(self) -> None:
        try:
            self.timer.stop()
            self.rbd.stop_all()
            self.rbd_sampling_active = False
            if self.rbd_logger:
                path = self.rbd_logger.output_path
                self.rbd_logger.close()
                self.rbd_logger = None
                self.append_log(f"Saved RBD CSV: {path}")
            self.rbd_start_btn.setEnabled(True)
            self.rbd_stop_btn.setEnabled(False)
            self.monitor_start_btn.setEnabled(True)
            self.monitor_stop_btn.setEnabled(False)
            self.append_log("RBD sampling stopped")
        except Exception as exc:
            self.error_box("RBD stop failed", exc)

    def read_rbd_once(self) -> None:
        try:
            readings = self.rbd.sample_all_once()
            self.handle_rbd_readings(readings)
        except Exception as exc:
            self.error_box("RBD read failed", exc)

    # ------------------------------------------------------------------
    # Power-supply tab
    # ------------------------------------------------------------------
    def _build_power_tab(self) -> None:
        layout = QtWidgets.QVBoxLayout(self.power_tab)

        safety = QtWidgets.QHBoxLayout()
        layout.addLayout(safety)
        self.hv_enable_check = QtWidgets.QCheckBox("Enable HV-changing buttons")
        self.hv_enable_check.setStyleSheet("font-weight: bold; color: #8b0000;")
        self.zero_all_btn = QtWidgets.QPushButton("ZERO ALL TDK + output OFF")
        self.safe_shutdown_btn = QtWidgets.QPushButton("Safe Shutdown: TDK zero/off + SRS *RST")
        safety.addWidget(self.hv_enable_check)
        safety.addWidget(self.zero_all_btn)
        safety.addWidget(self.safe_shutdown_btn)
        safety.addStretch()

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        layout.addWidget(splitter, stretch=1)

        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        splitter.addWidget(left)

        left_layout.addWidget(QtWidgets.QLabel("TDK/FUG PHV supplies"))
        self.power_tdk_table = self._make_tdk_table()
        left_layout.addWidget(self.power_tdk_table, stretch=1)

        tdk_controls = QtWidgets.QGroupBox("Selected TDK supply")
        tdk_form = QtWidgets.QGridLayout(tdk_controls)
        self.tdk_supply_combo = QtWidgets.QComboBox()
        for name in self.tdk.names():
            self.tdk_supply_combo.addItem(f"{name} — {TDK_ELECTRODE_LABELS.get(name, name)}", name)
        self.tdk_voltage_spin = QtWidgets.QDoubleSpinBox()
        self.tdk_voltage_spin.setRange(-100000.0, 100000.0)
        self.tdk_voltage_spin.setDecimals(4)
        self.tdk_voltage_spin.setValue(0.0)
        self.tdk_voltage_spin.setSuffix(" V")
        self.tdk_current_spin = QtWidgets.QDoubleSpinBox()
        self.tdk_current_spin.setRange(0.000001, 1000.0)
        self.tdk_current_spin.setDecimals(6)
        self.tdk_current_spin.setValue(0.1)
        self.tdk_current_spin.setSuffix(" mA")
        self.tdk_tol_spin = QtWidgets.QDoubleSpinBox()
        self.tdk_tol_spin.setRange(0.001, 1000.0)
        self.tdk_tol_spin.setValue(0.2)
        self.tdk_tol_spin.setSuffix(" V")
        self.tdk_timeout_spin = QtWidgets.QDoubleSpinBox()
        self.tdk_timeout_spin.setRange(0.5, 300.0)
        self.tdk_timeout_spin.setValue(10.0)
        self.tdk_timeout_spin.setSuffix(" s")
        self.tdk_ramp_mode_combo = QtWidgets.QComboBox()
        for m in ["no change", "0", "1", "2", "4"]:
            self.tdk_ramp_mode_combo.addItem(m)
        self.tdk_apply_btn = QtWidgets.QPushButton("Apply voltage && wait")
        self.tdk_zero_wait_btn = QtWidgets.QPushButton("Zero && wait")
        self.tdk_output_off_btn = QtWidgets.QPushButton("Output OFF")
        self.tdk_read_status_btn = QtWidgets.QPushButton("Read all TDK status")
        row = 0
        tdk_form.addWidget(QtWidgets.QLabel("Supply"), row, 0); tdk_form.addWidget(self.tdk_supply_combo, row, 1, 1, 3); row += 1
        tdk_form.addWidget(QtWidgets.QLabel("Signed voltage"), row, 0); tdk_form.addWidget(self.tdk_voltage_spin, row, 1)
        tdk_form.addWidget(QtWidgets.QLabel("Current limit"), row, 2); tdk_form.addWidget(self.tdk_current_spin, row, 3); row += 1
        tdk_form.addWidget(QtWidgets.QLabel("Tolerance"), row, 0); tdk_form.addWidget(self.tdk_tol_spin, row, 1)
        tdk_form.addWidget(QtWidgets.QLabel("Timeout"), row, 2); tdk_form.addWidget(self.tdk_timeout_spin, row, 3); row += 1
        tdk_form.addWidget(QtWidgets.QLabel("Ramp mode"), row, 0); tdk_form.addWidget(self.tdk_ramp_mode_combo, row, 1); row += 1
        tdk_form.addWidget(self.tdk_apply_btn, row, 0, 1, 2); tdk_form.addWidget(self.tdk_zero_wait_btn, row, 2, 1, 2); row += 1
        tdk_form.addWidget(self.tdk_output_off_btn, row, 0, 1, 2); tdk_form.addWidget(self.tdk_read_status_btn, row, 2, 1, 2)
        left_layout.addWidget(tdk_controls)

        srs_group = QtWidgets.QGroupBox("Collector SRS DC205")
        srs_form = QtWidgets.QGridLayout(srs_group)
        self.srs_range_combo = QtWidgets.QComboBox()
        self.srs_range_combo.addItems(["RANGE1", "RANGE10", "RANGE100"])
        self.srs_voltage_spin = QtWidgets.QDoubleSpinBox()
        self.srs_voltage_spin.setRange(-100.0, 100.0)
        self.srs_voltage_spin.setDecimals(6)
        self.srs_voltage_spin.setValue(0.0)
        self.srs_voltage_spin.setSuffix(" V")
        self.srs_connect_btn = QtWidgets.QPushButton("Connect/IDN")
        self.srs_set_range_btn = QtWidgets.QPushButton("Set range")
        self.srs_set_voltage_btn = QtWidgets.QPushButton("Set voltage")
        self.srs_output_on_btn = QtWidgets.QPushButton("Output ON")
        self.srs_output_off_btn = QtWidgets.QPushButton("Output OFF")
        self.srs_reset_btn = QtWidgets.QPushButton("*RST + status")
        self.srs_status_btn = QtWidgets.QPushButton("Read status")
        srs_form.addWidget(QtWidgets.QLabel("Range"), 0, 0); srs_form.addWidget(self.srs_range_combo, 0, 1)
        srs_form.addWidget(QtWidgets.QLabel("Voltage"), 0, 2); srs_form.addWidget(self.srs_voltage_spin, 0, 3)
        srs_form.addWidget(self.srs_connect_btn, 1, 0); srs_form.addWidget(self.srs_set_range_btn, 1, 1)
        srs_form.addWidget(self.srs_set_voltage_btn, 1, 2); srs_form.addWidget(self.srs_status_btn, 1, 3)
        srs_form.addWidget(self.srs_output_on_btn, 2, 0); srs_form.addWidget(self.srs_output_off_btn, 2, 1)
        srs_form.addWidget(self.srs_reset_btn, 2, 2, 1, 2)
        left_layout.addWidget(srs_group)

        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        splitter.addWidget(right)
        splitter.setSizes([950, 520])

        self.power_log = QtWidgets.QPlainTextEdit()
        self.power_log.setReadOnly(True)
        self.power_log.setMaximumBlockCount(1000)
        right_layout.addWidget(self.power_log, stretch=1)

        self.zero_all_btn.clicked.connect(self.zero_all_tdk)
        self.safe_shutdown_btn.clicked.connect(self.safe_shutdown)
        self.tdk_apply_btn.clicked.connect(self.tdk_apply_selected)
        self.tdk_zero_wait_btn.clicked.connect(self.tdk_zero_selected)
        self.tdk_output_off_btn.clicked.connect(self.tdk_output_off_selected)
        self.tdk_read_status_btn.clicked.connect(self.update_tdk_status)
        self.srs_connect_btn.clicked.connect(self.connect_srs)
        self.srs_set_range_btn.clicked.connect(self.srs_set_range)
        self.srs_set_voltage_btn.clicked.connect(self.srs_set_voltage)
        self.srs_output_on_btn.clicked.connect(lambda: self.srs_set_output(True))
        self.srs_output_off_btn.clicked.connect(lambda: self.srs_set_output(False))
        self.srs_reset_btn.clicked.connect(self.srs_reset)
        self.srs_status_btn.clicked.connect(self.update_srs_status)

    def _make_tdk_table(self) -> QtWidgets.QTableWidget:
        headers = ["Supply", "Electrode", "Port", "Polarity", "Output", "Setpoint", "Actual signed", "I limit", "I measured", "IDN"]
        names = self.tdk.names()
        table = QtWidgets.QTableWidget(len(names), len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.horizontalHeader().setStretchLastSection(True)
        table.setAlternatingRowColors(True)
        table.name_to_row = {}  # type: ignore[attr-defined]
        for row, name in enumerate(names):
            cfg = self.tdk.configs[name]
            table.name_to_row[name] = row  # type: ignore[attr-defined]
            values = [name, TDK_ELECTRODE_LABELS.get(name, name), cfg.port, cfg.polarity_mode, "", "", "", "", "", ""]
            for col, val in enumerate(values):
                item = QtWidgets.QTableWidgetItem(str(val))
                item.setFlags(item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
                table.setItem(row, col, item)
        table.resizeColumnsToContents()
        return table

    def connect_srs(self) -> None:
        try:
            self.srs.open()
            self.append_log(f"SRS IDN: {self.srs.identify()}")
            self.update_srs_status()
        except Exception as exc:
            self.error_box("SRS connect failed", exc)

    def srs_set_range(self) -> None:
        try:
            rng = self.srs_range_combo.currentText()
            self.srs.set_range(rng)
            self.append_log(f"SRS range set to {rng}")
            self.update_srs_status()
        except Exception as exc:
            self.error_box("SRS set range failed", exc)

    def srs_set_voltage(self) -> None:
        try:
            v = float(self.srs_voltage_spin.value())
            self.srs.set_voltage(v)
            self.append_log(f"SRS voltage setpoint set to {v:g} V")
            self.update_srs_status()
        except Exception as exc:
            self.error_box("SRS set voltage failed", exc)

    def srs_set_output(self, on: bool) -> None:
        try:
            self.srs.set_output(on)
            self.append_log(f"SRS output {'ON' if on else 'OFF'}")
            self.update_srs_status()
        except Exception as exc:
            self.error_box("SRS output command failed", exc)

    def srs_reset(self) -> None:
        try:
            self.srs.reset_default()
            self.append_log("SRS *RST sent")
            self.update_srs_status()
        except Exception as exc:
            self.error_box("SRS reset failed", exc)

    def tdk_selected_name(self) -> str:
        return str(self.tdk_supply_combo.currentData())

    def tdk_ramp_mode_value(self) -> int | None:
        txt = self.tdk_ramp_mode_combo.currentText()
        return None if txt == "no change" else int(txt)

    def tdk_apply_selected(self) -> None:
        try:
            self.arm_tdk_if_allowed()
            name = self.tdk_selected_name()
            dev = self.tdk.get(name)
            actual = dev.set_signed_voltage_and_wait(
                float(self.tdk_voltage_spin.value()),
                current_limit_A=float(self.tdk_current_spin.value()) * 1e-3,
                ramp_mode=self.tdk_ramp_mode_value(),
                tolerance_V=float(self.tdk_tol_spin.value()),
                timeout_s=float(self.tdk_timeout_spin.value()),
            )
            self.append_log(f"TDK {name}: applied {self.tdk_voltage_spin.value():g} V; actual {actual:g} V")
            self.update_tdk_status()
        except Exception as exc:
            self.error_box("TDK apply failed", exc)

    def tdk_zero_selected(self) -> None:
        try:
            self.arm_tdk_if_allowed()
            name = self.tdk_selected_name()
            dev = self.tdk.get(name)
            actual = dev.zero_and_wait(
                current_limit_A=float(self.tdk_current_spin.value()) * 1e-3,
                tolerance_V=float(self.tdk_tol_spin.value()),
                timeout_s=float(self.tdk_timeout_spin.value()),
            )
            self.append_log(f"TDK {name}: zero reached; actual {actual:g} V")
            self.update_tdk_status()
        except Exception as exc:
            self.error_box("TDK zero failed", exc)

    def tdk_output_off_selected(self) -> None:
        try:
            self.arm_tdk_if_allowed()
            name = self.tdk_selected_name()
            dev = self.tdk.get(name)
            dev.output(False)
            self.append_log(f"TDK {name}: output OFF")
            self.update_tdk_status()
        except Exception as exc:
            self.error_box("TDK output off failed", exc)

    def zero_all_tdk(self) -> None:
        try:
            self.arm_tdk_if_allowed()
            tol = float(self.tdk_tol_spin.value())
            timeout = float(self.tdk_timeout_spin.value())
            current_A = float(self.tdk_current_spin.value()) * 1e-3
            for name in self.tdk.names():
                dev = self.tdk.get(name)
                actual = dev.zero_and_wait(current_limit_A=current_A, tolerance_V=tol, timeout_s=timeout, output_off_after_zero=True)
                self.append_log(f"TDK {name}: zero/off done; actual {actual:g} V")
            self.update_tdk_status()
        except Exception as exc:
            self.error_box("TDK zero all failed", exc)

    def safe_shutdown(self) -> None:
        try:
            self.zero_all_tdk()
            # DC205 *RST is safer than output-off alone: it sets output OFF,
            # voltage 0 V, and RANGE1, matching the manual/default state.
            self.srs.reset_default()
            self.append_log("Safe shutdown: SRS *RST sent")
            self.update_srs_status()
        except Exception as exc:
            self.error_box("Safe shutdown failed", exc)


    # ------------------------------------------------------------------
    # Kimball / NI-DAQ safe foundation tab
    # ------------------------------------------------------------------
    def _build_kimball_tab(self) -> None:
        layout = QtWidgets.QVBoxLayout(self.kimball_tab)

        info = QtWidgets.QLabel(
            "v0.17.1 Kimball NI-DAQ: assume 0s, zero all configured AO outputs, "
            "preview/apply electrostatic controls, ramp Source in Source mode, and normal Source shutdown. "
            "ECC mode and scan integration are still intentionally disabled."
        )
        info.setWordWrap(True)
        layout.addWidget(info)

        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)
        self.kimball_list_devices_btn = QtWidgets.QPushButton("List NI devices")
        self.kimball_read_meters_btn = QtWidgets.QPushButton("Read meters")
        self.kimball_auto_meters_check = QtWidgets.QCheckBox("Auto-assume 0s")
        self.kimball_zero_btn = QtWidgets.QPushButton("Zero all Kimball AO outputs")
        self.kimball_enable_check = QtWidgets.QCheckBox("Enable gun-DAQ writes")
        self.kimball_enable_check.setStyleSheet("font-weight: bold; color: #8b0000;")
        controls.addWidget(self.kimball_list_devices_btn)
        controls.addWidget(self.kimball_read_meters_btn)
        controls.addWidget(self.kimball_auto_meters_check)
        controls.addWidget(self.kimball_zero_btn)
        controls.addWidget(self.kimball_enable_check)
        controls.addStretch()

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        layout.addWidget(splitter, stretch=1)

        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        splitter.addWidget(left)

        self.kimball_meter_table = QtWidgets.QTableWidget(len(self.kimball.config.meters), 5)
        self.kimball_meter_table.setHorizontalHeaderLabels(["Meter", "Channel", "Raw V", "Value", "Units"])
        self.kimball_meter_table.horizontalHeader().setStretchLastSection(True)
        self.kimball_meter_table.setAlternatingRowColors(True)
        self.kimball_meter_table.name_to_row = {}  # type: ignore[attr-defined]
        for row, (name, meter) in enumerate(self.kimball.config.meters.items()):
            self.kimball_meter_table.name_to_row[name] = row  # type: ignore[attr-defined]
            values = [name, meter.physical_channel, "", "", meter.units]
            for col, val in enumerate(values):
                item = QtWidgets.QTableWidgetItem(str(val))
                item.setFlags(item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
                self.kimball_meter_table.setItem(row, col, item)
        self.kimball_meter_table.resizeColumnsToContents()
        left_layout.addWidget(QtWidgets.QLabel("Kimball meter inputs"))
        left_layout.addWidget(self.kimball_meter_table, stretch=2)

        self.kimball_control_table = QtWidgets.QTableWidget(len(self.kimball.config.controls), 5)
        self.kimball_control_table.setHorizontalHeaderLabels(["Control", "Channel", "Eng. max", "AO range", "Units"])
        self.kimball_control_table.horizontalHeader().setStretchLastSection(True)
        self.kimball_control_table.setAlternatingRowColors(True)
        for row, (name, ctrl) in enumerate(self.kimball.config.controls.items()):
            values = [name, ctrl.physical_channel, ctrl.engineering_max, ctrl.output_range_V, ctrl.units]
            for col, val in enumerate(values):
                item = QtWidgets.QTableWidgetItem(str(val))
                item.setFlags(item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
                self.kimball_control_table.setItem(row, col, item)
        self.kimball_control_table.resizeColumnsToContents()
        left_layout.addWidget(QtWidgets.QLabel("Kimball analog-output controls"))
        left_layout.addWidget(self.kimball_control_table, stretch=1)

        electro_group = QtWidgets.QGroupBox("Electrostatic controls from LUT / manual targets")
        electro = QtWidgets.QGridLayout(electro_group)
        row = 0
        self.kimball_energy_eV_spin = QtWidgets.QDoubleSpinBox()
        self.kimball_energy_eV_spin.setRange(0, 20000)
        self.kimball_energy_eV_spin.setDecimals(1)
        self.kimball_energy_eV_spin.setSingleStep(50)
        self.kimball_energy_eV_spin.setValue(1000)
        self.kimball_grid_V_spin = QtWidgets.QDoubleSpinBox()
        self.kimball_grid_V_spin.setRange(0, 500)
        self.kimball_grid_V_spin.setDecimals(2)
        self.kimball_grid_V_spin.setSingleStep(1)
        self.kimball_focus_kV_spin = QtWidgets.QDoubleSpinBox()
        self.kimball_focus_kV_spin.setRange(0, 10)
        self.kimball_focus_kV_spin.setDecimals(4)
        self.kimball_focus_kV_spin.setSingleStep(0.01)
        self.kimball_x_V_spin = QtWidgets.QDoubleSpinBox()
        self.kimball_x_V_spin.setRange(-300, 300)
        self.kimball_x_V_spin.setDecimals(3)
        self.kimball_y_V_spin = QtWidgets.QDoubleSpinBox()
        self.kimball_y_V_spin.setRange(-300, 300)
        self.kimball_y_V_spin.setDecimals(3)
        electro.addWidget(QtWidgets.QLabel("Energy target (eV)"), row, 0)
        electro.addWidget(self.kimball_energy_eV_spin, row, 1)
        electro.addWidget(QtWidgets.QLabel("Grid target (V)"), row, 2)
        electro.addWidget(self.kimball_grid_V_spin, row, 3)
        row += 1
        electro.addWidget(QtWidgets.QLabel("Focus target (kV; manual)"), row, 0)
        electro.addWidget(self.kimball_focus_kV_spin, row, 1)
        electro.addWidget(QtWidgets.QLabel("X center (V)"), row, 2)
        electro.addWidget(self.kimball_x_V_spin, row, 3)
        electro.addWidget(QtWidgets.QLabel("Y center (V)"), row, 4)
        electro.addWidget(self.kimball_y_V_spin, row, 5)
        row += 1
        self.kimball_apply_energy_check = QtWidgets.QCheckBox("Apply Energy")
        self.kimball_apply_energy_check.setChecked(True)
        self.kimball_apply_grid_check = QtWidgets.QCheckBox("Apply Grid")
        self.kimball_apply_grid_check.setChecked(True)
        self.kimball_apply_focus_check = QtWidgets.QCheckBox("Apply Focus")
        self.kimball_apply_focus_check.setChecked(False)
        self.kimball_apply_deflection_check = QtWidgets.QCheckBox("Apply X/Y deflection")
        self.kimball_apply_deflection_check.setChecked(True)
        electro.addWidget(self.kimball_apply_energy_check, row, 0)
        electro.addWidget(self.kimball_apply_grid_check, row, 1)
        electro.addWidget(self.kimball_apply_focus_check, row, 2)
        electro.addWidget(self.kimball_apply_deflection_check, row, 3)
        row += 1
        self.kimball_deflection_on_btn = QtWidgets.QPushButton("Deflection switch ON")
        self.kimball_deflection_off_btn = QtWidgets.QPushButton("Deflection switch OFF")
        self.kimball_deflection_on_btn.setStyleSheet("font-weight: bold;")
        electro.addWidget(QtWidgets.QLabel("Manual deflection switch"), row, 0)
        electro.addWidget(self.kimball_deflection_on_btn, row, 1)
        electro.addWidget(self.kimball_deflection_off_btn, row, 2)
        row += 1
        self.kimball_load_lut_btn = QtWidgets.QPushButton("Load LUT target")
        self.kimball_apply_electro_btn = QtWidgets.QPushButton("Apply electrostatic controls")
        self.kimball_apply_electro_btn.setStyleSheet("font-weight: bold;")
        electro.addWidget(self.kimball_load_lut_btn, row, 0, 1, 2)
        electro.addWidget(self.kimball_apply_electro_btn, row, 2, 1, 2)
        row += 1
        self.kimball_lut_status = QtWidgets.QLabel("Load LUT target before applying. Source/ECC is never touched here.")
        self.kimball_lut_status.setWordWrap(True)
        electro.addWidget(self.kimball_lut_status, row, 0, 1, 6)
        left_layout.addWidget(electro_group)

        source_group = QtWidgets.QGroupBox("Source warm-up / Source-mode ramp")
        source = QtWidgets.QGridLayout(source_group)
        row = 0
        self.kimball_source_target_spin = QtWidgets.QDoubleSpinBox()
        self.kimball_source_target_spin.setRange(0, 5)
        self.kimball_source_target_spin.setDecimals(3)
        self.kimball_source_target_spin.setSingleStep(0.05)
        self.kimball_source_target_spin.setValue(0.0)
        self.kimball_source_start_spin = QtWidgets.QDoubleSpinBox()
        self.kimball_source_start_spin.setRange(0, 5)
        self.kimball_source_start_spin.setDecimals(3)
        self.kimball_source_start_spin.setSpecialValueText("assume 0")
        self.kimball_source_start_spin.setValue(0.0)
        self.kimball_source_step_spin = QtWidgets.QDoubleSpinBox()
        self.kimball_source_step_spin.setRange(0.001, 0.10)
        self.kimball_source_step_spin.setDecimals(3)
        self.kimball_source_step_spin.setSingleStep(0.005)
        self.kimball_source_step_spin.setValue(float(self.kimball.config.source_ramp_increment_V))
        self.kimball_source_delay_spin = QtWidgets.QDoubleSpinBox()
        self.kimball_source_delay_spin.setRange(0, 60)
        self.kimball_source_delay_spin.setDecimals(2)
        self.kimball_source_delay_spin.setSingleStep(1)
        self.kimball_source_delay_spin.setValue(float(self.kimball.config.source_ramp_delay_s))
        source.addWidget(QtWidgets.QLabel("Target Source display voltage (V)"), row, 0)
        source.addWidget(self.kimball_source_target_spin, row, 1)
        source.addWidget(QtWidgets.QLabel("Start Source display voltage (V; 0 = assume 0)"), row, 2)
        source.addWidget(self.kimball_source_start_spin, row, 3)
        row += 1
        source.addWidget(QtWidgets.QLabel("Step (V)"), row, 0)
        source.addWidget(self.kimball_source_step_spin, row, 1)
        source.addWidget(QtWidgets.QLabel("Delay per step (s)"), row, 2)
        source.addWidget(self.kimball_source_delay_spin, row, 3)
        row += 1
        self.kimball_source_max_emission_spin = QtWidgets.QDoubleSpinBox()
        self.kimball_source_max_emission_spin.setRange(0, 1000)
        self.kimball_source_max_emission_spin.setDecimals(3)
        self.kimball_source_max_emission_spin.setSpecialValueText("off")
        self.kimball_source_max_emission_spin.setValue(0.0)
        self.kimball_source_max_amps_spin = QtWidgets.QDoubleSpinBox()
        self.kimball_source_max_amps_spin.setRange(0, 10)
        self.kimball_source_max_amps_spin.setDecimals(4)
        self.kimball_source_max_amps_spin.setSpecialValueText("off")
        self.kimball_source_max_amps_spin.setValue(0.0)
        source.addWidget(QtWidgets.QLabel("Abort if Emission > uA (0=off)"), row, 0)
        source.addWidget(self.kimball_source_max_emission_spin, row, 1)
        source.addWidget(QtWidgets.QLabel("Abort if Source A > A (0=off)"), row, 2)
        source.addWidget(self.kimball_source_max_amps_spin, row, 3)
        row += 1
        self.kimball_warmup_before_ramp_check = QtWidgets.QCheckBox("Before Source ramp: set Energy then Grid")
        self.kimball_warmup_before_ramp_check.setChecked(True)
        self.kimball_warmup_energy_spin = QtWidgets.QDoubleSpinBox()
        self.kimball_warmup_energy_spin.setRange(100, 10000)
        self.kimball_warmup_energy_spin.setDecimals(1)
        self.kimball_warmup_energy_spin.setSingleStep(100)
        self.kimball_warmup_energy_spin.setValue(1000.0)
        self.kimball_warmup_grid_spin = QtWidgets.QDoubleSpinBox()
        self.kimball_warmup_grid_spin.setRange(0, 500)
        self.kimball_warmup_grid_spin.setDecimals(2)
        self.kimball_warmup_grid_spin.setSingleStep(10)
        self.kimball_warmup_grid_spin.setValue(500.0)
        source.addWidget(self.kimball_warmup_before_ramp_check, row, 0, 1, 2)
        source.addWidget(QtWidgets.QLabel("Warm-up Energy (eV)"), row, 2)
        source.addWidget(self.kimball_warmup_energy_spin, row, 3)
        source.addWidget(QtWidgets.QLabel("Warm-up Grid (V)"), row, 4)
        source.addWidget(self.kimball_warmup_grid_spin, row, 5)
        row += 1
        self.kimball_apply_warmup_btn = QtWidgets.QPushButton("Apply warm-up Energy/Grid only")
        source.addWidget(self.kimball_apply_warmup_btn, row, 0, 1, 2)
        row += 1

        self.kimball_source_raw_ao_spin = QtWidgets.QDoubleSpinBox()
        self.kimball_source_raw_ao_spin.setRange(0.0, float(self.kimball.config.max_raw_ao_test_V))
        self.kimball_source_raw_ao_spin.setDecimals(4)
        self.kimball_source_raw_ao_spin.setSingleStep(0.01)
        self.kimball_source_raw_ao_spin.setValue(0.05)
        self.kimball_source_raw_ao_btn = QtWidgets.QPushButton("Write Source AO test")
        self.kimball_source_raw_ao_btn.setStyleSheet("color: #8b0000; font-weight: bold;")
        source.addWidget(QtWidgets.QLabel("Direct Source AO test (V, bypass scaling)"), row, 0, 1, 2)
        source.addWidget(self.kimball_source_raw_ao_spin, row, 2)
        source.addWidget(self.kimball_source_raw_ao_btn, row, 3, 1, 2)
        row += 1
        self.kimball_set_source_btn = QtWidgets.QPushButton("Set Source voltage only")
        self.kimball_set_source_btn.setStyleSheet("font-weight: bold;")
        self.kimball_ramp_source_btn = QtWidgets.QPushButton("Regular warm-up: ramp Source")
        self.kimball_ramp_source_btn.setStyleSheet("font-weight: bold; color: #8b0000;")
        self.kimball_post_bake_btn = QtWidgets.QPushButton("Post-bake conditioning warm-up")
        self.kimball_post_bake_btn.setStyleSheet("font-weight: bold; color: #8b0000;")
        self.kimball_ramp_down_source_btn = QtWidgets.QPushButton("Ramp Source down to 0")
        self.kimball_normal_source_shutdown_btn = QtWidgets.QPushButton("Normal Source shutdown")
        self.kimball_zero_source_btn = QtWidgets.QPushButton("Zero Source AO only")
        self.kimball_abort_source_btn = QtWidgets.QPushButton("ABORT + ZERO Source AO")
        self.kimball_abort_source_btn.setStyleSheet("background-color: #b00020; color: white; font-weight: bold;")
        self.kimball_abort_source_btn.setEnabled(False)
        source.addWidget(self.kimball_set_source_btn, row, 0, 1, 2)
        source.addWidget(self.kimball_ramp_source_btn, row, 2, 1, 2)
        source.addWidget(self.kimball_ramp_down_source_btn, row, 4, 1, 2)
        row += 1
        source.addWidget(self.kimball_post_bake_btn, row, 0, 1, 2)
        source.addWidget(self.kimball_normal_source_shutdown_btn, row, 2, 1, 2)
        source.addWidget(self.kimball_zero_source_btn, row, 4, 1, 1)
        source.addWidget(self.kimball_abort_source_btn, row, 5, 1, 1)
        row += 1
        self.kimball_source_status = QtWidgets.QLabel("Set Source voltage only: adjust Source without touching Energy/Grid. Regular warm-up: apply Energy/Grid cutoff, then ramp Source voltage slowly. Post-bake conditioning: ramp until Source current reaches 0.8 A, hold 10 min, then ramp toward 1.7 A. Before real automatic Source ramp, use Direct Source AO test and compare with the Kimball front-panel Source display. Abort writes Source AO=0 immediately. ECC mode is not used.")
        self.kimball_source_status.setWordWrap(True)
        source.addWidget(self.kimball_source_status, row, 0, 1, 6)
        left_layout.addWidget(source_group)

        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        splitter.addWidget(right)
        splitter.setSizes([900, 550])

        self.kimball_device_box = QtWidgets.QPlainTextEdit()
        self.kimball_device_box.setReadOnly(True)
        self.kimball_device_box.setMaximumBlockCount(1000)
        right_layout.addWidget(QtWidgets.QLabel("NI device/channel output and electrostatic apply details"))
        right_layout.addWidget(self.kimball_device_box, stretch=1)

        self.kimball_log = QtWidgets.QPlainTextEdit()
        self.kimball_log.setReadOnly(True)
        self.kimball_log.setMaximumBlockCount(1000)
        right_layout.addWidget(QtWidgets.QLabel("Kimball DAQ log"))
        right_layout.addWidget(self.kimball_log, stretch=1)

        self.kimball_list_devices_btn.clicked.connect(self.kimball_list_devices)
        self.kimball_read_meters_btn.clicked.connect(self.kimball_read_meters)
        self.kimball_auto_meters_check.toggled.connect(self.kimball_auto_meters_toggled)
        self.kimball_zero_btn.clicked.connect(self.kimball_zero_outputs)
        self.kimball_load_lut_btn.clicked.connect(self.kimball_load_lut_target)
        self.kimball_apply_electro_btn.clicked.connect(self.kimball_apply_electrostatics)
        self.kimball_deflection_on_btn.clicked.connect(lambda: self.kimball_set_deflection_switch(True))
        self.kimball_deflection_off_btn.clicked.connect(lambda: self.kimball_set_deflection_switch(False))
        self.kimball_apply_warmup_btn.clicked.connect(self.kimball_apply_source_warmup)
        self.kimball_source_raw_ao_btn.clicked.connect(self.kimball_source_raw_ao_test)
        self.kimball_set_source_btn.clicked.connect(self.kimball_set_source_voltage)
        self.kimball_ramp_source_btn.clicked.connect(self.kimball_ramp_source)
        self.kimball_post_bake_btn.clicked.connect(self.kimball_post_bake_conditioning_warmup)
        self.kimball_ramp_down_source_btn.clicked.connect(self.kimball_ramp_source_down)
        self.kimball_normal_source_shutdown_btn.clicked.connect(self.kimball_normal_source_shutdown)
        self.kimball_zero_source_btn.clicked.connect(self.kimball_zero_source_only)
        self.kimball_abort_source_btn.clicked.connect(self.kimball_abort_source_ramp)

    def _kimball_token(self) -> str:
        if not self.kimball_enable_check.isChecked():
            raise RuntimeError("Check 'Enable gun-DAQ writes' first.")
        return GUN_DAQ_TOKEN

    def kimball_list_devices(self) -> None:
        try:
            lines = []
            for dev in self.kimball.list_devices():
                name = dev.get("name", "")
                lines.append(f"{name}  {dev.get('product_type', '')}  serial={dev.get('serial_num', '')}")
                try:
                    chans = self.kimball.list_channels(str(name))
                    for kind, values in chans.items():
                        if values:
                            lines.append(f"  {kind}: " + ", ".join(values))
                except Exception as exc:
                    lines.append(f"  channel listing error: {exc}")
            self.kimball_device_box.setPlainText("\n".join(lines))
            self.append_log("Kimball/NI devices listed")
        except Exception as exc:
            self.error_box("Kimball NI device listing failed", exc)

    def kimball_read_meters(self) -> None:
        try:
            meters = self.kimball.read_meters()
            for name, rowdata in meters.items():
                row = getattr(self.kimball_meter_table, "name_to_row", {}).get(name)
                if row is None:
                    continue
                self.kimball_meter_table.item(row, 2).setText(fmt_float(rowdata.get("raw_voltage_V"), " V"))
                self.kimball_meter_table.item(row, 3).setText(fmt_float(rowdata.get("value")))
                self.kimball_meter_table.item(row, 4).setText(str(rowdata.get("units", "")))
            self.append_log("Kimball meters read")
        except Exception as exc:
            self.error_box("Kimball meter read failed", exc)

    def kimball_zero_outputs(self) -> None:
        try:
            token = self._kimball_token()
            result = self.kimball.zero_all_outputs(token=token, include_digital=False)
            self.kimball_device_box.setPlainText(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
            self.append_log("Kimball AO outputs zeroed; digital controls skipped in v0.15")
        except Exception as exc:
            self.error_box("Kimball zero outputs failed", exc)


    def kimball_auto_meters_toggled(self, checked: bool) -> None:
        if checked:
            self.kimball_timer.start(1000)
            self.append_log("Kimball auto meter read started")
        else:
            self.kimball_timer.stop()
            self.append_log("Kimball auto meter read stopped")

    def kimball_load_lut_target(self) -> None:
        try:
            energy = float(self.kimball_energy_eV_spin.value())
            lut = self.kimball.lut_for_energy(energy)
            suggested_grid = self.kimball.suggested_grid_V_for_energy(energy)
            self.kimball_grid_V_spin.setValue(suggested_grid)
            self.kimball_x_V_spin.setValue(float(lut["x_center_V"]))
            self.kimball_y_V_spin.setValue(float(lut["y_center_V"]))
            self.kimball_lut_status.setText(
                f"Energy {energy:g} eV: X={lut['x_center_V']:.4g} V, "
                f"Y={lut['y_center_V']:.4g} V, suggested Grid={suggested_grid:.4g} V "
                f"({lut.get('source', 'lut')}). Focus remains manual."
            )
            self.append_log(f"Kimball LUT target loaded for {energy:g} eV")
        except Exception as exc:
            self.error_box("Kimball LUT load failed", exc)

    def kimball_set_deflection_switch(self, enabled: bool) -> None:
        try:
            token = self._kimball_token()
            result = self.kimball.set_deflection_switch(bool(enabled), token=token)
            self.kimball_device_box.setPlainText(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
            self.append_log(f"Kimball deflection switch {'ON' if enabled else 'OFF'}")
        except Exception as exc:
            self.error_box("Kimball deflection switch write failed", exc)

    def kimball_apply_electrostatics(self) -> None:
        try:
            token = self._kimball_token()
            target = self.kimball.make_electrostatic_target_from_lut(
                float(self.kimball_energy_eV_spin.value()),
                grid_V=float(self.kimball_grid_V_spin.value()),
                focus_kV=float(self.kimball_focus_kV_spin.value()),
                apply_grid=self.kimball_apply_grid_check.isChecked(),
                apply_focus=self.kimball_apply_focus_check.isChecked(),
                apply_energy=self.kimball_apply_energy_check.isChecked(),
                apply_deflection=self.kimball_apply_deflection_check.isChecked(),
            )
            # Use the displayed X/Y fields so the user can make small manual corrections
            # after loading the LUT row/interpolation.
            target.x_center_V = float(self.kimball_x_V_spin.value())
            target.y_center_V = float(self.kimball_y_V_spin.value())
            result = self.kimball.apply_electrostatic_target(target, token=token, delay_s=0.2)
            self.kimball_device_box.setPlainText(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
            self.append_log(
                f"Kimball electrostatic controls applied for {target.energy_eV:g} eV; Source/ECC not touched"
            )
            self.kimball_read_meters()
        except Exception as exc:
            self.error_box("Kimball electrostatic apply failed", exc)

    def kimball_apply_source_warmup(self) -> None:
        try:
            token = self._kimball_token()
            energy = float(self.kimball_warmup_energy_spin.value())
            grid = float(self.kimball_warmup_grid_spin.value())
            msg = (
                f"Apply Kimball warm-up electrostatics only\n"
                f"Energy = {energy:g} eV first, then Grid = {grid:g} V.\n\n"
                "Source/ECC will not be ramped by this button. Continue?"
            )
            answer = QtWidgets.QMessageBox.question(
                self,
                "Confirm Kimball warm-up Energy/Grid",
                msg,
                QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
                QtWidgets.QMessageBox.StandardButton.No,
            )
            if answer != QtWidgets.QMessageBox.StandardButton.Yes:
                return
            result = self.kimball.apply_source_warmup_electrostatics(
                energy_eV=energy,
                grid_V=grid,
                token=token,
                delay_s=0.2,
                read_meters=True,
                samples=20,
            )
            self.kimball_device_box.setPlainText(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
            self.kimball_source_status.setText(f"Warm-up electrostatics applied: Energy {energy:g} eV, Grid {grid:g} V.")
            self.append_log(f"Kimball warm-up electrostatics applied: Energy {energy:g} eV, Grid {grid:g} V")
            self.kimball_read_meters()
        except Exception as exc:
            self.error_box("Kimball warm-up apply failed", exc)

    def kimball_source_raw_ao_test(self) -> None:
        try:
            token = self._kimball_token()
            ao_v = float(self.kimball_source_raw_ao_spin.value())
            msg = (
                f"Write Source/ECC Source-mode AO channel directly to {ao_v:g} V.\n\n"
                "This bypasses all engineering scaling and is only for calibration. "
                "Watch the Kimball front-panel Source display, record what it shows, "
                "then press Zero Source AO only. Continue?"
            )
            answer = QtWidgets.QMessageBox.question(
                self,
                "Confirm direct Source AO test",
                msg,
                QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
                QtWidgets.QMessageBox.StandardButton.No,
            )
            if answer != QtWidgets.QMessageBox.StandardButton.Yes:
                return
            result = self.kimball.write_source_raw_ao_test(ao_v, token=token, read_meters=True, samples=20)
            self.kimball_device_box.setPlainText(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
            self.kimball_source_status.setText(f"Direct Source AO test wrote AO1={ao_v:g} V. Compare Kimball Source display manually, then zero Source AO.")
            self.append_log(f"Kimball direct Source AO test wrote AO1={ao_v:g} V")
            self.kimball_read_meters()
        except Exception as exc:
            self.error_box("Kimball Source raw AO test failed", exc)

    def kimball_abort_source_ramp(self) -> None:
        self.kimball_source_abort_requested = True
        self.append_log("Kimball Source ramp abort requested; writing Source AO = 0 now")
        try:
            token = self._kimball_token()
            result = self.kimball.zero_source_only(token=token, read_meters=False)
            self.kimball_last_source_target_V = 0.0
            self.kimball_source_status.setText("ABORT requested: Source AO written to 0. Energy/Grid/Focus/X/Y unchanged.")
            self.kimball_device_box.setPlainText(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
        except Exception as exc:
            self.append_log(f"WARNING: immediate Source zero during abort failed: {exc}")

    def _kimball_source_limits(self) -> tuple[float | None, float | None]:
        max_em = float(self.kimball_source_max_emission_spin.value())
        max_sa = float(self.kimball_source_max_amps_spin.value())
        return (max_em if max_em > 0 else None, max_sa if max_sa > 0 else None)

    def _kimball_set_source_buttons_running(self, running: bool) -> None:
        if hasattr(self, "kimball_set_source_btn"):
            self.kimball_set_source_btn.setEnabled(not running)
        self.kimball_ramp_source_btn.setEnabled(not running)
        if hasattr(self, "kimball_post_bake_btn"):
            self.kimball_post_bake_btn.setEnabled(not running)
        self.kimball_ramp_down_source_btn.setEnabled(not running)
        self.kimball_normal_source_shutdown_btn.setEnabled(not running)
        self.kimball_zero_source_btn.setEnabled(not running)
        if hasattr(self, "kimball_apply_warmup_btn"):
            self.kimball_apply_warmup_btn.setEnabled(not running)
        if hasattr(self, "kimball_source_raw_ao_btn"):
            self.kimball_source_raw_ao_btn.setEnabled(not running)
        self.kimball_abort_source_btn.setEnabled(running)

    def _kimball_source_event_callback(self, step_row):
        meters = step_row.get("meters", {}) or {}
        sv = meters.get("source_volts", {}).get("value")
        em = meters.get("emission", {}).get("value")
        sa = meters.get("source_amps", {}).get("value")
        if step_row.get("hold"):
            text = (
                f"Conditioning hold read {step_row.get('index')}: elapsed {step_row.get('elapsed_s'):.1f} s; "
                f"Source Volts diag={sv}, Emission={em}, Source A={sa}"
            )
        else:
            reason = step_row.get("safety_stop_reason")
            written = step_row.get("written", {}) or {}
            ao = written.get("ao_voltage_V")
            target = step_row.get("target_source_V")
            text = (
                f"Source step {step_row.get('index')}: target display {target:g} V; "
                f"AO={ao} V; Source Volts diag={sv}, Emission={em}, Source A={sa}"
            )
            if reason:
                text += f"; STOP: {reason}"
        self.kimball_source_status.setText(text)
        self.append_log(text)
        QtWidgets.QApplication.processEvents()

    def _kimball_source_abort_callback(self) -> bool:
        QtWidgets.QApplication.processEvents()
        return bool(self.kimball_source_abort_requested)


    def kimball_set_source_voltage(self) -> None:
        try:
            token = self._kimball_token()
            target_source = float(self.kimball_source_target_spin.value())
            step = float(self.kimball_source_step_spin.value())
            delay = float(self.kimball_source_delay_spin.value())
            start_source = None
            if float(self.kimball_source_start_spin.value()) > 0:
                start_source = float(self.kimball_source_start_spin.value())
            max_emission, max_source_amps = self._kimball_source_limits()
            start_text = f"{start_source:g} V" if start_source is not None else "read from Source Volts meter"
            msg = (
                "Set Source voltage only.\n\n"
                f"Target displayed Source = {target_source:g} V\n"
                f"Start Source = {start_text}\n"
                f"Step = {step:g} V, delay = {delay:g} s\n\n"
                "Energy/Grid/Focus/X/Y will not be touched. This is the right button for lowering the Source after conditioning. "
                "Watch the Kimball front panel and use ABORT + ZERO if needed. Continue?"
            )
            answer = QtWidgets.QMessageBox.question(
                self,
                "Confirm Source voltage set",
                msg,
                QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
                QtWidgets.QMessageBox.StandardButton.No,
            )
            if answer != QtWidgets.QMessageBox.StandardButton.Yes:
                return
            self.kimball_source_abort_requested = False
            self._kimball_set_source_buttons_running(True)
            self.kimball_source_status.setText("Setting Source voltage only...")
            result = self.kimball.set_source_voltage(
                target_source,
                token=token,
                start_source_V=start_source,
                step_V=step,
                delay_s=delay,
                read_meters_each_step=True,
                samples=20,
                abort_callback=self._kimball_source_abort_callback,
                event_callback=self._kimball_source_event_callback,
                max_emission_uA=max_emission,
                max_source_amps_A=max_source_amps,
            )
            self.kimball_device_box.setPlainText(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
            ramp = result.get("ramp", {}) or {}
            if ramp.get("aborted"):
                self.kimball_source_status.setText(f"Source set stopped: {ramp.get('safety_stop_reason') or 'aborted'}")
                self.append_log("Kimball Source set stopped")
            else:
                self.kimball_last_source_target_V = target_source
                self.kimball_source_status.setText(f"Source voltage set to {target_source:g} V; electrostatics unchanged.")
                self.append_log(f"Kimball Source voltage set to {target_source:g} V; electrostatics unchanged")
            self.kimball_read_meters()
        except Exception as exc:
            self.error_box("Kimball Source voltage set failed", exc)
        finally:
            self._kimball_set_source_buttons_running(False)

    def kimball_ramp_source(self) -> None:
        try:
            token = self._kimball_token()
            target_source = float(self.kimball_source_target_spin.value())
            step = float(self.kimball_source_step_spin.value())
            delay = float(self.kimball_source_delay_spin.value())
            start_source = None
            if float(self.kimball_source_start_spin.value()) > 0:
                start_source = float(self.kimball_source_start_spin.value())
            max_emission, max_source_amps = self._kimball_source_limits()

            do_warmup = self.kimball_warmup_before_ramp_check.isChecked()
            warmup_energy = float(self.kimball_warmup_energy_spin.value())
            warmup_grid = float(self.kimball_warmup_grid_spin.value())
            warmup_text = (
                f"First set Energy={warmup_energy:g} eV, then Grid={warmup_grid:g} V, then "
                if do_warmup else
                "Warm-up Energy/Grid step is disabled; "
            )
            msg = (
                f"{warmup_text}ramp Source in Source mode to displayed {target_source:g} V\n"
                f"step = {step:g} V, delay = {delay:g} s\n\n"
                "This is the regular warm-up: Energy/Grid cutoff first, then slow Source voltage ramp. "
                "Use Direct Source AO test first if scaling is not yet verified. "
                "Watch the Kimball front panel and use ABORT + ZERO if it rises too fast. ECC mode is not used. Continue?"
            )
            answer = QtWidgets.QMessageBox.question(
                self,
                "Confirm Kimball Source ramp",
                msg,
                QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
                QtWidgets.QMessageBox.StandardButton.No,
            )
            if answer != QtWidgets.QMessageBox.StandardButton.Yes:
                return

            self.kimball_source_abort_requested = False
            self._kimball_set_source_buttons_running(True)
            if do_warmup:
                self.kimball_source_status.setText("Applying warm-up Energy/Grid before Source ramp...")
                warmup_result = self.kimball.apply_source_warmup_electrostatics(
                    energy_eV=warmup_energy,
                    grid_V=warmup_grid,
                    token=token,
                    delay_s=0.2,
                    read_meters=True,
                    samples=20,
                )
                self.kimball_device_box.setPlainText(yaml.safe_dump(warmup_result, sort_keys=False, allow_unicode=True))
                self.append_log(f"Kimball warm-up applied before Source ramp: Energy {warmup_energy:g} eV, Grid {warmup_grid:g} V")
            self.kimball_source_status.setText("Source ramp running...")

            result = self.kimball.ramp_source_voltage(
                target_source,
                token=token,
                start_source_V=start_source,
                step_V=step,
                delay_s=delay,
                read_meters_each_step=True,
                samples=20,
                abort_callback=self._kimball_source_abort_callback,
                event_callback=self._kimball_source_event_callback,
                max_emission_uA=max_emission,
                max_source_amps_A=max_source_amps,
            )
            self.kimball_device_box.setPlainText(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
            if result.get("aborted"):
                self.kimball_source_status.setText(f"Source ramp stopped: {result.get('safety_stop_reason') or 'aborted'}")
                self.append_log("Kimball Source ramp stopped")
            else:
                self.kimball_last_source_target_V = target_source
                self.kimball_source_status.setText(f"Source ramp finished at target {target_source:g} V.")
                self.append_log(f"Kimball Source ramp finished at {target_source:g} V")
            self.kimball_read_meters()
        except Exception as exc:
            self.error_box("Kimball Source ramp failed", exc)
        finally:
            self._kimball_set_source_buttons_running(False)


    def kimball_post_bake_conditioning_warmup(self) -> None:
        try:
            token = self._kimball_token()
            energy = float(self.kimball_warmup_energy_spin.value())
            grid = float(self.kimball_warmup_grid_spin.value())
            step = float(self.kimball_source_step_spin.value())
            delay = float(self.kimball_source_delay_spin.value())
            max_emission, _max_source_amps = self._kimball_source_limits()
            msg = (
                "Post-bake conditioning warm-up:\n"
                f"1. Apply Energy={energy:g} eV and Grid/cutoff={grid:g} V\n"
                "2. Ramp Source until Source current reaches 0.8 A\n"
                "3. Hold 10 min at that condition\n"
                "4. Ramp Source until Source current reaches 1.7 A\n\n"
                f"Source step={step:g} V, delay={delay:g} s. Max commanded Source=3.0 V.\n"
                "This is intended only after baking/conditioning. Watch the Kimball front panel and use ABORT + ZERO if needed. Continue?"
            )
            answer = QtWidgets.QMessageBox.question(
                self,
                "Confirm post-bake conditioning warm-up",
                msg,
                QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
                QtWidgets.QMessageBox.StandardButton.No,
            )
            if answer != QtWidgets.QMessageBox.StandardButton.Yes:
                return
            self.kimball_source_abort_requested = False
            self._kimball_set_source_buttons_running(True)
            self.kimball_source_status.setText("Post-bake conditioning warm-up running...")
            result = self.kimball.post_bake_conditioning_warmup(
                token=token,
                energy_eV=energy,
                grid_V=grid,
                first_current_A=0.8,
                first_hold_s=600.0,
                second_current_A=1.7,
                max_source_V=3.0,
                step_V=step,
                delay_s=delay,
                hold_read_interval_s=10.0,
                samples=20,
                abort_callback=self._kimball_source_abort_callback,
                event_callback=self._kimball_source_event_callback,
                max_emission_uA=max_emission,
            )
            self.kimball_device_box.setPlainText(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
            if result.get("aborted"):
                self.kimball_source_status.setText("Post-bake conditioning stopped/aborted.")
                self.append_log("Kimball post-bake conditioning stopped/aborted")
            else:
                self.kimball_source_status.setText("Post-bake conditioning sequence finished.")
                self.append_log("Kimball post-bake conditioning sequence finished")
            self.kimball_read_meters()
        except Exception as exc:
            self.error_box("Kimball post-bake conditioning failed", exc)
        finally:
            self._kimball_set_source_buttons_running(False)

    def kimball_ramp_source_down(self) -> None:
        try:
            token = self._kimball_token()
            step = float(self.kimball_source_step_spin.value())
            delay = float(self.kimball_source_delay_spin.value())
            start_source = float(self.kimball_source_start_spin.value()) if float(self.kimball_source_start_spin.value()) > 0 else None
            if start_source is None and self.kimball_last_source_target_V > 0:
                start_source = self.kimball_last_source_target_V
            max_emission, max_source_amps = self._kimball_source_limits()

            msg = (
                f"Ramp Source down to 0 V\n"
                f"start = {start_source if start_source is not None else 'assume 0'} V, step = {step:g} V, delay = {delay:g} s\n\n"
                "This leaves Energy/Grid/Focus/X/Y unchanged. Continue?"
            )
            answer = QtWidgets.QMessageBox.question(
                self, "Confirm Source ramp-down", msg,
                QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
                QtWidgets.QMessageBox.StandardButton.No,
            )
            if answer != QtWidgets.QMessageBox.StandardButton.Yes:
                return
            self.kimball_source_abort_requested = False
            self._kimball_set_source_buttons_running(True)
            result = self.kimball.ramp_source_down_to_zero(
                token=token,
                start_source_V=start_source,
                step_V=step,
                delay_s=delay,
                read_meters_each_step=True,
                samples=20,
                abort_callback=self._kimball_source_abort_callback,
                event_callback=self._kimball_source_event_callback,
                max_emission_uA=max_emission,
                max_source_amps_A=max_source_amps,
            )
            self.kimball_device_box.setPlainText(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
            if not result.get("aborted"):
                self.kimball_last_source_target_V = 0.0
            self.kimball_source_status.setText("Source ramp-down finished." if not result.get("aborted") else "Source ramp-down stopped.")
            self.kimball_read_meters()
        except Exception as exc:
            self.error_box("Kimball Source ramp-down failed", exc)
        finally:
            self._kimball_set_source_buttons_running(False)

    def kimball_zero_source_only(self) -> None:
        try:
            token = self._kimball_token()
            result = self.kimball.zero_source_only(token=token, samples=20)
            self.kimball_device_box.setPlainText(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
            self.kimball_last_source_target_V = 0.0
            self.kimball_source_status.setText("Source AO set to 0 V; electrostatics unchanged.")
            self.append_log("Kimball Source AO only zeroed")
            self.kimball_read_meters()
        except Exception as exc:
            self.error_box("Kimball zero Source only failed", exc)

    def kimball_normal_source_shutdown(self) -> None:
        try:
            token = self._kimball_token()
            step = float(self.kimball_source_step_spin.value())
            delay = float(self.kimball_source_delay_spin.value())
            start_source = float(self.kimball_source_start_spin.value()) if float(self.kimball_source_start_spin.value()) > 0 else None
            if start_source is None and self.kimball_last_source_target_V > 0:
                start_source = self.kimball_last_source_target_V

            msg = (
                "Normal Source shutdown:\n"
                "1. Ramp Source down gradually to 0 V\n"
                "2. Write Source AO = 0 V\n\n"
                "Energy/Grid/Focus/X/Y will be left unchanged. Continue?"
            )
            answer = QtWidgets.QMessageBox.question(
                self, "Confirm normal Source shutdown", msg,
                QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
                QtWidgets.QMessageBox.StandardButton.No,
            )
            if answer != QtWidgets.QMessageBox.StandardButton.Yes:
                return
            self.kimball_source_abort_requested = False
            self._kimball_set_source_buttons_running(True)
            result = self.kimball.normal_source_shutdown(
                token=token,
                start_source_V=start_source,
                step_V=step,
                delay_s=delay,
                read_meters_each_step=True,
                samples=20,
                abort_callback=self._kimball_source_abort_callback,
                event_callback=self._kimball_source_event_callback,
                final_settle_s=10.0,
                final_read_interval_s=2.0,
            )
            self.kimball_device_box.setPlainText(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
            if not result.get("ramp_down", {}).get("aborted"):
                self.kimball_last_source_target_V = 0.0
                self.kimball_source_status.setText("Normal Source shutdown finished; electrostatics unchanged.")
                self.append_log("Kimball normal Source shutdown finished")
            else:
                self.kimball_source_status.setText("Normal Source shutdown stopped before zeroing Source.")
                self.append_log("Kimball normal Source shutdown stopped")
            self.kimball_read_meters()
        except Exception as exc:
            self.error_box("Kimball normal Source shutdown failed", exc)
        finally:
            self._kimball_set_source_buttons_running(False)


    # ------------------------------------------------------------------
    # MDrive bench-test motion tab
    # ------------------------------------------------------------------
    def _build_motion_tab(self) -> None:
        layout = QtWidgets.QVBoxLayout(self.motion_tab)
        note = QtWidgets.QLabel(
            "v0.19.3 MDrive bench/control: COM18 = lift/lid motor, COM23 = temporary rotation motor. "
            "Rotation angle moves use config/angles-steps-LUT.csv and absolute C1 microstep positions."
        )
        note.setWordWrap(True)
        layout.addWidget(note)

        top = QtWidgets.QHBoxLayout()
        layout.addLayout(top)
        self.motion_axis_combo = QtWidgets.QComboBox()
        self.motion_axis_combo.addItems(["lift", "rotation"])
        self.motion_enable_check = QtWidgets.QCheckBox("Enable motion-changing buttons")
        self.motion_status_btn = QtWidgets.QPushButton("Read Status")
        self.motion_init_btn = QtWidgets.QPushButton("Initialize/read axis")
        self.motion_inputs_btn = QtWidgets.QPushButton("Read Inputs I1-I4")
        self.motion_stop_btn = QtWidgets.QPushButton("STOP selected")
        self.motion_stop_btn.setStyleSheet("background-color: #b00020; color: white; font-weight: bold; padding: 4px;")
        self.motion_stop_all_btn = QtWidgets.QPushButton("STOP BOTH")
        self.motion_stop_all_btn.setStyleSheet("background-color: #b00020; color: white; font-weight: bold; padding: 4px;")
        self.motion_position_indicator = QtWidgets.QLabel("Last position: —")
        self.motion_position_indicator.setMinimumWidth(360)
        self.motion_position_indicator.setStyleSheet("font-weight: bold; padding: 4px; border: 1px solid #999;")
        self.motion_input_leds: dict[str, QtWidgets.QLabel] = {}
        inputs_box = QtWidgets.QWidget()
        inputs_layout = QtWidgets.QHBoxLayout(inputs_box)
        inputs_layout.setContentsMargins(0, 0, 0, 0)
        inputs_layout.addWidget(QtWidgets.QLabel("Limits/inputs:"))
        for key in ("I1", "I2", "I3", "I4"):
            led = QtWidgets.QLabel(f"{key} ?")
            led.setMinimumWidth(46)
            led.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            led.setStyleSheet("border-radius: 8px; padding: 3px; border: 1px solid #777; background: #d0d0d0; color: black;")
            self.motion_input_leds[key] = led
            inputs_layout.addWidget(led)
        top.addWidget(QtWidgets.QLabel("Axis:"))
        top.addWidget(self.motion_axis_combo)
        top.addWidget(self.motion_enable_check)
        top.addWidget(self.motion_status_btn)
        top.addWidget(self.motion_init_btn)
        top.addWidget(self.motion_inputs_btn)
        top.addWidget(self.motion_stop_btn)
        top.addWidget(self.motion_stop_all_btn)
        top.addWidget(self.motion_position_indicator)
        top.addWidget(inputs_box)
        top.addStretch()

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        layout.addWidget(splitter, stretch=1)

        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        splitter.addWidget(left)

        self.motion_table = QtWidgets.QTableWidget(2, 10)
        self.motion_table.setHorizontalHeaderLabels(["Axis", "Port", "Role", "Encoder", "C1", "Cal. angle", "Encoder C2", "VM", "VI", "State"])
        self.motion_table.horizontalHeader().setStretchLastSection(True)
        self.motion_table.setAlternatingRowColors(True)
        self.motion_table.axis_to_row = {}  # type: ignore[attr-defined]
        for row, axis in enumerate(["lift", "rotation"]):
            cfg = (self.mdrive_config.get("axes", {}) or {}).get(axis, {}) or {}
            self.motion_table.axis_to_row[axis] = row  # type: ignore[attr-defined]
            values = [axis, str(cfg.get("port", "")), str(cfg.get("role", "")), str(bool(cfg.get("has_encoder", False))), "", "", "", "", "", ""]
            for col, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                item.setFlags(item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
                self.motion_table.setItem(row, col, item)
        self.motion_table.resizeColumnsToContents()
        left_layout.addWidget(QtWidgets.QLabel("Configured MDrive axes"))
        left_layout.addWidget(self.motion_table, stretch=1)

        move_group = QtWidgets.QGroupBox("Manual relative moves")
        move_layout = QtWidgets.QGridLayout(move_group)
        self.motion_steps_spin = QtWidgets.QSpinBox()
        self.motion_steps_spin.setRange(-10000, 10000)
        self.motion_steps_spin.setValue(100)
        self.motion_move_relative_btn = QtWidgets.QPushButton("Move relative")
        self.motion_abs_spin = QtWidgets.QSpinBox()
        self.motion_abs_spin.setRange(-20000, 20000)
        self.motion_abs_spin.setValue(0)
        self.motion_abs_btn = QtWidgets.QPushButton("Move absolute C1")
        self.motion_zero_btn = QtWidgets.QPushButton("Zero C1/C2")
        move_layout.addWidget(QtWidgets.QLabel("Relative move, signed (lift µm / rotation µsteps):"), 0, 0)
        move_layout.addWidget(self.motion_steps_spin, 0, 1)
        move_layout.addWidget(self.motion_move_relative_btn, 0, 2)
        move_layout.addWidget(QtWidgets.QLabel("Absolute target (lift µm / rotation C1 µsteps):"), 1, 0)
        move_layout.addWidget(self.motion_abs_spin, 1, 1)
        move_layout.addWidget(self.motion_abs_btn, 1, 2)
        move_layout.addWidget(self.motion_zero_btn, 1, 3)
        left_layout.addWidget(move_group)

        angle_group = QtWidgets.QGroupBox("Calibrated rotation / lift presets")
        angle_layout = QtWidgets.QGridLayout(angle_group)
        self.motion_angle_spin = QtWidgets.QDoubleSpinBox()
        self.motion_angle_spin.setRange(-95.0, 25.0)
        self.motion_angle_spin.setDecimals(3)
        self.motion_angle_spin.setSingleStep(1.0)
        self.motion_angle_spin.setValue(0.0)
        self.motion_angle_btn = QtWidgets.QPushButton("Move rotation to angle")
        self.motion_home_repeatable_btn = QtWidgets.QPushButton("Repeatable home +2000 then HI")
        self.motion_lift_open_btn = QtWidgets.QPushButton("Lift open (-12000 µm)")
        self.motion_lift_close_btn = QtWidgets.QPushButton("Lift close (0 µm)")
        angle_layout.addWidget(QtWidgets.QLabel("Angle deg from LUT:"), 0, 0)
        angle_layout.addWidget(self.motion_angle_spin, 0, 1)
        angle_layout.addWidget(self.motion_angle_btn, 0, 2)
        angle_layout.addWidget(self.motion_home_repeatable_btn, 1, 0, 1, 3)
        angle_layout.addWidget(self.motion_lift_open_btn, 2, 0, 1, 2)
        angle_layout.addWidget(self.motion_lift_close_btn, 2, 2)
        left_layout.addWidget(angle_group)

        home_group = QtWidgets.QGroupBox("Home diagnostics")
        home_layout = QtWidgets.QGridLayout(home_group)
        self.motion_home_type_spin = QtWidgets.QSpinBox()
        self.motion_home_type_spin.setRange(1, 4)
        self.motion_home_type_spin.setValue(2)
        self.motion_home_btn = QtWidgets.QPushButton("Home rotation HI/index")
        home_layout.addWidget(QtWidgets.QLabel("HI type:"), 0, 0)
        home_layout.addWidget(self.motion_home_type_spin, 0, 1)
        home_layout.addWidget(self.motion_home_btn, 0, 2)
        left_layout.addWidget(home_group)
        left_layout.addStretch()

        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        splitter.addWidget(right)
        splitter.setSizes([850, 650])
        self.motion_log = QtWidgets.QPlainTextEdit()
        self.motion_log.setReadOnly(True)
        self.motion_log.setMaximumBlockCount(1000)
        right_layout.addWidget(QtWidgets.QLabel("MDrive motion log"))
        right_layout.addWidget(self.motion_log, stretch=1)

        self.motion_status_btn.clicked.connect(self.motion_read_status)
        self.motion_init_btn.clicked.connect(self.motion_initialize_axis)
        self.motion_inputs_btn.clicked.connect(self.motion_read_inputs)
        self.motion_stop_btn.clicked.connect(self.motion_stop)
        self.motion_stop_all_btn.clicked.connect(self.motion_stop_all)
        self.motion_move_relative_btn.clicked.connect(lambda: self.motion_move_relative(int(self.motion_steps_spin.value())))
        self.motion_abs_btn.clicked.connect(self.motion_move_absolute)
        self.motion_zero_btn.clicked.connect(self.motion_zero_position)
        self.motion_home_btn.clicked.connect(self.motion_home_index)
        self.motion_angle_btn.clicked.connect(self.motion_move_angle_lut)
        self.motion_home_repeatable_btn.clicked.connect(self.motion_home_repeatable)
        self.motion_lift_open_btn.clicked.connect(lambda: self.motion_lift_preset("open"))
        self.motion_lift_close_btn.clicked.connect(lambda: self.motion_lift_preset("close"))

    def _motion_axis(self):
        axis = self.motion_axis_combo.currentText()
        dev, cfg = make_mdrive_axis(self.mdrive_config, axis, simulate_override=self.simulate)
        return axis, dev, cfg

    def _motion_angle_lut(self, cfg: dict[str, Any]) -> AngleStepLUT:
        lut_path = cfg.get("angle_lut_csv") or "angles-steps-LUT.csv"
        p = Path(str(lut_path))
        if not p.is_absolute():
            p = Path(__file__).resolve().parents[1] / "config" / p
        return AngleStepLUT.from_csv(p)

    def _motion_check_enabled(self) -> bool:
        if not motion_requires_token(self.mdrive_config):
            return True
        if self.motion_enable_check.isChecked():
            return True
        self.error_box("Motion not enabled", f"Check 'Enable motion-changing buttons' first. Token: {MOTION_TOKEN}")
        return False

    def _motion_update_table(self, axis: str, status: dict[str, Any]) -> None:
        row = getattr(self.motion_table, "axis_to_row", {}).get(axis)
        if row is None:
            return
        cfg = (self.mdrive_config.get("axes", {}) or {}).get(axis, {}) or {}
        c1_val = status.get("counter1_steps", status.get("position_steps", ""))
        angle_txt = ""
        if axis == "rotation" and c1_val not in (None, ""):
            try:
                angle_txt = f"{self._motion_angle_lut(cfg).angle_for_steps(int(c1_val)):.3f}"
            except Exception:
                angle_txt = "out of LUT"
        values = [
            axis,
            str(cfg.get("port", "")),
            str(cfg.get("role", "")),
            str(bool(cfg.get("has_encoder", False))),
            str(c1_val),
            angle_txt,
            str(status.get("encoder_steps", "")),
            str(status.get("VM", "")),
            str(status.get("VI", "")),
            str("ready"),
        ]
        for col, value in enumerate(values):
            self.motion_table.item(row, col).setText(value)

    def _motion_update_indicator(self, axis: str, status: dict[str, Any]) -> None:
        c1_val = status.get("counter1_steps", status.get("position_steps", None))
        c2_val = status.get("encoder_steps", None)
        angle_txt = ""
        if axis == "rotation" and c1_val not in (None, ""):
            cfg = (self.mdrive_config.get("axes", {}) or {}).get(axis, {}) or {}
            try:
                angle_txt = f", angle≈{self._motion_angle_lut(cfg).angle_for_steps(int(c1_val)):.3f}°"
            except Exception:
                angle_txt = ", angle: out of LUT"
        self.motion_position_indicator.setText(f"Last {axis}: C1={c1_val}{angle_txt}, C2={c2_val}")
        self.motion_position_indicator.setStyleSheet("font-weight: bold; padding: 4px; border: 1px solid #999;")

    def _motion_update_input_leds(self, inputs: dict[str, Any]) -> None:
        for key, led in getattr(self, "motion_input_leds", {}).items():
            raw = str(inputs.get(key, "?")).strip()
            try:
                val = int(raw)
            except Exception:
                val = None
            if val == 1:
                led.setText(f"{key} ON")
                led.setStyleSheet("border-radius: 8px; padding: 3px; border: 1px solid #555; background: #ffcc00; color: black; font-weight: bold;")
            elif val == 0:
                led.setText(f"{key} off")
                led.setStyleSheet("border-radius: 8px; padding: 3px; border: 1px solid #555; background: #e6e6e6; color: black;")
            else:
                led.setText(f"{key} ?")
                led.setStyleSheet("border-radius: 8px; padding: 3px; border: 1px solid #777; background: #d0d0d0; color: black;")

    def motion_initialize_axis(self) -> None:
        if not self._motion_check_enabled():
            return
        axis, dev, cfg = self._motion_axis()
        enc = cfg.get("initialize_encoder_enable", None)
        c1 = cfg.get("initialize_c1", None)
        ppos = cfg.get("initialize_p", None)
        echo = cfg.get("echo_mode", None)
        msg = f"Initialize/read {axis} on {cfg.get('port')}: "
        parts = []
        if echo is not None: parts.append(f"EM {echo}")
        if enc is not None: parts.append(f"EE {1 if enc else 0}")
        if c1 is not None: parts.append(f"C1 {c1}")
        c2 = cfg.get("initialize_c2", None)
        if c2 is not None: parts.append(f"C2 {c2}")
        if ppos is not None: parts.append(f"P {ppos}")
        if parts:
            msg += ", ".join(parts)
            resp = QtWidgets.QMessageBox.question(self, "Confirm MDrive initialize", msg + "?")
            if resp != QtWidgets.QMessageBox.StandardButton.Yes:
                return
        else:
            msg += "read-only; no startup commands will be sent"
        try:
            dev.open()
            status = dev.initialize_axis(encoder_enable=enc, counter1=c1, counter2=c2, position_p=ppos, echo_mode=echo)
            self._motion_update_table(axis, status)
            self._motion_update_indicator(axis, status)
            self.append_log(f"MDrive {axis} initialize/read: {msg}; EE={status.get('encoder_enabled')}, C1={status.get('counter1_steps')}")
        except Exception as exc:
            self.error_box("MDrive initialize failed", exc)
        finally:
            try: dev.close()
            except Exception: pass

    def motion_read_status(self) -> None:
        axis, dev, cfg = self._motion_axis()
        try:
            dev.open()
            status = dev.status()
            self._motion_update_table(axis, status)
            self._motion_update_indicator(axis, status)
            self.append_log(f"MDrive {axis} status read: C1={status.get('counter1_steps', status.get('position_steps'))}")
        except Exception as exc:
            self.error_box("MDrive status failed", exc)
        finally:
            try: dev.close()
            except Exception: pass

    def motion_read_inputs(self) -> None:
        axis, dev, cfg = self._motion_axis()
        try:
            dev.open()
            inputs = dev.read_inputs()
            self._motion_update_input_leds(inputs)
            self.append_log(f"MDrive {axis} inputs: {inputs}")
        except Exception as exc:
            self.error_box("MDrive input read failed", exc)
        finally:
            try: dev.close()
            except Exception: pass

    def motion_move_relative(self, steps: int) -> None:
        if not self._motion_check_enabled():
            return
        axis, dev, cfg = self._motion_axis()
        motor_steps, info = motion_manual_relative_steps(axis, int(steps), cfg)
        limit = motion_limit_steps(self.mdrive_config, cfg)
        if abs(int(motor_steps)) > limit:
            self.error_box("Move too large", f"Requested {steps} {info['units']} -> {motor_steps} microsteps; limit is {limit} microsteps")
            return
        try:
            dev.open()
            dev.move_relative(int(motor_steps))
            status = dev.status()
            self._motion_update_table(axis, status)
            self._motion_update_indicator(axis, status)
            self.append_log(f"MDrive {axis} moved relative entered={steps} {info['units']} scale={info['scale']} -> {motor_steps} microsteps")
        except Exception as exc:
            self.error_box("MDrive relative move failed", exc)
        finally:
            try: dev.close()
            except Exception: pass

    def motion_move_absolute(self) -> None:
        if not self._motion_check_enabled():
            return
        axis, dev, cfg = self._motion_axis()
        entered = int(self.motion_abs_spin.value())
        target_steps, info = motion_user_units_to_microsteps(axis, entered, cfg)
        limit = motion_limit_steps(self.mdrive_config, cfg)
        if abs(target_steps) > limit:
            resp = QtWidgets.QMessageBox.question(self, "Large absolute target", f"Absolute target {entered} {info['units']} -> {target_steps} microsteps is larger than the single-move limit {limit}. Continue?")
            if resp != QtWidgets.QMessageBox.StandardButton.Yes:
                return
        try:
            dev.open()
            dev.move_absolute(target_steps)
            status = dev.status()
            self._motion_update_table(axis, status)
            self._motion_update_indicator(axis, status)
            self.append_log(f"MDrive {axis} moved absolute entered={entered} {info['units']} scale={info['scale']} -> C1 {target_steps} microsteps")
        except Exception as exc:
            self.error_box("MDrive absolute move failed", exc)
        finally:
            try: dev.close()
            except Exception: pass

    def motion_zero_position(self) -> None:
        if not self._motion_check_enabled():
            return
        axis, dev, cfg = self._motion_axis()
        try:
            dev.open()
            dev.zero_position()
            status = dev.status()
            self._motion_update_table(axis, status)
            self._motion_update_indicator(axis, status)
            self.append_log(f"MDrive {axis} counters set to C1 0 and C2 0")
        except Exception as exc:
            self.error_box("MDrive zero position failed", exc)
        finally:
            try: dev.close()
            except Exception: pass

    def motion_move_angle_lut(self) -> None:
        if not self._motion_check_enabled():
            return
        axis, dev, cfg = self._motion_axis()
        if axis != "rotation":
            self.error_box("Wrong axis", "Select rotation for calibrated angle moves.")
            return
        angle = float(self.motion_angle_spin.value())
        try:
            lut = self._motion_angle_lut(cfg)
            target = lut.steps_for_angle(angle)
        except Exception as exc:
            self.error_box("Angle LUT error", exc)
            return
        resp = QtWidgets.QMessageBox.question(self, "Confirm rotation angle move", f"Move rotation to {angle:g}° using LUT target C1={target} microsteps?")
        if resp != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        try:
            dev.open()
            dev.move_absolute(int(target), timeout_s=float(cfg.get("wait_timeout_s", 60.0)), poll_s=float(cfg.get("wait_poll_s", 0.1)))
            status = dev.status()
            self._motion_update_table(axis, status)
            self._motion_update_indicator(axis, status)
            self.append_log(f"MDrive rotation moved to calibrated angle {angle:g} deg -> C1 {target}")
        except Exception as exc:
            self.error_box("MDrive angle move failed", exc)
        finally:
            try: dev.close()
            except Exception: pass

    def motion_home_repeatable(self) -> None:
        if not self._motion_check_enabled():
            return
        axis, dev, cfg = self._motion_axis()
        if axis != "rotation":
            self.error_box("Wrong axis", "Select rotation for repeatable HI home.")
            return
        home_type = int(self.motion_home_type_spin.value())
        overshoot = int(cfg.get("home_overshoot_steps", 2000))
        resp = QtWidgets.QMessageBox.question(self, "Confirm repeatable rotation home", f"Move absolute to +{overshoot} microsteps, then send HI {home_type}, then zero C1/C2?")
        if resp != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        try:
            dev.open()
            dev.home_rotation_repeatable(overshoot_steps=overshoot, home_type=home_type, timeout_s=max(60.0, float(cfg.get("wait_timeout_s", 60.0))), poll_s=float(cfg.get("wait_poll_s", 0.1)))
            status = dev.status()
            self._motion_update_table(axis, status)
            self._motion_update_indicator(axis, status)
            self.append_log(f"MDrive rotation repeatable home complete: +{overshoot}, HI {home_type}, C1/C2=0")
        except Exception as exc:
            self.error_box("MDrive repeatable home failed", exc)
        finally:
            try: dev.close()
            except Exception: pass

    def motion_lift_preset(self, which: str) -> None:
        if not self._motion_check_enabled():
            return
        self.motion_axis_combo.setCurrentText("lift")
        axis, dev, cfg = self._motion_axis()
        target, info = motion_lift_target_microsteps(cfg, which)
        resp = QtWidgets.QMessageBox.question(self, "Confirm lift move", f"Move lift/lid to {which} position {info['entered']} {info['units']} -> C1={target} microsteps? Confirm the path is clear and limit/input LEDs look safe.")
        if resp != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        try:
            dev.open()
            dev.move_absolute(target, timeout_s=float(cfg.get("wait_timeout_s", 180.0)), poll_s=float(cfg.get("wait_poll_s", 0.1)))
            status = dev.status()
            self._motion_update_table(axis, status)
            self._motion_update_indicator(axis, status)
            self.append_log(f"MDrive lift moved to {which}: {info['entered']} {info['units']} scale={info['scale']} -> C1 {target} microsteps")
        except Exception as exc:
            self.error_box("MDrive lift preset failed", exc)
        finally:
            try: dev.close()
            except Exception: pass

    def motion_start_slew(self) -> None:
        if not self._motion_check_enabled():
            return
        axis, dev, cfg = self._motion_axis()
        speed = int(self.motion_slew_spin.value())
        limit = motion_limit_slew(self.mdrive_config, cfg)
        if abs(speed) > limit:
            self.error_box("Slew too fast", f"Requested {speed} steps/s; limit is {limit}")
            return
        try:
            dev.open()
            dev.slew(speed)
            self.append_log(f"MDrive {axis} slew set to {speed} steps/s. Use STOP / SL 0 to stop.")
        except Exception as exc:
            self.error_box("MDrive slew failed", exc)
        finally:
            try: dev.close()
            except Exception: pass

    def motion_stop(self) -> None:
        # Emergency stop is intentionally not gated by the motion-enable checkbox.
        axis, dev, cfg = self._motion_axis()
        try:
            dev.open()
            dev.stop()
            try:
                status = dev.status()
                self._motion_update_table(axis, status)
                self._motion_update_indicator(axis, status)
            except Exception:
                pass
            self.append_log(f"MDrive {axis} STOP / SL 0 sent")
        except Exception as exc:
            self.error_box("MDrive stop failed", exc)
        finally:
            try: dev.close()
            except Exception: pass

    def motion_stop_all(self) -> None:
        # Emergency stop for both configured bench axes; also not gated.
        errors: list[str] = []
        for axis_name in ("lift", "rotation"):
            try:
                dev, cfg = make_mdrive_axis(self.mdrive_config, axis_name, simulate_override=self.simulate)
                dev.open()
                dev.stop()
                try:
                    status = dev.status()
                    self._motion_update_table(axis_name, status)
                except Exception:
                    pass
                self.append_log(f"MDrive {axis_name} STOP / SL 0 sent")
            except Exception as exc:
                errors.append(f"{axis_name}: {exc}")
            finally:
                try: dev.close()  # type: ignore[name-defined]
                except Exception: pass
        if errors:
            self.error_box("MDrive stop-all warning", "; ".join(errors))

    def motion_home_index(self) -> None:
        if not self._motion_check_enabled():
            return
        axis, dev, cfg = self._motion_axis()
        if axis != "rotation" or not bool(cfg.get("has_encoder", False)) or not bool(cfg.get("allow_home", False)):
            self.error_box("Home not allowed", "HM homing is enabled only for the rotation axis in this bench-test config.")
            return
        home_type = int(self.motion_home_type_spin.value())
        resp = QtWidgets.QMessageBox.question(self, "Confirm rotation home", f"Send HI {home_type} to home to encoder index/fiducial, then set C1 after home, to the rotation MDrive on {cfg.get('port')}? Use Initialize axis first if needed; HI does not change EE automatically.")
        if resp != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        try:
            dev.open()
            if bool(cfg.get("enable_encoder_before_home", False)):
                dev.enable_encoder(True)
                self.append_log("MDrive rotation encoder enabled: EE 1")
            dev.home_to_index(home_type)
            if "set_c1_after_home" in cfg and cfg.get("set_c1_after_home") is not None:
                dev.set_counter1(int(cfg.get("set_c1_after_home", 0)))
            status = dev.status()
            self._motion_update_table(axis, status)
            self._motion_update_indicator(axis, status)
            self.append_log(f"MDrive rotation HI {home_type} index home finished; C1 set after home")
        except Exception as exc:
            self.error_box("MDrive HI/index home failed", exc)
        finally:
            try: dev.close()
            except Exception: pass

    # ------------------------------------------------------------------
    # Bias-scan measurement tab
    # ------------------------------------------------------------------
    def _build_measure_tab(self) -> None:
        layout = QtWidgets.QVBoxLayout(self.measure_tab)

        top = QtWidgets.QHBoxLayout()
        layout.addLayout(top)
        self.measure_start_btn = QtWidgets.QPushButton("Start bias scan")
        self.measure_stop_btn = QtWidgets.QPushButton("Stop after current point")
        self.measure_stop_btn.setEnabled(False)
        self.measure_hv_check = QtWidgets.QCheckBox("Enable measurement HV changes")
        self.measure_hv_check.setStyleSheet("font-weight: bold; color: #8b0000;")
        top.addWidget(self.measure_start_btn)
        top.addWidget(self.measure_stop_btn)
        top.addWidget(self.measure_hv_check)
        top.addStretch()

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        layout.addWidget(splitter, stretch=1)

        left = QtWidgets.QWidget()
        form = QtWidgets.QGridLayout(left)
        splitter.addWidget(left)

        self.measure_mode_combo = QtWidgets.QComboBox()
        for key, cfg in MEASUREMENT_MODES.items():
            self.measure_mode_combo.addItem(cfg["label"], key)
        self.measure_profile_combo = QtWidgets.QComboBox()
        profiles = self.rbd_profiles.get("profiles", {}) or {}
        default_profile = self.rbd_profiles.get("default_profile")
        default_index = 0
        for idx, (key, cfg) in enumerate(profiles.items()):
            implemented = cfg.get("implemented", True)
            label = key if implemented else f"{key} (future)"
            self.measure_profile_combo.addItem(label, key)
            if key == default_profile:
                default_index = idx
        if self.measure_profile_combo.count() == 0:
            self.measure_profile_combo.addItem("yield_standard", "yield_standard")
        self.measure_profile_combo.setCurrentIndex(default_index)
        self.measure_energy_spin = QtWidgets.QDoubleSpinBox()
        self.measure_energy_spin.setRange(0.0, 100.0)
        self.measure_energy_spin.setDecimals(4)
        self.measure_energy_spin.setValue(0.0)
        self.measure_energy_spin.setSuffix(" keV")
        self.measure_v_start = QtWidgets.QDoubleSpinBox()
        self.measure_v_end = QtWidgets.QDoubleSpinBox()
        self.measure_v_step = QtWidgets.QDoubleSpinBox()
        for w in (self.measure_v_start, self.measure_v_end, self.measure_v_step):
            w.setRange(-100000.0, 100000.0)
            w.setDecimals(4)
            w.setSuffix(" V")
        self.measure_v_start.setValue(0.0)
        self.measure_v_end.setValue(-5.0)
        self.measure_v_step.setValue(-1.0)
        self.measure_n_samples = QtWidgets.QSpinBox()
        self.measure_n_samples.setRange(1, 10000)
        self.measure_n_samples.setValue(5)
        self.measure_n_repeats = QtWidgets.QSpinBox()
        self.measure_n_repeats.setRange(1, 10000)
        self.measure_n_repeats.setValue(1)
        self.measure_sample_period = QtWidgets.QDoubleSpinBox()
        self.measure_sample_period.setRange(0.0, 60.0)
        self.measure_sample_period.setValue(0.2)
        self.measure_sample_period.setSuffix(" s")
        self.measure_post_settle = QtWidgets.QDoubleSpinBox()
        self.measure_post_settle.setRange(0.0, 300.0)
        self.measure_post_settle.setValue(2.0)
        self.measure_post_settle.setSuffix(" s")
        self.measure_initial_settle = QtWidgets.QDoubleSpinBox()
        self.measure_initial_settle.setRange(0.0, 600.0)
        self.measure_initial_settle.setValue(5.0)
        self.measure_initial_settle.setSuffix(" s")
        self.measure_collector_voltage = QtWidgets.QDoubleSpinBox()
        self.measure_collector_voltage.setRange(-100.0, 100.0)
        self.measure_collector_voltage.setDecimals(6)
        self.measure_collector_voltage.setValue(0.0)
        self.measure_collector_voltage.setSuffix(" V")
        self.measure_collector_range = QtWidgets.QComboBox()
        self.measure_collector_range.addItems(["RANGE1", "RANGE10", "RANGE100"])
        self.measure_srs_on_check = QtWidgets.QCheckBox("Ensure SRS collector output ON")
        self.measure_srs_on_check.setChecked(True)
        self.measure_srs_on_check.setToolTip("Keeps the collector actively held at the requested voltage, even when that voltage is 0 V.")
        self.measure_tol = QtWidgets.QDoubleSpinBox()
        self.measure_tol.setRange(0.001, 1000.0)
        self.measure_tol.setValue(0.2)
        self.measure_tol.setSuffix(" V")
        self.measure_timeout = QtWidgets.QDoubleSpinBox()
        self.measure_timeout.setRange(0.5, 600.0)
        self.measure_timeout.setValue(15.0)
        self.measure_timeout.setSuffix(" s")
        self.measure_current = QtWidgets.QDoubleSpinBox()
        self.measure_current.setRange(0.000001, 1000.0)
        self.measure_current.setDecimals(6)
        self.measure_current.setValue(0.1)
        self.measure_current.setSuffix(" mA")
        self.measure_ramp_mode = QtWidgets.QComboBox()
        for m in ["no change", "0", "1", "2", "4"]:
            self.measure_ramp_mode.addItem(m)
        self.measure_zero_after = QtWidgets.QCheckBox("Zero scanned supply after scan")
        self.measure_zero_after.setChecked(True)
        self.measure_output_off_after_zero = QtWidgets.QCheckBox("Output OFF after final zero")
        self.measure_output_off_after_zero.setChecked(False)
        self.measure_file_edit = QtWidgets.QLineEdit()
        self.measure_file_edit.setText(str(self.output_dir / f"bias_scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"))
        self.measure_file_btn = QtWidgets.QPushButton("Browse")

        row = 0
        form.addWidget(QtWidgets.QLabel("Mode"), row, 0); form.addWidget(self.measure_mode_combo, row, 1, 1, 3); row += 1
        self.measure_mode_description = QtWidgets.QLabel("")
        self.measure_mode_description.setWordWrap(True)
        form.addWidget(self.measure_mode_description, row, 0, 1, 4); row += 1
        form.addWidget(QtWidgets.QLabel("RBD profile"), row, 0); form.addWidget(self.measure_profile_combo, row, 1, 1, 3); row += 1
        self.measure_profile_description = QtWidgets.QLabel("")
        self.measure_profile_description.setWordWrap(True)
        form.addWidget(self.measure_profile_description, row, 0, 1, 4); row += 1
        form.addWidget(QtWidgets.QLabel("Energy label"), row, 0); form.addWidget(self.measure_energy_spin, row, 1); row += 1
        form.addWidget(QtWidgets.QLabel("Bias start"), row, 0); form.addWidget(self.measure_v_start, row, 1)
        form.addWidget(QtWidgets.QLabel("Bias end"), row, 2); form.addWidget(self.measure_v_end, row, 3); row += 1
        form.addWidget(QtWidgets.QLabel("Bias step"), row, 0); form.addWidget(self.measure_v_step, row, 1)
        form.addWidget(QtWidgets.QLabel("Repeats"), row, 2); form.addWidget(self.measure_n_repeats, row, 3); row += 1
        form.addWidget(QtWidgets.QLabel("Samples / point"), row, 0); form.addWidget(self.measure_n_samples, row, 1)
        form.addWidget(QtWidgets.QLabel("Sample period"), row, 2); form.addWidget(self.measure_sample_period, row, 3); row += 1
        form.addWidget(QtWidgets.QLabel("Extra settle after voltage"), row, 0); form.addWidget(self.measure_post_settle, row, 1)
        form.addWidget(QtWidgets.QLabel("Initial settle before first point"), row, 2); form.addWidget(self.measure_initial_settle, row, 3); row += 1
        form.addWidget(QtWidgets.QLabel("Collector voltage"), row, 0); form.addWidget(self.measure_collector_voltage, row, 1)
        form.addWidget(QtWidgets.QLabel("Collector range"), row, 2); form.addWidget(self.measure_collector_range, row, 3); row += 1
        form.addWidget(self.measure_srs_on_check, row, 0, 1, 4); row += 1
        form.addWidget(QtWidgets.QLabel("Voltage tolerance"), row, 0); form.addWidget(self.measure_tol, row, 1)
        form.addWidget(QtWidgets.QLabel("Voltage timeout"), row, 2); form.addWidget(self.measure_timeout, row, 3); row += 1
        form.addWidget(QtWidgets.QLabel("Current limit"), row, 0); form.addWidget(self.measure_current, row, 1); row += 1
        form.addWidget(QtWidgets.QLabel("Ramp mode"), row, 0); form.addWidget(self.measure_ramp_mode, row, 1); row += 1
        form.addWidget(self.measure_zero_after, row, 0, 1, 2); form.addWidget(self.measure_output_off_after_zero, row, 2, 1, 2); row += 1
        form.addWidget(QtWidgets.QLabel("Output CSV"), row, 0); form.addWidget(self.measure_file_edit, row, 1, 1, 2); form.addWidget(self.measure_file_btn, row, 3); row += 1
        form.setRowStretch(row, 1)

        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        splitter.addWidget(right)
        splitter.setSizes([520, 920])

        headers = ["Point", "Repeat", "Mode", "Command V", "Reached V", "Collector", "Sample", "RG1", "RG2", "SCG", "Drift", "Rod"]
        self.measure_table = QtWidgets.QTableWidget(0, len(headers))
        self.measure_table.setHorizontalHeaderLabels(headers)
        self.measure_table.horizontalHeader().setStretchLastSection(True)
        right_layout.addWidget(self.measure_table, stretch=3)

        self.measure_log = QtWidgets.QPlainTextEdit()
        self.measure_log.setReadOnly(True)
        self.measure_log.setMaximumBlockCount(2000)
        right_layout.addWidget(self.measure_log, stretch=1)

        self.measure_mode_combo.currentIndexChanged.connect(self.update_measure_mode_description)
        self.measure_profile_combo.currentIndexChanged.connect(self.update_measure_profile_fields)
        self.measure_file_btn.clicked.connect(self.browse_measure_file)
        self.measure_start_btn.clicked.connect(self.start_bias_scan)
        self.measure_stop_btn.clicked.connect(self.stop_bias_scan)
        self.update_measure_mode_description()
        self.update_measure_profile_fields()

    def append_measure_log(self, text: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.measure_log.appendPlainText(f"{stamp}  {text}")
        self.append_log(text)

    def update_measure_mode_description(self) -> None:
        mode = str(self.measure_mode_combo.currentData())
        cfg = MEASUREMENT_MODES.get(mode, {})
        self.measure_mode_description.setText(cfg.get("description", ""))

    def current_rbd_profile_name(self) -> str:
        return str(self.measure_profile_combo.currentData() or "yield_standard")

    def current_rbd_profile(self) -> dict[str, Any]:
        profiles = self.rbd_profiles.get("profiles", {}) or {}
        return dict(profiles.get(self.current_rbd_profile_name(), default_rbd_profiles()["profiles"]["yield_standard"]))

    def update_measure_profile_fields(self) -> None:
        profile = self.current_rbd_profile()
        if not profile.get("implemented", True):
            self.measure_profile_description.setText(
                f"{profile.get('description', '')} This profile is a placeholder and cannot be used yet."
            )
            return
        self.measure_profile_description.setText(
            f"{profile.get('description', '')}  "
            f"RBD: range={profile.get('range', 'auto')}, filter={profile.get('filter', 32)}, "
            f"interval={profile.get('sample_interval_ms', 50)} ms, "
            f"discard_first={profile.get('discard_first', 0)}."
        )
        if "default_n_samples" in profile:
            self.measure_n_samples.setValue(int(profile.get("default_n_samples", self.measure_n_samples.value())))
        if "default_sample_period_s" in profile:
            self.measure_sample_period.setValue(float(profile.get("default_sample_period_s", self.measure_sample_period.value())))

    def browse_measure_file(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save bias-scan CSV",
            self.measure_file_edit.text(),
            "CSV files (*.csv);;All files (*)",
        )
        if path:
            self.measure_file_edit.setText(path)

    def measure_ramp_mode_value(self) -> int | None:
        txt = self.measure_ramp_mode.currentText()
        return None if txt == "no change" else int(txt)

    def start_bias_scan(self) -> None:
        try:
            if not self.measure_hv_check.isChecked():
                raise RuntimeError("Check 'Enable measurement HV changes' first.")
            self.arm_tdk_if_allowed() if self.hv_enable_check.isChecked() else None
            for name in self.tdk.names():
                dev = self.tdk.get(name)
                if hasattr(dev, "arm_hv_changes"):
                    dev.arm_hv_changes(HV_TOKEN)

            profile_name = self.current_rbd_profile_name()
            profile = self.current_rbd_profile()
            if not profile.get("implemented", True):
                raise RuntimeError(f"RBD profile '{profile_name}' is marked as future/not implemented.")
            if self.timer.isActive():
                self.timer.stop()
                self.append_measure_log("Monitor timer paused while bias scan runs.")
            profile_results = self.rbd.apply_acquisition_profile(profile, start_sampling=True)
            bad = {k: v for k, v in profile_results.items() if str(v).startswith("ERROR")}
            if bad:
                raise RuntimeError("RBD profile apply failed: " + "; ".join(f"{k}: {v}" for k, v in bad.items()))
            self.rbd_sampling_active = True
            self.append_measure_log(f"RBD acquisition profile applied: {profile_name}")

            values = make_scan_values(self.measure_v_start.value(), self.measure_v_end.value(), self.measure_v_step.value())
            if not values:
                raise RuntimeError("No voltage points generated.")
            output_path = Path(self.measure_file_edit.text())
            if output_path.suffix.lower() != ".csv":
                output_path = output_path.with_suffix(".csv")
                self.measure_file_edit.setText(str(output_path))
            event_log_path = output_path.with_suffix(".events.log")
            metadata_path = output_path.with_suffix(".metadata.yaml")

            self.measure_table.setRowCount(0)
            self.scan_thread = QtCore.QThread(self)
            self.scan_worker = BiasScanWorker(
                rbd=self.rbd,
                tdk=self.tdk,
                srs=self.srs,
                mode=str(self.measure_mode_combo.currentData()),
                energy_kev=float(self.measure_energy_spin.value()),
                scan_values_V=values,
                n_samples=int(self.measure_n_samples.value()),
                n_repeats=int(self.measure_n_repeats.value()),
                sample_period_s=float(self.measure_sample_period.value()),
                post_settle_s=float(self.measure_post_settle.value()),
                initial_settle_s=float(self.measure_initial_settle.value()),
                voltage_tolerance_V=float(self.measure_tol.value()),
                voltage_timeout_s=float(self.measure_timeout.value()),
                current_limit_A=float(self.measure_current.value()) * 1e-3,
                output_path=output_path,
                event_log_path=event_log_path,
                metadata_path=metadata_path,
                rbd_profile_name=profile_name,
                rbd_profile=profile,
                discard_first=int(profile.get("discard_first", 0)),
                collector_voltage_V=float(self.measure_collector_voltage.value()),
                collector_range=str(self.measure_collector_range.currentText()),
                ensure_srs_output_on=self.measure_srs_on_check.isChecked(),
                zero_after_scan=self.measure_zero_after.isChecked(),
                output_off_after_zero=self.measure_output_off_after_zero.isChecked(),
                ramp_mode=self.measure_ramp_mode_value(),
            )
            self.scan_worker.moveToThread(self.scan_thread)
            self.scan_thread.started.connect(self.scan_worker.run)
            self.scan_worker.log.connect(self.append_measure_log)
            self.scan_worker.row_ready.connect(self.handle_measure_row)
            self.scan_worker.finished.connect(self.bias_scan_finished)
            self.scan_worker.failed.connect(self.bias_scan_failed)
            self.scan_worker.finished.connect(self.scan_thread.quit)
            self.scan_worker.failed.connect(self.scan_thread.quit)
            self.scan_thread.finished.connect(self.scan_worker.deleteLater)
            self.scan_thread.finished.connect(self.scan_thread.deleteLater)
            self.measure_start_btn.setEnabled(False)
            self.measure_stop_btn.setEnabled(True)
            self.scan_thread.start()
        except Exception as exc:
            self.error_box("Bias scan start failed", exc)

    def stop_bias_scan(self) -> None:
        if self.scan_worker is not None:
            self.scan_worker.request_stop()
            self.measure_stop_btn.setEnabled(False)

    def handle_measure_row(self, row: dict) -> None:
        table_row = self.measure_table.rowCount()
        self.measure_table.insertRow(table_row)
        def val(key: str) -> str:
            x = row.get(key)
            if isinstance(x, float):
                return f"{x:.6g}"
            return "" if x is None else str(x)
        mapping = [
            "point_index",
            "repeat_index",
            "mode",
            "commanded_voltage_V",
            "reached_voltage_V",
            "Dopey_mean_A",
            "Sneezy_mean_A",
            "Happy_mean_A",
            "Sleepy_mean_A",
            "Grumpy_mean_A",
            "Bashful_mean_A",
            "Doc_mean_A",
        ]
        for col, key in enumerate(mapping):
            item = QtWidgets.QTableWidgetItem(val(key))
            item.setFlags(item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
            self.measure_table.setItem(table_row, col, item)
        self.measure_table.scrollToBottom()
        self.append_measure_log(
            f"Saved point {row.get('point_index')}: V={row.get('commanded_voltage_V')} reached={row.get('reached_voltage_V')}"
        )

    def bias_scan_finished(self, path: str) -> None:
        self.measure_start_btn.setEnabled(True)
        self.measure_stop_btn.setEnabled(False)
        self.scan_worker = None
        self.scan_thread = None
        self.append_measure_log(f"Bias scan finished. Saved: {path}")
        self.update_tdk_status(log_errors_only=True)

    def bias_scan_failed(self, text: str) -> None:
        self.measure_start_btn.setEnabled(True)
        self.measure_stop_btn.setEnabled(False)
        self.scan_worker = None
        self.scan_thread = None
        self.error_box("Bias scan failed", text)


    # ------------------------------------------------------------------
    # XY imaging tab
    # ------------------------------------------------------------------
    def _build_imaging_tab(self) -> None:
        layout = QtWidgets.QVBoxLayout(self.imaging_tab)

        top = QtWidgets.QHBoxLayout()
        layout.addLayout(top)
        self.imaging_start_btn = QtWidgets.QPushButton("Start XY image")
        self.imaging_stop_btn = QtWidgets.QPushButton("Stop after current pixel")
        self.imaging_stop_btn.setEnabled(False)
        self.imaging_enable_check = QtWidgets.QCheckBox("Enable imaging hardware writes")
        self.imaging_enable_check.setStyleSheet("font-weight: bold; color: #8b0000;")
        top.addWidget(self.imaging_start_btn)
        top.addWidget(self.imaging_stop_btn)
        top.addWidget(self.imaging_enable_check)
        top.addStretch()

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        layout.addWidget(splitter, stretch=1)

        left = QtWidgets.QWidget()
        form = QtWidgets.QGridLayout(left)
        splitter.addWidget(left)

        self.imaging_x_start = QtWidgets.QDoubleSpinBox(); self.imaging_x_end = QtWidgets.QDoubleSpinBox(); self.imaging_x_step = QtWidgets.QDoubleSpinBox()
        self.imaging_y_start = QtWidgets.QDoubleSpinBox(); self.imaging_y_end = QtWidgets.QDoubleSpinBox(); self.imaging_y_step = QtWidgets.QDoubleSpinBox()
        for w in (self.imaging_x_start, self.imaging_x_end, self.imaging_x_step, self.imaging_y_start, self.imaging_y_end, self.imaging_y_step):
            w.setRange(-300.0, 300.0)
            w.setDecimals(3)
            w.setSingleStep(1.0)
            w.setSuffix(" V")
        self.imaging_x_start.setValue(-50.0); self.imaging_x_end.setValue(50.0); self.imaging_x_step.setValue(5.0)
        self.imaging_y_start.setValue(-50.0); self.imaging_y_end.setValue(50.0); self.imaging_y_step.setValue(5.0)

        self.imaging_n_samples = QtWidgets.QSpinBox(); self.imaging_n_samples.setRange(1, 10000); self.imaging_n_samples.setValue(3)
        self.imaging_sample_period = QtWidgets.QDoubleSpinBox(); self.imaging_sample_period.setRange(0.0, 60.0); self.imaging_sample_period.setValue(0.1); self.imaging_sample_period.setSuffix(" s")
        self.imaging_settle = QtWidgets.QDoubleSpinBox(); self.imaging_settle.setRange(0.0, 300.0); self.imaging_settle.setValue(0.5); self.imaging_settle.setSuffix(" s")
        self.imaging_collector_voltage = QtWidgets.QDoubleSpinBox(); self.imaging_collector_voltage.setRange(-100.0, 100.0); self.imaging_collector_voltage.setDecimals(4); self.imaging_collector_voltage.setValue(50.0); self.imaging_collector_voltage.setSuffix(" V")
        self.imaging_collector_range = QtWidgets.QComboBox(); self.imaging_collector_range.addItems(["RANGE1", "RANGE10", "RANGE100"]); self.imaging_collector_range.setCurrentText("RANGE100")
        self.imaging_zero_tdk = QtWidgets.QCheckBox("Zero TDK supplies first: all non-collector electrodes grounded")
        self.imaging_zero_tdk.setChecked(True)
        self.imaging_serpentine = QtWidgets.QCheckBox("Serpentine scan")
        self.imaging_serpentine.setChecked(True)
        self.imaging_abs_currents = QtWidgets.QCheckBox("Use absolute currents in yield calculation")
        self.imaging_abs_currents.setChecked(False)

        self.imaging_profile_combo = QtWidgets.QComboBox()
        profiles = self.rbd_profiles.get("profiles", {}) or {}
        default_profile = self.rbd_profiles.get("default_profile")
        default_index = 0
        for idx, (key, cfg) in enumerate(profiles.items()):
            label = key if cfg.get("implemented", True) else f"{key} (future)"
            self.imaging_profile_combo.addItem(label, key)
            if key == default_profile:
                default_index = idx
        if self.imaging_profile_combo.count() == 0:
            self.imaging_profile_combo.addItem("yield_standard", "yield_standard")
        self.imaging_profile_combo.setCurrentIndex(default_index)

        default_path = self.output_dir / f"xy_image_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.imaging_file_edit = QtWidgets.QLineEdit(str(default_path))
        self.imaging_browse_btn = QtWidgets.QPushButton("Browse")

        row = 0
        form.addWidget(QtWidgets.QLabel("X start"), row, 0); form.addWidget(self.imaging_x_start, row, 1)
        form.addWidget(QtWidgets.QLabel("X end"), row, 2); form.addWidget(self.imaging_x_end, row, 3)
        form.addWidget(QtWidgets.QLabel("X step"), row, 4); form.addWidget(self.imaging_x_step, row, 5); row += 1
        form.addWidget(QtWidgets.QLabel("Y start"), row, 0); form.addWidget(self.imaging_y_start, row, 1)
        form.addWidget(QtWidgets.QLabel("Y end"), row, 2); form.addWidget(self.imaging_y_end, row, 3)
        form.addWidget(QtWidgets.QLabel("Y step"), row, 4); form.addWidget(self.imaging_y_step, row, 5); row += 1
        form.addWidget(QtWidgets.QLabel("Samples/pixel"), row, 0); form.addWidget(self.imaging_n_samples, row, 1)
        form.addWidget(QtWidgets.QLabel("Sample period"), row, 2); form.addWidget(self.imaging_sample_period, row, 3)
        form.addWidget(QtWidgets.QLabel("Settle after X/Y"), row, 4); form.addWidget(self.imaging_settle, row, 5); row += 1
        form.addWidget(QtWidgets.QLabel("Collector voltage"), row, 0); form.addWidget(self.imaging_collector_voltage, row, 1)
        form.addWidget(QtWidgets.QLabel("SRS range"), row, 2); form.addWidget(self.imaging_collector_range, row, 3)
        form.addWidget(QtWidgets.QLabel("RBD profile"), row, 4); form.addWidget(self.imaging_profile_combo, row, 5); row += 1
        form.addWidget(self.imaging_zero_tdk, row, 0, 1, 3)
        form.addWidget(self.imaging_serpentine, row, 3, 1, 1)
        form.addWidget(self.imaging_abs_currents, row, 4, 1, 2); row += 1
        note = QtWidgets.QLabel("TEY per pixel = (Collector + grids + Rod + Drift Tube)/(same + Sample). CSV uses physical electrode labels; dwarf nicknames are hidden except internal-id metadata.")
        note.setWordWrap(True)
        form.addWidget(note, row, 0, 1, 6); row += 1
        form.addWidget(QtWidgets.QLabel("Output CSV"), row, 0); form.addWidget(self.imaging_file_edit, row, 1, 1, 4); form.addWidget(self.imaging_browse_btn, row, 5); row += 1

        self.imaging_table = QtWidgets.QTableWidget(0, 6)
        self.imaging_table.setHorizontalHeaderLabels(["Pixel", "X V", "Y V", "TEY", "Numerator A", "Denominator A"])
        self.imaging_table.horizontalHeader().setStretchLastSection(True)
        form.addWidget(self.imaging_table, row, 0, 1, 6); row += 1
        form.setRowStretch(row - 1, 1)

        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        splitter.addWidget(right)
        splitter.setSizes([650, 850])

        self.imaging_plot = pg.PlotWidget(title="Live XY total-yield image")
        self.imaging_plot.setLabel("bottom", "X deflection", units="V")
        self.imaging_plot.setLabel("left", "Y deflection", units="V")
        self.imaging_image_item = pg.ImageItem()
        self.imaging_plot.addItem(self.imaging_image_item)
        self.imaging_selected_marker = pg.ScatterPlotItem(size=14, brush=None, pen=pg.mkPen("y", width=2), symbol="s")
        self.imaging_plot.addItem(self.imaging_selected_marker)
        self.imaging_click_proxy = pg.SignalProxy(
            self.imaging_plot.scene().sigMouseClicked,
            rateLimit=30,
            slot=self.handle_imaging_plot_click,
        )
        right_layout.addWidget(self.imaging_plot, stretch=3)

        self.imaging_pixel_info = QtWidgets.QPlainTextEdit()
        self.imaging_pixel_info.setReadOnly(True)
        self.imaging_pixel_info.setMaximumHeight(150)
        self.imaging_pixel_info.setPlainText("Click a measured pixel in the live image to inspect X, Y, TEY, and electrode currents. This also works while the scan is running for pixels already measured.")
        right_layout.addWidget(QtWidgets.QLabel("Selected pixel"))
        right_layout.addWidget(self.imaging_pixel_info)

        self.imaging_log = QtWidgets.QPlainTextEdit()
        self.imaging_log.setReadOnly(True)
        self.imaging_log.setMaximumBlockCount(2000)
        right_layout.addWidget(QtWidgets.QLabel("Imaging log"))
        right_layout.addWidget(self.imaging_log, stretch=1)

        self.imaging_start_btn.clicked.connect(self.start_xy_imaging)
        self.imaging_stop_btn.clicked.connect(self.stop_xy_imaging)
        self.imaging_browse_btn.clicked.connect(self.browse_imaging_file)

    def browse_imaging_file(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save XY imaging CSV",
            self.imaging_file_edit.text(),
            "CSV files (*.csv);;All files (*)",
        )
        if path:
            self.imaging_file_edit.setText(path)

    def current_imaging_rbd_profile_name(self) -> str:
        return str(self.imaging_profile_combo.currentData())

    def current_imaging_rbd_profile(self) -> dict[str, Any]:
        name = self.current_imaging_rbd_profile_name()
        return dict((self.rbd_profiles.get("profiles", {}) or {}).get(name, {}))

    def start_xy_imaging(self) -> None:
        try:
            if np is None:
                raise RuntimeError("numpy is required for the live image array; install numpy in this environment.")
            if not self.imaging_enable_check.isChecked():
                raise RuntimeError("Check 'Enable imaging hardware writes' first.")
            x_values = make_scan_values(self.imaging_x_start.value(), self.imaging_x_end.value(), self.imaging_x_step.value())
            y_values = make_scan_values(self.imaging_y_start.value(), self.imaging_y_end.value(), self.imaging_y_step.value())
            if not x_values or not y_values:
                raise RuntimeError("No X/Y points generated.")
            if len(x_values) * len(y_values) > 20000:
                raise RuntimeError("Too many pixels for one GUI scan; reduce range or increase step.")

            if self.timer.isActive():
                self.timer.stop()
                self.append_log("Monitor timer paused while XY imaging runs.")

            profile_name = self.current_imaging_rbd_profile_name()
            profile = self.current_imaging_rbd_profile()
            if not profile.get("implemented", True):
                raise RuntimeError(f"RBD profile '{profile_name}' is marked as future/not implemented.")
            profile_results = self.rbd.apply_acquisition_profile(profile, start_sampling=True)
            bad = {k: v for k, v in profile_results.items() if str(v).startswith("ERROR")}
            if bad:
                raise RuntimeError("RBD profile apply failed: " + "; ".join(f"{k}: {v}" for k, v in bad.items()))
            self.rbd_sampling_active = True
            self.append_log(f"RBD acquisition profile applied for imaging: {profile_name}")

            output_path = Path(self.imaging_file_edit.text())
            if output_path.suffix.lower() != ".csv":
                output_path = output_path.with_suffix(".csv")
                self.imaging_file_edit.setText(str(output_path))
            event_log_path = output_path.with_suffix(".events.log")
            metadata_path = output_path.with_suffix(".metadata.yaml")

            self.imaging_x_values = list(x_values)
            self.imaging_y_values = list(y_values)
            self.imaging_pixel_rows = {}
            self.imaging_selected_ix = None
            self.imaging_selected_iy = None
            if hasattr(self, "imaging_selected_marker"):
                self.imaging_selected_marker.setData([], [])
            if hasattr(self, "imaging_pixel_info"):
                self.imaging_pixel_info.setPlainText("Scan started. Click a measured pixel to inspect X, Y, TEY, and electrode currents.")
            self.imaging_image_array = np.full((len(y_values), len(x_values)), np.nan, dtype=float)
            self.imaging_image_item.setImage(self.imaging_image_array, autoLevels=True)
            try:
                from PySide6 import QtCore as _QtCore
                if len(x_values) > 1 and len(y_values) > 1:
                    x0 = min(x_values); y0 = min(y_values)
                    w = max(x_values) - min(x_values)
                    h = max(y_values) - min(y_values)
                    self.imaging_image_item.setRect(_QtCore.QRectF(x0, y0, w if w else 1.0, h if h else 1.0))
            except Exception:
                pass
            self.imaging_table.setRowCount(0)

            self.imaging_thread = QtCore.QThread(self)
            self.imaging_worker = ImagingScanWorker(
                rbd=self.rbd,
                tdk=self.tdk,
                srs=self.srs,
                kimball=self.kimball,
                x_values=x_values,
                y_values=y_values,
                n_samples=int(self.imaging_n_samples.value()),
                sample_period_s=float(self.imaging_sample_period.value()),
                settle_s=float(self.imaging_settle.value()),
                collector_voltage_V=float(self.imaging_collector_voltage.value()),
                collector_range=str(self.imaging_collector_range.currentText()),
                output_path=output_path,
                event_log_path=event_log_path,
                metadata_path=metadata_path,
                zero_tdk_first=self.imaging_zero_tdk.isChecked(),
                serpentine=self.imaging_serpentine.isChecked(),
                rbd_profile_name=profile_name,
                rbd_profile=profile,
                discard_first=int(profile.get("discard_first", 0)),
                use_abs_currents=self.imaging_abs_currents.isChecked(),
            )
            self.imaging_worker.moveToThread(self.imaging_thread)
            self.imaging_thread.started.connect(self.imaging_worker.run)
            self.imaging_worker.log.connect(self.append_log)
            self.imaging_worker.pixel_ready.connect(self.handle_imaging_pixel)
            self.imaging_worker.finished.connect(self.xy_imaging_finished)
            self.imaging_worker.failed.connect(self.xy_imaging_failed)
            self.imaging_worker.finished.connect(self.imaging_thread.quit)
            self.imaging_worker.failed.connect(self.imaging_thread.quit)
            self.imaging_thread.finished.connect(self.imaging_worker.deleteLater)
            self.imaging_thread.finished.connect(self.imaging_thread.deleteLater)
            self.imaging_start_btn.setEnabled(False)
            self.imaging_stop_btn.setEnabled(True)
            self.imaging_thread.start()
        except Exception as exc:
            self.error_box("XY imaging start failed", exc)

    def stop_xy_imaging(self) -> None:
        if self.imaging_worker is not None:
            self.imaging_worker.request_stop()
            self.imaging_stop_btn.setEnabled(False)

    def handle_imaging_pixel(self, row: dict) -> None:
        table_row = self.imaging_table.rowCount()
        self.imaging_table.insertRow(table_row)
        def val(key: str) -> str:
            x = row.get(key)
            if isinstance(x, float):
                return f"{x:.6g}"
            return "" if x is None else str(x)
        keys = ["pixel_index", "x_deflection_V", "y_deflection_V", "total_yield", "yield_numerator_A", "yield_denominator_A"]
        for col, key in enumerate(keys):
            item = QtWidgets.QTableWidgetItem(val(key))
            item.setFlags(item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
            self.imaging_table.setItem(table_row, col, item)
        self.imaging_table.scrollToBottom()

        try:
            iy = int(row.get("iy")); ix = int(row.get("ix"))
            self.imaging_pixel_rows[(ix, iy)] = dict(row)
        except Exception:
            ix = iy = None

        if self.imaging_image_array is not None and row.get("total_yield") is not None:
            try:
                iy = int(row.get("iy")); ix = int(row.get("ix"))
                self.imaging_image_array[iy, ix] = float(row.get("total_yield"))
                self.imaging_image_item.setImage(self.imaging_image_array, autoLevels=True)
                if self.imaging_selected_ix == ix and self.imaging_selected_iy == iy:
                    self.show_imaging_pixel_info(ix, iy)
            except Exception as exc:
                self.append_log(f"Live image update error: {exc}")

    def nearest_imaging_index(self, values: list[float], value: float) -> int | None:
        if not values:
            return None
        return min(range(len(values)), key=lambda i: abs(values[i] - value))

    def handle_imaging_plot_click(self, event) -> None:
        try:
            ev = event[0] if isinstance(event, (list, tuple)) else event
            if ev is None:
                return
            pos = ev.scenePos()
            if not self.imaging_plot.sceneBoundingRect().contains(pos):
                return
            point = self.imaging_plot.plotItem.vb.mapSceneToView(pos)
            ix = self.nearest_imaging_index(self.imaging_x_values, float(point.x()))
            iy = self.nearest_imaging_index(self.imaging_y_values, float(point.y()))
            if ix is None or iy is None:
                return
            self.show_imaging_pixel_info(ix, iy)
        except Exception as exc:
            self.append_log(f"Pixel click inspect error: {exc}")

    def show_imaging_pixel_info(self, ix: int, iy: int) -> None:
        self.imaging_selected_ix = ix
        self.imaging_selected_iy = iy
        try:
            x = float(self.imaging_x_values[ix])
            y = float(self.imaging_y_values[iy])
            self.imaging_selected_marker.setData([x], [y])
        except Exception:
            x = y = None
        row = self.imaging_pixel_rows.get((ix, iy))
        if row is None:
            if x is None or y is None:
                msg = "Selected pixel has not been measured yet."
            else:
                msg = f"Selected pixel has not been measured yet.\nX = {x:g} V\nY = {y:g} V"
            self.imaging_pixel_info.setPlainText(msg)
            return

        def fnum(key: str, units: str = "") -> str:
            val = row.get(key)
            if val is None:
                return ""
            try:
                return f"{float(val):.6g}{units}"
            except Exception:
                return str(val)

        lines = [
            f"Pixel {row.get('pixel_index')}  (ix={ix}, iy={iy})",
            f"X = {fnum('x_deflection_V', ' V')}",
            f"Y = {fnum('y_deflection_V', ' V')}",
            f"TEY = {fnum('total_yield')}",
            f"Numerator = {fnum('yield_numerator_A', ' A')}",
            f"Denominator = {fnum('yield_denominator_A', ' A')}",
        ]
        current_keys = [
            ("Sample", "Sample_mean_A"),
            ("Collector", "Collector_mean_A"),
            ("Retarding Grid 1", "Retarding_Grid_1_mean_A"),
            ("Retarding Grid 2", "Retarding_Grid_2_mean_A"),
            ("Space-charge Grid", "Space_charge_Grid_mean_A"),
            ("Rod", "Rod_mean_A"),
            ("Drift Tube", "Drift_Tube_mean_A"),
        ]
        lines.append("Currents:")
        for label, key in current_keys:
            if key in row:
                lines.append(f"  {label}: {fnum(key, ' A')}")
        self.imaging_pixel_info.setPlainText("\n".join(lines))

    def xy_imaging_finished(self, path: str) -> None:
        self.imaging_start_btn.setEnabled(True)
        self.imaging_stop_btn.setEnabled(False)
        self.imaging_worker = None
        self.imaging_thread = None
        self.append_log(f"XY imaging finished. Saved: {path}")

    def xy_imaging_failed(self, text: str) -> None:
        self.imaging_start_btn.setEnabled(True)
        self.imaging_stop_btn.setEnabled(False)
        self.imaging_worker = None
        self.imaging_thread = None
        self.error_box("XY imaging failed", text)

    # ------------------------------------------------------------------
    # Status updates
    # ------------------------------------------------------------------
    def read_all_once(self) -> None:
        self.read_rbd_once()
        self.update_tdk_status()
        self.update_srs_status()

    def start_monitor(self) -> None:
        try:
            self.ensure_rbd_sampling_started()
            if self.rbd_save_check.isChecked() and self.rbd_logger is None:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.rbd_logger = RBDCsvLogger(self.output_dir / f"rbd_monitor_{ts}.csv")
                self.append_log(f"Saving RBD CSV: {self.rbd_logger.output_path}")
            self.timer.start(int(float(self.monitor_rbd_period.value()) * 1000))
            self.monitor_start_btn.setEnabled(False)
            self.monitor_stop_btn.setEnabled(True)
            self.rbd_start_btn.setEnabled(False)
            self.rbd_stop_btn.setEnabled(True)
            self.append_log("Monitor started")
        except Exception as exc:
            self.error_box("Monitor start failed", exc)

    def stop_monitor(self) -> None:
        self.timer.stop()
        try:
            self.rbd.stop_all()
        except Exception:
            pass
        self.rbd_sampling_active = False
        if self.rbd_logger:
            path = self.rbd_logger.output_path
            self.rbd_logger.close()
            self.rbd_logger = None
            self.append_log(f"Saved RBD CSV: {path}")
        self.monitor_start_btn.setEnabled(True)
        self.monitor_stop_btn.setEnabled(False)
        self.rbd_start_btn.setEnabled(True)
        self.rbd_stop_btn.setEnabled(False)
        self.append_log("Monitor stopped")

    def monitor_tick(self) -> None:
        self.monitor_ticks += 1
        try:
            readings = self.rbd.read_all_once(latest=True)
            self.handle_rbd_readings(readings)
            if self.monitor_ticks % 5 == 0 and self.monitor_tdk_check.isChecked():
                self.update_tdk_status(log_errors_only=True)
            if self.monitor_ticks % 5 == 0 and self.monitor_srs_check.isChecked():
                self.update_srs_status(log_errors_only=True)
        except Exception as exc:
            self.append_log(f"Monitor tick error: {exc}")

    def handle_rbd_readings(self, readings) -> None:
        t = time.time() - self.t0
        total = 0.0
        total_count = 0
        for r in readings:
            for table in (self.monitor_rbd_table, self.pico_table):
                row = getattr(table, "name_to_row", {}).get(r.name)
                if row is None:
                    continue
                table.item(row, 3).setText(fmt_current(r.current_A))
                table.item(row, 4).setText(str(r.status_code or ""))
                table.item(row, 5).setText(str(r.range_code or ""))
                table.item(row, 6).setText(r.raw_response or "")
                table.item(row, 7).setText(r.error or "")
            y = r.current_A
            if y is not None:
                self.rbd_history[r.name].append((t, y))
                total += y
                total_count += 1
            if r.error:
                self.append_log(f"RBD {self.rbd.labels.get(r.name, r.name)}: {r.error}")
        if self.rbd_logger:
            self.rbd_logger.write_many(readings)
        self.update_rbd_plot()
        if total_count:
            self.status_label.setText(f"RBD updated; sum over channels = {fmt_current(total)}")

    def update_rbd_plot(self) -> None:
        for name, curve in self.rbd_curves.items():
            hist = self.rbd_history[name]
            if hist:
                curve.setData([p[0] for p in hist], [p[1] for p in hist])

    def update_tdk_status(self, *, log_errors_only: bool = False) -> None:
        t = time.time() - self.t0
        for name in self.tdk.names():
            try:
                dev = self.tdk.get(name)
                dev.open()
                st = dev.status()
                d = st.to_dict()
                for table in (self.monitor_tdk_table, self.power_tdk_table):
                    row = getattr(table, "name_to_row", {}).get(name)
                    if row is None:
                        continue
                    table.item(row, 4).setText("ON" if d.get("output_on") else "OFF")
                    table.item(row, 5).setText(fmt_float(d.get("signed_voltage_setpoint_V"), " V"))
                    table.item(row, 6).setText(fmt_float(d.get("signed_measured_voltage_V"), " V"))
                    table.item(row, 7).setText(fmt_current(d.get("current_setpoint_A")))
                    table.item(row, 8).setText(fmt_current(d.get("measured_current_A")))
                    table.item(row, 9).setText(str(d.get("idn") or ""))
                actual = d.get("signed_measured_voltage_V")
                if actual is not None:
                    self.tdk_history[name].append((t, float(actual)))
            except Exception as exc:
                if log_errors_only:
                    self.append_log(f"TDK {name}: {exc}")
                else:
                    self.append_log(f"TDK {name} status error: {exc}")
        self.update_tdk_plot()

    def update_tdk_plot(self) -> None:
        for name, curve in self.tdk_curves.items():
            hist = self.tdk_history.get(name, [])
            if hist:
                curve.setData([p[0] for p in hist], [p[1] for p in hist])

    def update_srs_status(self, *, log_errors_only: bool = False) -> None:
        try:
            self.srs.open()
            st = self.srs.status().to_dict()
            values = [
                getattr(self.srs, "port_name", ""),
                st.get("range"),
                fmt_float(st.get("voltage_setpoint_V"), " V"),
                "ON" if st.get("output_on") else "OFF",
                "closed" if st.get("interlock_closed") else "open",
                "YES" if st.get("overload") else "no",
                st.get("idn"),
            ]
            for col, val in enumerate(values):
                item = self.monitor_srs_table.item(0, col)
                if item is None:
                    item = QtWidgets.QTableWidgetItem()
                    item.setFlags(item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
                    self.monitor_srs_table.setItem(0, col, item)
                item.setText(str(val or ""))
        except Exception as exc:
            if log_errors_only:
                self.append_log(f"SRS: {exc}")
            else:
                self.append_log(f"SRS status error: {exc}")

    def closeEvent(self, event) -> None:  # noqa: N802
        try:
            self.stop_monitor()
        except Exception:
            pass
        try:
            self.rbd.stop_all()
            self.rbd.close_all()
        except Exception:
            pass
        try:
            self.tdk.close_all()
        except Exception:
            pass
        try:
            self.srs.close()
        except Exception:
            pass
        event.accept()


def find_duplicate_ports(rbd: RBDManager, tdk: TDKManager, srs) -> list[str]:
    ports: dict[str, list[str]] = {}
    for name, dev in rbd.devices.items():
        ports.setdefault(str(dev.port_name).upper(), []).append(f"RBD:{name}")
    for name, cfg in tdk.configs.items():
        if cfg.enabled:
            ports.setdefault(str(cfg.port).upper(), []).append(f"TDK:{name}")
    srs_port = getattr(srs, "port_name", "")
    if srs_port:
        ports.setdefault(str(srs_port).upper(), []).append("SRS")
    return [f"{port}: {', '.join(names)}" for port, names in ports.items() if port and len(names) > 1]


def main() -> None:
    parser = argparse.ArgumentParser(description="Integrated RFA Python DAQ GUI v0.17")
    parser.add_argument("--rbd-config", default=default_config_path("rbd_channels.yaml"))
    parser.add_argument("--rbd-profiles", default=default_config_path("rbd_acquisition_profiles.yaml"))
    parser.add_argument("--tdk-config", default=default_config_path("tdk_supplies.yaml"))
    parser.add_argument("--srs-config", default=default_config_path("srs_dc205.yaml"))
    parser.add_argument("--kimball-config", default=default_config_path("kimball_daq.yaml"))
    parser.add_argument("--real", action="store_true", help="use real hardware; default is simulated for safety")
    parser.add_argument("--simulate", action="store_true", help="force simulation")
    parser.add_argument("--output-dir", default="data")
    args = parser.parse_args()

    # Safety default: GUI is simulated unless --real is supplied.
    simulate = True
    if args.real:
        simulate = False
    if args.simulate:
        simulate = True

    app = QtWidgets.QApplication(sys.argv)
    rbd = RBDManager.from_yaml(args.rbd_config, simulate=simulate)
    tdk = TDKManager.from_yaml(args.tdk_config, simulate_override=simulate)
    srs = load_srs_from_yaml(args.srs_config, simulate_override=simulate)
    kimball = KimballDAQ.from_yaml(args.kimball_config, simulate_override=simulate)
    rbd_profiles = load_rbd_acquisition_profiles(args.rbd_profiles)
    win = RFAControlWindow(
        rbd=rbd,
        tdk=tdk,
        srs=srs,
        kimball=kimball,
        simulate=simulate,
        output_dir=args.output_dir,
        rbd_profiles=rbd_profiles,
    )
    duplicates = find_duplicate_ports(rbd, tdk, srs)
    if duplicates and not simulate:
        win.append_log("WARNING: duplicate COM ports in config: " + "; ".join(duplicates))
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
