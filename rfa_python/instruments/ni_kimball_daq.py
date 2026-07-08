"""NI-DAQmx foundation layer for the Kimball/EGPS electron gun system.

v0.17.2 adds conservative Source-mode ramp support on top of v0.16:
- discover/list NI devices and channels;
- read metering analog inputs with engineering-unit scaling;
- write 0 V to all configured analog-output controls;
- optionally write tiny raw AO test voltages after an explicit safety token;
- load/interpolate the Kimball LUT;
- apply electrostatic controls only (Energy/Grid/Focus/X/Y) in a safe order;
- ramp Source/ECC in Source mode only, with meter logging and an optional abort callback.

ECC mode and automatic measurement integration are still intentionally disabled.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import csv
import time
import datetime as _dt

import yaml

GUN_DAQ_TOKEN = "I_UNDERSTAND_KIMBALL_GUN_DAQ"


def _import_nidaqmx():
    try:
        import nidaqmx  # type: ignore
        from nidaqmx.system import System  # type: ignore
        return nidaqmx, System
    except Exception as exc:  # pragma: no cover - depends on lab PC
        raise RuntimeError(
            "nidaqmx is not available. Install NI-DAQmx driver/runtime and then `pip install nidaqmx`."
        ) from exc


@dataclass
class KimballLUTRow:
    energy_eV: float
    x_center_V: float
    y_center_V: float


@dataclass
class KimballElectrostaticTarget:
    """Engineering-unit target for electrostatic Kimball controls.

    Source/ECC is intentionally excluded.  The values here are the values
    shown to the user / LabVIEW-style engineering values, not raw NI AO volts.
    """

    energy_eV: float
    grid_V: float | None = None
    focus_kV: float | None = None
    x_center_V: float | None = None
    y_center_V: float | None = None
    apply_grid: bool = True
    apply_focus: bool = False
    apply_energy: bool = True
    apply_deflection: bool = True

    @property
    def energy_keV(self) -> float:
        return float(self.energy_eV) / 1000.0


@dataclass
class KimballSourceRampStep:
    index: int
    target_source_V: float
    written: dict[str, Any]
    meters: dict[str, dict[str, Any]]
    timestamp_s: float


@dataclass
class MeterChannel:
    name: str
    device: str
    channel: str
    engineering_max: float
    input_range_V: float = 2.0
    units: str = ""
    bipolar: bool = False

    @property
    def physical_channel(self) -> str:
        return f"{self.device}/{self.channel}"

    def to_engineering(self, raw_voltage: float) -> float:
        # The original Kimball metering files use columns like:
        # name, device index, channel index, engineering scale maximum, DAQ input range, units.
        # For most meters, 0..input_range_V corresponds to 0..engineering_max.
        # For deflection meters, allow signed mapping if bipolar=True.
        if self.input_range_V == 0:
            return raw_voltage
        return float(raw_voltage) * float(self.engineering_max) / float(self.input_range_V)


@dataclass
class ControlChannel:
    name: str
    device: str
    channel: str
    engineering_max: float
    output_range_V: float = 10.0
    units: str = ""
    coarse_increment: float | None = None
    fine_increment: float | None = None
    bipolar: bool = False

    @property
    def physical_channel(self) -> str:
        return f"{self.device}/{self.channel}"

    def to_ao_voltage(self, engineering_value: float) -> float:
        if self.engineering_max == 0:
            return float(engineering_value)
        return float(engineering_value) * float(self.output_range_V) / float(self.engineering_max)

    def from_ao_voltage(self, ao_voltage: float) -> float:
        if self.output_range_V == 0:
            return float(ao_voltage)
        return float(ao_voltage) * float(self.engineering_max) / float(self.output_range_V)


@dataclass
class DigitalControl:
    name: str
    legacy_line: str | None = None
    daqmx_line: str | None = None
    note: str = ""
    on_value: bool = True
    off_value: bool = False


@dataclass
class KimballDAQConfig:
    simulate: bool = True
    metering_device: str = "PXI1Slot2"
    control_device: str = "PXI1Slot4"
    ao_min_V: float = -10.0
    ao_max_V: float = 10.0
    ai_min_V: float = -10.0
    ai_max_V: float = 10.0
    max_raw_ao_test_V: float = 0.2
    source_ramp_increment_V: float = 0.1
    source_ramp_delay_s: float = 5.0
    meter_samples_default: int = 20
    lut_path: str = "egunlut.csv"
    meters: dict[str, MeterChannel] = field(default_factory=dict)
    controls: dict[str, ControlChannel] = field(default_factory=dict)
    digital_controls: dict[str, DigitalControl] = field(default_factory=dict)

    @classmethod
    def from_yaml(cls, path: str | Path, *, simulate_override: bool | None = None) -> "KimballDAQConfig":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        cfg = cls()
        cfg.simulate = bool(data.get("simulate", cfg.simulate)) if simulate_override is None else bool(simulate_override)
        cfg.metering_device = str(data.get("metering_device", cfg.metering_device))
        cfg.control_device = str(data.get("control_device", cfg.control_device))
        safety = data.get("safety", {}) or {}
        cfg.ao_min_V = float(safety.get("ao_min_V", cfg.ao_min_V))
        cfg.ao_max_V = float(safety.get("ao_max_V", cfg.ao_max_V))
        cfg.ai_min_V = float(safety.get("ai_min_V", cfg.ai_min_V))
        cfg.ai_max_V = float(safety.get("ai_max_V", cfg.ai_max_V))
        cfg.max_raw_ao_test_V = float(safety.get("max_raw_ao_test_V", cfg.max_raw_ao_test_V))
        opts = data.get("user_options", {}) or {}
        cfg.source_ramp_increment_V = float(opts.get("source_ramp_increment_V", cfg.source_ramp_increment_V))
        cfg.source_ramp_delay_s = float(opts.get("source_ramp_delay_s", cfg.source_ramp_delay_s))
        cfg.meter_samples_default = int(opts.get("meter_samples_default", cfg.meter_samples_default))
        cfg.lut_path = str(data.get("lut_path", cfg.lut_path))

        cfg.meters = {}
        for name, m in (data.get("meters", {}) or {}).items():
            device = str(m.get("device", cfg.metering_device))
            channel = str(m["channel"])
            cfg.meters[name] = MeterChannel(
                name=name,
                device=device,
                channel=channel,
                engineering_max=float(m.get("engineering_max", m.get("scale_max", 1.0))),
                input_range_V=float(m.get("input_range_V", 2.0)),
                units=str(m.get("units", "")),
                bipolar=bool(m.get("bipolar", False)),
            )

        cfg.controls = {}
        for name, c in (data.get("controls", {}) or {}).items():
            device = str(c.get("device", cfg.control_device))
            channel = str(c["channel"])
            cfg.controls[name] = ControlChannel(
                name=name,
                device=device,
                channel=channel,
                engineering_max=float(c.get("engineering_max", c.get("scale_max", 1.0))),
                output_range_V=float(c.get("output_range_V", 10.0)),
                units=str(c.get("units", "")),
                coarse_increment=None if c.get("coarse_increment") is None else float(c.get("coarse_increment")),
                fine_increment=None if c.get("fine_increment") is None else float(c.get("fine_increment")),
                bipolar=bool(c.get("bipolar", False)),
            )

        cfg.digital_controls = {}
        for name, d in (data.get("digital_controls", {}) or {}).items():
            cfg.digital_controls[name] = DigitalControl(
                name=name,
                legacy_line=None if d.get("legacy_line") is None else str(d.get("legacy_line")),
                daqmx_line=None if d.get("daqmx_line") in (None, "", "null") else str(d.get("daqmx_line")),
                note=str(d.get("note", "")),
                on_value=bool(d.get("on_value", True)),
                off_value=bool(d.get("off_value", False)),
            )
        return cfg


class KimballDAQ:
    def __init__(self, config: KimballDAQConfig) -> None:
        self.config = config

    @classmethod
    def from_yaml(cls, path: str | Path, *, simulate_override: bool | None = None) -> "KimballDAQ":
        return cls(KimballDAQConfig.from_yaml(path, simulate_override=simulate_override))

    @property
    def simulate(self) -> bool:
        return bool(self.config.simulate)

    def list_devices(self) -> list[dict[str, Any]]:
        if self.simulate:
            return [
                {"name": self.config.metering_device, "product_type": "SIM PXIe-6341 metering", "serial_num": "SIM"},
                {"name": self.config.control_device, "product_type": "SIM PXI-6733 control", "serial_num": "SIM"},
            ]
        _nidaqmx, System = _import_nidaqmx()
        system = System.local()
        rows = []
        for dev in system.devices:
            rows.append({
                "name": dev.name,
                "product_type": getattr(dev, "product_type", ""),
                "serial_num": getattr(dev, "dev_serial_num", ""),
            })
        return rows

    def list_channels(self, device_name: str) -> dict[str, list[str]]:
        if self.simulate:
            ai = [m.physical_channel for m in self.config.meters.values() if m.device == device_name]
            ao = [c.physical_channel for c in self.config.controls.values() if c.device == device_name]
            do = [d.daqmx_line for d in self.config.digital_controls.values() if d.daqmx_line]
            return {"ai": ai, "ao": ao, "do": [x for x in do if x]}
        _nidaqmx, System = _import_nidaqmx()
        dev = System.local().devices[device_name]

        def names(collection: Any) -> list[str]:
            try:
                return [ch.name for ch in collection]
            except Exception:
                return [str(ch) for ch in collection]

        return {
            "ai": names(getattr(dev, "ai_physical_chans", [])),
            "ao": names(getattr(dev, "ao_physical_chans", [])),
            "di": names(getattr(dev, "di_lines", [])),
            "do": names(getattr(dev, "do_lines", [])),
            "ci": names(getattr(dev, "ci_physical_chans", [])),
            "co": names(getattr(dev, "co_physical_chans", [])),
        }

    def resolve_lut_path(self) -> Path:
        path = Path(self.config.lut_path)
        if path.is_absolute():
            return path
        # Relative LUT paths are resolved relative to this module's package config directory
        # when using the packaged default config.
        return Path(__file__).resolve().parents[1] / "config" / path

    def load_lut(self, path: str | Path | None = None) -> list[KimballLUTRow]:
        lut_path = Path(path) if path is not None else self.resolve_lut_path()
        rows: list[KimballLUTRow] = []
        with open(lut_path, "r", encoding="utf-8-sig", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                if not row:
                    continue
                energy_key = "Energy" if "Energy" in row else "energy_eV"
                x_key = "X" if "X" in row else "x_center_V"
                y_key = "Y" if "Y" in row else "y_center_V"
                try:
                    rows.append(KimballLUTRow(
                        energy_eV=float(row[energy_key]),
                        x_center_V=float(row[x_key]),
                        y_center_V=float(row[y_key]),
                    ))
                except Exception:
                    continue
        rows.sort(key=lambda r: r.energy_eV)
        if not rows:
            raise RuntimeError(f"No Kimball LUT rows found in {lut_path}")
        return rows

    def lut_for_energy(self, energy_eV: float, *, path: str | Path | None = None) -> dict[str, Any]:
        """Return exact or linearly interpolated X/Y center values for an energy in eV."""
        energy_eV = float(energy_eV)
        rows = self.load_lut(path)
        if energy_eV <= rows[0].energy_eV:
            r = rows[0]
            return {"energy_eV": energy_eV, "x_center_V": r.x_center_V, "y_center_V": r.y_center_V, "source": "clamped_low", "lut_energy_eV": r.energy_eV}
        if energy_eV >= rows[-1].energy_eV:
            r = rows[-1]
            return {"energy_eV": energy_eV, "x_center_V": r.x_center_V, "y_center_V": r.y_center_V, "source": "clamped_high", "lut_energy_eV": r.energy_eV}
        for lo, hi in zip(rows[:-1], rows[1:]):
            if lo.energy_eV <= energy_eV <= hi.energy_eV:
                if hi.energy_eV == lo.energy_eV:
                    t = 0.0
                else:
                    t = (energy_eV - lo.energy_eV) / (hi.energy_eV - lo.energy_eV)
                return {
                    "energy_eV": energy_eV,
                    "x_center_V": lo.x_center_V + t * (hi.x_center_V - lo.x_center_V),
                    "y_center_V": lo.y_center_V + t * (hi.y_center_V - lo.y_center_V),
                    "source": "exact" if abs(t) < 1e-12 or abs(t - 1.0) < 1e-12 else "interpolated",
                    "bracket_eV": [lo.energy_eV, hi.energy_eV],
                }
        raise RuntimeError("Internal LUT interpolation error")

    @staticmethod
    def suggested_grid_V_for_energy(energy_eV: float, fraction: float = 0.025) -> float:
        # Kimball support guidance: Grid should be roughly 2–3% of Energy
        # before heating the cathode. Energy in eV corresponds numerically to volts.
        return float(energy_eV) * float(fraction)

    def make_electrostatic_target_from_lut(
        self,
        energy_eV: float,
        *,
        grid_V: float | None = None,
        focus_kV: float | None = None,
        apply_grid: bool = True,
        apply_focus: bool = False,
        apply_energy: bool = True,
        apply_deflection: bool = True,
    ) -> KimballElectrostaticTarget:
        lut = self.lut_for_energy(energy_eV)
        if grid_V is None:
            grid_V = self.suggested_grid_V_for_energy(energy_eV)
        return KimballElectrostaticTarget(
            energy_eV=float(energy_eV),
            grid_V=grid_V,
            focus_kV=focus_kV,
            x_center_V=float(lut["x_center_V"]),
            y_center_V=float(lut["y_center_V"]),
            apply_grid=apply_grid,
            apply_focus=apply_focus,
            apply_energy=apply_energy,
            apply_deflection=apply_deflection,
        )

    def current_energy_eV_from_meters(self) -> float | None:
        try:
            meters = self.read_meters(samples=max(1, min(20, self.config.meter_samples_default)))
            if "energy" in meters:
                return float(meters["energy"].get("value", 0.0)) * 1000.0
        except Exception:
            return None
        return None

    def write_controls_engineering_ordered(
        self,
        sequence: list[tuple[str, float]],
        *,
        token: str | None = None,
        allow_large: bool = True,
        delay_s: float = 0.2,
    ) -> list[dict[str, Any]]:
        results: list[dict[str, Any]] = []
        for name, value in sequence:
            results.append(self.write_control_engineering(name, value, token=token, allow_large=allow_large))
            if delay_s > 0:
                time.sleep(float(delay_s))
        return results

    def write_digital_control(
        self,
        control_name: str,
        state: bool,
        *,
        token: str | None = None,
    ) -> dict[str, Any]:
        """Write a configured digital control line.

        Digital controls are safety-relevant for the Kimball gun.  The config
        must contain the exact DAQmx line; legacy LabVIEW names such as L1 are
        kept only as metadata.
        """
        if token != GUN_DAQ_TOKEN:
            raise RuntimeError("Refusing Kimball digital write: pass the explicit safety token.")
        if control_name not in self.config.digital_controls:
            raise KeyError(f"Unknown digital control {control_name!r}. Known: {list(self.config.digital_controls)}")
        d = self.config.digital_controls[control_name]
        if not d.daqmx_line:
            raise RuntimeError(
                f"Digital control {control_name!r} has no DAQmx line configured "
                f"(legacy line {d.legacy_line!r}). Configure it before using automatic deflection writes."
            )
        value = bool(state)
        if self.simulate:
            return {
                "control": control_name,
                "legacy_line": d.legacy_line,
                "daqmx_line": d.daqmx_line,
                "value": value,
                "action": "simulated digital write",
            }
        nidaqmx, _System = _import_nidaqmx()
        with nidaqmx.Task() as task:
            task.do_channels.add_do_chan(d.daqmx_line)
            task.write(value, auto_start=True)
            time.sleep(0.05)
        return {
            "control": control_name,
            "legacy_line": d.legacy_line,
            "daqmx_line": d.daqmx_line,
            "value": value,
            "action": "wrote digital line",
        }

    def set_deflection_switch(self, enabled: bool, *, token: str | None = None) -> dict[str, Any]:
        """Turn the Kimball deflection switch ON/OFF."""
        d = self.config.digital_controls.get("deflection_switch")
        if d is None:
            raise RuntimeError("No deflection_switch digital control is configured.")
        value = d.on_value if bool(enabled) else d.off_value
        result = self.write_digital_control("deflection_switch", bool(value), token=token)
        result["enabled"] = bool(enabled)
        result["note"] = "Deflection must be ON before X/Y deflection voltages are applied."
        return result

    def ensure_deflection_enabled(self, *, token: str | None = None) -> dict[str, Any]:
        """Turn deflection ON before any automatic X/Y write."""
        return self.set_deflection_switch(True, token=token)

    def apply_electrostatic_target(
        self,
        target: KimballElectrostaticTarget,
        *,
        token: str | None = None,
        current_energy_eV: float | None = None,
        delay_s: float = 0.2,
    ) -> dict[str, Any]:
        """Apply Energy/Grid/Focus/X/Y only; never touches Source/ECC.

        Ordering follows Kimball support guidance:
        - increasing energy: Grid -> Focus -> Energy;
        - decreasing energy: Energy -> Focus -> Grid.
        Deflection center controls are applied after the electrostatic energy sequence.
        """
        if token != GUN_DAQ_TOKEN:
            raise RuntimeError("Refusing Kimball electrostatic write: pass the explicit safety token.")
        if current_energy_eV is None:
            current_energy_eV = self.current_energy_eV_from_meters()
        increasing = True if current_energy_eV is None else (float(target.energy_eV) >= float(current_energy_eV))

        pre: list[tuple[str, float]] = []
        post: list[tuple[str, float]] = []
        energy_pair = ("energy", target.energy_keV)
        grid_pair = ("grid", float(target.grid_V)) if (target.apply_grid and target.grid_V is not None) else None
        focus_pair = ("focus", float(target.focus_kV)) if (target.apply_focus and target.focus_kV is not None) else None

        if increasing:
            if grid_pair: pre.append(grid_pair)
            if focus_pair: pre.append(focus_pair)
            if target.apply_energy: pre.append(energy_pair)
        else:
            if target.apply_energy: pre.append(energy_pair)
            if focus_pair: pre.append(focus_pair)
            if grid_pair: pre.append(grid_pair)

        deflection_switch_result = None
        if target.apply_deflection:
            if target.x_center_V is not None:
                post.append(("x_center", float(target.x_center_V)))
            if target.y_center_V is not None:
                post.append(("y_center", float(target.y_center_V)))
            if post:
                deflection_switch_result = self.ensure_deflection_enabled(token=token)

        sequence = pre + post
        results = self.write_controls_engineering_ordered(sequence, token=token, allow_large=True, delay_s=delay_s)
        return {
            "target": {
                "energy_eV": target.energy_eV,
                "energy_keV": target.energy_keV,
                "grid_V": target.grid_V,
                "focus_kV": target.focus_kV,
                "x_center_V": target.x_center_V,
                "y_center_V": target.y_center_V,
            },
            "current_energy_eV_before": current_energy_eV,
            "direction": "increasing_or_equal" if increasing else "decreasing",
            "sequence": sequence,
            "deflection_switch": deflection_switch_result,
            "writes": results,
            "note": "Source/ECC not touched by electrostatic apply. If X/Y deflection was applied, the deflection switch was turned ON first.",
        }

    def _source_control_name(self) -> str:
        if "source_ecc_source_mode" in self.config.controls:
            return "source_ecc_source_mode"
        if "source" in self.config.controls:
            return "source"
        raise KeyError("No Source-mode control is configured. Expected 'source_ecc_source_mode'.")

    def current_source_voltage_from_meters(self) -> float | None:
        try:
            meters = self.read_meters(samples=max(1, min(20, self.config.meter_samples_default)))
            if "source_volts" in meters:
                return float(meters["source_volts"].get("value", 0.0))
        except Exception:
            return None
        return None

    def write_source_raw_ao_test(
        self,
        ao_voltage_V: float,
        *,
        token: str | None = None,
        allow_large: bool = False,
        read_meters: bool = True,
        samples: int | None = None,
    ) -> dict[str, Any]:
        """Write the Source/ECC Source-mode physical AO channel directly.

        This bypasses LabVIEW-style engineering scaling and is intended only
        for cautious front-panel calibration, e.g. AO1 = 0.02, 0.05, 0.10 V.
        The raw-AO safety limit in config applies unless allow_large=True.
        """
        if token != GUN_DAQ_TOKEN:
            raise RuntimeError("Refusing Source raw-AO test: pass the explicit Kimball gun DAQ safety token.")
        control_name = self._source_control_name()
        ctrl = self.config.controls[control_name]
        ao_voltage_V = float(ao_voltage_V)
        written = self.write_raw_ao(ctrl.physical_channel, ao_voltage_V, token=token, allow_large=allow_large)
        return {
            "action": "source_raw_ao_test",
            "control": control_name,
            "channel": ctrl.physical_channel,
            "ao_voltage_V": written,
            "engineering_scaling_bypassed": True,
            "meters": self.read_meters(samples=samples) if read_meters else {},
            "note": "Direct AO calibration write only. Compare Kimball front-panel Source display manually, then use Zero Source AO.",
        }

    def apply_source_warmup_electrostatics(
        self,
        *,
        energy_eV: float = 1000.0,
        grid_V: float = 500.0,
        token: str | None = None,
        delay_s: float = 0.2,
        read_meters: bool = True,
        samples: int | None = None,
    ) -> dict[str, Any]:
        """Apply the conservative Kimball warm-up precondition before Source ramp.

        For cathode/source warm-up we intentionally use the vendor-style order
        requested for this setup: set Energy first, then set Grid/G1, and only
        after that ramp the Source voltage. Focus and X/Y deflection are not
        touched by this helper.
        """
        if token != GUN_DAQ_TOKEN:
            raise RuntimeError("Refusing Kimball warm-up write: pass the explicit safety token.")
        energy_eV = float(energy_eV)
        grid_V = float(grid_V)
        if energy_eV <= 0:
            raise RuntimeError("Warm-up energy must be positive.")
        sequence = [("energy", energy_eV / 1000.0), ("grid", grid_V)]
        writes = self.write_controls_engineering_ordered(sequence, token=token, allow_large=True, delay_s=delay_s)
        return {
            "action": "source_warmup_electrostatics",
            "energy_eV": energy_eV,
            "energy_keV": energy_eV / 1000.0,
            "grid_V": grid_V,
            "sequence": sequence,
            "writes": writes,
            "meters": self.read_meters(samples=samples) if read_meters else {},
            "note": "Warm-up precondition only: Energy then Grid. Source ramp is separate; Focus and X/Y are not touched.",
        }

    def ramp_source_voltage(
        self,
        target_source_V: float,
        *,
        token: str | None = None,
        start_source_V: float | None = None,
        step_V: float | None = None,
        delay_s: float | None = None,
        read_meters_each_step: bool = True,
        samples: int | None = None,
        abort_callback: Any | None = None,
        event_callback: Any | None = None,
        max_emission_uA: float | None = None,
        max_source_amps_A: float | None = None,
        source_meter_track_tolerance_V: float | None = None,
    ) -> dict[str, Any]:
        """Ramp Source/ECC control in Source mode only.

        This writes the Source-mode AO control (`source_ecc_source_mode`) in small
        engineering-unit voltage increments. ECC mode is intentionally not used.

        v0.19.13: after every Source AO write, wait `delay_s` before reading
        meters or continuing.  Direct AO tests showed the static scale is correct
        (Source display ~= AO1/2), but Source voltage/current take several seconds
        to settle, so immediate readback is misleading and the next step must not
        start too soon.
        """
        if token != GUN_DAQ_TOKEN:
            raise RuntimeError("Refusing Source ramp: pass the explicit Kimball gun DAQ safety token.")
        control_name = self._source_control_name()
        ctrl = self.config.controls[control_name]
        target_source_V = float(target_source_V)
        configured_max = float(ctrl.engineering_max)
        cautious_target_max = min(configured_max, 3.0)
        if target_source_V < 0 or target_source_V > cautious_target_max:
            raise RuntimeError(
                f"Refusing Source target {target_source_V:g} V; cautious allowed range is 0..{cautious_target_max:g} V "
                f"(AO scaling full-scale is {configured_max:g} V)."
            )
        step_V = abs(float(step_V if step_V is not None else self.config.source_ramp_increment_V))
        delay_s = float(delay_s if delay_s is not None else self.config.source_ramp_delay_s)
        if step_V <= 0:
            raise RuntimeError("Source ramp step must be positive.")
        if step_V > 0.10:
            raise RuntimeError("Refusing Source ramp step > 0.10 V. Use 0.02-0.05 V for cautious startup.")
        if delay_s < 5.0:
            raise RuntimeError("Refusing Source ramp delay < 5 s. Use about 10 s because Source voltage/current settle slowly.")

        if start_source_V is None:
            if target_source_V > 0:
                start_source_V = 0.0
            else:
                raise RuntimeError(
                    "Source ramp-down requires an explicit start_source_V because "
                    "the Source Volts meter is diagnostic and may lag the command."
                )
        start_source_V = float(start_source_V)
        if abs(start_source_V) < 1e-3:
            start_source_V = 0.0
        if start_source_V < 0 and target_source_V >= 0:
            start_source_V = 0.0

        direction = 1.0 if target_source_V >= start_source_V else -1.0
        values: list[float] = []
        v = start_source_V
        while (direction > 0 and v < target_source_V) or (direction < 0 and v > target_source_V):
            v = v + direction * step_V
            v = min(v, target_source_V) if direction > 0 else max(v, target_source_V)
            if not values or abs(v - values[-1]) > 1e-12:
                values.append(round(v, 10))

        steps: list[dict[str, Any]] = []
        warnings: list[str] = []
        t0 = time.time()
        aborted = False
        safety_stop_reason: str | None = None
        abort_zero_written: dict[str, Any] | None = None

        def _write_source_zero_for_abort() -> None:
            nonlocal abort_zero_written
            if abort_zero_written is None:
                try:
                    abort_zero_written = self.write_control_engineering(control_name, 0.0, token=token, allow_large=True)
                except Exception as exc:
                    abort_zero_written = {"error": str(exc)}

        def _wait_after_write() -> tuple[bool, float]:
            """Return (aborted, actual_wait_s)."""
            if delay_s <= 0:
                return False, 0.0
            wait_start = time.time()
            end = wait_start + delay_s
            while time.time() < end:
                if abort_callback is not None and bool(abort_callback()):
                    return True, time.time() - wait_start
                time.sleep(min(0.05, max(0.0, end - time.time())))
            return False, time.time() - wait_start

        for i, value in enumerate(values, start=1):
            if abort_callback is not None and bool(abort_callback()):
                aborted = True
                safety_stop_reason = "user_abort"
                _write_source_zero_for_abort()
                break

            written = self.write_control_engineering(control_name, value, token=token, allow_large=True)
            wait_aborted, actual_wait_s = _wait_after_write()
            if wait_aborted:
                aborted = True
                safety_stop_reason = "user_abort"
                _write_source_zero_for_abort()

            meters = self.read_meters(samples=samples) if read_meters_each_step else {}

            if source_meter_track_tolerance_V is not None and meters:
                try:
                    source_meter = float(meters.get("source_volts", {}).get("value"))
                    if abs(source_meter - float(value)) > float(source_meter_track_tolerance_V):
                        warnings.append(
                            f"step {i}: Source meter {source_meter:g} V differs from command {value:g} V "
                            f"by more than {source_meter_track_tolerance_V:g} V"
                        )
                except Exception:
                    pass

            if meters:
                try:
                    emission = float(meters.get("emission", {}).get("value"))
                    if max_emission_uA is not None and emission > float(max_emission_uA):
                        aborted = True
                        safety_stop_reason = f"emission {emission:g} uA exceeded limit {float(max_emission_uA):g} uA"
                except Exception:
                    pass
                try:
                    source_amps = float(meters.get("source_amps", {}).get("value"))
                    if max_source_amps_A is not None and source_amps > float(max_source_amps_A):
                        aborted = True
                        safety_stop_reason = f"source current {source_amps:g} A exceeded limit {float(max_source_amps_A):g} A"
                except Exception:
                    pass

            if aborted and safety_stop_reason != "user_abort":
                _write_source_zero_for_abort()

            step_row = {
                "index": i,
                "target_source_V": value,
                "written": written,
                "expected_source_display_V": value,
                "expected_ao_voltage_V": written.get("ao_voltage_V"),
                "settle_wait_s": actual_wait_s,
                "meters": meters,
                "meters_after_wait": meters,
                "timestamp_s": time.time() - t0,
                "safety_stop_reason": safety_stop_reason,
            }
            steps.append(step_row)
            if event_callback is not None:
                try:
                    event_callback(step_row)
                except Exception:
                    pass
            if aborted:
                break

        final_meters = self.read_meters(samples=samples) if read_meters_each_step else {}
        return {
            "mode": "source_mode_only",
            "control": control_name,
            "target_source_V": target_source_V,
            "start_source_V": start_source_V,
            "step_V": step_V,
            "delay_s": delay_s,
            "direction": "up" if direction > 0 else "down",
            "aborted": aborted,
            "safety_stop_reason": safety_stop_reason,
            "warnings": warnings,
            "abort_zero_written": abort_zero_written,
            "source_scaling": {
                "engineering_max_source_V": configured_max,
                "ao_full_scale_V": float(ctrl.output_range_V),
                "formula": "AO1 = Source_target_V / engineering_max_source_V * ao_full_scale_V",
                "measured_check": "Direct AO tests gave Source display approximately AO1/2, consistent with AO1 = 2*Source_target_V.",
            },
            "safety_limits": {
                "max_emission_uA": max_emission_uA,
                "max_source_amps_A": max_source_amps_A,
                "source_volts_read_after_settle": True,
                "source_meter_track_tolerance_V": source_meter_track_tolerance_V,
            },
            "steps": steps,
            "final_meters": final_meters,
            "note": "ECC mode is not used. Each Source step waits before meter readback and before the next step. Static Source scaling follows Output Device&Channel.txt: AO1 = 2*Source_target_V.",
        }


    def set_source_voltage(
        self,
        target_source_V: float,
        *,
        token: str | None = None,
        start_source_V: float | None = None,
        step_V: float | None = None,
        delay_s: float | None = None,
        read_meters_each_step: bool = True,
        samples: int | None = None,
        abort_callback: Any | None = None,
        event_callback: Any | None = None,
        max_emission_uA: float | None = None,
        max_source_amps_A: float | None = None,
    ) -> dict[str, Any]:
        """Adjust Source voltage only; do not touch Energy/Grid/Focus/X/Y.

        This is for normal operation after conditioning, e.g. lowering Source
        from the high conditioning value to a safer operating value.  If
        start_source_V is not supplied, the current Source Volts meter is read
        and used as the ramp start.  That avoids a dangerous jump from the
        previous Source value to a low assumed starting value.
        """
        if token != GUN_DAQ_TOKEN:
            raise RuntimeError("Refusing Source set: pass the explicit Kimball gun DAQ safety token.")
        initial_meters: dict[str, dict[str, Any]] = {}
        inferred_start_source_V: float | None = None
        if start_source_V is None:
            initial_meters = self.read_meters(samples=samples)
            try:
                inferred_start_source_V = float(initial_meters.get("source_volts", {}).get("value"))
            except Exception as exc:
                raise RuntimeError("Could not infer current Source voltage from Source Volts meter; enter Start Source manually.") from exc
            if inferred_start_source_V < -0.02:
                raise RuntimeError(
                    f"Source Volts meter read {inferred_start_source_V:g} V. Enter Start Source manually if this is expected."
                )
            start_source_V = max(0.0, inferred_start_source_V)
        ramp = self.ramp_source_voltage(
            target_source_V,
            token=token,
            start_source_V=start_source_V,
            step_V=step_V,
            delay_s=delay_s,
            read_meters_each_step=read_meters_each_step,
            samples=samples,
            abort_callback=abort_callback,
            event_callback=event_callback,
            max_emission_uA=max_emission_uA,
            max_source_amps_A=max_source_amps_A,
            source_meter_track_tolerance_V=None,
        )
        return {
            "action": "set_source_voltage_only",
            "target_source_V": float(target_source_V),
            "start_source_V": float(start_source_V),
            "inferred_start_source_V": inferred_start_source_V,
            "initial_meters": initial_meters,
            "ramp": ramp,
            "note": "Only Source-mode AO was changed. Energy/Grid/Focus/X/Y and ECC mode were not touched.",
        }

    def ramp_source_down_to_zero(
        self,
        *,
        token: str | None = None,
        start_source_V: float | None = None,
        step_V: float | None = None,
        delay_s: float | None = None,
        read_meters_each_step: bool = True,
        samples: int | None = None,
        abort_callback: Any | None = None,
        event_callback: Any | None = None,
        max_emission_uA: float | None = None,
        max_source_amps_A: float | None = None,
    ) -> dict[str, Any]:
        """Normal source turn-off: ramp Source-mode control down to 0 V."""
        return self.ramp_source_voltage(
            0.0,
            token=token,
            start_source_V=start_source_V,
            step_V=step_V,
            delay_s=delay_s,
            read_meters_each_step=read_meters_each_step,
            samples=samples,
            abort_callback=abort_callback,
            event_callback=event_callback,
            max_emission_uA=max_emission_uA,
            max_source_amps_A=max_source_amps_A,
            # Source Volts is diagnostic only until calibrated; do not warn on tracking.
            source_meter_track_tolerance_V=None,
        )

    def zero_source_only(
        self,
        *,
        token: str | None = None,
        read_meters: bool = True,
        samples: int | None = None,
        wait_after_s: float = 10.0,
    ) -> dict[str, Any]:
        """Write only the Source-mode AO control to 0 V; leave electrostatics unchanged.

        The Kimball Source supply does not follow the AO command instantly; it
        ramps/decays over several seconds.  For safety and diagnostics, the
        default behavior reads meters immediately, waits, and reads again.
        """
        if token != GUN_DAQ_TOKEN:
            raise RuntimeError("Refusing to zero Source: pass the explicit Kimball gun DAQ safety token.")
        control_name = self._source_control_name()
        written = self.write_control_engineering(control_name, 0.0, token=token, allow_large=True)
        meters_immediate = self.read_meters(samples=samples) if read_meters else {}
        wait_after_s = max(0.0, float(wait_after_s))
        meters_after_wait: dict[str, dict[str, Any]] = {}
        if read_meters and wait_after_s > 0:
            time.sleep(wait_after_s)
            meters_after_wait = self.read_meters(samples=samples)
        return {
            "action": "zero_source_only",
            "written": written,
            "meters_immediate": meters_immediate,
            "wait_after_s": wait_after_s,
            "meters_after_wait": meters_after_wait,
            "note": "Only Source/ECC Source-mode AO was zeroed; Energy/Grid/Focus/X/Y were left unchanged. Source voltage/current can decay slowly, so meters are re-read after the wait.",
        }


    def hold_source_current_condition(
        self,
        *,
        hold_s: float,
        token: str | None = None,
        read_interval_s: float = 10.0,
        samples: int | None = None,
        abort_callback: Any | None = None,
        event_callback: Any | None = None,
    ) -> dict[str, Any]:
        """Hold the present Source setting and periodically read meters."""
        if token != GUN_DAQ_TOKEN:
            raise RuntimeError("Refusing Source hold: pass the explicit Kimball gun DAQ safety token.")
        hold_s = max(0.0, float(hold_s))
        read_interval_s = max(0.5, float(read_interval_s))
        t0 = time.time()
        reads: list[dict[str, Any]] = []
        aborted = False
        i = 0
        while time.time() - t0 < hold_s:
            if abort_callback is not None and bool(abort_callback()):
                aborted = True
                break
            meters = self.read_meters(samples=samples)
            i += 1
            row = {"index": i, "elapsed_s": time.time() - t0, "meters": meters}
            reads.append(row)
            if event_callback is not None:
                try:
                    event_callback({"hold": True, **row})
                except Exception:
                    pass
            end = min(t0 + hold_s, time.time() + read_interval_s)
            while time.time() < end:
                if abort_callback is not None and bool(abort_callback()):
                    aborted = True
                    break
                time.sleep(min(0.05, max(0.0, end - time.time())))
            if aborted:
                break
        return {"action": "hold_source_condition", "hold_s": hold_s, "read_interval_s": read_interval_s, "aborted": aborted, "reads": reads}

    def ramp_source_until_current(
        self,
        target_source_amps_A: float,
        *,
        token: str | None = None,
        start_source_V: float = 0.0,
        max_source_V: float = 3.0,
        step_V: float | None = None,
        delay_s: float | None = None,
        read_meters_each_step: bool = True,
        samples: int | None = None,
        abort_callback: Any | None = None,
        event_callback: Any | None = None,
        max_emission_uA: float | None = None,
    ) -> dict[str, Any]:
        """Increase Source voltage in small steps until Source current reaches a target.

        This is for post-bake conditioning where the operator targets Source
        current (for example 0.8 A and then 1.7 A).  After each Source AO write,
        wait `delay_s` before reading Source current, because the Source voltage
        and current take several seconds to settle.
        """
        if token != GUN_DAQ_TOKEN:
            raise RuntimeError("Refusing Source current-target warm-up: pass the explicit Kimball gun DAQ safety token.")
        target_source_amps_A = float(target_source_amps_A)
        start_source_V = max(0.0, float(start_source_V))
        max_source_V = float(max_source_V)
        if target_source_amps_A <= 0:
            raise RuntimeError("Target Source current must be positive.")
        if max_source_V <= start_source_V:
            raise RuntimeError("max_source_V must be above start_source_V.")
        step_V = abs(float(step_V if step_V is not None else self.config.source_ramp_increment_V))
        delay_s = float(delay_s if delay_s is not None else self.config.source_ramp_delay_s)
        if step_V <= 0 or step_V > 0.10:
            raise RuntimeError("Source current-target warm-up requires 0 < step <= 0.10 V.")
        if delay_s < 5.0:
            raise RuntimeError("Refusing Source current-target warm-up delay < 5 s. Use about 10 s because Source voltage/current settle slowly.")

        control_name = self._source_control_name()
        steps: list[dict[str, Any]] = []
        warnings: list[str] = []
        aborted = False
        safety_stop_reason: str | None = None
        reached = False
        v = start_source_V
        t0 = time.time()
        i = 0

        def _zero_for_stop(reason: str) -> None:
            nonlocal safety_stop_reason
            safety_stop_reason = reason
            try:
                self.zero_source_only(token=token, read_meters=False)
            except Exception as exc:
                warnings.append(f"zero Source failed after {reason}: {exc}")

        def _wait_after_write() -> tuple[bool, float]:
            wait_start = time.time()
            end = wait_start + delay_s
            while time.time() < end:
                if abort_callback is not None and bool(abort_callback()):
                    return True, time.time() - wait_start
                time.sleep(min(0.05, max(0.0, end - time.time())))
            return False, time.time() - wait_start

        while v < max_source_V - 1e-12:
            if abort_callback is not None and bool(abort_callback()):
                aborted = True
                _zero_for_stop("user_abort")
                break

            v = min(max_source_V, v + step_V)
            i += 1
            written = self.write_control_engineering(control_name, v, token=token, allow_large=True)
            wait_aborted, actual_wait_s = _wait_after_write()
            if wait_aborted:
                aborted = True
                _zero_for_stop("user_abort")

            meters = self.read_meters(samples=samples) if read_meters_each_step else {}
            source_amps = None
            emission = None
            if meters:
                try:
                    source_amps = float(meters.get("source_amps", {}).get("value"))
                    if source_amps >= target_source_amps_A:
                        reached = True
                        safety_stop_reason = f"target source current reached: {source_amps:g} A >= {target_source_amps_A:g} A"
                except Exception:
                    pass
                try:
                    emission = float(meters.get("emission", {}).get("value"))
                    if max_emission_uA is not None and emission > float(max_emission_uA):
                        aborted = True
                        _zero_for_stop(f"emission {emission:g} uA exceeded limit {float(max_emission_uA):g} uA")
                except Exception:
                    pass

            step_row = {
                "index": i,
                "target_source_V": v,
                "written": written,
                "expected_ao_voltage_V": written.get("ao_voltage_V"),
                "settle_wait_s": actual_wait_s,
                "meters": meters,
                "meters_after_wait": meters,
                "source_amps_A": source_amps,
                "emission_uA": emission,
                "timestamp_s": time.time() - t0,
                "safety_stop_reason": safety_stop_reason,
            }
            steps.append(step_row)
            if event_callback is not None:
                try:
                    event_callback(step_row)
                except Exception:
                    pass
            if reached or aborted:
                break

        final_meters = self.read_meters(samples=samples) if read_meters_each_step else {}
        return {
            "action": "ramp_source_until_current",
            "target_source_amps_A": target_source_amps_A,
            "start_source_V": start_source_V,
            "max_source_V": max_source_V,
            "step_V": step_V,
            "delay_s": delay_s,
            "reached": reached,
            "aborted": aborted,
            "safety_stop_reason": safety_stop_reason,
            "warnings": warnings,
            "steps": steps,
            "final_meters": final_meters,
            "note": "Post-bake conditioning helper. ECC mode is not used. Each Source step waits before Source-current readback. Stop criterion is Source current meter.",
        }

    def post_bake_conditioning_warmup(
        self,
        *,
        token: str | None = None,
        energy_eV: float = 1000.0,
        grid_V: float = 500.0,
        first_current_A: float = 0.8,
        first_hold_s: float = 600.0,
        second_current_A: float = 1.7,
        max_source_V: float = 3.0,
        step_V: float | None = None,
        delay_s: float | None = None,
        hold_read_interval_s: float = 10.0,
        samples: int | None = None,
        abort_callback: Any | None = None,
        event_callback: Any | None = None,
        max_emission_uA: float | None = None,
    ) -> dict[str, Any]:
        """One-time post-bake conditioning warm-up.

        Sequence requested for this RFA gun setup:
        1) set Energy and Grid/cutoff;
        2) ramp Source until Source current reaches first_current_A;
        3) hold for first_hold_s;
        4) ramp Source further until Source current reaches second_current_A.
        """
        if token != GUN_DAQ_TOKEN:
            raise RuntimeError("Refusing post-bake conditioning: pass the explicit Kimball gun DAQ safety token.")
        prebias = self.apply_source_warmup_electrostatics(
            energy_eV=energy_eV, grid_V=grid_V, token=token, delay_s=0.2, read_meters=True, samples=samples
        )
        ramp1 = self.ramp_source_until_current(
            first_current_A,
            token=token,
            start_source_V=0.0,
            max_source_V=max_source_V,
            step_V=step_V,
            delay_s=delay_s,
            samples=samples,
            abort_callback=abort_callback,
            event_callback=event_callback,
            max_emission_uA=max_emission_uA,
        )
        if ramp1.get("aborted") or not ramp1.get("reached"):
            return {"action": "post_bake_conditioning_warmup", "prebias": prebias, "ramp_to_first_current": ramp1, "aborted": True, "note": "Stopped before hold/second current."}
        hold = self.hold_source_current_condition(
            hold_s=first_hold_s,
            token=token,
            read_interval_s=hold_read_interval_s,
            samples=samples,
            abort_callback=abort_callback,
            event_callback=event_callback,
        )
        if hold.get("aborted"):
            try:
                self.zero_source_only(token=token, read_meters=False)
            except Exception:
                pass
            return {"action": "post_bake_conditioning_warmup", "prebias": prebias, "ramp_to_first_current": ramp1, "hold_first_current": hold, "aborted": True}
        last_v = 0.0
        try:
            last_steps = ramp1.get("steps") or []
            if last_steps:
                last_v = float(last_steps[-1].get("target_source_V", 0.0))
        except Exception:
            last_v = 0.0
        ramp2 = self.ramp_source_until_current(
            second_current_A,
            token=token,
            start_source_V=last_v,
            max_source_V=max_source_V,
            step_V=step_V,
            delay_s=delay_s,
            samples=samples,
            abort_callback=abort_callback,
            event_callback=event_callback,
            max_emission_uA=max_emission_uA,
        )
        return {
            "action": "post_bake_conditioning_warmup",
            "prebias": prebias,
            "ramp_to_first_current": ramp1,
            "hold_first_current": hold,
            "ramp_to_second_current": ramp2,
            "aborted": bool(ramp2.get("aborted")),
            "note": "One-time post-bake conditioning sequence. Use normal Source shutdown when done.",
        }

    def normal_source_shutdown(
        self,
        *,
        token: str | None = None,
        start_source_V: float | None = None,
        step_V: float | None = None,
        delay_s: float | None = None,
        read_meters_each_step: bool = True,
        samples: int | None = None,
        abort_callback: Any | None = None,
        event_callback: Any | None = None,
        final_settle_s: float = 10.0,
        final_read_interval_s: float = 2.0,
    ) -> dict[str, Any]:
        """Normal shutdown for the gun source: ramp down, then write Source AO = 0.

        This intentionally does *not* zero Energy/Grid/Focus/X/Y.  Use
        zero_all_outputs only as the all-safe/emergency/end-of-day command.
        """
        ramp = self.ramp_source_down_to_zero(
            token=token,
            start_source_V=start_source_V,
            step_V=step_V,
            delay_s=delay_s,
            read_meters_each_step=read_meters_each_step,
            samples=samples,
            abort_callback=abort_callback,
            event_callback=event_callback,
        )
        zero = None
        final_settle_reads: list[dict[str, Any]] = []
        if not ramp.get("aborted"):
            zero = self.zero_source_only(token=token, read_meters=True, samples=samples)
            # After the Source command reaches 0 V, emission can continue to decay
            # for several seconds.  Record a few diagnostic meter readings so the
            # shutdown log shows the thermal/electrical settling behavior.
            final_settle_s = max(0.0, float(final_settle_s))
            final_read_interval_s = max(0.1, float(final_read_interval_s))
            if read_meters_each_step and final_settle_s > 0:
                t_end = time.time() + final_settle_s
                i = 0
                while time.time() < t_end:
                    time.sleep(min(final_read_interval_s, max(0.0, t_end - time.time())))
                    i += 1
                    final_settle_reads.append({
                        "index": i,
                        "elapsed_after_zero_s": max(0.0, final_settle_s - max(0.0, t_end - time.time())),
                        "meters": self.read_meters(samples=samples),
                    })
        return {
            "action": "normal_source_shutdown",
            "ramp_down": ramp,
            "zero_source_only": zero,
            "final_settle_s": final_settle_s,
            "final_read_interval_s": final_read_interval_s,
            "final_settle_reads": final_settle_reads,
            "note": "Source was ramped down gradually, then Source AO was set to 0 V. Electrostatics were not zeroed. Source Volts is diagnostic only until calibrated.",
        }

    @staticmethod
    def save_source_ramp_log(result: dict[str, Any], prefix: str | Path | None = None) -> dict[str, str]:
        """Save a compact CSV step log and full YAML ramp record."""
        if prefix is None:
            prefix = f"source_ramp_{_dt.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        prefix = Path(prefix)
        csv_path = prefix.with_suffix(".csv")
        yaml_path = prefix.with_suffix(".yaml")
        steps = result.get("steps") or result.get("ramp_down", {}).get("steps") or []
        with open(csv_path, "w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "index", "target_source_V", "ao_voltage_written_V",
                "source_volts_diagnostic_V", "emission_uA", "source_amps_A", "grid_V", "focus_kV", "timestamp_s",
                "safety_stop_reason",
            ])
            for step in steps:
                meters = step.get("meters", {}) or {}
                written = step.get("written", {}) or {}
                writer.writerow([
                    step.get("index"),
                    step.get("target_source_V"),
                    written.get("ao_voltage_V"),
                    (meters.get("source_volts", {}) or {}).get("value"),
                    (meters.get("emission", {}) or {}).get("value"),
                    (meters.get("source_amps", {}) or {}).get("value"),
                    (meters.get("grid", {}) or {}).get("value"),
                    (meters.get("focus", {}) or {}).get("value"),
                    step.get("timestamp_s"),
                    step.get("safety_stop_reason"),
                ])
        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(result, f, sort_keys=False, allow_unicode=True)
        return {"csv": str(csv_path), "yaml": str(yaml_path)}


    def read_meters(self, *, samples: int | None = None) -> dict[str, dict[str, Any]]:
        samples = int(samples or self.config.meter_samples_default)
        samples = max(1, samples)
        meters = list(self.config.meters.values())
        if self.simulate:
            rows: dict[str, dict[str, Any]] = {}
            for m in meters:
                rows[m.name] = {
                    "name": m.name,
                    "channel": m.physical_channel,
                    "raw_voltage_V": 0.0,
                    "value": 0.0,
                    "units": m.units,
                    "samples": samples,
                }
            return rows

        nidaqmx, _System = _import_nidaqmx()
        with nidaqmx.Task() as task:
            for m in meters:
                # Use a safe symmetric range unless the config is later narrowed.
                task.ai_channels.add_ai_voltage_chan(
                    m.physical_channel,
                    min_val=-abs(float(m.input_range_V)),
                    max_val=abs(float(m.input_range_V)),
                )
            raw = task.read(number_of_samples_per_channel=samples, timeout=10.0)

        # NI returns: one channel + one sample -> float; one channel + N -> list;
        # many channels + N -> list[list]. Normalize to list per channel.
        if len(meters) == 1:
            raw_by_ch = [raw if isinstance(raw, list) else [raw]]
        else:
            raw_by_ch = raw
        rows = {}
        for m, vals in zip(meters, raw_by_ch):
            if not isinstance(vals, list):
                vals = [vals]
            vals_f = [float(v) for v in vals]
            mean_v = sum(vals_f) / len(vals_f) if vals_f else 0.0
            rows[m.name] = {
                "name": m.name,
                "channel": m.physical_channel,
                "raw_voltage_V": mean_v,
                "value": m.to_engineering(mean_v),
                "units": m.units,
                "samples": len(vals_f),
            }
        return rows

    def write_raw_ao(self, physical_channel: str, voltage_V: float, *, token: str | None = None, allow_large: bool = False) -> float:
        if token != GUN_DAQ_TOKEN:
            raise RuntimeError("Refusing to write AO: pass the explicit Kimball gun DAQ safety token.")
        voltage_V = float(voltage_V)
        if not allow_large and abs(voltage_V) > self.config.max_raw_ao_test_V:
            raise RuntimeError(
                f"Refusing raw AO test {voltage_V:g} V; limit is ±{self.config.max_raw_ao_test_V:g} V."
            )
        if self.simulate:
            return voltage_V
        nidaqmx, _System = _import_nidaqmx()
        with nidaqmx.Task() as task:
            task.ao_channels.add_ao_voltage_chan(
                physical_channel,
                min_val=float(self.config.ao_min_V),
                max_val=float(self.config.ao_max_V),
            )
            task.write(voltage_V, auto_start=True)
            time.sleep(0.05)
        return voltage_V

    def write_control_engineering(
        self,
        control_name: str,
        engineering_value: float,
        *,
        token: str | None = None,
        allow_large: bool = False,
    ) -> dict[str, Any]:
        if control_name not in self.config.controls:
            raise KeyError(f"Unknown control {control_name!r}. Known: {list(self.config.controls)}")
        c = self.config.controls[control_name]
        deflection_switch_result = None
        if control_name in ("x_center", "y_center"):
            # Any direct X/Y write should also make sure the Kimball deflection
            # switch is enabled first.  This protects GUI, CLI, and imaging paths.
            deflection_switch_result = self.ensure_deflection_enabled(token=token)
        ao_v = c.to_ao_voltage(float(engineering_value))
        self.write_raw_ao(c.physical_channel, ao_v, token=token, allow_large=allow_large)
        out = {
            "control": control_name,
            "channel": c.physical_channel,
            "engineering_value": float(engineering_value),
            "units": c.units,
            "ao_voltage_V": ao_v,
        }
        if deflection_switch_result is not None:
            out["deflection_switch"] = deflection_switch_result
        return out

    def zero_all_outputs(self, *, token: str | None = None, include_digital: bool = False) -> dict[str, Any]:
        if token != GUN_DAQ_TOKEN:
            raise RuntimeError("Refusing to zero Kimball outputs: pass the explicit safety token.")
        controls_all = list(self.config.controls.values())
        # Some legacy logical controls share one physical AO channel, e.g.
        # Source/ECC in Source Mode and Source/ECC in ECC Mode both use ao1.
        # DAQmx will not allow the same physical channel twice in one task, so
        # zero unique physical channels while still reporting every logical name.
        unique_by_channel: dict[str, ControlChannel] = {}
        for c in controls_all:
            unique_by_channel.setdefault(c.physical_channel, c)
        controls = list(unique_by_channel.values())
        result: dict[str, Any] = {"analog_outputs": {}, "digital_outputs": {}}
        if self.simulate:
            for c in controls_all:
                result["analog_outputs"][c.name] = {"channel": c.physical_channel, "ao_voltage_V": 0.0}
            for name, d in self.config.digital_controls.items():
                result["digital_outputs"][name] = {"legacy_line": d.legacy_line, "daqmx_line": d.daqmx_line, "action": "simulated off" if d.daqmx_line else "skipped; no DAQmx line configured"}
            return result

        nidaqmx, _System = _import_nidaqmx()
        if controls:
            with nidaqmx.Task() as task:
                for c in controls:
                    task.ao_channels.add_ao_voltage_chan(
                        c.physical_channel,
                        min_val=float(self.config.ao_min_V),
                        max_val=float(self.config.ao_max_V),
                    )
                task.write([0.0] * len(controls), auto_start=True)
                time.sleep(0.05)
            for c in controls_all:
                result["analog_outputs"][c.name] = {"channel": c.physical_channel, "ao_voltage_V": 0.0}

        if include_digital:
            for name, d in self.config.digital_controls.items():
                if not d.daqmx_line:
                    result["digital_outputs"][name] = {"legacy_line": d.legacy_line, "daqmx_line": None, "action": "skipped; no DAQmx line configured"}
                    continue
                with nidaqmx.Task() as task:
                    task.do_channels.add_do_chan(d.daqmx_line)
                    task.write(bool(d.off_value), auto_start=True)
                result["digital_outputs"][name] = {"legacy_line": d.legacy_line, "daqmx_line": d.daqmx_line, "value": bool(d.off_value), "action": "wrote OFF value"}
        else:
            for name, d in self.config.digital_controls.items():
                result["digital_outputs"][name] = {"legacy_line": d.legacy_line, "daqmx_line": d.daqmx_line, "action": "skipped by default"}
        return result
