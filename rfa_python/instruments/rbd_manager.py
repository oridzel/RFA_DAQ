"""Multi-channel manager for the RBD 9103 picoammeters."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable
import csv
import statistics
import time

import yaml

from .rbd9103 import RBD9103, SimulatedRBD9103, RBDReading


@dataclass
class RBDChannelConfig:
    name: str
    port: str
    enabled: bool = True
    label: str | None = None
    range: str | int = "auto"
    filter: str | int = "off"
    sample_interval_ms: int = 15


class RBDManager:
    """Owns all enabled RBD devices and adds RFA-specific metadata/baselines.

    LabVIEW has separate states such as Read Sneezy, Read Sleepy, etc.
    In Python we keep one device class and use this manager to apply the same
    operation to every enabled channel.
    """

    def __init__(
        self,
        channels: Iterable[RBDChannelConfig],
        *,
        baudrate: int = 57600,
        timeout_s: float = 1.0,
        simulate: bool = False,
    ) -> None:
        self.configs = [c for c in channels if c.enabled]
        self.config_by_name = {c.name: c for c in self.configs}
        self.labels = {c.name: (c.label or c.name) for c in self.configs}
        self.offsets_A: dict[str, float] = {c.name: 0.0 for c in self.configs}
        cls = SimulatedRBD9103 if simulate else RBD9103
        self.devices: dict[str, RBD9103] = {
            c.name: cls(name=c.name, port=c.port, baudrate=baudrate, timeout_s=timeout_s)
            for c in self.configs
        }

    @classmethod
    def from_yaml(cls, path: str | Path, *, simulate: bool = False) -> "RBDManager":
        path = Path(path)
        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        serial_cfg = data.get("serial", {}) or {}
        channels = []
        for name, cfg in (data.get("channels", {}) or {}).items():
            channels.append(
                RBDChannelConfig(
                    name=name,
                    port=str(cfg.get("port", "")),
                    enabled=bool(cfg.get("enabled", True)),
                    label=cfg.get("label"),
                    range=cfg.get("range", "auto"),
                    filter=cfg.get("filter", "off"),
                    sample_interval_ms=int(cfg.get("sample_interval_ms", 15)),
                )
            )
        return cls(
            channels,
            baudrate=int(serial_cfg.get("baudrate", 57600)),
            timeout_s=float(serial_cfg.get("timeout_s", 1.0)),
            simulate=simulate,
        )

    def _annotate(self, reading: RBDReading) -> RBDReading:
        label = self.labels.get(reading.name, reading.name)
        # User-facing displays and CSVs use physical electrode labels.
        # The historical dwarf nickname remains only in the internal `name` field.
        reading.display_name = label
        offset = self.offsets_A.get(reading.name, 0.0)
        reading.offset_A = offset
        if reading.current_A is not None:
            reading.corrected_current_A = reading.current_A - offset
        else:
            reading.corrected_current_A = None
        return reading

    def _annotate_many(self, readings: list[RBDReading]) -> list[RBDReading]:
        return [self._annotate(r) for r in readings]

    def open_all(self) -> dict[str, str]:
        results: dict[str, str] = {}
        for name, dev in self.devices.items():
            try:
                dev.open()
                results[name] = "open"
            except Exception as exc:
                results[name] = f"ERROR: {exc}"
        return results

    def initialize_all(self) -> dict[str, str]:
        results: dict[str, str] = {}
        for cfg in self.configs:
            dev = self.devices[cfg.name]
            try:
                dev.initialize(range_setting=cfg.range, filter_setting=cfg.filter)
                results[cfg.name] = "initialized"
            except Exception as exc:
                results[cfg.name] = f"ERROR: {exc}"
        return results

    def start_all(self) -> dict[str, str]:
        results: dict[str, str] = {}
        for cfg in self.configs:
            dev = self.devices[cfg.name]
            try:
                dev.start_sampling(cfg.sample_interval_ms)
                results[cfg.name] = f"sampling every {cfg.sample_interval_ms} ms"
            except Exception as exc:
                results[cfg.name] = f"ERROR: {exc}"
        return results

    def apply_acquisition_profile(self, profile: dict[str, Any], *, start_sampling: bool = True) -> dict[str, str]:
        """Apply a named measurement acquisition profile to all RBD channels.

        The profile is intentionally simple and channel-uniform for now:
        range, filter, and sample_interval_ms are applied to every enabled
        RBD. This is the safest default for yield/spectrum scans and keeps
        the data files self-documenting. More advanced per-channel profiles
        can be added later if we need them.
        """
        range_setting = profile.get("range", "auto")
        filter_setting = profile.get("filter", 32)
        sample_interval_ms = int(profile.get("sample_interval_ms", 50))
        results: dict[str, str] = {}
        for cfg in self.configs:
            cfg.range = range_setting
            cfg.filter = filter_setting
            cfg.sample_interval_ms = sample_interval_ms
            dev = self.devices[cfg.name]
            try:
                dev.open()
                try:
                    dev.stop_sampling()
                except Exception:
                    pass
                dev.initialize(range_setting=cfg.range, filter_setting=cfg.filter)
                if start_sampling:
                    dev.start_sampling(cfg.sample_interval_ms)
                    results[cfg.name] = (
                        f"profile applied: range={cfg.range}, filter={cfg.filter}, "
                        f"interval={cfg.sample_interval_ms} ms; sampling started"
                    )
                else:
                    results[cfg.name] = (
                        f"profile applied: range={cfg.range}, filter={cfg.filter}, "
                        f"interval={cfg.sample_interval_ms} ms"
                    )
            except Exception as exc:
                results[cfg.name] = f"ERROR: {exc}"
        return results

    def flush_input_all(self) -> None:
        """Flush queued serial samples for every RBD channel."""
        for dev in self.devices.values():
            try:
                dev.flush_input()
            except Exception:
                pass

    def discard_samples(self, n: int, *, sample_period_s: float = 0.0) -> None:
        """Discard N fresh multi-channel RBD samples after a settle/flush step."""
        for _ in range(max(0, int(n))):
            self.read_all_once(latest=False)
            if sample_period_s > 0:
                time.sleep(sample_period_s)

    def stop_all(self) -> None:
        for dev in self.devices.values():
            try:
                dev.stop_sampling()
            except Exception:
                pass

    def close_all(self) -> None:
        for dev in self.devices.values():
            dev.close()

    def read_all_once(self, *, latest: bool = True) -> list[RBDReading]:
        readings: list[RBDReading] = []
        for name, dev in self.devices.items():
            try:
                if latest:
                    readings.append(dev.read_latest_sample(timeout_s=2.0))
                else:
                    readings.append(dev.read_next_sample(timeout_s=2.0))
            except Exception as exc:
                readings.append(
                    RBDReading(
                        name=name,
                        timestamp_utc="",
                        current_A=None,
                        raw_response="",
                        error=str(exc),
                    )
                )
        return self._annotate_many(readings)

    def sample_all_once(self) -> list[RBDReading]:
        readings: list[RBDReading] = []
        for cfg in self.configs:
            dev = self.devices[cfg.name]
            try:
                readings.append(dev.read_one_sample(cfg.sample_interval_ms))
            except Exception as exc:
                readings.append(
                    RBDReading(
                        name=cfg.name,
                        timestamp_utc="",
                        current_A=None,
                        raw_response="",
                        error=str(exc),
                    )
                )
        return self._annotate_many(readings)

    def clear_baseline(self) -> None:
        for name in self.offsets_A:
            self.offsets_A[name] = 0.0

    def measure_baseline(
        self,
        *,
        duration_s: float = 5.0,
        period_s: float = 0.5,
        latest: bool = True,
    ) -> dict[str, float]:
        """Measure and store dark-current/baseline offsets for each channel.

        This assumes the instrument is already in a suitable state, for example
        beam off or no intentional sample current. The returned offsets are in A.
        Corrected current is raw current minus this offset.
        """
        samples: dict[str, list[float]] = {name: [] for name in self.devices}
        deadline = time.time() + max(0.0, duration_s)
        while time.time() < deadline:
            readings = self.read_all_once(latest=latest)
            for r in readings:
                if r.current_A is not None:
                    samples[r.name].append(r.current_A)
            time.sleep(max(0.05, period_s))
        for name, values in samples.items():
            if values:
                self.offsets_A[name] = statistics.median(values)
        return dict(self.offsets_A)


class RBDCsvLogger:
    def __init__(self, output_path: str | Path) -> None:
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.output_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(
            self._file,
            fieldnames=[
                "timestamp_utc",
                "name",
                "display_name",
                "current_A",
                "stable",
                "unstable",
                "out_of_range",
                "status_code",
                "range_code",
                "unit",
                "raw_response",
                "error",
            ],
            extrasaction="ignore",
        )
        self._writer.writeheader()

    def write_many(self, readings: Iterable[RBDReading]) -> None:
        for r in readings:
            self._writer.writerow(r.to_dict())
        self._file.flush()

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> "RBDCsvLogger":
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()
