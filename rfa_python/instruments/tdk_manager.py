"""Manager for the four RFA TDK-Lambda PHV supplies."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .tdk_phv import PHV, SimulatedPHV, PolarityMode

HV_TOKEN = "I_UNDERSTAND_THIS_CONTROLS_HIGH_VOLTAGE"


@dataclass
class TDKSupplyConfig:
    name: str
    port: str
    enabled: bool = True
    polarity_mode: PolarityMode = "unknown"
    baudrate: int = 9600
    timeout_s: float = 1.0
    max_abs_voltage_V: float = 100.0
    max_current_A: float = 1e-3
    default_current_A: float = 1e-4
    require_enable_token: bool = True
    m0_is_magnitude_only: bool = False


def load_tdk_config(path: str | Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def supply_configs_from_yaml(cfg: dict[str, Any]) -> dict[str, TDKSupplyConfig]:
    serial_defaults = cfg.get("serial_defaults", {}) or {}
    safety_defaults = cfg.get("safety_defaults", {}) or {}
    supplies = cfg.get("supplies", {}) or {}
    out: dict[str, TDKSupplyConfig] = {}
    for name, raw in supplies.items():
        raw = raw or {}
        out[name] = TDKSupplyConfig(
            name=name,
            port=str(raw.get("port", "COM1")),
            enabled=bool(raw.get("enabled", True)),
            polarity_mode=str(raw.get("polarity_mode", "unknown")),  # type: ignore[arg-type]
            baudrate=int(raw.get("baudrate", serial_defaults.get("baudrate", 9600))),
            timeout_s=float(raw.get("timeout_s", serial_defaults.get("timeout_s", 1.0))),
            max_abs_voltage_V=float(raw.get("max_abs_voltage_V", safety_defaults.get("max_abs_voltage_V", 100.0))),
            max_current_A=float(raw.get("max_current_A", safety_defaults.get("max_current_A", 1e-3))),
            default_current_A=float(raw.get("default_current_A", safety_defaults.get("default_current_A", 1e-4))),
            require_enable_token=bool(raw.get("require_enable_token", safety_defaults.get("require_enable_token", True))),
            m0_is_magnitude_only=bool(raw.get("m0_is_magnitude_only", False)),
        )
    return out


class TDKManager:
    def __init__(self, configs: dict[str, TDKSupplyConfig], *, simulate: bool = False) -> None:
        self.configs = configs
        self.simulate = simulate
        self.devices: dict[str, PHV | SimulatedPHV] = {}

    @classmethod
    def from_yaml(cls, path: str | Path, *, simulate_override: bool | None = None) -> "TDKManager":
        cfg = load_tdk_config(path)
        simulate = bool(cfg.get("simulate", False)) if simulate_override is None else bool(simulate_override)
        return cls(supply_configs_from_yaml(cfg), simulate=simulate)

    def names(self, *, enabled_only: bool = True) -> list[str]:
        names = []
        for name, cfg in self.configs.items():
            if enabled_only and not cfg.enabled:
                continue
            names.append(name)
        return names

    def get(self, name: str) -> PHV | SimulatedPHV:
        if name not in self.configs:
            raise KeyError(f"Unknown TDK supply {name!r}. Available: {', '.join(self.configs)}")
        cfg = self.configs[name]
        if not cfg.enabled:
            raise RuntimeError(f"TDK supply {name!r} is disabled in config.")
        if name not in self.devices:
            if self.simulate:
                dev: PHV | SimulatedPHV = SimulatedPHV(name=name, polarity_mode=cfg.polarity_mode, m0_is_magnitude_only=cfg.m0_is_magnitude_only, max_abs_voltage_V=cfg.max_abs_voltage_V, max_current_A=cfg.max_current_A, default_current_A=cfg.default_current_A)
            else:
                dev = PHV(
                    port=cfg.port,
                    name=name,
                    baudrate=cfg.baudrate,
                    timeout_s=cfg.timeout_s,
                    max_abs_voltage_V=cfg.max_abs_voltage_V,
                    max_current_A=cfg.max_current_A,
                    default_current_A=cfg.default_current_A,
                    polarity_mode=cfg.polarity_mode,
                    require_enable_token=cfg.require_enable_token,
                    m0_is_magnitude_only=cfg.m0_is_magnitude_only,
                )
            self.devices[name] = dev
        return self.devices[name]

    def open(self, name: str) -> None:
        self.get(name).open()

    def close_all(self) -> None:
        for dev in self.devices.values():
            try:
                dev.close()
            except Exception:
                pass

    def arm_all(self) -> None:
        for name in self.names():
            dev = self.get(name)
            if hasattr(dev, "arm_hv_changes"):
                dev.arm_hv_changes(HV_TOKEN)
