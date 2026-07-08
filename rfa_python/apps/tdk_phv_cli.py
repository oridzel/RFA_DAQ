"""Command-line tester for the TDK-Lambda PHV high-voltage supply."""

from __future__ import annotations

import argparse
from pathlib import Path
import pprint

import yaml

from rfa_python.instruments.tdk_phv import PHV, SimulatedPHV

HV_TOKEN = "I_UNDERSTAND_THIS_CONTROLS_HIGH_VOLTAGE"


def load_config(path: str | Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def build_device(cfg: dict, simulate_override: bool | None) -> PHV | SimulatedPHV:
    serial_cfg = cfg.get("serial", {}) or {}
    safety = cfg.get("safety", {}) or {}
    simulate = bool(cfg.get("simulate", False)) if simulate_override is None else simulate_override
    if simulate:
        return SimulatedPHV(name=cfg.get("name", "TDK PHV"), polarity_mode=cfg.get("polarity_mode", "unknown"))
    return PHV(
        port=str(serial_cfg.get("port", "COM1")),
        name=cfg.get("name", "TDK PHV"),
        baudrate=int(serial_cfg.get("baudrate", 9600)),
        timeout_s=float(serial_cfg.get("timeout_s", 1.0)),
        max_abs_voltage_V=float(safety.get("max_abs_voltage_V", 100.0)),
        max_current_A=float(safety.get("max_current_A", 1e-3)),
        default_current_A=float(safety.get("default_current_A", 1e-4)),
        polarity_mode=cfg.get("polarity_mode", "unknown"),
        require_enable_token=bool(safety.get("require_enable_token", True)),
    )


def main() -> None:
    p = argparse.ArgumentParser(description="Test/control TDK-Lambda PHV HV supply")
    p.add_argument("--config", default=str(Path(__file__).resolve().parents[1] / "config" / "tdk_phv.yaml"))
    p.add_argument("--simulate", action="store_true")
    p.add_argument("--real", action="store_true", help="force real hardware even if config simulate=true")
    p.add_argument("--status", action="store_true")
    p.add_argument("--idn", action="store_true")
    p.add_argument("--raw", action="append", help='send raw PHV command/query, e.g. --raw ">DON?"; may be repeated')
    p.add_argument("--set-voltage", type=float, help="set magnitude voltage in V only; requires --i-understand-hv")
    p.add_argument("--apply-voltage", type=float, help="LabVIEW-style signed voltage in V: current -> output on -> set voltage")
    p.add_argument("--set-current", type=float, help="set current limit in A; requires --i-understand-hv")
    p.add_argument("--current-ma", type=float, help="set current limit in mA; convenient because LabVIEW controls use mA")
    p.add_argument("--ramp-rate", type=float, help="set voltage ramp rate V/s; requires --i-understand-hv")
    p.add_argument("--ramp-mode", type=int, choices=[0, 1, 2], help="set ramp mode; requires --i-understand-hv")
    p.add_argument("--output", choices=["on", "off"], help="set HV output; requires --i-understand-hv")
    p.add_argument("--zero-output", action="store_true", help="set voltage to 0, then turn output off")
    p.add_argument("--settle", type=float, default=2.0, help="seconds to wait after --apply-voltage")
    p.add_argument("--i-understand-hv", action="store_true", help="required for any HV-changing command")
    args = p.parse_args()

    cfg = load_config(args.config)
    simulate_override = True if args.simulate else (False if args.real else None)
    dev = build_device(cfg, simulate_override)
    dev.open()
    try:
        changes_requested = any([
            args.set_voltage is not None, args.apply_voltage is not None, args.set_current is not None,
            args.current_ma is not None, args.ramp_rate is not None, args.ramp_mode is not None,
            args.output is not None, args.zero_output,
        ])
        if changes_requested and hasattr(dev, "arm_hv_changes"):
            if not args.i_understand_hv:
                raise SystemExit("Refusing HV-changing command without --i-understand-hv")
            dev.arm_hv_changes(HV_TOKEN)
        if args.idn:
            print(dev.identify())
        if args.raw:
            for cmd in args.raw:
                if hasattr(dev, "raw_query"):
                    print(f"{cmd} -> {dev.raw_query(cmd)}")
                else:
                    print(f"{cmd} -> simulator has no raw_query")
        if args.ramp_mode is not None:
            dev.set_ramp_mode(args.ramp_mode); print(f"Ramp mode set to {args.ramp_mode}")
        if args.ramp_rate is not None:
            dev.set_voltage_ramp_rate(args.ramp_rate); print(f"Ramp rate set to {args.ramp_rate:g} V/s")
        current_A = args.set_current
        if args.current_ma is not None:
            current_A = args.current_ma * 1e-3
        if (args.set_current is not None or args.current_ma is not None) and args.apply_voltage is None:
            assert current_A is not None
            dev.set_current(current_A); print(f"Current limit set to {current_A:g} A ({current_A*1e3:g} mA)")
        if args.apply_voltage is not None:
            dev.apply_signed_voltage(args.apply_voltage, current_limit_A=current_A, ramp_rate_V_per_s=args.ramp_rate, ramp_mode=args.ramp_mode, settle_s=args.settle)
            print(f"Applied signed voltage {args.apply_voltage:g} V")
        if args.set_voltage is not None:
            dev.set_voltage(args.set_voltage); print(f"Magnitude voltage setpoint set to {abs(args.set_voltage):g} V")
        if args.output:
            dev.output(args.output == "on"); print(f"HV output: {'ON' if dev.query_output() else 'OFF'}")
        if args.zero_output:
            dev.zero_and_output_off(); print("Voltage set to zero and HV output OFF")
        if args.status or not any([args.idn, changes_requested, bool(args.raw)]):
            pprint.pp(dev.status().to_dict())
    finally:
        dev.close()


if __name__ == "__main__":
    main()
