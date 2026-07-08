"""Command-line tester for the SRS DC205 voltage source."""

from __future__ import annotations

import argparse
from pathlib import Path
import pprint

import yaml

from rfa_python.instruments.srs_dc205 import DC205, SimulatedDC205


def load_config(path: str | Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def build_device(cfg: dict, simulate_override: bool | None) -> DC205 | SimulatedDC205:
    serial_cfg = cfg.get("serial", {}) or {}
    safety = cfg.get("safety", {}) or {}
    simulate = bool(cfg.get("simulate", False)) if simulate_override is None else simulate_override
    if simulate:
        return SimulatedDC205(name=cfg.get("name", "SRS DC205"))
    return DC205(
        port=str(serial_cfg.get("port", "COM1")),
        name=cfg.get("name", "SRS DC205"),
        baudrate=int(serial_cfg.get("baudrate", 115200)),
        timeout_s=float(serial_cfg.get("timeout_s", 1.0)),
        max_abs_voltage_V=float(safety.get("max_abs_voltage_V", 10.0)),
        allow_100V_range=bool(safety.get("allow_100V_range", False)),
    )


def main() -> None:
    p = argparse.ArgumentParser(description="Test/control SRS DC205 voltage source")
    p.add_argument("--config", default=str(Path(__file__).resolve().parents[1] / "config" / "srs_dc205.yaml"))
    p.add_argument("--simulate", action="store_true", help="force simulator")
    p.add_argument("--real", action="store_true", help="force real hardware even if config simulate=true")
    p.add_argument("--status", action="store_true", help="read ID/range/voltage setpoint/output/interlock/errors")
    p.add_argument("--idn", action="store_true", help="query *IDN?")
    p.add_argument("--reset", action="store_true", help="send *RST: output OFF, 0 V, RANGE1, default configuration")
    p.add_argument("--range", dest="range_name", help="set range: 1, 10, or 100")
    p.add_argument("--set-voltage", type=float, help="set voltage setpoint in V")
    p.add_argument("--output", choices=["on", "off"], help="set output state")
    p.add_argument("--query-voltage", action="store_true", help="query VOLT? (programmed setpoint, not independent measured output)")
    p.add_argument("--query-setpoint", action="store_true", help="same as --query-voltage; explicit name")
    p.add_argument("--query-range", action="store_true", help="query RNGE?")
    p.add_argument("--query-output", action="store_true", help="query SOUT?")
    p.add_argument("--raw", action="append", default=[], help="send one raw command/query, e.g. --raw RNGE? --raw VOLT?")
    p.add_argument("--no-init", action="store_true", help="do not send TERM/TOKN setup commands first")
    args = p.parse_args()

    cfg = load_config(args.config)
    simulate_override = True if args.simulate else (False if args.real else None)
    dev = build_device(cfg, simulate_override)
    dev.open()
    mode = "SIMULATED" if getattr(dev, "simulate", False) else "REAL"
    print(f"SRS DC205 mode: {mode}")
    if mode == "SIMULATED":
        print("Note: simulator state is not persistent between separate CLI runs. Use --real or set simulate: false in config for hardware.")
    try:
        if not args.no_init and hasattr(dev, "set_termination_lf"):
            try:
                dev.set_termination_lf()
                dev.set_token_mode(True)
            except Exception as exc:
                print(f"Warning: TERM/TOKN init failed: {exc}")
        if args.idn:
            print(dev.identify())
        if args.reset:
            dev.reset_default()
            print("DC205 reset sent: *RST")
        if args.range_name is not None:
            # DC205 does not allow RNGE while output is on; turn it off first.
            dev.set_output(False)
            dev.set_range(args.range_name)
            print(f"Range set to {dev.query_range()}")
        if args.set_voltage is not None:
            dev.set_voltage(args.set_voltage)
            print(f"Voltage setpoint: {dev.query_voltage_setpoint():.9g} V")
        if args.output:
            dev.set_output(args.output == "on")
            print(f"Output: {'ON' if dev.query_output() else 'OFF'}")
        if args.query_range:
            print(f"Range: {dev.query_range()}")
        if args.query_output:
            print(f"Output: {'ON' if dev.query_output() else 'OFF'}")
        if args.query_voltage or args.query_setpoint:
            print(f"Voltage setpoint: {dev.query_voltage_setpoint():.9g} V")
        for cmd in args.raw:
            print(f">>> {cmd}")
            print(dev.raw(cmd))
        if args.status or not any([
            args.idn, args.reset, args.range_name, args.set_voltage is not None,
            args.output, args.query_voltage, args.query_setpoint, args.query_range,
            args.query_output, bool(args.raw)
        ]):
            pprint.pp(dev.status().to_dict())
    finally:
        dev.close()


if __name__ == "__main__":
    main()
