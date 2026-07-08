"""Command-line tester for the four RFA TDK-Lambda PHV supplies."""

from __future__ import annotations

import argparse
from pathlib import Path
import pprint

from rfa_python.instruments.tdk_manager import TDKManager, HV_TOKEN


def default_config_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "tdk_supplies.yaml")


def main() -> None:
    p = argparse.ArgumentParser(description="Test/control one of the four RFA TDK PHV supplies")
    p.add_argument("--config", default=default_config_path())
    p.add_argument("--simulate", action="store_true")
    p.add_argument("--real", action="store_true", help="force real hardware even if config simulate=true")
    p.add_argument("--list", action="store_true", help="list configured supply names")
    p.add_argument("--supply", "-s", default="R", help="supply name from config: PN, N, PNR, R, ...")
    p.add_argument("--idn", action="store_true")
    p.add_argument("--status", action="store_true")
    p.add_argument("--raw", action="append", help='send raw PHV command/query, e.g. --raw ">DON?"; may be repeated')
    p.add_argument("--query-ramp", action="store_true", help="query PHV voltage ramp mode/rate/status")
    p.add_argument("--apply-voltage", type=float, help="signed requested output voltage in V; sign controls polarity where supported")
    p.add_argument("--wait-voltage", type=float, help="wait until actual signed voltage reaches this value")
    p.add_argument("--set-voltage", type=float, help="set PHV magnitude setpoint only, without output or polarity logic")
    p.add_argument("--set-current", type=float, help="set current limit in A")
    p.add_argument("--current-ma", type=float, help="current limit in mA; convenient because LabVIEW controls use mA")
    p.add_argument("--ramp-rate", type=float, help="set voltage ramp rate in V/s")
    p.add_argument("--ramp-mode", type=int, choices=[0, 1, 2, 4], help="set voltage ramp mode")
    p.add_argument("--voltage-tolerance", type=float, default=0.2, help="voltage wait tolerance in V")
    p.add_argument("--wait-timeout", type=float, default=10.0, help="voltage wait timeout in s")
    p.add_argument("--poll", type=float, default=0.2, help="voltage polling interval in s")
    p.add_argument("--output", choices=["on", "off"], help="turn HV output on/off")
    p.add_argument("--zero-and-wait", action="store_true", help="set voltage to 0 V and wait for actual voltage to reach zero")
    p.add_argument("--output-off-after-zero", action="store_true", help="after --zero-and-wait reaches zero, turn HV output off")
    p.add_argument("--zero-output", action="store_true", help="set voltage to 0, then turn output off")
    p.add_argument("--settle", type=float, default=2.0, help="seconds to wait after legacy --apply-voltage if wait is not used")
    p.add_argument("--i-understand-hv", action="store_true", help="required for any HV-changing command")
    args = p.parse_args()

    simulate_override = True if args.simulate else (False if args.real else None)
    manager = TDKManager.from_yaml(args.config, simulate_override=simulate_override)

    if args.list:
        print("Configured TDK supplies:")
        for name in manager.names(enabled_only=False):
            cfg = manager.configs[name]
            state = "enabled" if cfg.enabled else "disabled"
            print(f"  {name:4s} {state:8s} port={cfg.port:8s} polarity={cfg.polarity_mode} m0_mag_only={cfg.m0_is_magnitude_only}")
        return

    dev = manager.get(args.supply)
    dev.open()
    try:
        changes_requested = any([
            args.apply_voltage is not None,
            args.set_voltage is not None,
            args.set_current is not None,
            args.current_ma is not None,
            args.ramp_rate is not None,
            args.ramp_mode is not None,
            args.output is not None,
            args.zero_output,
            args.zero_and_wait,
        ])
        if changes_requested:
            if not args.i_understand_hv:
                raise SystemExit("Refusing HV-changing command without --i-understand-hv")
            if hasattr(dev, "arm_hv_changes"):
                dev.arm_hv_changes(HV_TOKEN)

        current_A = args.set_current
        if args.current_ma is not None:
            current_A = args.current_ma * 1e-3

        if args.idn:
            print(dev.identify())
        if args.raw:
            for cmd in args.raw:
                if hasattr(dev, "raw_query"):
                    print(f"{cmd} -> {dev.raw_query(cmd)}")
                else:
                    print(f"{cmd} -> simulator has no raw_query")
        if args.query_ramp:
            print(f"Ramp mode:   {dev.query_voltage_ramp_mode()}")
            print(f"Ramp rate:   {dev.query_voltage_ramp_rate()} V/s")
            print(f"Ramp status: {dev.query_voltage_ramp_status()}")
            print(f"Ramp inst.:  {dev.query_instantaneous_ramp_voltage()} V")
        if args.ramp_mode is not None:
            dev.set_ramp_mode(args.ramp_mode)
            print(f"{args.supply}: ramp mode set to {args.ramp_mode}")
        if args.ramp_rate is not None:
            dev.set_voltage_ramp_rate(args.ramp_rate)
            print(f"{args.supply}: ramp rate set to {args.ramp_rate:g} V/s")
        if (args.set_current is not None or args.current_ma is not None) and args.apply_voltage is None and not args.zero_and_wait:
            assert current_A is not None
            dev.set_current(current_A)
            print(f"{args.supply}: current limit set to {current_A:g} A ({current_A*1e3:g} mA)")
        if args.apply_voltage is not None:
            # Measurement-style command: set signed voltage and wait for actual M0? readback.
            actual = dev.set_signed_voltage_and_wait(
                args.apply_voltage,
                current_limit_A=current_A,
                ramp_rate_V_per_s=args.ramp_rate,
                ramp_mode=args.ramp_mode,
                tolerance_V=args.voltage_tolerance,
                timeout_s=args.wait_timeout,
                poll_s=args.poll,
            )
            print(f"{args.supply}: applied signed voltage {args.apply_voltage:g} V; actual {actual:g} V")
        if args.set_voltage is not None:
            dev.set_voltage(args.set_voltage)
            print(f"{args.supply}: magnitude voltage setpoint set to {abs(args.set_voltage):g} V")
        if args.output:
            dev.output(args.output == "on")
            print(f"{args.supply}: HV output {'ON' if dev.query_output() else 'OFF'}")
        if args.zero_and_wait:
            actual = dev.zero_and_wait(
                current_limit_A=current_A,
                tolerance_V=args.voltage_tolerance,
                timeout_s=args.wait_timeout,
                poll_s=args.poll,
                output_off_after_zero=args.output_off_after_zero,
            )
            print(f"{args.supply}: reached zero; actual {actual:g} V")
        if args.zero_output:
            dev.zero_and_output_off()
            print(f"{args.supply}: voltage set to zero and output OFF")
        if args.wait_voltage is not None:
            actual = dev.wait_for_signed_voltage(
                args.wait_voltage,
                tolerance_V=args.voltage_tolerance,
                timeout_s=args.wait_timeout,
                poll_s=args.poll,
            )
            print(f"{args.supply}: actual voltage reached {actual:g} V")
        if args.status or not any([args.idn, changes_requested, bool(args.raw), args.list, args.query_ramp, args.wait_voltage is not None]):
            pprint.pp(dev.status().to_dict())
    finally:
        manager.close_all()


if __name__ == "__main__":
    main()
