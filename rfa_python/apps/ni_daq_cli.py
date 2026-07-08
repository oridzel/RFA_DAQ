"""Command-line tests for the NI/Kimball DAQ layer."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import yaml

from rfa_python.instruments.ni_kimball_daq import KimballDAQ, GUN_DAQ_TOKEN


def default_config_path(filename: str) -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / filename)


def print_yaml(obj) -> None:
    print(yaml.safe_dump(obj, sort_keys=False, allow_unicode=True).rstrip())


def main() -> None:
    parser = argparse.ArgumentParser(description="NI-DAQmx / Kimball gun DAQ safe-test CLI v0.19.17")
    parser.add_argument("--config", default=default_config_path("kimball_daq.yaml"))
    parser.add_argument("--real", action="store_true", help="use real NI hardware; default is simulated")
    parser.add_argument("--simulate", action="store_true", help="force simulation")
    parser.add_argument("--list-devices", action="store_true")
    parser.add_argument("--list-channels", metavar="DEVICE", help="list physical channels for a device, e.g. PXI1Slot4")
    parser.add_argument("--read-kimball-meters", "--read-meters", dest="read_kimball_meters", action="store_true", help="read all configured Kimball meter inputs")
    parser.add_argument("--samples", type=int, default=None, help="samples/channel for meter averaging")
    parser.add_argument("--zero-kimball-outputs", action="store_true", help="write 0 V to all configured Kimball AO controls")
    parser.add_argument("--include-digital", action="store_true", help="also write configured DO lines False; skipped if no DAQmx lines in config")
    parser.add_argument("--write-ao", nargs=2, metavar=("CHANNEL", "VOLTAGE"), help="tiny raw AO test, e.g. PXI1Slot4/ao0 0.1")
    parser.add_argument("--write-control", nargs=2, metavar=("CONTROL", "VALUE"), help="engineering-unit test for a named control")
    parser.add_argument("--allow-large-ao", action="store_true", help="allow AO values beyond the small-test limit")
    parser.add_argument("--show-lut", action="store_true", help="show the first/last few Kimball LUT rows")
    parser.add_argument("--lut-energy-eV", type=float, help="show interpolated LUT X/Y and suggested grid for this energy")
    parser.add_argument("--apply-electrostatics", action="store_true", help="apply Energy/Grid/Focus/X/Y controls only; never Source/ECC")
    parser.add_argument("--energy-eV", type=float, help="target beam energy in eV for electrostatic apply")
    parser.add_argument("--grid-V", type=float, default=None, help="target Grid/G-1 voltage; default suggests 2.5%% of energy")
    parser.add_argument("--focus-kV", type=float, default=None, help="target Focus value in kV engineering units")
    parser.add_argument("--apply-focus", action="store_true", help="actually write Focus control when applying electrostatics")
    parser.add_argument("--no-grid", action="store_true", help="do not write Grid control")
    parser.add_argument("--no-energy", action="store_true", help="do not write Energy control")
    parser.add_argument("--no-deflection", action="store_true", help="do not write X/Y center controls from LUT")
    parser.add_argument("--deflection-on", action="store_true", help="turn Kimball deflection switch ON using configured DO line")
    parser.add_argument("--deflection-off", action="store_true", help="turn Kimball deflection switch OFF using configured DO line")
    parser.add_argument("--current-energy-eV", type=float, default=None, help="optional current energy for ordering; otherwise read meter")
    parser.add_argument("--step-delay", type=float, default=0.2, help="delay between electrostatic control writes")
    parser.add_argument("--apply-source-warmup", action="store_true", help="set warm-up Energy then Grid only; does not ramp Source")
    parser.add_argument("--warmup-energy-eV", type=float, default=1000.0, help="warm-up Energy before Source ramp; default 1000 eV")
    parser.add_argument("--warmup-grid-V", type=float, default=500.0, help="warm-up Grid before Source ramp; default 500 V")
    parser.add_argument("--skip-warmup-before-ramp", action="store_true", help="for --ramp-source, skip the default warm-up Energy/Grid apply")
    parser.add_argument("--ramp-source", action="store_true", help="regular warm-up: set Energy/Grid, then ramp Source/ECC control in Source mode only")
    parser.add_argument("--set-source-V", type=float, default=None, help="adjust Source voltage only; do not touch Energy/Grid/Focus/X/Y; start is read from Source Volts meter unless --start-source-V is given")
    parser.add_argument("--post-bake-conditioning-warmup", action="store_true", help="one-time post-bake conditioning: Energy/Grid, ramp to first Source current, hold, ramp to second Source current")
    parser.add_argument("--first-source-current-A", type=float, default=0.8, help="post-bake conditioning first Source current target; default 0.8 A")
    parser.add_argument("--first-hold-s", type=float, default=600.0, help="post-bake conditioning hold time after first current; default 600 s")
    parser.add_argument("--second-source-current-A", type=float, default=1.7, help="post-bake conditioning second Source current target; default 1.7 A")
    parser.add_argument("--max-conditioning-source-V", type=float, default=3.0, help="max commanded Source voltage for current-target conditioning; default 3.0 V")
    parser.add_argument("--conditioning-hold-read-interval-s", type=float, default=10.0, help="meter read interval during conditioning hold; default 10 s")
    parser.add_argument("--ramp-source-down", action="store_true", help="normal Source turn-off: ramp Source down gradually to 0 V")
    parser.add_argument("--zero-source-only", action="store_true", help="write only Source-mode AO to 0 V; leave electrostatics unchanged")
    parser.add_argument("--source-raw-ao-test-V", type=float, default=None, help="write Source/ECC Source-mode AO channel directly for cautious calibration, e.g. 0.05 V; bypasses engineering scaling")
    parser.add_argument("--normal-source-shutdown", action="store_true", help="ramp Source down to 0 V, then zero Source AO only")
    parser.add_argument("--target-source-V", type=float, help="target Source voltage for --ramp-source")
    parser.add_argument("--start-source-V", type=float, default=None, help="optional starting Source voltage; otherwise read source meter")
    parser.add_argument("--source-step-V", type=float, default=None, help="Source ramp step; default from config, normally 0.1 V")
    parser.add_argument("--source-delay-s", type=float, default=None, help="delay after each Source write before meter read / next step; default from config, normally 10 s")
    parser.add_argument("--source-no-meter-each-step", action="store_true", help="do not read Kimball meters after each Source step")
    parser.add_argument("--max-emission-uA", type=float, default=None, help="abort Source ramp if emission meter exceeds this value")
    parser.add_argument("--max-source-amps-A", type=float, default=None, help="abort Source ramp if Source Amps meter exceeds this value")
    parser.add_argument("--source-log-prefix", default=None, help="save Source ramp CSV/YAML logs using this path prefix")
    parser.add_argument("--shutdown-final-settle-s", type=float, default=10.0, help="after normal Source shutdown, keep reading meters for this many seconds")
    parser.add_argument("--shutdown-final-read-interval-s", type=float, default=2.0, help="meter read interval during final shutdown settle")
    parser.add_argument("--i-understand-gun-daq", action="store_true", help="required for any AO/DO write")
    args = parser.parse_args()

    simulate = True
    if args.real:
        simulate = False
    if args.simulate:
        simulate = True

    daq = KimballDAQ.from_yaml(args.config, simulate_override=simulate)
    token = GUN_DAQ_TOKEN if args.i_understand_gun_daq else None

    did = False
    if args.list_devices:
        print_yaml({"simulate": daq.simulate, "devices": daq.list_devices()})
        did = True
    if args.list_channels:
        print_yaml({"simulate": daq.simulate, "device": args.list_channels, "channels": daq.list_channels(args.list_channels)})
        did = True
    if args.read_kimball_meters:
        print_yaml({"simulate": daq.simulate, "meters": daq.read_meters(samples=args.samples)})
        did = True
    if args.zero_kimball_outputs:
        result = daq.zero_all_outputs(token=token, include_digital=args.include_digital)
        print_yaml({"simulate": daq.simulate, "zero_outputs": result})
        did = True
    if args.write_ao:
        channel, voltage_s = args.write_ao
        value = daq.write_raw_ao(channel, float(voltage_s), token=token, allow_large=args.allow_large_ao)
        print_yaml({"simulate": daq.simulate, "write_ao": {"channel": channel, "voltage_V": value}})
        did = True
    if args.write_control:
        control, value_s = args.write_control
        result = daq.write_control_engineering(control, float(value_s), token=token, allow_large=args.allow_large_ao)
        print_yaml({"simulate": daq.simulate, "write_control": result})
        did = True
    if args.source_raw_ao_test_V is not None:
        result = daq.write_source_raw_ao_test(args.source_raw_ao_test_V, token=token, allow_large=args.allow_large_ao, samples=args.samples)
        print_yaml({"simulate": daq.simulate, "source_raw_ao_test": result})
        did = True

    if args.deflection_on:
        result = daq.set_deflection_switch(True, token=token)
        print_yaml({"simulate": daq.simulate, "deflection_switch": result})
        did = True
    if args.deflection_off:
        result = daq.set_deflection_switch(False, token=token)
        print_yaml({"simulate": daq.simulate, "deflection_switch": result})
        did = True

    if args.show_lut:
        rows = daq.load_lut()
        out = [{"energy_eV": r.energy_eV, "x_center_V": r.x_center_V, "y_center_V": r.y_center_V} for r in rows[:5]]
        if len(rows) > 10:
            out.append({"...": "..."})
        out.extend({"energy_eV": r.energy_eV, "x_center_V": r.x_center_V, "y_center_V": r.y_center_V} for r in rows[-5:])
        print_yaml({"lut_rows": len(rows), "preview": out})
        did = True
    if args.lut_energy_eV is not None:
        lut = daq.lut_for_energy(args.lut_energy_eV)
        lut["suggested_grid_V_2p5pct"] = daq.suggested_grid_V_for_energy(args.lut_energy_eV)
        print_yaml({"lut_target": lut})
        did = True
    if args.apply_electrostatics:
        if args.energy_eV is None:
            raise RuntimeError("--apply-electrostatics requires --energy-eV")
        target = daq.make_electrostatic_target_from_lut(
            args.energy_eV,
            grid_V=args.grid_V,
            focus_kV=args.focus_kV,
            apply_grid=not args.no_grid,
            apply_focus=args.apply_focus,
            apply_energy=not args.no_energy,
            apply_deflection=not args.no_deflection,
        )
        result = daq.apply_electrostatic_target(
            target,
            token=token,
            current_energy_eV=args.current_energy_eV,
            delay_s=args.step_delay,
        )
        print_yaml({"simulate": daq.simulate, "apply_electrostatics": result})
        did = True

    if args.apply_source_warmup:
        result = daq.apply_source_warmup_electrostatics(
            energy_eV=args.warmup_energy_eV,
            grid_V=args.warmup_grid_V,
            token=token,
            delay_s=args.step_delay,
            samples=args.samples,
        )
        print_yaml({"simulate": daq.simulate, "source_warmup": result})
        did = True


    if args.post_bake_conditioning_warmup:
        def emit_step(step):
            if step.get("hold"):
                meters = step.get("meters", {}) or {}
                sa = meters.get("source_amps", {}).get("value")
                em = meters.get("emission", {}).get("value")
                print(f"conditioning hold read {step.get('index')}: elapsed={step.get('elapsed_s')} s, source_A={sa}, emission={em}")
            else:
                meters = step.get("meters", {}) or {}
                sv = meters.get("source_volts", {}).get("value")
                em = meters.get("emission", {}).get("value")
                sa = meters.get("source_amps", {}).get("value")
                print(f"conditioning step {step.get('index')}: target_source_V={step.get('target_source_V')} V, source_volts_diag={sv}, source_A={sa}, emission={em}, reason={step.get('safety_stop_reason')}")
        result = daq.post_bake_conditioning_warmup(
            token=token,
            energy_eV=args.warmup_energy_eV,
            grid_V=args.warmup_grid_V,
            first_current_A=args.first_source_current_A,
            first_hold_s=args.first_hold_s,
            second_current_A=args.second_source_current_A,
            max_source_V=args.max_conditioning_source_V,
            step_V=args.source_step_V,
            delay_s=args.source_delay_s,
            hold_read_interval_s=args.conditioning_hold_read_interval_s,
            samples=args.samples,
            event_callback=emit_step,
            max_emission_uA=args.max_emission_uA,
        )
        print_yaml({"simulate": daq.simulate, "post_bake_conditioning_warmup": result})
        did = True


    if args.set_source_V is not None:
        def emit_step(step):
            meters = step.get("meters", {}) or {}
            sv = meters.get("source_volts", {}).get("value")
            em = meters.get("emission", {}).get("value")
            sa = meters.get("source_amps", {}).get("value")
            print(f"set-source step {step.get('index')}: target={step.get('target_source_V')} V, source_volts_diag={sv}, source_A={sa}, emission={em}")
        result = daq.set_source_voltage(
            args.set_source_V,
            token=token,
            start_source_V=args.start_source_V,
            step_V=args.source_step_V,
            delay_s=args.source_delay_s,
            read_meters_each_step=not args.source_no_meter_each_step,
            samples=args.samples,
            event_callback=emit_step,
            max_emission_uA=args.max_emission_uA,
            max_source_amps_A=args.max_source_amps_A,
        )
        out = {"simulate": daq.simulate, "set_source_voltage": result}
        if args.source_log_prefix is not None:
            out["saved_logs"] = daq.save_source_ramp_log(result.get("ramp", {}), args.source_log_prefix)
        print_yaml(out)
        did = True

    if args.ramp_source:
        if args.target_source_V is None:
            raise RuntimeError("--ramp-source requires --target-source-V")
        def emit_step(step):
            meters = step.get("meters", {}) or {}
            sv = meters.get("source_volts", {}).get("value")
            em = meters.get("emission", {}).get("value")
            print(f"source step {step.get('index')}: target={step.get('target_source_V')} V, source_volts_diag={sv}, emission={em}")
        warmup_result = None
        if not args.skip_warmup_before_ramp:
            warmup_result = daq.apply_source_warmup_electrostatics(
                energy_eV=args.warmup_energy_eV,
                grid_V=args.warmup_grid_V,
                token=token,
                delay_s=args.step_delay,
                samples=args.samples,
            )
        result = daq.ramp_source_voltage(
            args.target_source_V,
            token=token,
            start_source_V=args.start_source_V,
            step_V=args.source_step_V,
            delay_s=args.source_delay_s,
            read_meters_each_step=not args.source_no_meter_each_step,
            samples=args.samples,
            event_callback=emit_step,
            max_emission_uA=args.max_emission_uA,
            max_source_amps_A=args.max_source_amps_A,
        )
        out = {"simulate": daq.simulate, "source_warmup": warmup_result, "ramp_source": result}
        if args.source_log_prefix is not None:
            out["saved_logs"] = daq.save_source_ramp_log(result, args.source_log_prefix)
        print_yaml(out)
        did = True

    if args.ramp_source_down:
        def emit_step(step):
            meters = step.get("meters", {}) or {}
            sv = meters.get("source_volts", {}).get("value")
            em = meters.get("emission", {}).get("value")
            print(f"source down step {step.get('index')}: target={step.get('target_source_V')} V, source_volts_diag={sv}, emission={em}")
        result = daq.ramp_source_down_to_zero(
            token=token,
            start_source_V=args.start_source_V,
            step_V=args.source_step_V,
            delay_s=args.source_delay_s,
            read_meters_each_step=not args.source_no_meter_each_step,
            samples=args.samples,
            event_callback=emit_step,
            max_emission_uA=args.max_emission_uA,
            max_source_amps_A=args.max_source_amps_A,
        )
        out = {"simulate": daq.simulate, "ramp_source_down": result}
        if args.source_log_prefix is not None:
            out["saved_logs"] = daq.save_source_ramp_log(result, args.source_log_prefix)
        print_yaml(out)
        did = True

    if args.zero_source_only:
        result = daq.zero_source_only(token=token, samples=args.samples)
        print_yaml({"simulate": daq.simulate, "zero_source_only": result})
        did = True

    if args.normal_source_shutdown:
        def emit_step(step):
            meters = step.get("meters", {}) or {}
            sv = meters.get("source_volts", {}).get("value")
            em = meters.get("emission", {}).get("value")
            print(f"normal shutdown source step {step.get('index')}: target={step.get('target_source_V')} V, source_volts_diag={sv}, emission={em}")
        result = daq.normal_source_shutdown(
            token=token,
            start_source_V=args.start_source_V,
            step_V=args.source_step_V,
            delay_s=args.source_delay_s,
            read_meters_each_step=not args.source_no_meter_each_step,
            samples=args.samples,
            abort_callback=None,
            event_callback=emit_step,
            final_settle_s=args.shutdown_final_settle_s,
            final_read_interval_s=args.shutdown_final_read_interval_s,
        )
        out = {"simulate": daq.simulate, "normal_source_shutdown": result}
        if args.source_log_prefix is not None:
            out["saved_logs"] = daq.save_source_ramp_log(result, args.source_log_prefix)
        print_yaml(out)
        did = True

    if not did:
        parser.print_help()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
