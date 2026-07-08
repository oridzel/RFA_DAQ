"""Command-line RBD monitor for lab tests.

Examples:
    python -m rfa_python.apps.rbd_monitor_cli --once
    python -m rfa_python.apps.rbd_monitor_cli --duration 60 --period 1
    python -m rfa_python.apps.rbd_monitor_cli --settle-after-init 5 --discard-first 3 --duration 60
    python -m rfa_python.apps.rbd_monitor_cli --zero-baseline-seconds 5 --duration 60
    python -m rfa_python.apps.rbd_monitor_cli --simulate --duration 10
"""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import time

from rfa_python.instruments.rbd_manager import RBDManager, RBDCsvLogger


def print_results(title: str, results: dict[str, str]) -> None:
    print("\n" + title)
    print("-" * len(title))
    for k, v in results.items():
        print(f"{k:8s}: {v}")


def _format_current(value) -> str:
    if value is None:
        return "None"
    av = abs(value)
    if av >= 1e-3:
        return f"{value*1e3:.6g} mA"
    if av >= 1e-6:
        return f"{value*1e6:.6g} uA"
    if av >= 1e-9:
        return f"{value*1e9:.6g} nA"
    if av >= 1e-12:
        return f"{value*1e12:.6g} pA"
    return f"{value:.6e} A"


def print_offsets(offsets: dict[str, float]) -> None:
    print("\nBaseline offsets")
    print("----------------")
    for name, val in offsets.items():
        print(f"{name:8s}: {_format_current(val)}")


def print_readings(readings, *, corrected: bool = False) -> None:
    for r in readings:
        raw_val = _format_current(r.current_A)
        flags = []
        if r.unstable:
            flags.append("UNSTABLE")
        if r.out_of_range:
            flags.append("OUT_OF_RANGE")
        if r.status_code:
            flags.append(f"status=S{r.status_code}")
        if r.range_code:
            flags.append(f"range={r.range_code}")
        if r.error:
            flags.append(f"ERR={r.error}")
        label = r.display_name or r.name
        value_text = f"current={raw_val:>12s}"
        print(f"{r.timestamp_utc:24s} {label:28s} ({r.name:8s}) {value_text}  {' '.join(flags):45s} raw_response={r.raw_response!r}")


def main() -> None:
    parser = argparse.ArgumentParser(description="RBD 9103 multi-channel monitor")
    parser.add_argument("--config", default="rfa_python/config/rbd_channels.yaml", help="YAML config file")
    parser.add_argument("--simulate", action="store_true", help="Run without hardware using simulated data")
    parser.add_argument("--once", action="store_true", help="Take one sample per enabled channel, then exit")
    parser.add_argument("--duration", type=float, default=0.0, help="Monitor duration in seconds; 0 means until Ctrl+C")
    parser.add_argument("--period", type=float, default=1.0, help="Print/log period in seconds")
    parser.add_argument("--no-init", action="store_true", help="Do not send initialization commands")
    parser.add_argument("--settle-after-init", type=float, default=0.0, help="Wait this many seconds after init/start before logging")
    parser.add_argument("--discard-first", type=int, default=0, help="Discard this many read cycles before logging")
    parser.add_argument("--zero-baseline-seconds", type=float, default=0.0, help="Measure dark-current baseline for this many seconds and subtract it")
    parser.add_argument("--keep-queued", action="store_true", help="Read queued samples instead of flushing to newest sample")
    parser.add_argument("--output-dir", default="data", help="CSV output folder")
    args = parser.parse_args()

    manager = RBDManager.from_yaml(args.config, simulate=args.simulate)

    print_results("Open ports", manager.open_all())
    if not args.no_init:
        print_results("Initialize", manager.initialize_all())

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = Path(args.output_dir) / f"rbd_monitor_{ts}.csv"

    if args.once:
        if args.settle_after_init > 0:
            print(f"\nSettling for {args.settle_after_init:g} s...")
            time.sleep(args.settle_after_init)
        for i in range(args.discard_first):
            _ = manager.sample_all_once()
            print(f"Discarded read cycle {i+1}/{args.discard_first}")
        if args.zero_baseline_seconds > 0:
            print(f"\nMeasuring baseline for {args.zero_baseline_seconds:g} s...")
            print_offsets(manager.measure_baseline(duration_s=args.zero_baseline_seconds, latest=not args.keep_queued))
        readings = manager.sample_all_once()
        print_readings(readings)
        with RBDCsvLogger(output_path) as log:
            log.write_many(readings)
        print(f"\nSaved: {output_path}")
        manager.close_all()
        return

    print_results("Start sampling", manager.start_all())
    if args.settle_after_init > 0:
        print(f"\nSettling for {args.settle_after_init:g} s...")
        time.sleep(args.settle_after_init)
    for i in range(args.discard_first):
        _ = manager.read_all_once(latest=not args.keep_queued)
        print(f"Discarded read cycle {i+1}/{args.discard_first}")
    if args.zero_baseline_seconds > 0:
        print(f"\nMeasuring baseline for {args.zero_baseline_seconds:g} s...")
        print_offsets(manager.measure_baseline(duration_s=args.zero_baseline_seconds, latest=not args.keep_queued))

    deadline = None if args.duration <= 0 else time.time() + args.duration
    print("\nMonitoring. Press Ctrl+C to stop.\n")
    try:
        with RBDCsvLogger(output_path) as log:
            while deadline is None or time.time() < deadline:
                readings = manager.read_all_once(latest=not args.keep_queued)
                print_readings(readings)
                log.write_many(readings)
                time.sleep(args.period)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        manager.stop_all()
        manager.close_all()
        print(f"Saved: {output_path}")


if __name__ == "__main__":
    main()
