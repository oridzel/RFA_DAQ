"""Command-line tester for IMS MDrive motion controllers.

v0.18.3 is for bench testing only.  It uses small raw-step/count commands and
explicit safety acknowledgement for any motion.  Rotation homing now defaults
to the encoder index command HI; HM is kept as a separate home-switch diagnostic.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any
import yaml

from rfa_python.instruments.mdrive import (
    MDrive,
    SimulatedMDrive,
    MOTION_TOKEN,
    axis_steps_from_microns,
    counts_from_degrees,
    degrees_from_counts,
    AngleStepLUT,
)


def default_config_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "mdrive.yaml")

def default_angle_lut_path() -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / "angles-steps-LUT.csv")


def load_config(path: str | Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def axis_cfg(root: dict[str, Any], axis: str) -> dict[str, Any]:
    axes = root.get("axes", {}) or {}
    if axis not in axes:
        raise SystemExit(f"Unknown axis {axis!r}. Available: {', '.join(sorted(axes))}")
    cfg = dict(axes[axis] or {})
    cfg["name"] = axis
    defaults = root.get("serial_defaults", {}) or {}
    safety = root.get("motion_safety", {}) or {}
    cfg.setdefault("baudrate", defaults.get("baudrate", 9600))
    cfg.setdefault("timeout_s", defaults.get("timeout_s", 1.0))
    cfg.setdefault("max_single_move_steps", safety.get("max_single_move_steps", 10000))
    cfg.setdefault("max_slew_steps_per_s", safety.get("max_slew_steps_per_s", 10000))
    return cfg


def build_device(root: dict[str, Any], axis: str, simulate_override: bool | None):
    cfg = axis_cfg(root, axis)
    simulate = bool(root.get("simulate", False)) if simulate_override is None else bool(simulate_override)
    if simulate:
        return SimulatedMDrive(name=axis), cfg
    return MDrive(
        port=str(cfg.get("port", "COM1")),
        name=axis,
        baudrate=int(cfg.get("baudrate", 9600)),
        timeout_s=float(cfg.get("timeout_s", 1.0)),
    ), cfg


def require_motion(args: argparse.Namespace, root: dict[str, Any]) -> None:
    safety = root.get("motion_safety", {}) or {}
    required = bool(safety.get("require_enable_token", True))
    wants_motion = any([
        args.move_steps is not None,
        args.move_microns is not None,
        args.move_absolute is not None,
        args.move_deg is not None,
        args.slew is not None,
        args.stop,
        args.zero_position,
        args.home,
        args.home_index,
        args.home_switch,
        args.enable_encoder,
        args.disable_encoder,
        args.set_c1 is not None,
        args.zero_c1,
        getattr(args, "set_c2", None) is not None,
        getattr(args, "zero_c2", False),
        args.ctrl_c,
        args.init_axis,
    ])
    if required and wants_motion and not args.i_understand_motion:
        raise SystemExit(f"Motion command refused. Add --i-understand-motion after verifying the bench setup. Token: {MOTION_TOKEN}")


def check_limits(steps: int, cfg: dict[str, Any], what: str = "move") -> None:
    limit = int(cfg.get("max_single_move_steps", 10000))
    if abs(int(steps)) > limit:
        raise SystemExit(f"Requested {what} {steps} steps exceeds configured max_single_move_steps={limit}.")


def axis_user_units_to_microsteps(axis: str, value: int | float, cfg: dict[str, Any]) -> tuple[int, dict[str, Any]]:
    """Convert user-facing motion units to raw MDrive microsteps.

    Lift is always entered in microns and multiplied by the configured scale
    (100 microsteps/micron for the real RFA lift). Rotation remains raw
    microsteps for signed relative/absolute step commands; calibrated angle
    moves use the LUT separately.
    """
    scale = int(cfg.get("manual_move_scale", cfg.get("steps_per_micron", 1)) or 1)
    units = str(cfg.get("manual_move_units_label", "microsteps"))
    motor_steps = int(round(float(value) * scale))
    return motor_steps, {"entered": value, "scale": scale, "motor_microsteps": motor_steps, "units": units}


def manual_relative_steps(axis: str, requested_steps: int, cfg: dict[str, Any]) -> tuple[int, dict[str, Any]]:
    return axis_user_units_to_microsteps(axis, requested_steps, cfg)


def configured_lift_target_microsteps(cfg: dict[str, Any], which: str) -> tuple[int, dict[str, Any]]:
    """Return lift open/closed preset in raw MDrive microsteps.

    Prefer user-facing micron config keys and scale them; fall back to legacy
    raw *_position_steps keys for compatibility with older configs.
    """
    micron_key = "open_position_microns" if which == "open" else "closed_position_microns"
    step_key = "open_position_steps" if which == "open" else "closed_position_steps"
    if micron_key in cfg:
        target, info = axis_user_units_to_microsteps("lift", cfg[micron_key], cfg)
        info["source"] = micron_key
        return target, info
    target = int(cfg[step_key])
    return target, {"entered": target, "scale": 1, "motor_microsteps": target, "units": "microsteps", "source": step_key}


def check_slew_limit(steps_per_s: int, cfg: dict[str, Any]) -> None:
    limit = int(cfg.get("max_slew_steps_per_s", 10000))
    if abs(int(steps_per_s)) > limit:
        raise SystemExit(f"Requested slew {steps_per_s} steps/s exceeds configured max_slew_steps_per_s={limit}.")


def print_yaml(data: Any) -> None:
    print(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))


def main() -> None:
    p = argparse.ArgumentParser(description="Bench-test/control IMS MDrive motion controllers")
    p.add_argument("--config", default=default_config_path())
    p.add_argument("--axis", choices=["lift", "rotation"], default="lift")
    p.add_argument("--simulate", action="store_true")
    p.add_argument("--real", action="store_true")

    p.add_argument("--status", action="store_true", help="read position/velocity status")
    p.add_argument("--init-axis", action="store_true", help="read-only by default; applies only non-null configured startup settings, then reads status")
    p.add_argument("--position", action="store_true", help="query PR P")
    p.add_argument("--encoder", action="store_true", help="query encoder/counter if available")
    p.add_argument("--inputs", action="store_true", help="query I1..I4 diagnostic inputs")
    p.add_argument("--zero-position", action="store_true", help="set P=0")
    p.add_argument("--move-steps", type=int, help="MR relative move. For lift enter microns; software multiplies by 100 microsteps/micron. For rotation enter raw microsteps.")
    p.add_argument("--move-relative", type=int, dest="move_steps", help=argparse.SUPPRESS)
    p.add_argument("--move-microns", type=float, help="MR distance using configured steps_per_micron, if available")
    p.add_argument("--move-deg", type=float, help="rotation only: relative move in degrees using simple counts_per_revolution; diagnostic only")
    p.add_argument("--angle-lut", default=None, help="rotation angle LUT CSV; default from config or config/angles-steps-LUT.csv")
    p.add_argument("--move-angle", type=float, help="rotation only: absolute calibrated angle move using angle-step LUT")
    p.add_argument("--angle", type=float, dest="move_angle", help=argparse.SUPPRESS)
    p.add_argument("--show-angle-lut", action="store_true", help="print configured angle-step LUT path/range")
    p.add_argument("--home-repeatable", action="store_true", help="rotation only: move to +overshoot steps, HI home, then C1/C2 zero")
    p.add_argument("--home-overshoot-steps", type=int, default=None, help="positive-side overshoot position before HI home, default from config")
    p.add_argument("--lift-open", action="store_true", help="lift axis: move absolute to configured open position")
    p.add_argument("--lift-close", action="store_true", help="lift axis: move absolute to configured closed position")
    p.add_argument("--move-absolute", type=int, help="MA absolute target. For lift enter microns; software multiplies by 100 microsteps/micron. For rotation enter raw C1 microsteps.")
    p.add_argument("--slew", type=int, help="SL steps/s; use 0 to stop")
    p.add_argument("--stop", action="store_true", help="send SL 0")
    p.add_argument("--set-vm", type=int, help="set VM maximum velocity")
    p.add_argument("--set-vi", type=int, help="set VI initial/creep velocity")
    p.add_argument("--home", action="store_true", help="rotation only: normal home to encoder index, HI <type>, then set C1/C2 as configured")
    p.add_argument("--home-index", action="store_true", help="same as --home: HI <type>, find encoder index mark")
    p.add_argument("--home-switch", action="store_true", help="diagnostic only: HM <type>, home to external switch/input")
    p.add_argument("--home-type", type=int, default=None, help="HM/HI type, default from config")
    p.add_argument("--enable-encoder", action="store_true", help="send EE 1")
    p.add_argument("--disable-encoder", action="store_true", help="send EE 0")
    p.add_argument("--set-c1", type=int, help="set Counter 1 using C1 <value>")
    p.add_argument("--zero-c1", action="store_true", help="set Counter 1 to zero using C1 0")
    p.add_argument("--set-c2", type=int, help="set Counter 2 / encoder counter using C2 <value>")
    p.add_argument("--zero-c2", action="store_true", help="set Counter 2 / encoder counter to zero using C2 0")
    p.add_argument("--ctrl-c", action="store_true", help="send Ctrl-C software stop/reset")
    p.add_argument("--wait-timeout", type=float, default=None, help="seconds to wait for C1 after motion, default from config")
    p.add_argument("--wait-poll", type=float, default=None, help="seconds between C1 polls while waiting, default from config")
    p.add_argument("--i-understand-motion", action="store_true")
    args = p.parse_args()

    root = load_config(args.config)
    require_motion(args, root)
    simulate_override = True if args.simulate else (False if args.real else None)
    dev, cfg = build_device(root, args.axis, simulate_override)

    if not bool(cfg.get("enabled", True)):
        raise SystemExit(f"Axis {args.axis!r} is disabled in config.")

    dev.open()
    try:
        actions: dict[str, Any] = {"axis": args.axis, "port": cfg.get("port"), "simulate": bool(getattr(dev, "is_simulated", False))}
        wait_timeout = float(args.wait_timeout if args.wait_timeout is not None else cfg.get("wait_timeout_s", root.get("motion_safety", {}).get("default_wait_timeout_s", 30.0)))
        wait_poll = float(args.wait_poll if args.wait_poll is not None else cfg.get("wait_poll_s", root.get("motion_safety", {}).get("default_wait_poll_s", 0.10)))
        angle_lut = None
        angle_lut_path = None
        if args.axis == "rotation":
            angle_lut_path = args.angle_lut or cfg.get("angle_lut_csv") or default_angle_lut_path()
            if angle_lut_path and not Path(str(angle_lut_path)).is_absolute():
                angle_lut_path = str(Path(args.config).resolve().parent / str(angle_lut_path))
            try:
                angle_lut = AngleStepLUT.from_csv(angle_lut_path)
            except Exception as exc:
                if args.move_angle is not None or args.show_angle_lut:
                    raise SystemExit(f"Could not load angle LUT {angle_lut_path!r}: {exc}")

        if args.show_angle_lut:
            if args.axis != "rotation":
                raise SystemExit("--show-angle-lut is only valid for rotation")
            actions["angle_lut"] = {"path": str(angle_lut_path), "min_angle": angle_lut.min_angle, "max_angle": angle_lut.max_angle, "rows": [{"angle": a, "steps": st} for a, st in angle_lut.rows]}

        if args.init_axis:
            enc = cfg.get("initialize_encoder_enable", None)
            c1 = cfg.get("initialize_c1", None)
            c2 = cfg.get("initialize_c2", None)
            ppos = cfg.get("initialize_p", None)
            echo = cfg.get("echo_mode", None)
            status_after_init = dev.initialize_axis(encoder_enable=enc, counter1=c1, counter2=c2, position_p=ppos, echo_mode=echo)
            actions["init_axis"] = {"encoder_enable": enc, "C1": c1, "C2": c2, "P": ppos, "EM": echo, "status_after_init": status_after_init}
        if args.ctrl_c:
            dev.software_reset(); actions["ctrl_c"] = "sent"
        if args.set_vm is not None:
            dev.set_variable("VM", args.set_vm); actions["set_VM"] = args.set_vm
        if args.set_vi is not None:
            dev.set_variable("VI", args.set_vi); actions["set_VI"] = args.set_vi
        if args.enable_encoder:
            dev.enable_encoder(True); actions["enable_encoder"] = "EE 1"
        if args.disable_encoder:
            dev.enable_encoder(False); actions["disable_encoder"] = "EE 0"
        if args.set_c1 is not None:
            dev.set_counter1(int(args.set_c1)); actions["set_C1"] = int(args.set_c1)
        if args.zero_c1:
            dev.set_counter1(0); actions["set_C1"] = 0
        if getattr(args, "set_c2", None) is not None:
            dev.set_counter2(int(args.set_c2)); actions["set_C2"] = int(args.set_c2)
        if getattr(args, "zero_c2", False):
            dev.set_counter2(0); actions["set_C2"] = 0
        if args.zero_position:
            dev.zero_position(); actions["zero_position"] = "C1 0 and C2 0"
        if args.home_repeatable:
            if args.axis != "rotation":
                raise SystemExit("--home-repeatable is only valid for rotation")
            if not bool(cfg.get("allow_home", False)):
                raise SystemExit("repeatable homing is disabled in config")
            home_type = int(args.home_type if args.home_type is not None else cfg.get("home_type", 2))
            overshoot = int(args.home_overshoot_steps if args.home_overshoot_steps is not None else cfg.get("home_overshoot_steps", 2000))
            dev.home_rotation_repeatable(overshoot_steps=overshoot, home_type=home_type, timeout_s=max(60.0, wait_timeout), poll_s=wait_poll)
            actions["home_repeatable"] = {"move_absolute_overshoot_steps": overshoot, "HI": home_type, "zeroed": "C1 0 and C2 0"}
        if args.lift_open or args.lift_close:
            if args.axis != "lift":
                raise SystemExit("--lift-open/--lift-close are only valid for lift")
            which = "open" if args.lift_open else "close"
            target, info = configured_lift_target_microsteps(cfg, which)
            dev.move_absolute(target, timeout_s=wait_timeout, poll_s=wait_poll)
            actions["lift_open" if args.lift_open else "lift_close"] = {"target_microsteps": target, **info}
        if args.move_microns is not None:
            steps = axis_steps_from_microns(args.move_microns, cfg.get("steps_per_micron"))
            check_limits(steps, cfg, "move_microns")
            dev.move_relative(steps, timeout_s=wait_timeout, poll_s=wait_poll); actions["move_microns"] = {"microns": args.move_microns, "steps": steps}
        if args.move_deg is not None:
            if args.axis != "rotation":
                raise SystemExit("--move-deg is only valid for the rotation axis")
            counts = counts_from_degrees(args.move_deg, cfg.get("counts_per_revolution"))
            check_limits(counts, cfg, "move_deg")
            dev.move_relative(counts, timeout_s=wait_timeout, poll_s=wait_poll); actions["move_deg"] = {"relative_degrees": args.move_deg, "motor_counts": counts, "note": "diagnostic simple conversion, not calibrated LUT"}
        if args.move_angle is not None:
            if args.axis != "rotation":
                raise SystemExit("--move-angle/--angle is only valid for the rotation axis")
            if angle_lut is None:
                raise SystemExit("No rotation angle LUT loaded")
            target_steps = angle_lut.steps_for_angle(float(args.move_angle))
            check_limits(target_steps, cfg, "move_angle")
            dev.move_absolute(target_steps, timeout_s=wait_timeout, poll_s=wait_poll)
            actions["move_angle"] = {"angle_deg": args.move_angle, "target_microsteps_C1": target_steps, "lut": str(angle_lut_path)}
        if args.move_steps is not None:
            motor_steps, info = manual_relative_steps(args.axis, int(args.move_steps), cfg)
            check_limits(motor_steps, cfg)
            dev.move_relative(motor_steps, timeout_s=wait_timeout, poll_s=wait_poll); actions["move_steps"] = info
        if args.move_absolute is not None:
            target_steps, info = axis_user_units_to_microsteps(args.axis, args.move_absolute, cfg)
            check_limits(target_steps, cfg, "move_absolute")
            dev.move_absolute(target_steps, timeout_s=wait_timeout, poll_s=wait_poll); actions["move_absolute"] = info
        if args.slew is not None:
            check_slew_limit(args.slew, cfg)
            dev.slew(args.slew); actions["slew_steps_per_s"] = args.slew
        if args.stop:
            dev.stop(); actions["stop"] = "SL 0"
        if args.home or args.home_index:
            if not bool(cfg.get("has_encoder", False)) or not bool(cfg.get("allow_home", False)):
                raise SystemExit("home/home-index is only allowed for an axis with has_encoder: true and allow_home: true")
            home_type = int(args.home_type if args.home_type is not None else cfg.get("home_type", cfg.get("home_index_type", 2)))
            # Do not send EE automatically. LabVIEW/vendor VI did not appear to
            # require it for HI, and EE changes the interpretation of motion units.
            if bool(cfg.get("enable_encoder_before_home", False)):
                dev.enable_encoder(True); actions["enable_encoder_before_home"] = "EE 1"
            dev.home_to_index(home_type, timeout_s=max(60.0, wait_timeout), poll_s=wait_poll); actions["home_index"] = {"HI": home_type, "syntax": f"HI {home_type}"}
            if "set_c1_after_home" in cfg and cfg.get("set_c1_after_home") is not None:
                c1 = int(cfg.get("set_c1_after_home", 0))
                dev.set_counter1(c1); actions["set_C1_after_home"] = c1
        if args.home_switch:
            if not bool(cfg.get("allow_home_switch", False)):
                raise SystemExit("home-switch/HM is disabled in config; use --home for encoder-index HI homing")
            home_type = int(args.home_type if args.home_type is not None else cfg.get("home_switch_type", cfg.get("home_type", 2)))
            dev.home_to_switch(home_type, timeout_s=max(60.0, wait_timeout), poll_s=wait_poll); actions["home_switch_diagnostic"] = {"HM": home_type, "syntax": f"HM {home_type}"}

        # Always report status after actions unless user explicitly asks only inputs.
        if args.inputs:
            actions["inputs"] = dev.read_inputs()
        if args.position:
            actions["position_steps"] = dev.query_position()
        if args.encoder:
            actions["encoder_steps"] = dev.query_encoder_position()
        if args.status or not any([args.position, args.encoder, args.inputs]) or len(actions) > 3:
            actions["status"] = dev.status()
            if args.axis == "rotation" and cfg.get("counts_per_revolution"):
                c1 = actions["status"].get("counter1_steps", actions["status"].get("position_steps"))
                if c1 is not None:
                    try:
                        actions["status"]["position_deg_from_C1_motor_counts"] = degrees_from_counts(int(c1), cfg.get("counts_per_revolution"))
                    except Exception:
                        pass
                    if angle_lut is not None:
                        try:
                            actions["status"]["calibrated_angle_deg_from_C1"] = angle_lut.angle_for_steps(int(c1))
                        except Exception as exc:
                            actions["status"]["calibrated_angle_deg_from_C1_error"] = str(exc)
                c2 = actions["status"].get("encoder_steps")
                if c2 is not None and cfg.get("encoder_counts_per_revolution"):
                    try:
                        actions["status"]["encoder_deg_from_C2"] = degrees_from_counts(int(c2), cfg.get("encoder_counts_per_revolution"))
                    except Exception:
                        pass
        print_yaml(actions)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
