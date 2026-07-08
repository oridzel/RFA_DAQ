"""IMS MDrive motion-controller serial driver.

v0.18 scope: conservative bench testing for the RFA lift and temporary
rotation MDrive motors.  This is not yet the final RFA motion/angle layer.

The MDrive command language is ASCII-over-serial.  The manual examples use
commands such as ``PR VM`` to print a variable, ``P=0`` to zero the position
counter, ``MR <steps>`` for relative moves, ``MA <steps>`` for absolute moves,
``SL <steps/s>`` for slew, ``SL 0`` for stop, and ``H`` to hold until the move
finishes.  v0.19 adds calibrated angle LUT moves. v0.18.5 uses ``HI <type>`` for the RFA rotation encoder-index/fiducial
home operation.  The rotation axis is initialized in open-loop motor-count
mode can be checked with ``PR EE``. Do not rewrite stored controller settings during GUI startup, because ``EE 1``
changes motion units to encoder pulses on MDrive controllers.  After each
commanded move/home, the driver waits for motion to complete before reading
position.
"""

from __future__ import annotations

import csv
import re
import time
from pathlib import Path
from dataclasses import dataclass
from typing import Any

from .serial_ascii import SerialAsciiDevice, SerialSettings


MOTION_TOKEN = "I_UNDERSTAND_MOTION"


@dataclass
class MDriveAxisConfig:
    name: str
    role: str
    port: str
    baudrate: int = 9600
    timeout_s: float = 1.0
    has_encoder: bool = False
    steps_per_micron: float | None = None
    max_single_move_steps: int = 10000
    max_slew_steps_per_s: int = 10000
    default_velocity_steps_per_s: int | None = None
    default_initial_velocity_steps_per_s: int | None = None
    home_type: int = 2
    allow_home: bool = False
    enable_encoder_before_home: bool = False
    set_c1_after_home: int | None = 0
    counts_per_revolution: int | None = None
    encoder_counts_per_revolution: int | None = None
    initialize_encoder_enable: bool | None = None
    initialize_c1: int | None = None
    initialize_c2: int | None = None
    initialize_p: int | None = None
    echo_mode: int | None = None
    home_command: str = "HI"
    wait_timeout_s: float = 30.0
    wait_poll_s: float = 0.10


class SimulatedMDrive:
    def __init__(self, name: str = "MDrive", **kwargs: Any) -> None:
        self.name = name
        self.position = 0
        self.encoder_position = 0
        self.encoder_enabled = False
        self.vm = int(kwargs.get("vm", 36000))
        self.vi = int(kwargs.get("vi", 1000))
        self.slewing = 0
        self.is_simulated = True

    def open(self) -> None: pass
    def close(self) -> None: pass
    def flush(self) -> None: pass
    def software_reset(self) -> None: self.stop()
    def command(self, text: str, *, expect_response: bool = False) -> str:
        if text.upper().startswith("PR "):
            return self.print_variable(text.split(maxsplit=1)[1])
        return ""
    def print_variable(self, var: str) -> str:
        v = var.upper().strip()
        if v == "P": return str(self.position)
        if v in ("C1", "P1"): return str(self.position)
        if v in ("C2", "E", "ENC"): return str(self.encoder_position)
        if v == "EE": return "1" if self.encoder_enabled else "0"
        if v == "VM": return str(self.vm)
        if v == "VI": return str(self.vi)
        if v in ("I1", "I2", "I3", "I4"): return "0"
        return "0"
    def set_variable(self, var: str, value: int | float) -> None:
        v = var.upper().strip()
        if v in ("P", "C1", "P1"): self.position = int(value)
        elif v in ("C2", "E", "ENC"): self.encoder_position = int(value)
        elif v == "EE": self.encoder_enabled = bool(int(value))
        elif v == "VM": self.vm = int(value)
        elif v == "VI": self.vi = int(value)
    def move_relative(self, steps: int, *, wait: bool = True, timeout_s: float = 30.0, poll_s: float = 0.10) -> None:
        self.position += int(steps); self.encoder_position = self.position
    def move_absolute(self, steps: int, *, wait: bool = True, timeout_s: float = 30.0, poll_s: float = 0.10) -> None:
        self.position = int(steps); self.encoder_position = self.position
    def slew(self, steps_per_s: int) -> None: self.slewing = int(steps_per_s)
    def hold(self) -> None: pass
    def wait_for_counter1(self, target: int | None = None, *, timeout_s: float = 30.0, poll_s: float = 0.10, stable_samples: int = 3, tolerance: int = 0) -> int:
        return self.query_counter1()
    def stop(self) -> None: self.slewing = 0
    def query_position(self) -> int: return self.position
    def query_encoder_position(self) -> int: return self.encoder_position
    def read_inputs(self) -> dict[str, str]: return {f"I{i}": "0" for i in range(1,5)}
    def enable_encoder(self, enable: bool = True) -> None:
        self.encoder_enabled = bool(enable)
    def initialize_axis(self, *, encoder_enable: bool | None = None, counter1: int | None = None, counter2: int | None = None, position_p: int | None = None, echo_mode: int | None = None) -> dict[str, Any]:
        if encoder_enable is not None:
            self.enable_encoder(bool(encoder_enable))
        if counter1 is not None:
            self.set_counter1(int(counter1))
        if counter2 is not None:
            self.set_counter2(int(counter2))
        if position_p is not None:
            self.set_variable("P", int(position_p))
        return self.status()
    def set_counter1(self, value: int = 0) -> None:
        self.set_variable("C1", int(value))
    def set_counter2(self, value: int = 0) -> None:
        self.set_variable("C2", int(value))
    def query_counter1(self) -> int:
        return self.position
    def home_to_switch(self, home_type: int = 2, *, wait: bool = True, timeout_s: float = 60.0, poll_s: float = 0.10) -> None:
        self.position = 0; self.encoder_position = 0
    def home_to_index(self, home_type: int = 2, *, wait: bool = True, timeout_s: float = 60.0, poll_s: float = 0.10) -> None:
        self.position = 0; self.encoder_position = 0
    def zero_position(self) -> None:
        self.set_counter1(0); self.set_counter2(0)
    def home_rotation_repeatable(self, *, overshoot_steps: int = 2000, home_type: int = 2, wait: bool = True, timeout_s: float = 60.0, poll_s: float = 0.10) -> None:
        current = self.query_counter1()
        if current == 0:
            return
        if current < int(overshoot_steps):
            self.move_absolute(int(overshoot_steps), wait=wait, timeout_s=timeout_s, poll_s=poll_s)
        self.home_to_index(home_type, wait=wait, timeout_s=timeout_s, poll_s=poll_s)
        self.zero_position()
    def status(self) -> dict[str, Any]:
        return {"name": self.name, "position_steps": self.position, "counter1_steps": self.position, "encoder_steps": self.encoder_position, "encoder_enabled": self.encoder_enabled, "VM": self.vm, "VI": self.vi, "slew_steps_per_s": self.slewing, "simulated": True}


class MDrive(SerialAsciiDevice):
    def __init__(
        self,
        port: str,
        *,
        name: str = "MDrive",
        baudrate: int = 9600,
        timeout_s: float = 1.0,
    ) -> None:
        # IMS Terminal sends commands when Enter is pressed.  CR/LF is safe for
        # most serial adapters and works like a terminal newline.
        super().__init__(SerialSettings(port=port, baudrate=baudrate, timeout_s=timeout_s, terminator="\r\n"), simulate=False)
        self.name = name
        self.is_simulated = False

    def command(self, text: str, *, expect_response: bool = False) -> str:
        if expect_response:
            return self.query(text)
        self.write(text)
        return ""

    def query(self, command: str, *, delay_s: float = 0.08, flush_input: bool = True) -> str:  # type: ignore[override]
        self.open()
        if flush_input:
            self.flush_input()
        self.write(command)
        if delay_s:
            time.sleep(delay_s)
        # MDrive may echo commands depending on EM setting. Read until a line
        # that is not just the command/empty, or until timeout.
        end = time.time() + max(0.25, self.settings.timeout_s)
        last = ""
        while time.time() < end:
            line = self.read_line()
            if not line:
                continue
            last = line
            if line.strip().upper() != command.strip().upper():
                return line
        return last

    def software_reset(self) -> None:
        # The manual says Ctrl-C can be used as software reset/stop-like control.
        self.open()
        assert self._serial is not None
        self._serial.write(b"\x03")
        self._serial.flush()

    def print_variable(self, var: str) -> str:
        return self.query(f"PR {var}")

    def _parse_int(self, text: str) -> int:
        m = re.search(r"[-+]?\d+", str(text))
        if not m:
            raise ValueError(f"Could not parse integer from MDrive response {text!r}")
        return int(m.group(0))

    def query_position(self) -> int:
        # LabVIEW used Counter 1 for the motor-position readback.  Keep P as
        # a fallback because some MDrive configurations expose position there.
        try:
            return self.query_counter1()
        except Exception:
            return self._parse_int(self.print_variable("P"))

    def query_counter1(self) -> int:
        return self._parse_int(self.print_variable("C1"))

    def query_encoder_enabled(self) -> int:
        return self._parse_int(self.print_variable("EE"))

    def query_encoder_position(self) -> int:
        # Closed-loop drives often expose encoder/counter as C2.  If that fails,
        # return P so the status command remains useful during bench testing.
        try:
            return self._parse_int(self.print_variable("C2"))
        except Exception:
            return self.query_position()

    def wait_for_counter1(
        self,
        target: int | None = None,
        *,
        timeout_s: float = 30.0,
        poll_s: float = 0.10,
        stable_samples: int = 3,
        tolerance: int = 0,
    ) -> int:
        """Wait until Counter 1 reaches a target or becomes stable.

        The MDrive manual shows ``H`` after ``MR``/``MA`` in programs, but in
        immediate serial control our host can still query too quickly.  For the
        LabVIEW-replacement workflow we therefore explicitly poll C1 after a
        move before returning status.  If a target is known, wait for C1 to be
        within tolerance of that target.  If no target is known, wait until C1
        is unchanged for ``stable_samples`` polls.
        """
        deadline = time.time() + float(timeout_s)
        last: int | None = None
        stable = 0
        last_error: Exception | None = None
        # Give the controller a moment to start the motion before declaring it
        # stable, especially for home-to-index moves.
        time.sleep(min(float(poll_s), 0.20))
        while time.time() < deadline:
            try:
                pos = self.query_counter1()
                last_error = None
            except Exception as exc:
                last_error = exc
                time.sleep(float(poll_s))
                continue

            if target is not None and abs(pos - int(target)) <= int(tolerance):
                return pos

            if last is not None and pos == last:
                stable += 1
            else:
                stable = 0
            last = pos

            if target is None and stable >= int(stable_samples):
                return pos
            time.sleep(float(poll_s))

        if last is not None:
            if target is None:
                raise TimeoutError(f"Timed out waiting for C1 to become stable; last C1={last}")
            raise TimeoutError(f"Timed out waiting for C1 to reach {target}; last C1={last}")
        raise TimeoutError(f"Timed out waiting for C1; last error={last_error}")

    def set_variable(self, var: str, value: int | float) -> None:
        # This MDrive setup expects parameter writes without an equals sign
        # (e.g. EE 0, C1 0, C2 0, HI 2). LabVIEW/vendor blocks obscure this
        # by naming parameters HM/EE/C1, but the actual serial command uses a space.
        self.write(f"{var} {value}")
        # Give the controller a short moment to apply write-only parameter changes
        # before a follow-up PR query. This matters for EE/C1 initialization.
        time.sleep(0.05)

    def initialize_axis(self, *, encoder_enable: bool | None = None, counter1: int | None = None, counter2: int | None = None, position_p: int | None = None, echo_mode: int | None = None) -> dict[str, Any]:
        """Optionally apply explicitly configured settings and return status.

        On the installed RFA motors, GUI startup and ordinary Initialize should
        be read-only by default: do not zero C1/C2, do not rewrite VM/VI, and do
        not alter stored limit-switch or encoder settings.  Any writes occur
        only if the config explicitly provides non-null values.
        """
        if echo_mode is not None:
            self.set_variable("EM", int(echo_mode))
        if encoder_enable is not None:
            self.enable_encoder(bool(encoder_enable))
            time.sleep(0.10)
        if counter1 is not None:
            self.set_counter1(int(counter1))
            time.sleep(0.05)
        if counter2 is not None:
            self.set_counter2(int(counter2))
            time.sleep(0.05)
        if position_p is not None:
            self.set_variable("P", int(position_p))
            time.sleep(0.05)
        return self.status()

    def set_counter1(self, value: int = 0) -> None:
        self.write(f"C1 {int(value)}")

    def set_counter2(self, value: int = 0) -> None:
        self.write(f"C2 {int(value)}")

    def enable_encoder(self, enable: bool = True) -> None:
        self.write(f"EE {1 if enable else 0}")
        time.sleep(0.10)

    def zero_position(self) -> None:
        # Keep both counters in sync for bench testing.  C1 is the important
        # counter for our LabVIEW replacement; C2 is encoder/index counter.
        self.set_counter1(0)
        self.set_counter2(0)

    def move_relative(self, steps: int, *, wait: bool = True, timeout_s: float = 30.0, poll_s: float = 0.10) -> None:
        target: int | None = None
        if wait:
            try:
                target = self.query_counter1() + int(steps)
            except Exception:
                target = None
        self.write(f"MR {int(steps)}")
        if wait:
            self.write("H")
            # Prefer target wait for normal MR.  If target could not be read,
            # fall back to stability polling.
            self.wait_for_counter1(target, timeout_s=timeout_s, poll_s=poll_s)

    def move_absolute(self, steps: int, *, wait: bool = True, timeout_s: float = 30.0, poll_s: float = 0.10) -> None:
        self.write(f"MA {int(steps)}")
        if wait:
            self.write("H")
            self.wait_for_counter1(int(steps), timeout_s=timeout_s, poll_s=poll_s)

    def slew(self, steps_per_s: int) -> None:
        self.write(f"SL {int(steps_per_s)}")

    def hold(self) -> None:
        self.write("H")

    def stop(self) -> None:
        self.write("SL 0")

    def home_to_switch(self, home_type: int = 2, *, wait: bool = True, timeout_s: float = 60.0, poll_s: float = 0.10) -> None:
        # Home to external home switch / limit input. Not the normal bench
        # rotation fiducial/index operation.
        self.write(f"HM {int(home_type)}")
        if wait:
            self.write("H")
            self.wait_for_counter1(None, timeout_s=timeout_s, poll_s=poll_s, stable_samples=5)

    def home_to_index(self, home_type: int = 2, *, wait: bool = True, timeout_s: float = 60.0, poll_s: float = 0.10) -> None:
        # Home to encoder index mark / fiducial. This is the command used for
        # the temporary RFA rotation MDrive bench motor.
        self.write(f"HI {int(home_type)}")
        if wait:
            self.write("H")
            self.wait_for_counter1(None, timeout_s=timeout_s, poll_s=poll_s, stable_samples=5)

    def home_rotation_repeatable(self, *, overshoot_steps: int = 2000, home_type: int = 2, wait: bool = True, timeout_s: float = 60.0, poll_s: float = 0.10) -> None:
        """Backlash-controlled rotation home sequence.

        Read C1 first and then:
          - if C1 == 0, assume we are already at the repeatable home and do nothing;
          - if C1 >= overshoot_steps, run HI directly;
          - if C1 < overshoot_steps, move absolute to overshoot_steps first,
            then run HI.

        This ensures the index/fiducial is approached from the same side while
        avoiding an unnecessary home operation when C1 already equals zero.
        After successful HI home, both C1 and C2 are zeroed.
        """
        current = self.query_counter1()
        if current == 0:
            return
        if current < int(overshoot_steps):
            self.move_absolute(int(overshoot_steps), wait=wait, timeout_s=timeout_s, poll_s=poll_s)
        self.home_to_index(int(home_type), wait=wait, timeout_s=timeout_s, poll_s=poll_s)
        self.zero_position()

    def read_inputs(self) -> dict[str, str]:
        out: dict[str, str] = {}
        for i in range(1, 5):
            try:
                out[f"I{i}"] = self.print_variable(f"I{i}")
            except Exception as exc:
                out[f"I{i}"] = f"ERROR: {exc}"
        return out

    def status(self) -> dict[str, Any]:
        data: dict[str, Any] = {"name": self.name, "simulated": False}
        for key, var in [("position_p_steps", "P"), ("counter1_steps", "C1"), ("VM", "VM"), ("VI", "VI"), ("encoder_enabled", "EE")]:
            try:
                data[key] = self._parse_int(self.print_variable(var))
            except Exception as exc:
                data[key] = None
                data[f"{key}_error"] = str(exc)
        data["position_steps"] = data.get("counter1_steps", data.get("position_p_steps"))
        try:
            data["encoder_steps"] = self.query_encoder_position()
        except Exception as exc:
            data["encoder_steps"] = None
            data["encoder_steps_error"] = str(exc)
        return data


def axis_steps_from_microns(microns: float, steps_per_micron: float | None) -> int:
    if steps_per_micron is None:
        raise ValueError("This axis does not have steps_per_micron configured; use raw steps instead.")
    return int(round(float(microns) * float(steps_per_micron)))


def counts_from_degrees(degrees: float, counts_per_revolution: int | None) -> int:
    if not counts_per_revolution:
        raise ValueError("This axis does not have counts_per_revolution configured; use raw steps/counts instead.")
    return int(round(float(degrees) * float(counts_per_revolution) / 360.0))

def degrees_from_counts(counts: int, counts_per_revolution: int | None) -> float:
    if not counts_per_revolution:
        raise ValueError("This axis does not have counts_per_revolution configured.")
    return float(counts) * 360.0 / float(counts_per_revolution)


class AngleStepLUT:
    """Calibrated angle-to-microstep interpolation table.

    CSV columns: angle, steps.  Angles may be descending; interpolation is
    handled after sorting by angle.  Steps are absolute C1 microstep positions
    relative to the homed/zeroed fiducial.
    """

    def __init__(self, rows: list[tuple[float, float]]) -> None:
        if len(rows) < 2:
            raise ValueError("Angle LUT must contain at least two rows")
        self.rows = sorted((float(a), float(s)) for a, s in rows)

    @classmethod
    def from_csv(cls, path: str | Path) -> "AngleStepLUT":
        rows: list[tuple[float, float]] = []
        with open(path, "r", encoding="utf-8-sig", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                if not row:
                    continue
                a = row.get("angle") or row.get("Angle") or row.get("angle_deg")
                st = row.get("steps") or row.get("Steps") or row.get("microsteps")
                if a is None or st is None:
                    continue
                rows.append((float(a), float(st)))
        return cls(rows)

    @property
    def min_angle(self) -> float:
        return self.rows[0][0]

    @property
    def max_angle(self) -> float:
        return self.rows[-1][0]

    def steps_for_angle(self, angle_deg: float) -> int:
        x = float(angle_deg)
        if x < self.min_angle or x > self.max_angle:
            raise ValueError(f"Requested angle {x:g} deg is outside LUT range {self.min_angle:g}..{self.max_angle:g} deg")
        rows = self.rows
        for i, (a, s) in enumerate(rows):
            if x == a:
                return int(round(s))
            if i + 1 < len(rows):
                a2, s2 = rows[i + 1]
                if a <= x <= a2:
                    t = (x - a) / (a2 - a)
                    return int(round(s + t * (s2 - s)))
        return int(round(rows[-1][1]))

    def angle_for_steps(self, steps: int | float) -> float:
        y = float(steps)
        rows = sorted((s, a) for a, s in self.rows)
        if y < rows[0][0] or y > rows[-1][0]:
            raise ValueError(f"Steps {y:g} are outside LUT range {rows[0][0]:g}..{rows[-1][0]:g}")
        for i, (s, a) in enumerate(rows):
            if y == s:
                return float(a)
            if i + 1 < len(rows):
                s2, a2 = rows[i + 1]
                if s <= y <= s2:
                    t = (y - s) / (s2 - s)
                    return float(a + t * (a2 - a))
        return float(rows[-1][1])
