"""Stanford Research Systems DC205 precision voltage source driver.

Conservative Python driver for early RFA DAQ testing. It implements only the
commands we need first: identify, reset, set/query voltage setpoint, range,
output state, interlock/overload, errors, and raw-command debugging.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
import re
from typing import Any, Optional

from .serial_ascii import SerialAsciiDevice, SerialSettings


RANGE_LIMITS = {"RANGE1": 1.010000, "RANGE10": 10.10000, "RANGE100": 101.0000}
RANGE_TO_VOLTS = {"RANGE1": 1.0, "RANGE10": 10.0, "RANGE100": 100.0}


@dataclass
class DC205Status:
    idn: Optional[str] = None
    range: Optional[str] = None
    voltage_setpoint_V: Optional[float] = None
    output_on: Optional[bool] = None
    interlock_closed: Optional[bool] = None
    overload: Optional[bool] = None
    last_execution_error: Optional[str] = None
    last_command_error: Optional[str] = None
    raw: dict[str, str] | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class SimulatedDC205:
    def __init__(self, name: str = "SRS DC205", **_: Any) -> None:
        self.name = name
        self.voltage_V = 0.0
        self.range = "RANGE1"
        self.output_on = False
        self.interlock_closed = False
        self.simulate = True

    def open(self) -> None: pass
    def close(self) -> None: pass
    def identify(self) -> str: return "Stanford Research Systems,DC205,s/nSIM00000,ver1.00"
    def set_termination_lf(self) -> None: pass
    def set_token_mode(self, on: bool = True) -> None: pass
    def clear_status(self) -> None: pass
    def raw(self, command: str) -> str: return self._simulate_query(command)
    def query_range(self) -> str: return self.range
    def set_range(self, range_name: str | int | float) -> None:
        rng = normalize_user_range(range_name)
        if self.output_on:
            raise RuntimeError("Cannot change range while output is on.")
        self.range = rng
    def query_voltage(self) -> float: return self.voltage_V
    def query_voltage_setpoint(self) -> float: return self.query_voltage()
    def set_voltage(self, voltage_V: float) -> None:
        limit = RANGE_LIMITS[self.range]
        if abs(voltage_V) > limit:
            raise ValueError(f"{voltage_V} V exceeds {self.range} limit ±{limit} V")
        self.voltage_V = float(voltage_V)
    def set_output(self, on: bool) -> None:
        if on and self.range == "RANGE100" and not self.interlock_closed:
            raise RuntimeError("Refusing SOUT ON in RANGE100 with open interlock.")
        self.output_on = bool(on)
    def query_output(self) -> bool: return self.output_on
    def query_interlock(self) -> bool: return self.interlock_closed
    def query_overload(self) -> bool: return False
    def read_errors(self) -> tuple[str, str]: return ("0", "0")
    def reset_default(self) -> None:
        self.voltage_V = 0.0
        self.range = "RANGE1"
        self.output_on = False
    def status(self) -> DC205Status:
        return DC205Status(
            idn=self.identify(), range=self.range, voltage_setpoint_V=self.voltage_V,
            output_on=self.output_on, interlock_closed=self.interlock_closed,
            overload=False, last_execution_error="0", last_command_error="0",
            raw={"SIMULATED": "true"}
        )
    def _simulate_query(self, command: str) -> str:
        c = command.strip().upper()
        if c == "*IDN?": return self.identify()
        if c == "RNGE?": return self.range
        if c == "VOLT?": return f"{self.voltage_V:.9g}"
        if c == "SOUT?": return "ON" if self.output_on else "OFF"
        if c == "ILOC?": return "CLOSED" if self.interlock_closed else "OPEN"
        if c == "OVLD?": return "0"
        if c in {"LEXE?", "LCME?"}: return "0"
        return ""


def normalize_range(value: str | int | float) -> str:
    key = str(value).strip().upper().replace(" ", "")
    # Important: numeric token mode uses 0/1/2 for RANGE1/RANGE10/RANGE100.
    # Human-friendly input also accepts 1/10/100 V.
    if key in ("0", "1V", "RANGE1", "+/-1", "±1"):
        return "RANGE1"
    if key in ("1", "10", "10V", "RANGE10", "+/-10", "±10"):
        return "RANGE10"
    if key in ("2", "100", "100V", "RANGE100", "+/-100", "±100"):
        return "RANGE100"
    # Bare numeric 1 is ambiguous: SRS token 1 means RANGE10, but user often means 1 V.
    # The CLI handles --range 1 as RANGE1 before calling this; keep this fallback safe.
    raise ValueError(f"Unsupported DC205 range {value!r}; use 1V/RANGE1, 10V/RANGE10, or 100V/RANGE100")


def normalize_user_range(value: str | int | float) -> str:
    """Interpret CLI/user values. Here bare 1 means 1 V, not SRS token 1."""
    key = str(value).strip().upper().replace(" ", "")
    if key in ("1", "1V", "RANGE1", "0"):
        return "RANGE1"
    if key in ("10", "10V", "RANGE10"):
        return "RANGE10"
    if key in ("100", "100V", "RANGE100", "2"):
        return "RANGE100"
    raise ValueError(f"Unsupported DC205 range {value!r}; use 1, 10, or 100")


def parse_range_response(text: str) -> str:
    t = text.strip().upper().replace(" ", "")
    # Prefer keyword tokens when TOKN ON is active. Some units/interfaces have
    # been observed to return the human-readable full-scale value (1/10/100)
    # rather than the manual's numeric token (0/1/2), so accept both forms.
    if t in ("RANGE1", "0", "1", "1V", "+/-1", "±1"):
        return "RANGE1"
    if t in ("RANGE10", "10", "10V", "+/-10", "±10"):
        return "RANGE10"
    if t in ("RANGE100", "2", "100", "100V", "+/-100", "±100"):
        return "RANGE100"
    raise ValueError(f"Could not parse DC205 range response {text!r}")


def parse_bool_token(text: str, true_words: set[str], false_words: set[str] | None = None) -> bool:
    t = text.strip().upper().replace(" ", "")
    if t in {"1", *true_words}:
        return True
    if false_words is None:
        false_words = set()
    if t in {"0", *false_words}:
        return False
    # Some responses include both keyword and numeric token, e.g. CLOSED 1.
    if any(w in t for w in true_words):
        return True
    if any(w in t for w in false_words):
        return False
    return False


def parse_float_response(text: str) -> float:
    m = re.search(r"[-+]?\d+(?:\.\d*)?(?:[Ee][-+]?\d+)?", text.strip())
    if not m:
        raise ValueError(f"Could not parse float from response {text!r}")
    return float(m.group(0))


class DC205(SerialAsciiDevice):
    def __init__(
        self,
        port: str,
        *,
        name: str = "SRS DC205",
        baudrate: int = 115200,
        timeout_s: float = 1.0,
        max_abs_voltage_V: float = 10.0,
        allow_100V_range: bool = False,
        simulate: bool = False,
    ) -> None:
        super().__init__(SerialSettings(port=port, baudrate=baudrate, timeout_s=timeout_s), simulate=simulate)
        self.name = name
        self.max_abs_voltage_V = float(max_abs_voltage_V)
        self.allow_100V_range = bool(allow_100V_range)

    def identify(self) -> str:
        if self.simulate:
            return "Stanford Research Systems,DC205,s/nSIM00000,ver1.00"
        return self.query("*IDN?")

    def raw(self, command: str) -> str:
        return self.query(command)

    def clear_status(self) -> None:
        self.write("*CLS")

    def set_termination_lf(self) -> None:
        self.write("TERM LF")

    def set_token_mode(self, on: bool = True) -> None:
        self.write(f"TOKN {'ON' if on else 'OFF'}")

    def reset_default(self) -> None:
        """Reset DC205 to default configuration using *RST.

        Per the DC205 manual, *RST sets output off, 0 V, RANGE1, and default
        configuration. It does not change the RS-232 baud rate.
        """
        if self.simulate:
            return
        self.write("*RST")
        import time
        # The DC205 can take a moment before it is ready to answer queries after
        # reset. Do not query errors immediately here, because a delayed reply can
        # leave one stale line in the serial buffer and shift the next status query.
        time.sleep(0.8)
        self.flush_input()

    def query_range(self) -> str:
        if self.simulate:
            return "RANGE1"
        return parse_range_response(self.query("RNGE?"))

    def set_range(self, range_name: str | int | float) -> None:
        rng = normalize_user_range(range_name)
        if rng == "RANGE100" and not self.allow_100V_range:
            raise RuntimeError("RANGE100 is disabled by software safety. Set allow_100V_range=true in config only when intended.")
        # RNGE cannot be set while output is enabled, so leave that responsibility explicit.
        self.write(f"RNGE {rng}")
        self._raise_on_errors()

    def query_voltage(self) -> float:
        """Return the programmed voltage setpoint, not an independent voltage measurement."""
        if self.simulate:
            return 0.0
        return parse_float_response(self.query("VOLT?"))

    def query_voltage_setpoint(self) -> float:
        return self.query_voltage()

    def set_voltage(self, voltage_V: float) -> None:
        voltage_V = float(voltage_V)
        if abs(voltage_V) > self.max_abs_voltage_V:
            raise ValueError(
                f"Requested {voltage_V:g} V exceeds software limit ±{self.max_abs_voltage_V:g} V. "
                "Edit config intentionally before raising this limit."
            )
        # Also check against current range if possible.
        rng = self.query_range()
        limit = RANGE_LIMITS.get(rng, self.max_abs_voltage_V)
        if abs(voltage_V) > limit:
            raise ValueError(f"Requested {voltage_V:g} V exceeds current DC205 {rng} limit ±{limit:g} V")
        self.write(f"VOLT {voltage_V:.9g}")
        self._raise_on_errors()

    def query_output(self) -> bool:
        if self.simulate:
            return False
        resp = self.query("SOUT?")
        return parse_bool_token(resp, {"ON"}, {"OFF"})

    def set_output(self, on: bool) -> None:
        if on:
            rng = self.query_range()
            if rng == "RANGE100" and not self.query_interlock():
                raise RuntimeError("Refusing SOUT ON in RANGE100 because interlock is open.")
        self.write(f"SOUT {'ON' if on else 'OFF'}")
        self._raise_on_errors()

    def query_interlock(self) -> bool:
        if self.simulate:
            return False
        resp = self.query("ILOC?")
        return parse_bool_token(resp, {"CLOSED"}, {"OPEN"})

    def query_overload(self) -> bool:
        if self.simulate:
            return False
        resp = self.query("OVLD?")
        return parse_bool_token(resp, {"OVLD"}, {"OKAY"})

    def read_errors(self) -> tuple[str, str]:
        lex = self.query("LEXE?").strip()
        lcm = self.query("LCME?").strip()
        return lex, lcm

    def _raise_on_errors(self) -> None:
        try:
            lex, lcm = self.read_errors()
        except Exception:
            return
        if lex not in ("0", "0;0", "") or lcm not in ("0", ""):
            raise RuntimeError(f"DC205 reported errors: LEXE={lex}, LCME={lcm}")

    def status(self) -> DC205Status:
        # Make response formatting explicit before reading a block of status.
        try:
            self.set_termination_lf()
            self.set_token_mode(True)
        except Exception:
            pass
        raw: dict[str, str] = {}
        idn = self.identify(); raw["*IDN?"] = idn
        rng_raw = self.query("RNGE?"); raw["RNGE?"] = rng_raw
        rng = parse_range_response(rng_raw)
        vresp = self.query("VOLT?"); raw["VOLT?"] = vresp
        sresp = self.query("SOUT?"); raw["SOUT?"] = sresp
        iresp = self.query("ILOC?"); raw["ILOC?"] = iresp
        oresp = self.query("OVLD?"); raw["OVLD?"] = oresp
        lex, lcm = self.read_errors(); raw["LEXE?"] = lex; raw["LCME?"] = lcm
        return DC205Status(
            idn=idn,
            range=rng,
            voltage_setpoint_V=parse_float_response(vresp),
            output_on=parse_bool_token(sresp, {"ON"}, {"OFF"}),
            interlock_closed=parse_bool_token(iresp, {"CLOSED"}, {"OPEN"}),
            overload=parse_bool_token(oresp, {"OVLD"}, {"OKAY"}),
            last_execution_error=lex,
            last_command_error=lcm,
            raw=raw,
        )
