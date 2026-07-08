"""
RBD Instruments 9103 picoammeter driver.

Python replacement for the LabVIEW ENG-ActionEngine_RBD_multi.vi RBD layer.
It uses the serial protocol shown in the vendor Python samples.

Known vendor commands:
    &R0\n      autorange
    &F032\n    filter 32
    &G0\n      normal input
    &B0\n      bias off
    &I1000\n   start standard-speed sampling, one sample per message
    &I0000\n   stop standard-speed sampling
    &Q\n       query status -> "RBD Instruments: PicoAmmeter"

This driver preserves raw responses in every reading so parser issues can be
fixed quickly from saved CSV files.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from datetime import datetime, timezone
import math
import re
import time
from typing import Optional, Any

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover
    serial = None


UNIT_TO_AMP = {
    "A": 1.0,
    "mA": 1e-3,
    "uA": 1e-6,
    "µA": 1e-6,
    "nA": 1e-9,
    "pA": 1e-12,
    "fA": 1e-15,
}


@dataclass
class RBDReading:
    name: str
    timestamp_utc: str
    current_A: Optional[float]
    raw_response: str
    display_name: Optional[str] = None
    offset_A: Optional[float] = None
    corrected_current_A: Optional[float] = None
    stable: Optional[bool] = None
    unstable: Optional[bool] = None
    out_of_range: Optional[bool] = None
    status_code: Optional[str] = None
    range_code: Optional[str] = None
    unit: Optional[str] = None
    error: Optional[str] = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class RBD9103:
    """One RBD 9103 picoammeter connected over a serial COM port."""

    def __init__(
        self,
        name: str,
        port: str,
        baudrate: int = 57600,
        timeout_s: float = 1.0,
        write_delay_s: float = 0.05,
    ) -> None:
        self.name = name
        self.port_name = port
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self.write_delay_s = float(write_delay_s)
        self.port: Any = None
        self.last_raw: str = ""

    @property
    def is_open(self) -> bool:
        return bool(self.port and self.port.is_open)

    def open(self) -> None:
        if self.is_open:
            return
        if serial is None:
            raise ImportError("pyserial is required for hardware access. Install with: pip install pyserial")
        self.port = serial.Serial(
            self.port_name,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            timeout=self.timeout_s,
        )
        time.sleep(0.1)
        self.flush()

    def close(self) -> None:
        if self.port is not None:
            try:
                if self.port.is_open:
                    try:
                        self.stop_sampling()
                    except Exception:
                        pass
                    self.port.close()
            finally:
                self.port = None

    def flush(self) -> None:
        if not self.port:
            return
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()

    def flush_input(self) -> None:
        if self.port:
            self.port.reset_input_buffer()

    def _message(self, command: str) -> bytes:
        command = command.strip()
        if command.startswith("&"):
            command = command[1:]
        return f"&{command}\n".encode("utf-8")

    def write_command(self, command: str) -> None:
        self.open()
        assert self.port is not None
        self.port.write(self._message(command))
        self.port.flush()
        if self.write_delay_s:
            time.sleep(self.write_delay_s)

    def read_line(self) -> str:
        self.open()
        assert self.port is not None
        raw = self.port.readline()
        try:
            text = raw.decode("utf-8", errors="replace").strip()
        except Exception:
            text = repr(raw)
        self.last_raw = text
        return text

    def query_status(self, timeout_s: float = 2.0) -> list[str]:
        self.flush()
        self.write_command("Q")
        replies: list[str] = []
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            line = self.read_line()
            if line:
                replies.append(line)
                if "PicoAmmeter" in line:
                    break
        return replies

    def initialize(self, *, range_setting: str | int = "auto", filter_setting: str | int = "off") -> None:
        self.open()
        self.set_range(range_setting)
        self.set_filter(filter_setting)
        self.set_input_normal()
        self.set_bias_off()
        self.flush_input()

    def set_input_normal(self) -> None:
        self.write_command("G0")

    def set_bias_off(self) -> None:
        self.write_command("B0")

    def set_range(self, setting: str | int) -> None:
        if isinstance(setting, str):
            s = setting.strip().lower()
            if s in {"auto", "autorange", "auto range", "0"}:
                code = 0
            else:
                code = int(s)
        else:
            code = int(setting)
        if code < 0:
            raise ValueError("range code must be >= 0")
        self.write_command(f"R{code}")

    def set_filter(self, setting: str | int) -> None:
        if isinstance(setting, str):
            s = setting.strip().lower()
            if s in {"off", "none", "0"}:
                code = 0
            elif s in {"default", "32"}:
                code = 32
            else:
                code = int(s)
        else:
            code = int(setting)
        if not (0 <= code <= 999):
            raise ValueError("filter code must be between 0 and 999")
        self.write_command(f"F{code:03d}")

    def start_sampling(self, interval_ms: int = 1000) -> None:
        interval_ms = int(interval_ms)
        if not (1 <= interval_ms <= 9999):
            raise ValueError("sample interval must be 1..9999 ms")
        self.flush_input()
        self.write_command(f"I{interval_ms:04d}")
        # Remove the command acknowledgement so first read is normally a sample.
        self._discard_until_sample_or_timeout(timeout_s=0.25)

    def stop_sampling(self) -> None:
        self.write_command("I0000")

    def read_current(self) -> RBDReading:
        raw = self.read_line()
        return parse_standard_reading(self.name, raw)

    def read_next_sample(self, timeout_s: float = 3.0) -> RBDReading:
        """Read until a valid sample arrives, skipping command acknowledgements."""
        deadline = time.time() + timeout_s
        last = RBDReading(
            name=self.name,
            timestamp_utc=_now_iso(),
            current_A=None,
            raw_response="",
            error="timeout waiting for sample",
        )
        while time.time() < deadline:
            r = self.read_current()
            if r.raw_response:
                last = r
            if r.current_A is not None:
                return r
        if last.current_A is None and not last.error:
            last.error = "timeout waiting for sample"
        return last

    def read_latest_sample(self, timeout_s: float = 3.0) -> RBDReading:
        """Flush queued/stale serial samples and return the next fresh sample.

        This is important when the instrument samples faster than the GUI/CLI
        display period. Without flushing, a 1 Hz monitor can slowly walk through
        old samples that were queued at 20 ms intervals.
        """
        self.flush_input()
        return self.read_next_sample(timeout_s=timeout_s)

    def read_one_sample(self, interval_ms: int = 20, timeout_s: float = 3.0) -> RBDReading:
        """Start sampling, wait for one valid sample, then stop."""
        old_timeout = self.timeout_s
        if self.port is not None:
            self.port.timeout = min(old_timeout, 0.5)
        self.start_sampling(interval_ms)
        try:
            return self.read_next_sample(timeout_s=timeout_s)
        finally:
            try:
                self.stop_sampling()
            finally:
                if self.port is not None:
                    self.port.timeout = old_timeout

    def _discard_until_sample_or_timeout(self, timeout_s: float = 0.25) -> None:
        """Consume command ACK lines and at most one early sample."""
        deadline = time.time() + timeout_s
        old_timeout = None
        if self.port is not None:
            old_timeout = self.port.timeout
            self.port.timeout = 0.02
        try:
            while time.time() < deadline:
                line = self.read_line()
                if not line:
                    continue
                r = parse_standard_reading(self.name, line)
                if r.current_A is not None:
                    # We intentionally discard this early sample.
                    return
        finally:
            if self.port is not None and old_timeout is not None:
                self.port.timeout = old_timeout


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def parse_current_value(text: str, *, allow_unitless: bool = False) -> tuple[Optional[float], Optional[str]]:
    """Parse measured current from RBD standard-speed lines.

    Examples:
        &S*,Range=002mA,+0.0060,mA
        &S*,Range=200uA,+000.09,uA
        &S*,Range=020nA,+00.078,nA

    The range field is not the measured current. The measured value is the
    numeric field after Range=..., with the following unit field.
    """
    cleaned = text.strip().strip("\x00")
    if cleaned.startswith("&"):
        cleaned = cleaned[1:]
    tokens = [t.strip() for t in cleaned.split(",") if t.strip()]

    for i, tok in enumerate(tokens):
        if tok.lower().startswith("range=") and i + 2 < len(tokens):
            value_token = tokens[i + 1].strip()
            unit_token = tokens[i + 2].strip()
            if unit_token in UNIT_TO_AMP:
                try:
                    return float(value_token) * UNIT_TO_AMP[unit_token], unit_token
                except ValueError:
                    pass

    for i in range(len(tokens) - 1):
        value_token = tokens[i].strip()
        unit_token = tokens[i + 1].strip()
        if unit_token in UNIT_TO_AMP:
            try:
                return float(value_token) * UNIT_TO_AMP[unit_token], unit_token
            except ValueError:
                pass

    unit_pattern = r"(?<!Range=)([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*(fA|pA|nA|uA|µA|mA|A)"
    matches = list(re.finditer(unit_pattern, cleaned))
    if matches:
        m = matches[-1]
        value = float(m.group(1))
        unit = m.group(2)
        return value * UNIT_TO_AMP[unit], unit

    if allow_unitless:
        nums = re.findall(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?", cleaned)
        if nums:
            try:
                return float(nums[-1]), "A?"
            except ValueError:
                pass
    return None, None


def _parse_status_and_range(tokens: list[str]) -> tuple[Optional[str], Optional[str]]:
    status_code: Optional[str] = None
    range_code: Optional[str] = None
    if tokens:
        first = tokens[0].strip()
        # S*, S=, S?, etc. Preserve the symbol after S, but don't guess meaning.
        if first.upper().startswith("S"):
            status_code = first[1:] or None
    for tok in tokens:
        if tok.lower().startswith("range="):
            range_code = tok.split("=", 1)[1].strip()
            break
    return status_code, range_code


def parse_standard_reading(name: str, raw: str) -> RBDReading:
    reading = RBDReading(name=name, timestamp_utc=_now_iso(), current_A=None, raw_response=raw)
    if not raw:
        reading.error = "empty response"
        return reading

    text = raw.strip().strip("\x00")
    if text.startswith("&"):
        text = text[1:]

    lower_text = text.lower()
    if "picoammeter" in lower_text:
        reading.error = "status response, not sample"
        return reading
    if (
        lower_text.startswith("i,")
        or "sample interval" in lower_text
        or lower_text.startswith("r,")
        or lower_text.startswith("f,")
        or lower_text.startswith("g,")
        or lower_text.startswith("b,")
    ):
        reading.error = "command acknowledgement, not sample"
        return reading

    upper = text.upper()
    is_sample = upper.startswith("S") or upper.startswith("PA")
    if not is_sample:
        reading.error = "unexpected sample format"
        return reading

    tokens = [t.strip() for t in text.split(",") if t.strip()]
    reading.status_code, reading.range_code = _parse_status_and_range(tokens)

    # The exact meaning of S* vs S= should be confirmed from the manual.
    # Preserve status_code but do not mark stable/unstable until known.
    reading.stable = None
    reading.unstable = None

    low = text.lower()
    if "out" in low and "range" in low:
        reading.out_of_range = True
    elif "over" in low or "oor" in low:
        reading.out_of_range = True
    else:
        reading.out_of_range = False

    value_A, unit = parse_current_value(text, allow_unitless=is_sample)
    reading.current_A = value_A
    reading.unit = unit
    if value_A is None and reading.error is None:
        reading.error = "could not parse current"
    return reading


class SimulatedRBD9103(RBD9103):
    """Hardware-free simulator for testing the manager/GUI at home."""

    def __init__(self, name: str, port: str = "SIM", **kwargs: Any) -> None:
        super().__init__(name=name, port=port, **kwargs)
        self._open = False
        self._t0 = time.time()
        self._sampling = False

    @property
    def is_open(self) -> bool:
        return self._open

    def open(self) -> None:
        self._open = True

    def close(self) -> None:
        self._open = False

    def flush(self) -> None:
        pass

    def flush_input(self) -> None:
        pass

    def write_command(self, command: str) -> None:
        self.open()
        if command.upper().startswith("I") and command != "I0000":
            self._sampling = True
        elif command.upper() == "I0000":
            self._sampling = False

    def read_line(self) -> str:
        t = time.time() - self._t0
        phase = (sum(ord(c) for c in self.name) % 20) / 20 * math.tau
        current_nA = 0.5 + 0.2 * math.sin(t + phase)
        return f"&S*,Range=020nA,+{current_nA:06.3f},nA"
