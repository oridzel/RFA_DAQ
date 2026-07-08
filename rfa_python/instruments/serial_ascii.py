"""Small helpers for ASCII-over-serial lab instruments."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Optional

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover
    serial = None


@dataclass
class SerialSettings:
    port: str
    baudrate: int
    timeout_s: float = 1.0
    write_timeout_s: float = 1.0
    terminator: str = "\n"
    read_terminator: bytes = b"\n"


class SerialAsciiDevice:
    """Minimal line-oriented serial instrument base class.

    The class intentionally stays small. Instrument-specific safety checks and
    parsing live in the derived driver classes.
    """

    def __init__(self, settings: SerialSettings, *, simulate: bool = False) -> None:
        self.settings = settings
        self.simulate = simulate
        self._serial = None
        self.last_command: Optional[str] = None
        self.last_response: Optional[str] = None

    @property
    def is_open(self) -> bool:
        if self.simulate:
            return True
        return bool(self._serial and self._serial.is_open)

    def open(self) -> None:
        if self.simulate:
            return
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: pip install pyserial")
        if self._serial and self._serial.is_open:
            return
        self._serial = serial.Serial(
            self.settings.port,
            baudrate=self.settings.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.settings.timeout_s,
            write_timeout=self.settings.write_timeout_s,
            xonxoff=False,
        )
        time.sleep(0.05)
        self.flush()

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()

    def flush(self) -> None:
        if self.simulate:
            return
        if not self._serial:
            return
        try:
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except Exception:
            pass

    def flush_input(self) -> None:
        if self.simulate:
            return
        if not self._serial:
            return
        try:
            self._serial.reset_input_buffer()
        except Exception:
            pass

    def write(self, command: str) -> None:
        self.last_command = command
        if self.simulate:
            return
        self.open()
        assert self._serial is not None
        payload = (command + self.settings.terminator).encode("ascii")
        self._serial.write(payload)
        self._serial.flush()

    def read_line(self) -> str:
        if self.simulate:
            return ""
        self.open()
        assert self._serial is not None
        raw = self._serial.readline()
        text = raw.decode("ascii", errors="replace").strip("\r\n\x00 ")
        self.last_response = text
        return text

    def query(self, command: str, *, delay_s: float = 0.05, flush_input: bool = True) -> str:
        # Most of these instruments are strictly one-response-per-query, but during
        # early testing a delayed response from a previous command can otherwise shift
        # all subsequent query results by one command. Clearing only the input buffer
        # immediately before a query keeps the command/response stream synchronized.
        self.open()
        if flush_input:
            self.flush_input()
        self.write(command)
        if delay_s:
            time.sleep(delay_s)
        # Read the first non-empty line. Some devices/configurations can produce a
        # blank line around reset/termination changes.
        response = self.read_line()
        if response:
            return response
        end = time.time() + max(0.1, self.settings.timeout_s)
        while time.time() < end:
            response = self.read_line()
            if response:
                return response
        return response
