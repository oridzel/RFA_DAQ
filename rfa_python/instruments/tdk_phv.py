"""TDK-Lambda PHV high-voltage power supply digital-interface driver.

Conservative driver for RFA use.  It supports both individual PHV supplies and
RFA-style signed-voltage operation where the sign of a requested voltage is
converted into PHV magnitude plus optional polarity command.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
import re
import time
from typing import Any, Literal, Optional

from .serial_ascii import SerialAsciiDevice, SerialSettings

Polarity = Literal["positive", "negative"]
PolarityMode = Literal["reversible", "fixed_positive", "fixed_negative", "unknown"]


@dataclass
class PHVStatus:
    idn: Optional[str] = None
    output_on: Optional[bool] = None
    voltage_setpoint_V: Optional[float] = None
    current_setpoint_A: Optional[float] = None
    measured_voltage_V: Optional[float] = None
    measured_current_A: Optional[float] = None
    voltage_rating_V: Optional[float] = None
    current_rating_A: Optional[float] = None
    polarity_mode: Optional[str] = None
    requested_polarity: Optional[str] = None
    signed_voltage_setpoint_V: Optional[float] = None
    signed_measured_voltage_V: Optional[float] = None
    raw: dict[str, str] | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def parse_labeled_float(text: str) -> float:
    """Parse floats from PHV responses like M0:+5.00000E+3."""
    payload = text.split(":", 1)[1] if ":" in text else text
    matches = re.findall(r"[-+]?\d+(?:\.\d*)?(?:[Ee][-+]?\d+)?", payload)
    if not matches:
        raise ValueError(f"Could not parse float from PHV response {text!r}")
    return float(matches[-1])


class SimulatedPHV:
    def __init__(
        self,
        name: str = "TDK PHV",
        *,
        polarity_mode: PolarityMode = "reversible",
        m0_is_magnitude_only: bool = False,
        max_abs_voltage_V: float = 100.0,
        max_current_A: float = 1e-3,
        default_current_A: float = 1e-4,
        **_: Any,
    ) -> None:
        self.name = name
        self.voltage_setpoint_V = 0.0
        self.current_setpoint_A = float(default_current_A)
        self.output_on = False
        self.ramp_mode = 2
        self.ramp_rate_V_per_s = 25.0
        self.polarity_mode = polarity_mode
        self.m0_is_magnitude_only = bool(m0_is_magnitude_only)
        self.max_abs_voltage_V = float(max_abs_voltage_V)
        self.max_current_A = float(max_current_A)
        self.default_current_A = float(default_current_A)
        self._last_requested_polarity: Polarity | None = (
            "negative" if polarity_mode == "fixed_negative" else "positive"
        )
        self._last_signed_voltage_setpoint: float | None = 0.0

    def open(self) -> None: pass
    def close(self) -> None: pass
    def arm_hv_changes(self, token: str) -> None: pass
    def identify(self) -> str: return f"TDK-Lambda,PHV,SIM-{self.name}"
    def raw_query(self, command: str) -> str: return f"SIM:{command}"

    def _polarity_from_signed_voltage(self, voltage_V: float) -> Polarity:
        return "negative" if float(voltage_V) < 0 else "positive"

    def _validate_polarity_allowed(self, polarity: Polarity) -> None:
        if self.polarity_mode == "fixed_positive" and polarity != "positive":
            raise ValueError(f"{self.name} is configured as fixed-positive; negative voltage is not allowed.")
        if self.polarity_mode == "fixed_negative" and polarity != "negative":
            raise ValueError(f"{self.name} is configured as fixed-negative; positive voltage is not allowed.")

    def set_output_polarity(self, polarity: Polarity) -> None:
        self._validate_polarity_allowed(polarity)
        self._last_requested_polarity = polarity

    def output(self, on: bool) -> None:
        self.output_on = bool(on)

    def query_output(self) -> bool:
        return self.output_on

    def set_voltage(self, voltage_V: float) -> None:
        voltage_V = abs(float(voltage_V))
        if voltage_V > self.max_abs_voltage_V:
            raise ValueError(f"Requested {voltage_V:g} V exceeds software limit ±{self.max_abs_voltage_V:g} V")
        self.voltage_setpoint_V = voltage_V

    def set_signed_voltage(self, voltage_V: float) -> None:
        voltage_V = float(voltage_V)
        if abs(voltage_V) > 0:
            self.set_output_polarity(self._polarity_from_signed_voltage(voltage_V))
        self.set_voltage(abs(voltage_V))
        self._last_signed_voltage_setpoint = voltage_V

    def set_current(self, current_A: float) -> None:
        current_A = float(current_A)
        if current_A < 0 or current_A > self.max_current_A:
            raise ValueError(f"Requested current {current_A:g} A exceeds software limit {self.max_current_A:g} A")
        self.current_setpoint_A = current_A

    def query_voltage_setpoint(self) -> float:
        return self.voltage_setpoint_V

    def query_current_setpoint(self) -> float:
        return self.current_setpoint_A

    def _sign_for_current_polarity(self) -> float:
        polarity = self._last_requested_polarity
        if polarity is None:
            polarity = "negative" if self.polarity_mode == "fixed_negative" else "positive"
        return -1.0 if polarity == "negative" else 1.0

    def measure_voltage(self) -> float:
        if not self.output_on:
            return 0.0
        if self.m0_is_magnitude_only:
            return abs(self.voltage_setpoint_V)
        return self._sign_for_current_polarity() * abs(self.voltage_setpoint_V)

    def measure_signed_voltage(self) -> float:
        actual = self.measure_voltage()
        if self.m0_is_magnitude_only:
            return self._sign_for_current_polarity() * abs(actual)
        return actual

    def measure_current(self) -> float:
        return 0.0

    def query_voltage_rating(self) -> float:
        return 5000.0

    def query_current_rating(self) -> float:
        return 0.025

    def set_ramp_mode(self, mode: int = 2) -> None:
        if mode not in (0, 1, 2, 4):
            raise ValueError("PHV voltage ramp mode must be 0, 1, 2, or 4")
        self.ramp_mode = int(mode)

    def query_voltage_ramp_mode(self) -> int:
        return self.ramp_mode

    def set_voltage_ramp_rate(self, rate_V_per_s: float) -> None:
        if rate_V_per_s <= 0:
            raise ValueError("Ramp rate must be positive")
        self.ramp_rate_V_per_s = float(rate_V_per_s)

    def query_voltage_ramp_rate(self) -> float:
        return self.ramp_rate_V_per_s

    def query_voltage_ramp_status(self) -> int:
        return 0

    def query_instantaneous_ramp_voltage(self) -> float:
        return self.measure_signed_voltage()

    def apply_signed_voltage(
        self,
        voltage_V: float,
        current_limit_A: float | None = None,
        *,
        output_on: bool = True,
        ramp_rate_V_per_s: float | None = None,
        ramp_mode: int | None = None,
        settle_s: float = 0.0,
    ) -> None:
        current_A = self.default_current_A if current_limit_A is None else float(current_limit_A)
        self.set_current(current_A)
        if ramp_mode is not None:
            self.set_ramp_mode(ramp_mode)
        if ramp_rate_V_per_s is not None:
            self.set_voltage_ramp_rate(ramp_rate_V_per_s)
        self.set_signed_voltage(voltage_V)
        self.output(output_on)
        if settle_s:
            time.sleep(settle_s)

    def wait_for_signed_voltage(
        self,
        target_V: float,
        *,
        tolerance_V: float = 0.2,
        timeout_s: float = 10.0,
        poll_s: float = 0.2,
    ) -> float:
        actual = self.measure_signed_voltage()
        if abs(actual - target_V) <= tolerance_V:
            return actual
        # Simulation jumps immediately to the target.
        self.set_signed_voltage(target_V)
        self.output(True)
        return self.measure_signed_voltage()

    def set_signed_voltage_and_wait(
        self,
        voltage_V: float,
        current_limit_A: float | None = None,
        *,
        ramp_rate_V_per_s: float | None = None,
        ramp_mode: int | None = None,
        tolerance_V: float = 0.2,
        timeout_s: float = 10.0,
        poll_s: float = 0.2,
        output_off_after_zero: bool = False,
    ) -> float:
        self.apply_signed_voltage(
            voltage_V,
            current_limit_A=current_limit_A,
            output_on=True,
            ramp_rate_V_per_s=ramp_rate_V_per_s,
            ramp_mode=ramp_mode,
            settle_s=0.0,
        )
        actual = self.wait_for_signed_voltage(voltage_V, tolerance_V=tolerance_V, timeout_s=timeout_s, poll_s=poll_s)
        if output_off_after_zero and abs(voltage_V) <= tolerance_V:
            self.output(False)
        return actual

    def zero_and_wait(
        self,
        *,
        current_limit_A: float | None = None,
        tolerance_V: float = 0.2,
        timeout_s: float = 10.0,
        poll_s: float = 0.2,
        output_off_after_zero: bool = False,
    ) -> float:
        return self.set_signed_voltage_and_wait(
            0.0,
            current_limit_A=current_limit_A,
            tolerance_V=tolerance_V,
            timeout_s=timeout_s,
            poll_s=poll_s,
            output_off_after_zero=output_off_after_zero,
        )

    def zero_and_output_off(self) -> None:
        self.set_voltage(0.0)
        self.output(False)

    def status(self) -> PHVStatus:
        sign = self._sign_for_current_polarity()
        measured = self.measure_voltage()
        signed_measured = self.measure_signed_voltage()
        return PHVStatus(
            idn=self.identify(),
            output_on=self.output_on,
            voltage_setpoint_V=self.voltage_setpoint_V,
            current_setpoint_A=self.current_setpoint_A,
            measured_voltage_V=measured,
            measured_current_A=self.measure_current(),
            voltage_rating_V=self.query_voltage_rating(),
            current_rating_A=self.query_current_rating(),
            polarity_mode=self.polarity_mode,
            requested_polarity=self._last_requested_polarity,
            signed_voltage_setpoint_V=sign * self.voltage_setpoint_V,
            signed_measured_voltage_V=signed_measured,
            raw={"SIMULATED": "true"},
        )


class PHV(SerialAsciiDevice):
    def __init__(
        self,
        port: str,
        *,
        name: str = "TDK PHV",
        baudrate: int = 9600,
        timeout_s: float = 1.0,
        max_abs_voltage_V: float = 100.0,
        max_current_A: float = 1e-3,
        default_current_A: float = 1e-4,
        polarity_mode: PolarityMode = "unknown",
        require_enable_token: bool = True,
        simulate: bool = False,
        m0_is_magnitude_only: bool = False,
    ) -> None:
        super().__init__(SerialSettings(port=port, baudrate=baudrate, timeout_s=timeout_s), simulate=simulate)
        self.name = name
        self.max_abs_voltage_V = float(max_abs_voltage_V)
        self.max_current_A = float(max_current_A)
        self.default_current_A = float(default_current_A)
        self.polarity_mode: PolarityMode = polarity_mode
        self.require_enable_token = bool(require_enable_token)
        self.m0_is_magnitude_only = bool(m0_is_magnitude_only)
        self._armed_for_hv = False
        self._last_requested_polarity: Optional[Polarity] = None
        self._last_signed_voltage_setpoint: Optional[float] = None

    def arm_hv_changes(self, token: str) -> None:
        if token != "I_UNDERSTAND_THIS_CONTROLS_HIGH_VOLTAGE":
            raise RuntimeError("Wrong HV enable token.")
        self._armed_for_hv = True

    def _check_armed(self) -> None:
        if self.require_enable_token and not self._armed_for_hv:
            raise RuntimeError(
                "HV-changing command blocked. Call arm_hv_changes('I_UNDERSTAND_THIS_CONTROLS_HIGH_VOLTAGE') "
                "or pass --i-understand-hv in the CLI."
            )

    def _cmd(self, command: str) -> str:
        resp = self.query(command)
        if resp.upper().startswith("E") and not resp.upper().startswith("E0"):
            raise RuntimeError(f"PHV command {command!r} returned error {resp!r}")
        return resp

    def raw_query(self, command: str) -> str:
        return self._cmd(command)

    def identify(self) -> str:
        if self.simulate:
            return "TDK-Lambda,PHV,SIM00000"
        return self._cmd("*IDN?")

    def clear(self) -> str:
        self._check_armed()
        return self._cmd("=")

    def output(self, on: bool) -> None:
        self._check_armed()
        resp = self._cmd(f">BON {1 if on else 0}")
        if not resp.upper().startswith("E0"):
            raise RuntimeError(f"PHV output command failed: {resp!r}")

    def query_output(self) -> bool:
        if self.simulate:
            return False
        resp = self._cmd(">DON?").strip().upper()
        return resp.endswith("1")

    def _polarity_from_signed_voltage(self, voltage_V: float) -> Polarity:
        return "negative" if float(voltage_V) < 0 else "positive"

    def _validate_polarity_allowed(self, polarity: Polarity) -> None:
        if self.polarity_mode == "fixed_positive" and polarity != "positive":
            raise ValueError(f"{self.name} is configured as fixed-positive; negative voltage is not allowed.")
        if self.polarity_mode == "fixed_negative" and polarity != "negative":
            raise ValueError(f"{self.name} is configured as fixed-negative; positive voltage is not allowed.")

    def set_output_polarity(self, polarity: Polarity) -> None:
        self._check_armed()
        if polarity not in ("positive", "negative"):
            raise ValueError("polarity must be 'positive' or 'negative'")
        self._validate_polarity_allowed(polarity)
        if self.polarity_mode == "reversible":
            resp = self._cmd(">BX 0" if polarity == "positive" else ">BX 1")
            ok = resp.strip().upper() in {"0", "1", "E0"}
            if not ok:
                raise RuntimeError(f"PHV polarity command failed: {resp!r}")
        self._last_requested_polarity = polarity

    def set_voltage(self, voltage_V: float) -> None:
        self._check_armed()
        voltage_V = abs(float(voltage_V))
        if voltage_V > self.max_abs_voltage_V:
            raise ValueError(
                f"Requested {voltage_V:g} V exceeds software limit ±{self.max_abs_voltage_V:g} V. "
                "Edit config intentionally before raising this limit."
            )
        resp = self._cmd(f">S0 {voltage_V:.9g}")
        if not resp.upper().startswith("E0"):
            raise RuntimeError(f"PHV set voltage failed: {resp!r}")

    def query_voltage_setpoint(self) -> Optional[float]:
        if self.simulate:
            return 0.0
        for cmd in (">S0?", ">S0 ?"):
            try:
                return parse_labeled_float(self._cmd(cmd))
            except Exception:
                continue
        return None

    def set_current(self, current_A: float) -> None:
        self._check_armed()
        current_A = float(current_A)
        if current_A < 0 or current_A > self.max_current_A:
            raise ValueError(f"Requested current {current_A:g} A exceeds software limit {self.max_current_A:g} A")
        resp = self._cmd(f">S1 {current_A:.9g}")
        if not resp.upper().startswith("E0"):
            raise RuntimeError(f"PHV set current failed: {resp!r}")

    def query_current_setpoint(self) -> Optional[float]:
        if self.simulate:
            return 0.0
        for cmd in (">S1?", ">S1 ?"):
            try:
                return parse_labeled_float(self._cmd(cmd))
            except Exception:
                continue
        return None

    def measure_voltage(self) -> float:
        if self.simulate:
            return 0.0
        return parse_labeled_float(self._cmd(">M0?"))

    def measure_current(self) -> float:
        if self.simulate:
            return 0.0
        return parse_labeled_float(self._cmd(">M1?"))

    def query_voltage_rating(self) -> float:
        if self.simulate:
            return 5000.0
        return parse_labeled_float(self._cmd(">CS0T?"))

    def query_current_rating(self) -> float:
        if self.simulate:
            return 0.025
        return parse_labeled_float(self._cmd(">CS1T?"))

    def set_ramp_mode(self, mode: int = 2) -> None:
        self._check_armed()
        if mode not in (0, 1, 2, 4):
            raise ValueError("PHV voltage ramp mode must be 0, 1, 2, or 4")
        resp = self._cmd(f">S0B {mode}")
        if not resp.upper().startswith("E0"):
            raise RuntimeError(f"PHV ramp mode failed: {resp!r}")

    def query_voltage_ramp_mode(self) -> int | None:
        if self.simulate:
            return None
        return int(round(parse_labeled_float(self._cmd(">S0B?"))))

    def set_voltage_ramp_rate(self, rate_V_per_s: float) -> None:
        self._check_armed()
        if rate_V_per_s <= 0:
            raise ValueError("Ramp rate must be positive")
        resp = self._cmd(f">S0R {float(rate_V_per_s):.9g}")
        if not resp.upper().startswith("E0"):
            raise RuntimeError(f"PHV ramp rate failed: {resp!r}")

    def query_voltage_ramp_rate(self) -> float | None:
        if self.simulate:
            return None
        return parse_labeled_float(self._cmd(">S0R?"))

    def query_voltage_ramp_status(self) -> int | None:
        if self.simulate:
            return None
        return int(round(parse_labeled_float(self._cmd(">S0S?"))))

    def query_instantaneous_ramp_voltage(self) -> float | None:
        if self.simulate:
            return None
        return parse_labeled_float(self._cmd(">S0A?"))

    def _sign_for_current_polarity(self) -> float:
        polarity = self._last_requested_polarity
        if polarity is None:
            if self.polarity_mode == "fixed_negative":
                polarity = "negative"
            elif self.polarity_mode == "fixed_positive":
                polarity = "positive"
            else:
                polarity = "positive"
        return -1.0 if polarity == "negative" else 1.0

    def measure_signed_voltage(self) -> float:
        actual = float(self.measure_voltage())
        if self.m0_is_magnitude_only:
            return self._sign_for_current_polarity() * abs(actual)
        return actual

    def set_signed_voltage(self, voltage_V: float) -> None:
        self._check_armed()
        voltage_V = float(voltage_V)
        if abs(voltage_V) > 0:
            polarity = self._polarity_from_signed_voltage(voltage_V)
            self.set_output_polarity(polarity)
        self.set_voltage(abs(voltage_V))
        self._last_signed_voltage_setpoint = voltage_V

    def wait_for_signed_voltage(
        self,
        target_V: float,
        *,
        tolerance_V: float = 0.2,
        timeout_s: float = 10.0,
        poll_s: float = 0.2,
    ) -> float:
        target_V = float(target_V)
        tolerance_V = abs(float(tolerance_V))
        deadline = time.monotonic() + float(timeout_s)
        last_actual = None
        while True:
            last_actual = self.measure_signed_voltage()
            if abs(last_actual - target_V) <= tolerance_V:
                return last_actual
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"{self.name}: timed out waiting for {target_V:g} V "
                    f"within ±{tolerance_V:g} V. Last actual voltage was {last_actual:g} V."
                )
            time.sleep(poll_s)

    def set_signed_voltage_and_wait(
        self,
        voltage_V: float,
        current_limit_A: Optional[float] = None,
        *,
        ramp_rate_V_per_s: Optional[float] = None,
        ramp_mode: Optional[int] = None,
        tolerance_V: float = 0.2,
        timeout_s: float = 10.0,
        poll_s: float = 0.2,
        output_off_after_zero: bool = False,
    ) -> float:
        self._check_armed()
        target_V = float(voltage_V)
        current_A = self.default_current_A if current_limit_A is None else float(current_limit_A)
        if current_A <= 0:
            raise ValueError("Current limit must be > 0 before enabling PHV output.")
        if ramp_mode is not None:
            self.set_ramp_mode(ramp_mode)
        if ramp_rate_V_per_s is not None:
            self.set_voltage_ramp_rate(ramp_rate_V_per_s)
        self.set_current(current_A)
        if abs(target_V) > 0:
            target_polarity = self._polarity_from_signed_voltage(target_V)
            if (self._last_requested_polarity is not None and target_polarity != self._last_requested_polarity):
                self.set_voltage(0.0)
                self.output(True)
                self.wait_for_signed_voltage(0.0, tolerance_V=tolerance_V, timeout_s=timeout_s, poll_s=poll_s)
            self.set_output_polarity(target_polarity)
            self.output(True)
            self.set_voltage(abs(target_V))
        else:
            # Do not change polarity when going to zero. Actively drive magnitude to zero.
            self.output(True)
            self.set_voltage(0.0)
        self._last_signed_voltage_setpoint = target_V
        actual = self.wait_for_signed_voltage(target_V, tolerance_V=tolerance_V, timeout_s=timeout_s, poll_s=poll_s)
        if output_off_after_zero:
            if abs(target_V) > tolerance_V:
                raise ValueError("output_off_after_zero=True is only allowed for target 0 V.")
            self.output(False)
        return actual

    def zero_and_wait(
        self,
        *,
        current_limit_A: Optional[float] = None,
        tolerance_V: float = 0.2,
        timeout_s: float = 10.0,
        poll_s: float = 0.2,
        output_off_after_zero: bool = False,
    ) -> float:
        return self.set_signed_voltage_and_wait(
            0.0,
            current_limit_A=current_limit_A,
            tolerance_V=tolerance_V,
            timeout_s=timeout_s,
            poll_s=poll_s,
            output_off_after_zero=output_off_after_zero,
        )

    def apply_signed_voltage(
        self,
        voltage_V: float,
        current_limit_A: Optional[float] = None,
        *,
        output_on: bool = True,
        ramp_rate_V_per_s: Optional[float] = None,
        ramp_mode: Optional[int] = None,
        settle_s: float = 2.0,
    ) -> None:
        self._check_armed()
        current_A = self.default_current_A if current_limit_A is None else float(current_limit_A)
        if current_A <= 0:
            raise ValueError("Current limit must be > 0 before enabling PHV output.")
        if ramp_mode is not None:
            self.set_ramp_mode(ramp_mode)
        if ramp_rate_V_per_s is not None:
            self.set_voltage_ramp_rate(ramp_rate_V_per_s)
        polarity = self._polarity_from_signed_voltage(voltage_V)
        self.set_output_polarity(polarity)
        self.set_current(current_A)
        if output_on:
            self.output(True)
        self.set_voltage(abs(float(voltage_V)))
        self._last_signed_voltage_setpoint = float(voltage_V)
        if settle_s > 0:
            time.sleep(settle_s)

    def zero_and_output_off(self) -> None:
        self._check_armed()
        try:
            self.set_voltage(0.0)
            time.sleep(0.1)
        finally:
            self.output(False)

    def status(self) -> PHVStatus:
        raw: dict[str, str] = {}
        idn = self.identify(); raw["*IDN?"] = idn
        don = self._cmd(">DON?"); raw[">DON?"] = don
        s0 = self._cmd(">S0?"); raw[">S0?"] = s0
        s1 = self._cmd(">S1?"); raw[">S1?"] = s1
        m0 = self._cmd(">M0?"); raw[">M0?"] = m0
        m1 = self._cmd(">M1?"); raw[">M1?"] = m1
        cs0 = self._cmd(">CS0T?"); raw[">CS0T?"] = cs0
        cs1 = self._cmd(">CS1T?"); raw[">CS1T?"] = cs1

        magnitude_set = parse_labeled_float(s0)
        measured_raw = parse_labeled_float(m0)
        polarity = self._last_requested_polarity
        if polarity is None:
            if self.polarity_mode == "fixed_negative":
                polarity = "negative"
            elif self.polarity_mode == "fixed_positive":
                polarity = "positive"
        sign = -1.0 if polarity == "negative" else 1.0
        measured_signed = (sign * abs(measured_raw)) if self.m0_is_magnitude_only and polarity else measured_raw
        return PHVStatus(
            idn=idn,
            output_on=don.strip().upper().endswith("1"),
            voltage_setpoint_V=magnitude_set,
            current_setpoint_A=parse_labeled_float(s1),
            measured_voltage_V=measured_raw,
            measured_current_A=parse_labeled_float(m1),
            voltage_rating_V=parse_labeled_float(cs0),
            current_rating_A=parse_labeled_float(cs1),
            polarity_mode=self.polarity_mode,
            requested_polarity=polarity,
            signed_voltage_setpoint_V=sign * magnitude_set if polarity else None,
            signed_measured_voltage_V=measured_signed,
            raw=raw,
        )
