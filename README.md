# RFA Python DAQ Control Suite

Python control software for the RFA secondary-electron-yield measurement system.

This package provides a PySide6 GUI and command-line tools for controlling the electron gun, picoammeters, high-voltage supplies, motion stages, collector bias supply, and automated imaging/measurement procedures.

---

## 1. System Overview

The system controls the following hardware:

| Device | Purpose |
|---|---|
| Kimball electron gun / EGPS control interface | Beam energy, Source/ECC, grid/cutoff, focus, X/Y deflection, digital switches |
| RBD 9103 picoammeters | Current measurement from RFA electrodes |
| TDK/FUG/PHV supplies | Biases for RFA electrodes |
| SRS DC205 | Collector voltage source, typically +50 V for TEY/imaging |
| IMS MDrive motors | RFA lift and sample/holder rotation |
| NI DAQ | Analog/digital interface for Kimball electron gun control |
| PySide6 GUI | Integrated system control, monitoring, warm-up, and imaging |

The package is designed for cautious manual-supervised operation. It includes software limits, explicit real-hardware confirmation flags, and conservative ramping procedures for the electron gun Source supply.

---

## 2. Important Safety Notes

This software controls high voltages, electron-gun cathode heating, beam deflection, and motorized vacuum hardware.

Before running real hardware:

1. Confirm vacuum is safe for electron-gun operation.
2. Confirm correct grounding of chamber and unused feedthroughs.
3. Confirm the correct COM ports and NI DAQ channels.
4. Confirm the Source/ECC mode is correct.
5. Watch the Kimball front panel during Source ramping.
6. Keep hardware emergency stop and power controls accessible.
7. Do not leave automated warm-up or imaging unattended.

The Source supply has delayed response. After changing Source voltage, the actual Source voltage and Source current may continue to ramp for several seconds. The software therefore uses a default 10 s delay between Source steps.

---

## 3. Installation

From the package directory:

```bash
pip install -e .
