# RFA Python RBD / Kimball control package v0.19.19

Changes from v0.19.18:

## Live XY image pixel inspector

Added click-to-inspect support for the XY imaging live TEY image.

- Works while the scan is still running for pixels that have already been measured.
- Works after the scan is finished for all measured pixels.
- Click a pixel in the live image to show:
  - X deflection voltage
  - Y deflection voltage
  - TEY
  - yield numerator and denominator
  - raw physical-electrode mean currents when present in the row
- The selected pixel is marked on the image with a square marker.
- If you click a pixel that has not been measured yet, the panel reports that it is not measured yet and shows the nearest X/Y coordinates.

This is purely a GUI/data-display change. It does not change imaging hardware behavior, source warm-up, deflection-switch control, SRS settings, or RBD acquisition settings.

## Kept from v0.19.18

- SRS DC205 config:
  - port COM11
  - baudrate 115200
  - max_abs_voltage_V 100.0
  - allow_100V_range true
- RBD COM-port mapping:
  - Sample / Sneezy: COM10
  - Retarding Grid 1 / Sleepy: COM13
  - Collector / Dopey: COM9
  - Retarding Grid 2 / Happy: COM14
  - Space-charge Grid / Grumpy: COM16
  - Drift Tube / Bashful: COM17
  - Rod / Doc: COM7
- XY imaging turns Kimball deflection ON before X/Y writes.
- Source scaling remains verified:
  - AO1 = 2 * Source_display_V
- Source ramp delay remains 10 s by default.
