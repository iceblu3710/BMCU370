# BMCU370

Latest **BMCU Xing-C modified version**  
(BMCU-C 370 Hall version V0.1-0020) source code.

Original project: https://github.com/Xing-C/BMCU370x  
This version includes some minor personal optimizations.

---

## Links

- English wiki: https://wiki.yuekai.fr/
- Chinese wiki: https://bmcu.wanzii.cn/

---

## Changelog

Copied from the group files.

### 2025-12-25 — Version 0020_A1

- Complete refactoring of the code.
- English code translations.
- Abstraction of the command routing logic.
- Klipper support.

---

### 2025-07-17 — Version 0020

- Fixed an error in the lighting logic that caused some states to have no indicator lights.
- Fixed channels coming online unexpectedly.
- Fixed the anti-disconnect logic, which previously was not actually effective.
- Rewrote the lighting system:
  - Fixed flickering issues.
  - Reduced the refresh rate.
- When a channel is in an error state, the red indicator is retried every 3 seconds to prevent channels inserted after the BMCU enters working state from failing to light up.

---

### 2025-07-06 — Version 0019 (Modified)

- Dual micro-switch Hall version is also supported.

#### Changes in original 0019 compared to original 0013

- Depending on the flashed firmware, P1X1 can now support 16 colors.
- Fixed an issue where filament information could not be saved after:
  - Updating the P1X1 printer firmware (current latest: `00.01.06.62`), or
  - Using the latest slicer software (current latest: `2.1.1.52`).
- Modified online logic checks to prevent incorrect channels from appearing online in certain states.
- Modified motor control logic to use different calls for high-voltage and low-voltage conditions.

#### Changes relative to the modified 0013 “overheat-prevention lighting” version

- Mainboard lighting behavior:
  - Red breathing effect when not connected to a printer.
  - White breathing effect during normal operation.
- Further reduced the brightness of buffer lighting and mainboard lighting.
- Removed control of the A1 printer during filament unloading.
