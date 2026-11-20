# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

PlatformIO firmware for a MATRIX Mini R4 competition robot. The board is an
Arduino UNO R4 WiFi (Renesas RA4M1, Cortex-M4 at 48 MHz) on a MATRIX Mini R4
carrier, using the `matrix-robotics/MatrixMiniR4@^1.2.6` library.

- Differential drive: M1 = left, M2 = right.
- Servos RC1..RC4 (RC1 gripper, RC2 lift arm, RC3/RC4 front arms).
- PS2/MJ2 wireless controller via bit-bang SPI on D2/D3.
- `README.md` is the full user and engineering manual, in Thai (~1200 lines).
  Do not load it whole; search it by section instead.
- `MATRIX_MARC_READINESS.md` is the competition readiness report.

## Commands

`pio` may not be on PATH. Use the full path on Windows:

```powershell
$pio = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"
& $pio test -e native                          # Unity host tests, expect 44/44 pass
& $pio test -e native -f test_robot_logic      # the only test suite
& $pio run -e uno_r4_wifi                      # firmware build, -Werror on src/
& $pio run -e uno_r4_wifi --target upload      # flash board (competition build, gyro features off)
& $pio run -e uno_r4_wifi_field --target upload  # bench/field-test build: gyro Auto + stall cut + heading hold ON
& $pio device monitor -b 115200                # serial CLI and telemetry
& $pio check -e uno_r4_wifi --skip-packages    # cppcheck; target 0 HIGH, 0 MEDIUM in src/
& $pio check -e native --skip-packages
```

Bootloader entry if upload cannot find the port: hold BTN_UP + BTN_DOWN for
2.5 s, or send `u` over serial. Fallback: double-tap the board RESET button.

## Architecture

- **Host-testable pure C++11**: `src/serial_control.cpp` and the six `RobotLogic` files
  (`robot_logic`, `persistent_config`, `auto_heading`, `servo_logic`, `link_failsafe`,
  `power_stability`), all declared in `include/robot_logic.h`. These are the only sources in
  the `native` `build_src_filter` (add new pure files there). Keep them free of Arduino
  and hardware headers.
- **Hardware layer**:
  - `minir4_io`: STM32 UART link, motors, servos, smart hold-brake, EEPROM.
  - `ps2_driver`: register-level bit-bang SPI (POSR/PORR/PIDR).
  - `oled_ui`: SSD1306 OLED, RGB LEDs, buzzer, I2C bus recovery.
  - `auto_mode`: autonomous run engine, X/serial abort, wheel-stall detection.
  - `main.cpp`: `setup()`/`loop()`, scheduler, bootloader shortcut.
- **Where to edit what**:
  - `include/user_settings.h`: values a driver tunes (speeds, expo, slew, servo angles and
    inversion, motor reverse, battery policy, Auto timing, gyro/stall/heading-hold flags).
  - `include/controls.h`: PS2 button -> function map (`Controls::kXxx = Btn::...`). Logic
    reads buttons only through `Controls::held()`; never hard-code `frame.l1` etc.
  - `include/config.h`: pins, loop/comm timings, hardware limits (rarely edited).
  - `src/auto_block_code.cpp`: pasted MATRIXblock mission. Serial keys: `src/serial_control.cpp`.
  - `include/types.h`: shared enums and structs. All tunables are `Config::k...`.
- **Control loop**: 40 Hz (25 ms) cooperative, non-blocking. Hardware WDT is
  3000 ms. Drive arbitration priority:
  `E-stop > PS2 Failsafe > PS2 Cross Brake > Serial > PS2 > Coast`.
- **Drive pipeline** `RobotLogic::computeKinodynamicDrive()`:
  cubic expo, then CoM scale from RC2 angle, then centripetal limit, then
  inverse kinematics + sag compensation + motor trim + stiction, then 2-DOF
  slew with power envelope cap.
- **EEPROM**: `PersistentConfig` is 8 bytes with CRC-16-CCITT at address 128.
- **Opt-in gyro features** (off in `env:uno_r4_wifi`; `env:uno_r4_wifi_field` turns them on via
  `-DMATRIX_AUTO_GYRO_MODE=2 -DMATRIX_MANUAL_STALL_CUT=1 -DMATRIX_TELEOP_HEADING_HOLD=1`; the
  robot has only the STM32 onboard IMU, no external sensors):
  - `kAutoGyroMode` 1/2: `AutoDriveDCProxy::MoveDegs` routes straight moves to
    `MoveGyroDegs` and tracks a virtual heading (`AutoMode::headingTargetDeg()`); mode 2
    also turns pivots into `TurnGyro` to a snapped heading. IMU is zeroed on Auto entry;
    if that fails Auto silently stays open-loop. `[AUTO] #n ...` telemetry shows yaw error.
  - `kEnableManualStallCut`: `ManualStallDetector` cuts both wheels after
    `kManualStallTimeoutMs` of no encoder progress (fails open on read errors).
  - `kEnableTeleopHeadingHold`: `TeleopHeadingHold` + `applyTurnCorrection` add a P turn
    correction while throttle-only.
  - Not hardware-verified: the sign of yaw vs wheel direction (`kAutoPivotYawSign`), the
    `TurnGyro` mode/power sign, and `kAutoPivotWheelDegPerYawDegX100`. Bench-check with
    wheels lifted and read the `[AUTO]` lines before enabling in a match.

## Rules

- Put new pure logic in the matching `RobotLogic` file (or `serial_control`), and add a test in
  `test/test_robot_logic/test_main.cpp`. If you add a new pure source file,
  add it to the `native` `build_src_filter` in `platformio.ini`.
- Never block the loop. No long `delay()`. Keep the 25 ms cadence and the WDT
  refresh.
- Use 32-bit intermediates in motor and servo math. Tests guard against
  `int16_t` overflow.
- Do not weaken safety paths: ESTOP, `forceMotorsStop()`, PS2 failsafe state
  machine, auto stall and abort handling.
- `src/auto_block_code.cpp` is where users paste MATRIXblock R4 code. The code
  goes below the diagnostic pragmas, inside `namespace AutoBlockUser`. Proxy
  classes there wrap `MiniR4` and `Serial`. Do not remove the pragmas or proxies.
- `src/` must build warning-free (`-Wall -Wextra -Werror`).
- The README states the test count (38), the file count (28), and the
  `config.h` parameter table. Update the README when any of these change.
