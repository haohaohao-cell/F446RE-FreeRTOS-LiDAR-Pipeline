# Host-side unit tests (Part 2 pure logic)

These tests exist because the firmware can only be *compiled* without hardware - it
cannot be run.  Everything here is deliberately free of HAL, DMA and RTOS
dependencies, so the same source file that ships in the firmware is compiled for the
PC and exercised against real cases.

```cmd
tools\host_tests\run_tests.cmd
```

Requires any host C compiler on `PATH` (MinGW-w64 `gcc` is used on the development
machine).  `arm-none-eabi-gcc` cannot run on the PC, so it is not used here.

## What is covered

| Test | Module under test | What it proves |
|---|---|---|
| `test_bt_protocol.c` | `Core/Src/bt_protocol.c` | The command grammar: `G/S/P/?`, case-insensitivity, repeated spaces, `\r\n`, missing parameters, glued tokens, trailing junk, `\n`-then-junk, +/-`NAV_MAX_MM` boundary, 21-digit input cannot overflow int32, NULL arguments, and that `*out` is untouched when parsing fails. |
| `test_path.c` | `Core/Src/path.c` | Waypoint generation (count, exact final point, `PATH_FLAG_FINAL`, monotonic progress, step size, capacity refusal) and the tracking law (equal wheel targets when straight, in-place turn drives the wheels in opposite directions and symmetrically, yaw clamp, no NaN on a zero-length bearing). |
| `test_pose.c` | `Core/Src/pose.c` | Sign conventions: forward = +y, reverse = -y, gyro heading offset of +90 deg, wheel-difference fallback direction, turning in place does not translate, heading stays wrapped, `pose_reset()`, and no floating-point explosion on `period_ms = 0`. |

## What is NOT covered (and cannot be, here)

* Every HAL call, DMA stream, NVIC priority and I2C timing.
* The SSD1306 itself, the MPU-6500, the encoders, the motors.
* RTOS scheduling, queue blocking, mutex behaviour.

Those are on the "TBD on hardware" list in `Docs/HANDOVER.md` section 7.1.  A green
run here means **the logic is self-consistent**, not that the car works.

## Stub headers

`hostinc/` shadows `Core/Inc/board_pins.h` (only the wheel diameter / track width /
counts-per-rev constants, with the same unverified values) plus no-op
`FreeRTOS.h` / `task.h`.  The `-I hostinc` comes first so the real headers are never
pulled in.  Nothing in this directory is compiled into the firmware.
