# F446RE Differential-Drive Navigation Car (Block 4.5 Part 2)

**STM32F446RE + FreeRTOS 10.3.1 (CMSIS-RTOS v2)**.
Send a target coordinate over Bluetooth; the car plans waypoints and closes the
loop on wheel encoders and a gyro until it gets there.

> **Headline**: Bluetooth command, 1.4 m straight run, **tape error <= 10 mm**,
> heading error 1.2 deg; eight threads running concurrently, 10 ms control loop
> with an 11 us worst-case execution time and a zero overrun count.

*Chinese version: [README.zh-CN.md](README.zh-CN.md).*

---

## What this is

Part 2 of Block 4.5. Part 1 was the LiDAR pipeline analysis; that pipeline is
still here and still running - it is threads 4, 6 and 7 of the eight, and it is
where the queue and semaphore evidence comes from.

| Marking role | Where | Evidence |
|---|---|---|
| 1 Complete application | 8 threads + Bluetooth + 3 queues | the 1 Hz status block (everything but OLED) |
| 2 Scheduling | `osDelayUntil` + runtime timing | period / jitter / WCET / overruns |
| 3 Message queues | `Q_Lidar` / `Q_Cmd` / `Q_Path` | peak depth + overflow counters |
| 4 Semaphore | TIM6 -> IMU task | measured 100 Hz |
| 5 Mutex | I2C1 + `osMutexPrioInherit` | 1.5 ms hold + the `i` inheritance demo |
| 6 Bluetooth navigation | BT -> path -> PID -> motors | three on-floor endpoint/heading errors |

---

## Architecture

```
     BT COM10 --"G 0 1400\n"--> UART5 --DMA1_S0--> bt_rx.c --sem--> BT RX task
                                                                       | Q_Cmd (4x8B, overwrite oldest)
                                                                       v
                                                                 PathGen task (100 mm steps, axis-aligned L)
                                                                       | Q_Path (16x8B, producer blocks)
                                                                       v
   TIM1/TIM8 hardware QEP --> Pose task ---------------------------> PID+Motor task (10 ms)
   MPU-6500 --I2C1 (mutex)--> imu.c gyro --> pose.c                      | one compute, one CCR write
                                                                       v
                                                           TIM4 / TIM2 H-bridges --> wheels

   RPLidar C1 --USART3+DMA (circular 2x1024 B)--> lidar.c --> Q_Lidar (8x12B, drop) --> LiDAR task
   BT TX / OLED tasks: 50 ms, emit the 1 Hz block, binary telemetry, paged display
```

**Eight threads** (larger number = higher priority):

| # | Thread | Prio | Stack | Period / trigger | Scheduling |
|---|---|---|---|---|---|
| 1 | `pidMotor` | 40 | 1024 | 10 ms | `osDelayUntil` |
| 2 | `imuAcq` | 37 | 1024 | 10 ms | counting semaphore (TIM6) |
| 3 | `pose` | 36 | 768 | 10 ms | `osDelayUntil` |
| 4 | `lidarProc` | 34 | 1536 | ~22 ms/block | `Q_Lidar` (100 ms guard) |
| 5 | `pathGen` | 30 | 1024 | command arrival | `Q_Cmd` |
| 6 | `btRx` | 28 | 768 | byte arrival | counting semaphore |
| 7 | `btTx` | 22 | 1024 | 50 ms | `osDelayUntil` |
| 8 | `oled` | 20 | 2048 | 50 ms | `osDelayUntil` |

> **The encoder is not a thread**: TIM1/TIM8 are hardware quadrature decoders, and
> the Pose task reads `CNT` once per 10 ms. **PID and motor are one task**: the
> setpoint becomes a local variable, so there is no queue and no latency between
> them. That is why the course's 10-row reference table is eight threads here.

---

## Measured (target hardware, 2026-11-17 / 11-18)

| Metric | Measured | Field |
|---|---|---|
| Motor task period / jitter / WCET | **10.00 ms / 42 us / 11 us** (budget 700 us) | `M: per/jit/wc` |
| Periodic-task overruns | **0** | `ov m0 p0` |
| IMU sample rate | **100 /s** | `IMU:` |
| Longest I2C hold | **1.5 ms** (target <3 ms) | `hold` |
| LiDAR pipeline | **24-25 blocks/s**, `Q_Lidar` peak 0/8 | `blk` / `q` |
| Heap | **19 816 / 32 768 B** | `H:` |
| Stack free (8 tasks) | 680 / 504 / 464 / 1280 / 768 / 576 / 636 / 1096 B | `STK:` |
| Failure counters | all zero (`ql qc qpt imu bt log iw`) | `CNT:` |
| **Navigation, 1.4 m straight** | firmware **12 mm / 0.9 %**; tape **<=10 mm**; heading +1.2 deg; 13.2 s | `nav_run` table |
| Navigation (-0.70, 0.70) | firmware 18 mm / 1.8 %; tape 52 mm (includes a 45 deg pivot) | same |
| Navigation (0.70, -0.70) | firmware 14 mm / 1.5 %; tape >30 mm (includes a 135 deg pivot) | same |
| Left/right symmetry | R/L **1.03 / 1.14** (was 1.80 / 4.74 before the fix) | `bench_p0 -Test symmetry` |
| Priority inheritance | holder priority **11 -> 13** (mutex) / 11 (semaphore) | `i` key |

Raw logs live in `tools/logs/` (`serial_*` 1 Hz lines, `bench_*` bench runs,
`nav_*` navigation + CSV). Full analysis:
[`Docs/Block4.5_Part2_Design_Report_CN.docx`](Docs/Block4.5_Part2_Design_Report_CN.docx).

---

## Hardware

| Item | Connection |
|---|---|
| MCU | NUCLEO-F446RE, 168 MHz, 128 kB SRAM |
| Right motor / encoder | **PB6 / PB7** (TIM4 CH1/CH2, polarity inverted, compensated by `RIGHT_INVERT`) / **PA8 / PA9** (TIM1 QEP) |
| Left motor / encoder | **PA0 / PA1** (TIM2 CH1/CH2) / **PC6 / PC7** (TIM8 QEP) |
| IMU | MPU-6500, I2C1 (PB8/PB9), 0x68 |
| Bluetooth | HC-04, UART5 (PC12/PD2), Windows SPP = **COM10** |
| Debug console | USART2 (PA2/PA3) -> ST-Link VCP **COM8** @115 200 |
| LiDAR | RPLidar C1, USART3 @460 800 + DMA1_Stream1 (PC10/PC11) |
| OLED | SSD1306 on I2C1 (0x3C) - **driver written, panel never fitted** (`oled=-3`) |

[`Core/Inc/board_pins.h`](Core/Inc/board_pins.h) is the single source of truth for
pins, timers and NVIC priorities.

> **Do not swap the right motor's two power leads**: `BOARD_MOTOR_RIGHT_INVERT=1`
> already compensates for the wiring, and swapping would invert it twice.

---

## Quick start

### 1. Build

`make` is not on `PATH`; add CubeIDE's make and arm-none-eabi first:

```powershell
$env:PATH = "D:\CubeIDE\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.100.202601091506\tools\bin;D:\CubeIDE\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin;" + $env:PATH
```

```cmd
make -C Debug -j8 all
```

Must be **0 warnings / 0 errors**, baseline **text = 90 244 B**.

### 2. Flash and watch

```cmd
tools\flash_and_monitor.cmd              :: probe -> flash -> verify -> 15 s capture
tools\flash_and_monitor.cmd -Probe       :: probe only (no probe vs no power)
tools\serial_console.cmd                 :: live console (s/g/f/b/l/r/x/c/e/i/h)
```

### 3. Drive it

```cmd
:: tethered (bench: opens COM8 + COM10 and reads the endpoint pose back)
tools\nav_run.cmd -Only 0,1400

:: untethered (acceptance: Bluetooth only, endpoint measured with a tape)
tools\bt_send.cmd "G 0 1400"
```

Frame convention: **+y is forward, +x is the car's right**, millimetres, +/-5000.

---

## Console and commands

**COM8 @115 200** (USART2):

| Key | Action |
|---|---|
| `s` / `g` | stop / restart the LiDAR (legacy, untouched) |
| `f` / `b` | both wheels forward/reverse at 40 %, 1.5 s auto-stop |
| `l` / `r` | left/right channel only, 40 %, 1.5 s |
| `x` | request stop |
| `c` | zero pose and both encoder totals (done by the Pose task, so it cannot tear) |
| `e` | read `L= R= cnt \| rpm \| x y \| h gyro` |
| `i` | **role 5: priority-inheritance demo** |
| `h` / `?` | help |

**COM10 @115 200** (Bluetooth, one command per line): `G <x_mm> <y_mm>` / `Z` (zero pose+path) / `S` / `P` / `?`

> The jog duty is **40 %**, not 20 %: on this pack, 20-30 % is a dead zone (neither
> wheel moves).

---

## Reading the output

Seven lines per second, all prefixed `[1s]`:

```
[1s] L: blk24 q0/8 p2435 | M: per10.00 jit42 wc11 | C: q0/4
[1s] P: eh90.0 ex0 ey0 | QP: 0 | PATH: g0 r0
[1s] I2C: ok1401 err1 hold1.5ms | H: 19816/32768 | WD: 0 | IMU: 100/s
[1s] STK: 680 504 464 1280 768 576 636 1096
[1s] T10: m10.00/42/11 p11.38/1383/12 | ov m0 p0
[1s] T50: b51.26/1267/281 o51.09/1100/285 | ov b0 o0
[1s] CNT: ql0 qc0 qpt0 imu0 bt(s0 r0 o0 u0) log0 iw0
```

| Line | Meaning |
|---|---|
| 1 | LiDAR block rate / `Q_Lidar` peak / worst parse time; motor task period, jitter, WCET; `Q_Cmd` depth |
| 2 | Pose: `eh` (heading x0.1 deg), `ex`/`ey` (mm); `Q_Path` depth; **cumulative** waypoints generated/reached |
| 3 | I2C ok/err/longest hold; heap; watchdog (not enabled); measured IMU rate |
| 4 | Stack free for the 8 tasks (PID/IMU/Pose/LiDAR/Path/BTRX/BTTX/OLED) |
| 5-6 | period/jitter/WCET + overruns for the four periodic tasks |
| 7 | Failure counters - **all zero is healthy** |

> `Pose` shows 11.38 ms instead of 10 ms: the IMU task's blocking I2C (<=1.5 ms)
> pushes it. By design; the report keeps it in the "ugly numbers" section rather
> than hiding it.

---

## Demo material

- One-pager: [`Docs/Block4.5_Part2_Demo_Flowcard_CN.docx`](Docs/Block4.5_Part2_Demo_Flowcard_CN.docx)
- Full script with 12 Q&A: [`Docs/Block4.5_Part2_Demo_Script_CN.docx`](Docs/Block4.5_Part2_Demo_Script_CN.docx)

---

## Bugs that will come back if you rewrite this

| # | Symptom | Root cause | Fix |
|---|---|---|---|
| A | seven banners printed, then the whole car freezes while TIM7 keeps ticking | stack-overflow hook sets BASEPRI=0x50, which cannot mask priority-0 TIM7 | stacks 512->1024, 1024->2048 |
| B | `G` generates waypoints but the car never moves | `osEventFlagsWait` returns `0xFFFFFFFD` on "no event"; `& 1 == 1` read as STOP | test `flags & 0x80000000` first |
| C | right wheel spins backwards on `f`, car pivots | motor on PB6/PB7 but fitted on the right, and inverted | swap the two groups in `board_pins.h`, keep `RIGHT_INVERT` |
| D | 28 % of commanded speed | integral clamp 120 x ki 0.15 = 2 % of output, i.e. no integral | retune kp/ki/clamp together |
| E | queue empty right after `G` | the test script's late `S` cleared the new path | let the command arrive before closing the port |
| F | right wheel 0.5x forward, 4.7x reverse | QEP input filter was 190 ns, so brush noise counted as edges | 3 us filter in `encoder.c` |
| G | a 1.4 m run reported "complete" after 1.2 s | `PATH: g/r` are cumulative since boot, not per-run | `QP == 0` plus pose-at-target |

Root-cause write-ups and measurements: report section 11.

---

## Repository layout

```
Core/Src/app_rtos.c     8 threads, 3 queues, 1 Hz status block, role-5 demo
Core/Src/app.c          boot order, NVIC priority table, HAL dispatch
Core/Src/motor.c        TIM4/TIM2 PWM + dual PID
Core/Src/encoder.c      TIM1/TIM8 hardware QEP (3 us input filter)
Core/Src/pose.c         odometry + gyro heading (gyro first, wheels as fallback)
Core/Src/path.c         L-shaped (axis-aligned) waypoint generation + heading control law
Core/Src/imu.c          MPU-6500, sampled in task context
Core/Src/i2c_bus.c      sole owner of I2C1: prio-inherit mutex, bounded transfers
Core/Src/oled.c         SSD1306 + 6x8 font (never run on a real panel)
Core/Inc/board_pins.h   single source of truth for pins/timers/NVIC
tools/                  bench_p0 / nav_run / bt_send / bt_link_check / flash_and_monitor / serial_console
tools/host_tests/       host-side unit tests (parser grammar, path generation, control law)
Docs/                   report, demo material, coding standards, handover
```

---

## Documentation

| Document | Contents |
|---|---|
| `Docs/Block4.5_Part2_Design_Report_CN.docx` | **design report**: architecture argument, measurements, seven bugs |
| `Docs/Block4.5_Part2_Demo_Script_CN.docx` | demo script + Q&A |
| `Docs/Block4.5_Part2_Demo_Flowcard_CN.docx` | one-page flowcard |
| `Docs/CODING_STANDARDS.md` | iron rules, task table, queue policy, observability format |
| `Docs/HANDOVER.md` | on-hardware record (bug scenes, debugging technique) |
| `Docs/NEXT_SESSION.md` | handover: current state, TODO, timeline |

---

## Third-party components

- FreeRTOS 10.3.1 + CMSIS-RTOS v2 wrapper (ST)
- STM32F4 HAL
- `pc_app/`: PC-side point-cloud viewer (Python + matplotlib, decodes the binary telemetry)

Course example code and lecture material are **not** part of this repository.
