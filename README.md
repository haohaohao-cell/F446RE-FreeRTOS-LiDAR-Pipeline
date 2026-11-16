# F446RE FreeRTOS LiDAR Pipeline

Block 4.5 Coding Day — an interrupt-driven RPLidar C1 data pipeline on
**STM32F446RE + FreeRTOS 10.3.1 (CMSIS-RTOS v2)**.

DMA fills a circular buffer half by half, the ISR only announces which half is
ready, a parser task reads that block straight out of the DMA buffer, and a
second task prints/sends the result. Nothing is parsed in interrupt context.

```
        USART3 + DMA1_Stream1 (circular, 2048 B = 2 x 1024 B)
                         |
     half-complete IRQ -> lidar_on_half()     --+
     complete IRQ      -> lidar_on_full()     --+--> Queue 1 (8 x 12 B descriptor)
     UART error IRQ    -> lidar_on_uart_error()+             |
                                                             | osMessageQueueGet()
                                                             v
                                             LiDAR Parser Task (osPriorityAboveNormal)
                                                             |
                                          +------------------+------------------+
                                          |                                     |
                            360-bin scan map (shared)             Queue 2 (4 x 44 B result)
                            ~8 Hz telemetry frame                              |
                                                                               v
                                                            Output/Debug Task (osPriorityNormal)
                                                                  |                     |
                                                         USART2 text 115200     UART5 binary 230400
                                                                  |                     |
                                                        ST-Link VCP (COM8)    HC-04 BT SPP (COM10)
```

*中文说明见 [README.zh-CN.md](README.zh-CN.md).*

---

## Measured on real hardware

| Metric | Value | How it is read |
|---|---|---|
| DMA block rate | **25–26 blocks/s** (40 ms per half buffer) | `blk` in the 1 Hz line |
| Queue 1 peak occupancy | **0 / 8** | `q1` — the parser never falls behind |
| ISR → task latency | **12–15 µs** | `lat` / `latmax` |
| Parser time per block | **~800 µs** (`-O0`) → 2 % of the block period | `parsemax` |
| LiDAR byte rate | **24.9 – 25.3 kB/s** ≈ 5 100 packets/s | `L` — matches the C1 datasheet |
| Framing errors / resyncs / UART errors | **0 / 0 / 0** over ~1 M packets | `inv` / `slip` / `err` |
| FreeRTOS heap | 6.2 kB of 32 kB used, `free == min` | `heap` |
| Task stack left | 1256 B (parser) / 784 B (output) | `stk` |
| HAL tick vs kernel tick | **1.000 : 1** | `hal` / `k` |

Full analysis, design justification and the three hardware-only bugs that were
found and fixed are in [`Docs/Block4.5_Design_Report_EN.docx`](Docs/Block4.5_Design_Report_EN.docx)
(Chinese: [`Docs/Block4.5_Design_Report_CN.docx`](Docs/Block4.5_Design_Report_CN.docx)).

---

## Hardware

| Item | Detail |
|---|---|
| MCU | STM32F446RE (Nucleo-F446RE compatible), 168 MHz, 128 kB SRAM |
| LiDAR | RPLidar C1, SCAN mode, USART3 @ **460 800** baud, 8N1 (PC10 TX / PC11 RX) |
| LiDAR DMA | DMA1_Stream1, circular, 2048 B |
| IMU (optional) | MPU-6500 over I2C1 (PB8 SCL / PB9 SDA), 100 Hz sampled by TIM6 |
| Bluetooth (optional) | HC-04 on UART5 @ 230 400, carries the binary telemetry |
| Debug console | USART2 @ 115 200 (PA2 / PA3) → ST-Link Virtual COM Port |
| HAL time base | TIM7 @ 1 kHz (TIM6 is taken by the IMU) |

The LiDAR path works on its own; the IMU and the Bluetooth module are optional
and their absence only shows up as `IMU 0` / no point-cloud data.

---

## Repository layout

```
Core/               application code
  Src/app.c           boot sequence, NVIC priority map, HAL callbacks
  Src/app_rtos.c      queues, Parser Task, Output/Debug Task, console commands
  Src/lidar.c         C1 driver: ISR descriptor post, block parser, watchdog
  Src/stm32f4xx_hal_timebase_tim.c   HAL tick on TIM7 + DWT calibration
  Inc/FreeRTOSConfig.h               kernel configuration
  Inc/app_rtos.h, lidar.h            queue/task sizes, message structs
Middlewares/Third_Party/FreeRTOS/    FreeRTOS 10.3.1 + CMSIS-RTOS v2 + heap_4
Drivers/                             ST HAL + CMSIS (CubeF4 1.28.3)
pc_app/             PC side
  main.py             matplotlib viewer for the binary telemetry
  run_viewer.cmd      one-click launcher
tools/              helper scripts
  flash_and_monitor.cmd   probe → program → verify → capture USART2 → verdict
  serial_console.cmd      live console, type s / g / h
Docs/               report + demo material (EN and CN, .md and .docx)
CodingDay_F446RE.ioc, .cproject     STM32CubeIDE project files
```

---

## Quick start

### 1. Get the code

```bash
git clone https://github.com/haohaohao-cell/F446RE-FreeRTOS-LiDAR-Pipeline.git
```

### 2. Build

**STM32CubeIDE** (recommended for teammates)

1. *File → Import → General → Existing Projects into Workspace*
2. Select the cloned `F446RE-FreeRTOS-LiDAR-Pipeline` folder — the
   `CodingDay_F446RE` project appears.
3. **Project → Build Project** (Ctrl+B).

Expected result: **0 warnings, 0 errors**, ~55 kB flash, 42 kB RAM.

> The FreeRTOS include paths are already in `.cproject`, so nothing else needs
> configuring. Do **not** open the `.ioc` and press *Generate Code* — see
> [Gotchas](#gotchas).

**Command line** (if you have the CubeIDE toolchain on `PATH`)

```bash
make -C Debug -j8 all
```

### 3. Flash

```cmd
tools\flash_and_monitor.cmd
```

It checks the ST-Link can see the target, programs `Debug\CodingDay_F446RE.elf`,
verifies it, resets, then captures 15 s of the debug console and prints a verdict.

* Only want to know whether the board is alive? `tools\flash_and_monitor.cmd -Probe`
* Different toolchain location? `-Cli "C:\path\to\STM32_Programmer_CLI.exe"`
* No flash, just listen: `-NoFlash -Seconds 30`

### 4. Watch it run — two independent links

| What | Command | Port |
|---|---|---|
| **Text console** (`blk`, `idle`, `q1`, `lat`, `heap`, `stk`) | `tools\serial_console.cmd` | COM8 @ 115200 |
| **Point cloud** (2-D scan + heading dial) | `pc_app\run_viewer.cmd` | COM10 @ 115200 (Bluetooth) |

Both can run at the same time — they are different physical links.

The console is **interactive**:

| Key | Action |
|---|---|
| `s` | stop the LiDAR (no cable touching needed) |
| `g` | restart it |
| `h` / `?` | help |
| `Esc` / `Ctrl+C` | quit |

The viewer needs `matplotlib`, `pyserial`, `tkinter`:

```bash
py -3 -m pip install matplotlib pyserial
```

If the port is not COM10, find it with `pc_app\run_viewer.cmd -l`, or use
`run_viewer.cmd COM7`. `run_viewer.cmd --sim` runs it with synthetic data and no
hardware at all.

---

## What the console output means

**One line per DMA block** (`#`, ~8 Hz)

```
#0474 p205 ok151 nr54 iv0 n115 d[47..10574] lat12 p765 S 255 78 36 8 4 255 255 255
```

| Field | Meaning |
|---|---|
| `#0474` | block id — a gap means a descriptor was lost |
| `p205` | 5-byte LiDAR packets decoded in this block |
| `ok151 / nr54 / iv0` | valid / no-return / framing-invalid |
| `n115` | distinct angle bins touched |
| `d[47..10574]` | min / max distance, mm |
| `lat12` | **ISR → parser wake-up latency**, µs |
| `p765` | parser time for this block, µs |
| `S ...` | 8-sector obstacle summary, cm (255 = no return) |

**One line per second** (`[1s]`)

```
[1s] blk 25/447 q1 0/8 q2 1/4 idle 5 latmax 14 parsemax 801 | L 25259B/s
     pkt 91541 inv 0 nr 17495 sync 1 slip 0 err 0 cmd 1/0 | IMU 0 err 0
     | BT f 161 dr 0 | heap 26488 min 26488 | stk 1256/784 | hal 1054 k 1053
```

| Field | Meaning | Healthy value |
|---|---|---|
| `blk` | blocks in the last second / total | ~25 /s |
| `q1` | **peak** Queue 1 occupancy | 0–1 (8 = parser behind) |
| `q2` | peak Queue 2 occupancy | 0–2 |
| `idle` | Queue 1 wait timeouts | small, frozen in steady state |
| `latmax` | worst ISR → task latency | < 50 µs |
| `parsemax` | worst parse time | < 1 000 µs |
| `L` | measured LiDAR byte rate | ~25 000 B/s |
| `sync` | SCAN descriptor found | 1 (grows when the stream is re-locked) |
| `cmd` | SCAN commands sent / stream watchdog losses | `1 / 0` (both grow only if the sensor drops) |
| `err` | UART errors | 0 |
| `heap` | FreeRTOS heap free / lowest ever | equal (no leak) |
| `stk` | stack high-water **free**, parser / output | both > 500 B |
| `hal` / `k` | HAL tick vs kernel tick over the same second | 1:1 |

---

## Demonstrating the recovery (the important 60 seconds)

This is the part that proves the design is queue-driven rather than polling.

1. In the console window, type **`s`**.
   * `blk` → 0, `L` → 0 B/s, `idle` starts climbing (one timeout per 100 ms).
   * **Nothing hangs**: both tasks stay responsive and the 1 Hz line keeps printing.
2. Wait 5 s, then type **`g`**.
   * `cmd` jumps (`3/0 → 4/1 → 5/2`): the stream watchdog noticed 1 s without a DMA
     block and re-sent the SCAN command.
   * Within ~3 s `blk` returns to 25, `idle` freezes, `sync` goes 1 → 2.

Why it matters: the RPLidar C1 **does not stream after a power-up** — it always
needs the `0xA5 0x20` SCAN command again. A design that blocks on the queue with
`osWaitForever` would deadlock here forever. The 100 ms guard timeout is what lets
the parser run its stream watchdog.

Unplugging the LiDAR works too and shows the same thing, but typing `s` cannot
break the hardware — use it for the live demo.

---

## Gotchas

* **Two different COM ports.** COM8 is the ST-Link VCP = text debug (USART2).
  COM10 is the Bluetooth SPP port = binary telemetry (UART5). `blk` / `idle` only
  exist on COM8; the point-cloud window only shows the binary stream.
* **A Bluetooth SPP port cannot be opened above 115200** on Windows. The firmware
  transmits at 230 400, but the SPP baud is nominal — open the viewer at 115200
  and nothing is throttled (measured ≈ 6.4 kB/s of telemetry).
* **HAL time base is calibrated at boot.** `stm32f4xx_hal_timebase_tim.c` measures
  TIM7 against DWT because on this board TIM7 turned out to run at HCLK (168 MHz)
  instead of the documented 2 × PCLK1 (84 MHz). The boot banner prints
  `timebase: ... calib=40/20` and a `50 ms (dwt) -> 50 ms (hal) OK` check.
  If the tick is ever wrong, every `HAL_GetTick()` timeout and every measured byte
  rate is wrong by the same factor — check that line first.
* **Do not press "Generate Code" in STM32CubeMX.** The `.ioc` does not know about
  FreeRTOS (no `FREERTOS` IP was injected, to avoid corrupting the project). A
  regeneration would rewrite `.cproject` and drop the FreeRTOS include paths and
  the `Middlewares` source folder. If you must regenerate, re-add both, plus the
  TIM7 priority-0 and the boot-time calibration.
* **The IMU is optional.** `imu=-1, err=0x4, who=0x00` in the banner and `IMU 0`
  in the 1 Hz line simply mean the MPU-6500 is not connected; the LiDAR pipeline
  is unaffected.

---

## Documentation

| File | Contents |
|---|---|
| `Docs/Block4.5_Design_Report_EN.docx` / `_CN.docx` | full analysis: architecture, block-size justification, queue message design, interrupt priority rules, measured results, the three hardware-only bugs |
| `Docs/Block4.5_Demo_Flowcard_EN.docx` / `_CN.docx` | one-page demo card: 3 actions + a 4-person roles table |
| `Docs/Block4.5_Demo_Script_EN.docx` / `_CN.docx` | long-form demo script + Q&A cheat sheet |
| `Docs/*.md` | the Markdown sources of the above |

`tools/md2docx.py` regenerates the `.docx` files from the Markdown
(needs `python-docx`):

```bash
py -3 tools/md2docx.py Docs/Block4.5_Design_Report.md Docs/out.docx --lang en
```

---

## Third-party components

| Component | Version | License |
|---|---|---|
| FreeRTOS kernel | 10.3.1 | MIT (see `Middlewares/Third_Party/FreeRTOS/LICENSE`) |
| CMSIS-RTOS API v2 wrapper | as shipped with CubeF4 1.28.3 | Apache-2.0 |
| STM32F4 HAL / CMSIS device | CubeF4 1.28.3 | BSD-3-Clause (ST) |

Project code in `Core/`, `pc_app/` and `tools/` is coursework; use it as a
reference, not as a submission of your own.
