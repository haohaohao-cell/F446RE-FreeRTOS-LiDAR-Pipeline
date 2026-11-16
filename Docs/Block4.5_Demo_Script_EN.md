# Block 4.5 — Live Demonstration Script

Duration target: **5 minutes demo + Q&A**.
Everything below is reproducible with one command; nothing is staged or pre-recorded.

---

## 0. Five minutes before you present

| # | Do this | Why |
|---|---|---|
| 1 | Car **on blocks**, wheels off the ground, then switch main power ON | firmware drives PB6/PB7 low at init, but never assume |
| 2 | Plug in the ST-Link USB (gives you SWD **and** COM8) | `COM8` is the ST-Link Virtual COM Port = USART2 |
| 3 | Confirm the LiDAR spins and is wired to USART3 (PC10/PC11) | no spin = no blocks = demo dead |
| 4 | Open a terminal on **COM8, 115200 8N1** (PuTTY / Tera Term / CubeIDE console) | this is your evidence screen |
| 5 | Run `tools\flash_and_monitor.cmd -Probe` | you should see `Target OK (voltage 3.2x V)`, `Device name : STM32F446xx` |
| 6 | Run `tools\flash_and_monitor.cmd -Seconds 20` once | confirms end-to-end before the TA walks over |
| 7 | Keep the log window open with the last capture visible | fallback evidence if hardware dies mid-demo |

**One-line sanity check that the pipeline is alive** — you should see this within 2 s:

```
[1s] blk 25/.. q1 0/8 q2 1/4 idle 5 latmax 14 parsemax 801 | L 24902B/s
     pkt ... inv 0 nr ... sync 1 slip 0 err 0 | heap 26488 min 26488 | stk 1256/768
```

If you see `blk 0` and a rising `idle`, the LiDAR is not sending — check power and USART3.

---

## 1. The demo, step by step

### Step 1 — One sentence of framing (15 s)

> "Block 3.5 read the LiDAR from a `while(1)` loop that polled flags. This Coding
> Day moves the same data path onto FreeRTOS, so the hand-over between interrupt
> and task context is explicit: DMA fills a buffer, the ISR only announces which
> half is ready, a task does the parsing, a second task does the presentation."

Then point at the serial output and say:

> "One 12-byte descriptor per block, 25 blocks per second, and the parser never
> falls behind."

### Step 2 — Show the pipeline running (60 s)

Let the terminal scroll. Point at, in this order:

1. `#0474 p205 ok151 nr54 iv0 n115 d[47..10574] lat12 p765 S ...`
   > "Each line is one DMA block: 205 packets decoded, 151 valid, 54 no-return,
   > 0 framing errors. `n115` is the number of distinct angle bins that block
   > touched. `lat12` is 12 microseconds from the ISR to the parser running."
2. `[1s] blk 25/472 q1 0/8 ...`
   > "25 blocks per second. **`q1 0/8`** is the important one — the peak occupancy
   > of Queue 1 was zero, meaning the parser always drained the queue before DMA
   > wrapped around. That is the measurement that justifies the 1024-byte block."
3. `L 24902B/s` and `pkt 96662`
   > "24.9 kilobytes per second, about 5 100 packets per second. That matches the
   > C1 datasheet's ~5 000 samples per second, so we are not losing data."
4. `inv 0 slip 0 err 0 sync 1`
   > "Framing errors zero, resync events zero, UART errors zero, and the SCAN
   > descriptor was found once at start-up. A 5-byte packet can straddle two DMA
   > blocks, and these three counters prove the cross-block state machine is right."
5. `heap 26488 min 26488 | stk 1256/768`
   > "6.2 of 32 kilobytes used, and free equals minimum-free, so nothing leaks.
   > 1256 bytes of stack still free in the parser, 768 in the output task."

### Step 3 — Prove it is really RTOS-driven, not polling (45 s)

Do this live, it takes 10 seconds:

1. **Cover the LiDAR with your hand / unplug its USB** (or just point it at a wall).
   - `d[..]` min/max distance changes, `n` changes, `S` sector summary changes.
   - `blk` stays 25, `q1` stays 0 → the pipeline is still healthy, only the data changed.
2. **Stop the sensor completely** (unplug the LiDAR).
   - **Which cable:** the LiDAR's own 4/5-pin cable, pulled at the **LiDAR end** by
     the plastic connector housing (never by the wires). Do **not** pull the ST-Link
     USB cable - that is COM8 and your flashing port - and do **not** switch off the
     car's main power, that kills the MCU and the terminal with it.
   - `blk` goes to 0, `idle` starts climbing, `cmd` starts climbing (the watchdog
     re-sending SCAN), `sync` resets, `err` may tick once.
   - **Nothing hangs.** The tasks stay responsive, the 1 Hz line keeps printing.
   > "This is the failure mode that would deadlock a naive `osWaitForever` design.
   > The parser wakes every 100 ms; when no DMA block has arrived for 1 s it drops
   > the stream lock and re-sends the SCAN command once per second - that is `cmd`
   > climbing. The C1 never streams by itself after a power-up, so without this it
   > would stay silent forever. Plug it back in and it recovers in 1-2 s."

Then plug it back in and show that `blk` returns to about 25, `idle` stops growing
and `cmd` freezes again.
That recovery is the single most convincing thing you can show.

### Step 4 — The three-layer separation, on screen (30 s)

> "Three layers, three places to look:
> - **Interrupt layer** — `lidar_on_half()` / `lidar_on_full()` in `lidar.c`:
>   they build a 12-byte descriptor and post it. No parsing.
> - **Parsing layer** — `parser_task()` in `app_rtos.c` calls
>   `lidar_parser_process()`, which reads the block *out of the DMA buffer* using
>   the offset from the queue.
> - **Presentation layer** — `output_task()` only formats and transmits.
>
> The LiDAR bytes are never copied into a queue — only the offset and length are.
> That's why Queue 1 carries 12 bytes instead of 1024."

---

## 2. What to have open on screen

| Window | What it proves |
|---|---|
| Serial terminal on COM8 (scrolling) | the system is live, and every number in the report |
| `Core/Src/app_rtos.c` — `parser_task()` at line 106 | blocking on Queue 1, posting to Queue 2 |
| `Core/Src/lidar.c` — `post_block()` at line 125 | the ISR is trivial: 12-byte struct, no parsing |
| `Core/Src/lidar.h` — the two structs | 12 B descriptor vs 44 B result |
| `Core/Src/app.c` — `irq_priorities()` at line 81 | priority 5 for the LiDAR ISRs, and why |
| `Docs/Block4.5_Design_Report.md` §11 | the two hardware-only bugs you found and fixed |

Do **not** try to show more than these. Six windows is already too many.

---

## 3. Q&A cheat sheet

The eight questions the brief says you must be able to answer, with a short answer
and the number or file that backs it.

| Question | Answer (say this) |
|---|---|
| How does DMA fill the buffer half by half? | "One 2048-byte circular buffer, DMA1_Stream1 in circular mode. Half-complete IRQ means half 0 is done, complete IRQ means half 1 is done. The parser reads the half that DMA is *not* writing, so no lock is needed." |
| How do the half/complete events identify a ready block? | "`HAL_UART_RxHalfCpltCallback` → `lidar_on_half()` → offset 0; `HAL_UART_RxCpltCallback` → `lidar_on_full()` → offset 1024. Each posts one descriptor with that offset." |
| Why must the ISR stay short? | "Measured: handover latency 12–15 µs including the context switch. If the ISR parsed instead, it would be 801 µs of measured work per block, 25 times a second, added as non-preemptible latency to the IMU I2C and the UART5 DMA." |
| Why does Queue 1 carry only a small descriptor? | "So the ISR does no copying: 12 bytes instead of 1024, an 85× reduction. The data is already in the DMA buffer, so `offset`+`length` is all the task needs. The queue itself costs 96 bytes instead of 8 KB." |
| How do both tasks move between BLOCKED / READY / RUNNING? | "Parser: BLOCKED in `osMessageQueueGet` on Queue 1, the ISR's `portYIELD_FROM_ISR` makes it READY, it RUNs and posts to Queue 2. Output: BLOCKED on Queue 2, parser's put makes it READY. The parser is BLOCKED more than 99.5 % of the time — `q1_peak = 0` shows that." |
| Why must the parser outrank the output task? | "The parser's deadline is set by hardware: DMA overwrites a half buffer after 40 ms. The output task's deadline is a 100 ms frame for a human. Missing the parser deadline corrupts data; missing the output deadline prints a line late." |
| What FreeRTOS memory objects are created? | "2 queues (8×12 B and 4×44 B, ~1.2 KB), 2 × 1536/2048 B task stacks, 2 TCBs, plus the idle task and the timer daemon that `configUSE_TIMERS` forces. Total 6.2 KB of the 32 KB `heap_4` heap — `heap 26488 min 26488` proves it never grows." |
| Why is the parser priority 32 and output 24? | "`osPriorityAboveNormal` vs `osPriorityNormal`, from the CMSIS-RTOS v2 enum. Not arbitrary: one step apart is enough because the parser never waits on the output task." |

### Extra questions a sharp TA will ask

| Question | Answer |
|---|---|
| What happens if the parser falls behind? | "Queue 1 fills up (8 deep = 320 ms of slack). On the 9th descriptor the ISR drops it and increments `q_full`; the gap in `block_id` makes the loss visible instead of silent. Measured peak was 0, so it never happened." |
| Is there a race on the 360-bin scan map? | "One writer (parser task), one reader (output task). The reader copies under `taskENTER_CRITICAL()`, which stops the kernel from switching back mid-copy. A mutex would only add priority-inversion risk — there is nothing to block on." |
| Why not a semaphore instead of a queue? | "A semaphore carries no data. The parser needs to know *which* half is ready and which sequence number it is — that is 12 bytes of information, so a queue is the right primitive." |
| Why does the output task have a timeout at all? | "If the LiDAR stops there are no results, so the task would never service the IMU or print the 1 Hz line. The timeout keeps the presentation side alive independent of the data path." |
| Why is the HAL tick on TIM7 and not SysTick? | "SysTick belongs to the kernel. TIM6 is already the IMU's 100 Hz trigger, so TIM7 was the free basic timer. This is also what the lecture notes recommend." |
| What was the hardest bug? | **Have the answer ready — this is your best moment.** "Moving the tick to a TIM made HAL_GetTick() stop working entirely. FreeRTOS's ARM_CM4F port initialises `uxCriticalNesting` to `0xaaaaaaaa` and only zeroes it in `xPortStartScheduler()`, so every `osThreadNew` before the scheduler leaves BASEPRI at 0x50 forever. BASEPRI masks peripheral interrupts but not core exceptions, so SysTick-based ticks never cared and TIM-based ticks die. Fix: TIM7 at priority 0, above the syscall ceiling, which is safe because it calls no RTOS API." |

---

## 4. Numbers to memorise (only these four)

If you remember nothing else, remember these — they are the whole story:

| Number | Meaning |
|---|---|
| **1024 B / 40 ms / 25 blocks per s** | the block strategy |
| **12 bytes vs 1024 bytes** | why Queue 1 is small, why the ISR is short |
| **q1 peak = 0** | the parser never fell behind — the design works |
| **12 µs vs 801 µs** | handover vs parse — why the split is in the right place |

---

## 5. Optional but strong: the live point-cloud window

`pc_app/main.py` decodes the **binary telemetry** the firmware sends on UART5
(230400) and draws the 360-bin scan. It is the visual proof that the pipeline is
producing a real 2-D map, not just log lines.

Launch it (a window titled *CodingDay - RPLIDAR C1 + MPU6500* appears):

```
D:\嵌入式小车\CodingDay_F446RE\pc_app\run_viewer.cmd
```

It defaults to **COM10**, which is the outgoing SPP port of the paired **HC-04**
module (MAC `0426040302F4`). `run_viewer.cmd -l` lists the ports if Windows
renumbers it, and `run_viewer.cmd --sim` runs it with synthetic data and **no
hardware at all** — useful as a rehearsal fallback.

> **Baud note you may be asked about.** The firmware transmits on UART5 at
> 230400, but Windows refuses to open a Bluetooth SPP virtual COM port above
> 115200. That number is nominal for SPP — the radio link does the real work — so
> the viewer is opened at 115200 and nothing is throttled. Measured throughput
> is ~6.4 kB/s, which matches the firmware's ~8.7 frames/s.

What is worth showing while it runs:

| On screen | Say |
|---|---|
| The 2-D scatter (left) | "A full 360° ring at 1 cm resolution, rebuilt on the PC from the 720-byte distance array the parser task produced." |
| The polar heading dial (right) | "That is the MPU-6500 yaw from the same telemetry frame — the LiDAR and IMU are delivered together." |
| `fps 8.7  frames …  crc_err 0` | "The PC decodes every frame with a XOR checksum and never had to reject one." |
| `LiDAR … B/s  pkt …  inv …  max … us` | "These are the same counters as the `[1s]` line on COM8 — 24.9 kB/s, 0 invalid, 801 µs worst parse. Two independent paths agree." |
| Moving your hand in front of the sensor | "The ring deforms in real time — that is the ISR → queue → parser → queue → output chain end to end." |

**If the window does not appear:** check that the car is powered (the HC-04 only
answers then), and that COM10 still belongs to HC-04. In the worst case use
`--sim` and explain that the PC decoder and the firmware share one frame format.

---

## 6. If the hardware fails during the demo

1. Say so immediately. Do not fiddle silently.
2. Open `tools/logs/serial_*.txt` — the captured runs are on disk with timestamps.
3. Walk through the same numbers from the log, then explain the architecture from the code.
4. Fallback: `tools\flash_and_monitor.cmd -Probe` distinguishes "board unpowered"
   from "firmware hung" in one command — that is itself a good debugging story.

Common causes and the one-line check:

| Symptom | Cause | Check |
|---|---|---|
| `blk 0`, `idle` climbing | LiDAR silent | LiDAR power / USART3 PC10-PC11 / SCAN command |
| Nothing on the terminal | wrong COM port | `-Probe` shows the target; COM8 is the ST-Link VCP |
| Terminal silent **and** `-Probe` fails with 0.1 V | board unpowered | main power switch |
| Hangs immediately after reset | `configASSERT` / malloc / stack hook | pause with the debugger, read the PC |
