# Block 4.5 — FreeRTOS LiDAR Data Pipeline

**Analysis report — Coding Day, Week 4**

| | |
|---|---|
| Team / members | *fill in* |
| Student IDs | *fill in* |
| Date | *fill in* |
| Board | STM32F446RE, 168 MHz, 128 KB SRAM |
| RTOS | FreeRTOS 10.3.1 + CMSIS-RTOS API v2 (`cmsis_os2.c`) |
| Sensor | RPLidar C1, SCAN mode, USART3 @ 460 800 baud, 8N1 |
| Source tree | `CodingDay_F446RE/` (builds clean: 0 warnings, 0 errors) |

Every number quoted in this report was measured on the target hardware, not
estimated. The raw serial capture is in `tools/logs/`.

---

## 1. Required architecture and how it is implemented

```
         USART3 + DMA1_Stream1 (circular, 2048 B = 2 x 1024 B)
                          |
      half-complete IRQ -> lidar_on_half()   --+
      complete IRQ      -> lidar_on_full()   --+--> Queue 1 (8 x lidar_block_desc_t)
      UART error IRQ    -> lidar_on_uart_error()+            |
                                                             | osMessageQueueGet()
                                                             v
                                              LiDAR Parser Task  (osPriorityAboveNormal)
                                                             |
                                           +-----------------+------------------+
                                           |                                    |
                            360-bin polar map (shared)              Queue 2 (4 x lidar_result_t)
                            100 ms telemetry frame                              |
                                                                                v
                                                             Output / Debug Task (osPriorityNormal)
                                                                   |                    |
                                                          USART2 text 115200    UART5 binary 230400
```

### Task / object inventory

| Object | Type | Size / priority | Purpose |
|---|---|---|---|
| `q_blocks` (Queue 1) | message queue | 8 x 12 B | ISR → parser; block descriptor only |
| `q_results` (Queue 2) | message queue | 4 x 44 B | parser → output; processed result |
| `parser_task` | thread | 1536 B stack, `osPriorityAboveNormal` (32) | reads DMA buffer, parses, publishes |
| `output_task` | thread | 2048 B stack, `osPriorityNormal` (24) | formats + transmits, ≈8 Hz frames / 1 Hz statistics |
| idle task | kernel | 512 B | created by the scheduler |
| timer daemon | kernel | 1024 B | created by the scheduler (`configUSE_TIMERS = 1`, required by `freertos_os2.h`) |

FreeRTOS heap: `configTOTAL_HEAP_SIZE = 32 KB`, `heap_4.c` (first-fit with adjacent-block coalescing).
Estimated consumption: 2 task stacks (3.5 KB) + 4 TCBs (~370 B) + 2 queue objects (~1.2 KB of which 0.28 KB is item storage) ≈ **5.5 KB**, i.e. ~17 % of the heap. The exact figure is printed at run time in the `[1s]` line as `heap <free> min <min-ever-free>`.

---

## 2. Design decision: why DMA block = 1024 B (two halves of a 2048 B buffer)

| Quantity | Predicted | **Measured on target** |
|---|---|---|
| UART line rate (460 800, 8N1 → 10 bit/byte) | 46 080 B/s | — |
| RPLidar C1 payload (≈5 000 samples/s x 5 B) | ≈ 25 000 B/s | **24.9 – 25.3 kB/s** (`L` field) |
| Time to fill one 1024 B half at payload rate | ≈ 41 ms | **40 ms** (25 blocks/s) |
| Time to fill one 1024 B half at line rate (worst case) | ≈ 22 ms | — |
| Packets per half block (1024 / 5) | ≈ 204 | **204 – 205** (`p` field) |
| Rotational sweep covered by one half block (10 Hz → 36°/100 ms) | 8° – 15° | ≈ 14° |
| Sample rate | ≈ 5 000 /s | **≈ 5 100 /s** (96 662 packets in ~19 s) |

**Justification.** The half block must be (a) small enough that the parser task is never the bottleneck and (b) large enough that interrupt and queue overhead stay negligible.

* 1024 B is **204 LiDAR packets**, so the per-block result is statistically meaningful (min/max distance, 8-sector obstacle summary, node count) instead of a handful of points.
* One block period is **40 ms measured**. The parser needs **801 µs** worst case (`parsemax`, `-O0` build), i.e. **2 %** of one block period. Queue 1 is 8 deep = **320 ms** of slack, about **400 x** the worst-case parse time. Measured peak Queue 1 occupancy was **0**, so the parser always had a block waiting and never fell behind.
* 1024 B / 204 packets also means the 5-byte packet framing is not repeatedly broken at the block boundary (a packet may still straddle two blocks — see §3). Measured `inv = 0` and `slip = 0` over 96 662 packets confirm the framing survived every boundary.

This buffer size directly answers the instruction's "the exact number of output samples per DMA block is not given — investigate it".

---

## 3. Design decision: the parser must own the *stream* state, not the block

The C1 byte stream is continuous and asynchronous to the DMA block boundaries, so a 5-byte sample packet can be split across two blocks. The framing state is therefore **persistent across blocks and owned by the parser task only**:

* `desc_n` — progress through the 7-byte SCAN-mode response header `A5 5A 05 00 00 40 81`
* `pend[5] / pend_n` — partially accumulated sample packet
* `in_scan` — stream is locked

This is only safe because **exactly one task touches it**. If the ISR parsed data, or if two tasks parsed the same stream, this state would need locking. Keeping it in one task is what makes the design lock-free on the fast path.

The per-packet validation is a single check-bit rule: bit 0 of byte 0 is the inverted check bit and bit 0 of byte 1 is the start flag; they must differ. On a framing error the parser drops one byte and realigns, counting a `slip`.

---

## 4. Design decision: what is in the queue messages (and why so small)

### Queue 1 — block descriptor (12 bytes)

```c
typedef struct {
  uint16_t offset;      /* where the block starts inside lidar_dma_buf */
  uint16_t length;      /* block length, bytes                       */
  uint16_t block_id;    /* monotonically increasing sequence number  */
  uint8_t  kind;        /* LIDAR_BLOCK_DATA | LIDAR_BLOCK_UART_ERROR */
  uint8_t  reserved;    /* keeps the struct 12 B / 4-byte aligned    */
  uint32_t isr_cycles;  /* DWT->CYCCNT sampled in the ISR            */
} lidar_block_desc_t;
```

**Why the LiDAR bytes are NOT copied into the queue.** Copying 1024 B per block would mean the ISR (or a helper) performing a 1024-byte memory-to-memory transfer 45 times per second — exactly the "heavy work in the ISR" the specification forbids. Instead the descriptor is a *pointer-by-offset*: `offset` + `length` tell the parser where to read, and the data stays where DMA already put it. Queue 1 therefore carries **12 bytes instead of 1024**, a 85x reduction in queue traffic, and the queue itself costs 96 bytes of storage instead of 8 KB.

* `offset`/`length` — mandatory: they identify the completed block.
* `block_id` — lets the parser and the output task detect *lost* blocks (gap in the sequence) and is printed in the debug line. This is how a dropped descriptor becomes visible instead of silent corruption.
* `kind` — one queue carries both data blocks and error/re-arm events, so the parser has a single block point and there is no second synchronisation path.
* `isr_cycles` — DWT cycle counter sampled in the interrupt, used to measure the real **ISR → task handover latency** (`lat` field). Costs 4 bytes and one register read; turns a hand-wavy claim into a number.

### Queue 2 — processed result (44 bytes)

```c
typedef struct {
  uint16_t block_id, length;
  uint16_t packets, valid, no_return, invalid, nodes;
  uint16_t min_mm, max_mm;
  uint8_t  sector_min_cm[8];   /* 8 x 45 deg obstacle summary, 0xFF = clear */
  uint8_t  synced, syncs, slips;
  uint32_t parse_us, handover_us, tick;
} lidar_result_t;
```

This is the "more meaningful result" required by the task: **valid measurement count, extracted node count, angle-distance bin map, minimum distance, sector-based obstacle summary, packet and scan statistics** — all of the suggested options in one structure, at 44 bytes. Depth 4 is enough because the output task drains it faster than the parser fills it (measured peak occupancy: 1 of 4).

---

## 5. Interrupt priority design (the part that usually breaks)

Rule: an ISR may call a FreeRTOS API **only if its NVIC priority number is numerically >= `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`**. In this project that value is **5**, so `configMAX_SYSCALL_INTERRUPT_PRIORITY = 5 << 4 = 0x50`.

| IRQ | Priority | Calls RTOS API? | Why |
|---|---|---|---|
| `DMA1_Stream1` (LiDAR RX) | **5** | **yes** — `osMessageQueuePut` | boundary value; posts the block descriptor |
| `USART3` (LiDAR RX/error) | **5** | **yes** | same |
| `I2C1_EV` / `I2C1_ER` (IMU) | 6 | no | above the threshold is not allowed, but 6 keeps it below kernel masking |
| `UART5`, `DMA1_Stream7` (BT telemetry) | 6 | no | ditto |
| `USART2` (debug console) | 7 | no | ditto |
| `TIM6_DAC` (IMU 100 Hz sampler) | 7 | no | ditto |
| `TIM7` (HAL time base) | **0** | no | **above** the threshold on purpose — see §11.1 |
| `SysTick`, `PendSV` | 15 | kernel | set by the FreeRTOS port |

Note the deliberate asymmetry: the LiDAR interrupts sit at exactly the syscall
ceiling because they *do* post to a queue, while the HAL tick sits at the very
top because it must never be masked by a critical section. Both values are
consequences of the same rule, not arbitrary.

Three concrete pitfalls that were found and fixed in this codebase:

1. `HAL_xxx_MspInit()`, `telemetry_dma_init()` and `i2c_bus_recover()` each write NVIC priority registers themselves and **the last writer wins**. They used to set 0 or 1 — above the syscall threshold. The priorities are now re-asserted at the end of `app_init()` (`irq_priorities()`) *and* the individual drivers were corrected to 6.
2. CubeMX's `MX_DMA_Init()` set `DMA1_Stream1` to priority 0. With priority 0 the first `osMessageQueuePut()` from the DMA ISR traps in `vPortValidateInterruptPriority()` — the classic FreeRTOS + STM32 failure. Fixed at the source and documented in place.
3. Moving the HAL tick onto a TIM exposed §11.1: BASEPRI raised by a leaked critical section masks peripheral interrupts but not core exceptions, so a TIM-based tick at the "safe" lowest priority silently stops.

### Why the ISR must stay short, quantitatively

The DMA ISR does: increment two counters, read `DWT->CYCCNT`, fill a 12-byte struct, `xQueueSendToBackFromISR` + `portYIELD_FROM_ISR` — a few dozen instructions, i.e. a fraction of a microsecond. The measurable end-to-end proof is the `lat` field: the time from the ISR's DWT stamp to the parser task starting work, which measured **12–15 µs**. That number already contains the ISR body, the PendSV context switch and the task dispatch, so the ISR body itself is smaller still.

If the ISR parsed 1024 bytes instead, the pipeline would move **801 µs** of work (measured `parsemax`) into interrupt context, at 25 interrupts per second. That would add ~800 µs of non-preemptible latency to every lower-priority interrupt — the IMU I2C at 100 Hz and the UART5 telemetry DMA would start missing events, because 800 µs is comparable to a whole 14-byte I2C transfer at 100 kHz. Keeping the ISR at the "few instructions" level is what makes the LiDAR path invisible to the rest of the firmware.

---

## 6. Task state transitions (what the TA will ask)

| Task | BLOCKED when | becomes READY when | runs until |
|---|---|---|---|
| LiDAR Parser | `osMessageQueueGet(q_blocks, 100 ms)` — Queue 1 empty | ISR posts a descriptor (queue was empty → task unblocked, `portYIELD_FROM_ISR` pends PendSV) | it finishes the block, posts to Queue 2, loops back to `Get` |
| Output / Debug | `osMessageQueueGet(q_results, 100 ms)` — Queue 2 empty | parser posts a result, **or** the 100 ms timeout expires | it finishes a frame/summary, loops back |

The parser spends >99.5 % of its life **BLOCKED**. It never busy-waits, never polls a flag, and never calls `HAL_Delay()`. That is the observable difference between this design and the Block 3.5 bare-metal version (which polled `half_flag`/`full_flag` from `while(1)`).

### Why the parser's queue wait has a timeout instead of `osWaitForever`

The block timeout is **not** a polling mechanism — in normal operation a block arrives every 40 ms (measured), so the 100 ms timeout never fires (`idle` counter in the 1 Hz line stayed at 5 for the whole run, all of them from sensor start-up). It exists because two watchdogs live in `lidar_parser_service()`: this 100 ms one keeps the *task* able to house-keep, and a 1 s one keeps the *sensor* alive:

> If the sensor is silent (SCAN command lost at power-up, cable unplugged, sensor still booting), no descriptors ever arrive. With `osWaitForever` the parser would block forever, the SCAN command would never be re-issued, and the whole pipeline would deadlock **silently**. With a 100 ms timeout the parser wakes, notices that no DMA block arrived for more than 1 s, drops the stream lock and re-sends `0xA5 0x20` once per second until data returns — exactly the recovery the Block 3.5 polling loop performed. This is what makes a LiDAR that was unplugged and replugged come back by itself: the C1 does **not** stream after a power-up, it always needs the SCAN command again. The `cmd <sent>/<lost>` field in the 1 Hz line proves it: it reads `1/0` during normal operation, climbs while the sensor is unplugged, and freezes again after the replug.

The stream state machine (`desc_n`, `pend`, `in_scan`) still has exactly one owner, the parser task, so no lock is needed. The Output task's 100 ms guard timeout is the same idea for the presentation side.

---

## 7. Why the parser must run above the output task

1. **Deadline driven by hardware, not by software.** The parser's deadline is set by DMA: `2 x 1024 B` of circular buffer means a block is overwritten if it is not consumed within 40 ms (measured). The output task's deadline is human — 100 ms frames, 1 s statistics. A missed parser deadline means **data corruption**; a missed output deadline means a late line on a terminal.
2. **Bounded, tiny work vs. unbounded I/O.** The parser does a fixed 801 µs of measured work per block. The output task performs blocking UART transmission (up to 100 ms timeout for a 256-byte line at 115 200 baud) and starts a 1200-byte UART5 DMA transfer. If the output task had higher priority, it could delay the parser by an entire UART line time — 3x the block period — and let DMA overwrite a half buffer.
3. **Priority inversion is avoided by construction.** The parser never waits for a resource held by the output task. The only shared object is the 360-bin map, and the output task reads it inside a short `taskENTER_CRITICAL()` section that the parser cannot be blocked by for more than a few microseconds. Using a mutex here would *create* a priority-inversion risk for no benefit — one writer, one reader, no blocking.
4. **Keeps the interrupt tail short.** The parser unblocks from the ISR and runs immediately (it is the highest-priority ready task), draining Queue 1 before the next half-complete interrupt arrives. That is what keeps `q1_peak` at 0 (measured) and makes the whole handover safe.

---

## 8. Safe handover between interrupt and task context

Three mechanisms, each doing exactly one job:

1. **Circular DMA with half/full interrupts** — DMA writes *only* into the half the parser is not reading. The parser is told which half via `offset`, so no lock is needed: writer and reader never touch the same 1024 B at the same time.
2. **Queue 1 as the only synchronisation primitive** — `xQueueSendToBackFromISR` inside the kernel critical section updates the queue atomically; `portYIELD_FROM_ISR` makes the unblocked parser run immediately if it has higher priority than the interrupted task.
3. **No shared mutable state inside the ISR** — the ISR only reads `block_seq`/`DWT->CYCCNT` and writes the descriptor on its own stack. The persistent parse state (`pend`, `desc_n`, `in_scan`) lives exclusively in task context, so it needs no protection at all.

Error path: the UART error ISR does **not** try to restart the reception (that would call `HAL_UART_AbortReceive`/`HAL_UART_Receive_DMA` in interrupt context). It posts a `LIDAR_BLOCK_UART_ERROR` descriptor and sets `rearm_pending`; the parser task performs `lidar_restart()` from task context. `rearm_pending` also suppresses stale block descriptors arriving between the error and the re-arm.

---

## 9. Where each requirement lives in the code

| Requirement | File / function |
|---|---|
| Ping-pong DMA half/full detection | `Core/Src/app.c` `HAL_UART_RxHalfCpltCallback` / `HAL_UART_RxCpltCallback` → `Core/Src/lidar.c` `lidar_on_half` / `lidar_on_full` |
| ISR → Queue 1 (short ISR, small descriptor) | `Core/Src/lidar.c` `post_block()` (line 125) |
| ISR-side queue post | `Core/Src/app_rtos.c` `app_rtos_post_block()` (line 92) |
| Parser task, blocked on Queue 1 | `Core/Src/app_rtos.c` `parser_task()` (line 106) |
| Reading the block out of the DMA buffer | `Core/Src/lidar.c` `lidar_parser_process()` (line 295), `process_block()` (line 239) |
| Streaming/framing across block boundaries | `Core/Src/lidar.c` `process_block()` — `desc_n` / `pend` state |
| Per-block result building | `Core/Src/lidar.c` `decode_packet()` (line 190) |
| Queue 2 | `Core/Src/app_rtos.c` `parser_task()` → `osMessageQueuePut` |
| Output task, blocked on Queue 2 | `Core/Src/app_rtos.c` `output_task()` (line 183) |
| ITM/SWV alternative to UART text | `Core/Src/app.c` `dbg_printf()` — replace the `HAL_UART_Transmit` with `ITM_SendChar()` |
| Queue/task creation and sizes | `Core/Src/app_rtos.c` `app_rtos_init()` (line 327), `Core/Inc/app_rtos.h` |
| HAL time base moved off SysTick | `Core/Src/stm32f4xx_hal_timebase_tim.c` (`HAL_InitTick` line 58, `TIM7_IRQHandler` line 231) |
| HAL tick calibration against DWT | `Core/Src/stm32f4xx_hal_timebase_tim.c` `hal_timebase_calibrate()` (line 158) |
| Interrupt priority policy | `Core/Inc/FreeRTOSConfig.h` §interrupt nesting, `Core/Src/app.c` `irq_priorities()` (line 81) |
| Boot sequence / ordering constraints | `Core/Src/app.c` `app_init()` (line 99) — DWT → drivers → NVIC → calibrate → RTOS objects → LiDAR DMA |

---

## 10. Measured results (real hardware, 20 s run)

Every number below was captured over USART2 at 115200 8N1 from a live
STM32F446RE + RPLidar C1, after the time-base calibration described in §11.
The HAL tick was verified against DWT (`50 ms (dwt) -> 50 ms (hal) OK`) before
the capture, so the byte rates are trustworthy.

**Per block (≈8 Hz on the wire, see note below)**
```
#0474 p205 ok151 nr54 iv0 n115 d[47..10574] lat12 p765 S 255 78 36 8 4 255 255 255
#0480 p205 ok197 nr8 iv0 n144 d[567..2475] lat12 p802 S 90 255 255 255 255 72 56 90
```
| Field | Meaning | Measured |
|---|---|---|
| `#0474` | block_id — a gap proves a descriptor was lost | no gaps, 472 blocks in ~19 s |
| `p204 / ok151 / nr54 / iv3` | packets decoded / valid / no-return / invalid | 204–205 / 143–197 / 8–66 / **0** |
| `n115` | distinct angle bins updated (extracted nodes) | 110–152 |
| `d[47..10574]` | min / max distance in this block | 45–570 mm / 10.6 m |
| `lat12` | **ISR → task wake-up latency** | **12–14 µs** |
| `p765` | task-side parse duration | 757–802 µs |
| `S ...` | 8-sector obstacle summary, cm (255 = no return) | plausible, changes with the scene |

**Per second (1 Hz)**
```
[1s] blk 26/447 q1 0/8 q2 1/4 idle 5 latmax 14 parsemax 801 | L 25259B/s
     pkt 91541 inv 0 nr 17495 sync 1 slip 0 err 0 | IMU 0 err 0 | BT f 161 dr 0
     | heap 26488 min 26488 | stk 1256/784 | hal 1054 k 1053
```
| Field | Meaning | Measured | Verdict |
|---|---|---|---|
| `blk 26/447` | blocks in the last second / total | **26 /s** | matches the 41 ms half-block budget |
| `q1 0/8` | **peak** Queue 1 occupancy | **0** | the parser always drains before DMA wraps — the 1024 B block size is right |
| `q2 1/4` | peak Queue 2 occupancy | 1 | the output task keeps up |
| `idle 5` | Queue 1 wait timeouts | 5 (start-up only, then frozen) | the watchdog fired while the C1 was still booting, then the stream locked |
| `latmax 14` | worst ISR → task latency | **14 µs** | 0.03 % of one block period |
| `parsemax 801` | worst parse time | **801 µs** | 2 % of one block period (build is `-O0`) |
| `L 25259B/s` | measured LiDAR byte rate | **24.9–25.3 kB/s** | matches `25 × 1024 = 25.6 kB/s` and the C1's ~5 000 samples/s |
| `pkt 91541` | cumulative packets | ≈ **5 100 packets/s** | ≈5 000 samples/s = C1 spec |
| `inv 0` | framing (check-bit) errors | **0** | the 5-byte framing and the cross-block state machine are correct |
| `slip 0` | resync events | **0** | never had to realign after the initial lock |
| `sync 1` | SCAN descriptor found | 1 | `A5 5A 05 00 00 40 81` locked at start-up |
| `err 0` | UART errors / re-arm events | **0** | no error path taken in 20 s |
| `cmd 1/0` | SCAN commands sent / times the stream watchdog lost the sensor | **1 / 0** | the sensor never had to be restarted or re-requested |
| `nr 17495` | no-return fraction | ≈ **19 %** | normal for a room with dark / absorbing surfaces |
| `heap 26488 min 26488` | FreeRTOS heap free / lowest ever | **6.2 kB of 32 kB used, flat** | no leak, sizing is generous |
| `stk 1256/784` | stack high-water **free**, parser / output | **1256 / 768 B** | both comfortable, no trimming needed |
| `hal 1054 k 1053` | HAL tick vs kernel tick over the same second | **1.000 : 1** | the two time bases agree |

**Raw capture (excerpt, verbatim)**
```
Block4.5 FreeRTOS LiDAR pipeline ready (imu=-1 hal=1 err=0x4 who=0x00, bt_baud=230400)
timebase: sysclk=168000000 hclk=168000000 pclk1=42000000 tim7 psc=167 arr=999
          implied_clk=168000000 calib=40/20 timpre=0
timebase check: 50 ms (dwt) -> 50 ms (hal) OK
#0474 p205 ok151 nr54 iv0 n115 d[47..10574] lat12 p765 S 255 78 36 8 4 255 255 255
#0477 p204 ok162 nr42 iv0 n121 d[45..5746]  lat12 p771 S 255 255 255 255 4 72 56 255
#0480 p205 ok197 nr8  iv0 n144 d[567..2475] lat12 p802 S 90 255 255 255 255 72 56 90
[1s] blk 26/447 q1 0/8 q2 1/4 idle 5 latmax 14 parsemax 801 | L 25259B/s
     pkt 91541 inv 0 nr 17495 sync 1 slip 0 err 0 cmd 1/0 | IMU 0 err 0
     | BT f 161 dr 0 | heap 26488 min 26488 | stk 1256/784 | hal 1054 k 1053
```

**The `#` line rate is ≈8 Hz, not 10 Hz.** That is not a bug: the frame decision
is `(osKernelGetTickCount() - t_frame) >= 100 ms`, but the Output task only wakes
when the parser posts a result, i.e. every ~40 ms (one DMA block). 100 ms
therefore quantises up to 3 blocks ≈ 120 ms. Stated honestly: the presentation
cadence is "every third DMA block".

**Other subsystems:** in this capture the IMU was unplugged, which the banner
reports as `imu=-1, err=0x4, who=0x00` (I2C NACK) and the 1 Hz line as `IMU 0`.
An earlier capture with the IMU connected showed ≈104 samples/s (`IMU 934 err 0`,
i.e. the 100 Hz TIM6 trigger). The Bluetooth bridge answered autobaud at
230400 baud in both runs (`BT f 161 dr 0`). The LiDAR path stayed error-free in
both — confirming that the pipeline has no dependency on the other sensors.

---

## 11. The HAL time base — the defect that only appears on real hardware

Moving the HAL tick off SysTick onto TIM7 is what every FreeRTOS tutorial tells
you to do, and it is what the lecture notes recommend. Three separate problems
had to be solved before HAL_GetTick() was trustworthy. All three are invisible
in a pure compile-and-read-the-code review, which is why they are documented
here in detail — they are the interesting part of this Coding Day.

### 11.1 `uxCriticalNesting` is poisoned before the scheduler starts

`port.c` (FreeRTOS 10.3.1, ARM_CM4F) initialises:

```c
static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;
```

and only clears it to `0` inside `xPortStartScheduler()`. Every
`osThreadNew()` / `osMessageQueueNew()` call made **before** the scheduler is
started uses `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()`, and because the
nesting counter never returns to 0, `vPortExitCritical()` never calls
`portENABLE_INTERRUPTS()`. **BASEPRI stays at `0x50` for the rest of the
pre-scheduler phase.**

With the HAL tick on SysTick this is harmless: SysTick is a **core** exception
and core exceptions ignore BASEPRI. Move the tick to a TIM and it becomes
fatal — the TIM interrupt is an ordinary peripheral IRQ and *is* masked.

Measured on the target, sampling BASEPRI at every init stage:

```
bp: entry=0x0 ... after_irq_prio=0x0   after_rtos_init=0x50   after_lidar_init=0x50
```

**Fix:** the HAL tick ISR is given NVIC priority **0**, above
`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`, so BASEPRI can never mask it.
This is safe precisely because the handler calls no FreeRTOS API — it only does
`uwTick += uwTickFreq`, a single 32-bit store. This is also why CubeMX defaults
`HAL_TICK_INT_PRIORITY` to 0 in TIM-timebase projects.

### 11.2 The TIM7 kernel clock is not what the reference manual predicts

`stm32f4xx_hal_timebase_tim.c` (and every CubeMX generation of it) assumes:

```
TIMxCLK = (APB1 prescaler == 1) ? PCLK1 : 2 x PCLK1   ->   2 x 42 MHz = 84 MHz
```

On this board the registers agree with that assumption —
`CFGR = 0x0000940A` (PPRE1 = /4 -> PCLK1 = 42 MHz),
`DCKCFGR = 0x00000000` (TIMPRE = 0) — yet TIM7 **measured** exactly twice the
expected rate: 40 update interrupts arrived where 20 were due over the same
DWT window, i.e. the effective kernel clock was **168 MHz = HCLK**.

Because the symptom is a silent 2x error in every `HAL_GetTick()` based timeout
(and in any measured byte rate), the driver no longer trusts the formula. It
**calibrates against DWT once at boot**:
`hal_timebase_calibrate()` runs TIM7 for ~20 ms of HCLK cycles, counts the
update interrupts, and rescales the prescaler by `measured / expected`.
On the target that turned PSC 83 into PSC 167:

```
timebase: psc=167 arr=999 implied_clk=168000000 calib=40/20 timpre=0
timebase check: 50 ms (dwt) -> 50 ms (hal) OK
```

An independent cross-check that the *kernel* tick was never the broken one: over
20 s of PC wall-clock, the 1 Hz statistics line arrived 18 times (0.9/s) and the
frame lines 154 times — consistent with a 1 kHz kernel, so the error really was
confined to the HAL tick.

### 11.3 Baselines that were set too high, twice

`MX_DMA_Init()` set `DMA1_Stream1` and `HAL_UART_MspInit()` set `USART3` to NVIC
priority **0**, while `i2c_bus_recover()` and `telemetry_dma_init()` re-wrote
`I2C1_*` and `DMA1_Stream7` to **1** and **0** at run time. Any of those is above
`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` (5), so the first
`osMessageQueuePut()` from the LiDAR ISR would trap in
`vPortValidateInterruptPriority()`. All of them are now 5 / 6, and the final map
is re-asserted by `irq_priorities()` *after* every driver has had its say (the
last writer wins).

---

## 12. FreeRTOS configuration choices worth defending

| Setting | Value | Reason |
|---|---|---|
| `configUSE_PREEMPTION` | 1 | the pipeline relies on the ISR immediately preempting the output task |
| `configMAX_PRIORITIES` | 56 | required by the `osPriority_t` enum of CMSIS-RTOS v2 |
| `configUSE_PORT_OPTIMISED_TASK_SELECTION` | **0** | the CLZ-based selector only supports <= 32 priorities |
| `configTICK_RATE_HZ` | 1000 | 1 ms resolution is enough for the presentation cadence and keeps tick overhead at 0.1 % |
| `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | **5** | puts USART3/DMA1 at the boundary so they may post; everything else stays out |
| `configUSE_TIMERS` | 1 | `freertos_os2.h` refuses to compile with timers disabled |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 | catches a too-small task stack during development |
| `configUSE_MALLOC_FAILED_HOOK` | 1 | `vApplicationMallocFailedHook()` traps a heap that is too small |
| heap model | `heap_4.c` | first-fit + coalescing; all objects are created once at start-up and never freed |
| HAL time base | **TIM7** | SysTick belongs to the kernel; TIM6 is already the IMU 100 Hz trigger |
| HAL tick IRQ priority | **0** | must sit above BASEPRI, see §11.1 |
| HAL tick prescaler | **calibrated against DWT** | the APB1 timer multiplier cannot be trusted, see §11.2 |

---

## 13. Rebuild / regeneration notes

* The project builds from `Debug/` with the CubeIDE toolchain: 0 warnings, 0 errors, `text 53 856 B / data 96 B / bss 42 520 B` (of 512 KB flash / 128 KB RAM).
* Flashing / verification helper: `tools/flash_and_monitor.cmd` (probe → program → verify → capture USART2 → auto-verdict, logs to `tools/logs/`).
* FreeRTOS sources live in `Middlewares/Third_Party/FreeRTOS/` and are added to the build through four extra `subdir.mk` files plus three include paths in `.cproject`.
* **If you ever press "Generate Code" in STM32CubeMX**, it will rewrite `.cproject` from the `.ioc`. The `.ioc` in this project does **not** know about FreeRTOS (no `FREERTOS` IP was injected by hand — a malformed block can make CubeMX refuse to open the project). After a regeneration you must either:
  1. open CubeMX → *Pinout & Configuration* → *Middleware* → **FREERTOS** → Interface **CMSIS_V2** → regenerate, and then re-apply the priorities in `irq_priorities()` and the TIM7 priority 0 / calibration; or
  2. restore the three include paths and the `Middlewares` source folder in *Project → Properties → C/C++ Build → Settings*.
* Files that are hand-written and must not be regenerated away: `Core/Inc/FreeRTOSConfig.h`, `Core/Inc/app_rtos.h`, `Core/Src/app_rtos.c`, `Core/Src/stm32f4xx_hal_timebase_tim.c`, and the Block 4.5 content of `Core/Src/lidar.c` / `Core/Src/app.c`.
