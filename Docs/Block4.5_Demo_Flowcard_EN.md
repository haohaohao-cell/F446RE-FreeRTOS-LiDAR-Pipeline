# Block 4.5 — One-Page Demo Card

## Remember one sentence

> **The demo is: show the TA a pipeline running, then prove it is built the FreeRTOS way.**

No line-by-line code reading. **3 actions + answering questions.** 5 minutes.

---

## Action 1 · Let it run (30 s)

**Do**: nothing — the terminal is already scrolling. Wave a hand in front of the sensor.

**Say**:
> "LiDAR → USART3 → DMA half-buffer. The ISR only posts a 12-byte descriptor into Queue 1. The parser task reads that block out of the DMA buffer and parses it, the result goes into Queue 2, and the output task only prints. 25 blocks per second."

**Point at**: `blk 25/s`, `q1 0/8`

---

## Action 2 · Make the sensor "disappear", then recover (60 s) ★ the key 60 seconds

**Do (method A, recommended - no cable touching)**: type `s` in the terminal window,
wait 5 s, then type `g`.

> Use the window opened by `tools\serial_console.cmd`; just type into it.
>
> **Alternative method B** (only if you want to act out a real broken cable): unplug
> the LiDAR's own 4/5-pin cable at the **LiDAR end**, by the plastic connector
> housing. Do **not** pull the ST-Link USB (COM8 and your flash port) and do **not**
> switch off the car's main power.

**Say**:
> "After `s` the block count drops to zero and `idle` climbs — but **the system does
> not hang**: both tasks are still alive and the statistics line keeps printing.
> After `g`, `cmd` jumps twice: that is the watchdog noticing 1 s without any DMA
> block and re-sending SCAN. Then `blk` comes back to 25 by itself. The C1 never
> streams after a power-up, it always needs the SCAN command again — that is exactly
> why this watchdog exists."
>
> "With a plain `osWaitForever` this would deadlock permanently. That is the
> difference between queue-driven and polling."

**Point at**: `blk 0` → `idle` climbing from 5 to 70+ → after `g`, `cmd` jumps twice →
within **~3 s** `blk 26` is back, `idle` freezes, `sync` goes 1 → 2

> This is more convincing than any amount of code. **Do it.**

---

## Action 3 · Point at 3 functions (60 s)

**Do**: switch to the editor, 10 seconds each. Do not read line by line.

**Say**:

| Point at | One sentence |
|---|---|
| `Core/Src/lidar.c` → `post_block()` | "All the ISR does: fill a 12-byte struct, post it. **No parsing.**" |
| `Core/Src/app_rtos.c` → `parser_task()` | "This task blocks on Queue 1 and, when awake, reads the block straight out of the DMA buffer using the offset/length it was given." |
| `Core/Src/app_rtos.c` → `output_task()` | "This one only formats and transmits. It never touches the data path." |

---

## Action 4 · Answer questions

The TA will ask the 8 questions from slide 6. See the roles table below.

**Closing sentence**:
> "DMA fills a half → the ISR posts a descriptor → Queue 1 wakes the parser → the parser reads that block directly from the DMA buffer → the result goes to Queue 2 → the output task prints it. Three layers, and the interrupt never does heavy work."

---

## Roles table (4 people)

The brief does **not** mandate roles. Splitting the 8 required questions is simply the cheapest way to prepare.

| Role | Owns these questions (slide 6) | Main file to read | Numbers to memorise | Opening line in the demo |
|---|---|---|---|---|
| **A Data path** | ① How does DMA fill the buffer half by half?<br>② How do half/complete identify a ready block?<br>③ Why must the ISR stay short? | `Core/Src/lidar.c`<br>HAL callbacks in `Core/Src/app.c` | 1024 B / 40 ms / 25 blocks per s<br>handover latency **12 µs** | "DMA circularly fills 2048 bytes, split in two halves; the ISR only announces which half is ready." |
| **B Queues** | ④ Why does Queue 1 carry only a small descriptor?<br>⑤ How do both tasks move between BLOCKED / READY / RUNNING? | the two structs in `Core/Inc/lidar.h`<br>`app_rtos.c: parser_task()` | **12 bytes vs 1024 bytes**<br>`q1` peak **0** | "Only offset and length travel through the queue; the bytes stay in the DMA buffer, so queue traffic drops ~85×." |
| **C Priority & memory** | ⑥ Why must the parser outrank the output task?<br>⑦ What FreeRTOS memory objects are created? | `Core/Src/app.c: irq_priorities()`<br>`Core/Inc/app_rtos.h` | parser 32 / output 24<br>heap **6.2 KB of 32 KB** | "The parser's deadline is the 40 ms DMA period; the output's deadline is a 100 ms frame for a human." |
| **D Architecture & driving** | ⑧ The three-layer separation<br>**runs the commands, unplugs the LiDAR, switches windows** | the whole thing + `Docs/Block4.5_Design_Report_EN.md` §1 | one sentence per layer | "Three layers, each with an owner; the interrupt layer never parses." |

**More than 5 people**: split D into "driving the demo" and "architecture summary".
**Only 3 people**: give question ⑦ (memory objects) to B.

### ⚠️ One important warning

The brief says *"Mark will be awarded to team **members** who demonstrate understanding"* —
**marks go to the person who can explain, not to the group.** So:

- The table above is **depth of ownership**, not "nobody else may answer".
- Everyone must be able to give a **one-line answer** to all 8 questions (see the Q&A cheat sheet, §3 of the demo script).
- If you get asked something outside your part: answer one line, then **hand it over** — "A owns that one."

---

## 5-minute timeline

| Time | What | Who |
|---|---|---|
| 0:00 – 0:30 | Action 1: wave a hand, describe the data flow | D |
| 0:30 – 1:30 | Action 2: unplug → watch `idle` → replug | D (A adds "why it does not deadlock") |
| 1:30 – 2:30 | Action 3: point at the 3 functions | A, B (one each) |
| 2:30 – 3:00 | Action 4: three-layer summary | D |
| 3:00 – 5:00 | Questions: 8 questions by role | A / B / C / D |

---

## 60-second pre-demo checklist

```
1. Car on blocks, powered
2. tools\serial_console.cmd running (COM8 115200, [1s] lines in cyan - blk/idle live here)
3. pc_app\run_viewer.cmd point-cloud window open (COM10 Bluetooth, the other link)
4. Editor with lidar.c and app_rtos.c open
5. Report .docx in another window
```

> **Do not mix up the two links.** The point-cloud window only shows
> `fps / frames / crc_err / B/s / pkt / inv / max us`.
> `blk / idle / q1 / q2 / latmax / parsemax / heap / stk` exist **only in the
> `[1s]` line on COM8**. Both windows can run at the same time.

**Memorise only these four numbers**: `1024 B / 40 ms / 25 blocks per s`, `12 bytes vs 1024 bytes`,
`q1 peak = 0`, `12 µs vs 801 µs`.
