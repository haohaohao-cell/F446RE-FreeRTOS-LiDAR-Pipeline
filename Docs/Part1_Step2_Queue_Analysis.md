# Block 4.5 Coding Day — Part 1, Step 2

## Timing and Queue Behaviour of a Fixed-Period Producer / Event-Driven Consumer Design

| | |
|---|---|
| Module | EBU6470 — Block 4, Coding Day, Part 1 (CMSIS-RTOS2 Learning Activity) |
| Task | Step 2 — Analyse a Design Problem: four Investigations |
| Target | STM32F446RE, CMSIS-RTOS2 (FreeRTOS) kernel, 1 ms tick |
| Source of the problem | Block4.5-CodingDay-Part1.pptx, slides 3 and 4 |
| Method | Formal discrete-event model, hand trace, and independent simulation cross-check |

> Scope: this document answers only the four Investigations of Step 2. It does not cover Step 1 (running the W4.x examples), which is reported separately.

---

## 1. The Design Under Analysis

### 1.1 Code as given

```c
osMessageQueueId_t DataQueueHandle;

DataQueueHandle = osMessageQueueNew(3, sizeof(SensorData_t), NULL);

void ProducerTask(void *argument)
{
    SensorData_t data;
    for (;;) {
        data = AcquireSensorData();
        osMessageQueuePut(DataQueueHandle, &data, 0, 0);
        tick += 20;
        osDelayUntil(tick);      // Produces a data item every 20 ms
    }
}

void ConsumerTask(void *argument)
{
    SensorData_t data;
    for (;;) {
        osMessageQueueGet(DataQueueHandle, &data, NULL, osWaitForever);
        ProcessData(data);       // execution time = 47 ms
    }
}
```

### 1.2 Modelling rules derived from the stated assumptions

The problem statement is prose; a table has to be built from it, so the prose must first be turned into unambiguous rules. Each rule below is a direct restatement of one stated assumption, and together they make the trace reproducible.

| # | Rule | Derived from |
|---|---|---|
| R1 | Samples are generated at t = 0, 20, 40, 60, … ms. The k-th sample, written S_k, is generated at t = 20k ms, so ten samples (S0…S9) fall in the window 0–180 ms inclusive. | "The first sample is generated at t=0ms, followed by periodic samples" |
| R2 | A sample is removed from the queue at the instant the Consumer starts processing it, and processing lasts exactly 47 ms. Processing therefore occupies [0, 47), [47, 94), [94, 141), [141, 188). | "Sample processing takes exactly the stated time" + "A sample being processed is no longer in the queue" |
| R3 | Queue occupancy counts only *waiting* samples. The sample currently being processed is not counted. | "Exclude the sample being processed from queue occupancy" |
| R4 | If the queue already holds 3 samples, `osMessageQueuePut()` returns immediately, the sample is destroyed, and the Producer's next sample still appears exactly 20 ms later. There is no back-pressure of any kind. | "returns immediately without storing the sample … the Producer continues at its specified period" |
| R5 | The Consumer never blocks while a sample is waiting: on completion it takes the oldest waiting sample at that same instant. If nothing is waiting, it blocks in `osMessageQueueGet`. | "Upon completion, the Consumer immediately retrieves the oldest waiting sample, if available" |

Rule R4 is the assumption that decides the whole answer. A queue that blocks the producer would convert a rate mismatch into a *timing* problem (the producer slows down and stops being periodic). A queue that drops converts it into a *data loss* problem, and the producer stays perfectly periodic. The trace below shows loss, not slowdown.

### 1.3 Notation

| Symbol | Meaning | Value in this problem |
|---|---|---|
| T_p | Producer period | 20 ms |
| T_c | Consumer processing time per sample | 47 ms |
| C | Queue capacity, in samples | 3 |
| f_p | Production rate | 50 samples/s |
| f_c | Maximum consumption rate | 21.28 samples/s |
| P(t) | Samples produced in [0, t] | floor(t / 20) + 1 |
| R(t) | Samples taken for processing in [0, t] | floor(t / 47) + 1 |
| W(t) | Samples waiting (queue occupancy) at time t | P(t) − R(t) |

W(t) = P(t) − R(t) holds at any instant at which neither a production nor a retrieval is in progress, because every produced sample is either waiting, being processed, or already processed, and R(t) counts exactly the ones taken out of the queue.

---

## 2. Investigation 1 — Task Timing and Queue Configuration

### 2.1 The quantities asked for

| Quantity | Symbol | Value | How it is obtained |
|---|---|---|---|
| Producer period | T_p | **20 ms** | `tick += 20; osDelayUntil(tick);` |
| Consumer processing time | T_c | **47 ms** | stated: `ProcessData()` takes 47 ms |
| Production rate | f_p | **50 samples/s** | 1000 ms / 20 ms = 50 |
| Maximum consumption rate | f_c | **21.28 samples/s** | 1000 ms / 47 ms = 21.2766 |
| Queue capacity | C | **3 samples** | `osMessageQueueNew(3, sizeof(SensorData_t), NULL)` |
| Capacity in bytes | — | 3 × sizeof(SensorData_t) | allocated once, from the RTOS heap, at creation |

The capacity of 3 means three *slots*, not three bytes: the second argument is the size of one item, so the queue can hold 3 items of `sizeof(SensorData_t)` bytes each. The `NULL` attributes argument selects the default (FIFO, no special flags). No memory is allocated at run time; the storage comes from the RTOS heap once, inside `osMessageQueueNew()`.

### 2.2 Rate calculation, step by step

Production rate:

```
f_p = 1000 ms / T_p = 1000 / 20 = 50.0 samples per second
```

Maximum consumption rate:

```
f_c = 1000 ms / T_c = 1000 / 47 = 21.2765957... ≈ 21.28 samples per second
```

The word *maximum* matters. The Consumer achieves 21.28 samples/s only when a sample is always waiting, because then it never blocks and the 47 ms processing periods run back-to-back with no gap. When the queue runs dry the Consumer blocks in `osMessageQueueGet`, so its long-run throughput drops below 21.28 samples/s — it can never exceed it, because the 47 ms of processing is irreducible.

Utilisation of the Consumer by the Producer:

```
load = T_c / T_p = 47 / 20 = 2.35 = 235 %
```

The Consumer is asked to do 2.35 times as much work as it can do. This single ratio already predicts that no finite queue can help; Investigation 4 formalises it.

### 2.3 How `osDelayUntil()` establishes the fixed period

`osDelayUntil(tick)` suspends the calling thread until the kernel tick counter reaches the absolute value `tick`. It is not "sleep for 20 ms"; it is "wake me when the counter equals this absolute value". Each iteration adds a constant 20 to the stored deadline, so the deadlines form the fixed sequence 20, 40, 60, … and the *period* is exactly 20 ms regardless of how long the body of the loop took.

Two consequences follow, and both are needed for the trace:

- **No accumulated drift.** If `AcquireSensorData()` and the queue put took 3 ms in one iteration and 0.1 ms in the next, `osDelayUntil()` would still return at 20, 40, 60, … The execution time is absorbed by the delay, not added to the period. With `osDelay(20)` instead, the period would be 23 ms and 20.1 ms respectively, and the error would accumulate without bound.
- **The period holds even when the put fails.** Under rule R4 the put returns immediately whether or not there was room, so the deadline arithmetic is unaffected by queue state. This is why the Producer in the trace keeps producing every 20 ms while three of its samples are destroyed.

One assumption is implicit in the fragment: `tick` must be initialised before the loop, normally as `tick = osKernelGetTickCount();`. With that initialisation the first sample is produced at once and the following deadlines are 20, 40, …; that matches the stated assumption that the first sample appears at t = 0 ms. The fragment as printed shows `tick` being incremented but never declared, so the initialisation belongs to the surrounding code.

### 2.4 How the timeout arguments establish the two scheduling modes

The two behaviours asked about come entirely from the *timeout* arguments, not from the task priorities.

**Producer — fixed-period.** The third and fourth arguments of

```c
osMessageQueuePut(DataQueueHandle, &data, 0, 0);
```

are `msg_prio = 0` and `timeout = 0`. A timeout of 0 means "do not wait": if the queue is full the call returns `osErrorResource` immediately instead of blocking. Combined with `osDelayUntil()`, this makes the Producer a **pure fixed-period task**: its execution pattern is decided by its own deadline, never by the Consumer. It is decoupled from the Consumer in both directions — it neither waits for the Consumer to make room, nor is delayed by it.

**Consumer — event-driven.** The last argument of

```c
osMessageQueueGet(DataQueueHandle, &data, NULL, osWaitForever);
```

is `osWaitForever`, so the Consumer blocks with no timeout until an item exists. Its activations are therefore caused by data arrival, not by a clock: this is the definition of event-driven. The 47 ms processing time then becomes its *service time*, and the interval between consecutive retrievals is 47 ms whenever data is available — the Consumer is a closed loop of "wait for event, then serve for 47 ms".

**The asymmetry is the design flaw.** Because the Producer has `timeout = 0`, the rate mismatch cannot express itself as the Producer slowing down. It can only express itself as samples being destroyed inside `osMessageQueuePut()`, silently, since the return value is discarded in the given code. A non-blocking producer feeding an event-driven consumer with a longer service time is a lossy system by construction; the queue only decides how long the loss is deferred.

---

## 3. Investigation 2 — Queue Behaviour from 0 to 180 ms

### 3.1 The main trace: one row per Producer event

Each row is a Producer event. The "retrieved since the previous event" column carries the sample taken for processing in the interval preceding this event, together with the instant it was taken. Queue contents list the waiting samples only; the sample in process (rule R3) is never listed.

| # | t (ms) | Sample produced | Queue contents after the event | Waiting | Retrieved since the previous event | Outcome |
|---|---|---|---|---|---|---|
| 1 | 0 | S0 | (empty) | 0 | 0 ms → S0 | Stored, then taken at once by the blocked Consumer |
| 2 | 20 | S1 | S1 | 1 | — | Stored |
| 3 | 40 | S2 | S1, S2 | 2 | — | Stored |
| 4 | 60 | S3 | S2, S3 | 2 | 47 ms → S1 | Stored; S1 was taken 13 ms before this event |
| 5 | 80 | S4 | S2, S3, S4 | 3 | — | Stored; **queue becomes full** |
| 6 | 100 | S5 | S3, S4, S5 | 3 | 94 ms → S2 | Stored; a slot was freed 6 ms before, then refilled |
| 7 | 120 | S6 | S3, S4, S5 | 3 | — | **REJECTED — first loss** |
| 8 | 140 | S7 | S3, S4, S5 | 3 | — | **REJECTED** |
| 9 | 160 | S8 | S4, S5, S8 | 3 | 141 ms → S3 | Stored; the slot freed at 141 ms was taken by S8, not by S7 |
| 10 | 180 | S9 | S4, S5, S8 | 3 | — | **REJECTED** |

Totals over the window: 10 produced, 4 taken for processing (S0, S1, S2, S3), 3 rejected (S6, S7, S9), 3 still waiting at t = 180 ms (S4, S5, S8).

### 3.2 The same data as an occupancy strip

This is the compact view of the mechanism: the queue reaches 3 at t = 80 ms and stays pinned there, and the only thing that changes after that is *which* sample is thrown away. A retrieval buys one slot, which the next producer event immediately consumes; the sample that was already destroyed is not rescued by it.

```
t (ms)      0   20  40  60  80  100 120 140 160 180
Producer    S0  S1  S2  S3  S4  S5  S6  S7  S8  S9
Waiting     0   1   2   2   3   3   3   3   3   3
Retrieved   S0  -   -   S1  -   S2  -   -   S3  -
Rejected    -   -   -   -   -   -   S6  S7  -   S9
```

`Waiting = 3` means the queue is full (C = 3). Note the trap at t = 140 ms: from t = 141 ms one slot is free, so a superficial reading suggests S7 could have been stored. It could not — the put at 140 ms happened before the slot was freed, one millisecond too early.

### 3.3 Chasing the first rejection with arithmetic

The row-by-row trace answers "what happened"; the rejection must also be *explained*, as the Investigation asks. At the instant of the producer event at t = 120 ms:

```
samples produced by then      P(120) = 120/20 + 1        = 7   (S0 … S6)
samples already taken         R(120) = floor(120/47) + 1  = 3   (S0, S1, S2)
samples therefore unretrieved 7 − 3                      = 4
queue capacity                                        C  = 3
```

Four samples need storage and only three slots exist, so exactly one is destroyed — the newest, S6, because the put for it is the operation that finds the queue full. In general the first rejection happens at the first producer instant at which

```
P(t) − R(t) > C        i.e.      4 > 3        i.e.      t = 120 ms
```

and the earlier events satisfy

```
t = 100 ms:  P = 6, R = 3, P − R = 3   (equal to C, still storable)
t =  80 ms:  P = 5, R = 2, P − R = 3   (equal to C, still storable)
```

so 120 ms is the earliest possible instant. Two details are worth stating because they are the usual errors:

- **The rejected sample is the newest, not the oldest.** The queue is FIFO with a non-blocking put, so a full queue rejects the incoming item. The oldest samples are the ones that eventually get processed — the system drops fresh data and keeps stale data, which is the opposite of what a control application wants.
- **A rejection is not "one sample late".** It is unrecoverable: the Producer has no copy, the queue never receives the item, and nothing in the system counts the event because the return value of `osMessageQueuePut()` is ignored in the given code. The loss is silent.

### 3.4 Consistency check

A trace that loses samples must still conserve them. Every one of the ten samples is in exactly one state at t = 180 ms:

```
produced    10
  = processed        4   (S0, S1, S2, S3)
  + waiting          3   (S4, S5, S8)
  + rejected         3   (S6, S7, S9)
  = 10                ✓
```

The identity `P(t) = processed + waiting + rejected` also gives a second, independent way to obtain the rejection count: 10 − 4 − 3 = 3.

---

## 4. Investigation 3 — Minimum Queue Capacity

### 4.1 What changes when rejection is removed

Investigation 3 keeps both task timings identical and asks what capacity would be needed if nothing were ever rejected. Removing the rejections changes the *trace*, not the *rates*: the Producer is unaffected by queue state (rule R4), so the production instants stay at 20 ms intervals, and the Consumer's service intervals stay at 47 ms. Only the queue contents differ, because samples that were destroyed are now retained.

### 4.2 Sample counts over the same interval

Over 0–180 ms inclusive:

```
samples produced     10        (t = 0, 20, …, 180)
samples retrieved     4        (at t = 0, 47, 94, 141; the next one is at 188 ms)
samples waiting       6        (10 − 4)
```

The retrieval count is confirmed by the service intervals: starting at 0 ms, the Consumer starts a new sample every 47 ms, so starts occur at 0, 47, 94, 141, 188 and only the first four fall inside the window. The retrieval at t = 141 ms completes at 188 ms, i.e. after the window, so the sample taken at 141 ms is *in process* at t = 180 ms and — by rule R3 — is not part of the occupancy.

### 4.3 Occupancy with no rejection

| # | t (ms) | Sample produced | Samples produced (cumulative) | Samples retrieved (cumulative) | Waiting = P − R | Peak |
|---|---|---|---|---|---|---|
| 1 | 0 | S0 | 1 | 1 | 0 | — |
| 2 | 20 | S1 | 2 | 1 | 1 | |
| 3 | 40 | S2 | 3 | 1 | 2 | |
| 4 | 60 | S3 | 4 | 2 | 2 | |
| 5 | 80 | S4 | 5 | 2 | 3 | |
| 6 | 100 | S5 | 6 | 3 | 3 | |
| 7 | 120 | S6 | 7 | 3 | 4 | |
| 8 | 140 | S7 | 8 | 3 | 5 | |
| 9 | 160 | S8 | 9 | 4 | 5 | |
| 10 | 180 | S9 | 10 | 4 | **6** | **new maximum** |

No rejection occurs in this table by construction, so the third row of the previous table (S6, S7, S9 destroyed) does not apply here; S6, S7 and S9 now stay in the queue.

### 4.4 The maximum is 6, attained at the end of the window

The waiting count is a sawtooth: it rises by one at every producer event (every 20 ms) and falls by one at every retrieval (every 47 ms), so its local maxima always occur immediately after a producer event and its global maximum within the window occurs at the last producer event, t = 180 ms:

```
W(180) = P(180) − R(180)
       = (180/20 + 1) − (floor(180/47) + 1)
       = (9 + 1) − (3 + 1)
       = 10 − 4
       = 6
```

Checking the neighbouring events confirms that 6 is a maximum and not merely the last value: W(160) = 9 − 5 = 4 before the put of S8, becoming 5 after it; W(180) = 5 before the put of S9, becoming 6 after it. The function is monotonically increasing over the window at the level of its local maxima (3 at 80 ms, 3 at 100 ms, 4 at 120 ms, 5 at 140 ms, 6 at 180 ms), so the maximum of the window is the value at its right-hand end.

**Answer: the minimum queue capacity that avoids any rejection in 0–180 ms is 6 samples.**

### 4.5 Why the answer is 6 and not 5 or 7

- **Not 5.** At t = 180 ms six samples must be waiting simultaneously (S0 and S1 and S2 and S3 have all left; S4, S5, S6, S7, S8, S9 have all arrived and none of them has been taken). With 5 slots the put of S9 at 180 ms would fail, which contradicts the assumption of no rejection.
- **Not 7.** With 6 slots no put in the window fails, because the occupancy never exceeds 6. Capacity is the maximum simultaneous occupancy, so 6 is sufficient; 7 would also work but is not the minimum.
- **Why 3 failed.** The given capacity of 3 is exactly half the required 6, which is why the first loss occurs part-way through the window, at 120 ms, rather than at its end.

### 4.6 This minimum is only valid for this window

The number 6 must be stated with its scope, because the same arithmetic applied to a longer window gives a larger answer. The occupancy grows because the arrival rate exceeds the service rate, so the minimum capacity is a function of the observation interval:

| Interval | Produced | Retrieved | Minimum capacity |
|---|---|---|---|
| 0–180 ms | 10 | 4 | 6 |
| 0–360 ms | 19 | 8 | 11 |
| 0–1000 ms | 51 | 22 | 29 |

`W(t) ≈ t × (1/20 − 1/47) = t × 0.02872` samples, so the required capacity grows without bound. The value 6 answers the question as asked — "over the same interval" — but it is a local answer, and Investigation 4 explains why no capacity is sufficient in the long run. Concretely: a queue of 6 merely postpones the first rejection from 120 ms to 220 ms.

---

## 5. Investigation 4 — Long-Term Sustainability

### 5.1 Comparing the rates

| Quantity | Value | Direction |
|---|---|---|
| Production rate f_p | 50.00 samples/s | arrives |
| Maximum consumption rate f_c | 21.28 samples/s | leaves |
| Net accumulation | **+28.72 samples/s** | queue grows |

The Producer delivers samples 2.35 times faster than the Consumer can dispose of them. The queue is therefore not a buffer in the steady state — it is a delay line that is permanently filling. No finite capacity can prevent rejection indefinitely, because the number of waiting samples increases without bound while the capacity is constant; the queue reaches its capacity in finite time whatever that capacity is, and from then on every producer event that arrives while the queue is full destroys a sample.

Capacity can only postpone the first rejection. It cannot prevent it.

### 5.2 Rate of backlog growth

The backlog growth rate is the difference of the two rates:

```
Δf = f_p − f_c = 50 − 21.2766 = 28.7234 samples per second
```

In per-period terms, which is the form that matches Table 2.1:

```
net growth per producer period = 1 − T_p / T_c
                               = 1 − 20/47
                               = 1 − 0.42553
                               = 0.5745 samples per 20 ms
```

So on average every producer event adds about 0.57 of a sample more than the Consumer removes, i.e. after roughly seven producer periods one sample is irrecoverably surplus.

Time for a queue of capacity C to fill, from empty, at the average rate (with Δf in samples/s the quotient is in seconds):

```
t_fill ≈ C / Δf
```

| C | t_fill (average-rate estimate) | First rejection (discrete trace) |
|---|---|---|
| 3 | 3 / 28.7234 = 0.1044 s ≈ 104 ms | 120 ms |
| 6 | 6 / 28.7234 = 0.2089 s ≈ 209 ms | 220 ms |
| 16 | 16 / 28.7234 = 0.5570 s ≈ 557 ms | 560 ms |

The 104.4 ms estimate and the 120 ms trace result differ by 15.6 ms, and the difference is instructive rather than an error: production is a staircase, not a continuous flow, so the backlog crosses the capacity threshold *between* two producer events and the loss is only physically inflicted at the next one. Rate arithmetic locates the crossing; only the discrete trace identifies which sample dies. That is precisely why Table 2.1 has to be traced sample by sample, and why a rate-only analysis of a rejection instant is at best a bound.

### 5.3 The maximum Consumer processing time that prevents sustained growth

The queue stops growing exactly when the Consumer's service rate is at least the Producer's arrival rate:

```
f_c ≥ f_p
1000 / T_c ≥ 1000 / T_p
T_c ≤ T_p
T_c ≤ 20 ms
```

**Answer: the Consumer's processing time must be no more than 20 ms.**

At exactly T_c = 20 ms the two rates are equal, so the backlog is *stable* rather than growing: whatever occupancy the queue has is preserved, and after any transient it settles. This satisfies "no sustained growth" in the strict sense, but it leaves zero margin — with rates exactly equal, any jitter, any scheduling delay, or any occasional longer execution immediately creates a backlog that is never removed. The engineering answer is therefore T_c < 20 ms, and in practice comfortably less: the design should place the Consumer's utilisation T_c / T_p well below 100 %, since the queue capacity only has to absorb bursts and jitter rather than a permanent deficit.

The general condition, valid for any producer/consumer pair of this shape, is:

```
utilisation  u = T_c / T_p
u < 1        queue is stable; capacity absorbs jitter and bursts
u ≥ 1        queue grows without bound; capacity only delays the first loss
```

Here u = 47/20 = 2.35, so the system is in the second regime by a wide margin.

### 5.4 Why a bigger queue is not a fix, and what is

Enlarging the queue converts an immediate problem into a delayed one, and makes the failure worse in one specific respect: the data that finally reaches the Consumer is *older*. With a full queue the Consumer is always working on samples that arrived up to `C × T_p` milliseconds ago, so the system's effective latency equals the queue occupancy times 20 ms. For a control loop this is a stability problem as much as a data-loss problem.

| Option | Does it remove the root cause? | Cost | Comment |
|---|---|---|---|
| Enlarge the queue | No | RAM only | Delays the first loss by C / 28.72 s; increases data age |
| Overwrite the oldest on full | No | Needs a policy in the code | Keeps the *newest* data instead of the oldest — better for control, still lossy |
| Reduce the sampling rate to ≤ 21.28 Hz | Yes | Loses sensor bandwidth | Producer period must be ≥ 47 ms; changes the measurement |
| Speed up `ProcessData()` to < 20 ms | Yes | Algorithmic work | The only real fix if every sample matters |
| Split processing across several Consumer tasks | Yes | More tasks, more RAM | Parallel service raises f_c; the total work per second must still be T_c × f_p |
| Block the Producer when the queue is full | Yes, but | Producer stops being periodic | Converts data loss into timing disturbance; only valid if the sampling rate may slip |
| Decimate in an intermediate stage | Yes | Extra stage | Keeps the fast producer but processes a reduced stream downstream |

For this design the mismatch is 2.35:1, so the defensible choices are to reduce the production rate or to make processing faster. Queue sizing and overwrite policies are damage limitation.

---

## 6. Conclusions

- **Investigation 1.** T_p = 20 ms (50 samples/s), T_c = 47 ms (maximum 21.28 samples/s), capacity 3 samples. `osDelayUntil()` gives an absolute, drift-free fixed period; `osMessageQueuePut()` with timeout 0 makes the Producer non-blocking and therefore insensitive to queue state; `osMessageQueueGet()` with `osWaitForever` makes the Consumer event-driven with no timeout. The mismatch surfaces as silent data loss, never as back-pressure.
- **Investigation 2.** First rejection at t = 120 ms, because seven samples have been produced and only three retrieved, so four require storage against three slots. Three samples are lost in 0–180 ms (S6 at 120 ms, S7 at 140 ms, S9 at 180 ms); the queue is full from t = 80 ms onwards and stays full. The rejected sample is the newest, and the loss is silent because the return value is discarded.
- **Investigation 3.** With no rejection, 10 samples are produced and 4 retrieved in 0–180 ms, so the waiting count reaches 6 at t = 180 ms. **Minimum capacity = 6 samples**, exactly double the given value of 3. This is a window-specific figure; the required capacity grows linearly in time.
- **Investigation 4.** Capacity cannot prevent rejection indefinitely: 50 samples/s arrive and at most 21.28 samples/s leave, a permanent backlog growth of **+28.72 samples/s** (0.5745 per producer period). Sustained growth is avoided only if the Consumer's processing time is **≤ 20 ms**, and in practice strictly less with margin.

---

## 7. How These Results Apply to Part 2

The toy problem is a 2.35:1 overload; the Part 2 firmware is a deliberately matched design. Making that contrast explicit is the useful outcome of this analysis, because the same three quantities — service time, period, capacity — decide both.

| Aspect | Part 1 toy problem | Part 2 implementation | Lesson carried over |
|---|---|---|---|
| Producer | 50 samples/s (20 ms) | LiDAR DMA half-buffer, ~25 blocks/s (40 ms) | Period from the hardware, not the code |
| Consumer service time | 47 ms | Block parse ~0.8 ms (worst case ~0.9 ms) | Must be measured, not assumed |
| Utilisation u = T_c / T_p | 235 % → unstable | ≈ 2 % → stable | u < 1 is the design requirement |
| Queue and capacity | 3 samples | `Q_Lidar` = 8 descriptors (12 B each) | Capacity absorbs bursts and jitter only |
| Behaviour when full | Silent loss, return value ignored | Drop and increment `q_lidar_drop`, printed at 1 Hz | Never discard without a counter |
| Back-pressure | None | `Q_Path` blocks with a timeout, because path points must not be lost | Choose the policy per queue, explicitly |
| Deadline | Consumer has no deadline, only a rate | 40 ms, set by the DMA overwrite interval | The deadline, not the rate, is what matters |

Three concrete links:

1. **Capacity is sized against a deadline, not against a rate.** The capacity of `Q_Lidar` (8) exists because the Producer is the DMA, which overwrites its 2048-byte buffer every 40 ms, while the Consumer is over 40 times faster than it needs to be. Eight slots ride out a scheduling hiccup of a few blocks; that is the correct use of a queue in a stable system — exactly the role that capacity *cannot* play in the toy problem, where it would only be masking a permanent 28.72 samples/s deficit.

2. **Utilisation below 1 generalises into the timing budget.** The inequality `u < 1`, which here is T_c ≤ T_p, becomes for ten concurrent tasks on a 1 ms tick: the sum of all 10 ms task worst-case execution times must stay well under 10 ms. The budget in the standards (all 10 ms tasks together under 2 ms, i.e. 20 % of the period) is the multi-task form of the same inequality, and it is what keeps the 10 ms control loop stable.

3. **The toy problem's worst property — silent loss — is the one the standards forbid.** `osMessageQueuePut()`'s return value is discarded in the given code, and the only symptom is data that never arrived. That is why every queue in Part 2 has a declared full-queue policy and an overflow counter (`q_lidar_drop`, `q_cmd_overwrite`), and why the observability line exists: in a lossy design, the loss is invisible unless it is counted.

---

## Appendix A — Complete Chronological Event Log

All 14 events of 0–180 ms in time order, for cross-checking against Table 2.1. A sample is placed on the queue and, if the Consumer is idle, taken at the same instant, which is why t = 0 ms carries two rows.

| t (ms) | Event | Sample | Queue contents after | Waiting | Consumer state after |
|---|---|---|---|---|---|
| 0 | Producer put | S0 | S0 | 1 | idle, blocked in get |
| 0 | Consumer take | S0 | (empty) | 0 | processing S0, ends 47 |
| 20 | Producer put | S1 | S1 | 1 | processing S0 |
| 40 | Producer put | S2 | S1, S2 | 2 | processing S0 |
| 47 | Consumer take | S1 | S2 | 1 | processing S1, ends 94 |
| 60 | Producer put | S3 | S2, S3 | 2 | processing S1 |
| 80 | Producer put | S4 | S2, S3, S4 | 3 | processing S1 |
| 94 | Consumer take | S2 | S3, S4 | 2 | processing S2, ends 141 |
| 100 | Producer put | S5 | S3, S4, S5 | 3 | processing S2 |
| 120 | Producer put | S6 | S3, S4, S5 | 3 | processing S2 |
| 140 | Producer put | S7 | S3, S4, S5 | 3 | processing S2 |
| 141 | Consumer take | S3 | S4, S5 | 2 | processing S3, ends 188 |
| 160 | Producer put | S8 | S4, S5, S8 | 3 | processing S3 |
| 180 | Producer put | S9 | S4, S5, S8 | 3 | processing S3 |

The three puts at 120, 140 and 180 ms leave the queue contents unchanged, which is the signature of a rejection: the event happened, the data did not.

## Appendix B — Verification Method

The two tables were produced twice, independently, and compared. This matters because a hand trace of a lossy queue is easy to get plausibly wrong — the 141 ms release of one slot, one millisecond after S7 was already destroyed, is exactly the kind of detail that a rate-based argument hides.

**Method 1 — hand trace.** Apply rules R1–R5 one event at a time, maintaining a FIFO list and two scalars (whether the Consumer is busy, and when it finishes). Each rejection is recorded with the reason `P − R > C`.

**Method 2 — simulation.** The model was re-implemented as a discrete-event simulation over the same rules and run to 180 ms. Both the acceptance case (capacity 3) and the no-rejection case were simulated, and the rejection case was also run with capacity 6 to confirm that the first rejection then moves to 220 ms, as predicted by `t_fill ≈ C / Δf`.

The two methods agree on every event: 10 produced, 4 retrieved, 3 rejected (S6, S7, S9), 3 waiting at t = 180 ms; and, with rejection disabled, 10 produced, 4 retrieved, maximum waiting 6 at t = 180 ms.

The core of the simulation, for reference:

```python
CAP, PROD, SERV, END = 3, 20, 47, 180
q, busy_until, prod_t, sid = [], -1, 0, 0
while prod_t <= END or (busy_until >= 0 and busy_until + SERV <= END):
    comp = busy_until + SERV if busy_until >= 0 else 10**9
    if comp <= prod_t:                      # Consumer completes, takes oldest
        if q:
            q.pop(0); busy_until = comp
        else:
            busy_until = -1
        continue
    if prod_t > END:
        break
    if len(q) < CAP:                        # Producer put with timeout 0
        q.append(sid)
        if busy_until < 0:                  # Consumer idle: takes it at once
            q.pop(0); busy_until = prod_t
    else:
        pass                                # rejected: sample destroyed, no blocking
    sid += 1
    prod_t += PROD
```

Reporting the check rather than only the answer is deliberate: the numbers in Tables 2.1 and 4.3 are counts of events in a lossy system, and a single off-by-one at 141 ms would change both the rejection count and the minimum capacity from 3/6 to 2/5.
