# Block 4.5 — FreeRTOS LiDAR 数据流水线

**Coding Day 第 4 周 分析报告**

| | |
|---|---|
| 团队 / 成员 | *待填* |
| 学号 | *待填* |
| 日期 | *待填* |
| 开发板 | STM32F446RE，168 MHz，128 KB SRAM |
| 操作系统 | FreeRTOS 10.3.1 + CMSIS-RTOS API v2（`cmsis_os2.c`） |
| 传感器 | RPLidar C1，SCAN 模式，USART3 @ 460 800 baud，8N1 |
| 源码目录 | `CodingDay_F446RE/`（编译零警告、零错误） |

本报告中的**所有数字都是在目标硬件上实测得到的**，不是估算。原始串口记录见
`tools/logs/`。

---

## 1. 要求的架构与本实现

```
         USART3 + DMA1_Stream1 (circular, 2048 B = 2 x 1024 B)
                          |
      half-complete IRQ -> lidar_on_half()    --+
      complete IRQ      -> lidar_on_full()    --+--> Queue 1 (8 x lidar_block_desc_t)
      UART error IRQ    -> lidar_on_uart_error()+            |
                                                             | osMessageQueueGet()
                                                             v
                                              LiDAR Parser Task (osPriorityAboveNormal)
                                                             |
                                           +-----------------+------------------+
                                           |                                    |
                             360-bin scan map (shared)              Queue 2 (4 x lidar_result_t)
                             ~8 Hz telemetry frame                              |
                                                                                v
                                                             Output/Debug Task (osPriorityNormal)
                                                                   |                    |
                                                          USART2 text 115200    UART5 binary 230400
```

> 图中刻意使用英文标识：等宽字体下中文是双宽字符，混排会把 ASCII 图的对齐打乱。

### 任务与对象清单

| 对象 | 类型 | 大小 / 优先级 | 作用 |
|---|---|---|---|
| `q_blocks`（Queue 1） | 消息队列 | 8 × 12 B | 中断 → 解析任务，只放块描述符 |
| `q_results`（Queue 2） | 消息队列 | 4 × 44 B | 解析任务 → 输出任务，放处理结果 |
| `parser_task` | 线程 | 1536 B 栈，`osPriorityAboveNormal`(32) | 读 DMA 缓冲区、解析、发布结果 |
| `output_task` | 线程 | 2048 B 栈，`osPriorityNormal`(24) | 格式化并发送，≈8 Hz 帧 / 1 Hz 统计 |
| 空闲任务 | 内核 | 512 B | 调度器自动创建 |
| 定时器服务任务 | 内核 | 1024 B | 调度器自动创建（`configUSE_TIMERS = 1`，`freertos_os2.h` 强制要求） |

FreeRTOS 堆：`configTOTAL_HEAP_SIZE = 32 KB`，使用 `heap_4.c`（首次适配 + 相邻块合并）。

实测占用：**6.2 KB / 32 KB**（`heap 26488 min 26488`，即从启动到现在最低剩余也是
26488 B，说明没有泄漏）。其中 2 个任务栈 3.5 KB、4 个 TCB 约 370 B、2 个队列对象
约 1.2 KB（含 0.28 KB 消息存储）。

---

## 2. 设计决策：为什么 DMA 块取 1024 B（2048 B 缓冲区对半切）

| 物理量 | 理论值 | **目标板实测** |
|---|---|---|
| 串口线路速率（460 800，8N1 = 10 bit/字节） | 46 080 B/s | — |
| RPLidar C1 有效负载（≈5 000 点/s × 5 B） | ≈ 25 000 B/s | **24.9 – 25.3 kB/s**（`L` 字段） |
| 填满一个 1024 B 半块的时间 | ≈ 41 ms | **40 ms**（25 块/s） |
| 按线路满速时填满半块的最坏情况 | ≈ 22 ms | — |
| 每半块的包数（1024 / 5） | ≈ 204 | **204 – 205**（`p` 字段） |
| 一个半块覆盖的旋转角度（10 Hz → 36°/100 ms） | 8° – 15° | ≈ 14° |
| 采样率 | ≈ 5 000 /s | **≈ 5 100 /s**（约 19 s 内 96 662 包） |

**论证。** 半块尺寸要同时满足两点：(a) 小到解析任务绝不会成为瓶颈；(b) 大到中断与
队列开销可以忽略。

- 1024 B 正好是 **204 个 LiDAR 包**，因此单块结果在统计上是有意义的（最小/最大距离、
  8 扇区障碍摘要、节点数），而不是只有寥寥几个点。
- 实测块周期 **40 ms**。解析最坏耗时实测 **801 µs**（`parsemax`，`-O0` 编译），只占块
  周期的 **2%**。Queue 1 深度 8，相当于 **320 ms** 余量，约为最坏解析耗时的 **400 倍**。
  实测 Queue 1 峰值占用为 **0**，说明解析任务始终有块可处理，从未落后。
- 1024 B / 204 包还意味着 5 字节帧结构不会频繁在块边界被切断（但一个包仍可能跨两个
  块 —— 见第 3 节）。实测 96 662 个包中 `inv = 0`、`slip = 0`，证明帧同步在每一次块
  边界都正确存活。

这个缓冲区大小直接回答了题目里「每块到底有多少个输出样本，没人告诉你 —— 自己去调查」这一问。

---

## 3. 设计决策：流的解析状态必须归解析任务所有，而不是归「块」所有

C1 的字节流是连续的，与 DMA 块边界完全异步，因此一个 5 字节样本包可能被切在两个块
中间。帧同步状态必须**跨块保持，并且只由一个任务拥有**：

* `desc_n` —— 在 7 字节 SCAN 响应头 `A5 5A 05 00 00 40 81` 中的匹配进度
* `pend[5] / pend_n` —— 正在累积的样本包
* `in_scan` —— 流已锁定

这套做法之所以安全，唯一原因就是**只有一个任务碰它**。如果让 ISR 解析，或者两个任务
同时解析同一条流，这些状态就必须加锁。把它放在单个任务里，正是快路径可以完全无锁的
原因。

单包校验只有一条规则：字节 0 的 bit0 是校验位、字节 1 的 bit0 是起始标志位，两者必须
相反。帧错误时丢掉一个字节重新对齐，并计一次 `slip`。

---

## 4. 设计决策：队列消息里放什么（以及为什么这么小）

### Queue 1 —— 块描述符（12 字节）

```c
typedef struct {
  uint16_t offset;      /* 该块在 lidar_dma_buf 中的起始偏移 */
  uint16_t length;      /* 块长度，字节                     */
  uint16_t block_id;    /* 单调递增的块序号                 */
  uint8_t  kind;        /* LIDAR_BLOCK_DATA | LIDAR_BLOCK_UART_ERROR */
  uint8_t  reserved;    /* 保持 12 字节 / 4 字节对齐        */
  uint32_t isr_cycles;  /* 中断里采样的 DWT->CYCCNT         */
} lidar_block_desc_t;
```

**为什么不把 LiDAR 原始字节拷进队列。** 每个块拷 1024 B，就意味着每秒 25 次要执行一次
1024 字节的内存到内存搬运 —— 这正是题目明令禁止的「在 ISR 里干重活」。本设计改用
**按偏移寻址**：`offset` + `length` 告诉解析任务去哪里读，数据就留在 DMA 已经放好的位置。
于是 Queue 1 每次只传 **12 字节而不是 1024 字节**，队列流量降低约 85 倍，队列本身的存储
从 8 KB 降到 96 字节。

* `offset` / `length` —— 必需项，用来唯一标识一个已完成的块。
* `block_id` —— 让解析任务和输出任务都能发现**丢失的块**（序号出现跳变），并且会打印在
  调试行里。这样「描述符被丢了」就是可见的，而不是静默的数据错乱。
* `kind` —— 同一个队列既传数据块也传「错误/重新武装」事件，于是解析任务只有一个阻塞
  点，不需要第二条同步路径。
* `isr_cycles` —— 中断里采样的 DWT 周期计数，用于实测**中断 → 任务唤醒延迟**（`lat`
  字段）。代价是 4 字节加一次寄存器读，换来的是把「延迟很小」这种口头说法变成一个数字。

### Queue 2 —— 处理结果（44 字节）

```c
typedef struct {
  uint16_t block_id, length;
  uint16_t packets, valid, no_return, invalid, nodes;
  uint16_t min_mm, max_mm;
  uint8_t  sector_min_cm[8];   /* 8 × 45° 障碍摘要，0xFF 表示无回波 */
  uint8_t  synced, syncs, slips;
  uint32_t parse_us, handover_us, tick;
} lidar_result_t;
```

这就是题目要求的「更有意义的结果」，而且把题目列举的选项全都包含在一个结构里：
**有效测量计数、提取到的节点数、角度-距离格图、最小距离、扇区障碍摘要、包与扫描统计**，
总共 44 字节。深度 4 足够，因为输出任务取走的速度快于解析任务投放的速度（实测峰值占用 1/4）。

---

## 5. 中断优先级设计（最容易翻车的地方）

规则：只有 NVIC 优先级**数值 ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`** 的中断
才允许调用 FreeRTOS API。本工程该值为 **5**，于是
`configMAX_SYSCALL_INTERRUPT_PRIORITY = 5 << 4 = 0x50`。

记住 Cortex-M 上**数值越大、逻辑优先级越低**。

| 中断 | 优先级 | 是否调 RTOS API | 理由 |
|---|---|---|---|
| `DMA1_Stream1`（LiDAR 接收） | **5** | **是** —— `osMessageQueuePut` | 临界值；投递块描述符 |
| `USART3`（LiDAR 接收/错误） | **5** | **是** | 同上 |
| `I2C1_EV` / `I2C1_ER`（IMU） | 6 | 否 | 超过阈值就不允许调 API，但 6 仍在内核屏蔽范围之外 |
| `UART5`、`DMA1_Stream7`（蓝牙遥测） | 6 | 否 | 同上 |
| `USART2`（调试控制台） | 7 | 否 | 同上 |
| `TIM6_DAC`（IMU 100 Hz 采样） | 7 | 否 | 同上 |
| `TIM7`（HAL 时基） | **0** | 否 | **故意放在阈值之上** —— 见第 11.1 节 |
| `SysTick`、`PendSV` | 15 | 内核 | 由 FreeRTOS 端口层设置 |

注意这个刻意的非对称：LiDAR 中断正好落在系统调用上限上，因为它们**确实**要往队列投递；
而 HAL 时基放在最高优先级，因为它绝不能被临界区屏蔽。两个取值是同一条规则的两个推论，
不是随手定的。

本工程中发现并修掉的三个具体坑：

1. `HAL_xxx_MspInit()`、`telemetry_dma_init()` 和 `i2c_bus_recover()` 各自都会写 NVIC
   优先级寄存器，而**后写者赢**。它们原本把优先级设成 0 或 1 —— 都在系统调用阈值之上。
   现在改为在 `app_init()` 末尾由 `irq_priorities()` 统一重新断言，同时把各驱动内部
   的值改成了 6。
2. CubeMX 生成的 `MX_DMA_Init()` 把 `DMA1_Stream1` 设成优先级 0。优先级 0 时，DMA 中断
   里第一次 `osMessageQueuePut()` 就会撞进 `vPortValidateInterruptPriority()` —— 这是
   FreeRTOS + STM32 最经典的死法。已在源头改正并就地写明原因。
3. 把 HAL 时基搬到 TIM 上之后，暴露了第 11.1 节的问题：泄漏的临界区抬高 BASEPRI 后，
   **外设中断会被屏蔽而内核异常不会**，于是放在「安全的最低优先级」上的 TIM 时基会静默停摆。

### 为什么 ISR 必须短——用数字说话

DMA 中断只做这些：两个计数器自增、读 `DWT->CYCCNT`、填一个 12 字节结构、
`xQueueSendToBackFromISR` + `portYIELD_FROM_ISR` —— 只有几十条指令，不到 1 微秒。

可测的端到端证据是 `lat` 字段：从中断里打的 DWT 时间戳，到解析任务真正开始干活之间的
时长，实测 **12–15 µs**。这个数已经包含了 ISR 主体、PendSV 上下文切换和任务派发，
所以 ISR 主体本身比它更小。

如果反过来让 ISR 解析 1024 字节，就等于把实测 **801 µs** 的工作量（`parsemax`）搬进
中断上下文，而且每秒 25 次。那会给所有更低优先级的中断额外加上约 800 µs 的不可抢占
延迟 —— IMU 的 I2C（100 Hz）和 UART5 遥测 DMA 都会开始丢事件，因为 800 µs 已经相当于
100 kHz 下整整一次 14 字节 I2C 传输的时间。把 ISR 压到「几十条指令」这个量级，才让
LiDAR 这条路径对其余固件几乎不可见。

---

## 6. 任务状态迁移（TA 一定会问）

| 任务 | 什么情况下 BLOCKED | 什么情况下变 READY | 一直 RUNNING 到 |
|---|---|---|---|
| LiDAR 解析任务 | `osMessageQueueGet(q_blocks, 100 ms)` —— Queue 1 空 | 中断投递描述符（队列原本为空 → 任务解除阻塞，`portYIELD_FROM_ISR` 挂起 PendSV） | 处理完该块、投递到 Queue 2，然后回到 `Get` |
| 输出 / 调试任务 | `osMessageQueueGet(q_results, 100 ms)` —— Queue 2 空 | 解析任务投递结果，**或者** 100 ms 超时到期 | 发完一帧/一条摘要，回到 `Get` |

解析任务一生中 **>99.5% 的时间是 BLOCKED**。它从不忙等、从不轮询标志位、从不调用
`HAL_Delay()`。这正是本设计与 Block 3.5 裸机版（在 `while(1)` 里轮询
`half_flag`/`full_flag`）可观测的区别。

### 为什么解析任务的队列等待要带超时，而不是 `osWaitForever`

这个块超时**不是**轮询机制 —— 正常运行时每 40 ms 就有一个块（实测），100 ms 超时根本不会
触发（1 Hz 行里的 `idle` 计数器全程恒为 5，且都发生在传感器启动阶段）。它存在的原因是
`lidar_parser_service()` 里有两个看门狗：100 ms 这个让**任务**能定期做维护，1 s 这个让
**传感器**活着：

> 如果传感器静默（上电时 SCAN 命令丢了、线松了、传感器还在启动），就永远不会有描述符
> 到来。用 `osWaitForever` 的话解析任务会永久阻塞，SCAN 命令再也不会重发，整条流水线
> 就**静默死锁**了。
>
> 用 100 ms 超时，解析任务会醒来，发现**已超过 1 s 没有 DMA 块到达**，于是丢掉流锁定，
> 并每秒重发一次 `0xA5 0x20` 直到数据恢复 —— 正好是 Block 3.5 轮询循环做过的恢复动作。
> 这也是「拔掉雷达再插回去能自己活过来」的原因：C1 上电后**不会**自己开始输出，它永远
> 需要再收到一次 SCAN 命令。1 Hz 行里的 `cmd <已发>/<丢流>` 字段就是证据 —— 正常运行时
> 恒为 `1/0`，拔掉期间会往上跳，插回后又停住。

流状态机（`desc_n`、`pend`、`in_scan`）仍然只有一个所有者，即解析任务，因此无需加锁。
输出任务的 100 ms 保护超时是同一思路用在展示侧。

---

## 7. 为什么解析任务优先级必须高于输出任务

1. **截止时间由硬件决定，而不是由软件决定。** 解析任务的截止时间由 DMA 决定：
   `2 × 1024 B` 的循环缓冲意味着一个块若不在实测 40 ms 内被消费，就会被覆盖。输出任务的
   截止时间是人眼决定的 —— 100 ms 一帧、1 s 一条统计。**错过解析截止时间等于数据损坏**；
   错过输出截止时间只是终端上晚一行字。
2. **有界的小工作量 vs. 无界的 I/O。** 解析任务每块只做固定 801 µs（实测）的内存运算。输出
   任务执行阻塞式串口发送（115 200 波特下一行最多 256 字节，超时上限 100 ms），并启动
   一次 1200 字节的 UART5 DMA 传输。如果输出任务优先级更高，它可以把解析任务推迟整整
   一个串口行的时间 —— 那是一个块周期的数倍 —— 从而让 DMA 覆盖掉一个半缓冲区。
3. **优先级反转被结构性地避免了。** 解析任务从不等待被输出任务持有的资源。二者唯一的
   共享对象是 360 格扫描图，输出任务在一段很短的 `taskENTER_CRITICAL()` 里读取它，解析
   任务最多被挡几微秒。在这里用互斥量只会**制造**优先级反转风险，而带不来任何好处 ——
   单写者、单读者、无阻塞。
4. **让中断尾部保持短。** 解析任务从中断解除阻塞后立即运行（它是最高优先级的就绪任务），
   在下一个半满中断到来之前就把 Queue 1 排空。这正是 `q1_peak` 实测为 0 的原因，也是
   整个交接安全的原因。

---

## 8. 中断上下文与任务上下文之间的安全交接

三个机制，各司其职：

1. **循环 DMA 加半满/全满中断** —— DMA 只写解析任务没在读的那一半。解析任务通过
   `offset` 被告知是哪一半，因此不需要锁：写者和读者永远不会同时碰同一个 1024 B。
2. **Queue 1 作为唯一的同步原语** —— 内核临界区内的 `xQueueSendToBackFromISR` 原子地
   更新队列；`portYIELD_FROM_ISR` 在解除阻塞的任务优先级高于被中断任务时让它立即运行。
3. **ISR 内部没有共享可变状态** —— 中断只读 `block_seq` 和 `DWT->CYCCNT`，并把描述符
   写在自己的栈上。持久化的解析状态（`pend`、`desc_n`、`in_scan`）完全活在任务上下文，
   因此根本不需要保护。

错误路径：UART 错误中断**不**尝试重启接收（那会在中断上下文里调用
`HAL_UART_AbortReceive`/`HAL_UART_Receive_DMA`）。它投递一个
`LIDAR_BLOCK_UART_ERROR` 描述符并置位 `rearm_pending`；由解析任务在任务上下文里执行
`lidar_restart()`。`rearm_pending` 同时会抑制错误发生到重新武装之间到来的过期块描述符。

---

## 9. 每项要求对应到哪段代码

| 要求 | 文件 / 函数 |
|---|---|
| DMA 半满/全满乒乓检测 | `Core/Src/app.c` `HAL_UART_RxHalfCpltCallback` / `HAL_UART_RxCpltCallback` → `Core/Src/lidar.c` `lidar_on_half` / `lidar_on_full` |
| 中断 → Queue 1（短 ISR、小描述符） | `Core/Src/lidar.c` `post_block()`（第 125 行） |
| 中断侧投递队列 | `Core/Src/app_rtos.c` `app_rtos_post_block()`（第 92 行） |
| 解析任务，阻塞在 Queue 1 | `Core/Src/app_rtos.c` `parser_task()`（第 106 行） |
| 从 DMA 缓冲区读块 | `Core/Src/lidar.c` `lidar_parser_process()`（第 295 行）、`process_block()`（第 239 行） |
| 跨块延续的流式帧同步 | `Core/Src/lidar.c` `process_block()` —— `desc_n` / `pend` 状态 |
| 逐块结果的构建 | `Core/Src/lidar.c` `decode_packet()`（第 190 行） |
| Queue 2 | `Core/Src/app_rtos.c` `parser_task()` → `osMessageQueuePut` |
| 输出任务，阻塞在 Queue 2 | `Core/Src/app_rtos.c` `output_task()`（第 183 行） |
| 用 ITM/SWV 替代串口文本输出 | `Core/Src/app.c` `dbg_printf()` —— 把 `HAL_UART_Transmit` 换成 `ITM_SendChar()` 即可 |
| 队列/任务的创建与尺寸 | `Core/Src/app_rtos.c` `app_rtos_init()`（第 327 行）、`Core/Inc/app_rtos.h` |
| HAL 时基从 SysTick 搬走 | `Core/Src/stm32f4xx_hal_timebase_tim.c`（`HAL_InitTick` 第 58 行、`TIM7_IRQHandler` 第 231 行） |
| 用 DWT 校准 HAL tick | `Core/Src/stm32f4xx_hal_timebase_tim.c` `hal_timebase_calibrate()`（第 158 行） |
| 中断优先级策略 | `Core/Inc/FreeRTOSConfig.h` 中断嵌套段、`Core/Src/app.c` `irq_priorities()`（第 81 行） |
| 启动顺序与约束 | `Core/Src/app.c` `app_init()`（第 99 行）—— DWT → 各驱动 → NVIC → 校准 → RTOS 对象 → LiDAR DMA |

---

## 10. 实测结果（真实硬件，约 20 s 运行）

以下所有数字都在一块真实的 STM32F446RE + RPLidar C1 上、经 USART2（115200 8N1）采集，
且是在第 11 节所述的时基校准之后。采集前先用 DWT 验证了 HAL tick
（`50 ms (dwt) -> 50 ms (hal) OK`），所以字节率可信。

**逐块行（线上实测 ≈8 Hz，原因见下文）**
```
#0474 p205 ok151 nr54 iv0 n115 d[47..10574] lat12 p765 S 255 78 36 8 4 255 255 255
#0477 p204 ok162 nr42 iv0 n121 d[45..5746]  lat12 p771 S 255 255 255 255 4 72 56 255
#0480 p205 ok197 nr8  iv0 n144 d[567..2475] lat12 p802 S 90 255 255 255 255 72 56 90
```
| 字段 | 含义 | 实测 |
|---|---|---|
| `#0474` | `block_id` —— 序号有跳变就说明丢了一个描述符 | 无跳变，约 19 s 内 472 块 |
| `p204 / ok151 / nr54 / iv3` | 解出的包数 / 有效 / 无回波 / 无效 | 204–205 / 143–197 / 8–66 / **0** |
| `n115` | 本块更新到的不同角度格数（提取到的节点数） | 110–152 |
| `d[47..10574]` | 本块内最小 / 最大距离 | 45–570 mm / 10.6 m |
| `lat12` | **中断 → 任务唤醒延迟** | **12–14 µs** |
| `p765` | 任务侧解析耗时 | 757–802 µs |
| `S ...` | 8 扇区障碍摘要，单位 cm（255 = 无回波） | 合理，且随场景变化 |

**每秒行（1 Hz）**
```
[1s] blk 26/447 q1 0/8 q2 1/4 idle 5 latmax 14 parsemax 801 | L 25259B/s
     pkt 91541 inv 0 nr 17495 sync 1 slip 0 err 0 cmd 1/0 | IMU 0 err 0
     | BT f 161 dr 0 | heap 26488 min 26488 | stk 1256/784 | hal 1054 k 1053
```
| 字段 | 含义 | 实测 | 判定 |
|---|---|---|---|
| `blk 26/447` | 最近一秒的块数 / 累计块数 | **26 /s** | 与 40 ms 半块预算吻合 |
| `q1 0/8` | Queue 1 **峰值**占用 | **0** | 解析任务总能在 DMA 卷回之前排空 —— 1024 B 块大小选对了 |
| `q2 1/4` | Queue 2 峰值占用 | 1 | 输出任务跟得上 |
| `idle 5` | Queue 1 等待超时次数 | 5（仅启动阶段，之后不再增长） | C1 还在启动时看门狗触发过，之后流就锁定了 |
| `latmax 14` | 最坏中断 → 任务延迟 | **14 µs** | 一个块周期的 0.03% |
| `parsemax 801` | 最坏解析耗时 | **801 µs** | 一个块周期的 2%（`-O0` 编译） |
| `L 25259B/s` | 实测 LiDAR 字节率 | **24.9–25.3 kB/s** | 与 `25 × 1024 = 25.6 kB/s` 及 C1 的 ~5 000 点/s 吻合 |
| `pkt 91541` | 累计包数 | ≈ **5 100 包/s** | ≈5 000 点/s = C1 规格 |
| `inv 0` | 帧（校验位）错误 | **0** | 5 字节帧与跨块状态机正确 |
| `slip 0` | 重新同步次数 | **0** | 初始锁定后再没需要重新对齐 |
| `sync 1` | 找到 SCAN 响应描述符 | 1 | 启动时锁定 `A5 5A 05 00 00 40 81` |
| `err 0` | UART 错误 / 重新武装次数 | **0** | 20 s 内从未走错误路径 |
| `cmd 1/0` | 已下发 SCAN 命令数 / 流看门狗判定丢流的次数 | **1 / 0** | 传感器全程没有被重启或重新请求过 |
| `nr 17495` | 无回波占比 | ≈ **19%** | 室内深色/吸光表面的正常值 |
| `heap 26488 min 26488` | FreeRTOS 堆剩余 / 历史最低 | **32 KB 里用 6.2 KB，且持平** | 无泄漏，容量充裕 |
| `stk 1256/784` | 栈高水位**剩余量**，解析 / 输出 | **1256 / 768 B** | 都充裕，无需再调 |
| `hal 1054 k 1053` | 同一秒内 HAL tick 与内核 tick 的增量 | **1.000 : 1** | 两个时基一致 |

**原始采集（逐字节摘录）**
```
Block4.5 FreeRTOS LiDAR pipeline ready (imu=-1 hal=1 err=0x4 who=0x00, bt_baud=230400)
timebase: sysclk=168000000 hclk=168000000 pclk1=42000000 tim7 psc=167 arr=999
          implied_clk=168000000 calib=40/20 timpre=0
timebase check: 50 ms (dwt) -> 50 ms (hal) OK
[1s] blk 26/447 q1 0/8 q2 1/4 idle 5 latmax 14 parsemax 801 | L 25259B/s
     pkt 91541 inv 0 nr 17495 sync 1 slip 0 err 0 cmd 1/0 | IMU 0 err 0
     | BT f 161 dr 0 | heap 26488 min 26488 | stk 1256/784 | hal 1054 k 1053
```

**`#` 行的速率是 ≈8 Hz 而不是 10 Hz。** 这不是 bug：帧判定条件是
`(osKernelGetTickCount() - t_frame) >= 100 ms`，但输出任务只在解析任务投递结果时才被唤醒，
即每约 40 ms（一个 DMA 块）。于是 100 ms 会被量化到 3 个块 ≈ 120 ms。诚实说法是：
展示节拍是「每第 3 个 DMA 块」。

**其他子系统：** 本次采集中 IMU 未插，开机横幅报 `imu=-1, err=0x4, who=0x00`（I2C NACK），
1 Hz 行显示 `IMU 0`。而更早一次 IMU 已连接的采集达到约 104 样本/s（`IMU 934 err 0`，
即 TIM6 的 100 Hz 触发），蓝牙桥在两次采集中都在 230400 波特下自动波特率成功。
两次采集里 LiDAR 路径都零错误 —— 说明本流水线不依赖其他传感器。

---

## 11. HAL 时基 —— 只有在真实硬件上才会出现的问题

把 HAL tick 从 SysTick 搬到 TIM7 是每篇 FreeRTOS 教程都会说的、也是课件推荐的做法规。
但要让它可信，必须解决三个互相独立的问题。这三个问题在「只编译、只读代码」的审查中
全部不可见 —— 所以这里值得详细记录，它们才是这次 Coding Day 真正有意思的部分。

### 11.1 调度器启动前 `uxCriticalNesting` 就被污染了

FreeRTOS 10.3.1（ARM_CM4F）的 `port.c` 里：

```c
static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;
```

而它只在 `xPortStartScheduler()` 内部才被清成 `0`。所有在调度器启动**之前**调用的
`osThreadNew()` / `osMessageQueueNew()` 都会用到 `taskENTER_CRITICAL()` /
`taskEXIT_CRITICAL()`，由于嵌套计数器永远回不到 0，`vPortExitCritical()` 就永远不会调用
`portENABLE_INTERRUPTS()`。**于是整个「调度器启动前」阶段，BASEPRI 一直卡在 `0x50`。**

HAL tick 在 SysTick 上时这是无害的：SysTick 是**内核**异常，而内核异常不受 BASEPRI 屏蔽。
一旦把 tick 搬到 TIM 上，这就致命了 —— TIM 中断是普通外设中断，**会被屏蔽**。

在目标板上逐个阶段采样 BASEPRI 的实测结果：

```
bp: entry=0x0 ... after_irq_prio=0x0   after_rtos_init=0x50   after_lidar_init=0x50
```

**修法：** HAL tick 的中断优先级设为 **0**，高于
`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`，这样 BASEPRI 永远屏蔽不到它。这之所以
安全，恰恰是因为该 ISR 不调用任何 FreeRTOS API —— 它只做 `uwTick += uwTickFreq`，
一次 32 位写入。这也正是 CubeMX 在 TIM 时基工程里把 `HAL_TICK_INT_PRIORITY` 默认为 0
的原因。

### 11.2 TIM7 的内核时钟和参考手册的推算不一致

`stm32f4xx_hal_timebase_tim.c`（以及每一次 CubeMX 生成的版本）都假设：

```
TIMxCLK = (APB1 预分频 == 1) ? PCLK1 : 2 × PCLK1   ->   2 × 42 MHz = 84 MHz
```

本板上的寄存器与该假设完全一致 —— `CFGR = 0x0000940A`（PPRE1 = /4 → PCLK1 = 42 MHz）、
`DCKCFGR = 0x00000000`（TIMPRE = 0）—— 但 TIM7 的**实测**速率恰好是预期的两倍：
同一个 DWT 窗口内到来了 40 次更新中断，而预期只有 20 次，也就是等效内核时钟是
**168 MHz = HCLK**。

因为这个症状表现为所有基于 `HAL_GetTick()` 的超时（以及任何测得的字节率）都静默地偏
2 倍，驱动不再信任公式，而是在启动时**用 DWT 校准一次**：`hal_timebase_calibrate()`
让 TIM7 跑约 20 ms 的 HCLK 周期，数更新中断的个数，再按 `实测 / 预期` 重新缩放预分频。
在目标板上，这把 PSC 83 变成了 PSC 167：

```
timebase: psc=167 arr=999 implied_clk=168000000 calib=40/20 timpre=0
timebase check: 50 ms (dwt) -> 50 ms (hal) OK
```

一个独立的交叉验证可以证明**内核** tick 从来不是坏掉的那一个：在 20 s 的 PC 墙钟时间内，
1 Hz 统计行到达 18 次（0.9 次/s），逐块行到达 154 次 —— 与 1 kHz 内核一致，所以误差
确实只限于 HAL tick。

### 11.3 有两次基线被设得太高

`MX_DMA_Init()` 把 `DMA1_Stream1`、`HAL_UART_MspInit()` 把 `USART3` 设成 NVIC 优先级
**0**；而 `i2c_bus_recover()` 和 `telemetry_dma_init()` 在运行时又把 `I2C1_*` 和
`DMA1_Stream7` 重写成 **1** 和 **0**。这些都在
`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`（5）之上，因此 LiDAR 中断里第一次
`osMessageQueuePut()` 就会撞进 `vPortValidateInterruptPriority()`。现在全部改为 5 / 6，
并且最终映射由 `irq_priorities()` 在所有驱动都表态之后统一重新断言（后写者赢）。

---

## 12. 值得辩护的 FreeRTOS 配置选择

| 配置项 | 取值 | 理由 |
|---|---|---|
| `configUSE_PREEMPTION` | 1 | 流水线依赖中断立即抢占输出任务 |
| `configMAX_PRIORITIES` | 56 | CMSIS-RTOS v2 的 `osPriority_t` 枚举需要 |
| `configUSE_PORT_OPTIMISED_TASK_SELECTION` | **0** | 基于 CLZ 的选择算法只支持 ≤ 32 个优先级 |
| `configTICK_RATE_HZ` | 1000 | 1 ms 分辨率对展示节拍足够，tick 开销占 0.1% |
| `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | **5** | 让 USART3/DMA1 正好落在边界上以允许投递；其余都排除在外 |
| `configUSE_TIMERS` | 1 | `freertos_os2.h` 在定时器关闭时直接编译报错 |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 | 开发阶段捕获过小的任务栈 |
| `configUSE_MALLOC_FAILED_HOOK` | 1 | `vApplicationMallocFailedHook()` 捕获过小的堆 |
| 堆模型 | `heap_4.c` | 首次适配 + 相邻合并；所有对象只在启动时创建一次、永不释放 |
| HAL 时基 | **TIM7** | SysTick 归内核；TIM6 已被 IMU 的 100 Hz 触发占用 |
| HAL tick 中断优先级 | **0** | 必须位于 BASEPRI 之上，见 11.1 |
| HAL tick 预分频 | **用 DWT 校准** | APB1 定时器倍频关系不可信，见 11.2 |

---

## 13. 重新编译 / 代码重生成注意事项

* 从 `Debug/` 用 CubeIDE 工具链编译：零警告、零错误，
  `text 54 796 B / data 96 B / bss 42 536 B`（512 KB Flash / 128 KB RAM）。
* 烧录与验证辅助脚本：`tools/flash_and_monitor.cmd`（探测 → 烧录 → 校验 → 抓 USART2 → 自动判读，日志落在 `tools/logs/`）。
* FreeRTOS 源码位于 `Middlewares/Third_Party/FreeRTOS/`，通过 4 个额外的 `subdir.mk` 加 `.cproject` 里 3 条 include 路径接入构建。
* **如果你在 STM32CubeMX 里按下 Generate Code**，它会依据 `.ioc` 重写 `.cproject`。本工程的 `.ioc` **并不知道 FreeRTOS 的存在**（没有手工注入 `FREERTOS` IP —— 一个格式错误的块会让 CubeMX 直接打不开工程）。重生成之后你必须二选一：
  1. 在 CubeMX 里 *Pinout & Configuration* → *Middleware* → **FREERTOS** → Interface 选 **CMSIS_V2** → 重新生成，然后重新应用 `irq_priorities()` 里的优先级以及 TIM7 的优先级 0 和校准；或者
  2. 在 *Project → Properties → C/C++ Build → Settings* 里补回那 3 条 include 路径和 `Middlewares` 源目录。
* 手写且不能被重生成覆盖的文件：`Core/Inc/FreeRTOSConfig.h`、`Core/Inc/app_rtos.h`、`Core/Src/app_rtos.c`、`Core/Src/stm32f4xx_hal_timebase_tim.c`，以及 `Core/Src/lidar.c` / `Core/Src/app.c` 中的 Block 4.5 内容。
