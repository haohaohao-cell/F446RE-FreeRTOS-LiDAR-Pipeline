# 编码与设计规范 —— Block 4.5 Part 2 / 循迹小车

> 适用：`CodingDay_F446RE` 固件的后续开发（Part 2 自主导航 + 最终成品循迹小车）。
> 规则分三级：**铁律**（违反即返工，甚至有变砖风险）、**必须**（评审会查）、**建议**。
> 与本文冲突时，以课程 PPT 的硬性要求优先；本文是它的工程化落地。
> 最后更新：2026-11-16。

---

## 0. 铁律（任何情况下不得违反）

### 0.1 永远保留烧录与调试通道

| 规则 | 原因 |
|---|---|
| **PA13/PA14（SWDIO/SWCLK）永不复用为 GPIO 或其它功能** | 一旦复用，SWD 失效，只能靠 BOOT0 救砖 |
| 不调用 `HAL_GPIO_DeInit` / `__HAL_AFIO_REMAP_SWJ_DISABLE` 等关闭 SWD 的 API | 同上 |
| **绝不写 Option Bytes、绝不启用 RDP（读保护）** | RDP Level 1 会让调试口失效，Level 2 直接**永久锁死** |
| 不在代码里调用 `HAL_FLASH_*`（除 Bootloader 项目） | 误擦会连程序一起没 |
| 不把 `BOOT0` 拉高后忘记复位回低 | 会进 System Memory，看起来像"程序丢了" |
| 每次改完引脚配置，**先烧一次确认还能连上**再继续加功能 | 早发现比晚发现好 |

**救砖预案（写在纸上贴在桌上）**：BOOT0=1 上电 → ST-Link 用 `mode=UR`（connect under reset）连接 →
`STM32_Programmer_CLI -c port=SWD mode=UR -e all` 全片擦除 → BOOT0=0 重新上电。

### 0.2 任务里禁止任何阻塞

| 禁止 | 替代 |
|---|---|
| `HAL_Delay()` | `osDelay()` / `osDelayUntil()` |
| `while(!flag)`、`while(HAL_XXX_GetFlag())` | 信号量 / 事件标志 / 队列 |
| 无超时的 I2C/SPI 事务（`HAL_I2C_...(&h, ..., HAL_MAX_DELAY)`） | 明确超时（见 §5.1） |
| 在任务里调用 `HAL_UART_Transmit` 发长字符串 | DMA + 环形缓冲（见 §5.4） |
| 在临界区里调用可能阻塞的 API | 临界区只做内存操作（§1.5） |

### 0.3 每个资源只有一个所有者（gatekeeper）

| 资源 | 唯一所有者 |
|---|---|
| USART3 / DMA1_Stream1（雷达） | LiDAR 任务（在 ISR 里只投描述符） |
| I2C1 总线 | 只有"拿到互斥量"的任务 |
| TIM4 / H 桥引脚 | Motor Control 任务 |
| TIM1/TIM8 编码器 | Pose Estimation 任务（读 CNT 后清零） |
| USART5 / DMA1_Stream7（蓝牙） | BT RX 任务（收）/ BT TX 任务（发），用队列解耦 |
| OLED 显存缓冲 | OLED 任务（别人只写一个请求结构） |

**任何跨所有者的数据交换都必须走队列 / 信号量 / 互斥量，不许用全局变量传数据。**

### 0.4 中断里只做"通知"，不做"处理"

- ISR 里**只允许**：清标志、读一个寄存器、`osMessageQueuePut(timeout=0)`、`osSemaphoreRelease()`、`portYIELD_FROM_ISR`
- ISR 执行时间目标 **< 5 µs**；超过就必须把工作挪到任务里
- 任何在 ISR 里调用 RTOS API 的中断，**NVIC 优先级必须 ≥ 5**（见 §4.3）

---

## 1. 任务与调度规范

### 1.1 调度方式必须二选一，不许"顺便"

| 类型 | 用什么 | 适用 |
|---|---|---|
| 固定周期 | `osDelayUntil()`（绝对时刻，不累积漂移） | 电机、PID、位姿、OLED、BT TX |
| 事件驱动 | 阻塞在 `osMessageQueueGet` / `osSemaphoreAcquire` | LiDAR、IMU、BT RX、路径生成 |

**禁止**用 `osDelay(fixed)` 做周期任务 —— 会累积漂移。正确写法：

```c
uint32_t tick = osKernelGetTickCount();
for (;;) {
    tick += 10U;                 /* 周期必须常量 */
    do_the_work();
    osDelayUntil(tick);          /* 如果 do_the_work 超时，osDelayUntil 立即返回：
                                    必须用 §1.3 的"超时计数"暴露出来，不许静默 */
}
```

### 1.2 任务表（最终版：8 个线程）

1 ms tick。优先级取 CMSIS-RTOS2 枚举，数字越大越优先。

| # | 线程 | 调度 | 周期/触发 | 优先级 | 栈 (B) | WCET 预算 |
|---|---|---|---|---|---|---|
| 1 | PID + Motor（合并） | 固定 | 10 ms | `osPriorityHigh`(40) | 1024 | < 700 µs |
| 2 | IMU Acquisition | 事件（TIM6 信号量） | 10 ms | `osPriorityAboveNormal`(37) | 512 | < 300 µs |
| 3 | Pose Estimation | 固定 | 10 ms | `osPriorityAboveNormal2`(36) | 768 | < 400 µs |
| 4 | LiDAR Processing | 事件（`Q_Lidar`，100 ms 守卫） | ~22 ms/块 | `osPriorityAboveNormal1`(34) | 1536 | < 900 µs |
| 5 | Path Generation | 事件（`Q_Cmd`） | 命令到达 | `osPriorityNormal7`(30) | 1024 | < 500 µs |
| 6 | BT RX | 事件（UART5 RX 信号量） | 字节到达 | `osPriorityNormal6`(28) | 768 | < 200 µs |
| 7 | BT TX | 固定 | 50 ms | `osPriorityNormal4`(22) | 1024 | < 2 ms |
| 8 | OLED Display | 固定（分页） | 50 ms | `osPriorityNormal3`(20) | 1024 | < 3 ms |
| — | 空闲 / 定时器守护 | 内核 | — | 0 / 55 | 512 / 1024 | — |

**10 ms 任务的 WCET 之和 = 700 + 300 + 400 = 1.4 ms**，低于 §1.3 的 2 ms 上限。

**优先级排序理由**：执行机构 > 传感 > 位姿 > 雷达 > 路径 > 蓝牙收 > 蓝牙发 > 显示。
雷达的截止时间是 40 ms（DMA 覆盖），比 10 ms 环宽松，所以排在控制环之后。

#### 1.2.1 PPT 参考表（10 行）→ 实现（8 线程）映射

| PPT 参考表行 | 实现 | 说明 |
|---|---|---|
| LiDAR Processing | 线程 4 | 一致 |
| IMU Acquisition | 线程 2 | 一致 |
| **Encoder Capture** | **不是线程** | TIM1/TIM8 **硬件 QEP 计数器**，计数在后台由外设完成；Pose 任务在 10 ms 时隙开头读一次 CNT 并算差分。对应 PPT 原文 "Encoder Capture / Hardware interrupt/capture / Counts processed every 10 ms" |
| Bluetooth RX | 线程 6 | 一致 |
| Pose Estimation | 线程 3 | 内含编码器采样 → 里程计 |
| Path Generation | 线程 5 | 一致 |
| **PID Path Tracking** | **线程 1（合并）** | 依据 PPT 原文 "Related 10 ms functions may share a coordinated task where appropriate" |
| **Motor Control** | **线程 1（合并）** | 与 PID 同一任务：setpoint 成为任务内局部变量，零延迟、零额外队列 |
| OLED Display | 线程 8 | 一致 |
| Bluetooth TX | 线程 7 | 一致 |

**队列仍然是三个**：`Q_Lidar` / `Q_Cmd` / `Q_Path`。
**不引入 `Q_Enc`**：编码器没有独立线程，Pose 直接读硬件计数器，不需要交接。
**不引入 `Q_Set`**：PID 与 Motor 合并后 setpoint 是任务内局部变量，不存在跨所有者传递。

> 答辩口径：被问到"编码器任务在哪" → "硬件 QEP 计数 + 在 Pose 任务的 10 ms 时隙内采样，不是线程"；
> 被问到"为什么 PID 和电机只有一个任务" → 引 PPT 那句 coordinated task。

#### 1.2.2 合并的代价与约束

- 合并后单任务 WCET 预算从 500 µs 提到 **700 µs**（PID 500 + 电机输出 200 的合并估计），仍远低于 10 ms。
- 代价是这一环的**最坏情况必须一起测**：PID 算得慢会直接推迟 PWM 更新，两者不再有优先级保护。
- 因此 `motor_update()` 必须在 10 ms 周期内一次算完、一次写 CCR，**禁止**在中间调用任何阻塞 API。

### 1.3 时间预算（必须实测）

- **所有 10 ms 任务的 WCET 之和 < 2 ms**（占 20%）。初始表合计 1.1 ms，还有余量。
- 每个周期任务必须统计 **抖动（jitter）** 和 **超时次数**，写进 1 Hz 状态行（§7）。
- `osDelayUntil` 返回后立刻读时间戳，若 `实际 - 期望 > 本次周期` → 计一次 overrun，**必须可见**。

### 1.4 栈大小

- 初始按 §1.2 给，跑 10 分钟后看 `uxTaskGetStackHighWaterMark()`
- **剩余量必须 > 30%**，否则只能加大，不许"刚好够用"
- 新增任务的栈不得小于 **512 B**
- 总堆 `configTOTAL_HEAP_SIZE` 目前 32 KB，实测用 6.2 KB；加到 48 KB 也没问题（128 KB RAM）

### 1.5 临界区

- `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` **只用于保护几条指令**的内存操作（例如 360 格扫描图的快照拷贝）
- 临界区内**禁止**：任何 `osXxx()` 调用、任何 HAL 阻塞调用、循环超过 ~50 次
- 需要保护一段"较长"逻辑时用**互斥量**，不要拉长临界区

---

## 2. 同步与通信规范

### 2.1 队列消息必须是定长 POD

| 规则 | 说明 |
|---|---|
| 只传定长结构体，**≤ 16 字节**（`Q_Lidar` 12 B，`Q_Cmd` ≤ 8 B，`Q_Path` 建议 8 B） | 拷贝便宜，避免内存管理 |
| **禁止传指针**指向栈上的数据 | 接收方可能拿到已销毁的数据 |
| 需要传大块数据时**传描述符**（offset + length + id），数据留在原地 | 这正是 `Q_Lidar` 的做法 |
| 结构体用 `uint16_t`/`uint32_t` 显式定宽，注意对齐（加 `reserved` 填充） | 避免编译器差异 |

### 2.2 队列容量与溢出策略（必须显式，不许静默）

| 队列 | 容量 | 满时策略 | 计数器 |
|---|---|---|---|
| `Q_Lidar` | 8 | 丢弃并 `q_lidar_drop++`（雷达块会被 DMA 覆盖，丢是必然的） | 1 Hz 打印 |
| `Q_Cmd` | 4 | 覆盖最旧（新命令优先），`q_cmd_overwrite++` | 1 Hz 打印 |
| `Q_Path` | 16 | 生产方阻塞等待（路径点不能丢），带超时；超时则重新规划 | 1 Hz 打印 |

**禁止**：`if (osMessageQueuePut(...) != osOK) { /* 什么都不做 */ }`

### 2.3 ISR → 任务的唯一合法写法

```c
/* ISR 里 */
BaseType_t woken = pdFALSE;                 /* CMSIS-RTOS2 内部用 */
osMessageQueuePut(q, &desc, 0U, 0U);        /* timeout 必须 0 */
/* 或者 */
osSemaphoreRelease(sem);                    /* 计数信号量，ISR 安全 */
```

> CMSIS-RTOS2 **没有** `FromISR` 变体；ST 的 `cmsis_os2.c` 内部用 `__get_IPSR()` 判断上下文
> 并自动切到 `xQueueSendToBackFromISR` + `portYIELD_FROM_ISR`。
> **代价**：中断里调用时 `timeout` 必须为 0，否则返回 `osErrorParameter`。

### 2.4 互斥量

- **只用于 I2C1 总线**（MPU-6500 + OLED 共用）
- 创建时必须带 `osMutexPrioInherit`（评分点！）
- **必须检查返回值**，拿不到锁不许继续操作硬件
- 持有时间目标 **< 3 ms**（见 §5.1 的 OLED 分页策略）
- **禁止**嵌套：一个任务不得同时持有两个互斥量（避免死锁）
- 释放必须在**所有出口**（包括错误返回路径）；推荐单一出口写法

```c
osStatus_t st = osMutexAcquire(i2c_mutex, I2C_LOCK_TIMEOUT_MS);
if (st != osOK) { i2c_lock_fail++; return -1; }   /* 不许继续 */
do_i2c_work();
osMutexRelease(i2c_mutex);                        /* 成功失败都要释放 */
```

### 2.5 信号量的用途限定

- 计数信号量：**TIM6 的 10 ms 事件 → IMU 任务**（Part 2 硬性要求）
- 二值信号量：可以用于"一次性事件"（如某个 ISR 请求一次动作）
- **禁止**用信号量传递数据（数据用队列）
- 初始化计数与最大值要写清楚：`osSemaphoreNew(1, 0, ...)` 表示"最大 1、初始 0"
- 必须能实测出**采样率**（1 Hz 行打印 IMU 采样数/秒，应为 ~100）

---

## 3. 硬件与资源分配规范

### 3.1 引脚保留表（**任何新功能不得占用**）

| 引脚 | 用途 | 状态 |
|---|---|---|
| PA13 / PA14 | SWD 调试/烧录 | **永久保留** |
| PA2 / PA3 | USART2 调试串口 @115200 | 保留 |
| PC10 / PC11 | USART3 雷达 @460800 | 保留 |
| PC12 / PD2 | UART5 蓝牙 @230400 | 保留 |
| PB8 / PB9 | I2C1（MPU-6500 + 将来 OLED） | 保留 |
| PB6 / PB7 | 当前 `motors_safe()` 拉低的电机方向脚 | **待重定义**（见 §3.3） |

新增外设（编码器、PWM、OLED、按键、电位器）的引脚**必须先填进本表再写代码**。

### 3.2 定时器分配表（**不得重复占用**）

| 定时器 | 用途 | 备注 |
|---|---|---|
| TIM1 | 右轮编码器（x4 解码） | 来自 `lessonProgram/Encoder.c` |
| TIM8 | 左轮编码器（x4 解码） | 同上，1320 counts/rev |
| TIM3 | 输入捕获（如需测周期） | 可选 |
| TIM4 | 电机 PWM（CH1/CH2，Period 799） | 来自 `lessonProgram/Motor.c` |
| **TIM6** | **10 ms 系统节拍**（Part 2 要求）→ 信号量驱动 IMU | 现在跑 IMU 100 Hz，**要改成 100 Hz tick 只发信号量** |
| **TIM7** | HAL 时基 1 kHz | **不许动**（见交接文档 §五） |
| TIM2 | 空闲 | 可作微秒计时 |

### 3.3 电机层

- 现有 `motors_safe()` 只把 PB6/PB7 拉低；真正驱动要用 `lessonProgram` 的
  `Board_SetMotorPwm()` + `Motor.c` 的 PID
- **上电顺序**：GPIO 先设成"刹车/滑行"安全态 → 再启动 PWM → 最后允许 PID 输出
- **失能路径**：任何故障（I2C 失败、队列满、看门狗预警）都要能在一个周期内把 PWM 置 0
- 编码器计数读取后立刻清零，避免 16 位溢出（1320 counts/rev，10 ms 内不会溢出，但要写清假设）

### 3.4 I2C1 共享（互斥量的存在理由）

- 当前 100 kHz。**建议提到 400 kHz**：OLED 全屏刷新 1024 B 在 100 kHz 下约 **12 ms**，
  在 50 ms 周期里占 24%，且期间一直持锁，会拖累 IMU。
- 若维持 100 kHz，则 **OLED 必须分页刷新**：每 50 ms 只刷 1 页（128 B ≈ 1.5 ms）。
- MPU-6500 与 OLED（SSD1306，7 位地址 0x3C）都支持 400 kHz。
- **`lessonProgram/Oled.c` 现在用的是 GPIO 软 I2C（PB0/PA4），不是 I2C1** —— 必须改接到
  I2C1 上，否则"共享总线互斥量"这个评分点不成立。

---

## 4. 中断与优先级规范

### 4.1 三条硬规则

1. 调用 RTOS API 的中断，**NVIC 优先级数值 ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`（=5）**
2. **HAL 时基（TIM7）优先级必须为 0**（原因见交接文档坑 1）
3. 优先级表必须在 `app_init()` **最后**统一断言一次（因为 MspInit、各驱动的 init 都会自己写寄存器，**后写者赢**）

### 4.2 优先级表（当前值 + Part 2 需要的修改）

| IRQ | 现在 | Part 2 需要 | 理由 |
|---|---|---|---|
| DMA1_Stream1（雷达） | 5 | 5 | 调 `osMessageQueuePut` |
| USART3（雷达错误） | 5 | 5 | 投递重启描述符 |
| **TIM6** | 7 | **5** | 要调 `osSemaphoreRelease` |
| **UART5（蓝牙收）** | 6 | **5** | 要往 `Q_Cmd` 投递 |
| I2C1_EV / I2C1_ER | 6 | 6（若在回调里释放信号量则改 5） | 建议：异步事务用信号量通知完成，则改 5 |
| DMA1_Stream7（蓝牙发） | 6 | 6 | 不调 RTOS API |
| USART2 | 7 | 7 | 只做轮询读 |
| **TIM7（HAL tick）** | 0 | **0** | 不许改 |
| SysTick / PendSV | 15 | 15 | 内核 |

### 4.3 优先级坑的检查方法

若出现"进 `vPortValidateInterruptPriority()` 死循环"，说明某个**在 5 以下**的中断调了 RTOS API。
用调试器暂停，看 `SCB->ICSR` 的 VECTACTIVE，再对照本表。

### 4.4 定时器时钟必须开机实测，禁止写死预分频

本板的 APB1 定时器时钟与手册不符（TIM7 实测为标称的 2 倍，见交接文档坑 2）。
所以**任何对时间精度敏感的定时器都不许按手册算完就写死**，一律走：

```c
int hal_tim_calibrate(TIM_HandleTypeDef *htim, uint32_t nominal_hz,
                      tim_cal_result_t *out);
```

做法（`Core/Src/tim_cal.c`）：临时把 PSC=0、ARR=0xFFFF，用 DWT 量约 125 µs 内的 CNT 增量，
反推 TIMx 内核时钟；再把预分频修正到目标频率。**不需要中断**，所以对没有更新中断的 PWM 定时器也适用。

| 规定 | 内容 |
|---|---|
| 何时调用 | `osKernelStart()` **之前**，且 TIM6 必须在 `imu_start()` 之前 |
| 调谁 | TIM6（100 Hz，10 ms 节拍）+ TIM4（20 kHz PWM）各一次；TIM2 与 TIM4 同属 APB1，直接继承 TIM4 算出的预分频（`motor_sync_pwm_clock()`） |
| 容差 | 推导出的更新频率与标称偏差 **> 5%** 才改预分频；≤ 5% 保留原值，避免无谓扰动 |
| 必须可见 | 启动横幅打印 **标称 / 实测 / 修正前 / 修正后** 四个值，不许静默修正 |
| 前置条件 | DWT 已使能（`dwt_init()`）；PWM 定时器的 CCR 必须为 0（测量期间 ARR 会临时变大） |

**禁止**：在 `.c` 里直接写 `PSC = 8399` 这类"按手册算出来"的常数而不做校准；
也**禁止**因为定时器不准就去改 `BOARD_TIM_CLK_HZ` 了事——先量，再按量出来的值算。

---

## 5. 保护手段（不允许"裸奔"）

### 5.1 每个 I2C 事务必须有界

| 项 | 规定 |
|---|---|
| 超时 | 单次事务 ≤ 10 ms；用 `HAL_I2C_Mem_Read/Write(&h, ..., 10)` 显式给 |
| 重试 | ≤ 2 次，仍失败则计 `i2c_err++` 并**释放互斥量**后返回 |
| 恢复 | 连续失败 3 次 → 调用已有的 `i2c_bus_recover()`（时钟脉冲解锁 + 外设复位） |
| 异步 | 若用 IT/DMA 事务，必须有完成通知（信号量/回调），**不许只发不等** |

### 5.2 串口错误必须能自愈

雷达、蓝牙都要：错误回调 → 只置标志/投事件 → 任务上下文里 `AbortReceive` + 重新 `Receive_DMA`。
现有 `lidar.c` 的 `rearm_pending` + `lidar_restart()` 就是这个模式，**蓝牙收按同样模式写**。

### 5.3 看门狗

- 用 **IWDG**，超时约 **1 s**（LSI 32 kHz，预分频 32 → 1 kHz，重装 1000）
- **只有一个地方喂狗**，且喂狗条件必须是"所有关键任务都健康"：

```c
/* 每个关键任务每个周期置位自己的 bit */
health_tasks |= HEALTH_MOTOR;          /* 在 Motor 任务里 */
...
/* 监控任务（可放 OLED 任务里，20 Hz） */
if (health_tasks == HEALTH_ALL) { health_tasks = 0; HAL_IWDG_Refresh(&hiwdg); }
```

- **禁止**在 ISR 里喂狗（会让"任务卡死但中断还活着"这种情况永远发现不了）
- 演示期间如果调试中断点，先把 IWDG 关掉（`IWDG` 一旦启动无法关闭 → 用**独立的编译开关**，默认关，验收前打开）

### 5.4 调试输出必须非阻塞

- 现有 `dbg_printf()` 是**阻塞**的 `HAL_UART_Transmit`（超时 100 ms），一行 200 字符在 115200 下约 **17 ms** ——
  在 10 ms 周期系统里这是灾难
- 规定：
  - 优先改成 **DMA + 环形缓冲**（发不出去就丢，计 `log_drop++`）
  - 退而求其次：只在**最低优先级**任务里打印，单行 **≤ 80 字符**，且每 1 s 最多 1 行
- **禁止**在 ISR、临界区、持锁期间打印

### 5.5 可观测的健康计数器（1 Hz 打印，见 §7）

每个子系统的失败都要有计数器：`i2c_err`、`q_lidar_drop`、`q_cmd_overwrite`、`uart_err`、`overrun`（周期任务超时）、`imu_skip`、`log_drop`。

---

## 6. 编码风格

| 项 | 规定 |
|---|---|
| 语言 | C11（`-std=gnu11`），`-Wall` 必须零警告 |
| 类型 | 定宽类型（`uint8_t`/`uint16_t`/`uint32_t`/`int16_t`），不用 `int`/`long` 表示硬件量 |
| 命名 | 模块前缀：`lidar_*`、`motor_*`、`encoder_*`、`pose_*`、`path_*`、`pid_*`、`oled_*`、`bt_*`、`imu_*` |
| 文件 | 一个模块 = `.c` + `.h`；`.h` 只暴露公共 API 和类型，内部状态用 `static` |
| 返回 | 失败返回 `int`（0 成功 / 负值错误码）或 `osStatus_t`，**不许静默忽略** |
| 头文件包含 | `.h` 自带 `#ifndef` 保护；`.c` 先包含自己的 `.h` |
| 魔法数字 | 必须 `#define` 或 `enum`，并写单位（`_MS`、`_US`、`_MM`、`_DPS`） |
| 动态内存 | **运行时禁止** `malloc`/`osMemoryPoolNew`；所有对象在 init 阶段创建 |
| 大数组 | 必须 `static`（不能放任务栈） |
| 结构体 | 跨任务传递的结构体必须是 POD、定长、显式对齐 |
| 注释 | 解释"为什么"，不解释"是什么"；每个非显然的时序假设都要注明 |

---

## 7. 可观测性（评分点，不是可选项）

PPT 明确要求"通过实测运行时行为证明"，所以必须有稳定的观测出口。

### 7.1 1 Hz 状态块（实际实现格式，代码以 `app_rtos.c` 为准）

每秒输出 **7 行**，前缀固定为 `[1s]`，便于脚本/报告直接引用。

```
[1s] L: blk25 q0/8 p800 | M: per10.02 jit180 wc240 | C: q1/4
[1s] P: eh90.0 ex0 ey15470 | QP: 6 | PATH: g10 r4
[1s] I2C: ok950 err2 hold2.9ms | H: 24680/32768 | WD: 0 | IMU: 100/s
[1s] STK: 1180 236 312 1296 700 480 760 820
[1s] T10: m10.02/180/240 p10.01/95/130 | ov m0 p0
[1s] T50: b50.03/120/900 o50.08/2100/11800 | ov b0 o0
[1s] CNT: ql0 qc0 qpt0 imu0 bt(s12 r9 o0 u0) log0 imu2 0
```

| 行 | 字段 | 含义 | 谁用它 |
|---|---|---|---|
| 1 | `blk` | 每秒完成的 DMA 块数 | 角色 1 / 3 |
| 1 | `q<peak>/<cap>` | Q_Lidar 峰值占用 / 容量 | 角色 3（容量论证） |
| 1 | `p<us>` | 最坏解析耗时 | 角色 1 / 2 |
| 1 | `M: per/jit/wc` | PID+Motor 任务：实测周期 ms / 抖动 µs / WCET µs | **角色 2** |
| 1 | `C: q<n>/<cap>` | Q_Cmd 当前深度 / 容量 | 角色 3 |
| 2 | `P: eh/ex/ey` | 位姿：航向 `eh` = 角度×10（`eh90.0` 即 90.0°）/ x mm / y mm | 角色 6 |
| 2 | `QP` | Q_Path 当前深度 | 角色 3 / 6 |
| 2 | `PATH: g/r` | 已生成路径点数 / 已到达点数 | 角色 6 |
| 3 | `I2C: ok/err` | 事务成功 / 失败次数 | **角色 5** |
| 3 | `hold<ms>` | **最长持锁时间**，`i2c_bus` 用 DWT 实测 | 角色 5（时序影响） |
| 3 | `H: <free>/<total>` | FreeRTOS 堆剩余 / 总量 | 角色 2 |
| 3 | `WD` | IWDG 喂狗计数（当前恒 0，看门狗尚未启用） | — |
| 3 | `IMU: <n>/s` | 实测 IMU 采样率，应 ≈100 | **角色 4** |
| 4 | `STK:` 8 个数 | 栈剩余字节，**顺序固定**：PID+Motor / IMU / Pose / LiDAR / Path / BT RX / BT TX / OLED | 角色 2 |
| 5 | `T10: m… p…` | 两个 10 ms 任务的 `周期ms/抖动µs/WCETµs` | 角色 2 |
| 5 | `ov m/p` | 10 ms 任务的周期超时次数 | 角色 2 |
| 6 | `T50: b… o…` | 两个 50 ms 任务的同上（b = BT TX，o = OLED） | 角色 2 |
| 6 | `ov b/o` | 50 ms 任务的超时次数 | 角色 2 |
| 7 | `CNT:` | 失败计数器：`ql`=Q_Lidar 丢弃、`qc`=Q_Cmd 覆盖、`qpt`=Q_Path 超时、`imu`=信号量满丢拍、`bt(s/r/o/u)`=语法/越界/环满/串口错、`log`=日志丢弃、`imu2`=等不到节拍 | 角色 3 / 4 |

**演示时的读法**：先看 `q`（解析跟上了吗）→ 看 `M: wc`（周期预算）→ 看 `I2C: hold`（是否超 3 ms）
→ 看 `STK`（栈够不够）→ 最后看 `CNT`（有没有静默失败）。

> `T10` / `T50` 里的 周期/抖动/WCET 是 `timing_*` 机制在运行中测出来的，**不是常量**。
> 报告里的表格必须抄这里的数，不许手写。

### 7.2 每个评分角色都要有自己的"证据"

见 §8。**没有数字的"我实现了"不算证明。**

---

## 8. 评分对接（6 个角色 ↔ 代码 ↔ 证据）

Part 2 是**按角色给分**，每人 1 分，缺席 0 分。所以代码位置和证据要对得上。

| 角色 | 代码位置 | 必须能现场展示的证据 |
|---|---|---|
| 1 完整应用 | `app.c` / `app_rtos.c` 的启动与任务创建 | 并发跑起来 + 蓝牙下命令 + 车走完 + PC 可视化同步 |
| 2 任务调度 | 所有 `osDelayUntil` 调用点、`osThreadNew` 参数表 | 任务表（优先级/栈/周期）+ **实测周期与抖动** + 超时计数 |
| 3 消息队列 | `Q_Lidar` / `Q_Cmd` / `Q_Path` 的定义与收发 | 三个队列都工作 + 容量选择理由 + 溢出计数（可故意制造一次） |
| 4 信号量 | TIM6 ISR 的 `osSemaphoreRelease` + IMU 任务的 `Acquire` | **实测采样率 ~100 Hz** + 中断→任务同步的时序 |
| 5 互斥量 | `osMutexNew(PrioInherit)` + I2C 事务包裹处 | 持锁时长 + 优先级继承演示（人为制造优先级反转对比） |
| 6 蓝牙导航 | BT RX → `Q_Cmd` → 路径生成 → `Q_Path` → PID → 电机 | 三次试验的**端点误差与航向误差**数字 |

> 团队资格：**Step 2 与 Step 3 未完成 = 全队 0 分**。所以别只顾实现，分析题和文档也要交。

---

## 9. 为"最终成品"预留的扩展位（队友的要求）

队友描述的成品形态：4 按钮 + OLED 选场地大小/单格边长/起始格/目标格/朝向 → 雷达判断起点格内绝对位置 → 迷宫循迹。
**这不是 Part 2 的验收项**，但架构上必须能接得上，否则以后要大改。

### 9.1 关键设计：`Q_Path` 是唯一的抽象缝

```
坐标模式（Part 2）:  蓝牙 (x,y) → Path Generation → Q_Path → PID → 电机
网格/迷宫模式（成品）: 按钮+OLED 配置 → 迷宫求解  → Q_Path → PID → 电机
                                                       ↑
                                          两者共用，PID 与电机层完全不知道来源
```

**规定**：`Q_Path` 的消息必须是**世界坐标路径点**（`{x_mm, y_mm}` 或 `{x_mm, y_mm, flags}`），
路径生成阶段负责把"格子"换算成坐标。这样：
- Part 2 的 ICP/OGM/A*（Final Development Block）可以换掉 Path Generation 而不动控制层
- 迷宫模式只是另一个 Path Generation 实现
- **PID 与电机层代码一行都不用改**

### 9.2 配置结构（一次定义，两处共用）

```c
typedef struct {
    uint8_t  field_w, field_h;      /* 场地格数，如 4 x 4 */
    uint16_t cell_mm;               /* 单格边长 mm      */
    uint8_t  start_x, start_y;      /* 起始格           */
    uint8_t  target_x, target_y;    /* 目标格           */
    uint8_t  heading;               /* 0=N 1=E 2=S 3=W  */
} nav_config_t;
```

**规定**：`nav_config_t` 只能由 UI 任务写、由 Path Generation 读；
写入必须走一个队列/事件，不许直接共享全局变量。

### 9.3 按键与 OLED UI

- 4 个按键走 **EXTI + 20 ms 软件消抖**（在任务里做，不在 ISR 里 `osDelay`）
- 按键事件进一个小队列，UI 状态机在 OLED 任务里跑：
  `IDLE → FIELD → CELL → START → TARGET → HEADING → RUN`
- OLED 刷新必须分页（§3.4），不许一次刷全屏
- **UI 任务优先级最低**，任何情况下不能影响控制环

### 9.4 起点格内绝对定位（成品功能）

选好起始格与朝向后，用雷达测起点格的四壁距离，算出格内偏移，再叠加到"格子中心"作为初始位姿。
**实现时只读 360 格扫描图的快照**（一份拷贝），不要在持锁/临界区里做计算。

---

## 10. 禁止事项清单（评审会直接扣分）

1. ❌ 在任务里 `HAL_Delay`
2. ❌ 在 ISR 里做 I2C/串口收发或任何循环
3. ❌ 优先级 5 以下的中断调用 RTOS API
4. ❌ 改了 TIM7 的优先级或去掉开机时基校准
5. ❌ `osMessageQueuePut/Get` 的返回值不检查
6. ❌ 用全局变量在任务之间传数据（而不是队列）
7. ❌ 没拿互斥量就访问 I2C1
8. ❌ 用 `osDelay()` 冒充固定周期
9. ❌ 栈剩余 < 30% 还继续加功能
10. ❌ 提交/演示版本里留着 `IWDG` 却没接喂狗逻辑（会反复复位）
11. ❌ 把 `course_materials/` 或课程 PPT/PDF 提交进 public 仓库
12. ❌ 在 `.ioc` 里按 Generate Code（会冲掉 FreeRTOS 的 include 路径和优先级修正）

---

## 11. 提交前检查清单

```
[ ] 编译 0 warning / 0 error（-Wall）
[ ] 连续跑 10 分钟不复位、不 HardFault
[ ] IWDG 已启用且有健康门控喂狗
[ ] 1 Hz 状态行所有字段都是真实测量值，不是常量
[ ] 每个 10 ms 任务实测 WCET < 预算，超时计数 = 0
[ ] 所有任务栈剩余 > 30%
[ ] 堆 min_free 与 free 相等（无泄漏）
[ ] 三个队列的峰值占用记录在案，容量有理由
[ ] 互斥量最长持锁时间 < 3 ms
[ ] 三次导航试验的端点误差/航向误差已记录
[ ] 拔掉雷达 / 拔掉蓝牙 / 按 STOP 三种故障都能自恢复
[ ] SWD 仍能连接（烧录口没被占用）
[ ] 报告与 README 已更新到 Part 2 的内容
```
