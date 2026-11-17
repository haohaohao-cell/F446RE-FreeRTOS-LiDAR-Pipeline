# 交接文档 —— Block 4.5 Coding Day（改版后）

> 用途：把这个项目的全部上下文交给**新的对话/新的接手人**，使其在零上下文的情况下能继续工作。
> 最后更新：2026-11-16。读这份文档的人请先读完「一、二、五、六」四节再动手。

---

## 一、当前状态一句话

**旧任务（LiDAR 流水线）已完成、上板实测通过、已上 GitHub。**
**课程改版为 Part 1 + Part 2。Part 1 已完成；Part 2 的 Phase B（硬件层）与 Phase C（RTOS 骨架/8 线程）已完成代码，编译 0 warning / 0 error，但除旧任务已验证的部分外，全部未上板。**
详见 §十一（Phase B/C 实现记录）与 §七（待实测清单）。

---

## 二、任务变更（最重要）

| | 旧（已完成的那个） | 新 |
|---|---|---|
| 结构 | 单一交付物：FreeRTOS LiDAR 数据流水线 | **Part 1 + Part 2 两部分** |
| Part 1 | — | CMSIS-RTOS2 学习活动：跑示例代码 + 分析一道 Producer/Consumer 设计题（4 个 Investigation 要写计算与论证） |
| Part 2 | — | Sense→Think→Act 整车自主导航：蓝牙给目标坐标 → 路径生成 → 编码器里程计 + IMU 航向 → PID 跟踪 |
| 截止 | — | Part 1 **周一**，Part 2 **周五** |
| 评分 | 团队演示 | **6 个角色各 1 分**；Step2&3 没做完 = 全队 0 分；缺席 = 0 分；演示现场不许改代码调参 |

### Part 2 必须实现的 6 件事（原文要求，不可裁剪）

1. **完整应用**：LiDAR + IMU + 编码器 + 里程计 + 电机 + OLED + 蓝牙遥测全部并发；新增 **2 个任务**（蓝牙接收、PID 路径跟踪）
2. **任务调度**：重整成 **固定周期 10 ms / 50 ms + 事件驱动**混合架构
3. **三个队列**：`Q_Lidar`（DMA 半缓冲描述符）、`Q_Cmd`（蓝牙 x,y / STOP）、`Q_Path`（路径点）
4. **信号量**：10 ms 定时器中断 → `osSemaphoreRelease` → IMU 任务 `osSemaphoreAcquire`
5. **互斥量**：`osMutexPrioInherit` 保护 MPU6500 与 OLED **共享的 I2C**
6. **PID 路径跟踪**（10 ms）+ 三次实测导航：**(0, 1.40) / (−0.70, 0.70) / (0.70, −0.70)** 米

### 参考任务架构（PPT 给的，10 个任务）

| 任务 | 调度方式 | 时基/触发 |
|---|---|---|
| LiDAR Processing | 事件驱动 | DMA HT/TC → Q_Lidar |
| IMU Acquisition | 事件驱动 | 10 ms 定时器 → 信号量 |
| Encoder Capture | 硬件中断/捕获 | 计数每 10 ms 处理 |
| Bluetooth RX（新） | 事件驱动 | UART RX → Q_Cmd |
| Pose Estimation | 固定周期 | 10 ms / 100 Hz |
| Path Generation | 事件驱动 | Q_Cmd → Q_Path |
| PID Path Tracking（新） | 固定周期 | 10 ms / 100 Hz |
| Motor Control | 固定周期 | 10 ms / 100 Hz |
| OLED Display | 固定周期 | 50 ms / 20 Hz |
| Bluetooth TX | 固定周期 | 50 ms / 20 Hz |

> PPT 强调：**所有 CMSIS-RTOS2 机制必须通过实际运行、数据交换、同步和实测时序来证明，不能只靠"代码里有 API 调用"。**

### ⚠️ 任务范围的一个歧义（重要）

队友的提示词描述了「4 个按钮 + OLED 选场地大小/单格边长/起始格/目标格/朝向 → 雷达判断起点格内绝对位置 → 迷宫循迹」。
这是**最终成品（循迹小车）**的形态，**不是 Part 2 的评分要求**。Part 2 明确是**坐标路径点**（三次给定坐标）。
处理方式：Part 2 按坐标做；架构上给网格/迷宫模式**留出扩展位**（见规范文档 §9），别把迷宫当 Part 2 的验收项。

---

## 三、代码与仓库现状

| 项目 | 值 |
|---|---|
| GitHub（公开） | https://github.com/haohaohao-cell/F446RE-FreeRTOS-LiDAR-Pipeline |
| 本地工程 | `D:\嵌入式小车\CodingDay_F446RE\` |
| 分支 / 提交 | `main`，两个提交，工作区干净 |
| 编译 | **0 warning / 0 error**，约 55 kB Flash / 42 kB RAM（`-O0`） |
| 说明文档 | `README.md`（英文）、`README.zh-CN.md`（中文，队友用这个） |

> ⚠️ 仓库是 **public** 的。**课程资料（`course_materials/`）绝不能提交进去。**
> 报告/演示材料目前都是按**旧任务**写的，Part 2 定稿后要大改。

### 已完成代码（文件级）

| 文件 | 内容 | 对 Part 2 的价值 |
|---|---|---|
| `Core/Src/lidar.c` | C1 驱动：DMA 半缓冲 → 12 字节描述符入队；逐块解析；**流看门狗**（1 s 无块则重发 SCAN）；`s`/`g` 控制台请求标志 | **就是 `Q_Lidar` 的要求**，几乎不用改 |
| `Core/Src/app_rtos.c` | 两个队列 + 解析任务 + 输出任务；1 Hz 统计行；USART2 单字节轮询（`s`/`g`） | 要扩成 10 个任务 |
| `Core/Src/app.c` | 启动顺序（DWT→驱动→NVIC→时基校准→建 RTOS 对象→开 DMA）、NVIC 优先级表、HAL 回调 | 启动顺序和优先级表必须保留 |
| `Core/Src/stm32f4xx_hal_timebase_tim.c` | HAL tick 移到 TIM7 + **用 DWT 开机校准预分频** | 不要动，见 §五 |
| `Core/Inc/FreeRTOSConfig.h` | 内核配置（`configMAX_PRIORITIES 56`、`MAX_SYSCALL 5`、heap_4 32 KB 等） | 不要动 |
| `Core/Src/imu.c` | MPU-6500，**当前是 TIM6 100 Hz 中断里直接发 I2C** | Part 2 要改成信号量触发、任务上下文采集 |
| `Core/Src/telemetry.c` | UART5 230400 二进制遥测帧（XOR 校验）+ DMA1_Stream7 发送 | 保留，Part 2 仍要 PC 实时可视化 |
| `pc_app/main.py` | matplotlib 点云 + 航向显示 | 保留；Part 2 可能要加轨迹/位姿显示 |
| `tools/*.cmd` | 烧录/抓串口/实时终端/文档转换 | 直接用 |

---

## 四、硬件与接线事实（已核对，别再猜）

| 项目 | 事实 |
|---|---|
| MCU | **STM32F446RET6**，LQFP64，168 MHz，128 kB RAM |
| 调试/烧录 | SWD，PA13/PA14。ST-Link（FW V2J46，SN 0670FF525154887767045747） |
| 调试串口 | **USART2 @115200**（PA2/PA3）→ ST-Link 虚拟串口 = **COM8** |
| 雷达 | **USART3 @460800**（PC10/PC11）+ **DMA1_Stream1 循环 2048 B** |
| 蓝牙 | **UART5 @230400**（PC12/PD2）+ DMA1_Stream7 发送 = **COM10**（HC-04，MAC `0426040302F4`） |
| IMU | MPU-6500，I2C1 @100 kHz（PB8/PB9），WHO_AM_I 接受 0x70/0x71/0x73 |
| 定时器占用 | **TIM6** = IMU 100 Hz；**TIM7** = HAL tick 1 kHz。两者都不能再动 |
| 电机脚 | 当前固件只把 **PB6/PB7** 拉低（`motors_safe()`），**没有真正的 PWM/编码器** |
| 未接过的外设 | OLED、按键、电位器、电机驱动 —— 旧任务用不到，**接法未知** |

**PC 侧环境**

| 项 | 值 |
|---|---|
| 工具链 | `D:\CubeIDE\STM32CubeIDE_2.1.1\...\gnu-tools-for-stm32.14.3.rel1`（gcc 14.3）、`make 2.2.100` |
| 烧录 CLI | `STM32_Programmer_CLI.exe`（2.22.0），脚本已改为自动搜索 |
| Python（PC 可视化） | `py -3` = 3.14，含 matplotlib 3.10 / pyserial 3.5 / tkinter |
| Python（文档转换） | DSH 自带解释器，含 python-docx |
| GitHub CLI | `gh` 2.98.0，已登录 **haohaohao-cell** |
| 蓝牙串口限制 | Windows SPP **最高 115200**（固件发 230400，但 SPP 波特率只是名义值，不限速） |

---

## 五、已经踩过并修好的三个坑（**不要重新踩**）

这三个都只在真实硬件上才暴露，静态审查看不出来，重写时极易复发。

### 坑 1 · 把 HAL tick 搬到 TIM 上会整条停摆（BASEPRI）

FreeRTOS ARM_CM4F 端口把 `uxCriticalNesting` 初始化成 `0xaaaaaaaa`，只在
`xPortStartScheduler()` 里清零。**调度器启动前**每次 `osThreadNew` / `osMessageQueueNew`
都会 `taskENTER/EXIT_CRITICAL`，嵌套计数永远回不到 0，于是 `BASEPRI` 永久停在 `0x50`。

- BASEPRI **屏蔽外设中断，但不屏蔽内核异常** → SysTick 时基从来不受影响，**TIM 时基直接死**
- 现象：`HAL_GetTick()` 完全不走，所有 HAL 超时失效

> **修法（必须保留）**：HAL 时基中断（TIM7）优先级设 **0**，高于
> `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`（5）。安全，因为该 ISR 不调用任何 RTOS API。

### 坑 2 · TIM7 实测跑在 168 MHz 而不是手册算的 84 MHz

`CFGR=0x940A`（PPRE1=/4 → PCLK1=42 MHz）、`DCKCFGR=0`（TIMPRE=0），按手册
TIM7 应为 `2×PCLK1 = 84 MHz`，但实测**正好 2 倍**（40 次中断 / 预期 20 次）。

> **修法（必须保留）**：`hal_timebase_calibrate()` 开机用 **DWT 测 20 ms**，按
> `实测/预期` 修正预分频（PSC 83 → 167），横幅打印 `calib=40/20` 和
> `50 ms (dwt) -> 50 ms (hal) OK`。
> 如果 tick 错，所有 `HAL_GetTick()` 超时和**所有测出来的速率**都按同一比例错。

> **这条坑已经推广成规范**：同一个"APB1 定时器时钟与手册不符"的根因会影响 TIM6（10 ms 节拍）
> 和 TIM4/TIM2（20 kHz PWM）。所以 Part 2 不再逐个写死预分频，而是统一走
> `hal_tim_calibrate(htim, nominal_hz)`（`Core/Src/tim_cal.c`）：开机用 DWT 量一次实际时钟，
> 偏差 > 5% 自动修正，并把 **标称 / 实测 / 修正前 / 修正后** 打进启动横幅。
> TIM6 与 TIM4 各调一次，**必须在 `osKernelStart()` 之前**完成。
> 规范条文见 `CODING_STANDARDS.md` §4.4。**禁止**改 `BOARD_TIM_CLK_HZ` 了事 —— 先量再算。

### 坑 3 · NVIC 基线被别人写回 0/1

`MX_DMA_Init()`（DMA1_Stream1=0）、`HAL_UART_MspInit()`（USART3=0）、
`i2c_bus_recover()`（I2C1=1）、`telemetry_dma_init()`（DMA1_Stream7=0）都会自己写
NVIC 优先级，**后写者赢**。只要有一个在 5 之下，LiDAR 中断里第一次
`osMessageQueuePut()` 就撞 `vPortValidateInterruptPriority()` 死循环。

> **修法（必须保留）**：各驱动内部改成 5/6，且在 `app_init()` **最后**由
> `irq_priorities()` 统一重新断言。

### 另外两条经验（不是 bug，但重写会踩）

- **C1 上电后不会自己输出**，永远需要再收到一次 `A5 20`。拔线/断电后如果只重开 DMA 而不重发 SCAN，雷达永远静默。已实现 **1 s 流看门狗**（`cmd`、`lost` 计数可观测）。
- **蓝牙 SPP 只能开 115200**，别为了 230400 去改端口。

---

## 六、还没做 / 待办

### Part 1（周一，纯纸面，不依赖硬件）—— ✅ 已完成

- [x] Step 1：用 `course_materials` 里的 W4.x 示例跑任务/队列/信号量/互斥量 → 记录见 `Docs/Part1_Step1_FreeRTOS_Experiments.md`（源码核对版；真机复核清单待板子接上后勾选）
- [x] Step 2：4 个 Investigation 的完整解答

**参考答案（核对用，与下表不一致时以本表为准）**

| 项 | 结果 |
|---|---|
| 生产周期 / 速率 | 20 ms（`osDelayUntil(tick += 20)`）→ **50 样点/s** |
| 消费处理时间 / 最大消费速率 | 47 ms → **1 / 0.047 ≈ 21.28 样点/s** |
| 队列容量 | **3** |
| 0–180 ms 生产事件数 | **10 个**（t = 0, 20, …, 180） |
| 消费者时序 | t=0 取到 S0，0→47 处理；47 / 94 / 141 ms 各完成一次并取下一个 |
| **首次丢弃** | **t = 120 ms，样点 S6**（第 7 个）。t=100 后队列已满（S3,S4,S5），t=120 生产者以 `timeout=0` 非阻塞投递 → 无空位 → 丢失 |
| 0–180 ms 汇总 | 生产 10 = 消费 4（S0–S3，S3 于 141 ms 开始）+ 排队 3（S4,S5,S8）+ 丢弃 **3**（S6,S7,S9） |
| **最小队列容量（不丢弃）** | **6** —— 窗口内最大排队数出现在 t=180（S4–S9 共 6 个） |
| 能否长期不丢 | **不能**。50/s vs 21.28/s，**积压增长 ≈ 28.72 样点/s**，有限容量终将溢出 |
| 不持续积压的最大处理时间 | **≤ 20 ms**（消费速率 ≥ 生产速率） |

### Part 2（周五，主要工作量）

- [x] 移植硬件层（**motor + encoder + tim_cal，0 warning 编译通过，未上板**）：`motor.c`（TIM4+TIM2 PWM、PID）、`encoder.c`（TIM1/TIM8 x4，1320 counts/rev）
- [x] **OLED 驱动**（`oled.c`，128x64 SSD1306，走 I2C1 + 互斥量，按页刷新；6x8 字库在 `oled_font.c`，已离线渲染核对全部 95 个字符）
- [x] 引脚/定时器冲突（PB6/PB7 = 左轮 TIM4，PA0/PA1 = 右轮 TIM2 假设值；OLED 从 PB0/PA4 移到 I2C1）
- [x] **蓝牙接收 + 命令解析**（`bt_rx.c` DMA1_Stream0 + IDLE 行检测 + 环形缓冲；`bt_protocol.c` 的 `bt_parse_line()`/`bt_error_text()`，46 条主机侧单元测试全通过）
- [x] 路径生成 → `Q_Path`（`path.c`：直线路点生成 + 航向控制律，参数**待整定**）
- [x] 里程计（编码器 → 位姿，`pose.c`；18 条主机侧单元测试通过，**标定常数未实测**）
- [x] IMU 改成 **信号量**驱动（TIM6 10 ms ISR 只 `osSemaphoreRelease`，采样搬到 IMU 任务上下文）
- [x] **I2C 互斥量 + 优先级继承**（`i2c_bus.c`：`osMutexPrioInherit`、有界事务、重试、恢复、DWT 实测持锁时间）
- [x] **8 个线程**任务架构（`app_rtos.c`，见 §十一）+ 三个队列 + 事件标志
- [x] 1 Hz 状态行（7 行固定格式，见 §十一.6；无实测数据的字段标 0 并在启动横幅说明）
- [ ] **上板验证**（见 §七.1；这是现在唯一剩下的硬工作量）
- [ ] 三次导航实测 + 端点/航向误差记录
- [ ] 蓝牙下行回复 `A G x y` / `E code text`（`bt_protocol.h` 已定义格式，BT TX 侧尚未接线）
- [ ] IWDG（超时 1 s + 健康位门控喂狗，规范 §5.3；当前 `WD: 0` 明确表示未启用）
- [ ] 重写报告/README/演示材料（现在都是旧任务的）

---

## 七、开放项 TODO（含 ASSUMPTION 标记）

> 规则：带 `[待实测]` 的项，代码里对应常量**必须保留 ASSUMPTION 注释**，不许因为"能编译"就当成已知值。
> 已拍板的设计决策见 §二 与 `Docs/CODING_STANDARDS.md` §1.2，不再列为开放项。

### 7.1 必须实测（硬件，今晚量）

- [ ] **[待实测] OLED 型号**：拆模块背面看丝印 —— 4 针 `GND VCC SCL SDA` = I2C 版 SSD1306（地址 0x3C，少数 0x3D 有跳线）；7 针 `GND VCC D0 D1 RES DC CS` = **SPI 版，方案要改，立刻上报**。供电 **必须 3.3 V**（5 V 会把 SCL/SDA 上拉拉到 5 V）。
- [ ] **[待实测] OLED 是否应答**：启动横幅 `oled=` 字段。`0` 或有面板；`-3` = 地址不该 NACK（没插/地址不对）；`-1` = 总线错误。面板缺席时程序**照常运行**，只是不刷新。
- [ ] **[待实测] 1 Hz 行里 `I2C: hold` 的真实值**：这是 `i2c_bus` 用 DWT 实测的最长持锁时间。规范目标是 < 3 ms；OLED 一页按 32 字节分块传输，按 100 kHz 推算约 2.97 ms/块，**必须用实测值确认**（推算值不算数）。
- [ ] **[待实测] OLED 一页刷新的实际耗时**：`[1s] T50: o<per>/<jit>/<wc>` 的第三个数字就是 OLED 任务 WCET。按推算一页约 11–12 ms，**超出 §1.2 给的 3 ms 预算**；因为 OLED 是最低优先级，不影响控制环，但报告里必须写实测值 + 说明（或把 I2C 提到 400 kHz）。
- [ ] **[待实测] 轮径**：卡尺量轮胎外径，或推车让轮转 10 圈量直线距离 /10/π。
- [ ] **[待实测] 轮距**：两轮接地中心距（不是车宽）。
- [ ] **[待实测] 里程标定**：手推 1.000 m 读编码器总计数，`mm_per_count = 1000 / counts`，与 `1320 counts/rev × π × 轮径` 对比，**偏差 > 3% 说明轮径或 PPR 有误**。
- [ ] **[待实测] 左右编码器前向符号**：手推车前进，看两个 CNT 是增是减，确认 `BOARD_ENC_L/R_FORWARD_SIGN`。映射反了的表现是**原地打转**。
- [ ] **[待实测] 电机正反向与 PB6/PB7 的对应**：确认 `BOARD_MOTOR_L_FORWARD_CH/REVERSE_CH` 与实际 H 桥输入一致，以及 `BOARD_MOTOR_RIGHT_INVERT` 是否真的需要取反。
- [ ] **[待实测] TIM6 实际节拍周期**：看启动横幅 `fast tick:` 行的 标称/实测/修正后；若 `update ... -> …Hz` 不在 100 Hz 附近，说明校准失败，先查 TIM6 是否已启动。
- [ ] **[待实测] IMU 采样率**：1 Hz 行 `IMU: <n>/s` 必须 ≈ 100。若明显偏低，看 `CNT: imu<n>`（信号量满 = IMU 任务跟不上）和 `imu2<n>`（等不到节拍 = TIM6 停了）。
- [ ] **[待实测] 蓝牙命令链路**：串口助手（COM10，**115200**）发 `G 700 -700\n`。期望串口出现 `[nav] G 700 -700 accepted, 10 waypoints`；发 `S\n` 出现 `[nav] STOP`；`CNT: qc<n>` 在快速连发时会涨（Q_Cmd 覆盖最旧）。
- [ ] **[待实测] `?` / `P` 的回复**：目前解析器接受，但**下行回复还没接线**，所以蓝牙那头不会有回话，只有 USART2 调试口有日志。
- [ ] **[待实测] 8 个任务的栈剩余**：`[1s] STK:` 8 个数字（顺序见 §十一.5），每个都必须 > 30%。IMU 任务只有 512 B，最可能先不够。
- [ ] **[待实测] 真机 10 分钟跑机**：不能复位、不能 HardFault、堆 `H:` 不下降。
- [ ] **[待实测] 第二路电机到底接在哪四根线**：PB6/PB7 只能驱动一个电机，右轮当前按 **TIM2_CH1/CH2 = PA0/PA1** 假设实现（见 `board_pins.h` §3，标了 ASSUMPTION）。

### 7.2 必须问队友 / 课程

- [ ] **[待问队友] 团队几人**：6 个评分角色如何合并 —— 4 人：队列+信号量合并、互斥量并入调度；5 人：互斥量并入调度。**缺席 = 0 分**，Step2/Step3 不完成 = 全队 0 分。
- [ ] **[待问队友] `D:\stm32codespace\lessonProgram` 是不是 Coding Day 2 最终版**。
- [ ] **[待问队友] `Motor.h` 的 PID `3.0 / 0.15 / 0` 出处**：是不是同一台底盘上调的？换车必须重调，代码里已注明"需上板整定"，**只当保守初值**。
- [ ] **[待问队友] 蓝牙命令格式确认**：`G <x_mm> <y_mm>` / `S` / `P` / `?`，`\n` 结尾，y 向前为正（已按此实现 `bt_protocol.h`，若 PC 端格式不同要同步改）。

### 7.3 已拍板（不再是开放项）

| 决策 | 结论 |
|---|---|
| 蓝牙报文 | RX 用 ASCII 行文本，TX 保持二进制帧 `AA 55 …`；两个方向格式不同是刻意的 |
| 编码器 | 没有独立线程；TIM1/TIM8 硬件 QEP，Pose 在 10 ms 时隙内采样 |
| PID + Motor | 合并为 1 个 10 ms 线程（PPT 允许 coordinated task） |
| 队列 | 只有 `Q_Lidar` / `Q_Cmd` / `Q_Path` 三个，无 `Q_Enc`、无 `Q_Set` |
| 时基 | `hal_tim_calibrate()` 开机实测并自动修正，TIM6 与 TIM4 各一次，`osKernelStart()` 之前完成 |
| OLED 总线 | 确认接到 I2C1（PB8/PB9）与 MPU-6500 共享，否则互斥量评分点不成立 |

---

## 八、资料在哪

| 资料 | 路径 | 备注 |
|---|---|---|
| 课程示例代码（F446） | `D:\嵌入式小车\course_materials\EBU6470_Codes_F446\` | **有版权，禁止提交到仓库** |
| └ CMSIS-RTOS2 示例 | 上面目录下的 `W4.1_Tasks`、`W4.2.1~3_Queues_*`、`W4.3.1~2_Sem_*`、`W4.4_Event_flags`、`W4.5_Thread_flags`、`W4.6.1~2_Mutex_*` | Part 1 Step 1 就用这些 |
| └ 外设示例 | `W2.1_GPIO_LED`、`W2.2_EXTI_B1`、`W2.3_Timer_PWM`、`W2.4.x_ADC_*`、`W3.1_I2C_AHT30_Emulation` | 参考写法 |
| Coding Day 2 代码 | `D:\stm32codespace\lessonProgram\` | 同一颗 F446RET6；含 Motor/Encoder/Oled/Track/RunStopController/Board |
| 新任务 PPT | `D:\电脑管家迁移文件\xwechat_files\...\2026-11\Block4.5-CodingDay-Part1.pptx` / `-Part2.pptx` | 改版后的正式要求 |
| 旧任务 PPT/PDF | `D:\嵌入式小车\Block4.5-CodingDay.pptx`、`FreeRTOS_v2.0 - CMSIS OS API.pdf` | 旧版，已被取代；PDF 仍是有用的 API 参考 |

---

## 九、怎么用现有工具

```cmd
tools\flash_and_monitor.cmd              :: 探测→烧录→校验→抓 15 s 串口→自动判读
tools\flash_and_monitor.cmd -Probe       :: 只检查板子（区分"没插 ST-Link"和"板子没电"）
tools\serial_console.cmd                 :: 实时终端，可敲 s=停雷达 g=重启 h=帮助
pc_app\run_viewer.cmd                    :: 点云窗口（COM10，蓝牙）
pc_app\run_viewer.cmd --sim              :: 无硬件合成数据
py -3 tools\md2docx.py <in.md> <out.docx> --lang cn|en   :: 文档转换
```

编译用 STM32CubeIDE（Ctrl+B）。命令行 `make -C Debug` 要先在 IDE 里编过一次
（`Debug/` 是纯产物，已被 git 忽略）。

---

## 十、术语与命名约定

| 术语 | 含义 |
|---|---|
| `Q_Lidar` | 现有代码里的 `q_blocks` / "Queue 1"，12 字节块描述符 |
| `Q_Cmd` | 待建，蓝牙导航命令 `{x, y}` 或 `STOP` |
| `Q_Path` | 待建，路径点队列（Path Generation → PID Tracking） |
| 半缓冲 / half buffer | 2048 B DMA 缓冲的一半，1024 B |
| `blk` | 1 Hz 行里每秒完成的块数（~25） |
| `q1` / `q2` | 队列峰值占用 |
| `lat` | 中断 → 任务唤醒延迟（实测 12–15 µs） |
| `cmd` / `lost` | 已发 SCAN 命令数 / 流看门狗判定丢流次数 |

---

## 十一、Part 2 Phase B/C 实现记录（2026-11-17）

> 这一节是"代码里到底是什么"，与 §七（"还要在板上确认什么"）分开写。
> 除标注"主机侧已验证"的纯逻辑外，**本节所有数字都还没有在板子上量过**。

### 11.1 文件清单

**新增**

| 文件 | 作用 |
|---|---|
| `Core/Inc/i2c_bus.h` + `Src/i2c_bus.c` | I2C1 唯一所有者：`osMutexPrioInherit` 互斥量、有界事务、重试、时钟脉冲恢复、DWT 实测持锁时间 |
| `Core/Inc/oled.h` + `Src/oled.c` | SSD1306 128x64：显存/绘图、按页刷新、传输缝 `oled_bus_write()` |
| `Core/Inc/oled_font.h` + `Src/oled_font.c` | 6x8 ASCII 字库（0x20..0x7E 共 95 个，已离线渲染逐个核对） |
| `Core/Inc/bt_rx.h` + `Src/bt_rx.c` | UART5 RX：DMA1_Stream0 + IDLE 行检测 + 环形缓冲，ISR 只拷贝 |
| `Core/Src/bt_protocol.c` | `bt_parse_line()` / `bt_error_text()`（纯函数，无 HAL 无 RTOS） |
| `Core/Inc/pose.h` + `Src/pose.c` | 里程计 + 陀螺航向 → 位姿 |
| `Core/Inc/path.h` + `Src/path.c` | `Q_Path` 消息、直线路点生成、航向控制律 |
| `Core/Inc/log.h` + `Src/log.c` | USART2 TX 环形缓冲 + DMA（非阻塞日志，规范 §5.4） |

**修改**

| 文件 | 改了什么 |
|---|---|
| `Core/Src/app_rtos.c` / `Inc/app_rtos.h` | 2 个任务 → **8 个任务**、3 个队列、信号量、事件标志、1 Hz 状态行、任务计时 |
| `Core/Src/app.c` / `Inc/app.h` | 初始化顺序（RTOS → i2c → log → telemetry → imu → oled → pose → bt_rx → NVIC → 校准）、NVIC 表、新增回调（UART5 RX / TX / 错误）、非阻塞 `dbg_printf`、`dbg_task_started()` |
| `Core/Src/imu.c` / `Inc/imu.h` | 采样从 TIM6 中断搬到 IMU 任务；I2C 改走 `i2c_bus_*`；新增 `imu_sample_once()` |
| `Core/Src/stm32f4xx_it.c` | 新增 `DMA1_Stream0_IRQHandler`（蓝牙收）、`DMA1_Stream6_IRQHandler`（日志发） |
| `Core/Src/telemetry.c` / `Inc/telemetry.h` | `HAL_UART_TxCpltCallback` 移到 `app.c` 统一分发（否则与日志 DMA 抢同一个弱符号）；新增 `telemetry_on_tx_complete()` |
| `Core/Inc/i2c_bus.h` / `Inc/bt_rx.h` | **纯增量**：`I2C_BUS_*` 返回码、`stats.boot_locks`、`bt_rx_on_error()`、`bt_rx_dma_irq_handler()`、`stats.uart_errors`。既有函数签名与语义**未改** |

### 11.2 8 个任务的实际创建参数

| # | 线程 | 名字 | 优先级 | 栈(B) | 周期/触发 | 调度方式 |
|---|---|---|---|---|---|---|
| 1 | PID + Motor | `pidMotor` | 40 | 1024 | 10 ms | `osDelayUntil` |
| 2 | IMU | `imuAcq` | 37 | 512 | 10 ms | 计数信号量（TIM6） |
| 3 | Pose | `pose` | 36 | 768 | 10 ms | `osDelayUntil` |
| 4 | LiDAR | `lidarProc` | 34 | 1536 | ~22 ms/块 | `Q_Lidar`（100 ms 守卫超时） |
| 5 | PathGen | `pathGen` | 30 | 1024 | 命令到达 | `Q_Cmd`（`osWaitForever`） |
| 6 | BT RX | `btRx` | 28 | 768 | 字节到达 | 计数信号量（100 ms 守卫） |
| 7 | BT TX | `btTx` | 22 | 1024 | 50 ms | `osDelayUntil` |
| 8 | OLED | `oled` | 20 | 1024 | 50 ms | `osDelayUntil` |

- 优先级数字就是规范 §1.2 表里的数字（注意 CMSIS 枚举里 `osPriorityAboveNormal` 其实是 32，不是 37；**代码里用表里的数字并用 7 条 `_Static_assert` 锁死相对顺序**）。
- 每个任务启动时打印一行 `[task] <名字> per=<周期或 event> prio=<n> stk=<B>`，用来确认调度器真的启动了。这一行**不走 `vsnprintf`**，所以 512 B 栈的 IMU 任务也安全。
- 10 ms 任务的 WCET 之和预算 1.4 ms（§1.3 上限 2 ms），**实测值待上板**，见 1 Hz 行 `T10:` 字段。

### 11.3 三个队列

| 队列 | 容量 | 消息大小 | 生产者 → 消费者 | 溢出策略**落在哪一行代码** |
|---|---|---|---|---|
| `Q_Lidar` | 8 | 12 B `lidar_block_desc_t` | DMA ISR → LiDAR 任务 | 满则丢弃：`app_rtos_post_block()` 里 `blocks_dropped++` |
| `Q_Cmd` | 4 | 8 B `cmd_msg_t` | BT RX 任务 → PathGen | 满则覆盖最旧：`cmd_sink()` 里先 `osMessageQueueGet` 再 `Put`，`q_cmd_overwrite++` |
| `Q_Path` | 16 | 8 B `path_point_t` | PathGen → PID+Motor | 生产方阻塞 50 ms：`path_task()` 里 `osMessageQueuePut(..., APP_Q_PATH_PUT_TMO_MS)`，超时则 `q_path_timeout++` + 整条路径作废并回 `STOP` |

- **没有 `Q_Enc`**（编码器是 TIM1/TIM8 硬件计数器，Pose 任务直接读），**没有 `Q_Set`**（PID 与电机合并成一个任务，setpoint 是局部变量）。
- 停机通知用的是**事件标志** `evt_nav`（`STOP` / `NEW_PATH`），不是第四个队列：它没有载荷，而且 `Q_Path` 需要被整条清空。
- `Q_Path` 的消费端逻辑：追上当前路点后**在同一周期内继续取下一点**，避免每 100 mm 停一下。

### 11.4 中断优先级（`app_init()` 最后一次统一断言）

| IRQ | 优先级 | 调用 RTOS API | 理由 |
|---|---|---|---|
| TIM7（HAL tick） | **0（不许动）** | 否 | 交接文档坑 1 |
| USART3 / DMA1_Stream1（雷达） | 5 | 是（`Q_Lidar`） | 必须 ≥ `MAX_SYSCALL`(5) |
| **TIM6**（10 ms 节拍） | **5**（原 7） | 是（`osSemaphoreRelease`） | 同上，Part 2 要求 |
| **UART5**（蓝牙） | **5**（原 6） | 是（唤醒 BT RX） | 同上 |
| **DMA1_Stream0**（蓝牙收） | 5 | 是 | 新增 |
| I2C1 EV/ER | 6 | 否（事务已改为同步） | 不能抢占内核临界区 |
| DMA1_Stream6（日志发） / Stream7（遥测发） | 6 | 否 | 同上 |
| USART2 | 7 | 否（轮询） | — |

### 11.5 1 Hz 状态行（7 行，每行 ≤ 80 字符，都以 `[1s]` 开头便于 grep）

```
[1s] L: blk25 q0/8 p800 | M: per9.98 jit120 wc180 | C: q2/4
[1s] P: eh12.3 ex-4 ey0 | QP: 7 | PATH: g10 r3
[1s] I2C: ok950 err2 hold1.8ms | H: 24680/32768 | WD: 0 | IMU: 99/s
[1s] STK: 1040 520 498 512 380 690 900 1010
[1s] T10: m9.98/120/180 p10.02/90/120 | ov m0 p0
[1s] T50: b50.03/210/900 o50.01/140/1200 | ov b0 o0
[1s] CNT: ql0 qc0 qpt0 imu0 bt(s0 r0 o0 u0) log0 imu2 0
```

- **STK 顺序固定**：PID+Motor / IMU / Pose / LiDAR / PathGen / BT RX / BT TX / OLED（单位 B，剩余量）。
- `T10` / `T50` 每项是 `周期ms/抖动µs/WCETµs`，`ov` 是超时次数；`m`=电机、`p`=Pose、`b`=BT TX、`o`=OLED。
- `CNT`：`ql`=Q_Lidar 丢块、`qc`=Q_Cmd 覆盖、`qpt`=Q_Path 超时、`imu`=信号量满（IMU 落后）、`bt(s r o u)`=语法错/越界/超长/串口错、`log`=日志丢字节、`imu2`=等不到 TIM6 节拍次数。
- **`WD: 0` 是刻意的**：IWDG 还没启用，不许把 0 写成别的数字假装看门狗在跑。
- 与规范 §7 的差异（见 11.6）：为了每行 ≤ 80 字符且覆盖"每个周期任务"，拆成 7 行而不是 3 行，字段集合是 §7 的超集。

### 11.6 与现有文档的三处偏差（必须知道，别当 bug 又"修"回去）

1. **`oled.h` / 规范 §3.4 的"一页 ≈1.5 ms"算错了。** 100 kHz 下 128 B 数据 + 1 控制字节 = 129 B = 1032 bit ≈ **11.6 ms**，不是 1.5 ms（1.5 ms 对不上 100 kHz 也对不上 400 kHz）。代码因此把一页拆成 **32 字节/事务**（33 B ≈ 2.97 ms），这样单次持锁仍在 §2.4 的 3 ms 目标内；代价是一页要 5 次事务。**真实持锁时间以 `I2C: hold` 的实测值为准。**
   → 直接后果：OLED 任务单次执行时间约 11–12 ms，**超过 §1.2 给的 3 ms WCET 预算**。因为它是优先级最低的任务，不影响控制环；但报告里要么写实测值并说明，要么把 I2C 提到 400 kHz（约 3 ms/页）。
2. **`oled.h` 的 `OLED_ERR_ABSENT` 与 `OLED_ERR_BUS` 的区分**靠 `i2c_bus` 新增的 `I2C_BUS_NACK`/`I2C_BUS_ERR`：地址 NACK = 面板不在；其它错误 = 总线坏。既有返回码语义没变。
3. **状态行 7 行**（见 11.5）替代 §7 的 3 行示例；另外 §5.4 的"单行 ≤ 80 字符 / 每秒 1 行"改成了"DMA 环形缓冲 + 每行 ≤ 80 字符"—因为 `dbg_printf` 已改成非阻塞（`log.c`），不再占着 UART 阻塞任务。

### 11.7 编译结果

```
make -C Debug -j8 all
text 86468  data 96  bss 47712   0 warning / 0 error
```

（基线是 text 59048 / 0 warning；RAM 用量 data+bss = 47.8 KB / 128 KB。）
`Debug/Core/Src/subdir.mk` 与 `Debug/objects.list` 是 CubeIDE 生成的产物（`Debug/` 已被 git 忽略），
新增源文件后需要它们同步更新，否则**编译得出 .o 但链接时根本没带上**（`objects.list` 才是链接清单）。
在 IDE 里按 F5 刷新或在 IDE 里编一次即可自动重生成。

### 11.8 哪些逻辑真的验证过（只有这些，别扩大）

| 模块 | 验证方式 | 结果 |
|---|---|---|
| `bt_parse_line` / `bt_error_text` | 主机 gcc 单元测试 `tools/host_tests/test_bt_protocol.c` | 46 条全通过：大小写、重复空格、CRLF、缺参、粘连 token、尾随垃圾、`\n` 之后还有垃圾、±5000 边界、21 位数字不溢出、NULL 参数、失败时 `*out` 不被改写 |
| `path_generate_line` / `path_track_step` / `path_arrived` | 主机 gcc 单元测试 `tools/host_tests/test_path.c` | 61 条全通过：路点数量/末点精确/FINAL 标志/单调性/步长/容量边界、到达半径边界、直行左右等速、原地转向左右轮反向且对称、偏航限幅、零距离不产生 NaN |
| `pose_update` 符号约定 | 主机 gcc 单元测试 `tools/host_tests/test_pose.c` | 18 条全通过：前进 +y、倒车 −y、陀螺 +90° 偏移、轮差航向符号、原地转不位移、航向回绕、零周期不产生 inf |
| 字库 6x8 | 离线渲染成 PNG 逐个看 | 95 个字符全部可读、位置正确 |
| 其余（HAL 时序、DMA、RTOS 调度、OLED/IMU 真机行为） | **没有验证过** | 见 §七.1 |

跑法（任意主机 C 编译器，不需要板子）：

```cmd
tools\host_tests\run_tests.cmd
:: 期望最后一行 ALL HOST TESTS PASSED
```

> **注意分寸**：主机测试全绿只证明"逻辑自洽"，**不证明车能跑**。
> 凡是依赖 HAL / DMA / RTOS / 真面板的，一律算 §七.1 的待上板项。


