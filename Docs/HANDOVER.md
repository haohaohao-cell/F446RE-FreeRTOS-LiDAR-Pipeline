# 交接文档 —— Block 4.5 Coding Day（改版后）

> 用途：把这个项目的全部上下文交给**新的对话/新的接手人**，使其在零上下文的情况下能继续工作。
> 最后更新：2026-11-16。读这份文档的人请先读完「一、二、五、六」四节再动手。

---

## 一、当前状态一句话

**旧任务（LiDAR 流水线）已完成、上板实测通过、已上 GitHub。**
**但课程把任务改版了**：现在是 Part 1（周一交，纸面分析）+ Part 2（周五交，整车自主导航）。
Part 2 的工作量远大于旧任务，尚未开始。

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

- [x] 移植硬件层（**仅 motor + encoder，0 warning 编译通过，未上板**）：`motor.c`（TIM4+TIM2 PWM、PID）、`encoder.c`（TIM1/TIM8 x4，1320 counts/rev）
- [ ] `Oled.c` 移植（传输层要从软 I2C 换成 I2C1 + 互斥量；**等 §7.1 的 OLED 型号实测结果**，当前只有接口）
- [ ] 解决引脚/定时器冲突（已定：PB6/PB7 = 左轮 TIM4，PA0/PA1 = 右轮 TIM2 假设值；OLED 从 PB0/PA4 移到 I2C1）
- [ ] 蓝牙**接收** + 命令解析 → `Q_Cmd`（`bt_protocol.h` 接口已定，实现待写）
- [ ] 路径生成 → `Q_Path`（步长由团队定）
- [ ] 里程计（编码器 → 位姿）
- [ ] IMU 改成 **信号量**驱动（TIM6 10 ms ISR → 任务）
- [ ] **I2C 互斥量 + 优先级继承**（MPU6500 + OLED）
- [x] **接口已定**：`Core/Inc/i2c_bus.h`（I2C1 共享总线 + 互斥量唯一所有者）、
      `Core/Inc/oled.h`（显存/绘图 + 单页刷新 + 传输缝）、
      `Core/Inc/bt_rx.h`（IDLE 行检测 + 环形缓冲 + sink 回调）——实现 .c 待写
- [ ] PID 路径跟踪任务（10 ms）
- [ ] OLED 接入并挂到 **I2C1**（与 IMU 同总线）
- [ ] 任务架构重整（**8 个线程**，见 `CODING_STANDARDS.md` §1.2 与 10→8 映射表）
- [ ] 三次导航实测 + 端点/航向误差记录
- [ ] 重写报告/README/演示材料（现在都是旧任务的）

---

## 七、开放项 TODO（含 ASSUMPTION 标记）

> 规则：带 `[待实测]` 的项，代码里对应常量**必须保留 ASSUMPTION 注释**，不许因为"能编译"就当成已知值。
> 已拍板的设计决策见 §二 与 `Docs/CODING_STANDARDS.md` §1.2，不再列为开放项。

### 7.1 必须实测（硬件，今晚量）

- [ ] **[待实测] OLED 型号**：拆模块背面看丝印 —— 4 针 `GND VCC SCL SDA` = I2C 版 SSD1306（地址 0x3C，少数 0x3D 有跳线）；7 针 `GND VCC D0 D1 RES DC CS` = **SPI 版，方案要改，立刻上报**。供电 **必须 3.3 V**（5 V 会把 SCL/SDA 上拉拉到 5 V）。
- [ ] **[待实测] 轮径**：卡尺量轮胎外径，或推车让轮转 10 圈量直线距离 /10/π。
- [ ] **[待实测] 轮距**：两轮接地中心距（不是车宽）。
- [ ] **[待实测] 里程标定**：手推 1.000 m 读编码器总计数，`mm_per_count = 1000 / counts`，与 `1320 counts/rev × π × 轮径` 对比，**偏差 > 3% 说明轮径或 PPR 有误**。
- [ ] **[待实测] 左右编码器前向符号**：手推车前进，看两个 CNT 是增是减，确认 `BOARD_ENC_L/R_FORWARD_SIGN`。映射反了的表现是**原地打转**。
- [ ] **[待实测] 电机正反向与 PB6/PB7 的对应**：确认 `BOARD_MOTOR_L_FORWARD_CH/REVERSE_CH` 与实际 H 桥输入一致，以及 `BOARD_MOTOR_RIGHT_INVERT` 是否真的需要取反。
- [ ] **[待实测] TIM6 实际节拍周期**：看启动横幅 `fast tick:` 行的 标称/实测/修正后；若 `update ... -> …Hz` 不在 100 Hz 附近，说明校准失败，先查 TIM6 是否已启动。
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
