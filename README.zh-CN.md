# F446RE 双轮差速导航小车（Block 4.5 Part 2）

**STM32F446RE + FreeRTOS 10.3.1（CMSIS-RTOS v2）**。
蓝牙给目标坐标，车自己规划路点、用编码器 + 陀螺闭环走到点。

> **一句话成绩**：蓝牙下令走 1.4 m，**卷尺实测误差 10 mm 以内**，航向偏差 1.2°；
> 8 个线程并发，10 ms 控制环的最坏执行时间 11 µs，超时计数 0。

*English version: [README.md](README.md).*

---

## 这是什么

课程 Block 4.5 的第二部分（Part 1 是纯分析的雷达流水线，那个流水线在这里**仍然活着**，
是 8 个线程里的第 4/6/7 个，也是队列与信号量评分点的证据来源）。

| 评分角色 | 本工程对应 | 证据 |
|---|---|---|
| 1 完整应用 | 8 线程 + 蓝牙 + 3 队列 | 1 Hz 状态行（除 OLED 外全部并发） |
| 2 任务调度 | `osDelayUntil` + 实测计时 | 周期/抖动/WCET/超时计数 |
| 3 消息队列 | `Q_Lidar` / `Q_Cmd` / `Q_Path` | 峰值占用 + 溢出计数 |
| 4 信号量 | TIM6 → IMU 任务 | 实测 100 Hz |
| 5 互斥量 | I2C1 + `osMutexPrioInherit` | 持锁 1.5 ms + `i` 键继承对比 |
| 6 蓝牙导航 | BT → 路径 → PID → 电机 | 三趟地面的端点/航向误差 |

---

## 架构

```
     蓝牙 COM10 ──"G 0 1400\n"──> UART5 ──DMA1_S0──> bt_rx.c ──信号量──> BT RX 任务
                                                                          │ Q_Cmd (4×8B, 满则覆盖最旧)
                                                                          v
                                                                   PathGen 任务（100 mm 步长）
                                                                          │ Q_Path (16×8B, 生产方阻塞)
                                                                          v
   TIM1/TIM8 硬件 QEP ──> Pose 任务 ────────────────────────────────> PID+Motor 任务（10 ms）
   MPU-6500 ──I2C1(互斥量)──> imu.c 陀螺 ──> pose.c                         │ 一次算完一次写 CCR
                                                                          v
                                                              TIM4 / TIM2 H 桥 ──> 左右轮

   RPLidar C1 ──USART3+DMA(循环 2×1024B)──> lidar.c ──> Q_Lidar (8×12B, 满则丢) ──> LiDAR 任务
   BT TX / OLED 任务：50 ms 周期，输出 1 Hz 状态块 + 二进制遥测 + 分页显示
```

**8 个线程**（优先级：数大者优先）：

| # | 线程 | 优先级 | 栈 | 周期/触发 | 调度 |
|---|---|---|---|---|---|
| 1 | `pidMotor` | 40 | 1024 | 10 ms | `osDelayUntil` |
| 2 | `imuAcq` | 37 | 1024 | 10 ms | 计数信号量（TIM6） |
| 3 | `pose` | 36 | 768 | 10 ms | `osDelayUntil` |
| 4 | `lidarProc` | 34 | 1536 | ~22 ms/块 | `Q_Lidar`（100 ms 守卫） |
| 5 | `pathGen` | 30 | 1024 | 命令到达 | `Q_Cmd` |
| 6 | `btRx` | 28 | 768 | 字节到达 | 计数信号量 |
| 7 | `btTx` | 22 | 1024 | 50 ms | `osDelayUntil` |
| 8 | `oled` | 20 | 2048 | 50 ms | `osDelayUntil` |

> **编码器不是线程**（TIM1/TIM8 硬件正交解码，Pose 每 10 ms 读一次 CNT）；
> **PID 与电机合并成一个任务**（setpoint 变成任务内局部变量，零延迟）。
> 这就是 PPT 的 10 行参考表为什么在实现里是 8 个线程。

---

## 实测数据（目标板，2026-11-17 / 11-18）

| 指标 | 实测 | 看哪个字段 |
|---|---|---|
| 电机任务周期 / 抖动 / WCET | **10.00 ms / 42 µs / 11 µs**（预算 700 µs） | `M: per/jit/wc` |
| 周期任务超时计数 | **0** | `ov m0 p0` |
| IMU 采样率 | **100 /s** | `IMU:` |
| I2C 最长持锁 | **1.5 ms**（目标 <3 ms） | `hold` |
| LiDAR 流水线 | **24–25 块/s**，`Q_Lidar` 峰值 0/8 | `blk` / `q` |
| 堆 | **19 816 / 32 768 B** | `H:` |
| 栈剩余（8 个） | 680 / 504 / 464 / 1280 / 768 / 576 / 636 / 1096 B | `STK:` |
| 失败计数器 | 全 0（`ql qc qpt imu bt log iw`） | `CNT:` |
| **导航 1.4 m 直线** | 固件 **12 mm / 0.9 %**；卷尺 **≤10 mm**；航向 +1.2°；13.2 s | `nav_run` 表 |
| 导航 (−0.70, 0.70) | 固件 18 mm / 1.8 %；卷尺 52 mm（含 45° 原地转） | 同上 |
| 导航 (0.70, −0.70) | 固件 14 mm / 1.5 %；卷尺 >30 mm（含 135° 原地转） | 同上 |
| 左右轮对称性 | R/L **1.03 / 1.14**（修复前 1.80 / 4.74） | `bench_p0 -Test symmetry` |
| 优先级继承 | 持锁者优先级 **11 → 13**（mutex）/ 11（信号量） | `i` 键 |

原始日志在 `tools/logs/`（`serial_*` 1 Hz 行、`bench_*` 台架、`nav_*` 导航 + CSV）。
完整分析见 [`Docs/Block4.5_Part2_Design_Report_CN.docx`](Docs/Block4.5_Part2_Design_Report_CN.docx)。

---

## 硬件

| 项目 | 连接 |
|---|---|
| MCU | NUCLEO-F446RE，168 MHz，128 kB SRAM |
| 右轮电机 / 编码器 | **PB6 / PB7**（TIM4 CH1/CH2，极性反，由 `RIGHT_INVERT` 补偿）/ **PA8 / PA9**（TIM1 QEP） |
| 左轮电机 / 编码器 | **PA0 / PA1**（TIM2 CH1/CH2）/ **PC6 / PC7**（TIM8 QEP） |
| IMU | MPU-6500，I2C1（PB8/PB9），0x68 |
| 蓝牙 | HC-04，UART5（PC12/PD2），Windows SPP = **COM10** |
| 调试串口 | USART2（PA2/PA3）→ ST-Link 虚拟串口 **COM8** @115 200 |
| 雷达 | RPLidar C1，USART3 @460 800 + DMA1_Stream1（PC10/PC11） |
| OLED | SSD1306 走 I2C1（0x3C）—— **驱动已写，面板未接**（`oled=-3`） |

引脚/定时器/NVIC 的**唯一真源**是 [`Core/Inc/board_pins.h`](Core/Inc/board_pins.h)，不要在别处写死数字。

> ⚠️ **不要交换右电机的两根动力线**：固件里的 `BOARD_MOTOR_RIGHT_INVERT=1` 已经补偿过了，
> 换线会双重取反。

---

## 快速开始

### 1. 编译

`make` 不在 PATH 里，先加 CubeIDE 的 make 与 arm-none-eabi：

```powershell
$env:PATH = "D:\CubeIDE\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.100.202601091506\tools\bin;D:\CubeIDE\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin;" + $env:PATH
```

```cmd
make -C Debug -j8 all
```

要求 **0 warning / 0 error**，基线 **text = 90 244 B**。

### 2. 烧录 + 抓串口

```cmd
tools\flash_and_monitor.cmd              :: 探测 → 烧录 → 校验 → 抓 15 s
tools\flash_and_monitor.cmd -Probe       :: 只探测（区分"没插 ST-Link"和"板子没电"）
tools\serial_console.cmd                 :: 实时终端（s/g/f/b/l/r/x/c/e/i/h）
```

### 3. 让车走

```cmd
:: 有线（调试用：同时开 COM8 + COM10，自动读回终点位姿）
tools\nav_run.cmd -Only 0,1400

:: 无线（验收用：只走蓝牙，终点用卷尺量）
tools\bt_send.cmd "G 0 1400"
```

车头方向约定：**+y 是车头向前，+x 是车的右侧**，坐标单位 mm，范围 ±5000。

---

## 控制台与命令

**COM8 @115 200**（USART2）：

| 键 | 作用 |
|---|---|
| `s` / `g` | 停 / 重启激光（旧功能，未破坏） |
| `f` / `b` | 两轮正/反转 40%，1.5 s 自动停 |
| `l` / `r` | 只驱动左/右通道 40%，1.5 s |
| `x` | 立即请求停止 |
| `c` | 位姿 + 两路编码器清零（由 Pose 任务执行，保证不撕裂） |
| `e` | 读 `L= R= cnt \| rpm \| x y \| h gyro` |
| `i` | **角色 5：优先级继承对比演示** |
| `h` / `?` | 帮助 |

**COM10 @115 200**（蓝牙，一行一条）：`G <x_mm> <y_mm>` / `Z`（清零位姿+清路径）/ `S` / `P` / `?`

> 点动占空比是 **40%**，不是 20%：实测这块电池在 20–30% 是死区（两轮都不转）。

---

## 输出怎么读

每秒 7 行，前缀固定 `[1s]`：

```
[1s] L: blk24 q0/8 p2435 | M: per10.00 jit42 wc11 | C: q0/4
[1s] P: eh90.0 ex0 ey0 | QP: 0 | PATH: g0 r0
[1s] I2C: ok1401 err1 hold1.5ms | H: 19816/32768 | WD: 0 | IMU: 100/s
[1s] STK: 680 504 464 1280 768 576 636 1096
[1s] T10: m10.00/42/11 p11.38/1383/12 | ov m0 p0
[1s] T50: b51.26/1267/281 o51.09/1100/285 | ov b0 o0
[1s] CNT: ql0 qc0 qpt0 imu0 bt(s0 r0 o0 u0) log0 iw0
```

| 行 | 意思 |
|---|---|
| 1 | 激光块速率 / `Q_Lidar` 峰值 / 最坏解析耗时；电机任务 周期·抖动·WCET；`Q_Cmd` 深度 |
| 2 | 位姿：航向 `eh`（×0.1°）、`ex`/`ey`（mm）；`Q_Path` 深度；路径**累计**生成/到达数 |
| 3 | I2C 成功/失败/最长持锁；堆；看门狗（未启用）；IMU 实测采样率 |
| 4 | 8 个任务栈剩余（顺序：PID/IMU/Pose/LiDAR/Path/BTRX/BTTX/OLED） |
| 5–6 | 4 个周期任务的 周期/抖动/WCET + 超时计数 |
| 7 | 失败计数器，**全 0 才是健康** |

> `Pose` 周期显示 11.38 ms 而不是 10 ms：它被 IMU 任务里那次阻塞 I2C（≤1.5 ms）挤了。
> 设计使然，写进了报告的"不好看的数字"一节。

---

## 演示

- 一页纸：[`Docs/Block4.5_Part2_Demo_Flowcard_CN.docx`](Docs/Block4.5_Part2_Demo_Flowcard_CN.docx)
- 完整脚本（含 12 个问答）：[`Docs/Block4.5_Part2_Demo_Script_CN.docx`](Docs/Block4.5_Part2_Demo_Script_CN.docx)

---

## 踩过的坑（每个都会在重写时复现）

| # | 症状 | 根因 | 修法 |
|---|---|---|---|
| A | 打印 7 行后整车假死，但 TIM7 还在跳 | 栈溢出钩子设 BASEPRI=0x50，屏蔽不掉优先级 0 的 TIM7 | 栈 512→1024 / 1024→2048 |
| B | `G` 生成了路点但车不动 | `osEventFlagsWait` 无事件返回 `0xFFFFFFFD`，`&1 == 1` 被当成 STOP | 先判 `flags & 0x80000000` |
| C | `f` 时右轮倒转，车原地打转 | 电机在 PB6/PB7 上但装的是右轮，且极性反 | `board_pins.h` 左右对调 + `RIGHT_INVERT` |
| D | 速度只有目标的 28% | 积分限幅 120 × ki 0.15 = 2% 输出，积分等于禁用 | kp/ki/限幅一起改 |
| E | `G` 之后队列为空 | 测试脚本的 `S` 晚到，清掉了新路径 | 等命令送达再关串口 |
| F | 右轮正转 0.5×、反转 4.7× | 编码器 QEP 输入滤波 190 ns 太短，碳刷噪声被当边沿 | 滤波改 3 µs（`encoder.c`） |
| G | 1.4 m 的路 1.2 s 就判"完成" | `PATH: g/r` 是开机累计值，不是本趟进度 | 判据改为 `QP==0` + 位姿在目标附近 |

根因分析、定位过程与实测数据见报告 §11。

---

## 目录结构

```
Core/Src/app_rtos.c     8 线程、3 队列、1 Hz 状态行、角色 5 演示
Core/Src/app.c          启动顺序、NVIC 优先级表、HAL 回调分发
Core/Src/motor.c        TIM4/TIM2 PWM + 双轮 PID
Core/Src/encoder.c      TIM1/TIM8 硬件 QEP（3 µs 输入滤波）
Core/Src/pose.c         里程计 + 陀螺航向（陀螺优先，轮式兜底）
Core/Src/path.c         100 mm 路点生成 + 航向控制律
Core/Src/imu.c          MPU-6500，任务上下文采样
Core/Src/i2c_bus.c      I2C1 唯一所有者：prio-inherit 互斥量 + 有界事务
Core/Src/oled.c         SSD1306 + 6x8 字库（未上过真面板）
Core/Inc/board_pins.h   引脚/定时器/NVIC 唯一真源
tools/                  bench_p0 / nav_run / bt_send / flash_and_monitor / serial_console
tools/host_tests/       主机侧单元测试（解析语法、路点生成、跟踪律）
Docs/                   报告、演示脚本、编码规范、交接文档
```

---

## 文档

| 文档 | 内容 |
|---|---|
| `Docs/Block4.5_Part2_Design_Report_CN.docx` | **设计报告**：架构论证、实测、七个 bug |
| `Docs/Block4.5_Part2_Demo_Script_CN.docx` | 现场演示脚本 + 问答 |
| `Docs/Block4.5_Part2_Demo_Flowcard_CN.docx` | 一页纸 |
| `Docs/CODING_STANDARDS.md` | 铁律、任务表、队列策略、可观测性格式 |
| `Docs/HANDOVER.md` | 上板实测记录（bug 现场、调试技巧） |
| `Docs/NEXT_SESSION.md` | 交接：当前状态、待办、时间线 |

---

## 第三方组件

- FreeRTOS 10.3.1 + CMSIS-RTOS v2 封装（ST）
- STM32F4 HAL
- `pc_app/`：PC 侧点云查看器（Python + matplotlib，解二进制遥测）

课程示例代码与 PPT **不在本仓库内**，也未提交。
