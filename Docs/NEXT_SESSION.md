# 新对话交接文档 —— Block 4.5 Part 2 循迹小车（2026-11-18 起）

> 用途：把整个项目的上下文交给**零上下文的新对话**。
> 粘贴下面「给新对话的开场白」即可开始。需要细节时再看后面的章节。

---

## 0. 给新对话的开场白（直接复制这一段）

```
继续 Block 4.5 Part 2 循迹小车项目。

工程：D:\嵌入式小车\CodingDay_F446RE\ ，分支 part2，HEAD a42b803，工作区干净。
先读这三份文档再动手：
  Docs\HANDOVER.md    —— 尤其 §十二（2026-11-17 上板实测记录：5 个真 bug、实测数据、电池教训）
  Docs\CODING_STANDARDS.md —— 铁律、8 线程任务表、队列策略、可观测性格式
  Core\Inc\board_pins.h —— 引脚/定时器/NVIC 全部宏（已按实测更正过）

当前状态：8 个任务全部跑通，闭环导航空中验证 500mm→489mm（2.2% 误差）。
上一轮结束时卡在「驱动电池没电」，已充电，现在要接着做地面导航测试。

硬约束（不得违反）：
- 编译必须 0 warning / 0 error：make -C Debug -j8 all（基线 text 88016 B）
- 不许用改编译选项的方式压警告
- 不许 push
- 没有实测数据的，一律写「待实测」，不许编造数字
- 不许破坏现有 LiDAR 流水线和 s/g 控制台
- 每完成一个小步都要编译通过

下一步（按顺序）：
1. 确认电池有电（用 100% 占空比测，不要用 20%——弱电池下 20% 是死区）
2. 加电池电压监测（一路空闲 ADC → 打进 1 Hz 状态行）
3. 加堵转保护（占空比 >25% 且编码器 500ms 不动 → 停机 + 计数）
4. A/B 对照测试：抬起前轮 vs 放下，量横向偏移 + 航向偏差
5. 验证陀螺符号（只驱动左轮，航向应减小）
6. 处理右驱动通道的方向增益不对称（正转 0.5~0.64×，反转 4~5.5×）
7. 三次导航实测：(0,1.40) / (−0.70,0.70) / (0.70,−0.70) 米，记录端点/航向误差
```

---

## 1. 项目背景（一句话版）

STM32F446RE + FreeRTOS 的双轮差速小车。课程要求：蓝牙给目标坐标 → 路径生成 → 编码器/陀螺里程计 → PID 跟踪 → 走到点。
评分按 6 个角色各 1 分（完整应用 / 任务调度 / 消息队列 / 信号量 / 互斥量 / 蓝牙导航），
其中「Step2 与 Step3 未完成 = 全队 0 分」。

---

## 2. 工程环境（照抄可用）

| 项 | 值 |
|---|---|
| 工程路径 | `D:\嵌入式小车\CodingDay_F446RE\` |
| 分支 / HEAD | `part2` / `a42b803` |
| 编译 | `make -C Debug -j8 all`，必须 **0 warning / 0 error**，基线 **text 88016 B** |
| 课程示例（**有版权，禁止提交**） | `D:\嵌入式小车\course_materials\EBU6470_Codes_F446` |
| Coding Day 2 原始代码 | `D:\stm32codespace\lessonProgram`（**注意：那是另一块自定义板，引脚假设不适用于现在这块**） |

**`make` 不在 PATH 里**，每次开新 shell 要先加：

```powershell
$env:PATH = "D:\CubeIDE\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.100.202601091506\tools\bin;D:\CubeIDE\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin;" + $env:PATH
```

常用工具：

```cmd
tools\flash_and_monitor.cmd              :: 探测→烧录→校验→抓 15 s 串口→自动判读
tools\flash_and_monitor.cmd -Probe       :: 只探测（区分"没插 ST-Link"和"板子没电"）
tools\flash_and_monitor.cmd -NoFlash     :: 只抓串口
tools\serial_console.cmd                 :: 实时终端（可敲 s/g/f/b/l/r/x/c/e/h）
tools\host_tests\run_tests.cmd           :: 主机侧单元测试（125 条断言）
```

---

## 3. 硬件真相（2026-11-17 实测，**以本表为准**）

| 项目 | 实测结果 |
|---|---|
| 主板 | **NUCLEO-F446RE + Arduino 形态扩展板**（不是 lessonProgram 那块自定义板） |
| 右轮电机 | **PB6 / PB7**（TIM4_CH1/CH2），**极性反**，靠 `BOARD_MOTOR_RIGHT_INVERT=1` 补偿 |
| 左轮电机 | **PA0 / PA1**（TIM2_CH1/CH2）= 驱动板 **BIN1 / BIN2** |
| 右编码器 | PA8 / PA9（TIM1） |
| 左编码器 | PC6 / PC7（TIM8） |
| IMU | PB8 / PB9（I2C1，地址 0x68） |
| 调试串口 | USART2 = **COM8**（ST-Link 虚拟串口）@115200 |
| 蓝牙 | UART5 = **COM10**（SPP）@115200，**上下行都验证通过** |
| 激光 | USART3 @460800 + DMA1_Stream1 |
| OLED | **没接**（`oled=-3`），固件能缺席运行 |
| 电池 | **弱电池会让 20% 占空比进入死区**（轮子完全不转），必须用 100% 测 |

**⚠️ 不要再换电机动力线的极性**，固件已经补偿过，换线会双重取反。

---

## 4. 软件架构（8 线程 / 3 队列，细节见 CODING_STANDARDS §1.2）

| # | 线程 | 名字 | 优先级 | 栈(B) | 周期/触发 | 调度 |
|---|---|---|---|---|---|---|
| 1 | PID+Motor | `pidMotor` | 40 | 1024 | 10 ms | `osDelayUntil` |
| 2 | IMU | `imuAcq` | 37 | 1024 | 10 ms | 计数信号量(TIM6) |
| 3 | Pose | `pose` | 36 | 768 | 10 ms | `osDelayUntil` |
| 4 | LiDAR | `lidarProc` | 34 | 1536 | ~22 ms/块 | `Q_Lidar`(100ms 守卫) |
| 5 | PathGen | `pathGen` | 30 | 1024 | 命令到达 | `Q_Cmd` |
| 6 | BT RX | `btRx` | 28 | 768 | 字节到达 | 计数信号量 |
| 7 | BT TX | `btTx` | 22 | 1024 | 50 ms | `osDelayUntil` |
| 8 | OLED | `oled` | 20 | 2048 | 50 ms | `osDelayUntil` |

- 队列：`Q_Lidar` 8×12B（满则丢）、`Q_Cmd` 4×8B（满则覆盖最旧）、`Q_Path` 16×8B（生产方阻塞 50ms）
- **没有 Q_Enc**（硬件 QEP）、**没有 Q_Set**（PID 与电机合并）
- STOP/NEW_PATH 用**事件标志** `evt_nav`，不是队列
- 数据流：`BT → Q_Cmd → PathGen → Q_Path → PID+Motor → TIM2/TIM4 → 电机`
  　　　　`MPU6500 → imu.c(陀螺积分) → pose.c → path.c(航向误差→差速)`

---

## 5. 已验证（实测数据，可直接引用）

```
[1s] L: blk25 q0/8 p2428 | M: per10.00 jit42 wc13 | C: q0/4
[1s] P: eh90.0 ex0 ey489 | QP: 0 | PATH: g10 r7
[1s] I2C: ok1101 err1 hold1.5ms | H: 19816/32768 | WD: 0 | IMU: 100/s
[1s] STK: 680 504 464 1264 768 576 636 1096
[1s] T10: m10.00/42/13 p11.38/1384/12 | ov m0 p0
[1s] T50: b51.26/1266/281 o51.10/1106/1417 | ov b0 o0
[1s] CNT: ql0 qc0 qpt0 imu0 bt(s0 r0 o0 u0) log0 iw0
```

| 指标 | 实测 |
|---|---|
| 8 任务调度 | 全部运行，超时计数 0 |
| 电机任务 | 周期 10.00 ms / 抖动 42 µs / WCET 13 µs |
| IMU 采样率 | **100/s**（TIM6 = 100 Hz 契约达成） |
| I2C 最长持锁 | **1.5 ms**（目标 <3 ms） |
| 激光流水线 | 25 块/s，队列不积压 |
| 堆 / 栈剩余 | 19816/32768；8 个任务栈均 >30% |
| 蓝牙遥测 | 10–11 Hz，736 B/帧 |
| **闭环导航（空中）** | 指令 500 mm → **489 mm**（**2.2%**），x 零漂移，航向不变 |
| 里程标定 | 推 40 cm → `mm_per_count=0.1561`（假设 0.1547，差 0.9%） |

---

## 6. 五个已修的坑（**重写必复现，务必先读**）

| # | 坑 | 症状 | 根因 | 修法 |
|---|---|---|---|---|
| A | **任务栈溢出 → 整车假死** | 打印 7 个横幅后全停，但 TIM7 还在跳 | FreeRTOS 溢出钩子设的是 **BASEPRI(0x50)**，屏蔽 ≥5 的中断却屏蔽不掉优先级 0 的 TIM7 | IMU 栈 512→1024、OLED 1024→2048 |
| B | **`osEventFlagsWait` 返回值当标志位用** | `G` 生成路点但车不动，CCR=0 | 无事件时返回 `osErrorResource=-3=0xFFFFFFFD`，`&0x01=1` → 每周期误判 STOP | 先判 `(flags & 0x80000000)` 再按位与 |
| C | **左右轮映射反了** | `f` 正转时右轮倒转 | 唯一接着的电机在 PB6/PB7 上，但装的是右轮 | `board_pins.h` §3 两组对调 |
| D | **PID 积分被限死** | 只能跑到目标速度的 28% | `INTEGRAL_LIMIT=120 × ki=0.15` = 2% 输出，积分等于禁用 | kp 3→5、ki 0.15→0.5、限幅 120→4000、PWM 上限 325→840 |
| E | **测试脚本的晚到 STOP** | `G` 之后队列为空 | 上一条 `S` 在关串口后才送达 | 发完命令等送达再关串口 |

**排查心法**：
- "TIM7 在跳但任务全不动" → 查 **BASEPRI** 和栈溢出钩子，别怀疑调度器
- 任何 CMSIS-RTOS2 返回值 → 先按 `osOK` 语义判断，再做位运算
- 车不动 → 先读 **CCR 寄存器**确认固件有没有在输出，再读 **编码器**，最后才怀疑硬件

---

## 7. 待办清单（按优先级，明天／下次直接照做）

### P0 · 先决条件
- [ ] **确认电池有电**：架起车，用 **100% 占空比**测（20% 在弱电池下是死区）。
      方法：直接写 `TIM2->CCR1 = 840`（地址 `0x40000034`）看编码器变化，然后写回 0。
- [ ] 确认板子在线：`tools\flash_and_monitor.cmd -Probe` 应报 `Target OK (voltage ~3.24 V)`

### P1 · 两个保护（建议先写，再烧录一次）
- [ ] **电池电压监测**：一路空闲 ADC（建议 A5=PC0 或 A4=PC1）量电池分压，
      把电压打进 1 Hz 状态行（比如 `B: 7.8V`）。理由：上一轮就是吃了没电压读数的亏。
- [ ] **堵转保护**：占空比 >25% 且编码器 500 ms 不动 → 停机 + 计数（`CNT: stall1`）。
      副作用能保护驱动器，并把"没电/卡住"在日志里直接暴露。

### P2 · 地面测试
- [ ] **A/B 对照：抬起前轮 vs 放下**
      地面拉 1 m 直线胶带 → 车头压线 → `c` 清零 → `G 0 1000` → 量**横向偏移**和 `eh` 航向偏差。
      两次对比就能判断"画弧"里前轮拖拽占多少。**别用手扶，用垫块撑到前轮离地 2-3 mm。**
- [ ] **验证陀螺符号**：只驱动左轮（`l`），地面俯视应**顺时针**转、`eh` 应**减小**。
      如果增大 → 陀螺反向，航向环会正反馈（表现为车持续朝一个方向转）。
- [ ] **右驱动通道方向增益不对称**：正转 0.5~0.64×、反转 4~5.5×。
      怀疑 A 通道某根信号线串了电阻/接触不良，需查线或用 `MOTOR_RIGHT_SCALE` 做增益补偿。

### P3 · 验收
- [ ] 三次导航实测：**(0, 1.40)** / **(−0.70, 0.70)** / **(0.70, −0.70)** 米
- [ ] 记录每次的**端点误差**和**航向误差**
- [ ] 连续跑 10 分钟不复位
- [ ] 重写报告 / README / 演示材料（现在还是旧任务的）

### 未做（明确记录，别当成已完成）
- [ ] OLED 驱动已写好但**面板从未接过**（`oled=-3`），屏上显示从未见过
- [ ] 蓝牙下行回复（`A G x y` / `E code text`）**未接线**，`?`/`P` 命令没有回话
- [ ] IWDG 未启用（1 Hz 行里 `WD: 0` 是刻意的 0，不是漏了）
- [ ] 三次导航实测一次都没做

---

## 8. 关键文件地图

| 文件 | 作用 |
|---|---|
| `Core/Src/app_rtos.c` | 8 个任务、3 队列、1 Hz 状态行、任务计时、**调试计数器** |
| `Core/Src/app.c` | 启动顺序、NVIC 优先级表、HAL 回调分发、非阻塞 `dbg_printf` |
| `Core/Src/motor.c` | TIM4/TIM2 PWM + 双轮 PID（**参数已按实测改过**） |
| `Core/Src/encoder.c` | TIM1/TIM8 硬件 QEP，1320 counts/rev |
| `Core/Src/imu.c` | MPU-6500，**任务上下文**采样（不再从中断读 I2C） |
| `Core/Src/pose.c` | 里程计 + 陀螺航向（陀螺优先，轮式兜底） |
| `Core/Src/path.c` | 路点生成 + 航向控制律（算差速） |
| `Core/Src/i2c_bus.c` | I2C1 唯一所有者：`osMutexPrioInherit` + 有界事务 + 持锁计时 |
| `Core/Src/oled.c` + `oled_font.c` | SSD1306 驱动 + 6x8 字库（未上过真面板） |
| `Core/Src/bt_rx.c` / `bt_protocol.c` | UART5 IDLE 行接收 + 命令解析 |
| `Core/Src/log.c` | USART2 非阻塞日志（环形缓冲 + DMA） |
| `Core/Inc/board_pins.h` | **引脚/定时器/NVIC 唯一真源**（已按实测更正） |
| `Docs/HANDOVER.md` §十二 | 上一轮上板实测的完整记录 |
| `tools/host_tests/` | 主机侧单元测试（125 条断言，跑 `run_tests.cmd`） |

---

## 9. 串口控制台按键（USART2 = COM8，115200）

| 键 | 作用 |
|---|---|
| `s` / `g` | 停 / 重启激光（旧功能，**不许破坏**） |
| `f` / `b` | 两轮正转 / 反转 20%，1.5 s 后自动停 |
| `l` / `r` | 只驱动左 / 右通道 20%，1.5 s |
| `x` | 立即停 |
| `c` | 位姿 + 两路编码器清零（由 Pose 任务执行，单所有者） |
| `e` | 读编码器/位姿：`L=.. R=.. cnt \| rpm L/R \| x y \| h gyro`（角度 ×0.1） |
| `h` / `?` | 帮助 |

**注意**：这些点动都只有 20% 占空比，**弱电池下会表现为"完全不转"**，别据此判断硬件生死。

---

## 10. 调试技巧（省时间，上一轮靠这些才定位到问题）

```powershell
# 1) 取静态变量地址（不需要目标板在线）
arm-none-eabi-gdb -batch -ex "p/x &'motor.c'::s_motor" Debug\CodingDay_F446RE.elf

# 2) 读运行中的内存（mode=hotplug 不复位；mode=UR 会复位！）
STM32_Programmer_CLI -c port=SWD mode=hotplug -r32 0x20002300 32

# 3) 直接写寄存器（绕过固件，用来测硬件；写完记得写回 0）
STM32_Programmer_CLI -c port=SWD mode=hotplug -w32 0x40000034 0x348
```

**重要陷阱**：
- **插上 USB 会复位目标板**（ST-Link 拉 NRST）→ 无线跑完再插 USB 取数据会**丢位姿**。
  要么跑的时候插着线，要么把数据存 `.noinit` 段。
- 判断 MCU 是否真在输出 PWM：读 `TIMx->CR1`(CEN) / `CCER`(CCxE) / `CCR` / `CNT`
  和 `GPIOx->MODER/AFR`，比肉眼看轮子可靠。
- `Debug/Core/Src/subdir.mk` 和 `Debug/objects.list` 是 CubeIDE 生成的产物（`Debug/` 已 gitignore）。
  **新增 .c 文件后这两个文件必须同步更新，否则会出现"编译出 .o 但没链进去"**（text 一点不涨）。
  在 IDE 里刷新/编译一次会自动重生成。

---

## 11. 铁律摘要（完整版见 CODING_STANDARDS.md）

1. PA13/PA14（SWD）永不复用；绝不写 Option Bytes、绝不启用 RDP
2. 任务里禁止任何阻塞（`HAL_Delay`、无超时 I2C、忙等）
3. 每个资源一个所有者；跨任务数据只走队列/信号量/互斥量
4. ISR 只做通知（<5 µs）；调用 RTOS API 的中断优先级必须 ≥5
5. **TIM7 的优先级必须为 0，且开机 DWT 时基校准不许删**
6. 每个 I2C 事务必须有界（≤10 ms，重试 ≤2 次，连续 3 次失败要恢复）
7. 1 Hz 状态行的所有字段必须是真实测量值，不是常量
8. `course_materials/` 和课程 PPT/PDF **禁止提交**
