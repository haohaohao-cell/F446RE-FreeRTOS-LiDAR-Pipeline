# F446RE FreeRTOS 激光雷达流水线

Block 4.5 Coding Day —— 基于 **STM32F446RE + FreeRTOS 10.3.1（CMSIS-RTOS v2）**
的中断驱动 RPLidar C1 数据流水线。

DMA 把循环缓冲对半填，中断只负责宣告哪一半就绪，解析任务直接从 DMA 缓冲区读这一块
解析，第二个任务负责打印/发送。**中断里不做任何解析。**

*English version: [README.md](README.md).*

---

## 架构

```
        USART3 + DMA1_Stream1（循环，2048 B = 2 × 1024 B）
                         |
     半满中断  -> lidar_on_half()      --+
     全满中断  -> lidar_on_full()      --+--> Queue 1（8 × 12 B 描述符）
     串口错误  -> lidar_on_uart_error()+             |
                                                     | osMessageQueueGet()
                                                     v
                                      LiDAR 解析任务（osPriorityAboveNormal）
                                                     |
                                   +-----------------+------------------+
                                   |                                    |
                      360 格扫描图（共享）                  Queue 2（4 × 44 B 结果）
                      ≈8 Hz 遥测帧                                      |
                                                                        v
                                                    输出/调试任务（osPriorityNormal）
                                                           |                     |
                                                  USART2 文本 115200     UART5 二进制 230400
                                                           |                     |
                                                  ST-Link 虚拟串口(COM8)    HC-04 蓝牙(COM10)
```

---

## 实测数据（真实硬件）

| 指标 | 数值 | 看哪个字段 |
|---|---|---|
| DMA 块速率 | **25–26 块/秒**（半缓冲 40 ms） | `blk` |
| Queue 1 峰值占用 | **0 / 8** | `q1` —— 解析任务从没落后 |
| 中断 → 任务延迟 | **12–15 µs** | `lat` / `latmax` |
| 每块解析耗时 | **约 800 µs**（`-O0`）= 块周期的 2% | `parsemax` |
| 雷达字节率 | **24.9–25.3 kB/s**，约 5 100 包/秒 | `L`，与 C1 手册吻合 |
| 帧错误 / 重同步 / 串口错误 | **0 / 0 / 0**（约 100 万包） | `inv` / `slip` / `err` |
| FreeRTOS 堆 | 32 KB 用 6.2 KB，`free == min`（无泄漏） | `heap` |
| 任务栈剩余 | 解析 1256 B / 输出 784 B | `stk` |
| HAL tick 与内核 tick | **1.000 : 1** | `hal` / `k` |

完整分析（架构论证、块大小选择、队列消息设计、中断优先级规则、三个"只有上板才会
暴露"的 bug）见 [`Docs/Block4.5_Design_Report_CN.docx`](Docs/Block4.5_Design_Report_CN.docx)。

---

## 硬件

| 项目 | 说明 |
|---|---|
| MCU | STM32F446RE（兼容 Nucleo-F446RE），168 MHz，128 kB SRAM |
| 雷达 | RPLidar C1，SCAN 模式，USART3 @ **460 800** 8N1（PC10 TX / PC11 RX） |
| 雷达 DMA | DMA1_Stream1，循环模式，2048 B |
| IMU（可选） | MPU-6500，I2C1（PB8 SCL / PB9 SDA），TIM6 以 100 Hz 采样 |
| 蓝牙（可选） | HC-04 接 UART5 @ 230 400，承载二进制遥测 |
| 调试串口 | USART2 @ 115 200（PA2 / PA3）→ ST-Link 虚拟串口 |
| HAL 时基 | TIM7 @ 1 kHz（TIM6 已被 IMU 占用） |

雷达这条路可以单独工作；IMU 和蓝牙是可选的，没接只会表现为 `IMU 0` / 没有点云数据。

---

## 目录结构

```
Core/               应用代码
  Src/app.c           启动顺序、NVIC 优先级表、HAL 回调
  Src/app_rtos.c      队列、解析任务、输出任务、控制台命令
  Src/lidar.c         C1 驱动：中断投递描述符、逐块解析、流看门狗
  Src/stm32f4xx_hal_timebase_tim.c   HAL tick 移到 TIM7 + DWT 校准
  Inc/FreeRTOSConfig.h               内核配置
  Inc/app_rtos.h, lidar.h            队列/任务尺寸、消息结构体
Middlewares/Third_Party/FreeRTOS/    FreeRTOS 10.3.1 + CMSIS-RTOS v2 + heap_4
Drivers/                             ST HAL + CMSIS（CubeF4 1.28.3）
pc_app/             PC 端
  main.py             matplotlib 点云查看器（解二进制遥测）
  run_viewer.cmd      一键启动
tools/              辅助脚本
  flash_and_monitor.cmd   探测 → 烧录 → 校验 → 抓串口 → 自动判读
  serial_console.cmd      实时终端，可敲 s / g / h
Docs/               报告 + 演示材料（中英双份，.md 和 .docx）
CodingDay_F446RE.ioc, .cproject     STM32CubeIDE 工程文件
```

---

## 快速开始

### 1. 克隆

```bash
git clone https://github.com/haohaohao-cell/F446RE-FreeRTOS-LiDAR-Pipeline.git
```

### 2. 编译

**用 STM32CubeIDE（推荐）**

1. *File → Import → General → Existing Projects into Workspace*
2. 选克隆下来的 `F446RE-FreeRTOS-LiDAR-Pipeline` 目录，工程 `CodingDay_F446RE` 会出现
3. **Project → Build Project**（Ctrl+B）

预期：**0 warning、0 error**，约 55 kB Flash、42 kB RAM。

> FreeRTOS 的 include 路径已经写进 `.cproject`，不需要再配置任何东西。
> **不要**打开 `.ioc` 按 Generate Code —— 见下面「坑」。

**命令行**（CubeIDE 工具链已在 PATH 上时）

```bash
make -C Debug -j8 all
```

### 3. 烧录

```cmd
tools\flash_and_monitor.cmd
```

它会先确认 ST-Link 能看到目标板，然后烧 `Debug\CodingDay_F446RE.elf`、校验、复位，
再抓 15 秒调试串口并自动给健康判定。

* 只想看板子通不通：`tools\flash_and_monitor.cmd -Probe`
* 工具链装在别处：`-Cli "C:\路径\STM32_Programmer_CLI.exe"`
* 不烧只监听：`-NoFlash -Seconds 30`

### 4. 看它跑 —— 两条互相独立的链路

| 看什么 | 命令 | 端口 |
|---|---|---|
| **文本终端**（`blk`、`idle`、`q1`、`lat`、`heap`、`stk`） | `tools\serial_console.cmd` | COM8 @ 115200 |
| **点云图**（二维扫描 + 航向盘） | `pc_app\run_viewer.cmd` | COM10 @ 115200（蓝牙） |

两者可以同时开 —— 它们是不同的物理链路。

终端是**可交互**的：

| 按键 | 作用 |
|---|---|
| `s` | 停止雷达（不用碰任何线） |
| `g` | 重新启动 |
| `h` / `?` | 帮助 |
| `Esc` / `Ctrl+C` | 退出 |

点云查看器需要 `matplotlib`、`pyserial`、`tkinter`：

```bash
py -3 -m pip install matplotlib pyserial
```

端口不是 COM10 的话，用 `pc_app\run_viewer.cmd -l` 列出来，或者
`run_viewer.cmd COM7` 指定。`run_viewer.cmd --sim` 可以用合成数据跑，
**完全不需要硬件**。

---

## 输出怎么读

**每块一行**（`#`，约 8 Hz）

```
#0474 p205 ok151 nr54 iv0 n115 d[47..10574] lat12 p765 S 255 78 36 8 4 255 255 255
```

| 字段 | 含义 |
|---|---|
| `#0474` | 块序号 —— 出现跳变说明丢了描述符 |
| `p205` | 本块解出的 5 字节包数 |
| `ok151 / nr54 / iv0` | 有效 / 无回波 / 帧错误 |
| `n115` | 本块触及的不同角度格数 |
| `d[47..10574]` | 本块最小 / 最大距离，mm |
| `lat12` | **中断 → 解析任务唤醒延迟**，µs |
| `p765` | 本块解析耗时，µs |
| `S ...` | 8 扇区障碍摘要，cm（255 = 无回波） |

**每秒一行**（`[1s]`）

```
[1s] blk 25/447 q1 0/8 q2 1/4 idle 5 latmax 14 parsemax 801 | L 25259B/s
     pkt 91541 inv 0 nr 17495 sync 1 slip 0 err 0 cmd 1/0 | IMU 0 err 0
     | BT f 161 dr 0 | heap 26488 min 26488 | stk 1256/784 | hal 1054 k 1053
```

| 字段 | 含义 | 正常值 |
|---|---|---|
| `blk` | 最近一秒块数 / 累计块数 | 约 25 /秒 |
| `q1` | Queue 1 **峰值**占用 | 0–1（到 8 说明解析跟不上了） |
| `q2` | Queue 2 峰值占用 | 0–2 |
| `idle` | Queue 1 等待超时次数 | 很小且稳定不变 |
| `latmax` | 最坏中断→任务延迟 | < 50 µs |
| `parsemax` | 最坏解析耗时 | < 1 000 µs |
| `L` | 实测雷达字节率 | 约 25 000 B/s |
| `sync` | 抓到 SCAN 描述符的次数 | 1（重新锁定流时会增加） |
| `cmd` | 已发 SCAN 命令数 / 看门狗判定丢流次数 | `1 / 0`（只有传感器掉线才会涨） |
| `err` | 串口错误 | 0 |
| `heap` | 堆剩余 / 历史最低剩余 | 两者相等（无泄漏） |
| `stk` | 栈高水位**剩余**，解析 / 输出 | 都 > 500 B |
| `hal` / `k` | 同一秒内 HAL tick 与内核 tick 的增量 | 1:1 |

---

## 演示「掉线又恢复」（最关键的 60 秒）

这一段用来证明设计是**队列驱动**而不是轮询。

1. 在终端窗口敲 **`s`**
   * `blk` → 0，`L` → 0 B/s，`idle` 开始涨（每 100 ms 一次超时）
   * **系统不卡死**：两个任务都还响应，1 Hz 行照常打印
2. 等 5 秒，敲 **`g`**
   * `cmd` 会跳（`3/0 → 4/1 → 5/2`）：看门狗发现超过 1 秒没有 DMA 块，重发了 SCAN
   * 约 3 秒内 `blk` 回到 25、`idle` 停住、`sync` 从 1 变 2

为什么重要：RPLidar C1 **上电后不会自己开始输出**，它永远需要再收到一次
`0xA5 0x20` SCAN 命令。如果用 `osWaitForever` 死等队列，这里就永久死锁了。
100 ms 的保护超时正是让解析任务能跑起来跑看门狗的原因。

拔雷达线也能演示同样的事，但敲 `s` 不会弄坏硬件 —— 现场演示建议用这个。

---

## 坑

* **两个不同的 COM 口。** COM8 是 ST-Link 虚拟串口 = 文本调试（USART2）；
  COM10 是蓝牙 SPP = 二进制遥测（UART5）。`blk` / `idle` **只在 COM8** 上；
  点云窗口只显示二进制流。
* **Windows 下蓝牙 SPP 串口最高只能开 115200。** 固件实际以 230 400 发送，
  但 SPP 的波特率只是名义值 —— 查看器用 115200 打开，不会被限速
  （实测遥测吞吐约 6.4 kB/s）。
* **HAL 时基在启动时用 DWT 校准。** `stm32f4xx_hal_timebase_tim.c` 实测发现本板
  TIM7 跑在 HCLK（168 MHz）而不是手册推算的 2 × PCLK1（84 MHz），于是改为按
  `实测/预期` 修正预分频。开机横幅会打印 `timebase: ... calib=40/20` 和
  `50 ms (dwt) -> 50 ms (hal) OK`。**如果 tick 不对，所有 `HAL_GetTick()` 超时和
  所有测出来的字节率都会按同一比例出错** —— 先看这一行。
* **不要在 STM32CubeMX 里按 Generate Code。** `.ioc` 里没有 FreeRTOS 的 IP
  （为了避免写坏工程而刻意没有注入），重新生成会重写 `.cproject`，把 FreeRTOS 的
  include 路径和 `Middlewares` 源目录冲掉。真要重新生成，必须把这两样、以及 TIM7
  优先级 0 和启动校准加回去。
* **IMU 是可选的。** 横幅里的 `imu=-1, err=0x4, who=0x00` 和 1 Hz 行的 `IMU 0`
  只表示 MPU-6500 没接，不影响雷达流水线。

---

## 文档

| 文件 | 内容 |
|---|---|
| `Docs/Block4.5_Design_Report_CN.docx` / `_EN.docx` | 完整分析：架构、块大小论证、队列消息设计、中断优先级规则、实测结果、三个上板才发现的 bug |
| `Docs/Block4.5_Demo_Flowcard_CN.docx` / `_EN.docx` | 一页演示卡：3 个动作 + 4 人分工表 |
| `Docs/Block4.5_Demo_Script_CN.docx` / `_EN.docx` | 详细演示脚本 + 答辩问答速查 |
| `Docs/*.md` | 上面这些的 Markdown 源文件 |

`tools/md2docx.py` 可以从 Markdown 重新生成 `.docx`（需要 `python-docx`）：

```bash
py -3 tools/md2docx.py Docs/Block4.5_Design_Report_CN.md Docs/out.docx --lang cn
```

---

## 第三方组件

| 组件 | 版本 | 许可证 |
|---|---|---|
| FreeRTOS 内核 | 10.3.1 | MIT（见 `Middlewares/Third_Party/FreeRTOS/LICENSE`） |
| CMSIS-RTOS API v2 封装 | 随 CubeF4 1.28.3 | Apache-2.0 |
| STM32F4 HAL / CMSIS | CubeF4 1.28.3 | BSD-3-Clause（ST） |

`Core/`、`pc_app/`、`tools/` 里的代码是课程作业，**只作参考**，不要直接当自己的作业交。
