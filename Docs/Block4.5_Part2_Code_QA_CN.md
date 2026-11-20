# Block 4.5 Part 2 —— 讲代码小抄

> 用法：**左边是问题，右边是"打开哪个文件、第几行、说哪一句"**。
> 行号按当前 HEAD（`91080` 那版）核对过；改代码后行号会漂，认函数名更稳。

---

## 0. 先把这四个位置收藏好

| 要讲的东西 | 文件 | 行 | 函数 |
|---|---|---|---|
| 信号量：ISR 侧 | `Core/Src/imu.c` | 212 | `HAL_TIM_PeriodElapsedCallback()` |
| 信号量：Release | `Core/Src/app_rtos.c` | 262 / 267 | `app_rtos_imu_tick()` |
| 信号量：Acquire | `Core/Src/app_rtos.c` | 504 | `imu_task()` |
| 互斥量：属性 | `Core/Src/i2c_bus.c` | 68 | `.attr_bits = osMutexPrioInherit` |
| 互斥量：拿锁/放锁 | `Core/Src/i2c_bus.c` | 104 / 138 | `i2c_bus_lock()` / `unlock()` |
| 互斥量：创建 | `Core/Src/i2c_bus.c` | 271 | `osMutexNew(&bus_mutex_attr)` |
| 继承演示：创建 | `Core/Src/app_rtos.c` | 1432 / 1433 | `osMutexNew` vs `osSemaphoreNew(1,1)` |

---

## 1. 角色 4：信号量（TIM6 → IMU 任务）

**代码链路，按这个顺序念：**

```
imu.c:212   TIM6 每 10 ms 进中断
imu.c:221   → app_rtos_imu_tick()
app_rtos.c:267  → osSemaphoreRelease(sem_imu)     ← ISR 里只做这一件事
app_rtos.c:504  → imuAcq 任务 osSemaphoreAcquire(sem_imu, 100 ms)
imu.c:240       → imu_sample_once()：这次 I2C 事务在【任务上下文】里做
```

**一句话版本：**
> "定时器中断里只 Release 一个信号量，采样和 I2C 读都在任务里做。实测 100/s，`iw0` 表示一次都没等超时。"

### 可能的追问

| 问 | 答 |
|---|---|
| 为什么不在中断里直接读 I2C？ | 一次事务最长 1.5 ms，中断里做会把所有低于它的中断推迟；HAL 的 I2C 是**阻塞式**实现；而中断里碰 RTOS 对象有优先级限制。规范做法就是"中断只通知"。 |
| 为什么用信号量而不是队列？ | 队列用来**传数据**，信号量用来**传事件**。IMU 的数据在芯片寄存器里，任务自己去读；中断要传达的只是"该采样了"这个节拍。 |
| 那蓝牙为什么也用信号量？ | 同理：字节在 DMA 环形缓冲里，ISR 只要说"有数据了"，`bt_rx` 任务自己去搬。 |
| 计数最大值为什么是 2 不是 1？ | 1 的话，任务晚一拍 Release 就直接失败（丢节拍）。2 允许一拍缓冲；**满了说明任务落后两拍**，`imu_ticks_unused` 会计数，不是静默丢弃。 |
| Acquire 为什么等 100 ms？ | 这是**看门狗**：10 个节拍都没来，说明 TIM6 停了，计 `iw`（`imu_wait_timeouts`）。正常运行永远不超时，所以 `iw0` 是健康指标。 |
| 中断里调 RTOS API 有什么前提？ | 该中断的 NVIC 优先级必须**数值 ≥** `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`（本工程 = 5）。`board_pins.h` §8 用 `_Static_assert` 把 TIM6/USART3/UART5 钉死在 5。 |

> 加分细节：`imu_ticks_unused` 和 `iw` 是**一对**——前者是"任务落后"，后者是"节拍缺失"。
> 昨晚那个 IMU 初始化 bug 就是靠 `iw` 每秒 +10 暴露出来的。

---

## 2. 角色 5：互斥量（I2C1 共享总线）

**代码链路：**

```
i2c_bus.c:68    .attr_bits = osMutexPrioInherit   ← 关键就这一行
i2c_bus.c:271   osMutexNew(&bus_mutex_attr)
i2c_bus.c:104   osMutexAcquire(bus_mutex, 10 ms)  ← 有超时，不是 osWaitForever
i2c_bus.c:138   osMutexRelease(bus_mutex)
```

**一句话版本：**
> "I2C1 上挂着 MPU-6500 和 OLED 两个从机，任何事务被打断都会出总线错误。所以 I2C1 只有一个所有者，
> 用带优先级继承的 mutex 保护，最长持锁 2.4 ms。"

### 可能的追问

| 问 | 答 |
|---|---|
| mutex 和二值信号量到底差在哪？ | 三点：**所有权**（mutex 有持有者，只有持有者能释放；二值信号量任何任务都能 release）、**优先级继承**（FreeRTOS 的 mutex 天生带，二值信号量没有）、**用途**（mutex 保护资源，信号量做同步）。 |
| FreeRTOS 有不带继承的 mutex 吗？ | **没有**。`xSemaphoreCreateMutex()` 继承是内建的，只有"普通 / 递归"之分。所以对照物只能是二值信号量——这正是 `i` 键演示的写法。 |
| 不用继承会怎样？具体走一遍。 | OLED(20) 持锁 → IMU(37) 等锁 → 中间优先级 22 的 btTx 就绪 → 抢占 OLED → IMU 实际上在等"btTx 跑完 + OLED 跑完"。用 mutex 时 OLED 被抬到 37，22 抢不动它，等待时间收敛到"OLED 剩下的临界区"。 |
| 演示那三行数字怎么读？ | 持锁者**在自己持锁期间读自己的优先级**：mutex 时是 13（等于等待者），信号量时还是 11。这是优先级继承的**定义本身**，不依赖任何调度时序假设。 |
| 拿锁为什么要超时？ | 防死锁：从设备把 SDA 拉死时，`osWaitForever` 会把整条链挂住。10 ms 后返回失败，连续 3 次失败走 `i2c_bus_recover()`（发 9 个时钟把总线放开）。 |
| 会不会嵌套加锁导致死锁？ | 不会。I2C1 只有在 `i2c_bus.c` 内部拿锁，而且**一次事务只拿一次**；所有访问都走 `i2c_bus_mem_read/write` 门面接口，没有第二处直接碰 I2C。 |
| 持锁 2.4 ms 怎么做到的？一帧 1 KiB 在 100 kHz 要 ~12 ms。 | 两级拆分：**分页**（8 页轮转，每 50 ms 只刷 1 页）+ **16 字节分块**（每次持锁只发 1 个控制字节 + 16 字节数据）。SSD1306 自己维护列指针，分块不破坏画面。 |
| 持锁时间是怎么测的？ | 拿锁时记 `DWT->CYCCNT`，放锁时算差取最大值，进 1 Hz 的 `hold` 字段。不是估算。 |

---

## 3. 通用代码追问（大概率会问）

| 问 | 答（一句话） |
|---|---|
| 为什么 8 个任务不是参考表的 10 个？ | 编码器是**硬件 QEP**，计数在外设里，没有交接就没有队列；PID 与电机合并，setpoint 变局部变量，零延迟——课件原文允许 "related 10 ms functions may share a coordinated task"。 |
| 优先级顺序怎么定的？ | 执行机构(40) > 传感(37) > 位姿(36) > 雷达(34) > 路径(30) > 蓝牙收(28) > 蓝牙发(22) > 显示(20)。控制环上任何延迟都直接变成轨迹误差，所以电机最高。 |
| 为什么 Pose(36) 低于 IMU(37)？ | Pose 读的是 IMU 的**最新样本**；IMU 优先级低会让位姿用到过期数据。 |
| 雷达为什么排在控制环之后？ | 雷达块的截止时间是 40 ms（DMA 覆盖周期），比 10 ms 环宽松一个数量级。 |
| 队列满了怎么办？ | 三个不一样：`Q_Lidar` 丢（DMA 会覆盖，丢是物理必然）；`Q_Cmd` 覆盖最旧（新命令才是意图）；`Q_Path` 生产方阻塞（丢路点车就停错地方）。三个都有计数器。 |
| 怎么证明不是"只调了 API"？ | 周期/抖动/WCET 是 DWT 运行时测的、`ov` 超时计数、`CNT:` 失败计数、`i` 键的 holder 优先级、60 秒链路字节率。每个机制都有一个**当场能读的数**。 |
| 你们最难的 bug 是哪个？ | 右轮编码器滤波 190 ns 太短，碳刷噪声被当成边沿——同一占空比下计数方向会翻转。改 3 µs 后 R/L 从 1.80/4.74 变 1.03/1.14。 |

---

## 4. 三个"不能答错"的点

1. **FreeRTOS 的 mutex 天生带继承** —— 别说"我们给它加了继承"，也别说"二值信号量也能继承"。
2. **中断里只做 Release，不做 I2C** —— 说出来，这是规范条款。
3. **I2C 只有一个所有者** —— 别说"两个任务各自调 `HAL_I2C_*`"，那等于说互斥量没起作用。

---

## 5. 现场翻不到代码时的兜底

打开 1 Hz 行，用**数字**回答，比翻代码快：

```
[1s] I2C: ok2516 err0 hold2.4ms | H: 17104/32768 | WD: 0 | IMU: 100/s
[1s] CNT: ql0 qc0 qpt0 imu0 bt(s0 r0 o0 u0) log0 iw0
```

> "信号量这条链的证据是 `IMU: 100/s` 加 `iw0`；互斥量这条链的证据是 `hold2.4ms` 加 `err0`，
> 再加上刚才 `i` 键打出的 holder 11→13。"
