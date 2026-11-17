# Block 4.5 Coding Day — Part 1, Step 1

## Run and Experiment with the supplied CMSIS-RTOS2 codes

> 记录对象：`D:\嵌入式小车\course_materials\EBU6470_Codes_F446` 里的 W4.x 示例工程
> （同一颗 STM32F446RET6）。
>
> **说明**：本文档基于**示例工程的实际源码**逐行核对写成（不是凭记忆）。
> 撰写时开发板未连接（`No debug probe detected`），所以"真机观察"部分留成 §6 的勾选表，
> 插上板子跑一遍即可补齐。所有"预期行为"都能由源码 + CMSIS-RTOS2 语义严格推出。

---

## 1. 环境与输出方式（先搞清楚怎么看结果）

| 项 | 示例工程的做法 | 影响 |
|---|---|---|
| 输出通道 | `Task_action()` 内部是 `ITM_SendChar()` | 必须用 **SWO/SWV + ITM Data Console** 才能看到输出，**不是**串口！ |
| 时钟 | PLLN=360 + OverDrive → **180 MHz** | 我们自己的工程是 **168 MHz**，别照抄 |
| HAL 时基 | **TIM6**（`.ioc` 里 `NVIC.TimeBase=TIM6_DAC_IRQn`） | 我们被迫用 TIM7，因为 TIM6 被 IMU 占了 |
| 内核 tick | `configTICK_RATE_HZ = 1000` | 与我们一致 |
| 优先级上限 | `configMAX_PRIORITIES = 56` | 与我们一致（CMSIS-RTOS2 的 `osPriority_t` 要求） |
| 系统调用中断上限 | `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5` | 与我们**完全一致** |
| 堆 | `configTOTAL_HEAP_SIZE = 15360`（15 KB） | 我们用 32 KB |

**怎么看输出（三步）**：Debug 配置里选 SWD → Quick Access 搜 `SWV` → 打开 *SWV ITM Data Console* →
勾上 **Port 0** → Start Trace → 运行。

---

## 2. 实验一 · 任务与调度（`W4.1_Tasks`）

### 2.1 实际配置

```c
Task1Handle = osThreadNew(StartTask1, NULL, &Task1_attributes);  /* Normal, stack_size = 128 */
/* Task2 没有在 main 里创建，而是被注释掉，改由 Task1 在运行时创建 */
//Task2Handle = osThreadNew(StartTask2, NULL, &Task2_attributes);
```

```c
void StartTask1(void *argument) {
    const osThreadAttr_t Task2_attributes = {
        .name = "Task2", .priority = osPriorityNormal, .stack_size = 128 };
    for (;;) {
        Task_action('1');                     /* 打印 '1'          */
        Task2Handle = osThreadNew(StartTask2, NULL, &Task2_attributes);
        HAL_Delay(1000);                      /* 注意：任务里用 HAL_Delay */
    }
}

void StartTask2(void *argument) {
    for (;;) {
        Task_action('2');                     /* 打印 '2'          */
        osThreadTerminate(Task2Handle);       /* 自杀               */
        Task_action('x');                     /* 永远不执行         */
    }
}
```

### 2.2 可以严格推出的预期现象

| 观察点 | 预期 | 原因 |
|---|---|---|
| 输出顺序 | 每秒 `1` `2` 交替出现 | Task1 打印后创建 Task2（同优先级），Task2 立即被调度 |
| `'x'` **永远不会出现** | 看不到 `x` | `osThreadTerminate()` 之后该任务再不运行 |
| 每秒都出现一个新的 `2` | 是 | Task2 每轮都被重新创建，不是复用 |
| 只有 1 个 Task2 实例 | 是 | 上一轮的 Task2 已被 terminate |

### 2.3 这个实验真正要说明的三件事

1. **`osThreadNew()` 可以在任何上下文调用**（包括任务内部），不必在 `main()` 里 —— 但**不能在中断里调用**
2. **动态创建/删除任务要小心**：`osThreadTerminate()` 释放的栈和 TCB 要有人回收（FreeRTOS 靠空闲任务），
   频繁创建删除会拖慢系统 → **Part 2 里所有任务一律在 init 阶段一次性创建**
3. **`HAL_Delay()` 放在任务里是坏示范**：它忙等，这一秒里 CPU 空转（只是被同优先级任务分时复用）。
   正确做法是 `osDelay(1000)`

### 2.4 ⚠️ 一个真实的坑：`stack_size` 的单位

`osThreadAttr_t.stack_size` 在 CMSIS-RTOS2 里是**字节**，不是字。
`W4.1_Tasks` 写的是 `stack_size = 128` → **只有 128 字节 = 32 个字**，非常危险（能跑只是因为任务体几乎不用栈）。
`W4.3.1` 写的是 `256 * 4` = 1024 字节 —— 同一套示例里两种写法混用，容易误导。

> **我们的做法**：`APP_PARSER_STACK = 1536`、`APP_OUTPUT_STACK = 2048`（字节），
> 实测剩余 1256 B / 784 B，留了 30% 以上余量。

---

## 3. 实验二 · 消息队列（`W4.2.1` / `W4.2.2` / `W4.2.3`）

三个递进的例子：基本队列 → 两个发送者 → 传结构体。这里记最完整的 `W4.2.3_Queues_items`。

### 3.1 实际配置

```c
typedef struct { char Value; /* ... */ } Data;
osMessageQueueId_t Queue1Handle;
const osMessageQueueAttr_t Queue1_attributes = { .name = "Queue1" };

Queue1Handle = osMessageQueueNew(8, sizeof(Data), &Queue1_attributes);   /* 深度 8，消息 = Data */
```

```c
void StartSender1(void *argument) {
    for (;;) { Task_action('s');
               osMessageQueuePut(Queue1Handle, &DataToSend1, 0, 200);    /* timeout = 200 */
               osDelay(2000); }
}
void StartSender2(void *argument) {
    for (;;) { Task_action('S');
               osMessageQueuePut(Queue1Handle, &DataToSend2, 0, 200);
               osDelay(2000); }
}
void StartReceiver(void *argument) {
    for (;;) { osMessageQueueGet(Queue1Handle, &retvalue, NULL, osWaitForever);  /* 永久阻塞 */
               Task_action(retvalue.Value); }
}
```

### 3.2 预期现象与要点

| 观察点 | 预期 | 说明 |
|---|---|---|
| 接收者平时状态 | **BLOCKED** | `osWaitForever`，队列空时永久阻塞，**不消耗 CPU** |
| 发送后 | 立即被唤醒 | 队列由空变非空 → 接收者转 READY；若其优先级更高则立刻抢占（PendSV） |
| 输出节奏 | 每 2 s 出现 `s` / `S` 交替 | 两个发送者同周期、同优先级，时间片轮转 |
| 能收到的是结构体 | 收到 `Value` 字段 | CMSIS-RTOS2 的队列**按值拷贝整个结构体**，不是传指针 |

### 3.3 三个容易被忽略的语义

1. **`osMessageQueuePut()` 的 `msg_prio` 参数被忽略**
   ST 的 `cmsis_os2.c` 里明确写着 `(void)msg_prio;` —— 想按优先级插队，得用 FreeRTOS 原生
   `xQueueSendToFront`，CMSIS-RTOS2 没有对应封装。
2. **`timeout` 是"等多久"而不是"多久发一次"**
   `Put(..., 200)` 的意思是"队列满时最多等 200 ms"，等不到就丢；`Put(..., 0)` 表示**根本不等**。
   `Get(..., osWaitForever)` 才是"永久等"。Part 1 Step 2 那道题的第一次丢弃，正是因为
   `Put` 用了 `timeout = 0`。
3. **队列满/空必须显式处理返回值** —— 示例里都没检查返回值，这是示例的简化，**不能照抄到我们的项目里**。

> **我们的做法**：`Q_Lidar` 深度 8、消息 12 字节（存 offset+length+id，**原始数据不拷贝**）；
> 满了显式丢弃并计 `q_full`。实测峰值占用 **0**，说明容量足够。

---

## 4. 实验三 · 信号量（`W4.3.1_Sem_bin` / `W4.3.2_Sem_cnt`）

### 4.1 实际配置

```c
osSemaphoreId_t myBinarySem01Handle;
const osSemaphoreAttr_t myBinarySem01_attributes = { .name = "myBinarySem01" };

myBinarySem01Handle = osSemaphoreNew(1, 0, &myBinarySem01_attributes);  /* 最大 1，初值 0 */
```

```c
/* Task1：自己定时释放 */
for (;;) { osDelay(2000); Task_action('1');
           osSemaphoreRelease(myBinarySem01Handle); }

/* Task2：等信号量，最多等 4000 ms */
for (;;) { osSemaphoreAcquire(myBinarySem01Handle, 4000); Task_action('2'); }

/* 外部中断里也释放 —— 这才是信号量的典型用法 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    Task_action('!');
    osSemaphoreRelease(myBinarySem01Handle);     /* ISR 里释放 */
}
```

### 4.2 预期现象

| 输入 | 预期 |
|---|---|
| 上电后不动 | 每 2 s 出现 `1` 然后 `2`（Task1 释放 → Task2 被唤醒） |
| 按蓝色按钮 | 立刻出现 `!`，紧接着 `2`（中断释放 → 任务被唤醒） |
| 若 Task1 停掉、也不按键 | Task2 在 4000 ms 后 `Acquire` 超时返回（`osErrorTimeout`） |

### 4.3 关键点：**从中断释放信号量是合法的，但有前提**

`W4.3.1` 的 `.ioc` 里：

```
NVIC.EXTI15_10_IRQn = 5      ← 按钮中断的优先级
NVIC.TIM6_DAC_IRQn  = 0      ← HAL 时基的优先级
```

| 事实 | 结论 |
|---|---|
| 按钮 EXTI 优先级 = **5**，正好等于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | 只有 **优先级数值 ≥ 5** 的中断才允许调 `osSemaphoreRelease()`；5 是边界值 |
| HAL 时基 TIM6 优先级 = **0** | 时基中断在系统调用上限**之上**，所以它不做任何 RTOS 调用（只 `uwTick++`） |

> **这条对我们的价值极大**：我们自己把 HAL 时基搬到 TIM7 时踩过一个坑 ——
> FreeRTOS 端口把 `uxCriticalNesting` 初始化成 `0xaaaaaaaa`，调度器启动前的
> `osThreadNew`/`osMessageQueueNew` 会让 `BASEPRI` 永久停在 `0x50`，
> 而 **BASEPRI 屏蔽外设中断、不屏蔽内核异常** → SysTick 时基没事，**TIM 时基直接停摆**。
> 解决方式就是把时基中断放到优先级 0。
> **课程示例把 TIM6 设在 0，正好是这个结论的独立印证**（只是示例没写出原因）。

### 4.4 二值 vs 计数

- `Sem_bin`：`osSemaphoreNew(1, 0)` —— 只有"有/无"两态，**不累积**。连按两次按钮，只有一次有效。
- `Sem_cnt`：`osSemaphoreNew(N, 0)` —— 可累积到 N，`Acquire` 每次消耗一个。
- **Part 2 用的是计数信号量**（TIM6 每 10 ms 给一次，IMU 任务每取一次消耗一个）。

---

## 5. 实验四 · 互斥量（`W4.6.1_Mutex_basic` / `W4.6.2_Mutex_sem`）

### 5.1 实际配置 —— **注意这里少了一样东西**

```c
osMutexId_t myMutex01Handle;
const osMutexAttr_t myMutex01_attributes = {
  .name = "myMutex01"
};                                   /* ← 没有 .attr_bits = osMutexPrioInherit ！ */
```

```c
/* Task1 与 Task2 结构完全相同，都是： */
for (;;) { osDelay(2000);
           osMutexAcquire(myMutex01Handle, 1000);
           Task_action('1');                       /* 持锁期间输出 */
           osMutexRelease(myMutex01Handle); }
```

### 5.2 这个例子想说明什么

两个任务都要"打印一整行"这个**共享资源**（ITM 输出通道）。没有互斥量时，两行字符会交错成乱码；
加了互斥量后，每次只有一个任务持锁，输出成行。

### 5.3 ⚠️ 最重要的一点：示例的互斥量**没有开优先级继承**

`.attr_bits` 缺省 → **不含 `osMutexPrioInherit`**。
所以 `Mutex_basic` 只是一个"互斥"演示，**不能证明优先级继承**。
这正是 `W4.6.2_Mutex_sem` 存在的原因：把 mutex 换成二值信号量，人为制造
**优先级反转**（低优先级任务持锁 → 中优先级任务抢占 → 高优先级任务等锁被饿死），
再对比 mutex 的优先级继承能否解决。

> **Part 2 的硬性要求**：`osMutexPrioInherit` 必须显式打开，并且要能演示"持锁时长"和
> "优先级继承的效果"。示例代码**不能照抄**。

```c
/* Part 2 要这样写 */
static const osMutexAttr_t i2c_mutex_attr = {
  .name      = "i2cBus",
  .attr_bits = osMutexPrioInherit,
};
```

### 5.4 与 Part 2 的对应关系

| 项 | 示例 | Part 2 |
|---|---|---|
| 保护的资源 | ITM 输出（只为演示） | **I2C1 总线**（MPU-6500 + OLED 共用） |
| 优先级继承 | **未启用** | **必须启用** |
| 持锁时长 | 一次打印 | 目标 < 3 ms（见规范文档 §3.4 的 OLED 分页策略） |
| 失败处理 | 忽略返回值 | 必须检查，拿不到锁不许碰硬件 |

---

## 6. 真机复核清单（板子接上后逐项打勾）

撰写时开发板未连接，以下为待补的实测项。**每一项都能由 §2–§5 的预期行为对照验证。**

```
[ ] E1  看到 1/2 每秒交替，'x' 从未出现
[ ] E1  在 SWV 里观察 Task2 的 TCB 每轮被回收（osThreadList 或 uxTaskGetSystemState）
[ ] E2  队列空时接收者为 BLOCKED（调试器看任务状态）
[ ] E2  把 Put 的 timeout 从 200 改成 0，制造一次"满队列丢弃"
[ ] E3  按按钮 → 立刻出现 '!' 和 '2'（中断 → 任务唤醒）
[ ] E3  Sem_cnt 连按两次，任务是"醒两次"而不是"醒一次"
[ ] E4  Mutex_basic：两任务输出不交错
[ ] E4  Mutex_sem：人为制造优先级反转，再开 PrioInherit 对比持锁时长
[ ] 全部：用 DWT 测量"中断 → 任务唤醒延迟"（我们工程实测 12–15 µs）
```

---

## 7. 结论：四个机制在 Part 2 里各自的落点

| 机制 | 示例演示了什么 | Part 2 用它做什么 | 我们代码里的位置 |
|---|---|---|---|
| **任务 / 调度** | 创建、终止、同优先级分时 | 10 个任务，固定周期 + 事件驱动混合 | `app_rtos.c`（现有 2 个，要扩到 10 个） |
| **队列** | 结构体传值、阻塞接收、超时 | `Q_Lidar`（DMA 描述符）/ `Q_Cmd`（蓝牙命令）/ `Q_Path`（路径点） | `Q_Lidar` 已实现（12 B 描述符，深度 8） |
| **信号量** | 中断释放、任务等待、超时 | TIM6 10 ms 中断 → IMU 采集任务 | 待实现（现在是中断里直接发 I2C，必须改） |
| **互斥量** | 互斥保护共享输出 | I2C1 保护 MPU-6500 + OLED，**必须开优先级继承** | 待实现 |

**一句话总结**：示例代码演示了四个机制的**用法**，但有三处**不能照抄**——
① `stack_size` 的单位坑；② mutex 没开优先级继承；③ 队列返回值不检查。
这正是我们从"跑示例"过渡到"写 Part 2"要自己补上的部分。
