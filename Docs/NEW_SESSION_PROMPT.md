# 新对话开场提示词（复制粘贴用）

> 用法：新开一个对话，把下面 `====` 之间的内容整段贴进去。
> 当前进度：**Part 1 已完成**，所以这份提示词指向 **Part 2**。

---

```
====
这是嵌入式课程 Block 4.5 Coding Day 的续做。任务已改版成两部分：Part 1（纸面分析，已完成）
和 Part 2（整车自主导航，待做）。上一轮还完成并上板验证了旧版的 LiDAR 流水线。

本地工程：D:\嵌入式小车\CodingDay_F446RE\
GitHub（public）：https://github.com/haohaohao-cell/F446RE-FreeRTOS-LiDAR-Pipeline
课程示例代码：D:\嵌入式小车\course_materials\EBU6470_Codes_F446（有版权，禁止提交）
Coding Day 2 代码：D:\stm32codespace\lessonProgram（Motor/Encoder/Oled/Track/RunStopController/Board）

开工前必须先读这两个文件，并严格遵守：
  1. D:\嵌入式小车\CodingDay_F446RE\Docs\HANDOVER.md          —— 现状、硬件接线、三个已修的坑、Part 2 待办清单
  2. D:\嵌入式小车\CodingDay_F446RE\Docs\CODING_STANDARDS.md   —— 编码规范（铁律：保留烧录口、任务里禁止阻塞、资源单一所有者）

特别提醒（重写时最容易复发，详见 HANDOVER 第五节）：
  - HAL 时基在 TIM7 上，中断优先级必须是 0，否则 BASEPRI 会让它彻底停摆
  - 开机必须用 DWT 校准 TIM7 预分频（本板实测跑在 168 MHz，不是手册算的 84 MHz）
  - NVIC 优先级最后必须由 app_init() 里的 irq_priorities() 统一重新断言
  - TIM6 和 UART5 的 NVIC 优先级要从 6/7 改成 5（它们要调用 RTOS API 了）

现在开始 Part 2。不要直接写代码，先按下面顺序产出计划让我确认：

1) 一份实施计划表：现有代码里哪些直接复用、哪些从 lessonProgram 移植、哪些新写；
   引脚与定时器冲突怎么解（HANDOVER 第七节列了 5 个待确认项，先把它们标出来问我）。
2) 一张任务表：10 个任务的调度方式/周期/优先级/栈大小/WCET 预算，对照 PPT 的参考架构。
3) 三个队列（Q_Lidar / Q_Cmd / Q_Path）的消息结构体定义和容量/溢出策略。

计划确认后，第一步只做硬件层：把 Motor / Encoder / Oled 移植进来，用手动给定 PWM 的方式
验证电机转向和编码器计数正确，不碰 RTOS 架构。做完这一层再往上搭任务。
====
```

---

## 各阶段该贴什么

| 你想干什么 | 用哪段 |
|---|---|
| 开始 Part 2（当前） | 上面 `====` 之间的完整内容 |
| Part 1 的解答要复核 | 把 `HANDOVER.md` 第六节的"参考答案"表贴给它，再贴你自己的解答，让它逐项比对 |
| 只想做硬件层验证 | 保留提示词，把最后一段换成："现在只做硬件层：移植 Motor/Encoder/Oled，用固定 PWM 验证电机与编码器" |
| 要重写报告/README | 在提示词里加一句："Part 2 定稿后重写 Docs 里的报告与 README（现在是旧任务内容）" |

---

## 为什么这么写

| 设计 | 原因 |
|---|---|
| 强调"先读两个文件" | 新会话没有上下文；硬件接线和三个坑都在交接文档里 |
| 强调三个坑 | 只在真实硬件上暴露，静态审查看不出来，重写必然复发 |
| 要求先出三张表 | 10 个任务的优先级/栈/周期、队列消息结构，是后面所有代码的地基，定错了要重写 |
| 要求先只做硬件层 | 电机/编码器/OLED 是新引入的硬件，必须在 RTOS 架构之前单独验证，否则出问题分不清是硬件还是调度 |
| 明确"先问我 5 个待确认项" | OLED 接法、团队人数、lessonProgram 版本这几件事会改变方案 |
