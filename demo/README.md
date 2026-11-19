# 演示快捷方式（双击运行）

这些只是把 `tools\bt_run.ps1` / `bt_send.ps1` 包了一层，**不需要打字**。
双击后会弹出一个黑窗口，显示发出去的命令，跑完按任意键关闭。

| 双击 | 做什么 | 什么时候 |
|---|---|---|
| `demo_1_straight.cmd` | 清零位姿 → 走到 (0, 1400) mm | 拔掉 USB 之后，第一趟 |
| `demo_2_left.cmd` | 清零位姿 → 走到 (−700, 700) mm（先左转 45°） | 把车抱回起点之后 |
| `demo_3_right.cmd` | 清零位姿 → 走到 (700, −700) mm（先右转 135°） | 把车抱回起点之后 |
| `demo_0_stop.cmd` | 发 `S`：停车、清路径 | 任何时候，出问题就双击 |

## 顺序

```
1. 拔 USB，等 3 秒
2. 双击 demo_1_straight.cmd       → 等 13 秒，车自己停 → 卷尺量
3. 把车抱回起点（别推着走）
4. 双击 demo_2_left.cmd           → 等 10 秒 → 卷尺量
```

## 每个窗口里会看到什么

```
sent: Z  (pose + path zeroed)
sent: G 0 1400
      distance 1400 mm, expect about 12 s of driving
      the car stops by itself when the path is done - do not touch it
```

看到 `sent: Z` 和 `sent: G ...` 两行就说明**命令真的发出去了**。如果只看到 `cannot open COM10`，
说明蓝牙没连上（检查模块上电、或点云窗口占着端口）。

## 注意

- 绿色窗口**不要手动关**，等它自己提示按任意键
- 两趟之间必须**把车抱回起点**：脚本每次都清零位姿，它默认车又站在原点了
- 车跑的时候别碰它，它到点会自己停
