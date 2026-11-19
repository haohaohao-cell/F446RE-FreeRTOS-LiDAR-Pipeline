# 演示快捷方式（双击运行）

这些只是把 `tools\bt_run.ps1` / `bt_send.ps1` / `bt_link_check.ps1` 包了一层，
**不需要打字**。双击后会弹出一个黑窗口，显示发出去的命令，跑完按任意键关闭。

| 双击 | 做什么 | 什么时候 |
|---|---|---|
| `preflight_link.cmd` | 查蓝牙链路（读车上 10 Hz 遥测字节） | **开机后第一件事**，车要上电 |
| `demo_1_straight.cmd` | 清零位姿 → 直行到 (0, 1400) mm | 拔掉 USB 之后，第一趟 |
| `demo_2_left.cmd` | 清零 → 直行 700 → **左转 90°** → 直行 700，到 (−700, 700) | 把车抱回起点之后 |
| `demo_3_right.cmd` | 清零 → 直行 700 → **右转 90°** → 直行 700，到 (700, 700) | 把车抱回起点之后 |
| `demo_0_stop.cmd` | 发 `S`：停车、清路径 | 任何时候，出问题就双击 |

> 场地是十字形：起点在十字中心，三条臂分别是"正前 1.40 m / 左前 (−0.70, 0.70) /
> 右前 (0.70, 0.70)"（Part 2 课件第 3 页的原图）。**右转这趟的目标是 (700, 700)**，
> 不是 (700, −700)：先直行、再转弯，不是先转弯、再走斜线。

## 顺序

```
0. 车上电，双击 preflight_link.cmd    → 要看到 link: OK，约 7000 B/s
1. 拔 USB，等 3 秒
2. 双击 demo_1_straight.cmd           → 等约 13 秒，车自己停 → 卷尺量
3. 把车抱回起点（别推着走）
4. 双击 demo_2_left.cmd               → 等约 13 秒 → 卷尺量
5. 抱回起点
6. 双击 demo_3_right.cmd              → 等约 13 秒 → 卷尺量
```

## 每个窗口里会看到什么

```
link: OK, 5120 B/s of telemetry (2560 B in 500 ms)
sent: Z (twice - pose + path zeroed)
sent: G 0 1400
      target is 1400 mm away; the path is axis-aligned:
      straight 1400 mm along +y, then one 90 deg turn, then 0 mm along x
      expect about 12 s of driving (total path 1400 mm)
      the car stops by itself when the path is done - do not touch it
```

- 看到 `link: OK` 才继续。**看到 `link: NO DATA` 就别发车**：命令进不去。
- 看到 `sent: Z` 和 `sent: G ...` 两行就说明**命令真的发出去了**。
- `cannot open COM10` + `already open elsewhere` = 点云窗口占着端口，关掉它。
- `port COM10 is not listed` = 蓝牙掉线/没配对，脚本会把 Windows 现在看得见的端口列出来。

## 注意

- 绿色窗口**不要手动关**，等它自己提示按任意键
- 两趟之间必须**把车抱回起点**：脚本每次都清零位姿，它默认车又站在原点了
- 车跑的时候别碰它，它到点会自己停
- 脚本会**重试打开端口**（最多 10 次、每次 600 ms），所以偶发掉线时多试一次往往就好了
