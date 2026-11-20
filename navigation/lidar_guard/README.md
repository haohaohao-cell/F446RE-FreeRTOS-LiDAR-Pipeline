# 独立雷达防撞判定模块

本目录只新增独立 C11 源码、接口和主机测试。与 `flood_nav` 也完全解耦，
不包含现有工程头文件，不调用 HAL/RTOS，不读外设，不写电机，不创建任务，
没有全局可变状态和动态分配。现有 CubeIDE 的 source entries 不包含 navigation，
因此这个模块尚未进入固件，也尚未对真实小车提供自动停车保护。

## 已确认几何及坐标

- 圆形车体直径 200 mm，半径 100 mm。
- 雷达与两驱动轮轮轴中点重合，作为计算原点。
- 车体圆心位于该原点正前方 45 mm。
- 使用车体坐标：x 向车头为正，y 向车体左侧为正。
- 360 个方向，0 度为正前，90 度为左，180 度为后，270 度为右。
  实际雷达安装角和角度方向由将来的适配层转换。
- 几何前壳距离 145 mm、后壳距离 55 mm。转动时圆心随车头绕原点转动，
  完整转向车体扫掠最大半径为 145 mm，另加配置的余量。

## 仅两个接口

```c
void lidar_guard_default_config(lidar_guard_config_t *config);

lidar_guard_result_t lidar_guard_evaluate(
    const lidar_guard_config_t *config,
    const lidar_guard_scan_t *scan,
    const lidar_guard_motion_t *motion);
```

default_config 写入车体几何和可调整的工程初值。它不是硬件标定：

| 字段 | 默认值 | 含义 |
|---|---:|---|
| body_radius_mm | 100 mm | 车体半径，已确认 |
| center_forward_mm | 45 mm | 圆心在雷达前方的距离，已确认 |
| clearance_mm | 20 mm | 额外车体安全余量，待实车确认 |
| range_error_mm | 30 mm | 测距与配准误差上界，待确认 |
| reaction_ms | 200 ms | 扫描年龄之外的计算、通讯、控制、执行延迟上界，待测 |
| brake_mm_s2 | 200 mm/s² | 可保证的最小线减速度，必须实测 |
| brake_deg_s2 | 180 °/s² | 可保证的最小角减速度，必须实测 |
| max_linear_mm_s | 1000 mm/s | 允许评估的速度上限，超过即拒绝 |
| max_angular_deg_s | 360 °/s | 允许评估的角速度上限，超过即拒绝 |
| max_age_ms | 200 ms | 各方向观测的最大允许年龄 |

scan 是调用者持有的定长结构，三个数组各 360 项：

```c
float range_mm[360];
uint32_t age_ms[360];
uint8_t state[360];
```

每个方向必须显式设置状态：

- `LIDAR_GUARD_HIT`：在 range_mm 处有真实回波障碍。
- `LIDAR_GUARD_CLEAR`：沿此方向直到 range_mm，已验证为空闲。
- `LIDAR_GUARD_MISSING`：没有可靠观测。零回波、丢包、从未采样不等于 CLEAR。

age_ms 是每个方向在本次调用时的年龄，不是整个扫描数组共用的刷新时间。
历史回波必须先配准到当前车体原点和角度并重分箱；盲目把旧扫描和最新位姿
混用会产生误判。若不做运动补偿，需要通过实验为配准误差建立保守上界并
计入 range_error_mm；无法保证时按 MISSING 处理。

motion 的四个字段：

```c
float measured_linear_mm_s;
float measured_angular_deg_s;
float requested_linear_mm_s;
float requested_angular_deg_s;
```

正线速度为前进、负线速度为倒车；正角速度为逆时针、负角速度为顺时针。
输入必须是可靠的当前测量与本次拟执行指令。测量失效时不要调用后执行运动，
应直接禁止行驶。即使请求速度为零，实际速度不为零时仍检查停车轨迹。

## 返回值

| 返回值 | 调用方行为 |
|---|---|
| `LIDAR_GUARD_ALLOW` | 本次证据及配置下，保守包络未发现障碍，可继续检查后执行 |
| `LIDAR_GUARD_STOP_OBSTACLE` | 有障碍进入车体／制动扫掠范围，禁止指令并请求停车 |
| `LIDAR_GUARD_STOP_COVERAGE` | 存在缺失方向、非法状态或已证实空闲的距离不够，禁止行驶 |
| `LIDAR_GUARD_STOP_STALE` | 观测过期，禁止行驶 |
| `LIDAR_GUARD_ERR_ARGUMENT` | 配置、速度或有效光束距离非法，包括 NaN/Inf，禁止行驶 |

**仅 ALLOW 表示通过，其余所有结果都必须拒绝运动。** 不返回限速值、不做阻塞等待，
调用者可在后续控制周期用更低请求速度重新评估；实际未降下来的速度仍参与检查。
多种问题同时存在时优先报告已识别障碍，参数错误也可直接终止评估。返回一个
原因并不意味着其他方向不存在故障。

调用示意（模块本身不包含下述“停车/放行”动作的实现）：

```c
lidar_guard_config_t config;
lidar_guard_default_config(&config);
/* 用实测延迟、最低制动能力和误差替换初值。 */
/* scan、motion 必须由外部适配层填入最新有效数据。 */
lidar_guard_result_t result = lidar_guard_evaluate(&config, &scan, &motion);
if (result != LIDAR_GUARD_ALLOW) {
    /* 取消旧运动目标，并通过控制任务请求停车。 */
} else {
    /* 本周期才可放行请求；下一控制周期继续检查。 */
}
```

## 判定方法与执行边界

线速度幅值 v 的保守停车距离：

```text
T = (reaction_ms + max_age_ms) / 1000
D = |v| * T + v² / (2 * brake_mm_s2)
```

角速度幅值 w 使用相同形式计算最大转角。当前和请求的线速度分别构造包络，
角速度取两者幅值最大值，防止“当前仍前进却因收到倒车或转向命令而漏掉旧运动”。
测量到的速度与请求幅值必须覆盖停车期间的运动上界，且配置减速度必须能被保证。
超出上界的加速、侧滑、碰撞推移不能用该模型保证。

直行包络是沿轮轴运动方向的偏心圆胶囊；转向引起的圆心和运动路径偏移采用
保守膨胀，并与“最大行程＋圆心偏移＋车体半径”的原点圆盘取交集。
真实扫掠被这两个包络各自包含，因此交集仍保守，支持前进、倒车、转弯、掉头。
额外加入半个 1 度分箱对应的横向余量，缓解相邻射线之间的角度离散误差。

判定要求所有 360 个方向都有新鲜可靠数据。这是有意保守的第一版：雷达不能
给齐证据时会停，不能把 MISSING 自动补成远距离。每个 HIT 端点不得进入包络；
每个 CLEAR 射线必须足够长以覆盖该方向的包络。使用雷达为原点且传感器位于
圆形车体内，包络包含原点，使单射线的覆盖判定成立。

1 度离散采样、障碍高度、材料反射率与雷达盲区仍可能造成不可见障碍。
分箱膨胀不等同于证明“射线之间任意细小障碍都可见”；清晰回波与可靠自由空间
证据的含义必须由传感器适配层保证。当前固件的滚动距离图没有逐束有效年龄，
不能原样当作本接口 scan 输入。

一次评估固定扫描 360 项，使用有界数学计算，无递归、忙等、重试、动态内存。
模块无内部可变状态；scan 约占 3240 B，建议由调用者静态存放，不塞进现有
小任务栈。评估期间输入对象不可被其他任务修改，宜传入一致快照。
用到标准数学库，主机 GCC 链接 `-lm`。在 MCU 上的执行耗时尚未测量，未来应
放在合适任务中评估，再以有有效期的决策交给控制层；不能把未经测量的运算
直接加进现有 10 ms 任务。

## 验证

```powershell
& .\navigation\lidar_guard\tests\run_tests.ps1 -Compiler 'D:\BaiduNetdiskDownload\MinGW\bin\gcc.exe'
```

独立编译启用 C11、Wall、Wextra、Werror、Wpedantic、Wconversion、Wshadow。
编译产物仅在系统临时目录，测试有 20 秒超时保护。
覆盖 145/55 mm 偏心前后壳、左右侧、正反向制动、正反转向、换向时的当前
惯性、余量、逐方向缺测/过期、覆盖不足、非法值、延迟和数据年龄增加。
另以独立细步长动力学积分生成 120 条随机刹车轨迹，将真实移动车壳上的点
量化到激光角度箱，验证全部被判为障碍。测试是算法验证，不是实车制动标定。
