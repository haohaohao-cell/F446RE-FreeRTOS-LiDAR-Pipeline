# 新增独立导航组件：洪水漫灌与雷达防撞

本次新增两个可独立编译、测试和调用的 C11 模块：`flood_nav` 负责格网寻路，
`lidar_guard` 负责依据真实雷达距离与运动状态判断是否允许行驶。
两个模块相互独立，也完全不依赖当前 STM32 固件。

**当前状态：算法组件已实现并通过主机测试，尚未接入雷达驱动、导航任务或电机。**
现有固件不会因此自动开始寻路或获得实时停车保护。

## 1. 本次新增文件及隔离方式

```text
README_NEW.md
navigation/
  flood_nav/
    flood_nav.h
    flood_nav.c
    README.md
    tests/
      test_flood_nav.c
      run_tests.ps1
  lidar_guard/
    lidar_guard.h
    lidar_guard.c
    README.md
    tests/
      test_lidar_guard.c
      run_tests.ps1
```

- 不修改现有 Core、Drivers、Middlewares、任务、队列、引脚、时钟或控制逻辑。
- 不修改 `.cproject`、`.ioc`、链接脚本和原 README。
- 不包含 HAL、FreeRTOS、pose、lidar、path 或 motor 头文件。
- 不创建中断、线程、队列或同步对象，不访问外设，不进行动态内存分配。
- 洪水模块符号使用 `flood_nav_` 前缀，防撞模块使用 `lidar_guard_` 前缀。
- 当前 CubeIDE 的 Debug/Release source entries 仅收录 Core、Drivers、
  Middlewares；新增 navigation 目录不进入固件编译，不占用现有镜像 RAM/Flash。
- tests 含独立 main，不得作为固件源文件加入构建。

## 2. 已统一的地图与车体约定

### 地图配置

| 配置 | 约定 |
|---|---|
| 列数、行数 | 各自可配置 1～32，支持长方形地图，不固定 8×8 |
| 单格尺寸 | 正方形，`cell_mm` 为毫米边长 |
| 起始格、终点格 | `{col, row}`，左上 `(1,1)`，列向右增加、行向下增加 |
| 方位 | 上北、下南、左西、右东 |
| 初始朝向 | 小车上电时实际朝向，北/东/南/西 |
| 终点行为 | 到达目标格中心且停稳后完成，固定行为，无需额外输入枚举 |

32×32 是编译期静态容量上限；实际地图大小由初始化配置决定。

### 已确认机械尺寸

```text
                车头 / 前方
                     ↑
               圆形车体圆心
                     │ 45 mm
            雷达 = 两驱动轮轮轴中点
                     = 现有位姿的位置参考点
```

- 圆形车体直径 200 mm，半径 100 mm。
- 车体圆心位于雷达前方 45 mm，横向偏移为零。
- 雷达沿正前射线距前壳 145 mm，沿正后射线距后壳 55 mm。
- 雷达/轮轴原地旋转时，车体圆心仍会绕该点旋转；完整转向相对雷达的最大
  几何扫掠半径为 145 mm，另加安全余量。

## 3. 洪水漫灌模块

接口：[navigation/flood_nav/flood_nav.h](navigation/flood_nav/flood_nav.h)。
详细说明：[navigation/flood_nav/README.md](navigation/flood_nav/README.md)。

### 四个函数

```c
flood_nav_result_t flood_nav_init(const flood_nav_config_t *config);
flood_nav_result_t flood_nav_observe(flood_nav_cell_t cell,
                                   flood_nav_direction_t direction,
                                   flood_nav_edge_t edge);
flood_nav_result_t flood_nav_set_location(const flood_nav_location_t *location);
flood_nav_result_t flood_nav_next(flood_nav_step_t *out);
```

初始化示例，包含初始朝向：

```c
flood_nav_config_t config = {
    .cols = 8,
    .rows = 6,
    .cell_mm = 600,
    .start = {1, 6},
    .goal = {8, 1},
    .initial_heading = FLOOD_NAV_NORTH
};
flood_nav_result_t result = flood_nav_init(&config);
/* 调用者检查 result，再继续。 */
```

地图边有 UNKNOWN、OPEN、WALL 三态。外围初始化为 WALL，内部为 UNKNOWN；
observe 接收已确认的格边观测，自动同步相邻格的反向边，允许纠正内部墙体。
雷达原始距离转换墙体、观测投票和数据新鲜度处理属于未来感知适配层。

set_location 输入当前格、当前实际朝向、是否到格中心、是否停稳。初始化后的
首次规划也必须提供有效位置，不假定车辆已经居中。initial_heading 是初始
配置，运行时的朝向通过 location.heading 更新；初始朝向与坐标变换也需由未来
适配层使用，不能仅设置初始化字段就认为现有 pose 已经旋转到地图坐标。

next 从终点反向 BFS，未知边可参与路径估计，但只有下一边已确认 OPEN 才返回
READY。下一边 UNKNOWN 时返回 NEED_OBSERVATION 及待观察方向，不允许移动。
同距离优先当前朝向，再按 N/E/S/W 确定选择。距离严格递减，无访问惩罚导致的
固定地图内部循环。

| 返回状态 | 含义 |
|---|---|
| READY | 可以执行返回的下一格目标 |
| NEED_LOCATION | 等待有效位置 |
| WAIT_CENTER | 等待到当前格中心 |
| WAIT_STOP | 等待停稳 |
| NEED_OBSERVATION | 需要确认候选格边，禁止按该候选直接行驶 |
| NO_PATH | 当前地图下无路 |
| FINISHED | 已到终点中心且停稳 |
| 负值错误 | 参数、初始化或边界错误 |

只有 READY/NEED_OBSERVATION 写输出。其他状态下调用者不可复用旧目标继续开车。
再次 init 即清空地图并复位；非法配置不破坏上一份有效状态。

输出 `east_mm`（向东为正）、`north_mm`（向北为正）是相对起始格中心的下一格
中心坐标，使用 int32_t。它不是现有固件的轮轴坐标路点；初始朝向旋转、起点
格内偏移、圆心与轮轴的 45 mm 补偿由未来适配层负责。

每次 BFS 最多 1024 次出队、4096 次邻边检查，无递归、无限重试或等待。
地图工作数组共 8192 B，外加少量状态。该模块单实例、非线程安全，应由单个
导航所有者调用，禁止从多个任务并发修改。

## 4. 雷达防撞模块

接口：[navigation/lidar_guard/lidar_guard.h](navigation/lidar_guard/lidar_guard.h)。
详细说明：[navigation/lidar_guard/README.md](navigation/lidar_guard/README.md)。

### 两个函数

```c
void lidar_guard_default_config(lidar_guard_config_t *config);
lidar_guard_result_t lidar_guard_evaluate(const lidar_guard_config_t *config,
                                        const lidar_guard_scan_t *scan,
                                        const lidar_guard_motion_t *motion);
```

防撞模块**读取调用者传入的实际雷达距离**，但不自行读串口或调用现有驱动。

scan 包含 360 个方向的距离、年龄和状态：

```c
float range_mm[360];
uint32_t age_ms[360];
uint8_t state[360];
```

以雷达/轮轴中点为原点，车体 x 向前、y 向左；0° 前、90° 左、180° 后、270° 右。
与洪水模块的东/北地图坐标不同，未来必须在适配层显式转换。

状态区分 HIT（有真实障碍回波）、CLEAR（已验证直到给定距离为空闲）和
MISSING（缺测/无可靠证据）。雷达零回波不能自动当作 CLEAR。所有 360 个
方向都需要有效、新鲜且足够远的证据；不满足即拒绝行驶，第一版有意保守。

motion 输入当前测得和本次请求的线速度、角速度，前进为正、逆时针为正。
检查同时考虑当前惯性和新指令，不能因请求停止或倒车而忽略当前前进速度。

| 返回值 | 含义 |
|---|---|
| LIDAR_GUARD_ALLOW | 本次输入与配置下通过检查 |
| LIDAR_GUARD_STOP_OBSTACLE | 障碍进入车体或制动扫掠范围 |
| LIDAR_GUARD_STOP_COVERAGE | 缺测或已确认空闲距离不足 |
| LIDAR_GUARD_STOP_STALE | 数据过期 |
| LIDAR_GUARD_ERR_ARGUMENT | 参数非法、超出速度上限、NaN/Inf 等 |

**只有 ALLOW 能放行请求；其余结果都应由控制任务执行停车/拒绝指令。**
本模块不写电机，不返回自动限速值。请求降低速度后仍会检查尚未降下来的实测速度。

停车距离包含观测年龄、执行延迟和最低制动能力：

```text
T = (reaction_ms + max_age_ms) / 1000
D = |v| * T + v² / (2 * brake_mm_s2)
```

转向按角制动距离扩展车体扫掠包络，支持直行、倒车、转弯及掉头，并加入
偏心几何、安全余量、测距误差与角度分箱余量。

已确认的几何默认值为半径 100 mm、前移 45 mm。安全余量 20 mm、测距误差
30 mm、额外反应延迟 200 ms、最大数据年龄 200 ms、线减速度 200 mm/s²、
角减速度 180°/s² 都是工程初值，必须通过实车测量确认或替换。特别是
motor_stop 的实际制动/滑行能力必须符合配置，不能直接把默认值当作已测结果。

模块无内部可变状态，调用者提供一致输入快照。scan 约占 3240 B，建议静态
存放，不放到现有小任务栈。每次固定检查 360 项，无等待或动态分配；需要
标准数学库，GCC 链接时加 `-lm`。目标 MCU 耗时尚未测量。

## 5. 后续接入点（本次未实现）

```text
现有 LiDAR + Pose
        ↓
独立适配层：新鲜度、角度标定、运动补偿、格边提取、坐标变换
        ├─ 已确认格边 + 当前格状态 → flood_nav → 下一格中心
        └─ 新鲜雷达距离 + 实测/请求运动 → lidar_guard → 放行/禁止
                                                        ↓
                                   控制任务处理旧路点取消、急停与运动执行
```

必须先处理以下事实，才能让小车使用这些组件：

1. 当前雷达滚动图会留存旧数据，没有逐束年龄，不能直接假定全图新鲜。
2. 历史回波需配准到当前雷达/车体坐标，原始角度方向与安装零偏需标定。
3. 轮轴中点就是雷达原点；建图直接用该位置，不能再次扣除 45 mm。
4. 碰撞圆心是轮轴前方 45 mm。若要求车体圆心到终点中心 G，预期最终车头
   单位向量为 f，则轮轴位置目标为 `G - 45*f`，所有量须在同一坐标系。
5. 地图/防撞结果变化后，需要取消控制任务手中的活动路点；仅清空 Q_Path 不够。
6. 导航等待、扫描失效和停车确认需要外部超时与故障原因上报；组件不内部死等。
7. 原地转动占用的是完整扫掠空间，不能只用 100 mm 雷达同心圆检查掉头。
8. 测量制动能力和实时耗时，再选择调用任务、周期及决策有效期。

离散射线无法保证看见所有细小、低矮或不反光障碍；本次主机验证也不是整车
防撞验收。当前缺少实际数据桥接和停车执行链路，不能宣称实车防撞已启用。

## 6. 已完成验证与复现

2026-11-20 的主机验证记录：

| 检查 | 结果 |
|---|---|
| Flood Fill 独立编译 | C11 严格警告配置通过，无警告 |
| Flood Fill 测试 | 55,858 项通过 |
| 防撞独立编译 | C11 严格警告配置通过，无警告 |
| 防撞测试 | 3,682 项通过 |
| 新增防撞后 Flood Fill 回归 | 55,858 项再次通过 |
| 原固件已有主机测试（首次新增 Flood Fill 时） | 协议 50、路径 196、位姿 18 项通过 |
| 原文件完整性 | 首次 244 个原文件未变；新增防撞前已有的 249 个文件也未变 |
| 固件编译/烧录/实车运行 | 本次未执行；新增目录没有加入固件构建 |

原路径主机测试存在 abs/labs 缺少标准库声明的既有警告，未修改原测试文件。

Flood Fill 测试包括四朝向、1×1/长方形地图、未知边、无路、双向墙同步、动态
揭墙、100 张地图与参考 BFS 对照、21 趟在线运行，以及 32×32 的 1023 步蛇形
通道和最大单格尺寸。防撞测试包括偏心前后车壳、转向扫掠、换向惯性、360
方向逐项缺测/过期、非法值及独立积分得到的 120 条随机刹车轨迹。

运行测试需要主机 GCC（不是 arm-none-eabi-gcc）：

```powershell
& .\navigation\flood_nav\tests\run_tests.ps1
& .\navigation\lidar_guard\tests\run_tests.ps1

# 如果 gcc 不在 PATH，显式指定本机编译器路径：
& .\navigation\flood_nav\tests\run_tests.ps1 -Compiler 'D:\BaiduNetdiskDownload\MinGW\bin\gcc.exe'
& .\navigation\lidar_guard\tests\run_tests.ps1 -Compiler 'D:\BaiduNetdiskDownload\MinGW\bin\gcc.exe'
```

两个脚本都使用独立临时输出目录，不在原工程生成可执行文件；测试运行有
20 秒超时保护。通用 C 编译命令见各模块 README。
