# xCore SDK Python：CR7 命令行控制

基于珞石 SDK，提供统一入口 `uv run xcore COMMAND [OPTIONS]` 和可复用的 Python 连接／驱动层。开发细节见 [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)，厂商 SDK 安装说明见 [docs/README.md](docs/README.md)。

2026-10-04 已先用原生 SDK 完成实机连接、状态、关节角、法兰位姿、软限位和 DH 参数读取，再验证 CLI。实机返回：

| 项目 | 已验证值 |
| --- | --- |
| 机械臂型号／轴数 | `XMC7-R850-W4X3B4`／6 轴 |
| 控制器／SDK | `3.2.1`／`0.7.1` |
| Python／平台 | CPython `3.11.16`／Linux x86_64 |
| 机械臂 IP | **`192.168.2.160`** |
| 电脑有线网卡／附加地址 | `enx00e04c634750`／`192.168.2.100/24` |
| 最近实验结束状态 | `on`、`automatic`、`idle` |

原先提供的 `192.168.0.160` 未完成连接；使用实际地址后原生 SDK 和 CLI 均成功。**原生 SDK 运动已验证**：自动模式、上电，第六轴低速增加约 1°，再返回测试前位置，最后恢复下电／手动／空闲。随后已完成第六轴速度对比及 CLI `move-joint`／`movej` 实测。当前运动默认速度为 **`--speed 1000`**，需要低速时显式指定 `--speed 50`。

## 快速启动

在本目录执行：

```bash
cd /home/knight/projects/xcore/xcoresdk-python
uv sync --frozen --python 3.11
uv run xcore doctor
uv run xcore network
uv run xcore status
```

本机已经具备 `Release/linux/xCoreSDK_python.cpython-311-x86_64-linux-gnu.so`。新电脑需要先从 [官方 SDK v0.7.1 Release](https://github.com/RokaeRobot/xCoreSDK-Python/releases/tag/v0.7.1) 获取与操作系统、CPU 架构和 CPython 版本匹配的二进制；二进制不纳入 Git。`doctor` 检查扩展加载和 API，不连接机械臂。

项目当前固定 Python 3.11。SDK 不是只能使用 Python 3.10，但 `cpython-310` 二进制不能直接用于 3.11，单纯重命名文件也不能改变 ABI。更换解释器时需同时更换匹配扩展并调整项目版本约束。

默认机械臂 IP 为 `192.168.2.160`，可显式指定或通过环境变量配置：

```bash
uv run xcore status --ip 192.168.2.160 --timeout 20
export XCORE_ROBOT_IP=192.168.2.160
```

本次已在 NetworkManager 的有线连接配置中保存附加地址 `192.168.2.100/24`，保留原有 DHCP 配置；临时探测地址已清理。在另一台电脑上，可先通过以下命令临时添加同网段地址，接口名按实际网卡替换：

```bash
uv run xcore network configure --interface enx00e04c634750 --address 192.168.2.100/24
uv run xcore network
uv run xcore status
```

`network configure/reset` 仅修改当前活动网卡配置，需要系统允许当前用户操作 NetworkManager。永久配置方法及本次保存的连接 UUID 见开发文档。`network` 中 TCP 端口可达不代表 SDK 握手成功，应以 `status` 返回 `ok: true` 为准。

SDK 建连可能重置运动相关状态，断开连接可能停止已有运动。状态查询应在机械臂空闲、没有其他 SDK 控制会话时执行。

## 指令介绍

### 1. 基本用法与公共参数

所有命令均在项目根目录执行，参数放在子命令之后：

```bash
uv run xcore COMMAND [OPTIONS]
uv run xcore --help
uv run xcore move-joint --help
```

命令输出 JSON；成功退出码为 `0`，执行失败为 `1`，参数错误为 `2`，等待超时为 `124`，键盘中断为 `130`。`--output` 仅保存成功结果，自动创建父目录，不覆盖已有文件。

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| `--ip` | 机械臂控制器 IPv4 地址 | `XCORE_ROBOT_IP` 或 `192.168.2.160` |
| `--local-ip` | 电脑通信地址，非实时查询通常可省略 | `XCORE_LOCAL_IP` 或由 SDK 选择 |
| `--sdk-dir` | 厂商扩展所在目录 | `XCORE_SDK_DIR` 或平台对应的 `Release/` 子目录 |
| `--timeout` | SDK 命令的整体等待上限，单位秒 | `20` |
| `--output PATH` | 将成功结果保存为新 JSON 文件 | 仅在终端输出 |

例如，指定机械臂地址并保存状态：

```bash
uv run xcore status --ip 192.168.2.160 --timeout 20 --output logs/status.json
```

`network` 的系统查询各有独立时限，`check` 的 `--timeout` 分别用于每个检查进程；这两类命令不使用 SDK 的整体等待时限。详细规则见 [开发文档](docs/DEVELOPMENT.md#命令与参数)。

### 2. 查询指令（Read command）

查询指令读取控制器反馈，不发送上电或移动目标。每次命令建立一个 SDK 会话，`monitor` 在同一会话中连续采样；SDK 断连可能停止已有运动，因此应在空闲且没有其他 SDK 控制会话时使用。

#### 常用指令

| 命令 | 功能 | 最简示例 | 带参数示例 |
| --- | --- | --- | --- |
| `status` | 汇总型号、版本、电源、模式、运行状态、六轴角度和法兰位姿 | `uv run xcore status` | `uv run xcore status --ip 192.168.2.160 --output logs/status.json` |
| `joints` | 读取 J1～J6 当前角度，同时输出弧度和度 | `uv run xcore joints` | `uv run xcore joints --ip 192.168.2.160 --output logs/joints.json` |
| `pose` | 读取法兰或工具末端位姿 | `uv run xcore pose` | `uv run xcore pose --frame tool --output logs/tool-pose.json` |
| `monitor` | 在一个连接中采集多次状态，结束后统一输出 | `uv run xcore monitor` | `uv run xcore monitor --duration 10 --interval 0.5 --timeout 20 --output logs/status-samples.json` |

`pose` 默认 `--frame flange`，表示法兰相对基座；`--frame tool` 表示末端相对当前参考坐标系。位姿格式为 `[x, y, z, rx, ry, rz]`，位置单位为米，姿态角单位为弧度。

`monitor` 默认采样 `5` 秒、间隔 `0.5` 秒；实际频率受 SDK 查询耗时影响。`--timeout` 必须大于 `--duration + 5`。

#### 参数核查与建模指令

| 命令 | 功能 | 最简示例 | 带参数示例 |
| --- | --- | --- | --- |
| `info` | 读取机器人型号、标识、控制器版本、轴数和 MAC | `uv run xcore info` | `uv run xcore info --ip 192.168.2.160 --output logs/robot-info.json` |
| `limits` | 读取当前控制器软限位及启用状态 | `uv run xcore limits` | `uv run xcore limits --output logs/soft-limits.json` |
| `dh` | 读取六轴 DH 参数，默认返回校准／设置后的值 | `uv run xcore dh` | `uv run xcore dh --nominal --output logs/dh-nominal.json` |

`dh --nominal` 返回标称值；每轴参数顺序为 `[Alpha(度), A(毫米), D(毫米), Theta(度)]`。`joints` 与 `dh` 同时保留 SDK 原始数组和额外槽位，六轴解析及建模约定见 [开发文档](docs/DEVELOPMENT.md#反馈格式与建模参数)。

### 3. 控制指令（Hardware command）

控制指令会改变机械臂状态或产生实际运动。`movej` 和 `move-joint` 要求机械臂处于**上电、自动模式、空闲、软限位启用**状态；它们不会自动上电或切换操作模式。原生 SDK 与 CLI 已完成第六轴运动测试。当前 `movej` 和 `move-joint` 默认使用 **`--speed 1000`**；其他关节和负载条件未做最高速度实测。

| 命令 | 功能 | 最简示例 | 带参数示例 |
| --- | --- | --- | --- |
| `mode` | 切换手动／自动模式 | `uv run xcore mode automatic` | `uv run xcore mode manual --ip 192.168.2.160` |
| `power` | 显式上电／下电，并读取电源状态 | `uv run xcore power on` | `uv run xcore power off --ip 192.168.2.160` |
| `stop` | 请求停止并读取运行状态 | `uv run xcore stop` | `uv run xcore stop --ip 192.168.2.160 --timeout 10` |
| `move-joint` | 指定单轴，相对该轴当前角度增减 | `uv run xcore move-joint --joint 6 --delta-deg 1` | `uv run xcore move-joint --joint 6 --delta-deg 1 --speed 1000 --max-step-deg 2 --tolerance-deg 0.2 --motion-timeout 15 --timeout 20` |
| `movej` | 移动到 J1～J6 的六个绝对目标角度 | `uv run xcore movej --joints J1 J2 J3 J4 J5 J6` | `uv run xcore movej --joints J1 J2 J3 J4 J5 J6 --unit deg --speed 1000 --max-step-deg 2 --tolerance-deg 0.2 --motion-timeout 15 --timeout 20` |

`movej` 示例中的 `J1 … J6` 是占位符，使用时替换为实际规划的六个目标值。`--unit` 可选 `deg`／`rad`，默认 `deg`。`move-joint --joint` 使用 `1～6` 的轴编号，`--delta-deg` 始终以度为单位，正负值分别表示沿该关节坐标正向／反向移动。

本机在手动模式下直接执行 `power on` 曾返回 `ec: -514`（上下电失败）；2026-10-04 实测先执行 `uv run xcore mode automatic`，再执行 `uv run xcore power on`，两步均成功。SDK 规定，有外接使能开关或示教器时，手动模式的软件上电受限制。`power on` 不会隐式切换模式，初次上电应按上述顺序执行；`-514` 本身是通用上电失败码，其他情况下还需结合控制器状态判断。

#### 运动公共参数

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| `--speed` | SDK 速度参数，单位 mm/s，允许范围 `5～4000` | **`1000`** |
| `--max-step-deg` | 每个关节的目标相对当前角度的最大变化量，超出则拒绝执行 | `10°` |
| `--tolerance-deg` | 判断到位时允许的最大关节误差 | `0.2°` |
| `--motion-timeout` | 等待实际到位的上限，单位秒 | `15` |

`--speed` 不是关节角速度或速度百分比。`--timeout` 必须大于 `--motion-timeout + 2`。命令会同时检查运行状态回到空闲和六轴误差满足容差，才报告到位；超时机制限制程序等待，不能保证物理停止。

#### 第六轴低速小幅运动示例

以下示例显式使用 `--speed 50` 覆盖默认值。确认现场可以运动后，按顺序逐条执行，并检查每一步输出中的 `ok` 和反馈状态：

```bash
# 读取当前姿态，切换自动模式并上电
uv run xcore status
uv run xcore mode automatic
uv run xcore power on

# 第六轴相对当前角度增加 1°，然后相对新的当前位置减少 1°
uv run xcore move-joint --joint 6 --delta-deg 1 --speed 50 --max-step-deg 2
uv run xcore move-joint --joint 6 --delta-deg -1 --speed 50 --max-step-deg 2

# 结束后下电并切回手动模式
uv run xcore power off
uv run xcore mode manual
```

两次相反的相对移动会受实际到位误差影响；需要返回指定原始姿态时，应保存开始时的六轴角度，再用 `movej` 下发该绝对目标。需要中途停止时执行 `uv run xcore stop`。命令结束后不会自动下电或恢复操作模式，应按需要显式执行收尾命令。

#### 默认速度的实测依据

2026-10-04 在当前夹爪与控制器配置下，保持其他五轴目标固定，对第六轴进行 `−30° → +30° → −30°` 的逐档对比。每段行程 60°，下表取两个方向中较高的速度反馈峰值：

| `--speed`（mm/s 参数） | J6 实测峰值（°/s） | 60° 单程耗时 |
| ---: | ---: | ---: |
| 50 | 3.36 | 20.31～20.36 s |
| 150 | 6.30 | 11.20 s |
| 350 | 14.64 | 5.25 s |
| 650 | 26.34 | 3.19 s |
| 1000 | 39.72 | 2.30～2.35 s |
| 4000 | 39.84 | 2.30～2.34 s |

默认值为 `1000`；本次实验中，`1000` 与 `4000` 的实际速度已无明显差异，因此选用 `1000` 作为默认参数。SDK 允许的参数上限仍为 `4000`。**约 40°/s 是当前配置下第六轴的实测结果，不是 4000°/s，也不能推广为所有关节的硬件极限。** 轨迹、负载和控制器限速变化后需要重新验证。运动成功结果中的 `speed_mm_s` 会返回实际使用的参数；完整实验条件和原始日志说明见 [开发文档](docs/DEVELOPMENT.md#2026-10-04-第六轴速度实验与默认值调整)。

### 4. 环境、网络与开发指令

| 命令 | 功能 | 最简示例 | 带参数示例 |
| --- | --- | --- | --- |
| `doctor` | 检查 Python、SDK 二进制和必要 API，不连接机械臂 | `uv run xcore doctor` | `uv run xcore doctor --sdk-dir Release/linux --output logs/doctor.json` |
| `network` | 检查电脑 IPv4 地址、路由、邻居表及 SDK TCP 6666 端口 | `uv run xcore network` | `uv run xcore network check --ip 192.168.2.160 --output logs/network.json` |
| `network configure` | 给指定网卡临时增加电脑 IPv4 地址 | `uv run xcore network configure --interface enx00e04c634750` | `uv run xcore network configure --interface enx00e04c634750 --address 192.168.2.100/24` |
| `network reset` | 从指定网卡移除该活动地址 | `uv run xcore network reset --interface enx00e04c634750` | `uv run xcore network reset --interface enx00e04c634750 --address 192.168.2.100/24` |
| `check` | 执行离线测试、lint 和格式检查 | `uv run xcore check` | `uv run xcore check --fix --timeout 60` |

网卡名称按实际环境替换。`network configure/reset` 默认地址为 `192.168.2.100/24`，只操作电脑的活动网卡配置；`reset` 不删除 NetworkManager 已保存的配置。`check --fix` 会应用代码 lint／格式修正，再运行测试。网络诊断和环境检查通过后，仍需用 `status` 验证完整 SDK 通信。

## 实测记录与当前范围

状态日志保存在本机 `logs/direct_sdk_read_20261004.json` 和 `logs/cli_status_20261004.json`；原生运动日志为 `logs/native_motion_test_20261004.json` 和 `logs/native_motion_return_20261004.json`，测试脚本同目录留存。首次使用 0.05° 全轴容差超时，返回测试使用框架默认 0.2° 容差成功，详见开发文档。日志目录不纳入 Git。目前框架覆盖六轴状态查询、网络诊断和非实时关节运动接口；ROS2 仿真、连续实时跟随及夹爪适配尚未实现。
