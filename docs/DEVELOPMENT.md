# xCore SDK 项目搭建与开发

首次使用见 [README](../README.md)。本文命令默认在 `xcoresdk-python/` 目录执行。当前框架参考相邻 `agilexrobotics` 项目的 CLI、连接层、驱动层和测试组织方式，通信采用 xCore SDK 的以太网接口。

## 环境与安装

本次验证组合：Linux x86_64、CPython 3.11.16、SDK 0.7.1、控制器 3.2.1。厂商版本要求见 [SDK 原始说明](README.md)。当前上游代码 commit 为 `6691b0a`；本地新增框架与厂商示例分开存放。

```bash
uv sync --frozen --python 3.11
uv run xcore doctor
uv run xcore check
```

如果已有 pyenv 解释器，可用 `uv sync --frozen --python /home/knight/.pyenv/versions/3.11.16/bin/python`。`.python-version` 指定 3.11，`pyproject.toml` 当前要求 `>=3.11,<3.12`，`uv.lock` 锁定依赖。运行库无需额外 Python 依赖，开发组使用 pytest 和 Ruff；默认 `uv sync` 安装开发组。

厂商 `.so/.pyd/.dll` 通过 Release 单独分发，不在 Git 中。默认搜索路径为：

| 平台 | 扩展目录 |
| --- | --- |
| Linux x86_64 | `Release/linux/` |
| Linux aarch64 | `Release/linux/arm/` |
| Windows x86_64 | `Release/windows/` |

自动加载与当前解释器兼容的扩展后缀。显式 `--sdk-dir` 优先于 `XCORE_SDK_DIR`，然后使用上述默认目录。Windows 还需要配套 DLL。跨平台加载路径已实现，实机验证仅在当前 Linux x86_64 环境完成。

## 网络配置与本次排查

电脑原有 `192.255.2.100/24` 是 DHCP 地址；网关 `192.255.2.254` 的网页显示 TP-Link 工业路由器。不能因只看到 DHCP 子网就认定机器人不在同一二层网络。

本次在有线网卡上增加候选网段地址并逐网段探测，发现 **`192.168.2.160`** 的 SDK TCP 6666 端口可达，再通过原生 `xMateRobot.connectToRobot` 和 `robotInfo` 确认机器人身份。电脑增加 `192.168.2.100/24` 后，路由为 `192.168.2.160 dev enx00e04c634750 src 192.168.2.100`；该通信不再走 VPN 的默认路由。

```bash
uv run xcore network --ip 192.168.2.160
uv run xcore status --ip 192.168.2.160
```

诊断输出包含接口 IPv4 地址、路由、邻居表和 TCP 6666 可达性。`same_subnet_on_route_interface` 用于核查流量是否走有同网段地址的接口。代理可能接受 TCP 连接却无法完成机器人协议，所以端口探测不能替代 SDK 查询。

临时添加／移除地址：

```bash
uv run xcore network configure --interface enx00e04c634750 --address 192.168.2.100/24
uv run xcore network reset --interface enx00e04c634750 --address 192.168.2.100/24
```

底层使用 `nmcli device modify` 并核查地址是否实际生效；重连或重启后不保证保留。`reset` 移除当前地址，不会删除已保存的连接配置。当前电脑已另行完成永久配置，连接 UUID 为 `1ab4e868-1502-357e-af94-180b52fcaf9a`，保存的是 DHCP 加附加静态地址：

```bash
nmcli -f ipv4.method,ipv4.addresses connection show 1ab4e868-1502-357e-af94-180b52fcaf9a
```

另一台机器需要永久配置时，应先查询其自己的连接 UUID，在 NetworkManager 中增加附加地址；本次实际执行的配置命令为：

```bash
nmcli connection modify 1ab4e868-1502-357e-af94-180b52fcaf9a +ipv4.addresses 192.168.2.100/24
nmcli device reapply enx00e04c634750
```

要撤销本次保存的地址，对同一连接执行 `-ipv4.addresses 192.168.2.100/24`，然后 `device reapply`。以上操作可能受系统 NetworkManager 权限限制。没有修改机械臂 IP 或路由器配置。

## 先验证原生 SDK

本次先直接导入厂商扩展，创建 `xMateRobot()`，连接 `192.168.2.160`，逐项检查 `ec['ec'] == 0`，最后断开连接。以下是与实际测试相同的最小查询流程，可在项目根目录通过 Python 复现：

```python
import sys
sys.path.insert(0, "Release/linux")
import xCoreSDK_python as sdk

robot = sdk.xMateRobot()
robot.connectToRobot("192.168.2.160")

def call(name, *args):
    ec = {}
    result = getattr(robot, name)(*args, ec)
    if ec.get("ec") != 0:
        raise RuntimeError(f"{name}: {ec}")
    return result

try:
    info = call("robotInfo")
    print(info.type, info.version, info.joint_num)
    print(call("powerState"), call("operateMode"), call("operationState"))
    print(call("jointPos")[:info.joint_num])
finally:
    call("disconnectFromRobot")
```

实测 `XMC7-R850-W4X3B4`、控制器 3.2.1、6 轴；SDK 为 0.7.1。首次原生查询测试读取了关节、法兰位姿、软限位、标称与校准 DH；随后另外进行了原生 SDK 运动测试，结果见下文。厂商部分 `example/` 包含上电和移动，不能仅根据文件名把它当成状态查询脚本。

## 架构与扩展

```text
uv run xcore
  → cli.py 参数校验、JSON 输出、进程超时
  → worker.py 私有 SDK 子进程
  → commands.py 单次会话分派
  → reader.py RobotConnection / driver.py RobotDriver
  → sdk.py 厂商扩展 → 以太网 → xCore 控制器

network → network.py → ip / NetworkManager / TCP 探测
check   → pytest / Ruff（不访问机械臂）
```

| 文件 | 职责 |
| --- | --- |
| `pyproject.toml`、`uv.lock` | 解释器约束、依赖、构建和 `xcore` 入口 |
| `src/xcoresdk_python/sdk.py` | 按平台与 ABI 延迟加载扩展，环境检查 |
| `reader.py` | 建连／断连、错误码检查、机器人信息与反馈转换 |
| `driver.py` | 显式上电、模式、停止、带条件检查的非实时运动 |
| `commands.py` | 将 CLI 请求转换为会话中的 SDK 调用 |
| `worker.py`、`cli.py` | 隔离原生阻塞调用、参数与文件输出 |
| `network.py` | 网络诊断及临时地址配置 |
| `tests/` | 使用假 SDK 验证异常、数据解析、运动条件和进程超时 |
| `example/`、`Release/` | 厂商示例、类型声明及本地二进制 |

Python 应用可复用导出的类：

```python
from xcoresdk_python import RobotConnection

with RobotConnection("192.168.2.160") as arm:
    print(arm.status())
    print(arm.dh(nominal=True))
```

`RobotDriver` 继承连接层并提供 `power(on)`、`mode(name)`、`stop()`、`movej(target_rad, ...)`。Python 类直接调用没有 CLI 的进程超时保护，嵌入应用需管理自己的生命周期和等待策略。每个 CLI 查询建立一个会话；`monitor` 在一个会话内循环采样。不要并行启动多个连接同一机械臂的命令；SDK 断连可能停止已有运动。

## 命令与参数

统一格式为 `uv run xcore COMMAND [OPTIONS]`，公共参数放在子命令之后，`uv run xcore COMMAND --help` 可查看完整帮助。

| 命令 | 参数／行为 |
| --- | --- |
| `doctor` | 检查 Python、二进制与必要 API，无硬件连接 |
| `network [check]` | 检查 IP、路由与 TCP 6666 |
| `network configure/reset` | `--interface` 必填；`--address` 默认 `192.168.2.100/24` |
| `status` | 型号、版本、模式、电源、运行状态、关节和法兰位姿 |
| `info`、`joints`、`limits` | 机器人身份、六轴角度、当前控制器软限位 |
| `pose` | `--frame flange` 默认法兰相对基座；`--frame tool` 为末端相对当前参考坐标系 |
| `dh` | 默认校准参数，`--nominal` 返回标称参数 |
| `monitor` | `--duration 5`、`--interval 0.5`，单位秒，默认非实时采样 |
| `power on/off` | 显式上电／下电并读取反馈 |
| `mode manual/automatic` | 显式设置模式并读取反馈 |
| `stop` | 请求停止并读取运行状态 |
| `movej` | `--joints` 六个绝对目标必填；`--unit deg/rad` 默认 `deg` |
| `move-joint` | `--joint 1..6`、`--delta-deg` 必填，单轴相对当前角度偏移 |
| `check` | 离线测试、lint、格式检查；`--fix` 先应用代码修正 |

| 公共参数 | 默认／用途 |
| --- | --- |
| `--ip` | `XCORE_ROBOT_IP` 或 `192.168.2.160` |
| `--local-ip` | `XCORE_LOCAL_IP` 或空串，由 SDK 选择；当前非实时查询不必填写 |
| `--sdk-dir` | 覆盖扩展目录，也支持 `XCORE_SDK_DIR` |
| `--timeout` | 默认 20 秒；SDK 命令的子进程整体等待上限 |
| `--output PATH` | 保存成功 JSON，创建父目录，不覆盖已有文件 |

`network` 每条系统查询最长等待 5 秒，TCP 探测最长 `min(2, --timeout)` 秒；`check` 的 `--timeout` 分别用于每个检查进程。这两类命令不经过 SDK 子进程，因此该参数不是它们的全程总时限。

| 运动参数 | 默认／含义 |
| --- | --- |
| `--speed` | 50 mm/s，SDK 要求范围 5..4000 |
| `--max-step-deg` | 10 度，每轴目标相对当前值的最大差值 |
| `--tolerance-deg` | 0.2 度，到位判断误差 |
| `--motion-timeout` | 15 秒，到位轮询上限 |

`--timeout` 必须大于 `--motion-timeout + 2`；采样命令必须大于 `--duration + 5`。所有浮点参数必须有限；采样最多按时长／间隔配置 10000 个间隔，实际采样频率受 SDK 查询耗时影响。

成功返回 JSON `{ "ok": true, "command": "...", "result": ... }` 到 stdout；错误写入 stderr。退出码：成功 0、执行失败 1、参数错误 2、进程等待超时 124、键盘中断 130。SDK 自己的日志由私有工作进程捕获，CLI 提取约定的结果行。

## 反馈格式与建模参数

此 SDK／控制器组合的 `jointPos` 返回 12 个值，而 `robotInfo.joint_num` 为 6。框架只将前六项用于机械臂角度与运动计算，后六项保存为 `extra_sdk_values`，并保留完整 `raw`。额外槽位在本次测试中全为零，不能据此认定本机存在 12 轴。

DH 返回 28 项；前 24 项整理为六行 `rows`，尾部四项零值保留在 `extra_sdk_values`，完整结果保留在 `raw`。每行单位为 `[Alpha(度), A(毫米), D(毫米), Theta(度)]`。实测标称值：

| 轴 | Alpha | A | D | Theta |
| --- | ---: | ---: | ---: | ---: |
| 1 | 0 | 0 | 296 | 0 |
| 2 | -90 | 0 | 0 | -90 |
| 3 | 180 | 490 | 0 | -90 |
| 4 | -90 | 0 | 360 | 0 |
| 5 | 90 | 0 | 150 | 0 |
| 6 | -90 | 0 | 127 | 0 |

可用 `uv run xcore dh --output logs/dh-calibrated.json` 导出本机校准值。转换 URDF 前仍需确认厂商 DH 坐标系约定、零位、方向和工具变换；不能只凭上述长度推断 CR35-45/1.9C 与本机结构同构。

实测软限位为 A1/A4/A5/A6 ±360°、A2 ±135°、A3 [-170°, 140°]。这些是控制器当前配置，不能当作机械本体的绝对范围，也不能用手册的统一 ±175°覆盖它们；框架每次运动前读取实际配置。

## 运动执行与验证边界

运动之前检查：六个有限目标值、空闲、上电、自动模式、软限位已启用、目标在限位内、每轴跨度不超过配置上限。条件不满足时不发送运动准备指令；框架不会在移动命令中自动上电或切换模式。

通过检查后调用 `setMotionControlMode(NrtCommandMode)`、`moveReset`、`moveAppend([MoveAbsJCommand(target, speed, 0)], id)`、`moveStart`。转弯区为 0。只有运行状态回到空闲且所有关节误差满足容差才报告到位；仅收到 `moveStart` 成功不算完成。执行异常或到位超时尝试 `stop`。

SDK 原生调用可能阻塞，CLI 到总时限后终止工作进程，必要时强制结束。中断清理及 `stop` 都属于尽力执行，网络断开或原生调用卡住时不能保证机器人停止。一次性 CLI 也不适合作为外部控制器运行时的状态镜像服务，持续服务需复用单一连接。

2026-10-04 验证记录：原生 SDK 的完整反馈读取成功；CLI `status`、`dh --nominal`、`limits` 和网络诊断成功；离线测试覆盖额外数组槽位、数据错误、条件拒绝、到位与超时处理。原生 SDK 的模式切换、上电、非实时运动、停止和下电已随后实测成功；CLI 运动封装、RCI 实时控制／ROS2 驱动仍未实机验证。

## 2026-10-04 原生 SDK 运动实测

在用户授权运动测试后直接使用厂商扩展执行，未通过 `RobotDriver` 或 CLI 运动封装。机械臂 IP 为 `192.168.2.160`，测试前为下电／手动／空闲。核对型号、六轴软限位、当前关节和外部轴槽位后，设置自动模式、非实时命令模式并上电。

使用 `MoveAbsJCommand(target, 10, 0)`：速度 10 mm/s，转弯区 0；同时设置 `command.jointSpeed = 0.01`，限制关节速度为最大值的 1%。其余五轴目标取测试前值，第六轴目标增加 1°。每次通过 `moveReset → moveAppend → moveStart` 下发，并轮询实际关节和运行状态。

| 阶段 | 实测结果 |
| --- | --- |
| 测试前 J6 | 12.597034° |
| 第一次运动后 J6 | 13.598204°，实际增加约 1.001° |
| 第一次全轴到位判断 | 容差 0.05°，10 秒超时；其他轴最终最大偏差约 0.080°，停止／下电／恢复手动成功 |
| 返回测试 | 使用框架默认 0.2° 全轴容差，向第一次测试前保存的六轴位置运动 |
| 返回运动到位 J6 | 12.596581°，相对起始 J6 误差约 0.000453° |
| 返回最大六轴误差 | 0.079665°；61 次反馈中观测到 moving → idle，约 3.19 秒到位 |
| 最终状态 | stop、下电、恢复手动、断开均成功；最终 off／manual／idle |

第一次超时不代表未发生运动，实际反馈已证明第六轴转动。该测试的全轴到位条件比框架默认容差严格。返回测试保留原始目标位置，没有改用当前位置覆盖目标；在 0.2° 容差内确认返回。其他轴约 0.08° 的偏差也一并记录，后续高精度定位需要另外分析其来源，不能据本次小幅测试声称高精度轨迹控制已验证。

日志和当次原生测试脚本保存在本机（Git 忽略 `logs/`）：

- `logs/native_motion_test_20261004.json`／`.py`：第一次 +1° 运动及超时清理记录。
- `logs/native_motion_return_20261004.json`／`.py`：返回原始位置的成功记录，含逐次关节角和运行状态。

返回脚本依赖第一次日志中保存的原始姿态，是当次实验记录；重跑前需要重新核对现场状态和目标。本次仅验证小幅非实时关节运动及状态恢复，未验证大范围运动、连续实时跟随或 CLI 运动封装。

## 日常开发

```bash
uv run xcore --help
uv run xcore move-joint --help
uv run xcore check
uv run xcore check --fix
```

测试采用假 SDK，不访问网卡或控制机械臂；进程超时测试使用真实睡眠子进程核查清理。增加命令时先实现连接／驱动方法，再在 `commands.py` 分派、`cli.py` 定义参数，并针对错误和行为边界补测试。新硬件方法应记录实机验证状态。

ROS2 状态镜像可基于 `joints()['rad']` 适配 `JointState`，但准确的旧款 R850 URDF、关节名称及方向仍需核对。GELLO、夹爪和连续实时跟随需要独立适配，不应直接复制相邻项目的 SocketCAN 或关节限位常量。
