# xCore SDK 项目搭建与开发

首次使用见 [README](../README.md)。本文命令默认在 `xcore-sdk-python/` 目录执行。工程结构以 `../../agilex/agilex-controller` 的跟随启动流程及其 `agilex_sdk_python` 包的 CLI、连接层、驱动层和测试组织为参考；`../../gello for CR7` 用于参考 CR7 SDK 接口与实时运动控制。通信采用 xCore SDK 的以太网接口。

## 环境与安装

本次验证组合：Linux x86_64、CPython 3.11.16、SDK 0.7.1、控制器 3.2.1。厂商版本要求见 [SDK 原始说明](README.md)。当前上游代码 commit 为 `6691b0a`；本地新增框架与厂商示例分开存放。

```bash
uv sync --frozen --python 3.11
uv run xcore-sdk-python doctor
uv run xcore-sdk-python check
```

如果已有 pyenv 解释器，可用 `uv sync --python /home/knight/.pyenv/versions/3.11.16/bin/python`。`.python-version` 指定 3.11，`pyproject.toml` 当前要求 `>=3.11,<3.12`。跟随功能增加 numpy、pyzmq 和 Dynamixel SDK 依赖；依赖变更后使用 `uv lock` 更新锁文件，再用 `uv sync --frozen` 安装锁定版本。开发组使用 pytest 和 Ruff。

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
uv run xcore-sdk-python network --ip 192.168.2.160
uv run xcore-sdk-python status --ip 192.168.2.160
```

诊断输出包含接口 IPv4 地址、路由、邻居表和 TCP 6666 可达性。`same_subnet_on_route_interface` 用于核查流量是否走有同网段地址的接口。代理可能接受 TCP 连接却无法完成机器人协议，所以端口探测不能替代 SDK 查询。

临时添加／移除地址：

```bash
uv run xcore-sdk-python network configure --interface enx00e04c634750 --address 192.168.2.100/24
uv run xcore-sdk-python network reset --interface enx00e04c634750 --address 192.168.2.100/24
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
uv run xcore-sdk-python
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
| `src/xcore_sdk_python/sdk.py` | 按平台与 ABI 延迟加载扩展，环境检查 |
| `reader.py` | 建连／断连、错误码检查、机器人信息与反馈转换 |
| `driver.py` | 显式上电、模式、停止、带条件检查的非实时运动 |
| `commands.py` | 将 CLI 请求转换为会话中的 SDK 调用 |
| `worker.py`、`cli.py` | 隔离原生阻塞调用、参数与文件输出 |
| `network.py` | 网络诊断及临时地址配置 |
| `tests/` | 使用假 SDK 验证异常、数据解析、运动条件和进程超时 |
| `example/`、`Release/` | 厂商示例、类型声明及本地二进制 |

Python 应用可复用导出的类：

```python
from xcore_sdk_python import RobotConnection

with RobotConnection("192.168.2.160") as arm:
    print(arm.status())
    print(arm.dh(nominal=True))
```

`RobotDriver` 继承连接层并提供 `power(on)`、`mode(name)`、`stop()`、`movej(target_rad, ...)`。Python 类直接调用没有 CLI 的进程超时保护，嵌入应用需管理自己的生命周期和等待策略。每个 CLI 查询建立一个会话；`monitor` 在一个会话内循环采样。不要并行启动多个连接同一机械臂的命令；SDK 断连可能停止已有运动。

## 命令与参数

统一格式为 `uv run xcore-sdk-python COMMAND [OPTIONS]`，公共参数放在子命令之后，`uv run xcore-sdk-python COMMAND --help` 可查看完整帮助。

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
| `--speed` | **1000 mm/s**；范围 5..4000，可显式指定 50 降速 |
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

可用 `uv run xcore-sdk-python dh --output logs/dh-calibrated.json` 导出本机校准值。转换 URDF 前仍需确认厂商 DH 坐标系约定、零位、方向和工具变换；不能只凭上述长度推断 CR35-45/1.9C 与本机结构同构。

实测软限位为 A1/A4/A5/A6 ±360°、A2 ±135°、A3 [-170°, 140°]。这些是控制器当前配置，不能当作机械本体的绝对范围，也不能用手册的统一 ±175°覆盖它们；框架每次运动前读取实际配置。

## 运动执行与验证边界

运动之前检查：六个有限目标值、空闲、上电、自动模式、软限位已启用、目标在限位内、每轴跨度不超过配置上限。条件不满足时不发送运动准备指令；框架不会在移动命令中自动上电或切换模式。

通过检查后调用 `setMotionControlMode(NrtCommandMode)`、`moveReset`、`moveAppend([MoveAbsJCommand(target, speed, 0)], id)`、`moveStart`。转弯区为 0。只有运行状态回到空闲且所有关节误差满足容差才报告到位；仅收到 `moveStart` 成功不算完成。执行异常或到位超时尝试 `stop`。

SDK 原生调用可能阻塞，CLI 到总时限后终止工作进程，必要时强制结束。中断清理及 `stop` 都属于尽力执行，网络断开或原生调用卡住时不能保证机器人停止。一次性 CLI 也不适合作为外部控制器运行时的状态镜像服务，持续服务需复用单一连接。

2026-10-04 验证记录：原生 SDK 的完整反馈读取成功；CLI `status`、`dh --nominal`、`limits` 和网络诊断成功；离线测试覆盖额外数组槽位、数据错误、条件拒绝、到位与超时处理。原生 SDK 的模式切换、上电、非实时运动、停止和下电已随后实测成功；CLI `mode automatic` 和 `power on` 也已实机验证成功：手动模式直接上电返回 -514，切换自动模式后上电成功。随后 CLI `move-joint` 和 `movej` 已使用新默认速度完成 J6 +1°／返回原始位置验证；实时跟随仍未实机验证。

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

返回脚本依赖第一次日志中保存的原始姿态，是当次实验记录；重跑前需要重新核对现场状态和目标。本轮只验证小幅非实时关节运动及状态恢复。随后进行的速度对比和 CLI 实测见下一节，连续实时跟随仍未验证。

## 2026-10-04 第六轴速度实验与默认值调整

用户要求测量 `move-joint` 对应运动的最高速度并调整默认值，明确允许 J6 在 −180°～180° 内转动，并确认已安装夹爪、工具与线缆有足够空间、运动范围无人且无障碍物。

实验使用与 CLI 相同的 `MoveAbsJCommand(target, speed, 0)` 路径，保留默认 `jointSpeed = -1`，由 SDK 根据 speed 参数计算速度；没有设置 `jointSpeed = 0.01`。因此本轮与前述 1% 关节速度的小幅原生测试不同。控制器配置负载为 1.95 kg，质心为 `[0.05438, 0.03214, 0.06687]` m，加速度与加加速度比例均为 1.0；未修改这些配置或控制器保护参数。

初次计划使用 ±90°。`speed=50` 的首段定位在 30 秒内未完成，脚本请求停止；紧接着下电因尚在减速而返回 `-20`。随后重新检查并确认已空闲，保持上电。该次失败日志保留，后续清理流程先等待空闲再进行其他操作。根据基线约 3.3°/s 的速度，正式比较范围缩为 ±30°，每档完成正反向各一段 60° 行程，其他五轴始终使用实验前保存的固定目标。

每次采样依次读取 `jointPos`、`jointVel`、`operationState`，然后等待 5 ms；实际采样间隔包含通信耗时，最大间隔约 51 ms。六档共记录 10352 个运动样本，速度来自 SDK 的 `jointVel` 反馈并转换为 °/s。耗时从调用 `moveStart` 前到读到空闲且误差 ≤0.2° 为止，包含协议等待、加减速和到位等待。

| speed 参数（mm/s） | 正向 60°（s） | 反向 60°（s） | J6 峰值（°/s） | 六轴最大到位误差（°） |
| ---: | ---: | ---: | ---: | ---: |
| 50 | 20.361 | 20.313 | 3.36 | 0.0805 |
| 150 | 11.196 | 11.204 | 6.30 | 0.0806 |
| 350 | 5.249 | 5.246 | 14.64 | 0.0807 |
| 650 | 3.190 | 3.187 | 26.34 | 0.0808 |
| 1000 | 2.345 | 2.302 | 39.72 | 0.0809 |
| 4000 | 2.300 | 2.343 | 39.84 | 0.0809 |

最高档观测峰值为 39.84°/s；两方向均有超过 1.16 秒的采样区间达到峰值的 90% 以上，高速段样本中位数约 38.28°/s。`1000` 与 `4000` 的峰值和耗时差异很小，说明本次条件下已达到该命令路径的速度平台。这里记录的是有限频率采样的观测峰值，不能据此推导机器人所有姿态、负载及控制器配置下的绝对机械极限。

本地 `setDefaultSpeed` 文档描述了 speed 到关节速度百分比的分档换算；实验直接给每条 `MoveAbsJCommand` 传入 speed。不同档位的具体响应以上表实测为准，不能把文档百分比直接等同于本机 °/s。

根据速度实验结果及用户最终选择，将 CLI `movej`、`move-joint` 及 Python `RobotDriver.movej` 的默认值统一为 **1000 mm/s**，共用 `driver.DEFAULT_SPEED`；合法范围仍为 5..4000，目标跨度和软限位检查仍生效。显式 `--speed 50` 可以覆盖默认值。成功返回中新增 `speed_mm_s`，方便确认当前命令实际使用的参数。

默认值曾设为 4000 时，未传 `--speed`，用 CLI `move-joint --joint 6 --delta-deg 1 --max-step-deg 2` 实测成功；再用 CLI `movej` 下发整个实验前保存的六轴绝对目标，确认返回。当次两条命令都返回 `speed_mm_s: 4000`，该历史日志保留原值；当前默认值已改为 1000。最大到位误差分别为 0.0833°、0.0776°。最终 J6 约 13.596°，机械臂保持上电／自动／空闲。以上仅验证 J6 的速度及两条命令的基本执行，未测其他轴的最高速度。

离线检查通过 40 项测试，包括默认速度和显式低速参数从 CLI 到驱动／SDK 指令的传递。实验文件保存在本机 `logs/`（不纳入 Git）：

- `speed_preflight_20261004.json`：状态、配置负载、加速度和静止速度反馈。
- `speed_benchmark_20261004.json`：初次大行程基线的超时记录。
- `speed_benchmark_30deg_20261004.json`：完整正式实验，包含每次角度、速度和状态采样。
- `speed_summary_20261004.json`／`.csv`：每档双方向速度与耗时汇总。
- `speed_benchmark_20261004.py`、`summarize_speed_20261004.py`：当次实验和汇总脚本；实验脚本引用初次日志中的起始姿态，重跑前需重新核对现场与目标。
- `cli_default_max_speed_20261004.json`、`cli_default_max_return_20261004.json`：默认值设为 4000 时的 CLI 实测结果。

## 日常开发

```bash
uv run xcore-sdk-python --help
uv run xcore-sdk-python move-joint --help
uv run xcore-sdk-python check
uv run xcore-sdk-python check --fix
```

测试采用假 SDK，不访问网卡或控制机械臂；进程超时测试使用真实睡眠子进程核查清理。增加命令时先实现连接／驱动方法，再在 `commands.py` 分派、`cli.py` 定义参数，并针对错误和行为边界补测试。新硬件方法应记录实机验证状态。

夹爪单独适配尚未实现。跟随使用 CR7 以太网 SDK；AgileX 的 SocketCAN 通信和关节配置不用于本项目。

## 六轴示教臂跟随的工程组织

```text
xcore-controller/start_gello_follow.sh                 工作区便捷入口
  → xcore-sdk-python/scripts/start_gello_follow.sh
      → uv run --locked --project ... xcore doctor / follow-check
      → 后台 follow-server                 独占 CR7 SDK 会话
      → follow 客户端                      读取 GELLO 并发目标
      → 退出：先停客户端，再停服务端
```

脚本参考 AgileX 控制项目的预检、`flock`、`setsid`、日志、启动等待和退出清理。脚本只做编排，设备协议与运动策略留在 Python 包；没有移植 AgileX 的机械臂归位动作或 CAN 驱动。SDK 仓库中保留完整脚本实现，单独克隆该仓库即可运行。

| 模块 | 职责 |
| --- | --- |
| `scripts/start_gello_follow.sh` | 参数、串口／依赖预检、单实例锁、子进程组、启动等待、日志与清理 |
| `cli.py` | 统一 `xcore` 子命令、离线参数校验及错误出口 |
| `calibration.py` | 标定文件校验、只读预检、同姿态采样与圆周均值零位标定 |
| `dynamixel_reader.py` | 六轴 Dynamixel ID、只读串口采样与角度解缠 |
| `gello_leader.py` | 主臂偏移／方向映射、2π 分支和读取失败处理 |
| `gello_client.py` | ZMQ 客户端跟随循环、初始姿态对齐、dry-run 预览 |
| `gello_server.py` | GELLO 四方法协议、单个 SDK 会话、错误码检查与长驻服务生命周期 |
| `gello_follower.py` | CR7 RT 回调、平滑限速／限加速度、软限位交集及看门狗 |

ZMQ 采用 GELLO 的 `num_dofs`、`get_joint_state`、`command_joint_state`、`get_observations` 四方法，使用 pickle 编解码并只绑定 localhost。它是本机可信进程间接口，不应开放给不可信网络。客户端请求超时默认 2 秒；服务端状态采样由伺服线程完成，回调只使用内存目标。`follow-server` 持有长驻连接，不经过一次性 CLI 的 worker；现场不得同时运行另外的 SDK 控制会话。

### 标定和启动

```bash
uv sync --frozen --python 3.11
uv run xcore-sdk-python doctor
# 两臂摆到相同关节姿态并保持不动；方向按现场确认
uv run xcore-sdk-python follow-calibrate --ref-current --save config/cr7_calib.json
./scripts/start_gello_follow.sh
./scripts/start_gello_follow.sh --enable-motion
```

现场默认串口、IP、频率和速度参数见 [README](../README.md#六轴示教臂跟随一个脚本启动)。脚本先在打开设备之前解析参数，再执行 SDK 离线加载检查和标定／串口只读检查，关闭预检串口后再启动子进程。标定文件为六个有限 `joint_offsets`（弧度）、六个 `joint_signs`（±1），映射公式 `q = sign * (raw - offset)`。均值使用圆周统计，防止 0／2π 边界造成错误；采样中明显移动时拒绝保存。单姿态标定不能推断方向，需要 `--signs` 并逐轴预览核对。

默认脚本只读，`--enable-motion` 才启用运动；脚本统一确认后向子命令传入 `--yes`。独立启动 CLI 时服务端和客户端各自确认。服务端首个目标还需通过对齐闸门才进入 SDK RT 模式；不会自动规划从臂到主臂姿态。

脚本日志位于 `logs/follow-日期-时间-随机串/server.log`。启动失败不运行客户端；客户端报错保留其退出码。退出／信号清理先向客户端进程组发 TERM，再向服务端发 TERM，分别等待最多 15 秒，超时强制终止并提示检查控制器。单实例锁由脚本持有，子进程不继承锁描述符；该锁只约束此脚本，不能阻止手工另开的 SDK 会话。

### CR7 运动策略与退出

沿用同学项目的实时关节目标、平滑与看门狗思路，具体 SDK 调用按本项目接口校验。上电与进入 RT 在首个有效目标到达后发生：设置网络容忍、RT 控制模式、automatic、上电、获取 RT 控制器、滤波与当前姿态 MoveJ，然后注册 `JointPosition` 回调并非阻塞启动循环。

跟随默认速度上限 3 °/s，与非实时 `movej` 的 1000 mm/s 独立。驱动使用本地保守限位与实际控制器 `PyTypeVectorArrayDouble2.content()` 软限位的交集；控制器软限位关闭或反馈异常时拒绝创建驱动。目标必须是六个有限值，超限目标裁剪到交集边界。回调按有限 dt 更新平滑输出，避免长时间间隔造成目标跳变。看门狗检查目标断流、跟踪偏差、回调停滞和控制器错误；伺服线程异常记录中止原因并尝试停止 RT。

正常退出请求回调 `setFinished`、`stopMove`，恢复 NRT／manual，再断开连接；不自动回零或下电。代码避免直接阻塞调用旧参考中可能持有 GIL 的 `stopLoop`。线程异常、网络中断或强制终止时，停止属于尽力执行，需核对示教器反馈。

### 验证记录和边界

运行依赖 `numpy`、`pyzmq`、`dynamixel-sdk` 已解析到 `uv.lock` 并安装；新机器使用 `uv sync --frozen --python 3.11`。离线测试覆盖圆周标定、格式错误、SDK 错误码、控制器限位、非有限目标拒绝、RT 插值与异常停机，以及真实 shell 的服务端启动失败、客户端报错、信号退出和进程组清理；硬件接口均采用替身，不触发运动。

当前扩展已离线检查到 `getRtMotionController`、`JointPosition`、`RtControllerMode`、`MotionControlMode.RtCommandMode`。原生 SDK 和非实时控制的实机记录见前文。新的实时跟随尚未在 CPython 3.11 + SDK 0.7.1 + 当前 CR7 上完成真机启停／连续跟随验收；同学项目实测只作为驱动实现参考。

现场验收顺序：核对 IP／SDK 与串口，完成静止同姿态标定，逐轴 dry-run 核对方向和分支，再验证低速静止启停、短行程跟随及 Ctrl+C 收尾，确认后再调整速度与负载条件。此任务只接入六轴跟随；夹爪和仿真显示没有实现。
