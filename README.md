# xCore SDK Python：CR7 命令行控制

基于珞石 SDK，提供统一入口 `uv run xcore-sdk-python COMMAND [OPTIONS]` 和可复用的 Python 连接／驱动层。开发细节见 [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)，厂商 SDK 安装说明见 [docs/README.md](docs/README.md)。

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
cd /home/knight/projects/xcore/xcore-sdk-python
uv sync --frozen --python 3.11
uv run xcore-sdk-python doctor
uv run xcore-sdk-python network
uv run xcore-sdk-python status
```

仓库已纳入 SDK 0.7.1 的 `Release/linux/xCoreSDK_python.cpython-311-x86_64-linux-gnu.so`。Linux x86_64、CPython 3.11 环境 clone 后即可使用，无需另行下载。其他平台或 Python 版本仍需从 [官方 SDK v0.7.1 Release](https://github.com/RokaeRobot/xCoreSDK-Python/releases/tag/v0.7.1) 获取匹配库。`doctor` 检查扩展加载和 API，不连接机械臂。

项目当前固定 Python 3.11。SDK 不是只能使用 Python 3.10，但 `cpython-310` 二进制不能直接用于 3.11，单纯重命名文件也不能改变 ABI。更换解释器时需同时更换匹配扩展并调整项目版本约束。

默认机械臂 IP 为 `192.168.2.160`，可显式指定或通过环境变量配置：

```bash
uv run xcore-sdk-python status --ip 192.168.2.160 --timeout 20
export XCORE_ROBOT_IP=192.168.2.160
```

本次已在 NetworkManager 的有线连接配置中保存附加地址 `192.168.2.100/24`，保留原有 DHCP 配置；临时探测地址已清理。在另一台电脑上，可先通过以下命令临时添加同网段地址，接口名按实际网卡替换：

```bash
uv run xcore-sdk-python network configure --interface enx00e04c634750 --address 192.168.2.100/24
uv run xcore-sdk-python network
uv run xcore-sdk-python status
```

`network configure/reset` 仅修改当前活动网卡配置，需要系统允许当前用户操作 NetworkManager。永久配置方法及本次保存的连接 UUID 见开发文档。`network` 中 TCP 端口可达不代表 SDK 握手成功，应以 `status` 返回 `ok: true` 为准。

SDK 建连可能重置运动相关状态，断开连接可能停止已有运动。状态查询应在机械臂空闲、没有其他 SDK 控制会话时执行。

## 六轴示教臂跟随：一个脚本启动

工程组织以相邻 AgileX 项目为参考：顶层脚本管理设备预检、单实例锁、子进程、日志和退出；CLI、GELLO 客户端、ZMQ 服务端、CR7 驱动分别放在包内。CR7 的实时接口和回调平滑策略参考 `../gello for CR7`。

### 1. 首次标定

连接 GELLO 串口，将两臂摆到相同关节姿态并保持不动。只读取现场参考姿态，不发送运动目标：

```bash
uv run xcore-sdk-python follow-calibrate --ref-current \
  --serial /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0 \
  --save config/cr7_calib.json
```

标定文件不纳入 Git，已有文件不会被覆盖。单姿态标定无法自动判断轴方向；默认六轴方向均为 `+1`，若现场方向不同，用 `--signs S1 S2 S3 S4 S5 S6` 指定六个 `+1`／`-1`，再重新标定并逐轴核对 dry-run。完整帮助：`uv run xcore-sdk-python follow-calibrate --help`。

首次仅使用 shell 标定时，GELLO 保持 CR7 六轴均为 0° 的标准姿态：

```bash
./scripts/start_gello_follow.sh --enable-motion --calibrate-zero
```

脚本先把 CR7 移至全零，采集偏移后进入跟随。已有标定不会覆盖；
重标定请通过 `--calib` 指定新路径。日常启动复用标定，主臂无需回到零位。
当前的六轴方向沿用本机配置；不同装配需使用上方标定 CLI 的 `--signs`。

### 2. 预览，再启动跟随

在 SDK 仓库根目录运行：

```bash
# 只读预览主臂映射与从臂反馈
./scripts/start_gello_follow.sh

# 按已有标定对齐到 GELLO 当前姿态，再进入六轴跟随；输入 y 确认
./scripts/start_gello_follow.sh --enable-motion
```

在控制器目录 xcore-controller 中也可直接使用 `./start_gello_follow.sh`，但控制器
顶层入口默认同时启用夹爪，需先启动夹爪服务；仅六轴使用 `--arm-only`。
SDK 的独立脚本默认仅六轴，可用 `--gripper-host` 或 `XCORE_GRIPPER_HOST` 启用夹爪。
脚本先运行离线参数检查；启用夹爪时在打开 GELLO／连接 CR7 前检查夹爪服务。
随后运行 `doctor` 和只读 `follow-check`，启用运动时先按已有标定
移动 CR7 到静止的 GELLO 当前目标，对齐完成后关闭准备会话，再启动服务端
并等待就绪，最后启动主臂客户端。准备期间保持 GELLO 不动。服务端独占一个 SDK 连接；跟随期间不要另开 `status`、`power`、`movej` 等连接同一机械臂的命令。

| 脚本参数 | 用途／默认值 |
| --- | --- |
| `--ip` | CR7 地址；`XCORE_ROBOT_IP` 或 `192.168.2.160` |
| `--local-ip` | 已配置的本机有线地址；`XCORE_LOCAL_IP` 或 `192.168.2.100` |
| `--gello-port` | 示教臂串口；`XCORE_GELLO_PORT` 或上述 FTDI 路径 |
| `--calib` | 标定文件；SDK 仓库 `config/cr7_calib.json` |
| `--hz` | 主臂读取与发送频率；`50` Hz |
| `--port` | 本机 ZMQ 端口；`6001`，绑定 `127.0.0.1` |
| `--max-speed-deg` | 跟随关节速度上限；`XCORE_FOLLOW_MAX_SPEED_DEG` 或 `75` °/s |
| `--prepare-speed` | 启动对齐 SDK 速度；`XCORE_PREPARE_SPEED` 或 `4000` mm/s |
| `--show-state` | 循环打印关节／夹爪状态；默认关闭 |
| `--gripper-host` | 独立夹爪服务地址；可由 `XCORE_GRIPPER_HOST` 指定 |
| `--arm-only` | 仅六轴，禁用环境变量中的夹爪配置 |
| `--enable-motion` | 启用实际运动；默认只读预览 |
| `--yes` | 跳过已确认现场条件后的交互确认 |

例如 `./scripts/start_gello_follow.sh --enable-motion --max-speed-deg 10` 设置
每轴最高 `10°/s`；命令行优先于环境变量，启动时显示实际限速。
软件允许 `0 < V <= 75°/s`，这个上限不代表硬件最大速度或已经实测的速度。
加速度仍限制为 `40°/s²`，短距离运动可能达不到设定速度。调整后需重启跟随。

跟随速度与 `movej --speed 1000` 是两个独立参数，单位分别为 °/s 和 mm/s。准备及跟随均选择与实际关节反馈最近的 2π 分支。`--enable-motion` 默认先
以 `--prepare-speed 4000`（mm/s 参数）、每段 `--prepare-motion-timeout 600` 秒
移动到主臂目标，软限位和每轴 `--prepare-max-step-deg 180` 角度差限制提前校验。
正常准备直接对齐，不回零或改写标定。`--skip-prepare` 保持原手动对齐模式。

脚本默认采用当前程序允许的对齐和实时跟随速度上限；实际速度仍受控制器、
轨迹和 `40°/s²` 加速度限制。客户端默认不循环打印状态，
使用 `--show-state` 恢复逐帧显示。关闭显示时省去显示用的反馈 RPC；
初始对齐检查、控制指令、录制时实际反馈读取和故障检查继续执行。
RT 启动仍检查默认 `17.1887°` 闸门。客户端目标以 50 Hz 发送，驱动通过 SDK RT 回调平滑下发，带速度、加速度、软限位和断流检查。

按 **Ctrl+C** 结束。脚本先结束主臂客户端，再请求服务端关闭；驱动请求 RT 回调结束、`stopMove`、恢复 NRT／manual 并断开连接。不会自动回零或下电；退出后核对示教器状态。服务端日志留在 `logs/follow-*/server.log`。

### 3. 独立 CLI 入口（调试）

| 指令 | 功能 | 示例 |
| --- | --- | --- |
| `follow-check` | 只读采样示教臂并校验标定，不连接 CR7 | `uv run xcore-sdk-python follow-check --calib config/cr7_calib.json` |
| `follow-calibrate` | 用两臂相同参考姿态生成六轴零位偏移 | `uv run xcore-sdk-python follow-calibrate --ref-current --save config/cr7_calib.json` |
| `follow-server` | 独占 CR7 SDK 会话，通过 ZMQ 提供状态和目标接口 | `uv run xcore-sdk-python follow-server --local-ip 192.168.2.100` |
| `follow` | 读取 GELLO，向服务端发送六轴目标或只读预览 | `uv run xcore-sdk-python follow --dry-run --calib config/cr7_calib.json` |

`follow-server` 默认只读；真机调试时需显式加 `--enable-motion`，客户端 `follow` 省略 `--dry-run`。这两条长驻命令输出运行日志，区别于一次性查询的 JSON 输出。通常使用上面的脚本统一管理。

## CR7 六轴与外接夹爪同时跟随

先在夹爪 USB/RS485 所在电脑启动更新后的 `xcore-gripper-2F85` 服务。
在控制器目录运行 `./start_gripper.sh --serial-port <夹爪串口>`，将占位符替换为
实际夹爪适配器路径。该串口与 GELLO 串口不同；服务启动会执行夹爪激活。

同机服务使用 `127.0.0.1`；服务在另一台电脑时填写那台电脑的地址：

```bash
# 只读预览六轴、主臂闭合度，并检查夹爪服务；不发送运动指令
./scripts/start_gello_follow.sh --gripper-host 127.0.0.1

# 对齐姿态并核对标定后，启用六轴与夹爪的同时跟随
./scripts/start_gello_follow.sh --gripper-host 127.0.0.1 --enable-motion
```

同一个客户端进程每帧同步读取 ID 1～7。六轴仍通过原 ZMQ/xCore SDK 链路发送，
第七项是夹爪闭合度（0=全开，1=全闭），交给独立 TCP 工作线程。
默认六轴读取/发送 50 Hz，夹爪最多 5 Hz；工作线程只保留最新目标，不积压历史动作。
夹爪服务的 `set_target` 等待串口指令与一次反馈，不等待机械运动完成，因此能在运动中改目标。

扳机默认全开 194.8°、全闭 153°，须按实测确认；夹爪行程、速度和力度可单独设置：

```bash
./scripts/start_gello_follow.sh --gripper-host 127.0.0.1 --enable-motion \
  --gripper-open-deg 194.8 --gripper-close-deg 153 \
  --gripper-open-pos 2 --gripper-closed-pos 230 \
  --gripper-speed 150 --gripper-force 30
```

上面的实际夹爪端点是示例，需按本机行程调整；默认协议端点为 0/255，力度为 0。
`--gripper-port` 默认 5005。直接使用 `xcore-sdk-python follow` 时也支持相同参数。
不指定 `--gripper-host` 时保持原六轴模式。跟随期间不要同时运行仿真/`read`
来读取同一 GELLO 串口，也不要使用手动夹爪运动命令。

夹爪工作线程故障会使统一客户端退出，CR7 原有断流保护停止跟随；Ctrl+C/SIGTERM
退出时发送夹爪 `stop`（停止手指，不复位或自动释放）。服务端另有默认 1.5 s 的断流看门狗，
停止请求不可达时由该看门狗尝试停止；串口事务可能延迟停止，软件保护不能代替硬件急停。
看门狗故障会锁定夹爪跟随，重启夹爪服务或显式调用 SDK `stop()` 后才能重新进入。
已完成离线分流、协议、超时与退出测试；真机的连续跟随和停止效果尚待验收。

## 跟随期间记录从臂实际状态

控制器提供 `start_data_record.sh`，沿用普通跟随的标定、对齐、限速和退出流程，
准备阶段同样先对齐到主臂当前姿态；结束后由控制器的 `tools/convert_cr7.py` 复用原转换器写入 LeRobot 数据集；
转换使用独立 Python 3.12，控制进程仍使用 Python 3.11。
先启动夹爪服务，再在控制器目录运行：

```bash
./start_data_record.sh --task "pick up the object"
# 只保留原始数据，不自动转换
./start_data_record.sh --task "pick up the object" --skip-conversion
```

直接使用 SDK 也可记录，需先启动六轴 `follow-server` 和夹爪服务：

```bash
uv run xcore-sdk-python follow --gripper-host 127.0.0.1 \
  --calib config/cr7_calib.json --raw-data-root data/raw --task "pick object"
```

使用 R 开始 episode、S 保存、D 丢弃、P 查看状态、H 查看帮助。
`--start-recording` 在对齐后立即开始第一段；Ctrl+C/SIGTERM 保留未保存的
`.jsonl.partial`，转换器只处理通过 S 保存的 `.jsonl`。

每帧 raw 的 `joint_positions` 是六轴实际 SDK 反馈加实际夹爪闭合度；
`action` 是请求的六轴目标和扳机闭合度，执行时仍由 CR7 服务限位/插值，
夹爪由独立线程发送最新目标。记录不访问另一个 SDK 会话，也不再读一次 GELLO。
夹爪闭合度由实际 `position_raw` 和 `--gripper-open-pos/--gripper-closed-pos`
端点映射，不使用主臂扳机值冒充实际状态。

每行还保存两路反馈时间戳、反馈年龄及夹爪原始位置。CR7 时间戳来自同机 SDK
服务的成功读取；夹爪时间戳是客户端收到实际反馈的本机单调时间，因此异机夹爪
服务无需共享系统时钟。SDK 记录端要求 SDK 服务同机运行。
默认超过 `--record-feedback-max-age 0.75` 秒的反馈会中止跟随，保留不完整 episode。
写盘使用有界异步队列，满队列或写入失败不会静默丢帧。

CR7 记录格式只包含实测关节和夹爪，不记录 SDK 占位的零速度或零末端位姿。
离线转换自动输出七维 `observation.state` / `action`，按观测时间重采样，
另附实际采样率、反馈刷新频率和反馈年龄质量报告。

六轴和独立夹爪的统一跟随已接入，CR7 接口仍保持六轴，夹爪由独立 TCP 系统控制。
2026-10-08 已完成六轴实机跟随验证；新的默认对齐速度及独立夹爪实机联动尚待验收。
可用 `uv run xcore-sdk-python gripper-check --gripper-host 127.0.0.1` 单独检查
夹爪能力和真实位置，不连接 CR7，也不发送运动指令。

## 指令介绍

### 1. 基本用法与公共参数

所有命令均在项目根目录执行，参数放在子命令之后：

```bash
uv run xcore-sdk-python COMMAND [OPTIONS]
uv run xcore-sdk-python --help
uv run xcore-sdk-python move-joint --help
```

一次性 SDK 查询与控制命令输出 JSON；成功退出码为 `0`，执行失败为 `1`，参数错误为 `2`，等待超时为 `124`，键盘中断为 `130`。`--output` 仅保存成功结果，自动创建父目录，不覆盖已有文件。

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| `--ip` | 机械臂控制器 IPv4 地址 | `XCORE_ROBOT_IP` 或 `192.168.2.160` |
| `--local-ip` | 电脑通信地址，非实时查询通常可省略 | `XCORE_LOCAL_IP` 或由 SDK 选择 |
| `--sdk-dir` | 厂商扩展所在目录 | `XCORE_SDK_DIR` 或平台对应的 `Release/` 子目录 |
| `--timeout` | SDK 命令的整体等待上限，单位秒 | `20` |
| `--output PATH` | 将成功结果保存为新 JSON 文件 | 仅在终端输出 |

例如，指定机械臂地址并保存状态：

```bash
uv run xcore-sdk-python status --ip 192.168.2.160 --timeout 20 --output logs/status.json
```

`network` 的系统查询各有独立时限，`check` 的 `--timeout` 分别用于每个检查进程；这两类命令不使用 SDK 的整体等待时限。详细规则见 [开发文档](docs/DEVELOPMENT.md#命令与参数)。

### 2. 查询指令（Read command）

查询指令读取控制器反馈，不发送上电或移动目标。每次命令建立一个 SDK 会话，`monitor` 在同一会话中连续采样；SDK 断连可能停止已有运动，因此应在空闲且没有其他 SDK 控制会话时使用。

#### 常用指令

| 命令 | 功能 | 最简示例 | 带参数示例 |
| --- | --- | --- | --- |
| `status` | 汇总型号、版本、电源、模式、运行状态、六轴角度和法兰位姿 | `uv run xcore-sdk-python status` | `uv run xcore-sdk-python status --ip 192.168.2.160 --output logs/status.json` |
| `joints` | 读取 J1～J6 当前角度，同时输出弧度和度 | `uv run xcore-sdk-python joints` | `uv run xcore-sdk-python joints --ip 192.168.2.160 --output logs/joints.json` |
| `pose` | 读取法兰或工具末端位姿 | `uv run xcore-sdk-python pose` | `uv run xcore-sdk-python pose --frame tool --output logs/tool-pose.json` |
| `monitor` | 在一个连接中采集多次状态，结束后统一输出 | `uv run xcore-sdk-python monitor` | `uv run xcore-sdk-python monitor --duration 10 --interval 0.5 --timeout 20 --output logs/status-samples.json` |

`pose` 默认 `--frame flange`，表示法兰相对基座；`--frame tool` 表示末端相对当前参考坐标系。位姿格式为 `[x, y, z, rx, ry, rz]`，位置单位为米，姿态角单位为弧度。

`monitor` 默认采样 `5` 秒、间隔 `0.5` 秒；实际频率受 SDK 查询耗时影响。`--timeout` 必须大于 `--duration + 5`。

#### 参数核查与建模指令

| 命令 | 功能 | 最简示例 | 带参数示例 |
| --- | --- | --- | --- |
| `info` | 读取机器人型号、标识、控制器版本、轴数和 MAC | `uv run xcore-sdk-python info` | `uv run xcore-sdk-python info --ip 192.168.2.160 --output logs/robot-info.json` |
| `limits` | 读取当前控制器软限位及启用状态 | `uv run xcore-sdk-python limits` | `uv run xcore-sdk-python limits --output logs/soft-limits.json` |
| `dh` | 读取六轴 DH 参数，默认返回校准／设置后的值 | `uv run xcore-sdk-python dh` | `uv run xcore-sdk-python dh --nominal --output logs/dh-nominal.json` |

`dh --nominal` 返回标称值；每轴参数顺序为 `[Alpha(度), A(毫米), D(毫米), Theta(度)]`。`joints` 与 `dh` 同时保留 SDK 原始数组和额外槽位，六轴解析及建模约定见 [开发文档](docs/DEVELOPMENT.md#反馈格式与建模参数)。

### 3. 控制指令（Hardware command）

控制指令会改变机械臂状态或产生实际运动。`movej` 和 `move-joint` 要求机械臂处于**上电、自动模式、空闲、软限位启用**状态；它们不会自动上电或切换操作模式。原生 SDK 与 CLI 已完成第六轴运动测试。当前 `movej` 和 `move-joint` 默认使用 **`--speed 1000`**；其他关节和负载条件未做最高速度实测。

| 命令 | 功能 | 最简示例 | 带参数示例 |
| --- | --- | --- | --- |
| `mode` | 切换手动／自动模式 | `uv run xcore-sdk-python mode automatic` | `uv run xcore-sdk-python mode manual --ip 192.168.2.160` |
| `power` | 显式上电／下电，并读取电源状态 | `uv run xcore-sdk-python power on` | `uv run xcore-sdk-python power off --ip 192.168.2.160` |
| `stop` | 请求停止并读取运行状态 | `uv run xcore-sdk-python stop` | `uv run xcore-sdk-python stop --ip 192.168.2.160 --timeout 10` |
| `move-joint` | 指定单轴，相对该轴当前角度增减 | `uv run xcore-sdk-python move-joint --joint 6 --delta-deg 1` | `uv run xcore-sdk-python move-joint --joint 6 --delta-deg 1 --speed 1000 --max-step-deg 2 --tolerance-deg 0.2 --motion-timeout 15 --timeout 20` |
| `movej` | 移动到 J1～J6 的六个绝对目标角度 | `uv run xcore-sdk-python movej --joints J1 J2 J3 J4 J5 J6` | `uv run xcore-sdk-python movej --joints J1 J2 J3 J4 J5 J6 --unit deg --speed 1000 --max-step-deg 2 --tolerance-deg 0.2 --motion-timeout 15 --timeout 20` |

`movej` 示例中的 `J1 … J6` 是占位符，使用时替换为实际规划的六个目标值。`--unit` 可选 `deg`／`rad`，默认 `deg`。`move-joint --joint` 使用 `1～6` 的轴编号，`--delta-deg` 始终以度为单位，正负值分别表示沿该关节坐标正向／反向移动。

本机在手动模式下直接执行 `power on` 曾返回 `ec: -514`（上下电失败）；2026-10-04 实测先执行 `uv run xcore-sdk-python mode automatic`，再执行 `uv run xcore-sdk-python power on`，两步均成功。SDK 规定，有外接使能开关或示教器时，手动模式的软件上电受限制。`power on` 不会隐式切换模式，初次上电应按上述顺序执行；`-514` 本身是通用上电失败码，其他情况下还需结合控制器状态判断。

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
uv run xcore-sdk-python status
uv run xcore-sdk-python mode automatic
uv run xcore-sdk-python power on

# 第六轴相对当前角度增加 1°，然后相对新的当前位置减少 1°
uv run xcore-sdk-python move-joint --joint 6 --delta-deg 1 --speed 50 --max-step-deg 2
uv run xcore-sdk-python move-joint --joint 6 --delta-deg -1 --speed 50 --max-step-deg 2

# 结束后下电并切回手动模式
uv run xcore-sdk-python power off
uv run xcore-sdk-python mode manual
```

两次相反的相对移动会受实际到位误差影响；需要返回指定原始姿态时，应保存开始时的六轴角度，再用 `movej` 下发该绝对目标。需要中途停止时执行 `uv run xcore-sdk-python stop`。命令结束后不会自动下电或恢复操作模式，应按需要显式执行收尾命令。

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
| `doctor` | 检查 Python、SDK 二进制和必要 API，不连接机械臂 | `uv run xcore-sdk-python doctor` | `uv run xcore-sdk-python doctor --sdk-dir Release/linux --output logs/doctor.json` |
| `network` | 检查电脑 IPv4 地址、路由、邻居表及 SDK TCP 6666 端口 | `uv run xcore-sdk-python network` | `uv run xcore-sdk-python network check --ip 192.168.2.160 --output logs/network.json` |
| `network configure` | 给指定网卡临时增加电脑 IPv4 地址 | `uv run xcore-sdk-python network configure --interface enx00e04c634750` | `uv run xcore-sdk-python network configure --interface enx00e04c634750 --address 192.168.2.100/24` |
| `network reset` | 从指定网卡移除该活动地址 | `uv run xcore-sdk-python network reset --interface enx00e04c634750` | `uv run xcore-sdk-python network reset --interface enx00e04c634750 --address 192.168.2.100/24` |
| `check` | 执行离线测试、lint 和格式检查 | `uv run xcore-sdk-python check` | `uv run xcore-sdk-python check --fix --timeout 60` |

网卡名称按实际环境替换。`network configure/reset` 默认地址为 `192.168.2.100/24`，只操作电脑的活动网卡配置；`reset` 不删除 NetworkManager 已保存的配置。`check --fix` 会应用代码 lint／格式修正，再运行测试。网络诊断和环境检查通过后，仍需用 `status` 验证完整 SDK 通信。

## 实测记录与当前范围

状态日志保存在本机 `logs/direct_sdk_read_20261004.json` 和 `logs/cli_status_20261004.json`；原生运动日志为 `logs/native_motion_test_20261004.json` 和 `logs/native_motion_return_20261004.json`，测试脚本同目录留存。首次使用 0.05° 全轴容差超时，返回测试使用框架默认 0.2° 容差成功，详见开发文档。日志目录不纳入 Git。目前框架覆盖六轴状态查询、网络诊断、非实时运动和六轴示教臂跟随；已接入独立夹爪联动和从臂数据记录。当前 SDK／现场组合的连续跟随、夹爪联动与数据采集仍需按文档完成受控实机验证。
