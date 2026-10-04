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
| 测试结束状态 | `off`、`manual`、`idle` |

原先提供的 `192.168.0.160` 未完成连接；使用实际地址后原生 SDK 和 CLI 均成功。**原生 SDK 运动已验证**：自动模式、上电，第六轴低速增加约 1°，再返回测试前位置，最后恢复下电／手动／空闲。CLI 查询已经实机验证，CLI 运动封装尚未单独实测。

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

## 常用操作

```bash
uv run xcore info
uv run xcore joints
uv run xcore pose
uv run xcore limits
uv run xcore dh --nominal
uv run xcore dh --output logs/dh-calibrated.json
uv run xcore monitor --duration 5 --interval 0.5 --output logs/status-samples.json
uv run xcore check
```

输出为 JSON，`--output` 保存成功结果且不覆盖已有文件。关节输出同时包含 `rad` 和 `deg`；位姿使用米和弧度；DH 每轴为 `[Alpha(度), A(毫米), D(毫米), Theta(度)]`。

运动接口要求机械臂已上电、自动模式、空闲、软限位启用。确认现场可以运动后，可选择第六轴小幅增量测试；以下为 CLI 使用示例，原生 SDK 实测参数与结果见开发文档：

```bash
uv run xcore mode automatic
uv run xcore power on
uv run xcore move-joint --joint 6 --delta-deg 1 --speed 50 --max-step-deg 2
uv run xcore move-joint --joint 6 --delta-deg -1 --speed 50 --max-step-deg 2
uv run xcore power off
uv run xcore mode manual
```

`movej --joints J1 J2 J3 J4 J5 J6` 接收绝对目标，默认单位为度。`--speed` 是 SDK 的 mm/s 参数，不是关节角速度。`uv run xcore stop` 请求停止；超时机制约束 CLI 进程等待时间，不能保证物理停止。完整参数、执行条件和返回码见 [开发文档](docs/DEVELOPMENT.md)。

状态日志保存在本机 `logs/direct_sdk_read_20261004.json` 和 `logs/cli_status_20261004.json`；原生运动日志为 `logs/native_motion_test_20261004.json` 和 `logs/native_motion_return_20261004.json`，测试脚本同目录留存。首次使用 0.05° 全轴容差超时，返回测试使用框架默认 0.2° 容差成功，详见开发文档。日志目录不纳入 Git。目前框架覆盖六轴状态查询、网络诊断和非实时关节运动接口；ROS2 仿真、连续实时跟随及夹爪适配尚未实现。
