# 配置参考

默认文件：[config/default.toml](../../config/default.toml)。优先级：

```text
显式命令行参数 > XCORE_ROBOT_IP / XCORE_LOCAL_IP > config/default.toml > 内置默认值
```

未知表、未知字段、非法维度或非有限数值会在连接硬件前拒绝。

## robot

```toml
[robot]
ip = "192.168.2.160"
local_ip = ""
timeout_s = 20.0
sdk_dir = ""
```

`local_ip` 为空表示不绑定本机地址；实时控制通常需要显式设置。
`sdk_dir` 为空时使用包内 `Release/linux` 扩展。

## monitor / motion

```toml
[monitor]
duration = 5.0
interval = 0.5

[motion]
speed = 1000.0
max_step_deg = 10.0
tolerance_deg = 0.2
motion_timeout_s = 15.0
```

`speed` 是 SDK 参数，范围 5..4000 mm/s；不是关节角速度。

## server

```toml
[server]
host = "127.0.0.1"
port = 6001
state_hz = 25.0
max_speed_deg = 3.0
accel_deg_s2 = 40.0
stale_timeout_s = 1.0
gate_deg = 17.1887
```

`max_speed_deg` 是 RT 六轴软件限速，上限 75；顶层 controller 使用 75。
`gate_deg` 是对齐门限（约 0.3 rad）。

## gello

```toml
[gello]
port = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0"
baudrate = 57600
signs = [1, 1, 1, 1, 1, 1]
```

`signs` 是主臂到从臂的方向映射，六个值必须为 ±1。设备层标定留在
`calibration.file`，不要与方向映射混为一谈。

## calibration / prepare / return

```toml
[calibration]
file = "config/cr7_calib.json"
samples = 30
interval_s = 0.03

[prepare]
speed = 1000.0
max_step_deg = 180.0
tolerance_deg = 0.2
motion_timeout_s = 600.0
gripper_port = 5005
gripper_timeout_s = 0.75

[return]
pose_file = "config/exit_pose.json"
speed = 1000.0
max_step_deg = 360.0
tolerance_deg = 0.2
motion_timeout_s = 600.0
```

标定和退出姿态是现场文件，通常被 Git 忽略。新 clone 不能静默退回零位。

退出返回模式由启动器的 `--return-mode`／`XCORE_RETURN_MODE` 选择，不属于本文件：
SDK 启动器和顶层 controller 的默认都是 `zero`（返回六轴 0°）；`start` 会在启动前
记录实际姿态并在退出时返回，目标写到会话 `logs/follow-*/exit_pose.json`。
`return.pose_file` 只用于直接执行 `return-pose` CLI；启动器不读取它。

## gripper

```toml
[gripper]
port = 5005
id = 7
open_deg = 194.8
close_deg = 153.0
open_pos = 0
closed_pos = 255
hz = 5.0
speed = 150
force = 0
timeout_s = 0.75
stale_timeout_s = 1.5
```

`id` 必须避开机械臂的 1..6；角度端点和原始位置端点不能相同；
`stale_timeout_s` 必须大于一个周期加请求超时。

## follow / recording

```toml
[follow]
host = "127.0.0.1"
port = 6001
hz = 50.0
gate_deg = 17.1887

[recording]
task = "CR7 GELLO teleoperation"
queue_size = 500
feedback_max_age_s = 0.75
```

`recording.task` 必须非空；记录要求真实跟随和 `--gripper-host`。

## network

```toml
[network]
interface = ""
address = "192.168.2.100/24"
```

`network configure/reset` 必须提供接口；PC 地址不能与机器人地址相同。

## 自定义实验

```bash
cp config/default.toml config/experiment.toml
uv run --project xcore-sdk-python xcore --config config/experiment.toml status
```

直接调用 SDK 的准备和返回命令保持历史默认 1000 mm/s；controller 跟随脚本会按其配置显式传入速度（当前 4000 mm/s）。
