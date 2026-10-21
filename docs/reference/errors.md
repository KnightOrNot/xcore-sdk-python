# 故障排查

## 配置错误

`xcore: invalid configuration: ...` 会在连接硬件前退出。检查 TOML 表名、
字段拼写、端口范围和正数约束；可用 `xcore --config FILE doctor` 验证加载。

## Python ABI 或 SDK 二进制错误

`doctor` 显示 `sdk_library` 与 Python 版本。该项目只支持 CPython 3.11；
其他 ABI 的扩展会导入失败。使用 `uv sync --python 3.11` 重新创建环境。

## 网络连接错误

`connectToRobot(...) 网络异常` 表示控制器不可达、未上电、急停或网段错误。
先用 `xcore network check`、`ping 192.168.2.160` 和 `xcore status` 排查。
网络配置只改变控制电脑地址，不会运动机械臂。

## 串口打开失败

检查 `[gello].port`、`dialout` 权限和是否已有其他进程占用。GELLO 与夹爪
必须使用不同串口。

## 跟随锁被占用

已有跟随、对齐或退出返回会话。先正常退出旧会话；不要删除
`.follow.lock` 绕过。锁文件存在不代表被持有。

## 启动误差或限位失败

启动误差由 `gate_deg` 和限位交集共同约束。确认主臂姿态与从臂当前姿态，
不要通过增大阈值、自动截断或 ±2π 包装绕过。J1/J6 的历史报错属于限位问题。

## RT 停止或夹爪断流

- RT 被外部停止时，`return-pose --recover` 可尝试恢复一次再返回。
- 夹爪 `target stream timed out` 表示上次退出留下的断流状态；先结束跟随，
  再运行顶层 `./scripts/start_gripper.sh --reset-stream`。
- 夹爪反馈过期超过 `stale_timeout_s` 会报错，不要在跟随中重启夹爪服务。

## 退出姿态无效

`Invalid exit pose` 表示姿态文件缺失、格式错误或地址不匹配。检查
`exit_pose.json` 的 `robot_ip`、`schema_version` 和六个有限角度；
不要用全零文件冒充现场姿态。
