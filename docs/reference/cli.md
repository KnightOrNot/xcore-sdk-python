# CLI 参考

```bash
uv run --project xcore-sdk-python xcore [--config FILE] COMMAND [OPTIONS]
```

`--config` 选择 TOML 默认值文件，默认使用仓库的 `config/default.toml`。
公共选项：`--ip`、`--local-ip`、`--timeout`、`--sdk-dir`、`--output`；
环境变量 `XCORE_ROBOT_IP`、`XCORE_LOCAL_IP` 优先于文件，显式参数优先于两者。

## 只读命令

| 命令 | 行为 |
| --- | --- |
| `doctor` | 检查 Python、SDK 二进制和 API；不连接机器人 |
| `status` | 机器人信息、模式、关节和法兰位姿 |
| `info` | 型号、控制器版本和序列号 |
| `joints` | 六个关节角度（rad 与 deg） |
| `limits` | 控制器软限位 |
| `pose [--frame flange|tool]` | 笛卡尔位姿 |
| `dh [--nominal]` | DH 参数 |
| `monitor --duration N --interval S` | 单会话按周期采样 |
| `controller-logs` | 最近控制器错误和警告 |
| `network check|configure|reset` | 有线网络诊断与临时地址 |

## 运动命令

| 命令 | 行为 |
| --- | --- |
| `movej --joints J1..J6 [--unit deg|rad]` | 有界关节运动 |
| `move-joint --joint N --delta-deg D` | 单轴相对运动 |
| `power on|off` | 显式上下电 |
| `mode manual|automatic` | 显式切换操作模式 |
| `stop` | 受控停止 |

运动命令必须显式调用。`--speed` 是 SDK 速度参数（5..4000 mm/s），
`--max-step-deg`、`--tolerance-deg`、`--motion-timeout` 用于约束和验收。

## 跟随命令

| 命令 | 行为 |
| --- | --- |
| `follow-server [--enable-motion]` | 常驻 RT 服务；默认 dry-run |
| `follow` | 读取 GELLO 并发送六轴目标；`--gripper-host` 启用夹爪 |
| `follow-prepare` | 启动对齐到主臂当前目标 |
| `return-pose`（别名 `return-zero`） | 返回保存姿态；`--zero` 才回零 |
| `capture-pose --save FILE [--zero]` | 记录当前六轴为返回姿态；只读，无运动；`--zero` 写零目标且不连接 |
| `follow-check` | 校验标定并只读一次主臂 |
| `follow-calibrate --ref-current` | 生成现场单姿态标定 |
| `gripper-check --gripper-host H` | 只读夹爪反馈 |

关键参数：

| 参数 | 默认值 / 用途 |
| --- | --- |
| `--host` / `--port` | `follow.host` / `follow.port` |
| `--hz` | `follow.hz`，控制频率 |
| `--serial` / `--baudrate` | `gello.port` / `gello.baudrate` |
| `--calib` | `calibration.file` |
| `--gate-deg` | 对齐门限，默认约 0.3 rad |
| `--show-state` | 每周期打印状态 |
| `--dry-run` | 不发送运动 |
| `--raw-data-root` | 启用 raw 记录（需要真实跟随和夹爪） |
| `--task` / `--record-queue-size` | 记录任务与队列 |
| `--pose-file` | `return-pose` 的姿态文件（`saved` 模式） |
| `--recover` | 返回前尝试恢复一次被停止的 RT |

## 跟随启动器的返回模式

`xcore-sdk-python/scripts/start_gello_follow.sh`（以及顶层 controller 包装）在退出时
按 `--return-mode` 选择目标：

| 模式 | 行为 |
| --- | --- |
| `start` | 启动对齐前调用 `capture-pose` 记录当前实际姿态，退出时返回该姿态 |
| `saved` | 返回 `--return-pose-file`／`XCORE_EXIT_POSE` 指定的固定姿态（SDK 启动器默认） |
| `zero` | 返回六轴 0°，不连接设备即可生成目标 |
| `none` | 不自动返回，等价于 `--no-return` |

模式也可用 `XCORE_RETURN_MODE` 设置。`start` 模式的目标写入本次会话的
`logs/follow-*/exit_pose.json`，不会修改现场 `config/exit_pose.json`。
Ctrl+C、SIGTERM、正常结束和故障退出都会请求返回；只读预览不返回。
`--return-pose-file` 只在 `saved` 模式下生效。

## 退出码

成功返回 0，失败返回 1，参数或配置错误返回 2，超时返回 124，
Ctrl+C 返回 130。只读命令输出 JSON。
