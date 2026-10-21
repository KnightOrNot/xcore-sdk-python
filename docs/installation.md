# 安装与环境

## 要求

- Linux x86_64，CPython **3.11**。厂商扩展为
  `Release/linux/xCoreSDK_python.cpython-311-x86_64-linux-gnu.so`，
  不能替换为其他 ABI 的 Python。
- Git、uv、pyenv（由顶层 `scripts/setup.sh` 使用）。
- 与 CR7 控制器同一网段；默认地址见 `config/default.toml` 的 `[robot].ip`。
- GELLO 串口权限：用户属于 `dialout`。

## 安装

```bash
uv sync --project xcore-sdk-python --frozen --python 3.11
uv run --project xcore-sdk-python xcore doctor
```

`doctor` 只检查 Python、SDK 二进制和必需 API，不连接机器人。

## 首次配置

1. 修改 `config/default.toml` 的 `[robot].ip`、`[gello].port` 和
   `[network].interface`。
2. 用只读命令确认反馈：

```bash
uv run --project xcore-sdk-python xcore status --timeout 5
uv run --project xcore-sdk-python xcore follow-check
```

3. 首次标定（主臂必须与 CR7 当前姿态一致）：

```bash
uv run --project xcore-sdk-python xcore follow-calibrate --ref-current
```

标定写入 `config/cr7_calib.json`，退出姿态写入 `config/exit_pose.json`；
两者默认被 Git 忽略，换现场后必须重新采集。

## 验证

```bash
uv run --project xcore-sdk-python xcore check       # Ruff + pytest
uv run --project xcore-sdk-python xcore monitor --duration 2
```

`check` 只运行离线测试和静态检查，不访问硬件。
