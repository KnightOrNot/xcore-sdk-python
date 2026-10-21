# 贡献与开发检查

## 规则

- 只支持 CPython 3.11 和厂商扩展 `Release/linux`。
- 默认参数写入 `config/default.toml`；不要在其他位置复制默认值。
  现场标定和退出姿态保持 Git 忽略。
- 不删除限位检查、不自动截断目标、不做无条件 ±2π 包装来规避错误。
- 不提交生成的 raw、数据集、日志、现场 JSON 或 `.venv`。

## 检查

```bash
uv run --project xcore-sdk-python xcore check
uv run --project xcore-sdk-python xcore doctor
```

`xcore check` 运行 Ruff 和 pytest，不连接机器人。涉及运动路径的修改需要
单独的真机验收；离线测试通过不能替代现场确认。

## 提交

提交前检查 `git status --short`；只包含本次任务范围。子模块先提交再更新父仓库
指针。用户只要求 commit 时不自动 push。
