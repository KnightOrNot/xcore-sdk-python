# 更新记录

## 2026-10-21

- 审查修复：支持命令前后指定配置；保持直接 SDK 准备／返回默认速度 1000 mm/s，拒绝非法类型及非有限数值。

- 新增 `config/default.toml` 和 `configuration.py`；CLI 默认值来自 TOML，
  `--config` 可在任意位置覆盖。环境变量仍然优先于文件。
- 新增 `docs/index.md`、`docs/installation.md`、`docs/reference/{cli,config,errors}.md`
  和 `CONTRIBUTING.md`。
- 统一跟随限位并通过 `follow_limits.py` 在准备阶段和执行阶段共用交集。
- 退出返回改为 `return-pose`（`return-zero` 兼容别名），返回实测保存姿态。
- 拒绝启动错误向上传播；新增配置和 CLI 测试。

## 2026-10-08

- 六轴跟随、夹爪跟随、raw 记录和现场标定入口完成。

## 早期版本

- 厂商 SDK 封装、隔离 worker、网络和只读诊断命令。
  详细历史见 `docs/CHANGELOG.md`。
