# 更新记录

## 2026-10-26

- RT 启动不再调用额外 MoveJ；非阻塞调度在最终回调结束后显式 stopLoop，再 stopMove。
  close 确认伺服线程退出并报告清理异常。实机连续两轮同会话启停通过。
- GripperFollower 新增可选的启动只读轮询，供远端 arm 阶段保持反馈新鲜，默认不改变旧链路。
- `RobotDriver.movej()` 新增 `motion_mode_prepared`，供已先设置 NRT 再上电的
  独占 SDK owner 跳过重复模式切换；下发运动前再次确认上电，切换导致下电时拒绝排队，
  不自动重新上电。controller 的 prepare／Q 返回使用此路径；真机返回待现场验证。

## 2026-10-22

- 新增 `capture-pose --save FILE [--zero]`：只读记录当前六轴为返回姿态，`--zero`
  生成零位目标且不连接设备。
- 跟随启动器新增 `--return-mode start|zero|none`；默认 `zero`，
  `start` 在运动前采集姿态并在退出时返回。正常结束、SIGTERM、Ctrl+C 和故障都请求返回，
  只读预览不返回。
- 新增启动器与 CLI 回归测试。

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
