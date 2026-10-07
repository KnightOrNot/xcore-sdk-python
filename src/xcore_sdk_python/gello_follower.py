"""Six-axis GELLO robot adapter backed by xCore RT joint callbacks.

The callback interpolates cached targets; the servo thread reads feedback and
checks watchdogs. RT initialization and shutdown follow the CR7 reference
implementation. This project records its own validation status in DEVELOPMENT.md.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Dict, Optional, Sequence

import numpy as np

# Reference speed table; the callback cap is conservative for the slowest axis.
JOINT_SPEED_LIMIT_DEG = (120.0, 120.0, 180.0, 180.0, 200.0, 200.0)
DEFAULT_ARM_DOFS = 6
DEFAULT_CONTROL_HZ = 25.0  # feedback polling, independent of SDK callback cadence
DEFAULT_MAX_SPEED_DEG = 3.0
DEFAULT_ACCEL_DEG_S2 = 40.0
MAX_SPEED_DEG_CEIL = 75.0
DEFAULT_MAX_STEP_RAD = 0.0  # automatic: max_speed * DT_CLAMP_S
DT_CLAMP_S = 0.0015  # bounds a delayed callback's implicit 1 ms command velocity
DEFAULT_DEADBAND_DEG = 0.05
CB_STALL_S = 0.5
DEFAULT_MOVEJ_SPEED = 0.1  # zero-distance RT initialization
DEFAULT_TRACK_TOL_RAD = 0.15
DEFAULT_TRACK_TIMEOUT = 1.0
# Conservative local bounds, intersected with actual controller limits.
DEFAULT_SOFT_LO_DEG = (-180.0, -130.0, -165.0, -175.0, -175.0, -175.0)
DEFAULT_SOFT_HI_DEG = (+180.0, +130.0, +135.0, +175.0, +175.0, +175.0)


def approach_step(dist, v_prev, dt, v_max, a_max, max_step, deadband):
    """单关节：朝目标走一步。返回 (step, v_new)。**平滑的全部数学就在这里。**

    用**真实 dt** 而不是"每次固定走多少"，所以 RT 回调频率抖动不会变成速度抖动。
    沿用 CR7 参考项目的比例逼近、加速度限幅和步长保护。
    本项目的独立验证状态见 DEVELOPMENT.md。

    dist     目标 − 当前输出（rad）
    v_prev   上一回调的速度（rad/s）
    dt       距上一回调的秒数
    v_max    速度上限（rad/s）      a_max 加速度上限（rad/s²）
    max_step 单回调绝对步长上限（rad）
    deadband 死区（rad），|dist| 小于它就不动
    """
    if dt <= 0.0:
        return 0.0, v_prev
    dt = min(dt, DT_CLAMP_S)  # 回调停摆后不要把这一步算得过大
    if abs(dist) <= deadband:
        return 0.0, 0.0  # 到位：速度也归零，不留残余推力

    # 1) 期望速度：比例逼近 + 限速
    v_des = max(-v_max, min(v_max, KP_APPROACH * dist))
    # 2) 加速度限幅：速度只能按 a_max 变化（这一步管"平滑"）
    dv = max(-a_max * dt, min(a_max * dt, v_des - v_prev))
    v = v_prev + dv
    step = v * dt
    # 3) 防过冲：朝目标走且这一步会越过目标 → 直接停在目标上，速度归零
    if (step > 0.0) == (dist > 0.0) and abs(step) > abs(dist):
        step = dist
        v = 0.0
    # 4) 兜底：单回调绝对步长硬闸
    if max_step > 0.0:
        step = max(-max_step, min(max_step, step))
    return step, v


KP_APPROACH = 6.0  # 比例增益 1/s（接近目标的时间常数 ≈1/KP）


def _soft_pair(v, default_deg, dofs, which):
    """None→默认表；标量→广播；序列→逐轴。统一成弧度数组（长度 dofs）。"""
    if v is None:
        base = np.radians(np.asarray(default_deg, dtype=float))
        if base.size < dofs:
            base = np.full(dofs, base[0] if base.size else math.radians(175.0))
        return base[:dofs].copy()
    a = np.asarray(v, dtype=float).ravel()
    if a.size == 1:
        return np.full(dofs, float(a[0]))
    if a.size != dofs:
        raise ValueError(f"软限位{which}维度须为 1 或 {dofs}，收到 {a.size}")
    return a.copy()


def resolve_soft_limits(
    robot, sdk, ec, dofs, soft_lo_rad, soft_hi_rad, check_ctrl_limit=True, verbose=True
):
    """Intersect local and controller limits; reject missing or invalid feedback."""
    lo = _soft_pair(soft_lo_rad, DEFAULT_SOFT_LO_DEG, dofs, "下限")
    hi = _soft_pair(soft_hi_rad, DEFAULT_SOFT_HI_DEG, dofs, "上限")
    if not np.all(np.isfinite(lo)) or not np.all(np.isfinite(hi)):
        raise ValueError("Local soft limits must be finite")
    if check_ctrl_limit:
        buf = sdk.PyTypeVectorArrayDouble2()
        enabled = robot.getSoftLimit(buf, ec)
        if not enabled:
            raise RuntimeError("Controller soft limits are disabled")
        ctrl = np.asarray(buf.content(), dtype=float)
        if ctrl.shape != (dofs, 2) or not np.all(np.isfinite(ctrl)):
            raise RuntimeError("Invalid controller soft-limit feedback")
        if np.any(ctrl[:, 0] > ctrl[:, 1]):
            raise RuntimeError("Controller soft-limit bounds are reversed")
        lo = np.maximum(lo, ctrl[:, 0])
        hi = np.minimum(hi, ctrl[:, 1])
    if np.any(lo > hi):
        raise ValueError("Local and controller soft limits do not overlap")
    if verbose:
        print(
            "CR7 effective soft limits (deg):",
            np.degrees(np.column_stack((lo, hi))).round(2).tolist(),
        )
    return lo, hi


DEFAULT_FILTER_HZ = 10.0  # RT 回调路径的限幅滤波截止频率（setloop.py 用 10）
DEFAULT_STALE_TIMEOUT = 1.0  # s，超过即认为 leader/上位机断流
DEFAULT_ARM_GATE_RAD = 0.3  # rad，启动对齐闸门


class GelloCr7Robot:
    """GELLO Robot 协议实现（真机 CR7，只做 6 轴）。"""

    def __init__(
        self,
        robot,
        sdk,
        *,
        arm_dofs: int = DEFAULT_ARM_DOFS,
        enable_motion: bool = False,
        control_hz: float = DEFAULT_CONTROL_HZ,
        max_speed: Optional[float] = None,  # rad/s；None → DEFAULT_MAX_SPEED_DEG
        accel: Optional[float] = None,  # rad/s²；None → DEFAULT_ACCEL_DEG_S2
        max_step: float = DEFAULT_MAX_STEP_RAD,  # 0 = 自动
        deadband: float = math.radians(DEFAULT_DEADBAND_DEG),
        track_tol: float = DEFAULT_TRACK_TOL_RAD,
        track_timeout: float = DEFAULT_TRACK_TIMEOUT,
        soft_lo_rad: Optional[Sequence[float]] = None,
        soft_hi_rad: Optional[Sequence[float]] = None,
        check_ctrl_limit: bool = True,
        stale_timeout: float = DEFAULT_STALE_TIMEOUT,
        arm_gate_rad: float = DEFAULT_ARM_GATE_RAD,
        verbose: bool = True,
    ) -> None:
        self._robot = robot
        self._sdk = sdk
        self._ec: Dict = {}
        self._dofs = int(arm_dofs)

        self._enable_motion_requested = bool(enable_motion)
        self._control_hz = float(control_hz)
        self._max_speed = float(
            max_speed if max_speed is not None else math.radians(DEFAULT_MAX_SPEED_DEG)
        )
        self._max_accel = float(
            accel if accel is not None else math.radians(DEFAULT_ACCEL_DEG_S2)
        )
        self._max_step = float(max_step)
        self._deadband = float(deadband)
        self._track_tol = float(track_tol)
        self._track_timeout = float(track_timeout)
        self._soft_lo, self._soft_hi = resolve_soft_limits(
            robot,
            sdk,
            self._ec,
            self._dofs,
            soft_lo_rad,
            soft_hi_rad,
            check_ctrl_limit=check_ctrl_limit,
            verbose=verbose,
        )
        self._stale_timeout = float(stale_timeout)
        self._arm_gate = float(arm_gate_rad)
        self._verbose = bool(verbose)

        self._lock = threading.Lock()
        self._target: Optional[np.ndarray] = None  # leader 下发的目标（已限位）
        self._target_time = 0.0  # 最近一次收到命令的时刻
        self._state: Optional[np.ndarray] = None  # 实测关节角
        self._vel: Optional[np.ndarray] = None  # 差分估计的关节速度
        self._last_state: Optional[np.ndarray] = None
        self._n_commands = 0
        self._n_read_fail = 0
        self._over_limit_hits = 0

        self._running = True
        self._motion_active = False  # 已进入真实跟随
        self._abort_reason: Optional[str] = None
        # SDK callbacks must remain referenced for the lifetime of the RT loop.
        self._cb = None  # 回调对象**必须持引用**，否则会被 GC
        self._cb_safe = None  # 起跟时冻结的合法角度，回调异常时兜底
        self._cb_state = {
            "calls": 0,
            "stop": False,
            "ended": False,
            "errors": 0,
            "jump": 0.0,
            "jump_all": 0.0,
            "t_first": None,
            "t_last": None,
            "last_error": "",
        }
        self._loop_started = False
        self._rt_con = None
        self._powered_by_us = False
        self._mode_changed = False
        self._shutdown_done = False
        # 新路径：回调的输出（**回调线程原地写**）与每轴速度（回调线程写）
        self._out: Optional[list] = None
        self._accl_v: Optional[list] = None

        # 首帧：读一次当前关节角，作为 dry-run 显示与对齐闸门的基准
        self._state = self._read_joints(required=True)
        self._state_time_ns = time.monotonic_ns()
        self._last_state = self._state.copy()
        self._vel = np.zeros(self._dofs)

        if self._verbose:
            cap = self._max_step if self._max_step > 0 else self._max_speed * DT_CLAMP_S
            print(
                f"==> Cr7Robot 就绪：dofs={self._dofs} "
                f"模式={'MOTION(上电跟随)' if enable_motion else 'DRY-RUN(只读缓存，不上电)'}"
            )
            print(
                f"    设定值来源=SDK RT 回调插值"
                f"  速度≤{math.degrees(self._max_speed):.1f}°/s"
                f"  加速度≤{math.degrees(self._max_accel):.0f}°/s²"
                f"  单步≤{cap:.6f} rad（折算≤{math.degrees(cap / 0.001):.0f}°/s）"
            )
            print(f"    当前真机关节角(rad): {np.round(self._state, 4).tolist()}")

        self._thread = threading.Thread(target=self._servo_loop, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ 基本属性
    def num_dofs(self) -> int:
        return self._dofs

    def abort_reason(self) -> Optional[str]:
        """非 None 表示发生了安全中止，主程序应退出。"""
        return self._abort_reason

    def motion_active(self) -> bool:
        return self._motion_active

    def stats(self) -> Dict[str, float]:
        """伺服/回调仪表。**`cb_jump_all` 是判断 32006 风险的核心指标**：
        它是"相邻两条下发指令的最大差"，按控制器 1ms 周期折算即隐含速度，
        必须始终远小于各轴 120~200°/s 的上限。"""
        st = self._cb_state
        t_first, t_last = st.get("t_first"), st.get("t_last")
        rate = 0.0
        if t_first is not None and t_last is not None and t_last > t_first:
            rate = st["calls"] / (t_last - t_first)
        with self._lock:
            return {
                "n_commands": self._n_commands,
                "n_read_fail": self._n_read_fail,
                "over_limit_hits": self._over_limit_hits,
                "since_last_command": time.time() - self._target_time
                if self._target_time
                else -1.0,
                "cb_calls": st["calls"],
                "cb_rate": rate,
                "cb_jump_all": st["jump_all"],
                "cb_errors": st["errors"],
            }

    # ------------------------------------------------------------------ GELLO 接口
    def get_joint_state(self) -> np.ndarray:
        """实测关节角（只读缓存，不碰 SDK）。"""
        with self._lock:
            return self._state.copy()

    def command_joint_state(self, joint_state: Sequence[float]) -> None:
        """**只写缓存**：立即返回，真正的下发由后台线程做。"""
        q = np.asarray(joint_state, dtype=float).ravel()
        if q.size != self._dofs:
            raise ValueError(
                f"command_joint_state 维度错误：收到 {q.size}，期望 {self._dofs}"
            )
        if not np.all(np.isfinite(q)):
            raise ValueError("command_joint_state requires finite joint values")

        # 软限位（逐轴、非对称）：超限则饱和并计数（不直接中止——现场测试要看得到越界多少）
        clamped = np.clip(q, self._soft_lo, self._soft_hi)
        if not np.allclose(clamped, q, atol=1e-9):
            self._over_limit_hits += 1
            if self._verbose and self._over_limit_hits % 25 == 1:
                bad = [
                    f"J{i + 1}({math.degrees(q[i]):+.1f}° 限 "
                    f"{math.degrees(self._soft_lo[i]):+.0f}~{math.degrees(self._soft_hi[i]):+.0f}°)"
                    for i in range(self._dofs)
                    if q[i] < self._soft_lo[i] - 1e-9 or q[i] > self._soft_hi[i] + 1e-9
                ]
                print(
                    f"[限位] {'; '.join(bad)}，已饱和（原始 {np.round(q, 4).tolist()}）"
                )

        with self._lock:
            self._target = clamped
            self._target_time = time.time()
            self._n_commands += 1

    def get_observations(self) -> Dict[str, np.ndarray]:
        """RobotEnv 要求这 4 个 key 必须存在（见 gello/env.py:73-79）。"""
        with self._lock:
            return {
                "joint_positions": self._state.copy(),
                "joint_state_time_ns": self._state_time_ns,
                # 差分估计：step ① 不消费，录制阶段应改为 robot.jointVel(ec)
                "joint_velocities": self._vel.copy(),
                # 占位：step ① 不消费；需要时用 robot.posture(CoordinateType.flangeInBase, ec) 填
                "ee_pos_quat": np.zeros(7),
                # step ① 无夹爪；阶段② 接上后填真实开合度
                "gripper_position": np.zeros(1),
            }

    # ------------------------------------------------------------------ 生命周期
    def close(self) -> None:
        self._running = False
        try:
            self._thread.join(timeout=3.0)
        except Exception:
            pass
        self._shutdown_motion(verbose=self._verbose)

    # ------------------------------------------------------------------ 内部：读关节
    def _read_joints(self, required: bool = False) -> np.ndarray:
        try:
            q = np.asarray(self._robot.jointPos(self._ec)[: self._dofs], dtype=float)
            if q.size != self._dofs or not np.all(np.isfinite(q)):
                raise RuntimeError(f"jointPos 返回 {q.size} 维，期望 {self._dofs}")
            self._n_read_fail = 0
            return q
        except Exception as e:
            self._n_read_fail += 1
            if required or self._n_read_fail > 50:
                raise RuntimeError(
                    f"读取 jointPos 失败（连续 {self._n_read_fail} 次）：{e}"
                ) from e
            return None  # 调用方负责沿用上一帧

    # ------------------------------------------------------------------ 内部：伺服线程
    def _servo_loop(self) -> None:
        try:
            self._run_servo()
        except Exception as exc:
            self._set_abort(f"Servo thread failed: {type(exc).__name__}: {exc}")
        finally:
            self._shutdown_motion(verbose=self._verbose)

    def _run_servo(self) -> None:
        interval = 1.0 / self._control_hz
        lag_since: Optional[float] = None
        cb_stall_since: Optional[float] = None
        last_cb_calls = -1
        n_cycle = 0

        while self._running:
            t0 = time.perf_counter()

            # (1) 读实测状态 —— 本线程是**唯一**调用 SDK 的地方
            try:
                q = self._read_joints()
            except Exception as e:
                self._set_abort(f"读取真机状态失败：{e}")
                break
            if q is None:
                time.sleep(interval)
                continue

            with self._lock:
                prev = self._last_state
                self._state = q
                self._state_time_ns = time.monotonic_ns()
                if prev is not None:
                    self._vel = (q - prev) / max(interval, 1e-6)
                self._last_state = q.copy()
                target = None if self._target is None else self._target.copy()
                t_cmd = self._target_time
                have_cmd = self._n_commands > 0

            # (2) 未请求运动：dry-run，只刷缓存
            if not self._enable_motion_requested:
                n_cycle += 1
                if self._verbose and n_cycle % max(1, int(self._control_hz * 5)) == 0:
                    extra = (
                        ""
                        if target is None
                        else f" 目标(缓存) {np.round(target, 4).tolist()}"
                    )
                    print(
                        f"[dry-run {time.strftime('%H:%M:%S')}] 实测 {np.round(q, 4).tolist()}{extra}"
                    )
                self._sleep_to(t0, interval)
                continue

            # (3) 请求了运动但还没armed：先做对齐闸门，再做上电序列
            if not self._motion_active:
                if not have_cmd:
                    self._sleep_to(t0, interval)
                    continue
                if not self._arm_motion(q, target):
                    break  # 闸门拒绝 或 上电失败 → 已设置 abort
                continue  # 下一周期开始真正跟随

            # (4) 真实跟随
            if self._cb_state["errors"]:
                self._set_abort(f"RT callback failed: {self._cb_state['last_error']}")
                break

            if self._rt_con is not None and self._rt_con.hasMotionError():
                self._set_abort("实时模式报告运动错误（hasMotionError=True）")
                break

            if not have_cmd or (time.time() - t_cmd) > self._stale_timeout:
                self._set_abort(
                    f"leader 断流：{time.time() - t_cmd:.2f}s 未收到新命令"
                    f"（阈值 {self._stale_timeout}s）"
                )
                break

            # 真机若已在限位之外，clip 会把它往限位内拉 = 不受控运动 -> 直接中止。
            # （_arm_motion 的前置检查挡的是起跟那一刻；这里是运行中途的兜底）
            outside_now = (q < self._soft_lo - 1e-9) | (q > self._soft_hi + 1e-9)
            if np.any(outside_now):
                bad = [
                    f"J{i + 1}({math.degrees(q[i]):+.1f}°)"
                    for i in np.where(outside_now)[0]
                ]
                self._set_abort(
                    f"真机实测姿态已在软限位之外 —— {', '.join(bad)}，停止跟随"
                    f"（避免 clip 把关节往限位内拉）"
                )
                break

            # Callback owns interpolation; this thread monitors output and feedback.
            if self._out is None:
                self._set_abort("内部状态错误：_out 未初始化（arming 未走完？）")
                break
            # --- 看门狗：回调停摆（ZMQ 线程抢 GIL 把 RT 线程饿死时唯一的哨兵）
            if self._cb_state["calls"] != last_cb_calls:
                last_cb_calls = self._cb_state["calls"]
                cb_stall_since = None
            else:
                if cb_stall_since is None:
                    cb_stall_since = time.time()
                elif time.time() - cb_stall_since > CB_STALL_S:
                    self._set_abort(
                        f"RT 回调 {CB_STALL_S}s 未推进（RT 循环停摆）—— "
                        f"多半是 ZMQ/GIL 争用把实时线程饿死了。已下发计数 "
                        f"{self._cb_state['calls']}"
                    )
                    break
            # --- 看门狗：跟随滞后 = |回调输出 − 实测|
            err = float(np.max(np.abs(np.asarray(self._out, dtype=float) - q)))
            lag_tol, lag_to = self._track_tol, self._track_timeout
            lag_what = "|下发−实测|"

            if err > lag_tol:
                if lag_since is None:
                    lag_since = time.time()
                elif time.time() - lag_since > lag_to:
                    self._set_abort(
                        f"滞后超限：{lag_what}={err:.3f} rad "
                        f"({math.degrees(err):.1f}°) 持续 >{lag_to}s"
                        f"（真机可能被挡住/跟不上）"
                    )
                    break
            else:
                lag_since = None

            if self._verbose and n_cycle % max(1, int(self._control_hz * 2)) == 0:
                print(
                    f"[follow {time.strftime('%H:%M:%S')}] 实测 {np.round(q, 4).tolist()} "
                    f"目标 {np.round(target, 4).tolist()} 下发 {np.round(self._out if self._out is not None else q, 4).tolist()} "
                    f"滞后 {err:.3f}"
                )
            n_cycle += 1
            self._sleep_to(t0, interval)

    @staticmethod
    def _sleep_to(t0: float, interval: float) -> None:
        slack = interval - (time.perf_counter() - t0)
        if slack > 0:
            time.sleep(slack)

    # ------------------------------------------------------------------ 内部：上电跟随序列
    def _arm_motion(self, q: np.ndarray, target: np.ndarray) -> bool:
        """启动对齐闸门 + 上电序列。成功返回 True。"""
        # 软限位前置检查：若真机**当前**就站在限位之外，起跟后第一周期
        # `np.clip(q + delta, lo, hi)` 会立刻把该关节往限位内拉 —— 那是一段
        # 不受控的小幅运动（操作者手没动，臂却动了）。与其悄悄拉一把，直接拒绝并指名关节。
        outside = np.where((q < self._soft_lo - 1e-9) | (q > self._soft_hi + 1e-9))[0]
        if outside.size:
            bad = [
                f"J{i + 1}({math.degrees(q[i]):+.1f}° 限 "
                f"{math.degrees(self._soft_lo[i]):+.0f}~{math.degrees(self._soft_hi[i]):+.0f}°)"
                for i in outside
            ]
            self._set_abort(
                f"拒绝启动：真机当前姿态已在逐轴软限位之外 —— {', '.join(bad)}。"
                f"起跟会把这些关节拉回限位内（不受控运动）。"
                f"请先在示教器上将真机调整至允许范围，再重新核对两臂姿态。"
            )
            return False

        diff = float(np.max(np.abs(target - q)))
        if diff > self._arm_gate:
            bad = np.where(np.abs(target - q) > self._arm_gate)[0].tolist()
            self._set_abort(
                f"拒绝启动：leader 与真机姿态差 {diff:.3f} rad > 闸门 {self._arm_gate} rad，"
                f"越界关节 {bad}。请先把两臂摆到接近的姿态再启动。"
            )
            return False

        sdk, robot, ec = self._sdk, self._robot, self._ec
        try:
            print(
                f"==> 对齐通过（最大差 {diff:.3f} rad ≤ {self._arm_gate} rad），开始上电跟随序列："
            )
            # ⚠️ 每步都是「**先打印、后调用**」：万一底层 SDK 段错误（Python 捕不到），
            #    最后一行输出就直接点名崩在哪次调用上，不必再二分。
            print("    ① setRtNetworkTolerance(20)")
            robot.setRtNetworkTolerance(20, ec)

            print("    ② setMotionControlMode(RtCommandMode)")
            robot.setMotionControlMode(sdk.MotionControlMode.RtCommandMode, ec)
            self._mode_changed = True

            print("    ③ setOperateMode(automatic)")
            robot.setOperateMode(sdk.OperateMode.automatic, ec)

            print("    ④ setPowerState(True)")
            robot.setPowerState(True, ec)
            self._powered_by_us = True

            # Feedback uses jointPos on the servo thread; no 1 ms state stream.
            print("    ⑤ getRtMotionController()")
            self._rt_con = robot.getRtMotionController()

            print(f"    ⑥ rtCon.setFilterLimit(True, {DEFAULT_FILTER_HZ})")
            self._rt_con.setFilterLimit(True, DEFAULT_FILTER_HZ)

            # Initialize RT with current feedback as both endpoints.
            print(f"    ⑦ rtCon.MoveJ({DEFAULT_MOVEJ_SPEED}, 当前角, 当前角)")
            self._rt_con.MoveJ(DEFAULT_MOVEJ_SPEED, q, q)

            print("    ⑧ 建回调（**回调自己插值**，从当前角连续爬升）")
            # The first callback starts exactly at current feedback, with zero velocity.
            self._out = [float(x) for x in q]
            self._accl_v = [0.0] * self._dofs
            self._cb_safe = list(self._out)  # 兜底的最后手段（正常走不到）
            self._make_callback(sdk)

            print("    ⑨ rtCon.setControlLoopJoi(callback)")
            self._rt_con.setControlLoopJoi(self._cb)

            # startMove() 文档：「指定控制模式，机器人准备开始运动，在每段回调执行前
            # 需要先调用此接口」。没有它，回调不会被调度。
            print("    ⑩ rtCon.startMove(RtControllerMode.jointPosition)")
            self._rt_con.startMove(sdk.RtControllerMode.jointPosition)

            print("    ⑪ rtCon.startLoop(False)   ← 非阻塞，循环跑在 SDK 自己的线程里")
            self._rt_con.startLoop(False)
            self._loop_started = True

            cap = self._max_step if self._max_step > 0 else self._max_speed * DT_CLAMP_S
            print(
                f"==> 已进入真实跟随：速度 ≤{math.degrees(self._max_speed):.1f}°/s，"
                f"加速度 ≤{math.degrees(self._max_accel):.0f}°/s²"
            )
            print(
                f"    设定值由 RT 回调插值（非伺服线程 {self._control_hz:.0f}Hz 阶梯）；"
                f"单步 ≤{cap:.6f} rad → 控制器按 1ms 看到的隐含速度 "
                f"≤{math.degrees(cap / 0.001):.0f}°/s（轴上限 "
                f"{JOINT_SPEED_LIMIT_DEG[0]:.0f}~{JOINT_SPEED_LIMIT_DEG[-1]:.0f}°/s）"
            )
            self._motion_active = True
            return True
        except Exception as e:
            self._set_abort(f"上电/进入实时模式失败：{type(e).__name__}: {e}")
            return False

    def _make_callback(self, sdk) -> None:
        """建 RT 回调：SDK 的实时线程每 ~1ms 调一次。

        回调从当前反馈连续逼近内存目标，限速、限加速度并防止过冲。

        ⚠️ 回调跑在**实时线程**上：不能做通信/耗时操作，**更不能抛异常**。
        """
        st = self._cb_state
        dofs = self._dofs
        lo, hi = self._soft_lo, self._soft_hi
        out = self._out  # list；起跟时已设成真机当前角
        v = self._accl_v  # 每轴速度（rad/s），回调线程独占
        t_prev = [None]  # 上一回调时刻（放 list 里以便闭包内赋值）

        def callback():
            try:
                now = time.perf_counter()
                dt = 0.0 if t_prev[0] is None else (now - t_prev[0])
                t_prev[0] = now
                if st["t_first"] is None:
                    st["t_first"] = now
                st["t_last"] = now
                st["calls"] += 1

                goal = self._target if self._target is not None else out
                v_max = self._max_speed
                a_max = self._max_accel
                cap = self._max_step if self._max_step > 0.0 else v_max * DT_CLAMP_S
                dead = self._deadband
                jump = 0.0
                for i in range(dofs):
                    g = goal[i]
                    # 纵深防御：软限位在回调里**再夹一次**（目标可能绕过 command_joint_state）
                    if g < lo[i]:
                        g = float(lo[i])
                    elif g > hi[i]:
                        g = float(hi[i])
                    step, v[i] = approach_step(
                        g - out[i], v[i], dt, v_max, a_max, cap, dead
                    )
                    out[i] += step
                    d = abs(step)
                    if d > jump:
                        jump = d
                # ★ 仪表：相邻两条下发指令的最大差。按控制器 1ms 周期折算即隐含速度，
                #   必须始终远小于各轴轴速度上限（旧实现是 0.01 rad → 573°/s）。
                st["jump"] = jump
                if jump > st["jump_all"]:
                    st["jump_all"] = jump

                cmd = sdk.JointPosition()
                cmd.joints = out
                cmd.external = []
                if st["stop"]:
                    # 让 **SDK 自己**结束循环。直接调 rtCon.stopLoop() 会和本线程抢 GIL
                    # 死锁：主线程持 GIL 等 RT 线程停，RT 线程要跑本回调、等 GIL。
                    # 2026-09-30 实测卡死过一次，只能 kill -9。
                    cmd.setFinished()
                    st["ended"] = True
                return cmd
            except Exception as e:
                # ⚠️ 实时线程里**绝不能抛出去**。兜底返回**上一次发出的值** `out` ——
                #    不能返回起跟姿态：它可能离当前值很远，跳回去本身就是一次大跳变。
                st["errors"] += 1
                st["last_error"] = f"{type(e).__name__}: {e}"
                cmd = sdk.JointPosition()
                cmd.joints = out if out is not None else self._cb_safe
                cmd.external = []
                cmd.setFinished()
                st["ended"] = True
                return cmd

        # 必须持引用，否则回调对象被 GC、RT 线程调用悬空指针
        self._cb = callback

    def _finish_loop(self, verbose: bool = True) -> None:
        """通知 RT 回调返回 setFinished()，让循环**自己**停（绕开 stopLoop 的 GIL 死锁）。"""
        st = self._cb_state
        if not self._loop_started or st["ended"]:
            return
        if verbose:
            print("    · 通知回调 setFinished()，让 RT 循环自结束 ...")
        st["stop"] = True
        t0 = time.time()
        while not st["ended"] and time.time() - t0 < 3.0:
            time.sleep(0.05)  # sleep 释放 GIL，RT 线程才有机会跑回调
        if verbose:
            print(
                f"      → {'已自行结束' if st['ended'] else '3s 内未结束（超时）'}"
                f"，累计回调 {st['calls']} 次"
            )

    def _shutdown_motion(self, verbose: bool = True) -> None:
        """退出实时模式的安全序列（幂等；伺服线程退出与 close() 可能同时走到这里）。"""
        with self._lock:
            if self._shutdown_done:
                return
            self._shutdown_done = True
        self._finish_loop(verbose)
        # ★ 防死锁守卫：循环没停下来之前，**任何** SDK 调用都可能和 RT 线程抢 GIL 死锁。
        #   宁可提前返回、把话说清楚，也不要让收尾卡死（卡死过一次，只能 kill -9）。
        if self._loop_started and not self._cb_state["ended"]:
            print("    !! RT 循环未在 3s 内自行结束 —— 跳过后续 SDK 调用以免死锁。")
            print("       请在示教器确认停止，并按控制器流程恢复模式。")
            return
        # startMove() 的官方文档：「正确停止方法是调用 stopMove」。
        # 不调 stopMove 就切模式/下电，「可能会失败」（原文）。
        if self._rt_con is not None:
            try:
                self._rt_con.stopMove()
                if verbose:
                    print("    · rtCon.stopMove()")
            except Exception as e:
                print(f"    ! rtCon.stopMove() 异常：{e}")
            self._rt_con = None
        if self._mode_changed:
            # 防御性调用：本类已不开状态流，但若**上一次**进程段错误留下悬挂的
            # 1ms 流，这里是唯一能把它关掉的地方（SDK 无查询接口，只能盲关）。
            try:
                self._robot.stopReceiveRobotState()
                if verbose:
                    print("    · stopReceiveRobotState()")
            except Exception:
                pass
            # Return to non-real-time control before manual mode.
            try:
                self._robot.setMotionControlMode(
                    self._sdk.MotionControlMode.NrtCommandMode, self._ec
                )
                if verbose:
                    print("    · setMotionControlMode(NrtCommandMode)")
            except Exception as e:
                print(f"    ! setMotionControlMode(NrtCommandMode) 异常：{e}")
            try:
                self._robot.setOperateMode(self._sdk.OperateMode.manual, self._ec)
                if verbose:
                    print("    · setOperateMode(manual)")
            except Exception as e:
                print(f"    ! setOperateMode(manual) 异常：{e}")
            if self._powered_by_us:
                print(
                    "[提示] 机器人仍处于上电状态（抱闸保持）。确认安全后请在示教器/HMI 上下电。"
                )
        self._motion_active = False

    def _set_abort(self, reason: str) -> None:
        if self._abort_reason is None:
            self._abort_reason = reason
            print("\n" + "!" * 62)
            print(f"!! 安全中止：{reason}")
            print("!" * 62)
