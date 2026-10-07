"""Read-only GELLO leader agent: calibration mapping and angle unwrapping."""

from __future__ import annotations

import math
import time
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from .dynamixel_reader import (
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    JOINT_IDS,
    JOINT_OFFSETS,
    JOINT_SIGNS,
    TeachArm,
    ticks_to_rad,
)

DEFAULT_ALPHA = 0.99  # 与 gello DynamixelRobot 一致的轻指数平滑
DEFAULT_JUMP_GUARD = (
    1.0  # rad，单周期最大合理变化；超过视为读数毛刺（100Hz 下 1rad/10ms = 5700°/s）
)
DEFAULT_FAIL_TIMEOUT = 2.0  # s，连续读失败超过此值 → 抛异常退出

N_JOINTS = len(JOINT_IDS)


def wrap_pi(x):
    """把角度折到 (-π, π]。"""
    return (np.asarray(x) + math.pi) % (2 * math.pi) - math.pi


class LeaderReadError(RuntimeError):
    """小臂读取持续失败。抛出后主程序应退出，真机侧断流看门狗会自动停止跟随。"""


class Cr7LeaderAgent:
    """GELLO Agent 协议实现（leader = 小示教臂，只读）。"""

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        *,
        joint_offsets: Sequence[float] = JOINT_OFFSETS,
        joint_signs: Sequence[int] = JOINT_SIGNS,
        baudrate: int = DEFAULT_BAUDRATE,
        gripper_id: Optional[int] = None,
        gripper_config: Optional[Tuple[float, float]] = None,
        alpha: float = DEFAULT_ALPHA,
        jump_guard_rad: float = DEFAULT_JUMP_GUARD,
        fail_timeout: float = DEFAULT_FAIL_TIMEOUT,
        unwrap: bool = True,
        verbose: bool = True,
    ) -> None:
        self._offsets = np.asarray(joint_offsets, dtype=float)
        self._signs = np.asarray(joint_signs, dtype=float)
        if self._offsets.size != N_JOINTS or self._signs.size != N_JOINTS:
            raise ValueError(
                f"offsets/signs 维度须为 {N_JOINTS}，"
                f"收到 {self._offsets.size}/{self._signs.size}"
            )
        if not np.all(np.abs(self._signs) == 1):
            raise ValueError(f"joint_signs 只能是 ±1：{self._signs.tolist()}")

        # 第 ② 步才用夹爪（扳机 ID7 → [0,1]）
        self._gripper_id = gripper_id
        self._gripper_open_close = gripper_config  # (开° , 关°) 或 None
        if gripper_id is not None and gripper_config is None:
            raise ValueError(
                "指定了 gripper_id 就必须同时给 gripper_config=(开deg, 关deg)"
            )
        if gripper_id in JOINT_IDS:
            raise ValueError("gripper_id must differ from the six arm IDs")
        if gripper_config is not None:
            o, c = gripper_config
            if not math.isfinite(o) or not math.isfinite(c) or abs(c - o) < 1e-6:
                raise ValueError("Gripper endpoints must be finite and different")

        self._alpha = float(alpha)
        self._jump_guard = float(jump_guard_rad)
        self._fail_timeout = float(fail_timeout)
        self._unwrap_on = bool(unwrap)
        self._verbose = bool(verbose)

        self._arm = TeachArm(port, baudrate, JOINT_IDS, gripper_id)
        self.port = port

        self._r_cont: Optional[np.ndarray] = None  # 连续解缠后的原始角
        self._last_raw: Optional[np.ndarray] = None  # 上一帧连续原始角（毛刺判定用）
        self._last_q: Optional[np.ndarray] = None
        self._last_g: Optional[float] = None
        self._branch = np.zeros(N_JOINTS)  # 2π 整数分支
        self._fail_since: Optional[float] = None
        self._n_read = 0
        self._n_fail = 0
        self._n_glitch = 0
        self._n_wrap = 0
        self._frame_ticks = None

        # 首帧：建立解缠锚点
        r0 = self._read_single_turn()
        if r0 is not None:
            self._r_cont = r0
            self._last_raw = r0.copy()

        if self._verbose:
            roles = "6 关节" + (
                f" + 夹爪 ID{gripper_id}" if gripper_id else "（无夹爪）"
            )
            print(
                f"==> Cr7LeaderAgent 就绪：{port} @ {baudrate}，{roles}（只读，无写指令）"
            )
            print("    映射 cr7_q[i] = sign[i]*(raw[i] - offset[i]) - 2π*branch[i]")
            print(f"    offsets = {np.round(self._offsets, 4).tolist()}")
            print(f"    signs   = {self._signs.astype(int).tolist()}")

    # ------------------------------------------------------------------ 维度
    def num_dofs(self) -> int:
        return N_JOINTS + (1 if self._gripper_id is not None else 0)

    @property
    def has_gripper(self) -> bool:
        return self._gripper_id is not None

    # ------------------------------------------------------------------ 2π 分支
    def set_branch(self, branch: Sequence[float]) -> None:
        """设置每关节的 2π 整数分支（由 cr7_teleop.py 启动对齐时解出）。"""
        b = np.asarray(branch, dtype=float).ravel()
        if b.size != N_JOINTS:
            raise ValueError(f"branch 维度须为 {N_JOINTS}")
        self._branch = np.round(b)
        if self._verbose:
            print(
                f"==> 已设置 2π 分支 branch = {self._branch.astype(int).tolist()}"
                f"（对应修正 {np.round(self._branch * 360.0, 1).tolist()} °）"
            )

    def branch(self) -> np.ndarray:
        return self._branch.copy()

    def q_without_branch(self) -> np.ndarray:
        """返回未做 2π 分支修正的映射值（对齐求解用）。"""
        r = self.read_raw_rad()
        if r is None:
            raise LeaderReadError("对齐求解时读小臂失败")
        return (r - self._offsets) * self._signs

    # ------------------------------------------------------------------ 只读原语
    def _read_single_turn(self) -> Optional[np.ndarray]:
        """硬件读：返回 6 关节原始角 rad（0~2π 单圈），失败返回 None。不改变解缠状态。"""
        ticks = self._arm.read_ticks()
        self._frame_ticks = ticks
        if ticks is None:
            self._n_fail += 1
            return None
        vals = [ticks.get(i) for i in JOINT_IDS]
        if self.has_gripper:
            vals.append(ticks.get(self._gripper_id))
        if any(v is None for v in vals):
            self._n_fail += 1
            return None
        return np.array([ticks_to_rad(v) for v in vals[:N_JOINTS]], dtype=float)

    def read_single_turn_rad(self) -> Optional[np.ndarray]:
        """读 6 关节**单圈**原始角 rad ∈ [0, 2π)，**不做任何解缠**。

        标定专用：标定的位形之间两臂被人工挪动，帧间跳变常 > π，
        连续解缠会串到错误分支；而标定模型本身只在 mod 2π 意义下成立，
        所以必须逐位形独立、按圆统计处理（见 cr7_calib.fit_joint）。
        """
        return self._read_single_turn()

    def read_raw_rad(self) -> Optional[np.ndarray]:
        """读 6 关节**连续解缠后**的原始角（未减 offset、未乘 sign）。"""
        raw = self._read_single_turn()
        if raw is None:
            return None
        if self._r_cont is None or not self._unwrap_on:
            self._r_cont = raw
            return raw.copy()

        delta = wrap_pi(raw - self._r_cont)
        if np.any(np.abs(raw - self._r_cont) > math.pi + 1e-9):
            self._n_wrap += 1
        self._r_cont = self._r_cont + delta
        return self._r_cont.copy()

    def read_raw_gripper_rad(self) -> Optional[float]:
        """读夹爪扳机 ID7 的原始角度（rad，单圈）。"""
        if self._gripper_id is None:
            return None
        ticks = self._arm.read_ticks()
        if ticks is None:
            return None
        t = ticks.get(self._gripper_id)
        return None if t is None else ticks_to_rad(t)

    # ------------------------------------------------------------------ 关节状态
    def get_joint_state(self) -> np.ndarray:
        """返回 6 维（第②步 7 维）关节角。异常时返回上一帧，持续失败则抛异常。"""
        now = time.time()
        raw = self.read_raw_rad()

        if raw is None:
            if self._fail_since is None:
                self._fail_since = now
                if self._verbose:
                    print(
                        f"[leader] 读小臂失败，暂保持上一帧（累计 {self._n_fail} 次）"
                    )
            elif now - self._fail_since > self._fail_timeout:
                raise LeaderReadError(
                    f"小臂连续 {now - self._fail_since:.1f}s 读不到数据"
                    f"（累计失败 {self._n_fail} 次），请检查串口/供电"
                )
            if self._last_q is None:
                raise LeaderReadError("小臂首次读取就失败，无法确定起始姿态")
            return self._with_gripper(self._last_q)

        self._fail_since = None
        self._n_read += 1

        # 毛刺保护：解缠之后，真实运动不可能单帧变化 > jump_guard
        if self._last_raw is not None:
            jump = float(np.max(np.abs(raw - self._last_raw)))
            if jump > self._jump_guard:
                self._n_glitch += 1
                if self._verbose and self._n_glitch % 25 == 1:
                    print(
                        f"[leader] 检测到读数跳变 {jump:.3f} rad > {self._jump_guard}，"
                        f"已忽略该帧（累计 {self._n_glitch} 次）"
                    )
                self._r_cont = self._last_raw.copy()
                return self._with_gripper(
                    self._last_q
                    if self._last_q is not None
                    else (raw - self._offsets) * self._signs
                )
        self._last_raw = raw.copy()

        q = self._raw_to_q(raw)

        # 轻指数平滑（与 gello DynamixelRobot 同款：alpha=0.99 时几乎等于原值）
        if self._last_q is not None:
            q = self._last_q * (1.0 - self._alpha) + q * self._alpha
        self._last_q = q

        if self.has_gripper:
            g = self._gripper_from_raw(self._frame_ticks)
            if g is not None:
                self._last_g = g
        return self._with_gripper(q)

    def _with_gripper(self, q: np.ndarray) -> np.ndarray:
        if self.has_gripper:
            if self._last_g is None:
                raise LeaderReadError("No valid gripper sample available")
            return np.concatenate([q, [self._last_g]])
        return q.copy()

    def _raw_to_q(self, raw: np.ndarray) -> np.ndarray:
        return (raw - self._offsets) * self._signs - 2 * math.pi * self._branch

    def _gripper_from_raw(self, ticks=None) -> Optional[float]:
        """扳机角度 → [0,1]（gello 同款归一化）。"""
        if self._gripper_id is None or self._gripper_open_close is None:
            return None
        if ticks is None:
            r = self.read_raw_gripper_rad()
        else:
            t = ticks.get(self._gripper_id)
            r = None if t is None else ticks_to_rad(t)
        if r is None:
            return None
        o, c = self._gripper_open_close
        o, c = math.radians(o), math.radians(c)
        if abs(c - o) < 1e-6:
            return None
        v = (r - o) / (c - o)
        return float(min(max(0.0, v), 1.0))

    # ------------------------------------------------------------------ GELLO Agent 协议
    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        """GELLO 的 Agent 只有一个方法：直接回读 leader 关节角。"""
        return self.get_joint_state()

    def get_observations(self) -> Dict[str, np.ndarray]:
        return {"joint_state": self.get_joint_state()}

    def stats(self) -> Dict[str, float]:
        return {
            "n_read": self._n_read,
            "n_fail": self._n_fail,
            "n_glitch": self._n_glitch,
            "n_wrap": self._n_wrap,
        }

    def close(self) -> None:
        try:
            self._arm.close()
        except Exception:
            pass
