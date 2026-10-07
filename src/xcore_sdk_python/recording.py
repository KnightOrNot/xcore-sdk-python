"""Recording controls and assembly of measured CR7 and Robotiq feedback."""

from __future__ import annotations

import os
import select
import sys
import termios
import tty
from typing import Any

import numpy as np

from .raw_recorder import RawEpisodeRecorder, RecordingError


class RecordingKeyboard:
    def __init__(self) -> None:
        self._fd: int | None = None
        self._original: list[Any] | None = None

    def open(self) -> None:
        if sys.stdin.isatty():
            self._fd = sys.stdin.fileno()
            self._original = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        else:
            print(
                "Non-interactive input: use --start-recording; episode remains partial "
                "until explicitly saved with S.",
                file=sys.stderr,
            )

    def close(self) -> None:
        if self._fd is not None and self._original is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._original)

    def poll(self) -> str | None:
        if self._fd is None:
            return None
        readable, _, _ = select.select([self._fd], [], [], 0)
        if not readable:
            return None
        return os.read(self._fd, 1).decode(errors="ignore").lower()


def handle_key(key: str | None, recorder: RawEpisodeRecorder) -> None:
    if key == "r" and not recorder.is_recording:
        print(f"\n[记录] 开始：{recorder.start_episode()}")
    elif key == "s" and recorder.is_recording:
        print(f"\n[记录] 已保存：{recorder.save_episode()}")
    elif key == "d" and recorder.is_recording:
        recorder.discard_episode()
        print("\n[记录] 当前 episode 已丢弃")
    elif key == "p":
        print(
            f"\n[记录] recording={recorder.is_recording}; "
            f"episode={recorder.episode_index}; session={recorder.session_dir}"
        )
    elif key == "h":
        print("\nR=开始，S=保存，D=丢弃，P=状态，H=帮助；退出保留未保存 .partial")


def sample_from_cycle(
    *,
    action: np.ndarray,
    arm: dict[str, Any],
    gripper: dict[str, Any],
    command_time_ns: int,
    observation_time_ns: int,
    wall_time_ns: int,
    control_period_ns: int,
    open_pos: int,
    closed_pos: int,
    max_age_s: float,
) -> dict[str, Any]:
    joints = np.asarray(arm.get("joint_positions"), dtype=float)
    target = np.asarray(action, dtype=float)
    if (
        joints.shape != (6,)
        or target.shape != (7,)
        or not np.all(np.isfinite(joints))
        or not np.all(np.isfinite(target))
    ):
        raise RecordingError(
            "Recording requires finite six-axis feedback and seven-channel action"
        )
    if not 0 <= target[-1] <= 1:
        raise RecordingError("Invalid gripper action")
    timestamps = (arm.get("joint_state_time_ns"), gripper.get("feedback_time_ns"))
    for timestamp in timestamps:
        if (
            isinstance(timestamp, bool)
            or not isinstance(timestamp, int)
            or not 0 < timestamp <= observation_time_ns
            or observation_time_ns - timestamp > max_age_s * 1e9
        ):
            raise RecordingError(
                "Missing or stale follower feedback; episode remains partial"
            )
    raw = gripper.get("position_raw")
    if isinstance(raw, bool) or not isinstance(raw, int) or not 0 <= raw <= 255:
        raise RecordingError("Invalid measured gripper position")
    closure = float(np.clip((raw - open_pos) / (closed_pos - open_pos), 0, 1))
    return {
        "command_time_ns": command_time_ns,
        "observation_time_ns": observation_time_ns,
        "wall_time_ns": wall_time_ns,
        "control_period_ns": control_period_ns,
        "action": target.tolist(),
        "joint_positions": joints.tolist() + [closure],
        "gripper_position": closure,
        "gripper_position_raw": raw,
        "arm_feedback_time_ns": timestamps[0],
        "gripper_feedback_time_ns": timestamps[1],
        "arm_feedback_age_ns": observation_time_ns - timestamps[0],
        "gripper_feedback_age_ns": observation_time_ns - timestamps[1],
    }
