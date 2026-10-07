"""Asynchronous CR7/Robotiq episodes using the shared raw-session layout."""

from __future__ import annotations

import json
import math
import os
import queue
import threading
from datetime import datetime
from pathlib import Path
from typing import Any


class RecordingError(RuntimeError):
    """Raised when raw recording cannot continue without losing data."""


class RawEpisodeRecorder:
    """Write raw JSONL episodes without blocking the robot control loop on disk I/O."""

    def __init__(
        self,
        root: str | Path,
        *,
        control_hz: float,
        joint_signs: list[int],
        task: str,
        metadata: dict[str, Any] | None = None,
        queue_size: int = 500,
        session_time: datetime | None = None,
    ) -> None:
        if not math.isfinite(control_hz) or control_hz <= 0:
            raise ValueError("control_hz must be positive")
        if queue_size <= 0:
            raise ValueError("queue_size must be positive")
        if len(joint_signs) != 6 or any(sign not in (-1, 1) for sign in joint_signs):
            raise ValueError("joint_signs must contain six values, each +1 or -1")
        if not task.strip():
            raise ValueError("task must not be empty")

        now = session_time or datetime.now().astimezone()
        session_name = now.strftime("session_%Y%m%d_%H%M%S")
        self.session_dir = self._unique_session_dir(Path(root), session_name)
        self.episodes_dir = self.session_dir / "episodes"
        self.episodes_dir.mkdir(parents=True, exist_ok=False)

        self._control_hz = float(control_hz)
        self._joint_signs = list(joint_signs)
        self._task = task.strip()
        self._queue_size = queue_size
        self._episode_index = 0
        self._sequence = 0
        self._sample_queue: queue.Queue[dict[str, Any] | None] | None = None
        self._writer_thread: threading.Thread | None = None
        self._writer_error: Exception | None = None
        self._partial_path: Path | None = None
        self._recording = False

        manifest = {
            "format": "xcore_cr7_gello_raw",
            "format_version": 1,
            "created_at": now.isoformat(),
            "clock": "time.monotonic_ns",
            "robot_type": "xmate_cr7",
            "control_hz": self._control_hz,
            "task": self._task,
            "joint_units": "rad",
            "gripper_range": [0.0, 1.0],
            "joint_names": [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
                "gripper",
            ],
            "joint_signs": self._joint_signs,
            "sample_fields": [
                "sequence",
                "command_time_ns",
                "observation_time_ns",
                "wall_time_ns",
                "control_period_ns",
                "action",
                "joint_positions",
                "gripper_position",
                "arm_feedback_time_ns",
                "gripper_feedback_time_ns",
                "gripper_position_raw",
                "arm_feedback_age_ns",
                "gripper_feedback_age_ns",
            ],
            "action_semantics": "requested six-axis goal and gripper closure; "
            "SDK interpolation/limits and asynchronous gripper dispatch follow",
            "observation_semantics": (
                "measured CR7 joints and measured Robotiq position"
            ),
            "hardware": metadata or {},
        }
        self._write_json_atomic(self.session_dir / "manifest.json", manifest)

    @staticmethod
    def _unique_session_dir(root: Path, session_name: str) -> Path:
        root.mkdir(parents=True, exist_ok=True)
        candidate = root / session_name
        suffix = 1
        while candidate.exists():
            candidate = root / f"{session_name}_{suffix:02d}"
            suffix += 1
        return candidate

    @staticmethod
    def _write_json_atomic(path: Path, value: dict[str, Any]) -> None:
        temporary = path.with_suffix(path.suffix + ".partial")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def episode_index(self) -> int:
        return self._episode_index

    def start_episode(self) -> Path:
        if self._recording:
            raise RecordingError("an episode is already recording")
        self._check_writer_error()

        self._sequence = 0
        self._partial_path = (
            self.episodes_dir / f"episode_{self._episode_index:06d}.jsonl.partial"
        )
        self._sample_queue = queue.Queue(maxsize=self._queue_size)
        self._writer_error = None
        self._recording = True
        self._writer_thread = threading.Thread(
            target=self._writer_main,
            args=(self._partial_path, self._sample_queue),
            name=f"raw-episode-writer-{self._episode_index:06d}",
            daemon=False,
        )
        self._writer_thread.start()
        return self._partial_path

    def add_sample(self, sample: dict[str, Any]) -> None:
        if not self._recording or self._sample_queue is None:
            return
        self._check_writer_error()
        frame = dict(sample)
        frame["sequence"] = self._sequence
        self._sequence += 1
        try:
            self._sample_queue.put_nowait(frame)
        except queue.Full as exc:
            self._sequence -= 1
            raise RecordingError(
                f"raw recorder queue is full after {self._sequence} samples; "
                "the current episode is incomplete"
            ) from exc

    def save_episode(self) -> Path:
        partial_path = self._finish_writer()
        if self._sequence == 0:
            partial_path.unlink(missing_ok=True)
            raise RecordingError("cannot save an empty episode")
        final_path = partial_path.with_suffix("")
        os.replace(partial_path, final_path)
        self._episode_index += 1
        return final_path

    def discard_episode(self) -> Path:
        partial_path = self._finish_writer()
        partial_path.unlink(missing_ok=True)
        return partial_path

    def close_interrupted(self) -> Path | None:
        """Flush an active episode but intentionally keep its .partial suffix."""

        if not self._recording:
            return None
        return self._finish_writer()

    def _finish_writer(self) -> Path:
        if (
            not self._recording
            or self._sample_queue is None
            or self._partial_path is None
        ):
            raise RecordingError("no episode is recording")
        sample_queue = self._sample_queue
        writer_thread = self._writer_thread
        partial_path = self._partial_path
        self._recording = False
        if writer_thread is not None:
            while writer_thread.is_alive():
                try:
                    sample_queue.put(None, timeout=0.1)
                    break
                except queue.Full:
                    continue
            writer_thread.join()
        self._sample_queue = None
        self._writer_thread = None
        self._partial_path = None
        self._check_writer_error()
        return partial_path

    def _writer_main(
        self,
        path: Path,
        sample_queue: queue.Queue[dict[str, Any] | None],
    ) -> None:
        try:
            with path.open("x", encoding="utf-8", buffering=64 * 1024) as stream:
                samples_since_flush = 0
                flush_interval = max(1, round(self._control_hz))
                while True:
                    sample = sample_queue.get()
                    try:
                        if sample is None:
                            break
                        stream.write(
                            json.dumps(
                                sample,
                                ensure_ascii=False,
                                separators=(",", ":"),
                                allow_nan=False,
                            )
                        )
                        stream.write("\n")
                        samples_since_flush += 1
                        if samples_since_flush >= flush_interval:
                            stream.flush()
                            samples_since_flush = 0
                    finally:
                        sample_queue.task_done()
                stream.flush()
                os.fsync(stream.fileno())
        # The worker must report any serialization or filesystem failure to the
        # control thread; limiting this to OSError would hide malformed samples.
        except Exception as exc:  # noqa: BLE001
            self._writer_error = exc

    def _check_writer_error(self) -> None:
        if self._writer_error is not None:
            raise RecordingError(
                f"raw episode writer failed: {self._writer_error}"
            ) from self._writer_error
