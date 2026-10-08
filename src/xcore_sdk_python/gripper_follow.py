"""Independent Robotiq TCP streaming worker with a latest-target mailbox.

The arm client never waits for gripper I/O. One request is in flight at a time;
new samples replace the pending target rather than accumulating in a queue.
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time
from typing import Any


class GripperStreamFault(RuntimeError):
    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(f"Gripper stream fault: {reason}")


class GripperFollowClient:
    def __init__(self, host: str, port: int, timeout: float) -> None:
        self.host, self.port, self.timeout = host, port, timeout

    def request(self, command: str, **kwargs: Any) -> dict[str, Any]:
        with socket.create_connection(
            (self.host, self.port), timeout=self.timeout
        ) as connection:
            connection.sendall((json.dumps({"cmd": command, **kwargs}) + "\n").encode())
            with connection.makefile("rb") as stream:
                line = stream.readline(4097)
        if not line or len(line) > 4096:
            raise RuntimeError("Invalid gripper response length")
        response = json.loads(line)
        if response.get("ok") is not True:
            raise RuntimeError(f"Gripper: {response.get('error', 'request failed')}")
        result = response.get("result")
        if not isinstance(result, dict):
            raise RuntimeError("Invalid gripper result")
        return result

    def check(self) -> dict[str, Any]:
        state = self.request("follow_status")
        if state.get("streaming") is not True:
            raise RuntimeError("Update the gripper server to support set_target")
        if state.get("stream_error"):
            raise GripperStreamFault(str(state["stream_error"]))
        code, pos = state.get("status_code"), state.get("position_raw")
        if not isinstance(code, int) or code & 0x31 != 0x31:
            raise RuntimeError("Gripper is not activated; start its server first")
        if not isinstance(pos, int) or not 0 <= pos <= 255:
            raise RuntimeError("Invalid gripper position feedback")
        return state


class GripperFollower:
    def __init__(
        self,
        client: GripperFollowClient,
        *,
        hz: float = 5.0,
        speed: int = 150,
        force: int = 0,
        open_pos: int = 0,
        closed_pos: int = 255,
        stale_timeout: float = 1.5,
        initial_feedback: dict[str, Any] | None = None,
    ) -> None:
        self._client = client
        self._period = 1.0 / hz
        self._speed, self._force = speed, force
        self._open, self._closed = open_pos, closed_pos
        self._stale_timeout = stale_timeout
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._target: tuple[float, float] | None = None
        self._error: Exception | None = None
        self._feedback: dict[str, Any] = dict(initial_feedback or {})
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, closure: float) -> None:
        if not math.isfinite(closure) or not 0 <= closure <= 1:
            raise ValueError("Gripper closure must be finite and in [0,1]")
        self.check()
        with self._lock:
            self._target = (closure, time.monotonic())

    def check(self) -> None:
        with self._lock:
            error = self._error
        if error is not None:
            raise RuntimeError(f"Gripper following stopped: {error}") from error

    def feedback(self) -> dict[str, Any]:
        with self._lock:
            return self._feedback.copy()

    def _run(self) -> None:
        attempted = False
        try:
            while not self._stopping.is_set():
                started = time.monotonic()
                with self._lock:
                    target = self._target
                if target is not None:
                    closure, sampled = target
                    if started - sampled > self._stale_timeout:
                        raise RuntimeError("Leader target stream timed out")
                    pos = round(self._open + closure * (self._closed - self._open))
                    attempted = True
                    state = self._client.request(
                        "set_target",
                        pos=pos,
                        speed=self._speed,
                        force=self._force,
                        stale_timeout=self._stale_timeout,
                    )
                    state = dict(
                        state,
                        feedback_time_ns=time.monotonic_ns(),
                        requested_closure=closure,
                        command_position_raw=pos,
                    )
                    with self._lock:
                        self._feedback = state
                self._stopping.wait(max(0, self._period - (time.monotonic() - started)))
        except Exception as exc:
            with self._lock:
                self._error = exc
        finally:
            if attempted:
                try:
                    self._client.request("stop")
                except Exception as exc:
                    with self._lock:
                        self._error = self._error or exc
                    print(
                        f"Gripper stop request failed: {exc}; server watchdog applies"
                    )

    def close(self) -> None:
        self._stopping.set()
        self._thread.join(timeout=2 * self._client.timeout + 1)
        if self._thread.is_alive():
            raise RuntimeError("Gripper worker did not exit; server watchdog applies")
