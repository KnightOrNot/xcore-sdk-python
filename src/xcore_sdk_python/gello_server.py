"""Small, local ZMQ bridge implementing GELLO's four-method robot protocol."""

from __future__ import annotations

import math
import pickle
import signal
import threading
from typing import Any

from .gello_follower import GelloCr7Robot
from .reader import RobotConnection


class CheckedRobot:
    """Raise on SDK error-code dictionaries used by the direct RT adapter."""

    def __init__(self, robot: Any) -> None:
        self._robot = robot

    def __getattr__(self, name: str) -> Any:
        operation = getattr(self._robot, name)

        def call(*args: Any, **kwargs: Any) -> Any:
            ec = args[-1] if args and isinstance(args[-1], dict) else None
            if ec is not None:
                ec.clear()
            result = operation(*args, **kwargs)
            if ec and ec.get("ec") != 0:
                raise RuntimeError(f"{name}: {ec}")
            return result

        return call


def dispatch(robot: GelloCr7Robot, request: dict[str, Any]) -> Any:
    method = request.get("method")
    args = request.get("args", {})
    if method == "num_dofs":
        return robot.num_dofs()
    if method == "get_joint_state":
        return robot.get_joint_state()
    if method == "command_joint_state":
        return robot.command_joint_state(**args)
    if method == "get_observations":
        return robot.get_observations()
    raise ValueError(f"unsupported robot method: {method!r}")


def run_server(args: Any) -> int:
    """Run the long-lived CR7 endpoint; motion is opt-in and dry-run is default."""
    try:
        import zmq
    except ImportError as exc:
        raise RuntimeError("ZMQ support missing; run `uv sync`") from exc

    if args.enable_motion and not args.yes:
        answer = input("即将上电并进入 CR7 实时跟随模式，输入 y 确认: ").strip().lower()
        if answer not in ("y", "yes"):
            print("已取消（未连接机器人）。")
            return 1

    connection = RobotConnection(args.ip, local_ip=args.local_ip, sdk_dir=args.sdk_dir)
    bot = None
    context = None
    socket = None
    stopping = threading.Event()
    old_int = signal.signal(signal.SIGINT, lambda *_: stopping.set())
    old_term = signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    try:
        connection.__enter__()
        robot, sdk = CheckedRobot(connection.robot), connection.sdk
        if (
            args.enable_motion
            and connection.call("operationState") != sdk.OperationState.idle
        ):
            raise RuntimeError("Following requires an idle robot")
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.setsockopt(zmq.RCVTIMEO, 500)
        socket.setsockopt(zmq.LINGER, 0)
        socket.bind(f"tcp://{args.host}:{args.port}")
        bot = GelloCr7Robot(
            robot,
            sdk,
            enable_motion=args.enable_motion,
            control_hz=args.state_hz,
            max_speed=math.radians(args.max_speed_deg),
            accel=math.radians(args.accel_deg_s2),
            stale_timeout=args.stale_timeout,
            arm_gate_rad=math.radians(args.gate_deg),
            verbose=not args.quiet,
        )
        print(
            f"CR7 follow server: tcp://{args.host}:{args.port}; "
            f"{'motion armed on first target' if args.enable_motion else 'dry-run'}",
            flush=True,
        )
        while not stopping.is_set() and bot.abort_reason() is None:
            try:
                request = pickle.loads(socket.recv())
            except zmq.Again:
                continue
            try:
                response = dispatch(bot, request)
            except Exception as exc:
                response = {"error": f"{type(exc).__name__}: {exc}"}
            socket.send(pickle.dumps(response))
        return 1 if bot.abort_reason() else 0
    finally:
        if bot is not None:
            bot.close()
        if socket is not None:
            socket.close(0)
        if context is not None:
            context.term()
        connection.close()
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)


class ZmqRobotClient:
    """Synchronous REQ client with bounded waits, compatible with GELLO's wire API."""

    def __init__(self, host: str, port: int, timeout_ms: int = 2000) -> None:
        import zmq

        self._zmq = zmq
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._socket.connect(f"tcp://{host}:{port}")

    def call(self, method: str, **args: Any) -> Any:
        try:
            self._socket.send(pickle.dumps({"method": method, "args": args}))
            result = pickle.loads(self._socket.recv())
        except self._zmq.Again as exc:
            raise RuntimeError(
                f"CR7 service timed out during {method}; "
                "check the server/controller error"
            ) from exc
        if isinstance(result, dict) and "error" in result:
            raise RuntimeError(result["error"])
        return result

    def close(self) -> None:
        self._socket.close(0)
        self._context.term()
