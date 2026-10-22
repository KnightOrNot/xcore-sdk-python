"""Unified `uv run xcore-sdk-python COMMAND OPTIONS` entry point."""

from __future__ import annotations

import argparse
import copy
import ipaddress
import json
import math
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .configuration import Config, load_config
from .exceptions import XCoreError
from .network import configure_address, diagnose
from .return_zero import DEFAULT_EXIT_POSE, load_exit_pose
from .worker import RESULT_PREFIX


def finite(value: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("Value must be finite")
    return number


def ipv4(value: str) -> str:
    try:
        return str(ipaddress.IPv4Address(value))
    except ipaddress.AddressValueError as exc:
        raise argparse.ArgumentTypeError("Expected an IPv4 address") from exc


def parser(config: Config | None = None) -> argparse.ArgumentParser:
    config = config or load_config()
    root = argparse.ArgumentParser(
        prog="xcore-sdk-python", description="xCore robot diagnostics and control"
    )
    root.add_argument("--config", type=Path, help="TOML defaults file")
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--ip", type=ipv4, default=os.environ.get("XCORE_ROBOT_IP", config.robot.ip)
    )
    common.add_argument(
        "--local-ip",
        type=ipv4,
        default=os.environ.get("XCORE_LOCAL_IP", config.robot.local_ip),
    )
    common.add_argument(
        "--sdk-dir",
        default=config.robot.sdk_dir or None,
        help="Directory containing the vendor extension",
    )
    common.add_argument(
        "--timeout",
        type=finite,
        default=config.robot.timeout_s,
        help="Overall deadline in seconds",
    )
    common.add_argument(
        "--output", type=Path, help="Save JSON to a new file (no overwrite)"
    )
    common.add_argument(
        "--config",
        type=Path,
        default=argparse.SUPPRESS,
        help="TOML defaults file (default: <repository>/config/default.toml)",
    )
    # An empty local IP is valid for non-real-time SDK queries.
    common.set_defaults(
        local_ip=os.environ.get("XCORE_LOCAL_IP") or config.robot.local_ip or None
    )
    sub = root.add_subparsers(dest="command", required=True)
    for name, help_text in (
        (
            "doctor",
            "Check Python, SDK binary and API availability; no robot connection",
        ),
        ("status", "Read robot information, modes, joints and flange pose"),
        ("info", "Read model, controller version and serial number"),
        ("joints", "Read all six joint angles in radians and degrees"),
        ("limits", "Read controller soft limits"),
        ("controller-logs", "Read the latest controller errors and warnings"),
        ("stop", "Request a controlled stop"),
        ("check", "Run offline tests and lint checks"),
    ):
        entry = sub.add_parser(name, parents=[common], help=help_text)
        if name == "check":
            entry.add_argument(
                "--fix",
                action="store_true",
                help="Apply lint/format fixes before testing",
            )
    pose = sub.add_parser("pose", parents=[common], help="Read Cartesian pose")
    pose.add_argument("--frame", choices=("flange", "tool"), default="flange")
    dh = sub.add_parser("dh", parents=[common], help="Read DH parameters")
    dh.add_argument(
        "--nominal", action="store_true", help="Read nominal instead of calibrated DH"
    )
    monitor = sub.add_parser(
        "monitor", parents=[common], help="Collect status in one session"
    )
    monitor.add_argument("--duration", type=finite, default=config.monitor.duration)
    monitor.add_argument("--interval", type=finite, default=config.monitor.interval)
    power = sub.add_parser("power", parents=[common], help="Explicitly power on/off")
    power.add_argument("value", choices=("on", "off"))
    mode = sub.add_parser(
        "mode", parents=[common], help="Explicitly select operation mode"
    )
    mode.add_argument("value", choices=("manual", "automatic"))
    for name in ("movej", "move-joint"):
        motion = sub.add_parser(
            name, parents=[common], help="Execute a bounded joint move"
        )
        motion.add_argument(
            "--speed",
            type=finite,
            default=config.motion.speed,
            help="SDK speed in mm/s (default: %(default)s; allowed: 5..4000)",
        )
        motion.add_argument(
            "--max-step-deg", type=finite, default=config.motion.max_step_deg
        )
        motion.add_argument(
            "--tolerance-deg", type=finite, default=config.motion.tolerance_deg
        )
        motion.add_argument(
            "--motion-timeout", type=finite, default=config.motion.motion_timeout_s
        )
        if name == "movej":
            motion.add_argument("--joints", type=finite, nargs=6, required=True)
            motion.add_argument("--unit", choices=("deg", "rad"), default="deg")
        else:
            motion.add_argument("--joint", type=int, choices=range(1, 7), required=True)
            motion.add_argument("--delta-deg", type=finite, required=True)
    follow_server = sub.add_parser(
        "follow-server",
        parents=[common],
        help="Run the long-lived CR7 GELLO endpoint (dry-run unless enabled)",
    )
    follow_server.add_argument("--host", default=config.server.host)
    follow_server.add_argument("--port", type=int, default=config.server.port)
    follow_server.add_argument("--enable-motion", action="store_true")
    follow_server.add_argument("--yes", action="store_true")
    follow_server.add_argument(
        "--state-hz", type=finite, default=config.server.state_hz
    )
    follow_server.add_argument(
        "--max-speed-deg", type=finite, default=config.server.max_speed_deg
    )
    follow_server.add_argument(
        "--accel-deg-s2", type=finite, default=config.server.accel_deg_s2
    )
    follow_server.add_argument(
        "--stale-timeout", type=finite, default=config.server.stale_timeout_s
    )
    follow_server.add_argument(
        "--gate-deg", type=finite, default=config.server.gate_deg
    )
    follow_server.add_argument("--quiet", action="store_true")

    prepare = sub.add_parser(
        "follow-prepare",
        parents=[copy.deepcopy(common)],
        help="Align CR7 to the stationary calibrated GELLO target before RT follow",
    )
    prepare.set_defaults(timeout=None)
    prepare.add_argument("--serial", default=config.gello.port)
    prepare.add_argument("--baudrate", type=int, default=config.gello.baudrate)
    prepare.add_argument("--calib", type=Path, default=Path(config.calibration.file))
    prepare.add_argument("--calibrate-zero", action="store_true")
    prepare.add_argument(
        "--signs", type=int, nargs=6, choices=(-1, 1), default=list(config.gello.signs)
    )
    prepare.add_argument("--speed", type=finite, default=config.prepare.speed)
    prepare.add_argument(
        "--max-step-deg", type=finite, default=config.prepare.max_step_deg
    )
    prepare.add_argument(
        "--tolerance-deg", type=finite, default=config.prepare.tolerance_deg
    )
    prepare.add_argument(
        "--motion-timeout", type=finite, default=config.prepare.motion_timeout_s
    )
    prepare.add_argument("--gripper-host")
    prepare.add_argument(
        "--gripper-port", type=int, default=config.prepare.gripper_port
    )
    prepare.add_argument(
        "--gripper-timeout", type=finite, default=config.prepare.gripper_timeout_s
    )

    zero = sub.add_parser(
        "return-pose",
        aliases=["return-zero"],
        parents=[copy.deepcopy(common)],
        help="Return to the saved exit posture (return-zero is a legacy alias)",
    )
    zero.set_defaults(timeout=None)
    zero.add_argument("--speed", type=finite, default=config.return_.speed)
    zero.add_argument(
        "--max-step-deg", type=finite, default=config.return_.max_step_deg
    )
    zero.add_argument(
        "--tolerance-deg", type=finite, default=config.return_.tolerance_deg
    )
    zero.add_argument(
        "--motion-timeout", type=finite, default=config.return_.motion_timeout_s
    )
    pose_target = zero.add_mutually_exclusive_group()
    pose_target.add_argument(
        "--pose-file", type=Path, default=Path(config.return_.pose_file)
    )
    pose_target.add_argument(
        "--zero",
        action="store_true",
        help="Explicitly return to six zero angles instead of the saved exit pose",
    )
    zero.add_argument(
        "--recover",
        action="store_true",
        help="Recover stopped RT once before returning; finish powered off/manual",
    )

    capture_pose = sub.add_parser(
        "capture-pose",
        parents=[common],
        help="Save the current six joints as a return target; no motion",
    )
    capture_pose.add_argument(
        "--save", type=Path, required=True, help="New pose JSON file (no overwrite)"
    )
    capture_pose.add_argument(
        "--zero",
        action="store_true",
        help="Write six zero angles without connecting to the robot",
    )

    gripper_check = sub.add_parser(
        "gripper-check",
        parents=[common],
        help="Check independent gripper feedback without connecting to CR7 or moving",
    )
    gripper_check.add_argument("--gripper-host", required=True)
    gripper_check.add_argument("--gripper-port", type=int, default=config.gripper.port)
    gripper_check.add_argument(
        "--gripper-timeout", type=finite, default=config.gripper.timeout_s
    )

    follow = sub.add_parser(
        "follow", parents=[common], help="Read GELLO leader and stream six CR7 joints"
    )
    follow.add_argument("--host", default=config.follow.host, help="follow-server host")
    follow.add_argument("--port", type=int, default=config.follow.port)
    follow.add_argument(
        "--show-state", action="store_true", help="Print per-cycle state (default: off)"
    )
    follow.add_argument("--serial", default=config.gello.port)
    follow.add_argument("--baudrate", type=int, default=config.gello.baudrate)
    follow.add_argument("--calib", type=Path, default=Path(config.calibration.file))
    follow.add_argument("--hz", type=finite, default=config.follow.hz)
    follow.add_argument("--gate-deg", type=finite, default=config.follow.gate_deg)
    follow.add_argument("--dry-run", action="store_true")
    follow.add_argument("--allow-uncalibrated", action="store_true")
    follow.add_argument("--yes", action="store_true")
    follow.add_argument(
        "--gripper-host", help="Enable independent gripper following at this TCP host"
    )
    follow.add_argument("--gripper-port", type=int, default=config.gripper.port)
    follow.add_argument("--gripper-id", type=int, default=config.gripper.id)
    follow.add_argument(
        "--gripper-open-deg", type=finite, default=config.gripper.open_deg
    )
    follow.add_argument(
        "--gripper-close-deg", type=finite, default=config.gripper.close_deg
    )
    follow.add_argument("--gripper-open-pos", type=int, default=config.gripper.open_pos)
    follow.add_argument(
        "--gripper-closed-pos", type=int, default=config.gripper.closed_pos
    )
    follow.add_argument("--gripper-hz", type=finite, default=config.gripper.hz)
    follow.add_argument("--gripper-speed", type=int, default=config.gripper.speed)
    follow.add_argument("--gripper-force", type=int, default=config.gripper.force)
    follow.add_argument(
        "--gripper-timeout", type=finite, default=config.gripper.timeout_s
    )
    follow.add_argument(
        "--gripper-stale-timeout", type=finite, default=config.gripper.stale_timeout_s
    )
    follow.add_argument(
        "--raw-data-root", type=Path, help="Enable raw episode recording"
    )
    follow.add_argument("--session-path-file", type=Path)
    follow.add_argument("--task", default=config.recording.task)
    follow.add_argument(
        "--record-queue-size", type=int, default=config.recording.queue_size
    )
    follow.add_argument(
        "--record-feedback-max-age",
        type=finite,
        default=config.recording.feedback_max_age_s,
    )
    follow.add_argument("--start-recording", action="store_true")

    follow_check = sub.add_parser(
        "follow-check",
        parents=[common],
        help="Validate calibration and read the leader once; no CR7 SDK session",
    )
    follow_check.add_argument("--serial", default=config.gello.port)
    follow_check.add_argument("--baudrate", type=int, default=config.gello.baudrate)
    follow_check.add_argument(
        "--calib", type=Path, default=Path(config.calibration.file)
    )

    calibration = sub.add_parser(
        "follow-calibrate",
        parents=[common],
        help="Create read-only single-pose leader offsets; requires matching poses",
    )
    calibration.add_argument("--serial", default=config.gello.port)
    calibration.add_argument("--baudrate", type=int, default=config.gello.baudrate)
    calibration.add_argument(
        "--ref-current",
        action="store_true",
        required=True,
        help="Declare that the leader is already posed like current CR7",
    )
    calibration.add_argument(
        "--signs", type=int, nargs=6, default=list(config.gello.signs)
    )
    calibration.add_argument("--samples", type=int, default=config.calibration.samples)
    calibration.add_argument(
        "--interval", type=finite, default=config.calibration.interval_s
    )
    calibration.add_argument("--save", type=Path, default=Path(config.calibration.file))
    network = sub.add_parser(
        "network",
        parents=[common],
        help="Inspect network or modify a temporary PC address",
    )
    network.add_argument(
        "action", choices=("check", "configure", "reset"), nargs="?", default="check"
    )
    network.add_argument(
        "--interface",
        default=config.network.interface or None,
        help="Wired interface for configure/reset",
    )
    network.add_argument(
        "--address",
        default=config.network.address,
        help="Temporary PC IPv4 address",
    )
    return root


def validate(args: argparse.Namespace, root: argparse.ArgumentParser) -> None:
    args.local_ip = args.local_ip or ""
    if args.command == "follow-prepare" and args.timeout is None:
        args.timeout = 2 * args.motion_timeout + 30
    if args.command in ("return-pose", "return-zero") and args.timeout is None:
        args.timeout = args.motion_timeout + 30
    if args.timeout <= 0:
        root.error("--timeout must be positive")
    if args.output and args.output.exists():
        root.error("--output already exists; choose a new file")
    if args.command == "monitor":
        if (
            args.interval <= 0
            or args.duration <= 0
            or args.duration + 5 >= args.timeout
        ):
            root.error(
                "monitor needs positive duration/interval; "
                "--timeout must exceed --duration + 5"
            )
        if args.duration / args.interval > 10000:
            root.error("monitor is limited to 10000 samples per invocation")
    if args.command in ("movej", "move-joint"):
        if not 5 <= args.speed <= 4000:
            root.error("--speed must be in [5, 4000] mm/s")
        if min(args.max_step_deg, args.tolerance_deg, args.motion_timeout) <= 0:
            root.error("step limit, tolerance and motion timeout must be positive")
        if args.motion_timeout + 2 >= args.timeout:
            root.error("--timeout must exceed --motion-timeout + 2")
        if args.command == "move-joint" and abs(args.delta_deg) > args.max_step_deg:
            root.error("--delta-deg exceeds --max-step-deg")
    if args.command in ("return-pose", "return-zero"):
        if not 5 <= args.speed <= 4000:
            root.error("return-zero speed must be in [5, 4000] mm/s")
        if min(args.max_step_deg, args.tolerance_deg, args.motion_timeout) <= 0:
            root.error("return-zero limits and timeouts must be positive")
        if args.timeout <= args.motion_timeout + 10:
            root.error("return-zero timeout must exceed motion timeout plus 10s")
        if not args.zero and (
            args.command == "return-pose"
            or args.pose_file.exists()
            or args.pose_file != DEFAULT_EXIT_POSE
        ):
            try:
                load_exit_pose(args.pose_file, args.ip)
            except (OSError, ValueError, XCoreError) as exc:
                root.error(f"Invalid exit pose: {exc}")
    if args.command == "gripper-check":
        if not 1 <= args.gripper_port <= 65535 or args.gripper_timeout <= 0:
            root.error("invalid gripper port or timeout")
    if args.command == "follow-prepare":
        if not 5 <= args.speed <= 4000:
            root.error("preparation speed must be in [5,4000] mm/s")
        if min(args.max_step_deg, args.tolerance_deg, args.motion_timeout) <= 0:
            root.error("preparation limits and timeouts must be positive")
        if args.timeout <= 2 * args.motion_timeout + 10:
            root.error("preparation timeout must exceed both motion timeouts plus 10s")
        if not 1 <= args.gripper_port <= 65535 or args.gripper_timeout <= 0:
            root.error("invalid gripper port or timeout")
    if args.command == "network" and args.action != "check" and not args.interface:
        root.error("network configure/reset requires --interface")
    if args.command == "network" and args.action == "configure":
        try:
            address = ipaddress.IPv4Interface(args.address)
        except ValueError:
            root.error("--address must be an IPv4 address with prefix length")
        if str(address.ip) == args.ip:
            root.error("The PC address must differ from the robot IP")
    if args.command == "follow-server":
        if (
            args.port < 1
            or args.port > 65535
            or min(
                args.state_hz,
                args.max_speed_deg,
                args.accel_deg_s2,
                args.stale_timeout,
                args.gate_deg,
            )
            <= 0
        ):
            root.error("follow-server port and motion/watchdog values must be positive")
        if args.max_speed_deg > 75:
            root.error("--max-speed-deg is limited to 75 deg/s")
    if args.command == "follow":
        if args.port < 1 or args.port > 65535 or min(args.hz, args.gate_deg) <= 0:
            root.error("follow port, frequency and alignment gate must be positive")
        if not args.dry_run and args.allow_uncalibrated:
            root.error("--allow-uncalibrated is only allowed with --dry-run")
        if args.raw_data_root is not None:
            if args.dry_run or not args.gripper_host:
                root.error("Recording requires motion following and --gripper-host")
            if (
                args.record_queue_size <= 0
                or args.record_feedback_max_age <= 0
                or not args.task.strip()
            ):
                root.error("Invalid recording queue, feedback age or task")
        elif args.start_recording or args.session_path_file is not None:
            root.error("Recording options require --raw-data-root")
        if not 1 <= args.gripper_port <= 65535 or not 7 <= args.gripper_id <= 252:
            root.error("Invalid gripper port or ID (arm IDs 1..6 are reserved)")
        if args.gripper_open_deg == args.gripper_close_deg:
            root.error("Gripper angle endpoints must differ")
        if (
            any(
                not 0 <= value <= 255
                for value in (
                    args.gripper_open_pos,
                    args.gripper_closed_pos,
                    args.gripper_speed,
                    args.gripper_force,
                )
            )
            or args.gripper_open_pos == args.gripper_closed_pos
        ):
            root.error("Gripper byte values must be 0..255 and endpoints must differ")
        if (
            not 0 < args.gripper_hz <= 10
            or args.gripper_timeout <= 0
            or not 0.5 <= args.gripper_stale_timeout <= 10
            or args.gripper_stale_timeout <= 1 / args.gripper_hz + args.gripper_timeout
        ):
            root.error("Gripper stale timeout must exceed period + request timeout")
    if args.command == "follow-calibrate" and (args.samples < 2 or args.interval <= 0):
        root.error("calibration needs at least two samples and a positive interval")
    if args.command == "follow-calibrate" and args.save.exists():
        root.error("--save already exists; choose a new calibration file")
    if args.command == "capture-pose":
        if args.save.exists():
            root.error("--save already exists; choose a new pose file")
        if args.timeout <= 0:
            root.error("--timeout must be positive")


def terminate(process: subprocess.Popen, *, grace: float = 1) -> None:
    process.terminate()
    try:
        process.communicate(timeout=grace)
    except subprocess.TimeoutExpired:
        process.kill()
        process.communicate()


def isolated(options: dict[str, Any]) -> dict[str, Any]:
    child = subprocess.Popen(
        [sys.executable, "-m", "xcore_sdk_python.worker"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        output, errors = child.communicate(
            json.dumps(options), timeout=options["timeout"]
        )
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        terminate(
            child,
            grace=10
            if options["command"] in ("follow-prepare", "return-zero", "return-pose")
            else 1,
        )
        raise
    if errors.strip():
        print(errors.rstrip(), file=sys.stderr, flush=True)
    for line in reversed(output.splitlines()):
        if line.startswith(RESULT_PREFIX):
            return json.loads(line[len(RESULT_PREFIX) :])
    raise XCoreError(
        f"SDK worker exited {child.returncode}: {errors.strip() or output.strip()}"
    )


def capture_pose(args: argparse.Namespace) -> dict[str, Any]:
    """Write a schema-1 return pose from current feedback or explicit zero."""

    path = Path(args.save)
    path.parent.mkdir(parents=True, exist_ok=True)
    if args.zero:
        radians = [0.0] * 6
        degrees = [0.0] * 6
        source = "explicit zero return target; no robot connection"
    else:
        response = isolated(
            {
                "command": "joints",
                "ip": args.ip,
                "local_ip": args.local_ip or "",
                "timeout": args.timeout,
                "sdk_dir": args.sdk_dir,
            }
        )
        if not response.get("ok"):
            raise XCoreError(response.get("error", "joints query failed"))
        result = response["result"]
        radians = [float(value) for value in result["rad"][:6]]
        degrees = [float(value) for value in result["deg"][:6]]
        if len(radians) != 6 or len(degrees) != 6:
            raise XCoreError("Current feedback did not contain six joints")
        source = "CR7 actual joint feedback captured before follow; read-only"
    data = {
        "schema_version": 1,
        "robot_ip": args.ip,
        "joints_rad": radians,
        "joints_deg": degrees,
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "source": source,
    }
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return {
        "saved": str(path.resolve()),
        "joints_rad": radians,
        "joints_deg": degrees,
        "source": source,
    }


def development_check(timeout: float, *, fix: bool = False) -> dict[str, Any]:
    root = Path(__file__).resolve().parents[2]
    reports = {}
    for name, command in (
        (
            "lint",
            [sys.executable, "-m", "ruff", "check", "."] + (["--fix"] if fix else []),
        ),
        (
            "format",
            [sys.executable, "-m", "ruff", "format"]
            + ([] if fix else ["--check"])
            + ["src", "tests"],
        ),
        ("tests", [sys.executable, "-m", "pytest", "-q"]),
    ):
        completed = subprocess.run(
            command, cwd=root, capture_output=True, text=True, timeout=timeout
        )
        reports[name] = {
            "passed": completed.returncode == 0,
            "output": completed.stdout + completed.stderr,
        }
    return {
        "ok": all(report["passed"] for report in reports.values()),
        "command": "check",
        "result": reports,
    }


def config_path(argv: list[str]) -> Path | None:
    """Find ``--config`` before argparse so defaults can be built from it."""

    selected = None
    for index, value in enumerate(argv):
        if value == "--config" and index + 1 < len(argv):
            selected = Path(argv[index + 1])
        if value.startswith("--config="):
            selected = Path(value.split("=", 1)[1])
    return selected


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    try:
        config = load_config(config_path(arguments))
    except (OSError, ValueError) as exc:
        print(f"xcore: invalid configuration: {exc}", file=sys.stderr)
        return 2
    root = parser(config)
    args = root.parse_args(arguments)
    validate(args, root)
    if args.command in ("follow-server", "follow", "follow-calibrate", "follow-check"):
        try:
            if args.command == "follow-server":
                from .gello_server import run_server

                return run_server(args)
            if args.command == "follow":
                from .gello_client import run_client

                return run_client(args)
            if args.command == "follow-check":
                from .calibration import run_check

                return run_check(args)
            from .calibration import calibrate

            return calibrate(args)
        except KeyboardInterrupt:
            return 130
        except Exception as exc:
            print(f"xcore {args.command}: {type(exc).__name__}: {exc}", file=sys.stderr)
            return 1
    options = vars(args).copy()
    options.pop("output")
    options.pop("config", None)
    if args.command == "follow-prepare":
        options["calib"] = str(args.calib)
    if args.command in ("return-pose", "return-zero"):
        options["pose_file"] = str(args.pose_file)
    try:
        if args.command == "network":
            result = (
                diagnose(args.ip, probe_timeout=min(2, args.timeout))
                if args.action == "check"
                else configure_address(
                    args.interface, args.address, remove=args.action == "reset"
                )
            )
            response = {"ok": True, "command": "network", "result": result}
        elif args.command == "check":
            response = development_check(args.timeout, fix=args.fix)
        elif args.command == "capture-pose":
            response = {
                "ok": True,
                "command": "capture-pose",
                "result": capture_pose(args),
            }
        else:
            response = isolated(options)
    except subprocess.TimeoutExpired:
        response = {
            "ok": False,
            "command": args.command,
            "error": f"Timed out after {args.timeout}s",
        }
        print(json.dumps(response), file=sys.stderr)
        return 124
    except KeyboardInterrupt:
        print(json.dumps({"ok": False, "error": "Interrupted"}), file=sys.stderr)
        return 130
    except (XCoreError, ValueError, OSError) as exc:
        response = {"ok": False, "command": args.command, "error": str(exc)}
    text = json.dumps(response, ensure_ascii=False, indent=2, allow_nan=False)
    if response["ok"] and args.output:
        try:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(text + "\n")
        except OSError as exc:
            print(json.dumps({"ok": False, "error": str(exc)}), file=sys.stderr)
            return 1
    print(text, file=sys.stdout if response["ok"] else sys.stderr)
    return 0 if response["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
