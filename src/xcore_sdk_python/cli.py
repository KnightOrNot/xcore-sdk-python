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
from pathlib import Path
from typing import Any

from .driver import DEFAULT_SPEED
from .exceptions import XCoreError
from .network import configure_address, diagnose
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


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(
        prog="xcore-sdk-python", description="xCore robot diagnostics and control"
    )
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--ip", type=ipv4, default=os.environ.get("XCORE_ROBOT_IP", "192.168.2.160")
    )
    common.add_argument(
        "--local-ip", type=ipv4, default=os.environ.get("XCORE_LOCAL_IP", "")
    )
    common.add_argument("--sdk-dir", help="Directory containing the vendor extension")
    common.add_argument(
        "--timeout", type=finite, default=20, help="Overall deadline in seconds"
    )
    common.add_argument(
        "--output", type=Path, help="Save JSON to a new file (no overwrite)"
    )
    # An empty local IP is valid for non-real-time SDK queries.
    common.set_defaults(local_ip=os.environ.get("XCORE_LOCAL_IP") or None)
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
    monitor.add_argument("--duration", type=finite, default=5)
    monitor.add_argument("--interval", type=finite, default=0.5)
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
            default=DEFAULT_SPEED,
            help="SDK speed in mm/s (default: %(default)s; allowed: 5..4000)",
        )
        motion.add_argument("--max-step-deg", type=finite, default=10)
        motion.add_argument("--tolerance-deg", type=finite, default=0.2)
        motion.add_argument("--motion-timeout", type=finite, default=15)
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
    follow_server.add_argument("--host", default="127.0.0.1")
    follow_server.add_argument("--port", type=int, default=6001)
    follow_server.add_argument("--enable-motion", action="store_true")
    follow_server.add_argument("--yes", action="store_true")
    follow_server.add_argument("--state-hz", type=finite, default=25.0)
    follow_server.add_argument("--max-speed-deg", type=finite, default=3.0)
    follow_server.add_argument("--accel-deg-s2", type=finite, default=40.0)
    follow_server.add_argument("--stale-timeout", type=finite, default=1.0)
    follow_server.add_argument("--gate-deg", type=finite, default=17.1887)
    follow_server.add_argument("--quiet", action="store_true")

    prepare = sub.add_parser(
        "follow-prepare",
        parents=[copy.deepcopy(common)],
        help="Align CR7 to the stationary calibrated GELLO target before RT follow",
    )
    prepare.set_defaults(timeout=None)
    prepare.add_argument(
        "--serial",
        default="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0",
    )
    prepare.add_argument("--baudrate", type=int, default=57600)
    prepare.add_argument("--calib", type=Path, default=Path("config/cr7_calib.json"))
    prepare.add_argument("--calibrate-zero", action="store_true")
    prepare.add_argument("--signs", type=int, nargs=6, choices=(-1, 1), default=[1] * 6)
    prepare.add_argument("--speed", type=finite, default=DEFAULT_SPEED)
    prepare.add_argument("--max-step-deg", type=finite, default=180)
    prepare.add_argument("--tolerance-deg", type=finite, default=0.2)
    prepare.add_argument("--motion-timeout", type=finite, default=600)
    prepare.add_argument("--gripper-host")
    prepare.add_argument("--gripper-port", type=int, default=5005)
    prepare.add_argument("--gripper-timeout", type=finite, default=0.75)

    gripper_check = sub.add_parser(
        "gripper-check",
        parents=[common],
        help="Check independent gripper feedback without connecting to CR7 or moving",
    )
    gripper_check.add_argument("--gripper-host", required=True)
    gripper_check.add_argument("--gripper-port", type=int, default=5005)
    gripper_check.add_argument("--gripper-timeout", type=finite, default=0.75)

    follow = sub.add_parser(
        "follow", parents=[common], help="Read GELLO leader and stream six CR7 joints"
    )
    follow.add_argument("--host", default="127.0.0.1", help="follow-server host")
    follow.add_argument("--port", type=int, default=6001)
    follow.add_argument(
        "--serial",
        default="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0",
    )
    follow.add_argument("--baudrate", type=int, default=57600)
    follow.add_argument("--calib", type=Path, default=Path("config/cr7_calib.json"))
    follow.add_argument("--hz", type=finite, default=50.0)
    follow.add_argument("--gate-deg", type=finite, default=17.1887)
    follow.add_argument("--dry-run", action="store_true")
    follow.add_argument("--allow-uncalibrated", action="store_true")
    follow.add_argument("--yes", action="store_true")
    follow.add_argument(
        "--gripper-host", help="Enable independent gripper following at this TCP host"
    )
    follow.add_argument("--gripper-port", type=int, default=5005)
    follow.add_argument("--gripper-id", type=int, default=7)
    follow.add_argument("--gripper-open-deg", type=finite, default=194.8)
    follow.add_argument("--gripper-close-deg", type=finite, default=153.0)
    follow.add_argument("--gripper-open-pos", type=int, default=0)
    follow.add_argument("--gripper-closed-pos", type=int, default=255)
    follow.add_argument("--gripper-hz", type=finite, default=5.0)
    follow.add_argument("--gripper-speed", type=int, default=150)
    follow.add_argument("--gripper-force", type=int, default=0)
    follow.add_argument("--gripper-timeout", type=finite, default=0.75)
    follow.add_argument("--gripper-stale-timeout", type=finite, default=1.5)
    follow.add_argument(
        "--raw-data-root", type=Path, help="Enable raw episode recording"
    )
    follow.add_argument("--session-path-file", type=Path)
    follow.add_argument("--task", default="CR7 GELLO teleoperation")
    follow.add_argument("--record-queue-size", type=int, default=500)
    follow.add_argument("--record-feedback-max-age", type=finite, default=0.75)
    follow.add_argument("--start-recording", action="store_true")

    follow_check = sub.add_parser(
        "follow-check",
        parents=[common],
        help="Validate calibration and read the leader once; no CR7 SDK session",
    )
    follow_check.add_argument(
        "--serial",
        default="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0",
    )
    follow_check.add_argument("--baudrate", type=int, default=57600)
    follow_check.add_argument(
        "--calib", type=Path, default=Path("config/cr7_calib.json")
    )

    calibration = sub.add_parser(
        "follow-calibrate",
        parents=[common],
        help="Create read-only single-pose leader offsets; requires matching poses",
    )
    calibration.add_argument(
        "--serial",
        default="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0",
    )
    calibration.add_argument("--baudrate", type=int, default=57600)
    calibration.add_argument(
        "--ref-current",
        action="store_true",
        required=True,
        help="Declare that the leader is already posed like current CR7",
    )
    calibration.add_argument("--signs", type=int, nargs=6, default=[1, 1, 1, 1, 1, 1])
    calibration.add_argument("--samples", type=int, default=30)
    calibration.add_argument("--interval", type=finite, default=0.03)
    calibration.add_argument("--save", type=Path, default=Path("config/cr7_calib.json"))
    network = sub.add_parser(
        "network",
        parents=[common],
        help="Inspect network or modify a temporary PC address",
    )
    network.add_argument(
        "action", choices=("check", "configure", "reset"), nargs="?", default="check"
    )
    network.add_argument("--interface", help="Wired interface for configure/reset")
    network.add_argument(
        "--address", default="192.168.2.100/24", help="Temporary PC IPv4 address"
    )
    return root


def validate(args: argparse.Namespace, root: argparse.ArgumentParser) -> None:
    args.local_ip = args.local_ip or ""
    if args.command == "follow-prepare" and args.timeout is None:
        args.timeout = 2 * args.motion_timeout + 30
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
        terminate(child, grace=10 if options["command"] == "follow-prepare" else 1)
        raise
    for line in reversed(output.splitlines()):
        if line.startswith(RESULT_PREFIX):
            return json.loads(line[len(RESULT_PREFIX) :])
    raise XCoreError(
        f"SDK worker exited {child.returncode}: {errors.strip() or output.strip()}"
    )


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


def main(argv: list[str] | None = None) -> int:
    root = parser()
    args = root.parse_args(argv)
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
    if args.command == "follow-prepare":
        options["calib"] = str(args.calib)
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
