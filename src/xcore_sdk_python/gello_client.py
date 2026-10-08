"""Read-only leader calibration and six-axis CR7 teleoperation client."""

from __future__ import annotations

import math
import os
import signal
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from .calibration import load_calibration
from .gello_server import ZmqRobotClient
from .gripper_follow import GripperFollowClient, GripperFollower
from .raw_recorder import RawEpisodeRecorder
from .recording import RecordingKeyboard, handle_key, sample_from_cycle


def run_client(args: Any) -> int:
    from .gello_leader import Cr7LeaderAgent

    calibration_path = Path(args.calib)
    if calibration_path.exists():
        offsets, signs = load_calibration(calibration_path)
    elif args.dry_run and args.allow_uncalibrated:
        from .dynamixel_reader import JOINT_OFFSETS, JOINT_SIGNS

        offsets, signs = JOINT_OFFSETS, JOINT_SIGNS
        print("WARNING: using placeholder mapping for dry-run only")
    else:
        raise ValueError(
            f"Calibration file required: {calibration_path}; run follow-calibrate first"
        )

    gripper_client = None
    if args.gripper_host:
        gripper_client = GripperFollowClient(
            args.gripper_host, args.gripper_port, args.gripper_timeout
        )
        state = gripper_client.check()  # Read-only capability/activation check.
        print(f"Gripper ready: {args.gripper_host}:{args.gripper_port}; {state}")
    agent_options = {}
    if gripper_client is not None:
        agent_options = {
            "gripper_id": args.gripper_id,
            "gripper_config": (args.gripper_open_deg, args.gripper_close_deg),
        }
    agent = None
    client = None
    gripper = None
    recorder = None
    keyboard = None
    primary_error = None
    old_term = signal.signal(signal.SIGTERM, _interrupt)
    try:
        agent = Cr7LeaderAgent(
            args.serial,
            baudrate=args.baudrate,
            joint_offsets=offsets,
            joint_signs=signs,
            **agent_options,
        )
        client = ZmqRobotClient(args.host, args.port)
        if client.call("num_dofs") != 6:
            raise RuntimeError("CR7 endpoint must expose exactly six arm joints")
        q_robot = np.asarray(client.call("get_joint_state"), dtype=float)
        q_raw = agent.q_without_branch()
        branch = np.round((q_raw - q_robot) / (2 * math.pi))
        agent.set_branch(branch)
        q_leader = q_raw - branch * 2 * math.pi
        delta = q_leader - q_robot
        error = float(np.max(np.abs(delta)))
        print(
            f"Startup alignment max error: {math.degrees(error):.2f} deg "
            f"(gate {args.gate_deg:.1f} deg); per-axis deg="
            f"{np.round(np.degrees(delta), 2).tolist()}"
        )
        if not args.dry_run and error > math.radians(args.gate_deg):
            raise RuntimeError("startup alignment failed; no motion command sent")
        if args.dry_run:
            print("Dry-run only: reading arm/gripper values; no motion commands sent.")
        else:
            print(
                "Enable server motion explicitly; keep the physical E-stop reachable."
            )
            if not args.yes and input(
                "Type y to start following: "
            ).strip().lower() not in ("y", "yes"):
                print("Cancelled")
                return 1
            if gripper_client is not None:
                initial_feedback = None
                if args.raw_data_root is not None:
                    initial_feedback = dict(
                        gripper_client.check(), feedback_time_ns=time.monotonic_ns()
                    )
                gripper = GripperFollower(
                    gripper_client,
                    hz=args.gripper_hz,
                    speed=args.gripper_speed,
                    force=args.gripper_force,
                    open_pos=args.gripper_open_pos,
                    closed_pos=args.gripper_closed_pos,
                    stale_timeout=args.gripper_stale_timeout,
                    initial_feedback=initial_feedback,
                )

        if args.raw_data_root is not None:
            recorder = RawEpisodeRecorder(
                args.raw_data_root,
                control_hz=args.hz,
                joint_signs=signs,
                task=args.task,
                queue_size=args.record_queue_size,
                metadata={
                    "calibration": str(calibration_path.resolve()),
                    "joint_offsets": offsets,
                    "gripper_host": args.gripper_host,
                    "gripper_port": args.gripper_port,
                    "gripper_hz": args.gripper_hz,
                    "gripper_open_deg": args.gripper_open_deg,
                    "gripper_close_deg": args.gripper_close_deg,
                    "gripper_open_pos": args.gripper_open_pos,
                    "gripper_closed_pos": args.gripper_closed_pos,
                    "feedback_max_age_s": args.record_feedback_max_age,
                },
            )
            if args.session_path_file is not None:
                path = args.session_path_file.resolve()
                path.parent.mkdir(parents=True, exist_ok=True)
                temporary = path.with_suffix(path.suffix + ".partial")
                temporary.write_text(str(recorder.session_dir.resolve()) + "\n")
                os.replace(temporary, path)
            print(f"Raw session: {recorder.session_dir}")
            print("R=开始，S=保存，D=丢弃，P=状态，H=帮助；退出保留未保存 .partial")
            keyboard = RecordingKeyboard()
            keyboard.open()
            if args.start_recording:
                recorder.start_episode()

        period = 1.0 / args.hz
        previous_command_ns = None
        while True:
            started = time.perf_counter()
            if recorder is not None:
                handle_key(keyboard.poll(), recorder)
            if gripper is not None:
                gripper.check()
            sample = np.asarray(agent.get_joint_state(), dtype=float)
            expected = 7 if gripper_client is not None else 6
            if sample.shape != (expected,) or not np.all(np.isfinite(sample)):
                raise RuntimeError(f"Expected {expected} valid leader channels")
            target = sample[:6]
            command_ns = time.monotonic_ns()
            wall_ns = time.time_ns()
            if not args.dry_run:
                client.call("command_joint_state", joint_state=target.tolist())
                if gripper is not None:
                    gripper.submit(float(sample[6]))
            if recorder is not None:
                observations = client.call("get_observations")
                actual = np.asarray(observations["joint_positions"], dtype=float)
                feedback = gripper.feedback()
                observed_ns = time.monotonic_ns()
                if recorder.is_recording:
                    recorder.add_sample(
                        sample_from_cycle(
                            action=sample,
                            arm=observations,
                            gripper=feedback,
                            command_time_ns=command_ns,
                            observation_time_ns=observed_ns,
                            wall_time_ns=wall_ns,
                            control_period_ns=0
                            if previous_command_ns is None
                            else command_ns - previous_command_ns,
                            open_pos=args.gripper_open_pos,
                            closed_pos=args.gripper_closed_pos,
                            max_age_s=args.record_feedback_max_age,
                        )
                    )
            elif args.show_state:
                actual = np.asarray(client.call("get_joint_state"), dtype=float)[:6]
            previous_command_ns = command_ns
            if args.show_state:
                print(
                    "\rleader="
                    + str(np.round(np.degrees(target), 1).tolist())
                    + " robot="
                    + str(np.round(np.degrees(actual), 1).tolist())
                    + (
                        f" gripper_target={sample[6]:.3f}"
                        + (f" gripper_feedback={gripper.feedback()}" if gripper else "")
                        if gripper_client is not None
                        else ""
                    ),
                    end="",
                    flush=True,
                )
            remaining = period - (time.perf_counter() - started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nStopping leader client; server watchdog will stop on command timeout.")
        return 0
    except Exception as exc:
        primary_error = exc
        print(f"[跟随错误] {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        raise
    finally:
        # uv may forward the same group termination to Python again. Cleanup
        # must finish the gripper stop acknowledgement and recorder flush.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        cleanup_errors = []

        def clean(label, operation):
            try:
                operation()
            except Exception as exc:
                cleanup_errors.append(exc)
                print(
                    f"[{label}] {type(exc).__name__}: {exc}",
                    file=sys.stderr,
                    flush=True,
                )

        try:
            if keyboard is not None:
                clean("键盘清理错误", keyboard.close)
            if gripper is not None:
                clean("夹爪停止错误", gripper.close)
                clean("夹爪跟随错误", gripper.check)
            if agent is not None:
                clean("GELLO 关闭错误", agent.close)
            if client is not None:
                clean("CR7 客户端关闭错误", client.close)
            if recorder is not None and recorder.is_recording:

                def finish_recording():
                    print(
                        f"Unfinished episode retained: {recorder.close_interrupted()}"
                    )

                clean("记录关闭错误", finish_recording)
        finally:
            signal.signal(signal.SIGTERM, old_term)
        if cleanup_errors and primary_error is None:
            raise cleanup_errors[0]


def _interrupt(*_args: Any) -> None:
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    raise KeyboardInterrupt
