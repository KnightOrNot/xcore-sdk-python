"""Read-only leader calibration and six-axis CR7 teleoperation client."""

from __future__ import annotations

import math
import signal
import time
from pathlib import Path
from typing import Any

import numpy as np

from .calibration import load_calibration
from .gello_server import ZmqRobotClient
from .gripper_follow import GripperFollowClient, GripperFollower


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
            print("Dry-run only: showing arm/gripper values; no motion commands sent.")
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
                gripper = GripperFollower(
                    gripper_client,
                    hz=args.gripper_hz,
                    speed=args.gripper_speed,
                    force=args.gripper_force,
                    open_pos=args.gripper_open_pos,
                    closed_pos=args.gripper_closed_pos,
                    stale_timeout=args.gripper_stale_timeout,
                )

        period = 1.0 / args.hz
        while True:
            started = time.perf_counter()
            if gripper is not None:
                gripper.check()
            sample = np.asarray(agent.get_joint_state(), dtype=float)
            expected = 7 if gripper_client is not None else 6
            if sample.shape != (expected,) or not np.all(np.isfinite(sample)):
                raise RuntimeError(f"Expected {expected} valid leader channels")
            target = sample[:6]
            if not args.dry_run:
                client.call("command_joint_state", joint_state=target.tolist())
                if gripper is not None:
                    gripper.submit(float(sample[6]))
            actual = np.asarray(client.call("get_joint_state"), dtype=float)[:6]
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
    finally:
        try:
            if gripper is not None:
                gripper.close()
                gripper.check()
        finally:
            if agent is not None:
                agent.close()
            if client is not None:
                client.close()
            signal.signal(signal.SIGTERM, old_term)


def _interrupt(*_args: Any) -> None:
    raise KeyboardInterrupt
