"""Read-only leader calibration and six-axis CR7 teleoperation client."""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any

import numpy as np

from .calibration import load_calibration
from .gello_server import ZmqRobotClient


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

    agent = Cr7LeaderAgent(
        args.serial, baudrate=args.baudrate, joint_offsets=offsets, joint_signs=signs
    )
    client = None
    try:
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
            print(
                "Dry-run only: showing joint values; no target commands will be sent."
            )
        else:
            print(
                "Enable server motion explicitly; keep the physical E-stop reachable."
            )
            if not args.yes and input(
                "Type y to start following: "
            ).strip().lower() not in ("y", "yes"):
                print("Cancelled")
                return 1

        period = 1.0 / args.hz
        while True:
            started = time.perf_counter()
            target = np.asarray(agent.get_joint_state(), dtype=float)[:6]
            if not args.dry_run:
                client.call("command_joint_state", joint_state=target.tolist())
            actual = np.asarray(client.call("get_joint_state"), dtype=float)[:6]
            print(
                "\rleader="
                + str(np.round(np.degrees(target), 1).tolist())
                + " robot="
                + str(np.round(np.degrees(actual), 1).tolist()),
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
        agent.close()
        if client is not None:
            client.close()
