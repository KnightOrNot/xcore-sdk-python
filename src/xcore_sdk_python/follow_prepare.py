"""Prepare a stationary GELLO target before starting the RT follow session."""

from __future__ import annotations

import json
import math
import os
import tempfile
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from .calibration import load_calibration, reference_offsets
from .driver import RobotDriver
from .exceptions import XCoreError

HOLD_TOLERANCE = math.radians(0.5)


def stable_raw(leader: Any, samples: int = 20) -> np.ndarray:
    readings = []
    for _ in range(samples):
        raw = leader.read_raw_rad()
        if raw is None:
            raise XCoreError("Missing GELLO feedback during preparation")
        readings.append(raw)
        time.sleep(0.02)
    values = np.asarray(readings, dtype=float)
    if values.shape != (samples, 6) or not np.all(np.isfinite(values)):
        raise XCoreError("Preparation requires six finite GELLO channels")
    if np.max(np.ptp(values, axis=0)) > HOLD_TOLERANCE:
        raise XCoreError("Keep GELLO still during zeroing and alignment")
    return values


def check_held(initial: np.ndarray, current: np.ndarray) -> None:
    if np.max(np.abs(current.mean(axis=0) - initial.mean(axis=0))) > HOLD_TOLERANCE:
        raise XCoreError("GELLO moved during preparation; hold it still and restart")


def leader_target(
    raw: np.ndarray, offsets: Any, signs: Any, reference: Any
) -> list[float]:
    mapped = (raw.mean(axis=0) - np.asarray(offsets)) * np.asarray(signs)
    # Use the same nearest branch as the follow client against actual feedback.
    branch = np.round((mapped - np.asarray(reference)) / (2 * math.pi))
    return (mapped - branch * 2 * math.pi).tolist()


def validate_leg(current: Any, target: Any, limits: dict, max_step_deg: float) -> None:
    q, goal = np.asarray(current), np.asarray(target)
    bounds = np.asarray(limits["rad"])
    if not limits["enabled"]:
        raise XCoreError("Preparation requires enabled controller soft limits")
    if (
        q.shape != (6,)
        or goal.shape != (6,)
        or bounds.shape != (6, 2)
        or not np.all(np.isfinite(q))
        or not np.all(np.isfinite(goal))
        or not np.all(np.isfinite(bounds))
    ):
        raise XCoreError("Invalid preparation joints or soft limits")
    if np.any(q < bounds[:, 0]) or np.any(q > bounds[:, 1]):
        raise XCoreError("Current robot posture exceeds controller soft limits")
    if np.any(goal < bounds[:, 0]) or np.any(goal > bounds[:, 1]):
        raise XCoreError("Preparation target exceeds controller soft limits")
    if np.max(np.abs(goal - q)) > math.radians(max_step_deg):
        raise XCoreError(f"Preparation step exceeds {max_step_deg} degrees")


def save_calibration(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    # Publish a complete file without overwriting any existing calibration.
    with tempfile.NamedTemporaryFile(
        mode="w",
        dir=path.parent,
        prefix=path.name + ".",
        suffix=".partial",
        delete=False,
    ) as stream:
        temporary = Path(stream.name)
        try:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        except BaseException:
            temporary.unlink(missing_ok=True)
            raise
    try:
        os.link(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def restore(arm: Any, power: Any, mode: Any) -> None:
    arm.stop()
    deadline = time.monotonic() + 5
    while arm.call("operationState") != arm.sdk.OperationState.idle:
        if time.monotonic() >= deadline:
            raise XCoreError("Robot did not stop; verify the teach pendant state")
        time.sleep(0.05)
    arm.power(power == arm.sdk.PowerState.on)
    arm.mode(mode.name)


def prepare(args: dict[str, Any]) -> dict[str, Any]:
    from .gello_leader import Cr7LeaderAgent
    from .gripper_follow import GripperFollowClient

    path = Path(args["calib"])
    calibrating = args["calibrate_zero"]
    if calibrating:
        if path.exists():
            raise XCoreError("Calibration already exists; choose another --calib path")
        offsets, signs = [0.0] * 6, args["signs"]
    else:
        offsets, signs = load_calibration(path)
    if args.get("gripper_host"):
        GripperFollowClient(
            args["gripper_host"], args["gripper_port"], args["gripper_timeout"]
        ).check()
    leader = Cr7LeaderAgent(
        args["serial"],
        baudrate=args["baudrate"],
        joint_offsets=offsets,
        joint_signs=signs,
        verbose=False,
    )
    try:
        initial = stable_raw(leader)
        zero = [0.0] * 6
        with RobotDriver(
            args["ip"], local_ip=args["local_ip"], sdk_dir=args.get("sdk_dir")
        ) as arm:
            if arm.call("operationState") != arm.sdk.OperationState.idle:
                raise XCoreError("Preparation requires an idle robot")
            power, mode = arm.call("powerState"), arm.call("operateMode")
            limits = arm.limits()
            current = arm.joints()["rad"]
            goal = (
                zero if calibrating else leader_target(initial, offsets, signs, current)
            )
            validate_leg(current, goal, limits, args["max_step_deg"])
            result = {"initial_rad": current, "target_rad": goal}
            attempted = False
            try:

                def move(target: list[float]) -> dict:
                    nonlocal attempted
                    actual = arm.joints()["rad"]
                    if max(
                        abs(a - b) for a, b in zip(actual, target, strict=True)
                    ) <= math.radians(args["tolerance_deg"]):
                        return {"reached": True, "skipped": True, "actual_rad": actual}
                    attempted = True
                    arm.mode("automatic")
                    arm.power(True)
                    return arm.movej(
                        target,
                        speed=args["speed"],
                        max_step_deg=args["max_step_deg"],
                        tolerance_deg=args["tolerance_deg"],
                        motion_timeout=args["motion_timeout"],
                    )

                if calibrating:
                    result["zero_motion"] = move(zero)
                    zero_samples = stable_raw(leader)
                    check_held(initial, zero_samples)
                    actual_zero = arm.joints()["rad"]
                    offsets = reference_offsets(
                        zero_samples, actual_zero, signs
                    ).tolist()
                    save_calibration(
                        path,
                        {
                            "joint_offsets": offsets,
                            "joint_signs": signs,
                            "created": datetime.now().astimezone().isoformat(),
                            "ok": True,
                            "n_samples": len(zero_samples),
                            "robot_ip": args["ip"],
                            "method": "GELLO zero matched to measured CR7 zero",
                        },
                    )
                    goal = leader_target(zero_samples, offsets, signs, actual_zero)
                    validate_leg(actual_zero, goal, limits, args["max_step_deg"])
                result["target_rad"] = goal
                result["alignment_motion"] = move(goal)
                check_held(initial, stable_raw(leader))
                result["actual_rad"] = arm.joints()["rad"]
                result["calibration"] = str(path.resolve())
                result["calibrated_zero"] = calibrating
                return result
            finally:
                if attempted:
                    restore(arm, power, mode)
    finally:
        leader.close()
