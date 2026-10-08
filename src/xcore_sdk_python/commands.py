"""Dispatch one robot operation inside its own SDK session."""

from __future__ import annotations

import math
import time
from typing import Any

from .driver import RobotDriver
from .reader import RobotConnection
from .sdk import doctor


def execute(options: dict[str, Any]) -> dict[str, Any]:
    command = options["command"]
    if command == "doctor":
        return doctor(options.get("sdk_dir"))
    if command == "follow-prepare":
        from .follow_prepare import prepare

        return prepare(options)
    if command == "return-zero":
        from .return_zero import return_zero

        return return_zero(options)
    if command == "gripper-check":
        from .gripper_follow import GripperFollowClient

        return GripperFollowClient(
            options["gripper_host"],
            options["gripper_port"],
            options["gripper_timeout"],
        ).check()
    driver_commands = {"power", "mode", "stop", "movej", "move-joint"}
    client = RobotDriver if command in driver_commands else RobotConnection
    with client(
        options["ip"], local_ip=options["local_ip"], sdk_dir=options.get("sdk_dir")
    ) as arm:
        if command in ("status", "info", "joints", "limits"):
            return getattr(arm, command)()
        if command == "pose":
            return arm.pose(options["frame"])
        if command == "dh":
            return arm.dh(options["nominal"])
        if command == "monitor":
            started = time.monotonic()
            samples = []
            while True:
                samples.append(arm.status())
                remaining = options["duration"] - (time.monotonic() - started)
                if remaining <= 0:
                    break
                time.sleep(min(options["interval"], remaining))
            return {"samples": samples, "count": len(samples)}
        if command == "power":
            return arm.power(options["value"] == "on")
        if command == "mode":
            return arm.mode(options["value"])
        if command == "stop":
            return arm.stop()
        if command in ("movej", "move-joint"):
            if command == "movej":
                target = options["joints"]
                if options["unit"] == "deg":
                    target = [math.radians(v) for v in target]
            else:
                target = arm.joints()["rad"]
                target[options["joint"] - 1] += math.radians(options["delta_deg"])
            return arm.movej(
                target,
                speed=options["speed"],
                max_step_deg=options["max_step_deg"],
                tolerance_deg=options["tolerance_deg"],
                motion_timeout=options["motion_timeout"],
            )
    raise ValueError(f"Unknown command: {command}")
