"""Return six CR7 joints to zero after the follow session has closed."""

from __future__ import annotations

import math
from typing import Any

from .driver import RobotDriver
from .exceptions import XCoreError
from .follow_prepare import restore, validate_leg


def return_zero(options: dict[str, Any]) -> dict[str, Any]:
    with RobotDriver(
        options["ip"],
        local_ip=options["local_ip"],
        sdk_dir=options.get("sdk_dir"),
    ) as arm:
        if arm.call("operationState") != arm.sdk.OperationState.idle:
            raise XCoreError("Return to zero requires an idle robot")
        current = arm.joints()["rad"]
        target = [0.0] * 6
        validate_leg(current, target, arm.limits(), options["max_step_deg"])
        if max(abs(q) for q in current) <= math.radians(options["tolerance_deg"]):
            return {"reached": True, "skipped": True, "actual_rad": current}
        power = arm.call("powerState")
        mode = arm.call("operateMode")
        try:
            arm.mode("automatic")
            arm.power(True)
            return arm.movej(
                target,
                speed=options["speed"],
                max_step_deg=options["max_step_deg"],
                tolerance_deg=options["tolerance_deg"],
                motion_timeout=options["motion_timeout"],
            )
        finally:
            restore(arm, power, mode)
