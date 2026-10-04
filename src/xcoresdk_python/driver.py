"""Explicit control operations; validate feedback before issuing motion."""

from __future__ import annotations

import math
import time
from typing import Any

from .exceptions import XCoreError
from .reader import RobotConnection, vector


class RobotDriver(RobotConnection):
    def power(self, on: bool) -> dict[str, Any]:
        self.call("setPowerState", on)
        return {
            "requested": "on" if on else "off",
            "power": self.call("powerState").name,
        }

    def mode(self, name: str) -> dict[str, Any]:
        if name not in ("manual", "automatic"):
            raise ValueError("Mode must be manual or automatic")
        self.call("setOperateMode", getattr(self.sdk.OperateMode, name))
        return {"requested": name, "mode": self.call("operateMode").name}

    def stop(self) -> dict[str, Any]:
        self.call("stop")
        return {"stop_requested": True, "operation": self.call("operationState").name}

    def movej(
        self,
        target_rad: list[float],
        *,
        speed: float = 50,
        max_step_deg: float = 10,
        tolerance_deg: float = 0.2,
        motion_timeout: float = 15,
    ) -> dict[str, Any]:
        target = vector(target_rad, 6, "joint target")
        if not math.isfinite(speed) or not 5 <= speed <= 4000:
            raise ValueError("Speed must be in [5, 4000] mm/s")
        for value in (max_step_deg, tolerance_deg, motion_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    "Step limit, tolerance, and motion timeout must be positive"
                )
        if self.call("operationState") != self.sdk.OperationState.idle:
            raise XCoreError("Motion requires an idle robot")
        if self.call("powerState") != self.sdk.PowerState.on:
            raise XCoreError("Robot is not powered on; use power on explicitly")
        if self.call("operateMode") != self.sdk.OperateMode.automatic:
            raise XCoreError(
                "Motion requires automatic mode; use mode automatic explicitly"
            )
        current = self.joints()["rad"]
        limits = self.limits()
        if not limits["enabled"]:
            raise XCoreError(
                "Soft limits are disabled; configure them in RobotAssist first"
            )
        for index, (goal, now, (low, high)) in enumerate(
            zip(target, current, limits["rad"], strict=True), 1
        ):
            if not low <= goal <= high:
                raise XCoreError(f"Joint {index} target exceeds controller soft limits")
            if abs(goal - now) > math.radians(max_step_deg):
                raise XCoreError(f"Joint {index} step exceeds {max_step_deg} degrees")
        self.call("setMotionControlMode", self.sdk.MotionControlMode.NrtCommandMode)
        self.call("moveReset")
        command = self.sdk.MoveAbsJCommand(target, speed, 0)
        identifier = self.sdk.PyString()
        self.call("moveAppend", [command], identifier)
        try:
            self.call("moveStart")
            deadline = time.monotonic() + motion_timeout
            while time.monotonic() < deadline:
                state = self.call("operationState")
                actual = self.joints()["rad"]
                error = max(abs(a - b) for a, b in zip(actual, target, strict=True))
                if state == self.sdk.OperationState.idle and error <= math.radians(
                    tolerance_deg
                ):
                    return {
                        "reached": True,
                        "command_id": identifier.content(),
                        "target_rad": target,
                        "actual_rad": actual,
                        "max_error_deg": math.degrees(error),
                    }
                if state not in (
                    self.sdk.OperationState.idle,
                    self.sdk.OperationState.moving,
                ):
                    raise XCoreError(f"Unexpected motion state: {state.name}")
                time.sleep(0.05)
            raise XCoreError("Motion timed out before the target was reached")
        except BaseException:
            # Best effort only: hardware emergency stop remains independent.
            try:
                self.call("stop")
            except XCoreError:
                pass
            raise
