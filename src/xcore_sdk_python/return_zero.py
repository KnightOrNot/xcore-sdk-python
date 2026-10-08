"""Return six CR7 joints to zero after the follow session has closed."""

from __future__ import annotations

import math
import sys
import time
from typing import Any

from .driver import RobotDriver
from .exceptions import XCoreError
from .follow_prepare import restore, validate_leg


def recover_rt(arm: Any) -> bool:
    """End a leftover RT session without starting a new RT control loop."""
    state = arm.call("operationState")
    power = arm.call("powerState")
    if power.name not in ("on", "off"):
        raise XCoreError(
            f"Recovery refused: power state {power.name}; check the pendant"
        )
    if state.name == "idle":
        return False
    if state.name != "rtControlling":
        raise XCoreError(f"Recovery refused: operation state {state.name}")
    print(
        "[故障恢复] 停止遗留实时会话，调用 automaticErrorRecovery 一次。",
        file=sys.stderr,
    )
    try:
        # A new SDK session defaults to NRT even when the controller still
        # reports rtControlling. stopMove requires the matching SDK mode.
        arm.call("setMotionControlMode", arm.sdk.MotionControlMode.RtCommandMode)
        rt = arm.robot.getRtMotionController()
        rt.stopMove()
        ec: dict[str, Any] = {}
        rt.automaticErrorRecovery(ec)
        if ec.get("ec") != 0:
            raise XCoreError(f"automaticErrorRecovery: {ec}")
        arm.call("setMotionControlMode", arm.sdk.MotionControlMode.NrtCommandMode)
        arm.stop()
        deadline = time.monotonic() + 5
        while arm.call("operationState") != arm.sdk.OperationState.idle:
            if time.monotonic() >= deadline:
                raise XCoreError("Recovery did not reach idle; no zero target sent")
            time.sleep(0.05)
    except BaseException:
        try:
            arm.stop()
        except Exception as exc:
            print(f"[恢复停止错误] {exc}", file=sys.stderr)
        raise
    return True


def return_zero(options: dict[str, Any]) -> dict[str, Any]:
    with RobotDriver(
        options["ip"],
        local_ip=options["local_ip"],
        sdk_dir=options.get("sdk_dir"),
    ) as arm:
        recovering = options.get("recover", False)
        if not recovering and arm.call("operationState") != arm.sdk.OperationState.idle:
            raise XCoreError("Return to zero requires an idle robot")
        current = arm.joints()["rad"]
        target = [0.0] * 6
        validate_leg(current, target, arm.limits(), options["max_step_deg"])
        recovered = recover_rt(arm) if recovering else False
        if recovered:
            current = arm.joints()["rad"]
            validate_leg(current, target, arm.limits(), options["max_step_deg"])
        if max(abs(q) for q in current) <= math.radians(options["tolerance_deg"]):
            if recovering:
                arm.power(False)
                arm.mode("manual")
            return {"reached": True, "skipped": True, "actual_rad": current}
        power = arm.sdk.PowerState.off if recovering else arm.call("powerState")
        mode = arm.sdk.OperateMode.manual if recovering else arm.call("operateMode")
        failure = None
        try:
            arm.mode("automatic")
            arm.power(True)
            result = arm.movej(
                target,
                speed=options["speed"],
                max_step_deg=options["max_step_deg"],
                tolerance_deg=options["tolerance_deg"],
                motion_timeout=options["motion_timeout"],
            )
            return dict(result, rt_recovered=recovered)
        except BaseException as exc:
            failure = exc
            raise
        finally:
            try:
                restore(arm, power, mode)
            except Exception as exc:
                if failure is None:
                    raise
                print(f"[回零清理错误] {exc}", file=sys.stderr)
