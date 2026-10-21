"""Return to a saved CR7 exit posture after the follow session has closed."""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import Any

from .driver import RobotDriver
from .exceptions import XCoreError
from .follow_prepare import restore, validate_leg

DEFAULT_EXIT_POSE = Path(__file__).resolve().parents[2] / "config/exit_pose.json"


def load_exit_pose(path: str | Path, robot_ip: str) -> list[float]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise XCoreError("Exit pose requires schema_version=1")
    if data.get("robot_ip") != robot_ip:
        raise XCoreError("Exit pose robot_ip does not match the selected robot")
    values = data.get("joints_rad")
    if not isinstance(values, list) or len(values) != 6:
        raise XCoreError("Exit pose requires six joints_rad values")
    if any(isinstance(q, bool) or not isinstance(q, (int, float)) for q in values):
        raise XCoreError("Exit pose joint values must be numbers")
    target = [float(q) for q in values]
    if not all(math.isfinite(q) for q in target):
        raise XCoreError("Exit pose joint values must be finite")
    return target


def return_target(options: dict[str, Any]) -> tuple[list[float], Path | None]:
    if options.get("zero"):
        return [0.0] * 6, None
    path = Path(options.get("pose_file") or DEFAULT_EXIT_POSE)
    # Old launchers already running at deployment time call return-zero.
    # Honor the configured exit pose for that compatibility entry as well.
    if (
        options.get("command") == "return-zero"
        and path == DEFAULT_EXIT_POSE
        and not path.exists()
    ):
        return [0.0] * 6, None
    return load_exit_pose(path, options["ip"]), path


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
                raise XCoreError("Recovery did not reach idle; no return target sent")
            time.sleep(0.05)
    except BaseException:
        try:
            arm.stop()
        except Exception as exc:
            print(f"[恢复停止错误] {exc}", file=sys.stderr)
        raise
    return True


def return_pose(options: dict[str, Any]) -> dict[str, Any]:
    target, pose_file = return_target(options)
    if pose_file is not None:
        print(
            "[退出返回] 固定目标（度）："
            + str([round(math.degrees(q), 4) for q in target])
            + f"；文件：{pose_file}",
            file=sys.stderr,
            flush=True,
        )
    with RobotDriver(
        options["ip"],
        local_ip=options["local_ip"],
        sdk_dir=options.get("sdk_dir"),
    ) as arm:
        recovering = options.get("recover", False)
        if not recovering and arm.call("operationState") != arm.sdk.OperationState.idle:
            raise XCoreError("Return motion requires an idle robot")
        current = arm.joints()["rad"]
        validate_leg(current, target, arm.limits(), options["max_step_deg"])
        recovered = recover_rt(arm) if recovering else False
        if recovered:
            current = arm.joints()["rad"]
            validate_leg(current, target, arm.limits(), options["max_step_deg"])
        if max(
            abs(q - goal) for q, goal in zip(current, target, strict=True)
        ) <= math.radians(options["tolerance_deg"]):
            if recovering:
                arm.power(False)
                arm.mode("manual")
            return {
                "reached": True,
                "skipped": True,
                "actual_rad": current,
                "target_rad": target,
                "pose_file": str(pose_file) if pose_file else None,
            }
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
            return dict(
                result,
                target_rad=target,
                rt_recovered=recovered,
                pose_file=str(pose_file) if pose_file else None,
            )
        except BaseException as exc:
            failure = exc
            raise
        finally:
            try:
                restore(arm, power, mode)
            except Exception as exc:
                if failure is None:
                    raise
                print(f"[返回清理错误] {exc}", file=sys.stderr)


def return_zero(options: dict[str, Any]) -> dict[str, Any]:
    """Compatibility entry for old launchers; --zero requests literal zero."""
    return return_pose(options)
