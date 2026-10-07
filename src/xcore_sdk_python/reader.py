"""Connection lifecycle and queries; no power or motion preparation."""

from __future__ import annotations

import math
from datetime import datetime, timezone
from typing import Any

from .exceptions import XCoreError
from .sdk import load_sdk


def vector(values: Any, size: int, label: str) -> list[float]:
    result = list(values)
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise XCoreError(
            f"Invalid {label}: expected {size} finite values, got {result}"
        )
    return result


class RobotConnection:
    """Own one collaborative six-axis SDK session.

    SDK connection initialization can reset motion bookkeeping. Disconnect can
    stop existing robot motion. Do not overlap sessions with another controller.
    """

    def __init__(
        self,
        ip: str,
        *,
        local_ip: str = "",
        sdk_dir: str | None = None,
        sdk: Any = None,
        robot: Any = None,
    ):
        self.ip = ip
        self.local_ip = local_ip
        self.sdk = sdk if sdk is not None else load_sdk(sdk_dir)
        self.robot = robot if robot is not None else self.sdk.xMateRobot()
        self.connected = False

    def call(self, name: str, *args: Any) -> Any:
        ec: dict[str, Any] = {}
        try:
            result = getattr(self.robot, name)(*args, ec)
        except Exception as exc:
            raise XCoreError(f"{name}: {exc}") from exc
        if ec.get("ec") != 0:
            raise XCoreError(f"{name}: {ec}")
        return result

    def __enter__(self) -> RobotConnection:
        try:
            self.robot.connectToRobot(self.ip, self.local_ip)
        except Exception as exc:
            raise XCoreError(f"connectToRobot({self.ip}): {exc}") from exc
        self.connected = True
        try:
            info = self.info()
            if info["joint_num"] != 6:
                raise XCoreError(f"Expected six axes, got {info['joint_num']}")
        except BaseException:
            self.close()
            raise
        return self

    def close(self) -> None:
        if self.connected:
            try:
                self.call("disconnectFromRobot")
            finally:
                self.connected = False

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        try:
            self.close()
        except XCoreError:
            if exc is None:
                raise

    def info(self) -> dict[str, Any]:
        info = self.call("robotInfo")
        return {
            key: getattr(info, key)
            for key in ("id", "version", "type", "joint_num", "mac")
        }

    def joints(self) -> dict[str, Any]:
        raw = list(self.call("jointPos"))
        angles = vector(raw[:6], 6, "joint positions")
        extra = vector(raw[6:], len(raw[6:]), "extra SDK joint values")
        return {
            "rad": angles,
            "deg": [math.degrees(v) for v in angles],
            "raw": raw,
            "extra_sdk_values": extra,
        }

    def pose(self, frame: str = "flange") -> dict[str, Any]:
        coordinate = (
            self.sdk.CoordinateType.flangeInBase
            if frame == "flange"
            else self.sdk.CoordinateType.endInRef
        )
        return {
            "frame": "flangeInBase" if frame == "flange" else "endInRef",
            "xyz_m_rpy_rad": vector(self.call("posture", coordinate), 6, "pose"),
        }

    def limits(self) -> dict[str, Any]:
        data = self.sdk.PyTypeVectorArrayDouble2()
        enabled = self.call("getSoftLimit", data)
        bounds = list(data.content())
        if len(bounds) != 6:
            raise XCoreError(f"Expected six joint limits, got {bounds}")
        bounds = [vector(pair, 2, "joint limits") for pair in bounds]
        if any(low > high for low, high in bounds):
            raise XCoreError("Invalid soft-limit interval")
        return {
            "enabled": bool(enabled),
            "rad": bounds,
            "deg": [[math.degrees(v) for v in pair] for pair in bounds],
        }

    def dh(self, nominal: bool = False) -> dict[str, Any]:
        raw = list(self.call("getRobotCfg_DHparam", nominal))
        values = vector(raw[:24], 24, "DH parameters")
        extra = vector(raw[24:], len(raw[24:]), "extra SDK DH values")
        return {
            "nominal": nominal,
            "raw": raw,
            "rows": [values[start : start + 4] for start in range(0, 24, 4)],
            "extra_sdk_values": extra,
            "sdk_units": "per axis: Alpha[deg], A[mm], D[mm], Theta[deg]",
            "coordinate_convention": "Verify with vendor before URDF conversion",
        }

    def status(self) -> dict[str, Any]:
        return {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "connected": self.connected,
            "sdk_version": self.sdk.BaseRobot.sdkVersion(),
            "robot": self.info(),
            "power": self.call("powerState").name,
            "mode": self.call("operateMode").name,
            "operation": self.call("operationState").name,
            "joints": self.joints(),
            "pose": self.pose(),
        }
