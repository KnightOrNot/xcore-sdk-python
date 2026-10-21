"""Validated CLI defaults loaded from ``config/default.toml``.

Priority is explicit command-line arguments, then environment variables used by
the existing entry points, then this file, then the dataclass defaults. Paths
are kept as strings and resolved by the commands that use them.
"""

from __future__ import annotations

import math
import tomllib
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any


def _default_config_path() -> Path | None:
    package = Path(__file__).resolve().parent
    # Editable checkout: src/xcore_sdk_python -> repository root.
    checkout = package.parents[1] / "config/default.toml"
    if checkout.exists():
        return checkout
    # Installed wheel: the build materializes src/xcore_sdk_python/default.toml.
    bundled = package / "default.toml"
    if bundled.exists():
        return bundled
    return None


DEFAULT_CONFIG = _default_config_path()


@dataclass(frozen=True)
class RobotConfig:
    ip: str = "192.168.2.160"
    local_ip: str = ""
    timeout_s: float = 20.0
    sdk_dir: str = ""


@dataclass(frozen=True)
class MonitorConfig:
    duration: float = 5.0
    interval: float = 0.5


@dataclass(frozen=True)
class MotionConfig:
    speed: float = 1000.0
    max_step_deg: float = 10.0
    tolerance_deg: float = 0.2
    motion_timeout_s: float = 15.0


@dataclass(frozen=True)
class ServerConfig:
    host: str = "127.0.0.1"
    port: int = 6001
    state_hz: float = 25.0
    max_speed_deg: float = 3.0
    accel_deg_s2: float = 40.0
    stale_timeout_s: float = 1.0
    gate_deg: float = 17.1887


@dataclass(frozen=True)
class GelloConfig:
    port: str = (
        "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0"
    )
    baudrate: int = 57600
    signs: tuple[int, ...] = (1, 1, 1, 1, 1, 1)


@dataclass(frozen=True)
class CalibrationConfig:
    file: str = "config/cr7_calib.json"
    samples: int = 30
    interval_s: float = 0.03


@dataclass(frozen=True)
class PrepareConfig:
    speed: float = 1000.0
    max_step_deg: float = 180.0
    tolerance_deg: float = 0.2
    motion_timeout_s: float = 600.0
    gripper_port: int = 5005
    gripper_timeout_s: float = 0.75


@dataclass(frozen=True)
class ReturnConfig:
    pose_file: str = "config/exit_pose.json"
    speed: float = 1000.0
    max_step_deg: float = 360.0
    tolerance_deg: float = 0.2
    motion_timeout_s: float = 600.0


@dataclass(frozen=True)
class GripperConfig:
    port: int = 5005
    id: int = 7
    open_deg: float = 194.8
    close_deg: float = 153.0
    open_pos: int = 0
    closed_pos: int = 255
    hz: float = 5.0
    speed: int = 150
    force: int = 0
    timeout_s: float = 0.75
    stale_timeout_s: float = 1.5


@dataclass(frozen=True)
class FollowConfig:
    host: str = "127.0.0.1"
    port: int = 6001
    hz: float = 50.0
    gate_deg: float = 17.1887


@dataclass(frozen=True)
class RecordingConfig:
    task: str = "CR7 GELLO teleoperation"
    queue_size: int = 500
    feedback_max_age_s: float = 0.75


@dataclass(frozen=True)
class NetworkConfig:
    interface: str = ""
    address: str = "192.168.2.100/24"


SECTIONS = {
    "robot": RobotConfig,
    "monitor": MonitorConfig,
    "motion": MotionConfig,
    "server": ServerConfig,
    "gello": GelloConfig,
    "calibration": CalibrationConfig,
    "prepare": PrepareConfig,
    "return": ReturnConfig,
    "gripper": GripperConfig,
    "follow": FollowConfig,
    "recording": RecordingConfig,
    "network": NetworkConfig,
}


@dataclass(frozen=True)
class Config:
    path: Path | None
    robot: RobotConfig = RobotConfig()
    monitor: MonitorConfig = MonitorConfig()
    motion: MotionConfig = MotionConfig()
    server: ServerConfig = ServerConfig()
    gello: GelloConfig = GelloConfig()
    calibration: CalibrationConfig = CalibrationConfig()
    prepare: PrepareConfig = PrepareConfig()
    return_: ReturnConfig = ReturnConfig()
    gripper: GripperConfig = GripperConfig()
    follow: FollowConfig = FollowConfig()
    recording: RecordingConfig = RecordingConfig()
    network: NetworkConfig = NetworkConfig()

    def validate(self) -> None:
        sections = {
            name: getattr(self, "return_" if name == "return" else name)
            for name in SECTIONS
        }
        for section, values in sections.items():
            for item in fields(values):
                value = getattr(values, item.name)
                default = item.default
                kind = type(default)
                valid = type(value) is kind
                if kind is float:
                    valid = (
                        not isinstance(value, bool)
                        and isinstance(value, (int, float))
                        and math.isfinite(value)
                    )
                if not valid:
                    raise ValueError(
                        f"{section}.{item.name} has an invalid type or non-finite value"
                    )
        if not isinstance(self.robot.ip, str) or not self.robot.ip:
            raise ValueError("robot.ip must be a non-empty string")
        if not isinstance(self.robot.local_ip, str):
            raise ValueError("robot.local_ip must be a string")
        for name in ("timeout_s",):
            _positive(self.robot, name, "robot")
        for name in ("duration", "interval"):
            _positive(self.monitor, name, "monitor")
        for name in ("speed", "max_step_deg", "tolerance_deg", "motion_timeout_s"):
            _positive(self.motion, name, "motion")
        if not 1 <= self.server.port <= 65535:
            raise ValueError("server.port must be in 1..65535")
        for name in (
            "state_hz",
            "max_speed_deg",
            "accel_deg_s2",
            "stale_timeout_s",
            "gate_deg",
        ):
            _positive(self.server, name, "server")
        if self.server.max_speed_deg > 75:
            raise ValueError("server.max_speed_deg is limited to 75 deg/s")
        if not isinstance(self.gello.port, str) or not self.gello.port:
            raise ValueError("gello.port must be a non-empty string")
        if not 1 <= self.gello.baudrate <= 4_000_000:
            raise ValueError("gello.baudrate must be a positive integer")
        if len(self.gello.signs) != 6 or any(
            type(value) is not int or value not in (-1, 1) for value in self.gello.signs
        ):
            raise ValueError("gello.signs must contain six values of -1 or 1")
        if self.calibration.samples < 2:
            raise ValueError("calibration.samples must be at least 2")
        _positive(self.calibration, "interval_s", "calibration")
        for name in ("speed", "max_step_deg", "tolerance_deg", "motion_timeout_s"):
            _positive(self.prepare, name, "prepare")
        for name in ("speed", "max_step_deg", "tolerance_deg", "motion_timeout_s"):
            _positive(self.return_, name, "return")
        if not isinstance(self.return_.pose_file, str) or not self.return_.pose_file:
            raise ValueError("return.pose_file must be a non-empty string")
        if not 1 <= self.gripper.port <= 65535:
            raise ValueError("gripper.port must be in 1..65535")
        if not 7 <= self.gripper.id <= 252:
            raise ValueError("gripper.id must be in 7..252")
        if self.gripper.open_deg == self.gripper.close_deg:
            raise ValueError("gripper open/close angles must differ")
        for name in ("open_pos", "closed_pos", "speed", "force"):
            value = getattr(self.gripper, name)
            if not isinstance(value, int) or not 0 <= value <= 255:
                raise ValueError(f"gripper.{name} must be an integer in 0..255")
        if self.gripper.open_pos == self.gripper.closed_pos:
            raise ValueError("gripper open/closed positions must differ")
        if not 0 < self.gripper.hz <= 10:
            raise ValueError("gripper.hz must be in (0, 10]")
        _positive(self.gripper, "timeout_s", "gripper")
        _positive(self.gripper, "stale_timeout_s", "gripper")
        if self.gripper.stale_timeout_s <= 1 / self.gripper.hz + self.gripper.timeout_s:
            raise ValueError(
                "gripper.stale_timeout_s must exceed period + request timeout"
            )
        if not 1 <= self.follow.port <= 65535:
            raise ValueError("follow.port must be in 1..65535")
        for name in ("hz", "gate_deg"):
            _positive(self.follow, name, "follow")
        if not self.recording.task.strip():
            raise ValueError("recording.task must be non-empty")
        if self.recording.queue_size <= 0:
            raise ValueError("recording.queue_size must be positive")
        _positive(self.recording, "feedback_max_age_s", "recording")
        if not isinstance(self.network.interface, str):
            raise ValueError("network.interface must be a string")
        if not isinstance(self.network.address, str):
            raise ValueError("network.address must be a string")


def _positive(section: Any, name: str, section_name: str) -> None:
    value = getattr(section, name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{section_name}.{name} must be a number")
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{section_name}.{name} must be positive and finite")


def load_config(path: str | Path | None = None) -> Config:
    """Load ``path`` over built-in defaults; ``None`` selects the repository file."""

    selected = Path(path).resolve() if path is not None else DEFAULT_CONFIG
    if path is None and selected is None:
        data: dict[str, Any] = {}
    else:
        with Path(selected).open("rb") as stream:
            data = tomllib.load(stream)
    unknown = data.keys() - SECTIONS.keys()
    if unknown:
        raise ValueError(f"unknown configuration sections: {sorted(unknown)}")
    sections: dict[str, Any] = {}
    for name, cls in SECTIONS.items():
        values = data.get(name, {})
        if not isinstance(values, dict):
            raise ValueError(f"{name} must be a TOML table")
        unknown = values.keys() - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError(f"unknown {name} options: {sorted(unknown)}")
        converted = {
            key: tuple(value) if isinstance(value, list) else value
            for key, value in values.items()
        }
        try:
            sections[name] = cls(**converted)
        except TypeError as exc:
            raise ValueError(f"invalid {name} options: {exc}") from exc
    result = Config(
        path=Path(selected) if selected is not None else None,
        return_=sections.pop("return"),
        **sections,
    )
    result.validate()
    return result
