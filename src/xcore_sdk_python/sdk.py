"""Load the vendor extension lazily; help and network tools need no SDK."""

from __future__ import annotations

import importlib.util
import os
import platform
import sys
from importlib.machinery import EXTENSION_SUFFIXES
from pathlib import Path
from typing import Any

from .exceptions import XCoreError

_DLL_HANDLES: list[Any] = []


def sdk_directory(explicit: str | None = None) -> Path:
    override = explicit or os.environ.get("XCORE_SDK_DIR")
    if override:
        return Path(override).expanduser().resolve()
    release = Path(__file__).resolve().parents[2] / "Release"
    machine = platform.machine().lower()
    if platform.system() == "Linux":
        if machine in ("x86_64", "amd64"):
            return release / "linux"
        if machine in ("aarch64", "arm64"):
            return release / "linux" / "arm"
    if platform.system() == "Windows" and machine in ("x86_64", "amd64"):
        return release / "windows"
    raise XCoreError(f"Unsupported SDK platform: {platform.system()} / {machine}")


def load_sdk(directory: str | None = None) -> Any:
    location = sdk_directory(directory)
    binary = next(
        (
            location / f"xCoreSDK_python{suffix}"
            for suffix in EXTENSION_SUFFIXES
            if (location / f"xCoreSDK_python{suffix}").is_file()
        ),
        None,
    )
    if binary is None:
        found = sorted(p.name for p in location.glob("xCoreSDK_python*"))
        raise XCoreError(
            f"No extension matching Python {platform.python_version()} / "
            f"{platform.machine()} in {location}. Found: {found}. "
            "Install the matching official Release package or use --sdk-dir."
        )
    existing = sys.modules.get("xCoreSDK_python")
    if existing is not None:
        if Path(existing.__file__).resolve() != binary.resolve():
            raise XCoreError("Another xCoreSDK_python binary is already loaded")
        return existing
    if platform.system() == "Windows":
        _DLL_HANDLES.append(os.add_dll_directory(str(location)))
    spec = importlib.util.spec_from_file_location("xCoreSDK_python", binary)
    if spec is None or spec.loader is None:
        raise XCoreError(f"Cannot load extension: {binary}")
    try:
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        sys.modules["xCoreSDK_python"] = module
        return module
    except (ImportError, OSError) as exc:
        raise XCoreError(f"SDK import failed: {exc}") from exc


def doctor(directory: str | None = None) -> dict[str, Any]:
    sdk = load_sdk(directory)
    methods = (
        "robotInfo",
        "jointPos",
        "getRobotCfg_DHparam",
        "moveAppend",
        "stop",
        "getRtMotionController",
    )
    missing = [name for name in methods if not hasattr(sdk.xMateRobot, name)]
    rt_types = ("JointPosition", "RtControllerMode")
    missing.extend(name for name in rt_types if not hasattr(sdk, name))
    if not hasattr(sdk.MotionControlMode, "RtCommandMode"):
        missing.append("MotionControlMode.RtCommandMode")
    if missing:
        raise XCoreError(f"SDK lacks required APIs: {missing}")
    return {
        "python": platform.python_version(),
        "platform": platform.system(),
        "architecture": platform.machine(),
        "sdk_version": sdk.BaseRobot.sdkVersion(),
        "sdk_library": sdk.__file__,
        "required_apis": list(methods),
        "rt_follow_apis": [*rt_types, "MotionControlMode.RtCommandMode"],
        "hardware_connected": False,
    }
