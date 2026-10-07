"""Leader-to-CR7 calibration schema and read-only reference-pose sampling."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np

from .reader import RobotConnection


def load_calibration(path: str | Path) -> tuple[list[float], list[int]]:
    """Validate six-axis calibration before either device is opened."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    offsets = np.asarray(data["joint_offsets"], dtype=float)
    signs = np.asarray(data["joint_signs"], dtype=float)
    if offsets.shape != (6,) or not np.all(np.isfinite(offsets)):
        raise ValueError("Calibration needs six finite offsets")
    if signs.shape != (6,) or not np.all(np.isin(signs, (-1, 1))):
        raise ValueError("Calibration needs six signs, each -1 or 1")
    if data.get("ok") is False:
        raise ValueError("Calibration is marked ok=false; recalibrate before following")
    return offsets.tolist(), signs.astype(int).tolist()


def reference_offsets(raw_samples: Any, robot_joints: Any, signs: Any) -> np.ndarray:
    """Solve q=sign*(raw-offset) using circular means, modulo 2π."""
    raw = np.asarray(raw_samples, dtype=float)
    joints = np.asarray(robot_joints, dtype=float)
    signs = np.asarray(signs, dtype=float)
    if raw.ndim != 2 or raw.shape[1] != 6 or raw.shape[0] < 2:
        raise ValueError("Calibration requires at least two six-axis samples")
    if joints.shape != (6,) or signs.shape != (6,):
        raise ValueError("Reference joints and signs must each contain six values")
    if not np.all(np.isfinite(raw)) or not np.all(np.isfinite(joints)):
        raise ValueError("Calibration sample values must be finite")
    if not np.all(np.isin(signs, (-1, 1))):
        raise ValueError("Calibration signs must be -1 or 1")
    resultant = np.mean(np.exp(1j * raw), axis=0)
    if np.any(np.abs(resultant) < 0.99):
        raise ValueError("Leader moved during sampling; hold both arms still")
    return (np.angle(resultant) - signs * joints + math.pi) % (2 * math.pi) - math.pi


def run_check(args: Any) -> int:
    """Read the leader once and close it, without opening a robot SDK session."""
    from .gello_leader import Cr7LeaderAgent

    offsets, signs = load_calibration(args.calib)
    agent = Cr7LeaderAgent(
        args.serial,
        baudrate=args.baudrate,
        joint_offsets=offsets,
        joint_signs=signs,
        verbose=False,
    )
    try:
        joints = agent.get_joint_state().tolist()
        print(
            json.dumps(
                {
                    "ok": True,
                    "command": "follow-check",
                    "result": {
                        "serial": args.serial,
                        "calibration": str(args.calib),
                        "leader_joints_rad": joints,
                        "joint_signs": signs,
                    },
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    finally:
        agent.close()
    return 0


def calibrate(args: Any) -> int:
    """Create offsets from one matching pose; does not move either arm."""
    from .gello_leader import Cr7LeaderAgent, LeaderReadError

    if not args.ref_current:
        raise ValueError("--ref-current requires both arms to be posed identically")
    signs = np.asarray(args.signs, dtype=float)
    if signs.shape != (6,) or not np.all(np.isin(signs, (-1, 1))):
        raise ValueError("--signs must contain six values, each -1 or 1")
    with RobotConnection(args.ip, local_ip=args.local_ip, sdk_dir=args.sdk_dir) as arm:
        leader = Cr7LeaderAgent(
            args.serial,
            baudrate=args.baudrate,
            joint_offsets=[0.0] * 6,
            joint_signs=signs,
        )
        try:
            samples = []
            for _ in range(args.samples):
                raw = leader.read_raw_rad()
                if raw is None:
                    raise LeaderReadError("示教臂采样读取失败")
                samples.append(np.asarray(raw, dtype=float))
                time.sleep(args.interval)
            current = np.asarray(arm.joints()["rad"], dtype=float)
            offsets = reference_offsets(samples, current, signs)
            result = {
                "joint_offsets": offsets.tolist(),
                "joint_signs": signs.astype(int).tolist(),
                "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                "n_samples": len(samples),
                "ok": True,
                "method": "single matching pose; offsets modulo 2pi",
                "robot_ip": args.ip,
            }
        finally:
            leader.close()
    path = Path(args.save)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(
        f"Saved calibration to {path}: offsets={np.round(offsets, 4).tolist()}, "
        f"signs={signs.astype(int).tolist()}"
    )
    print("Single-pose calibration cannot infer axis signs. Verify them with dry-run.")
    return 0
