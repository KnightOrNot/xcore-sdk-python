from __future__ import annotations

import pytest

from xcore_sdk_python.cli import parser, validate


def test_follow_server_is_dry_run_by_default() -> None:
    root = parser()
    args = root.parse_args(["follow-server"])

    assert args.enable_motion is False
    assert args.host == "127.0.0.1"
    assert args.port == 6001
    assert args.max_speed_deg == 3.0
    validate(args, root)


def test_follow_client_requires_calibration_for_motion() -> None:
    root = parser()
    args = root.parse_args(["follow", "--allow-uncalibrated"])

    with pytest.raises(SystemExit):
        validate(args, root)


def test_follow_client_allows_uncalibrated_dry_run() -> None:
    root = parser()
    args = root.parse_args(["follow", "--dry-run", "--allow-uncalibrated"])

    validate(args, root)


def test_follow_calibration_requires_matching_pose_attestation() -> None:
    root = parser()

    with pytest.raises(SystemExit):
        root.parse_args(["follow-calibrate"])
