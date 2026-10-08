from __future__ import annotations

import json
import math
from enum import Enum
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from xcore_sdk_python import cli, follow_prepare, gello_leader
from xcore_sdk_python.exceptions import XCoreError


class Power(Enum):
    off = 0
    on = 1


class Mode(Enum):
    manual = 0
    automatic = 1


class State(Enum):
    idle = 0
    moving = 1


class Arm:
    sdk = SimpleNamespace(PowerState=Power, OperationState=State)

    def __init__(self):
        self.q = [0.7] * 6
        self.power_state = Power.off
        self.mode_state = Mode.manual
        self.pending_stop = 0
        self.moves = []
        self.events = []
        self.bounds = {"enabled": True, "rad": [[-2 * math.pi, 2 * math.pi]] * 6}
        self.fail_move = False

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.events.append("disconnect")

    def call(self, name):
        if name == "operationState":
            if self.pending_stop:
                self.pending_stop -= 1
                return State.moving
            return State.idle
        return {"powerState": self.power_state, "operateMode": self.mode_state}[name]

    def limits(self):
        return self.bounds

    def joints(self):
        return {"rad": self.q.copy()}

    def mode(self, value):
        assert self.pending_stop == 0
        self.mode_state = Mode[value]
        self.events.append(("mode", value))

    def power(self, value):
        assert self.pending_stop == 0
        self.power_state = Power.on if value else Power.off
        self.events.append(("power", value))

    def movej(self, target, **options):
        self.moves.append((target.copy(), options))
        if self.fail_move:
            raise KeyboardInterrupt
        self.q = target.copy()
        return {"reached": True, "actual_rad": self.q.copy()}

    def stop(self):
        self.events.append("stop")
        self.pending_stop = 2


@pytest.fixture
def hardware(tmp_path, monkeypatch):
    path = tmp_path / "calib.json"
    path.write_text(json.dumps({"joint_offsets": [0.2] * 6, "joint_signs": [1] * 6}))
    args = cli.parser().parse_args(["follow-prepare", "--calib", str(path)])
    cli.validate(args, cli.parser())
    leader = Mock()
    leader.read_raw_rad.return_value = np.array([0.45] * 6)
    arm = Arm()
    factory = Mock(return_value=arm)
    monkeypatch.setattr(gello_leader, "Cr7LeaderAgent", Mock(return_value=leader))
    monkeypatch.setattr(follow_prepare, "RobotDriver", factory)
    monkeypatch.setattr(follow_prepare.time, "sleep", lambda _: None)
    return vars(args), arm, leader, factory


def test_existing_calibration_aligns_directly_without_zeroing_or_recalibration(
    hardware,
):
    args, arm, leader, _ = hardware
    before = args["calib"].read_bytes()
    result = follow_prepare.prepare(args)
    assert len(arm.moves) == 1
    assert arm.moves[0][0] == pytest.approx([0.25] * 6)
    assert arm.moves[0][1]["speed"] == 50
    assert "zero_motion" not in result
    assert not result["calibrated_zero"]
    assert args["calib"].read_bytes() == before
    assert arm.power_state == Power.off and arm.mode_state == Mode.manual
    assert arm.events.index("stop") < arm.events.index(("power", False))
    leader.close.assert_called_once()


def test_first_declared_zero_calibration_moves_to_zero_and_saves_offsets(hardware):
    args, arm, leader, _ = hardware
    args["calib"].unlink()
    args["calibrate_zero"] = True
    result = follow_prepare.prepare(args)
    assert arm.moves[0][0] == [0] * 6
    data = json.loads(args["calib"].read_text())
    assert data["joint_offsets"] == pytest.approx([0.45] * 6)
    assert data["ok"]
    assert result["calibrated_zero"]
    assert arm.q == pytest.approx([0] * 6)
    leader.close.assert_called_once()


@pytest.mark.parametrize(
    "problem", ["limits_disabled", "target_outside", "step_too_large"]
)
def test_invalid_motion_is_rejected_before_mode_or_power_change(hardware, problem):
    args, arm, leader, _ = hardware
    if problem == "limits_disabled":
        arm.bounds["enabled"] = False
    elif problem == "target_outside":
        arm.q = [0] * 6
        arm.bounds["rad"] = [[-0.1, 0.1]] * 6
    else:
        args["max_step_deg"] = 1
    with pytest.raises(XCoreError):
        follow_prepare.prepare(args)
    assert not arm.moves
    assert arm.events == ["disconnect"]
    leader.close.assert_called_once()


def test_interrupted_motion_stops_waits_for_idle_and_restores_initial_state(hardware):
    args, arm, leader, _ = hardware
    arm.fail_move = True
    with pytest.raises(KeyboardInterrupt):
        follow_prepare.prepare(args)
    assert arm.pending_stop == 0
    assert arm.power_state == Power.off and arm.mode_state == Mode.manual
    assert arm.events[-4:] == [
        "stop",
        ("power", False),
        ("mode", "manual"),
        "disconnect",
    ]
    leader.close.assert_called_once()


def test_gello_movement_during_alignment_prevents_follow_handoff(hardware):
    args, arm, leader, _ = hardware
    leader.read_raw_rad.side_effect = [np.array([0.45] * 6)] * 20 + [
        np.array([0.6] * 6)
    ] * 20
    with pytest.raises(XCoreError, match="GELLO moved"):
        follow_prepare.prepare(args)
    assert arm.power_state == Power.off
    leader.close.assert_called_once()


def test_calibration_cannot_be_overwritten_before_devices_open(hardware):
    args, _, _, factory = hardware
    args["calibrate_zero"] = True
    before = args["calib"].read_bytes()
    with pytest.raises(XCoreError, match="already exists"):
        follow_prepare.prepare(args)
    assert args["calib"].read_bytes() == before
    factory.assert_not_called()


def test_nearest_branch_matches_follower_current_posture():
    target = follow_prepare.leader_target(
        np.zeros((20, 6)), [0] * 6, [1] * 6, [6.2] * 6
    )
    assert target == pytest.approx([2 * math.pi] * 6)


def test_atomic_calibration_publication_does_not_replace_existing_file(tmp_path):
    path = tmp_path / "calib.json"
    path.write_text("old calibration")
    with pytest.raises(FileExistsError):
        follow_prepare.save_calibration(path, {"ok": True})
    assert path.read_text() == "old calibration"
    assert not list(tmp_path.glob("*.partial"))


def test_prepare_cli_serializes_paths_and_auto_sizes_worker_deadline(
    monkeypatch, tmp_path
):
    def isolated(options):
        json.dumps(options)
        assert options["calib"] == str(tmp_path / "calib.json")
        assert options["timeout"] == 150
        return {"ok": True, "command": "follow-prepare", "result": {}}

    monkeypatch.setattr(cli, "isolated", isolated)
    assert (
        cli.main(
            [
                "follow-prepare",
                "--calib",
                str(tmp_path / "calib.json"),
                "--motion-timeout",
                "60",
            ]
        )
        == 0
    )


def test_already_aligned_target_does_not_change_power_or_mode(hardware):
    args, arm, _, _ = hardware
    arm.q = [0.25] * 6
    result = follow_prepare.prepare(args)
    assert result["alignment_motion"]["skipped"]
    assert not arm.moves
    assert arm.events == ["disconnect"]


def test_unstable_gello_is_rejected_before_robot_connection(hardware):
    args, _, leader, factory = hardware
    leader.read_raw_rad.side_effect = [np.array([0.45] * 6)] * 19 + [
        np.array([0.6] * 6)
    ]
    with pytest.raises(XCoreError, match="Keep GELLO still"):
        follow_prepare.prepare(args)
    factory.assert_not_called()
    leader.close.assert_called_once()


def test_missing_gripper_service_is_rejected_before_robot_connection(
    hardware, monkeypatch
):
    from xcore_sdk_python import gripper_follow

    args, _, _, factory = hardware
    args["gripper_host"] = "127.0.0.1"
    transport = Mock()
    transport.check.side_effect = RuntimeError("gripper unavailable")
    monkeypatch.setattr(
        gripper_follow, "GripperFollowClient", Mock(return_value=transport)
    )
    with pytest.raises(RuntimeError, match="gripper unavailable"):
        follow_prepare.prepare(args)
    factory.assert_not_called()


def test_repeated_termination_does_not_interrupt_worker_cleanup():
    import signal
    import subprocess
    import sys

    code = """
import signal,time
from xcore_sdk_python.worker import _interrupt
signal.signal(signal.SIGTERM,_interrupt)
try:
    print('ready',flush=True)
    time.sleep(10)
except KeyboardInterrupt:
    print('cleanup_started',flush=True)
    time.sleep(.3)
    print('cleanup_finished',flush=True)
"""
    process = subprocess.Popen(
        [sys.executable, "-u", "-c", code],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        assert process.stdout.readline().strip() == "ready"
        process.send_signal(signal.SIGTERM)
        assert process.stdout.readline().strip() == "cleanup_started"
        process.send_signal(signal.SIGTERM)
        stdout, stderr = process.communicate(timeout=3)
        assert process.returncode == 0, stderr
        assert "cleanup_finished" in stdout
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
