from __future__ import annotations

import json
import queue
import time
from unittest.mock import Mock

import numpy as np
import pytest

from xcore_sdk_python import gello_client, gello_leader
from xcore_sdk_python.cli import parser, validate
from xcore_sdk_python.raw_recorder import RawEpisodeRecorder, RecordingError
from xcore_sdk_python.recording import sample_from_cycle


def cycle(**changes):
    values = dict(
        action=np.array([0.1] * 6 + [1.0]),
        arm={"joint_positions": [-0.2] * 6, "joint_state_time_ns": 990_000_000},
        gripper={"position_raw": 116, "feedback_time_ns": 800_000_000},
        command_time_ns=999_000_000,
        observation_time_ns=1_000_000_000,
        wall_time_ns=2_000_000_000,
        control_period_ns=20_000_000,
        open_pos=2,
        closed_pos=230,
        max_age_s=0.75,
    )
    values.update(changes)
    return sample_from_cycle(**values)


def test_records_measured_feedback_instead_of_leader_targets():
    sample = cycle()
    assert sample["action"] == [0.1] * 6 + [1.0]
    assert sample["joint_positions"] == [-0.2] * 6 + [0.5]
    assert sample["gripper_position_raw"] == 116
    assert sample["gripper_feedback_age_ns"] == 200_000_000
    assert "ee_pos_quat" not in sample and "joint_velocities" not in sample


@pytest.mark.parametrize(
    "feedback",
    [
        {},
        {"position_raw": 256, "feedback_time_ns": 800_000_000},
        {"position_raw": 116, "feedback_time_ns": 1},
        {"position_raw": 116, "feedback_time_ns": 1_100_000_000},
    ],
)
def test_invalid_or_stale_feedback_is_not_recorded(feedback):
    with pytest.raises(RecordingError):
        cycle(gripper=feedback)


def test_queue_full_is_reported_without_silently_dropping_samples(
    tmp_path, monkeypatch
):
    recorder = RawEpisodeRecorder(
        tmp_path, control_hz=50, joint_signs=[1] * 6, task="test"
    )
    partial = recorder.start_episode()

    def full(_sample):
        raise queue.Full

    monkeypatch.setattr(recorder._sample_queue, "put_nowait", full)
    with pytest.raises(RecordingError, match="queue is full"):
        recorder.add_sample(cycle())
    assert recorder.close_interrupted() == partial
    assert partial.exists()


@pytest.mark.parametrize(
    "options",
    [
        ["--dry-run"],
        [],
        ["--gripper-host", "127.0.0.1", "--record-queue-size", "0"],
        ["--gripper-host", "127.0.0.1", "--task", " "],
    ],
)
def test_invalid_recording_options_rejected_before_devices_open(options):
    root = parser()
    args = root.parse_args(["follow", "--raw-data-root", "/tmp/unused", *options])
    with pytest.raises(SystemExit):
        validate(args, root)


@pytest.mark.parametrize(
    "stale,cleanup_error,save",
    [
        (False, None, True),
        (False, None, False),
        (True, None, False),
        (False, "leader", False),
        (False, "arm", False),
        (False, "gripper", False),
    ],
)
def test_follow_records_actual_arm_and_gripper_and_saves_only_on_s(
    tmp_path,
    monkeypatch,
    stale,
    cleanup_error,
    save,
):
    calibration = tmp_path / "calib.json"
    calibration.write_text(
        json.dumps({"joint_offsets": [0] * 6, "joint_signs": [1] * 6})
    )
    args = parser().parse_args(
        [
            "follow",
            "--calib",
            str(calibration),
            "--yes",
            "--gripper-host",
            "127.0.0.1",
            "--raw-data-root",
            str(tmp_path / "raw"),
            "--session-path-file",
            str(tmp_path / "session.txt"),
            "--start-recording",
        ]
    )
    agent = Mock()
    agent.q_without_branch.return_value = np.zeros(6)
    agent.get_joint_state.side_effect = [
        np.array([0.1] * 6 + [0.9]),
        np.array([0.2] * 6 + [0.8]),
        KeyboardInterrupt(),
    ]
    monkeypatch.setattr(gello_leader, "Cr7LeaderAgent", Mock(return_value=agent))
    arm = Mock()

    def call(cmd, **kwargs):
        if cmd == "num_dofs":
            return 6
        if cmd == "get_observations":
            return {
                "joint_positions": [-0.3] * 6,
                "joint_state_time_ns": time.monotonic_ns(),
            }
        return [0] * 6

    arm.call.side_effect = call
    monkeypatch.setattr(gello_client, "ZmqRobotClient", Mock(return_value=arm))
    transport = Mock()
    transport.check.return_value = {"position_raw": 100}
    monkeypatch.setattr(
        gello_client, "GripperFollowClient", Mock(return_value=transport)
    )
    worker = Mock()
    worker.feedback.side_effect = lambda: {
        "position_raw": 100,
        "feedback_time_ns": 1 if stale else time.monotonic_ns(),
    }
    monkeypatch.setattr(gello_client, "GripperFollower", Mock(return_value=worker))
    keyboard = Mock()
    keyboard.poll.side_effect = [None, None, "s" if save else None]
    monkeypatch.setattr(gello_client, "RecordingKeyboard", Mock(return_value=keyboard))
    if cleanup_error:
        devices = {"leader": agent, "arm": arm, "gripper": worker}
        devices[cleanup_error].close.side_effect = RuntimeError("cleanup failed")
        with pytest.raises(RuntimeError, match="cleanup failed"):
            gello_client.run_client(args)
    elif stale:
        with pytest.raises(RecordingError, match="stale"):
            gello_client.run_client(args)
    else:
        assert gello_client.run_client(args) == 0
    session = tmp_path.joinpath("session.txt").read_text().strip()
    from pathlib import Path

    episodes = list(Path(session).joinpath("episodes").iterdir())
    assert len(episodes) == 1
    if stale or cleanup_error or not save:
        assert episodes[0].name.endswith(".partial")
        if cleanup_error:
            assert len(episodes[0].read_text().splitlines()) == 2
    else:
        assert episodes[0].suffix == ".jsonl"
        rows = [json.loads(line) for line in episodes[0].read_text().splitlines()]
        assert len(rows) == 2
        assert rows[0]["joint_positions"][:6] == [-0.3] * 6
        assert rows[0]["joint_positions"][-1] == pytest.approx(100 / 255)
        assert rows[0]["action"][-1] == 0.9
    worker.close.assert_called_once()
    arm.close.assert_called_once()
    agent.close.assert_called_once()
    keyboard.close.assert_called_once()
    if not stale and not cleanup_error:
        # The real recording loop has closed its writer before the exit move.
        # Verify both saved and interrupted raw episodes remain byte-identical.
        from test_follow_prepare import Arm

        from xcore_sdk_python import return_zero as zero_module

        raw_files = {
            path: path.read_bytes()
            for path in Path(session).rglob("*")
            if path.is_file()
        }
        home_arm = Arm()
        monkeypatch.setattr(zero_module, "RobotDriver", Mock(return_value=home_arm))
        monkeypatch.setattr(
            "xcore_sdk_python.follow_prepare.time.sleep", lambda _: None
        )
        zero_args = parser().parse_args(["return-zero"])
        validate(zero_args, parser())
        assert zero_module.return_zero(vars(zero_args))["reached"]
        assert home_arm.q == [0.0] * 6
        assert raw_files == {
            path: path.read_bytes()
            for path in Path(session).rglob("*")
            if path.is_file()
        }
        rows = [json.loads(line) for line in episodes[0].read_text().splitlines()]
        assert len(rows) == 2
        assert all(row["joint_positions"][:6] == [-0.3] * 6 for row in rows)
