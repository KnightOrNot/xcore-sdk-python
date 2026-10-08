from __future__ import annotations

import json
import socketserver
import threading
import time
from unittest.mock import Mock

import numpy as np
import pytest

from xcore_sdk_python import gello_client, gello_leader
from xcore_sdk_python.cli import parser, validate
from xcore_sdk_python.gripper_follow import GripperFollower


def test_gripper_check_never_opens_arm_session(monkeypatch):
    from xcore_sdk_python import commands, gripper_follow

    transport = Mock()
    transport.check.return_value = {"streaming": True, "position_raw": 128}
    create = Mock(return_value=transport)
    monkeypatch.setattr(gripper_follow, "GripperFollowClient", create)
    monkeypatch.setattr(
        commands, "RobotConnection", lambda **_: pytest.fail("must not connect CR7")
    )
    args = parser().parse_args(["gripper-check", "--gripper-host", "127.0.0.1"])
    validate(args, parser())
    result = commands.execute(vars(args))
    assert result["position_raw"] == 128
    create.assert_called_once_with("127.0.0.1", 5005, 0.75)
    transport.request.assert_not_called()


def wait_for(predicate):
    deadline = time.monotonic() + 2
    while not predicate():
        assert time.monotonic() < deadline
        time.sleep(0.005)


def test_one_serial_frame_contains_arm_and_trigger_and_failure_keeps_seven(monkeypatch):
    arm = Mock()
    ticks = {i: 0 for i in range(1, 7)}
    ticks[7] = round(173.9 / 360 * 4096)
    arm.read_ticks.side_effect = [ticks, ticks, None, {**ticks, 7: None}]
    create = Mock(return_value=arm)
    monkeypatch.setattr(gello_leader, "TeachArm", create)
    agent = gello_leader.Cr7LeaderAgent(
        joint_offsets=[0] * 6,
        gripper_id=7,
        gripper_config=(194.8, 153),
        verbose=False,
    )
    try:
        sample = agent.get_joint_state()
        assert sample.shape == (7,)
        assert sample[6] == pytest.approx(0.5, abs=0.002)
        assert arm.read_ticks.call_count == 2  # Constructor plus exactly one frame.
        np.testing.assert_equal(agent.get_joint_state(), sample)
        np.testing.assert_equal(agent.get_joint_state(), sample)
        assert arm.read_ticks.call_count == 4
        assert create.call_args.args[-1] == 7
    finally:
        agent.close()


def test_worker_never_queues_old_targets_or_blocks_submit():
    entered, release = threading.Event(), threading.Event()
    requests = []

    def request(cmd, **args):
        requests.append((cmd, args))
        if cmd == "set_target" and not entered.is_set():
            entered.set()
            assert release.wait(2)
        return {"position_raw": args.get("pos", 0)}

    client = Mock(timeout=0.75, request=request)
    worker = GripperFollower(client, hz=10)
    try:
        worker.submit(0)
        assert entered.wait(1)
        # All submissions finish while the first network request is still held.
        for closure in np.linspace(0, 1, 50):
            worker.submit(float(closure))
        assert len(requests) == 1
        release.set()
        wait_for(lambda: len(requests) >= 2)
        assert requests[1][1]["pos"] == 255
    finally:
        release.set()
        worker.close()
    assert requests[-1][0] == "stop"
    assert not worker._thread.is_alive()


@pytest.mark.parametrize("failure", ["network", "source_stale"])
def test_worker_failure_stops_gripper_and_propagates(failure):
    calls = []

    def request(cmd, **kwargs):
        calls.append(cmd)
        if cmd == "set_target" and failure == "network":
            raise OSError("gripper disconnected")
        return {}

    worker = GripperFollower(
        Mock(timeout=0.1, request=request), hz=100, stale_timeout=0.05
    )
    try:
        worker.submit(0.4)
        wait_for(lambda: worker._error is not None)
        with pytest.raises(RuntimeError, match="disconnected|timed out"):
            worker.check()
        wait_for(lambda: "stop" in calls)
    finally:
        worker.close()


@pytest.mark.parametrize("mode", ["dry-run", "motion", "arm-only"])
def test_unified_client_routes_one_sample_to_two_endpoints(monkeypatch, tmp_path, mode):
    calibration = tmp_path / "calib.json"
    calibration.write_text(
        json.dumps({"joint_offsets": [0] * 6, "joint_signs": [1] * 6})
    )
    arguments = ["follow", "--calib", str(calibration), "--yes"]
    if mode != "arm-only":
        arguments += ["--gripper-host", "127.0.0.1"]
    if mode == "dry-run":
        arguments += ["--dry-run"]
    args = parser().parse_args(arguments)
    sample = np.array([0.1, -0.2, 0.15, -0.1, 0.2, -0.15, 0.75])
    agent = Mock()
    agent.q_without_branch.return_value = np.zeros(6)
    agent.get_joint_state.side_effect = [
        sample if mode != "arm-only" else sample[:6],
        KeyboardInterrupt(),
    ]
    create_agent = Mock(return_value=agent)
    monkeypatch.setattr(gello_leader, "Cr7LeaderAgent", create_agent)
    arm = Mock()
    arm.call.side_effect = lambda cmd, **kw: 6 if cmd == "num_dofs" else [0] * 6
    monkeypatch.setattr(gello_client, "ZmqRobotClient", Mock(return_value=arm))
    transport = Mock()
    monkeypatch.setattr(
        gello_client, "GripperFollowClient", Mock(return_value=transport)
    )
    worker = Mock()
    create_worker = Mock(return_value=worker)
    monkeypatch.setattr(gello_client, "GripperFollower", create_worker)
    assert gello_client.run_client(args) == 0
    assert create_agent.call_count == 1
    commands = [
        call
        for call in arm.call.call_args_list
        if call.args[0] == "command_joint_state"
    ]
    if mode == "dry-run":
        assert commands == []
        create_worker.assert_not_called()
        assert [c.args[0] for c in transport.request.call_args_list] == []
    else:
        assert commands[0].kwargs["joint_state"] == pytest.approx(sample[:6])
        if mode == "motion":
            worker.submit.assert_called_once_with(0.75)
            worker.close.assert_called_once()
    assert ("gripper_id" in create_agent.call_args.kwargs) == (mode != "arm-only")
    agent.close.assert_called_once()
    arm.close.assert_called_once()


@pytest.mark.parametrize("show_state", [False, True])
def test_state_output_does_not_change_arm_commands(
    monkeypatch, tmp_path, capsys, show_state
):
    path = tmp_path / "calib.json"
    path.write_text(json.dumps({"joint_offsets": [0] * 6, "joint_signs": [1] * 6}))
    args = parser().parse_args(
        ["follow", "--calib", str(path), "--yes"]
        + (["--show-state"] if show_state else [])
    )
    agent = Mock()
    agent.q_without_branch.return_value = np.zeros(6)
    agent.get_joint_state.side_effect = [np.full(6, 0.1), KeyboardInterrupt()]
    monkeypatch.setattr(gello_leader, "Cr7LeaderAgent", Mock(return_value=agent))
    arm = Mock()
    arm.call.side_effect = lambda cmd, **_: 6 if cmd == "num_dofs" else [0] * 6
    monkeypatch.setattr(gello_client, "ZmqRobotClient", Mock(return_value=arm))
    assert gello_client.run_client(args) == 0
    output = capsys.readouterr().out
    assert ("leader=" in output) is show_state
    assert "Startup alignment max error:" in output
    commands = [
        c for c in arm.call.call_args_list if c.args[0] == "command_joint_state"
    ]
    assert len(commands) == 1
    assert commands[0].kwargs["joint_state"] == pytest.approx([0.1] * 6)
    feedback_calls = [
        c for c in arm.call.call_args_list if c.args[0] == "get_joint_state"
    ]
    assert len(feedback_calls) == (2 if show_state else 1)


def test_single_reader_drives_six_arm_joints_and_real_tcp_gripper(
    monkeypatch, tmp_path
):
    requests = []
    positions = []

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            request = json.loads(self.rfile.readline())
            requests.append(request)
            if request["cmd"] == "set_target":
                positions.append(request["pos"])
            result = {
                "streaming": True,
                "stream_error": None,
                "status_code": 0x31,
                "position_raw": positions[-1] if positions else 0,
            }
            self.wfile.write(
                (json.dumps({"ok": True, "result": result}) + "\n").encode()
            )

    path = tmp_path / "calib.json"
    path.write_text(json.dumps({"joint_offsets": [0] * 6, "joint_signs": [1] * 6}))
    with socketserver.TCPServer(("127.0.0.1", 0), Handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            args = parser().parse_args(
                [
                    "follow",
                    "--calib",
                    str(path),
                    "--yes",
                    "--gripper-host",
                    "127.0.0.1",
                    "--gripper-port",
                    str(server.server_address[1]),
                ]
            )
            samples = [
                np.array([closure / 10] * 6 + [closure]) for closure in (0, 0.5, 1)
            ]
            index = 0

            def read_sample():
                nonlocal index
                if index:
                    previous = round(float(samples[index - 1][6]) * 255)
                    wait_for(lambda: positions and positions[-1] == previous)
                if index == len(samples):
                    raise KeyboardInterrupt
                sample = samples[index]
                index += 1
                return sample

            agent = Mock()
            agent.q_without_branch.return_value = np.zeros(6)
            agent.get_joint_state.side_effect = read_sample
            create = Mock(return_value=agent)
            monkeypatch.setattr(gello_leader, "Cr7LeaderAgent", create)
            arm = Mock()
            arm.call.side_effect = lambda cmd, **_: 6 if cmd == "num_dofs" else [0] * 6
            monkeypatch.setattr(gello_client, "ZmqRobotClient", Mock(return_value=arm))
            assert gello_client.run_client(args) == 0
            assert create.call_count == 1
            assert create.call_args.kwargs["gripper_id"] == 7
            targets = [
                call.kwargs["joint_state"]
                for call in arm.call.call_args_list
                if call.args[0] == "command_joint_state"
            ]
            np.testing.assert_allclose(targets, [sample[:6] for sample in samples])
            assert positions == [0, 128, 255]
            assert requests[0]["cmd"] == "follow_status"
            assert requests[-1]["cmd"] == "stop"
            agent.close.assert_called_once()
            arm.close.assert_called_once()
        finally:
            server.shutdown()
            thread.join(timeout=2)


@pytest.mark.parametrize(
    "options",
    [
        ["--gripper-id", "6"],
        ["--gripper-port", "0"],
        ["--gripper-open-deg", "153"],
        ["--gripper-force", "256"],
        ["--gripper-hz", "0"],
        ["--gripper-stale-timeout", "0.5"],
        ["--gripper-open-pos", "255"],
    ],
)
def test_invalid_gripper_settings_rejected_before_opening_devices(options):
    root = parser()
    args = root.parse_args(["follow", *options])
    with pytest.raises(SystemExit):
        validate(args, root)
