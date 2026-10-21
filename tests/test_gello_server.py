from __future__ import annotations

import pickle
from unittest.mock import MagicMock, Mock

import pytest
import zmq

from xcore_sdk_python import cli, gello_server


@pytest.mark.parametrize("stage", ["before_request", "during_command", "idle"])
def test_abort_reason_reaches_pending_client_before_server_cleanup(monkeypatch, stage):
    """Exercise the REP loop with fake SDK/socket; no hardware or TCP session."""
    reason = "拒绝启动：J1(+183.5° 限 -180~+180°)"
    aborted = [None if stage == "during_command" else reason]
    bot = Mock()
    bot.abort_reason.side_effect = lambda: aborted[0]

    def reject(**_kwargs):
        aborted[0] = reason
        raise RuntimeError(reason)

    bot.command_joint_state.side_effect = reject
    connection = MagicMock()
    socket = Mock()
    if stage == "idle":
        socket.recv.side_effect = zmq.Again()
    else:
        socket.recv.return_value = pickle.dumps(
            {"method": "command_joint_state", "args": {"joint_state": [0] * 6}}
        )
    events = []
    socket.send.side_effect = lambda _: events.append("reply")
    bot.close.side_effect = lambda: events.append("close")
    context = Mock()
    context.socket.return_value = socket
    monkeypatch.setattr(zmq, "Context", Mock(return_value=context))
    monkeypatch.setattr(gello_server, "RobotConnection", Mock(return_value=connection))
    monkeypatch.setattr(gello_server, "GelloCr7Robot", Mock(return_value=bot))
    monkeypatch.setattr(gello_server.signal, "signal", lambda *_: 0)
    args = cli.parser().parse_args(["follow-server"])

    assert gello_server.run_server(args) == 1
    socket.recv.assert_called_once()
    if stage == "idle":
        socket.send.assert_not_called()
    else:
        response = pickle.loads(socket.send.call_args.args[0])
        assert reason in response["error"]
        assert events == ["reply", "close"]
    bot.close.assert_called_once()
    connection.close.assert_called_once()
