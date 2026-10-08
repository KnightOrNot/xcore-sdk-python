from __future__ import annotations

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from test_follow_prepare import Arm, Mode, Power, State

from xcore_sdk_python import cli
from xcore_sdk_python import return_zero as zero_module
from xcore_sdk_python.exceptions import XCoreError


@pytest.fixture
def hardware(monkeypatch):
    arm = Arm()
    factory = Mock(return_value=arm)
    monkeypatch.setattr(zero_module, "RobotDriver", factory)
    # The shared restore routine waits for idle before restoring mode/power.
    monkeypatch.setattr("xcore_sdk_python.follow_prepare.time.sleep", lambda _: None)
    args = cli.parser().parse_args(["return-zero", "--speed", "4000"])
    cli.validate(args, cli.parser())
    return vars(args), arm, factory


def test_zero_uses_exact_six_axis_target_and_restores_mode_and_power(hardware):
    args, arm, _ = hardware
    arm.q[0] = math.radians(270)
    result = zero_module.return_zero(args)
    assert result["reached"]
    assert arm.moves[0][0] == [0.0] * 6
    assert arm.moves[0][1]["speed"] == 4000
    assert arm.moves[0][1]["max_step_deg"] == 360
    assert arm.power_state == Power.off and arm.mode_state == Mode.manual
    assert arm.events.index("stop") < arm.events.index(("power", False))
    assert arm.events[-1] == "disconnect"


@pytest.mark.parametrize("condition", ["busy", "disabled", "outside", "zero_outside"])
def test_zero_rejects_invalid_motion_before_mode_or_power_changes(hardware, condition):
    args, arm, _ = hardware
    if condition == "busy":
        original_call = arm.call
        arm.call = lambda name: (
            State.moving if name == "operationState" else original_call(name)
        )
    elif condition == "disabled":
        arm.bounds["enabled"] = False
    elif condition == "outside":
        arm.q[0] = 3 * math.pi
    else:
        arm.bounds["rad"][0] = [0.1, 1]
    with pytest.raises(XCoreError):
        zero_module.return_zero(args)
    assert not arm.moves
    assert arm.events == ["disconnect"]


def test_interrupted_zero_stops_before_restoring_and_disconnecting(hardware):
    args, arm, _ = hardware
    arm.fail_move = True
    with pytest.raises(KeyboardInterrupt):
        zero_module.return_zero(args)
    assert arm.events[-4:] == [
        "stop",
        ("power", False),
        ("mode", "manual"),
        "disconnect",
    ]


def test_already_zero_does_not_power_on(hardware):
    args, arm, _ = hardware
    arm.q = [0.0] * 6
    assert zero_module.return_zero(args)["skipped"]
    assert arm.events == ["disconnect"]


def test_zero_cli_auto_sizes_deadline_and_dispatches_in_worker(monkeypatch):
    def inspect(options):
        assert options["timeout"] == 730
        assert options["motion_timeout"] == 700
        return {"ok": True, "command": "return-zero", "result": {"reached": True}}

    monkeypatch.setattr(cli, "isolated", inspect)
    assert cli.main(["return-zero", "--motion-timeout", "700"]) == 0


@pytest.mark.parametrize(
    "options",
    [
        ["--speed", "4001"],
        ["--speed", "nan"],
        ["--motion-timeout", "0"],
        ["--max-step-deg", "0"],
        ["--timeout", "20"],
    ],
)
def test_invalid_zero_options_never_connect(monkeypatch, options):
    monkeypatch.setattr(cli, "isolated", lambda _: pytest.fail("must not connect"))
    with pytest.raises(SystemExit):
        cli.main(["return-zero", *options])


@pytest.fixture
def recovering(hardware):
    args, arm, factory = hardware
    args["recover"] = True
    arm.sdk = SimpleNamespace(
        PowerState=Power,
        OperationState=State,
        OperateMode=Mode,
        MotionControlMode=SimpleNamespace(RtCommandMode="rt", NrtCommandMode="nrt"),
    )
    state = {"current": SimpleNamespace(name="rtControlling")}
    original = arm.call

    def call(name, *values):
        if name == "setMotionControlMode":
            arm.events.append(("motion_mode", values[0]))
            return
        if name == "operationState" and state["current"] != State.idle:
            return state["current"]
        return original(name)

    def recover(ec):
        arm.events.append("recover")
        ec.update(ec=0)
        state["current"] = State.idle

    rt = SimpleNamespace(
        stopMove=lambda: arm.events.append("rt_stop"),
        automaticErrorRecovery=recover,
    )
    arm.call = call
    arm.robot = SimpleNamespace(getRtMotionController=lambda: rt)
    return args, arm, state, rt


def test_fault_recovery_initializes_rt_stops_recovers_then_zeroes(recovering):
    args, arm, _, _ = recovering
    result = zero_module.return_zero(args)
    assert result["reached"] and result["rt_recovered"]
    assert arm.events[:4] == [
        ("motion_mode", "rt"),
        "rt_stop",
        "recover",
        ("motion_mode", "nrt"),
    ]
    assert len(arm.moves) == 1 and arm.moves[0][0] == [0.0] * 6
    assert arm.power_state == Power.off and arm.mode_state == Mode.manual
    assert arm.events.count("recover") == 1


@pytest.mark.parametrize(
    "condition", ["estop", "gstop", "unknown", "busy", "limits", "recovery_failed"]
)
def test_recovery_failure_or_interlock_never_sends_zero_or_powers_on(
    recovering, condition
):
    args, arm, state, rt = recovering
    if condition in ("estop", "gstop", "unknown"):
        arm.power_state = SimpleNamespace(name=condition)
    elif condition == "busy":
        state["current"] = State.moving
    elif condition == "limits":
        arm.bounds["enabled"] = False
    else:
        rt.automaticErrorRecovery = lambda ec: ec.update(ec=-1, message="refused")
    with pytest.raises(XCoreError):
        zero_module.return_zero(args)
    assert not arm.moves
    assert ("power", True) not in arm.events


def test_recovered_already_zero_finishes_idle_off_manual(recovering):
    args, arm, _, _ = recovering
    arm.q = [0.0] * 6
    arm.power_state = Power.on
    arm.mode_state = Mode.automatic
    assert zero_module.return_zero(args)["skipped"]
    assert not arm.moves
    assert arm.power_state == Power.off and arm.mode_state == Mode.manual
