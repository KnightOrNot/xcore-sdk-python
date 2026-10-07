import math

import pytest

from xcore_sdk_python import RobotConnection, RobotDriver, XCoreError


def test_query_session_does_not_issue_preparation_commands(fake):
    sdk, robot = fake
    with RobotConnection("192.168.0.160", sdk=sdk, robot=robot) as arm:
        status = arm.status()
        assert status["connected"] is True
        assert status["robot"]["joint_num"] == 6
        assert status["mode"] == "automatic"
        assert arm.dh()["nominal"] is False
    assert robot.calls[-1] == "disconnectFromRobot"
    assert not any(name.startswith(("set", "move")) for name in robot.calls)


def test_axis_mismatch_disconnects(fake):
    sdk, robot = fake
    robot.axes = 7
    with pytest.raises(XCoreError, match="six axes"):
        with RobotConnection("192.168.0.160", sdk=sdk, robot=robot):
            pass
    assert robot.calls[-1] == "disconnectFromRobot"


def test_query_failure_is_not_silently_converted_to_zero(fake):
    sdk, robot = fake
    robot.failure = "jointPos"
    arm = RobotConnection("192.168.0.160", sdk=sdk, robot=robot)
    with pytest.raises(XCoreError, match="jointPos.*42"):
        arm.joints()


def test_sdk_padded_arrays_keep_only_arm_axes_in_control_data(fake, monkeypatch):
    sdk, robot = fake
    robot.position = [0.1] * 6 + [0.0] * 6
    monkeypatch.setattr(
        robot,
        "getRobotCfg_DHparam",
        lambda nominal, ec: ec.update(ec=0) or [1.0] * 24 + [0.0] * 4,
    )
    arm = RobotConnection("192.168.2.160", sdk=sdk, robot=robot)
    assert arm.joints()["rad"] == [0.1] * 6
    assert arm.joints()["raw"] == robot.position
    assert arm.joints()["extra_sdk_values"] == [0.0] * 6
    assert arm.dh()["rows"] == [[1.0] * 4 for _ in range(6)]
    assert len(arm.dh()["raw"]) == 28
    assert arm.dh()["extra_sdk_values"] == [0.0] * 4


@pytest.mark.parametrize("positions", [[0.0] * 5, [math.nan] * 12])
def test_invalid_sdk_joint_arrays_are_rejected(fake, positions):
    sdk, robot = fake
    robot.position = positions
    arm = RobotConnection("192.168.2.160", sdk=sdk, robot=robot)
    with pytest.raises(XCoreError, match="finite values"):
        arm.joints()


@pytest.mark.parametrize("target", [[math.nan] * 6, [0] * 5, [math.inf] * 6])
def test_invalid_joint_data_is_rejected_before_hardware_calls(fake, target):
    sdk, robot = fake
    arm = RobotDriver("192.168.0.160", sdk=sdk, robot=robot)
    with pytest.raises(XCoreError, match="finite values"):
        arm.movej(target)
    assert robot.calls == []


@pytest.mark.parametrize(
    "condition", ["off", "manual", "busy", "limits", "range", "step"]
)
def test_motion_preconditions_do_not_send_motion(fake, condition):
    sdk, robot = fake
    target = [0.01] * 6
    if condition == "off":
        robot.power = sdk.PowerState.off
    elif condition == "manual":
        robot.mode = sdk.OperateMode.manual
    elif condition == "busy":
        robot.operation = sdk.OperationState.moving
    elif condition == "limits":
        robot.limit_enabled = False
    elif condition == "range":
        robot.limit_bounds[0] = [-0.005, 0.005]
    else:
        target = [0.5] * 6
    arm = RobotDriver("192.168.0.160", sdk=sdk, robot=robot)
    with pytest.raises(XCoreError):
        arm.movej(target)
    assert not any(name.startswith(("set", "move")) for name in robot.calls)


def test_motion_reports_actual_target_and_requests_zero_blend(fake):
    sdk, robot = fake
    arm = RobotDriver("192.168.0.160", sdk=sdk, robot=robot)
    result = arm.movej([0.01] * 6)
    assert result["reached"]
    assert result["actual_rad"] == [0.01] * 6
    assert result["command_id"] == "test-command"
    assert robot.calls.index("moveReset") < robot.calls.index("moveStart")


@pytest.mark.parametrize("requested, expected", [(None, 1000), (50, 50)])
def test_motion_speed_default_and_override_are_forwarded_to_sdk(
    fake, monkeypatch, requested, expected
):
    sdk, robot = fake
    factory = sdk.MoveAbsJCommand
    sent = []

    def command(target, speed, zone):
        sent.append((speed, zone))
        return factory(target, speed, zone)

    monkeypatch.setattr(sdk, "MoveAbsJCommand", command)
    arm = RobotDriver("192.168.2.160", sdk=sdk, robot=robot)
    options = {} if requested is None else {"speed": requested}
    result = arm.movej([0.01] * 6, **options)
    assert sent == [(expected, 0)]
    assert result["speed_mm_s"] == expected


def test_failed_start_requests_stop(fake):
    sdk, robot = fake
    robot.failure = "moveStart"
    arm = RobotDriver("192.168.0.160", sdk=sdk, robot=robot)
    with pytest.raises(XCoreError, match="moveStart"):
        arm.movej([0.01] * 6)
    assert robot.calls[-1] == "stop"


def test_idle_without_reaching_target_is_not_success(fake):
    sdk, robot = fake
    robot.reach_target = False
    arm = RobotDriver("192.168.0.160", sdk=sdk, robot=robot)
    with pytest.raises(XCoreError, match="timed out"):
        arm.movej([0.1] * 6, motion_timeout=0.001)
    assert robot.calls[-1] == "stop"
