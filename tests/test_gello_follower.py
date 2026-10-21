import time
from types import SimpleNamespace

import numpy as np
import pytest

from xcore_sdk_python import gello_follower as follower
from xcore_sdk_python.gello_server import CheckedRobot, dispatch


class Limits:
    def content(self):
        return [[-1.0, 1.0]] * 6


class JointCommand:
    def setFinished(self):
        self.finished = True


class Robot:
    def __init__(self):
        self.enabled = True

    def getSoftLimit(self, buffer, ec):
        ec.update(ec=0)
        return self.enabled

    def jointPos(self, ec):
        ec.update(ec=0)
        return [0.0] * 6 + [0.0]


@pytest.fixture
def adapter(monkeypatch):
    monkeypatch.setattr(follower.threading.Thread, "start", lambda _: None)
    monkeypatch.setattr(follower.threading.Thread, "join", lambda *a, **kw: None)
    sdk = SimpleNamespace(PyTypeVectorArrayDouble2=Limits, JointPosition=JointCommand)
    raw = Robot()
    bot = follower.GelloCr7Robot(CheckedRobot(raw), sdk, verbose=False)
    yield bot, sdk, raw
    bot.close()


def test_controller_limits_use_content_api_and_cannot_be_disabled(adapter):
    bot, sdk, raw = adapter
    assert bot._soft_lo == pytest.approx([-1.0] * 6)
    raw.enabled = False
    with pytest.raises(RuntimeError, match="disabled"):
        follower.resolve_soft_limits(raw, sdk, {}, 6, None, None)


def test_controller_limits_are_intersected_with_shared_following_bounds(adapter):
    _, sdk, _ = adapter
    bounds = [[-2 * np.pi, 2 * np.pi]] * 6
    bounds[1] = [-1, 1]  # a stricter controller setting must still take precedence

    class Buffer:
        def content(self):
            return bounds

    sdk.PyTypeVectorArrayDouble2 = Buffer
    lo, hi = follower.resolve_soft_limits(Robot(), sdk, {}, 6, None, None)
    assert lo[0] == pytest.approx(-np.pi)
    assert hi[0] == pytest.approx(np.pi)
    assert [lo[1], hi[1]] == pytest.approx([-1, 1])


def test_first_target_returns_outside_posture_error_before_caching_or_power(adapter):
    bot, _, _ = adapter
    bot._enable_motion_requested = True
    bot._soft_lo[0], bot._soft_hi[0] = -np.pi, np.pi
    bot._state[0] = np.radians(183.5)
    with pytest.raises(RuntimeError, match=r"J1\(\+183.5"):
        dispatch(
            bot,
            {
                "method": "command_joint_state",
                "args": {"joint_state": bot._state.tolist()},
            },
        )
    assert bot._target is None
    assert bot._n_commands == 0
    assert bot.abort_reason() is not None
    assert not bot._powered_by_us


def test_first_target_alignment_gate_is_checked_before_caching(adapter):
    bot, _, _ = adapter
    bot._enable_motion_requested = True
    with pytest.raises(RuntimeError, match="leader 与真机姿态差"):
        bot.command_joint_state([0.5] * 6)
    assert bot._target is None
    assert not bot._powered_by_us


def test_commands_are_cached_and_finite_and_protocol_is_six_axis(adapter):
    bot, _, _ = adapter
    assert dispatch(bot, {"method": "num_dofs"}) == 6
    dispatch(bot, {"method": "command_joint_state", "args": {"joint_state": [2] * 6}})
    assert bot._target == pytest.approx([1] * 6)
    assert bot.get_joint_state() == pytest.approx([0] * 6)
    with pytest.raises(ValueError, match="finite"):
        bot.command_joint_state([float("nan")] * 6)
    with pytest.raises(ValueError):
        bot.command_joint_state([0] * 7)


def test_callback_starts_at_feedback_caps_delayed_steps_and_finishes(
    adapter, monkeypatch
):
    bot, sdk, _ = adapter
    bot._out = [0.0] * 6
    bot._accl_v = [0.0] * 6
    bot._cb_safe = [0.0] * 6
    bot._target = np.full(6, 0.3)
    ticks = iter([1.0, 1.1, 1.2])
    monkeypatch.setattr(follower.time, "perf_counter", lambda: next(ticks))
    bot._make_callback(sdk)
    first = list(bot._cb().joints)
    assert first == pytest.approx([0] * 6)
    second = list(bot._cb().joints)
    assert 0 < second[0] <= bot._max_speed * follower.DT_CLAMP_S
    bot._cb_state["stop"] = True
    assert bot._cb().finished
    assert bot._cb_state["ended"] is True
    assert bot._cb_state["errors"] == 0


def test_rt_verbose_cycle_does_not_reference_legacy_target(adapter):
    bot, _, _ = adapter
    bot._enable_motion_requested = True
    bot._motion_active = True
    bot._verbose = True
    bot.command_joint_state([0.0] * 6)
    bot._out = [0.0] * 6
    bot._rt_con = SimpleNamespace(hasMotionError=lambda: False, stopMove=lambda: None)
    bot._sleep_to = lambda *_: setattr(bot, "_running", False)
    bot._servo_loop()
    assert bot.abort_reason() is None


def test_servo_exception_records_abort_and_runs_shutdown(adapter):
    bot, _, _ = adapter
    bot._enable_motion_requested = True
    bot._motion_active = True
    bot._target_time = time.time()
    shutdown = []

    def failed():
        raise RuntimeError("RT feedback failed")

    bot._rt_con = SimpleNamespace(
        hasMotionError=failed, stopMove=lambda: shutdown.append(1)
    )
    bot._servo_loop()
    assert "RT feedback failed" in bot.abort_reason()
    assert shutdown == [1]


def test_checked_robot_raises_error_code_instead_of_continuing():
    def refused(ec):
        ec.update(ec=-514, message="power refused")

    checked = CheckedRobot(SimpleNamespace(setPowerState=refused))
    with pytest.raises(RuntimeError, match="-514"):
        checked.setPowerState({"ec": 0})


def test_callback_error_finishes_at_previous_output(adapter):
    bot, sdk, _ = adapter
    bot._out = [0.1] * 6
    bot._accl_v = [0.0] * 6
    bot._target = [None] * 6  # inject an internal callback fault
    bot._make_callback(sdk)
    command = bot._cb()
    assert command.joints == pytest.approx([0.1] * 6)
    assert command.finished
    assert bot._cb_state["errors"] == 1
    bot._enable_motion_requested = True
    bot._motion_active = True
    bot.command_joint_state([0.0] * 6)
    bot._servo_loop()
    assert "RT callback failed" in bot.abort_reason()


def test_failed_power_restores_changed_modes(adapter):
    bot, sdk, _ = adapter
    events = []
    sdk.MotionControlMode = SimpleNamespace(RtCommandMode="rt", NrtCommandMode="nrt")
    sdk.OperateMode = SimpleNamespace(automatic="automatic", manual="manual")

    def refused(on, ec):
        ec.update(ec=-514)

    bot._robot = CheckedRobot(
        SimpleNamespace(
            setRtNetworkTolerance=lambda *args: None,
            setMotionControlMode=lambda mode, ec: events.append(mode),
            setOperateMode=lambda mode, ec: events.append(mode),
            setPowerState=refused,
            stopReceiveRobotState=lambda: None,
        )
    )
    assert bot._arm_motion(np.zeros(6), np.zeros(6)) is False
    bot.close()
    assert events == ["rt", "automatic", "nrt", "manual"]
