from enum import Enum
from types import SimpleNamespace

import pytest


class Operation(Enum):
    idle = 0
    moving = 9
    unknown = -1


class Power(Enum):
    on = 0
    off = 1


class Mode(Enum):
    manual = 0
    automatic = 1


class Box:
    def __init__(self):
        self.value = None

    def content(self):
        return self.value


class FakeRobot:
    def __init__(self):
        self.calls = []
        self.position = [0.0] * 6
        self.power = Power.on
        self.mode = Mode.automatic
        self.operation = Operation.idle
        self.limit_enabled = True
        self.limit_bounds = [[-1.0, 1.0] for _ in range(6)]
        self.failure = None
        self.axes = 6
        self.reach_target = True

    def connectToRobot(self, ip, local_ip):
        self.calls.append("connectToRobot")

    def __getattr__(self, name):
        def method(*args):
            self.calls.append(name)
            ec = args[-1]
            ec.update(ec=42 if self.failure == name else 0, message="test error")
            if name == "robotInfo":
                return SimpleNamespace(
                    id="test",
                    version="3.2.1",
                    type="CR7",
                    joint_num=self.axes,
                    mac="test-mac",
                )
            if name == "powerState":
                return self.power
            if name == "operateMode":
                return self.mode
            if name == "operationState":
                return self.operation
            if name == "jointPos":
                return self.position.copy()
            if name == "posture":
                return [0.0] * 6
            if name == "getRobotCfg_DHparam":
                return [0.0] * 24
            if name == "getSoftLimit":
                args[0].value = self.limit_bounds
                return self.limit_enabled
            if name == "moveAppend":
                self.target = args[0][0].target
                args[1].value = "test-command"
            if name == "moveStart" and self.reach_target:
                self.position = self.target.copy()

        return method


@pytest.fixture
def fake():
    sdk = SimpleNamespace(
        OperationState=Operation,
        PowerState=Power,
        OperateMode=Mode,
        CoordinateType=SimpleNamespace(flangeInBase=1, endInRef=2),
        MotionControlMode=SimpleNamespace(NrtCommandMode=1),
        BaseRobot=SimpleNamespace(sdkVersion=lambda: "0.7.1"),
        PyTypeVectorArrayDouble2=Box,
        PyString=Box,
        MoveAbsJCommand=lambda target, speed, zone: SimpleNamespace(
            target=target, speed=speed, zone=zone
        ),
    )
    return sdk, FakeRobot()
