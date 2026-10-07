"""Dynamixel Protocol 2 read-only transport for the six-axis leader."""

import math

# dynamixel_sdk 必须用 python/src（DynamixelSDK/ros 是 ROS ament 结构，无顶层 __init__.py，
# 会被当成命名空间包加载，导致 group_sync_read 等子模块全部找不到）
from dynamixel_sdk.group_sync_read import GroupSyncRead
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.robotis_def import COMM_SUCCESS

# --- 控制表地址 ---
ADDR_MODEL_NUMBER = 0
LEN_MODEL_NUMBER = 2
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4

TICKS_PER_PI = 2048.0  # XL330/XC330 系列 4096 ticks/rev

DEFAULT_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0"
DEFAULT_BAUDRATE = 57600

JOINT_IDS = (1, 2, 3, 4, 5, 6)
# 与 cr7_gello_package/.../gello_agent.py 中 FTB4C7PQ 条目一致
JOINT_OFFSETS = (
    3 * math.pi / 2,
    2 * math.pi / 2,
    2 * math.pi / 2,
    2 * math.pi / 2,
    2 * math.pi / 2,
    -2 * math.pi / 2,
)
JOINT_SIGNS = (1, 1, 1, 1, 1, 1)

DEFAULT_GRIPPER_ID = 7


def signed32(v):
    """32 位无符号 → 有符号"""
    return v - 0x100000000 if v > 0x7FFFFFFF else v


def ticks_to_rad(t):
    return t / TICKS_PER_PI * math.pi


class TeachArm:
    """小臂只读句柄：同步读 7 个舵机的 present position，读 torque 状态。"""

    def __init__(self, port, baudrate, ids, gripper_id):
        self._ids = tuple(ids) + ((gripper_id,) if gripper_id is not None else ())
        self.port = port
        self.baudrate = baudrate
        self._ph = PortHandler(port)
        self._pk = PacketHandler(2.0)
        if not self._ph.openPort():
            raise RuntimeError(
                f"打不开串口 {port}（检查是否被占用、是否有 dialout 权限）"
            )
        if not self._ph.setBaudRate(baudrate):
            raise RuntimeError(f"设置波特率 {baudrate} 失败")
        self._gsr = GroupSyncRead(
            self._ph, self._pk, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
        )
        for i in self._ids:
            if not self._gsr.addParam(i):
                raise RuntimeError(f"GroupSyncRead 添加 ID {i} 失败")
        self._fail_streak = 0

    def describe(self):
        """启动时读一次各 ID 的型号 / 固件 / torque 状态（全部是读操作）。"""
        rows = []
        for i in self._ids:
            model, r, _ = self._pk.read2ByteTxRx(self._ph, i, ADDR_MODEL_NUMBER)
            fw, _, _ = self._pk.read1ByteTxRx(self._ph, i, 6)
            tq, _, _ = self._pk.read1ByteTxRx(self._ph, i, ADDR_TORQUE_ENABLE)
            rows.append(
                {
                    "id": i,
                    "online": r == COMM_SUCCESS,
                    "model": model if r == COMM_SUCCESS else None,
                    "fw": fw if r == COMM_SUCCESS else None,
                    "torque": tq if r == COMM_SUCCESS else None,
                }
            )
        return rows

    def read_torque(self):
        """读各 ID 的 torque enable（只读，用于显示"力矩:on/off"）。"""
        out = {}
        for i in self._ids:
            v, r, _ = self._pk.read1ByteTxRx(self._ph, i, ADDR_TORQUE_ENABLE)
            out[i] = bool(v) if r == COMM_SUCCESS else None
        return out

    def read_ticks(self):
        """同步读全部 ID 的 present position。返回 {id: ticks or None}。"""
        r = self._gsr.txRxPacket()
        if r != COMM_SUCCESS:
            self._fail_streak += 1
            return None
        self._fail_streak = 0
        out = {}
        for i in self._ids:
            if self._gsr.isAvailable(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                out[i] = signed32(
                    self._gsr.getData(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                )
            else:
                out[i] = None
        return out

    def close(self):
        try:
            self._ph.closePort()
        except Exception:
            pass
