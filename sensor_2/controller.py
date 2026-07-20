"""力/扭矩传感器业务控制与寄存器解析。"""

from __future__ import annotations

import logging
import struct
import time
from typing import Optional, Sequence, Tuple

from .communication import SensorCommunication


logger = logging.getLogger(__name__)


class SensorDataError(ValueError):
    """传感器响应长度、字值或量程不合法。"""


class SensorController:
    DATA_ADDRESS = 0x01C2
    DATA_REGISTER_COUNT = 4
    SCALE = 0.005
    FORCE_RAW_RANGE = (-10000, 10000)
    TORQUE_RAW_RANGE = (-1000, 1000)
    ERROR_LOG_INTERVAL = 5.0

    def __init__(self, communication: SensorCommunication):
        self.comm = communication
        self.last_error: Optional[Exception] = None
        self._last_error_log_time = 0.0

    @staticmethod
    def _decode_i32(high_word: int, low_word: int) -> int:
        """将两个大端16位寄存器解析为有符号32位整数。"""
        words = (int(high_word), int(low_word))
        if any(word < 0 or word > 0xFFFF for word in words):
            raise SensorDataError(f"非法16位寄存器值: {words!r}")
        return struct.unpack(">i", struct.pack(">HH", *words))[0]

    @classmethod
    def _parse_sensor_data(cls, data: Sequence[int]) -> Tuple[float, float]:
        if data is None or len(data) != cls.DATA_REGISTER_COUNT:
            length = 0 if data is None else len(data)
            raise SensorDataError(
                f"期望{cls.DATA_REGISTER_COUNT}个寄存器，实际收到{length}个"
            )

        pulling_pressure_raw = cls._decode_i32(data[0], data[1])
        torque_raw = cls._decode_i32(data[2], data[3])
        if not cls.FORCE_RAW_RANGE[0] <= pulling_pressure_raw <= cls.FORCE_RAW_RANGE[1]:
            raise SensorDataError(f"拉压力原始值越界: {pulling_pressure_raw}")
        if not cls.TORQUE_RAW_RANGE[0] <= torque_raw <= cls.TORQUE_RAW_RANGE[1]:
            raise SensorDataError(f"扭矩原始值越界: {torque_raw}")
        return (
            round(pulling_pressure_raw * cls.SCALE, 2),
            round(torque_raw * cls.SCALE, 2),
        )

    def _record_read_error(self, exc: Exception) -> None:
        self.last_error = exc
        now = time.monotonic()
        # 断线时20 Hz循环不会再持续刷屏；仍保留周期性诊断信息。
        if now - self._last_error_log_time >= self.ERROR_LOG_INTERVAL:
            logger.warning("读取传感器数据失败: %s", exc)
            self._last_error_log_time = now

    def monitor_2_sensor(self):
        """读取拉压力和扭矩；读取失败时兼容地返回 ``(None, None)``。"""
        try:
            data = self.comm.read_registers(
                self.DATA_ADDRESS, self.DATA_REGISTER_COUNT, functioncode=3
            )
            result = self._parse_sensor_data(data)
            self.last_error = None
            return result
        except Exception as exc:
            self._record_read_error(exc)
            return None, None

    def reset_pulling_pressure(self):
        """重置拉压力；通信失败时将异常交给调用方显示。"""
        return self.comm.write_registers(0x005E, [0x0001])

    def reset_torque(self):
        """重置扭矩；通信失败时将异常交给调用方显示。"""
        return self.comm.write_registers(0x0252, [0x0002])

    def reset_all(self):
        """重置全部传感器数据；通信失败时将异常交给调用方显示。"""
        return self.comm.write_registers(0x005E, [0x00FF])
