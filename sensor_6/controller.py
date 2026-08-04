"""六维力/力矩数据读取与IEEE-754浮点解析。"""

from __future__ import annotations

import logging
import math
import struct
import time
from dataclasses import dataclass
from typing import Iterator, Optional, Sequence, Tuple

from .communication import SixAxisSensorCommunication


logger = logging.getLogger(__name__)


class SensorDataError(ValueError):
    """六维传感器响应长度或浮点数据不合法。"""


@dataclass(frozen=True, slots=True)
class Wrench6D:
    """一次六维力/力矩测量。

    Fx/Fy/Fz单位为N，Mx/My/Mz单位为N·m。
    """

    fx: float
    fy: float
    fz: float
    mx: float
    my: float
    mz: float

    def as_tuple(self) -> Tuple[float, float, float, float, float, float]:
        return self.fx, self.fy, self.fz, self.mx, self.my, self.mz

    def __iter__(self) -> Iterator[float]:
        return iter(self.as_tuple())


class SixAxisSensorController:
    """六维力传感器业务控制层。"""

    DATA_ADDRESS = 0x0000
    DATA_REGISTER_COUNT = 12
    CHANNEL_NAMES = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")
    ZERO_ADDRESS = 0x4604
    ZERO_ALL_VALUE = 255.0
    ERROR_LOG_INTERVAL = 5.0

    def __init__(self, communication: SixAxisSensorCommunication):
        self.comm = communication
        self.last_error: Optional[Exception] = None
        self._last_error_log_time = 0.0

    @staticmethod
    def _decode_float(high_word: int, low_word: int) -> float:
        """按说明书的高字节在前顺序解析一个大端Float。"""
        words = int(high_word), int(low_word)
        if any(word < 0 or word > 0xFFFF for word in words):
            raise SensorDataError(f"非法16位寄存器值: {words!r}")
        value = struct.unpack(">f", struct.pack(">HH", *words))[0]
        if not math.isfinite(value):
            raise SensorDataError(f"传感器返回非有限浮点数: {value!r}")
        return float(value)

    @classmethod
    def parse_sensor_data(cls, data: Sequence[int]) -> Wrench6D:
        """将12个输入寄存器解析为Fx/Fy/Fz/Mx/My/Mz。"""
        if data is None or len(data) != cls.DATA_REGISTER_COUNT:
            length = 0 if data is None else len(data)
            raise SensorDataError(
                f"期望{cls.DATA_REGISTER_COUNT}个寄存器，实际收到{length}个"
            )
        values = tuple(
            cls._decode_float(data[index], data[index + 1])
            for index in range(0, cls.DATA_REGISTER_COUNT, 2)
        )
        return Wrench6D(*values)

    def read_wrench(self) -> Wrench6D:
        """读取一次六维力/力矩；失败时抛出原始诊断异常。"""
        data = self.comm.read_input_registers(
            self.DATA_ADDRESS, self.DATA_REGISTER_COUNT
        )
        result = self.parse_sensor_data(data)
        self.last_error = None
        return result

    @staticmethod
    def _encode_float(value: float) -> Tuple[int, int]:
        """将Float编码为说明书使用的高字节/高寄存器在前格式。"""
        raw = struct.pack(">f", float(value))
        return struct.unpack(">HH", raw)

    def zero_channel(self, channel: int):
        """清零单个通道；1~6分别对应Fx/Fy/Fz/Mx/My/Mz。"""
        channel = int(channel)
        if not 1 <= channel <= 6:
            raise ValueError("清零通道必须在1~6范围内")
        return self.comm.write_registers(
            self.ZERO_ADDRESS, self._encode_float(float(channel))
        )

    def zero_all(self):
        """清零全部六个通道。"""
        return self.comm.write_registers(
            self.ZERO_ADDRESS, self._encode_float(self.ZERO_ALL_VALUE)
        )

    def reset_channel(self, channel: int):
        """兼容reset命名：清零指定的1~6通道。"""
        return self.zero_channel(channel)

    def reset_all(self):
        """兼容sensor_2命名：清零全部六个通道。"""
        return self.zero_all()

    def _record_read_error(self, exc: Exception) -> None:
        self.last_error = exc
        now = time.monotonic()
        if now - self._last_error_log_time >= self.ERROR_LOG_INTERVAL:
            logger.warning("读取六维力传感器失败: %s", exc)
            self._last_error_log_time = now

    def monitor_6_sensor(
        self,
    ) -> Tuple[
        Optional[float],
        Optional[float],
        Optional[float],
        Optional[float],
        Optional[float],
        Optional[float],
    ]:
        """兼容sensor_2风格读取；失败时返回六个None。"""
        try:
            return self.read_wrench().as_tuple()
        except Exception as exc:
            self._record_read_error(exc)
            return None, None, None, None, None, None


# 与sensor_2保持相似命名，方便调用方最小改动迁移。
SensorController = SixAxisSensorController
