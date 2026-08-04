"""六维力传感器的Modbus RTU通信封装。"""

from __future__ import annotations

import threading
from typing import Iterable, List

import minimalmodbus
import serial


class SensorCommunicationError(IOError):
    """六维力传感器串口或Modbus事务失败。"""


class SixAxisSensorCommunication:
    """线程安全的六维力传感器通信适配器。

    默认配置来自《六维力传感器》说明书及现场参数：
    COM6、115200、8-N-1、Modbus RTU、从站地址1、读功能码04。
    """

    DEFAULT_PORT = "COM6"
    DEFAULT_SLAVE_ADDRESS = 1
    DEFAULT_BAUDRATE = 115200
    DEFAULT_TIMEOUT = 0.2

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        slave_address: int = DEFAULT_SLAVE_ADDRESS,
    ):
        self._lock = threading.RLock()
        self.instrument = minimalmodbus.Instrument(
            str(port), int(slave_address), mode=minimalmodbus.MODE_RTU
        )
        self.setup_communication()

    def setup_communication(self) -> None:
        """设置115200-8-N-1和Modbus RTU。"""
        with self._lock:
            serial_port = self.instrument.serial
            serial_port.baudrate = self.DEFAULT_BAUDRATE
            serial_port.bytesize = 8
            serial_port.parity = serial.PARITY_NONE
            serial_port.stopbits = 1
            serial_port.timeout = self.DEFAULT_TIMEOUT
            self.instrument.mode = minimalmodbus.MODE_RTU
            self.instrument.clear_buffers_before_each_transaction = True

    def read_input_registers(self, address: int, count: int) -> List[int]:
        """使用功能码04读取输入寄存器。"""
        address = int(address)
        count = int(count)
        if address < 0:
            raise ValueError("寄存器地址不能为负数")
        if count <= 0:
            raise ValueError("寄存器数量必须大于0")
        try:
            with self._lock:
                return self.instrument.read_registers(
                    address, count, functioncode=4
                )
        except (minimalmodbus.ModbusException, serial.SerialException, OSError) as exc:
            raise SensorCommunicationError(
                f"读取输入寄存器0x{address:04X}失败: {exc}"
            ) from exc

    def write_registers(self, address: int, values: Iterable[int]):
        """使用功能码16写入多个保持寄存器。"""
        address = int(address)
        register_values = [int(value) for value in values]
        if address < 0:
            raise ValueError("寄存器地址不能为负数")
        if not register_values:
            raise ValueError("写入值不能为空")
        if any(value < 0 or value > 0xFFFF for value in register_values):
            raise ValueError("寄存器值必须在0至65535之间")
        try:
            with self._lock:
                return self.instrument.write_registers(address, register_values)
        except (minimalmodbus.ModbusException, serial.SerialException, OSError) as exc:
            raise SensorCommunicationError(
                f"写入寄存器0x{address:04X}失败: {exc}"
            ) from exc

    def close(self) -> None:
        """释放串口。重复调用是安全的。"""
        with self._lock:
            serial_port = self.instrument.serial
            if serial_port and serial_port.is_open:
                serial_port.close()

    def __enter__(self) -> "SixAxisSensorCommunication":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


# 与sensor_2保持相似命名，方便调用方最小改动迁移。
SensorCommunication = SixAxisSensorCommunication
