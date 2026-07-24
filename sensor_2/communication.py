"""力/扭矩传感器的 Modbus RTU 通信封装。"""

from __future__ import annotations

import threading
from typing import Iterable, List

import minimalmodbus
import serial


class SensorCommunicationError(IOError):
    """传感器串口或 Modbus 事务失败。"""


class SensorCommunication:
    """线程安全的 MinimalModbus 适配器。

    ``instrument`` 属性和原有读写方法均保留，避免影响现有主程序。
    默认使用115200-8-N-1、Modbus RTU、从站地址1。
    """

    # 保持原协议经过实机使用的200 ms容错；主程序连接时会主动握手，
    # 因此断线不会再被误判为连接成功。
    DEFAULT_TIMEOUT = 0.2

    def __init__(self, port, slave_address=1):
        self._lock = threading.RLock()
        self.instrument = minimalmodbus.Instrument(port, slave_address)
        self.setup_communication()

    def setup_communication(self):
        """设置115200-8-N-1、Modbus RTU通信参数。"""
        with self._lock:
            serial_port = self.instrument.serial
            serial_port.baudrate = 115200
            serial_port.bytesize = 8
            serial_port.parity = serial.PARITY_NONE
            serial_port.stopbits = 1
            serial_port.timeout = self.DEFAULT_TIMEOUT
            self.instrument.mode = minimalmodbus.MODE_RTU
            # 清除过期响应，避免一次超时污染后续20 Hz采样。
            self.instrument.clear_buffers_before_each_transaction = True

    def read_registers(self, address, count, functioncode=3) -> List[int]:
        """原子地读取多个保持寄存器。"""
        if int(address) < 0:
            raise ValueError("寄存器地址不能为负数")
        if int(count) <= 0:
            raise ValueError("寄存器数量必须大于0")
        try:
            with self._lock:
                return self.instrument.read_registers(
                    int(address), int(count), functioncode=int(functioncode)
                )
        except (minimalmodbus.ModbusException, serial.SerialException, OSError) as exc:
            raise SensorCommunicationError(
                f"读取寄存器0x{int(address):04X}失败: {exc}"
            ) from exc

    def write_registers(self, address, values: Iterable[int]):
        """原子地写入多个寄存器。"""
        if int(address) < 0:
            raise ValueError("寄存器地址不能为负数")
        register_values = [int(value) for value in values]
        if not register_values:
            raise ValueError("写入值不能为空")
        if any(value < 0 or value > 0xFFFF for value in register_values):
            raise ValueError("寄存器值必须在0至65535之间")
        try:
            with self._lock:
                return self.instrument.write_registers(int(address), register_values)
        except (minimalmodbus.ModbusException, serial.SerialException, OSError) as exc:
            raise SensorCommunicationError(
                f"写入寄存器0x{int(address):04X}失败: {exc}"
            ) from exc
