import minimalmodbus
import serial

class SensorCommunication:
    def __init__(self, port, slave_address=1):
        self.instrument = minimalmodbus.Instrument(port, slave_address)
        self.setup_communication()
        
    def setup_communication(self):
        """设置通信参数"""
        self.instrument.serial.baudrate = 57600  ## 波特率
        self.instrument.serial.bytesize = 8   # 数据位
        self.instrument.serial.parity = serial.PARITY_NONE     # 校验位
        self.instrument.serial.stopbits = 1   # 停止位
        self.instrument.serial.timeout = 0.2         # 超时时间
        self.instrument.mode = minimalmodbus.MODE_RTU       # 使用RTU模式
        self.instrument.clear_buffers_before_each_transaction = True         # 每次发送前清空缓冲区
        
    def read_registers(self, address, count, functioncode=3):
        """读取多个寄存器"""
        return self.instrument.read_registers(address, count, functioncode=functioncode)

    def write_registers(self, address, values):
        """写入多个寄存器"""
        self.instrument.write_registers(address, values)  # 写入多个寄存器
