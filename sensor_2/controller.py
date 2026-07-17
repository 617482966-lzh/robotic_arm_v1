import struct
from .communication import SensorCommunication

class SensorController:
    def __init__(self, communication):
        self.comm = communication

    def monitor_2_sensor(self):
        """监控传感器数据"""
        try:
            # 读取寄存器数据
            data = self.comm.read_registers(0x01C2, 4, functioncode=3)
            # 解析前两个寄存器为32位有符号整数，并乘以分度值0.005
            pulling_pressure_raw = struct.unpack('>i', struct.pack('>HH', data[0], data[1]))[0]
            # 解析后两个寄存器为32位有符号整数，并乘以分度值0.005
            torque_raw = struct.unpack('>i', struct.pack('>HH', data[2], data[3]))[0]
            # 校验数据范围（示例范围：拉压力0-100，扭矩0-10）
            if -10000 <= pulling_pressure_raw <= 10000 and -1000 <= torque_raw <= 1000:
                pulling_pressure = round(pulling_pressure_raw * 0.005, 2)
                torque = round(torque_raw * 0.005, 2)
                # print(f"拉压力: {pulling_pressure}, 扭矩: {torque}")
                return pulling_pressure, torque
            else:
                print(f"数据异常: 拉压力原始值={pulling_pressure_raw}, 扭矩原始值={torque_raw}")
                return None, None
        except Exception as e:
            print(f"无法监控传感器数据: {str(e)}")
            return None, None

    # def monitor_2_sensor(self):
    #     """监控传感器数据"""
    #     try:
    #         # 读取寄存器数据
    #         data = self.comm.read_registers(0x01C2, 4, functioncode=3)
    #         # 解析前两个寄存器为32位有符号整数（补码）
    #         pulling_pressure = struct.unpack('>i', struct.pack('>HH', data[0], data[1]))[0]
    #         # 解析后两个寄存器为32位有符号整数（补码）
    #         torque = struct.unpack('>i', struct.pack('>HH', data[2], data[3]))[0]
    #         return round(pulling_pressure, 2), round(torque, 2)
    #     except Exception as e:
    #         print(f"无法监控传感器数据: {str(e)}")
    #         return None, None

    def reset_pulling_pressure(self):
        """重置拉压力"""
        try:
            self.comm.write_registers(0x005E, [0x0001])
            print("拉压力已重置")
        except Exception as e:
            print(f"无法重置拉压力: {str(e)}")

    def reset_torque(self):
        """重置扭矩"""
        try:
            self.comm.write_registers(0x0252, [0x0002])
            print("扭矩已重置")
        except Exception as e:
            print(f"无法重置扭矩: {str(e)}")

    def reset_all(self):
        """重置所有数据"""
        try:
            self.comm.write_registers(0x005E, [0x00FF])
            print("所有数据已重置")
        except Exception as e:
            print(f"无法重置所有数据: {str(e)}") 