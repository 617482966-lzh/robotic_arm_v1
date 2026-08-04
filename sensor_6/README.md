# 六维力传感器调用说明

本目录对应`pdf/六维力传感器.pdf`中的Modbus RTU轮询协议，不依赖
`sensor_2`，可由其他程序单独调用。

## 默认通信参数

- 端口：COM6
- 波特率：115200
- 数据位：8
- 校验位：NONE
- 停止位：1
- 从站地址：1
- 读功能码：04
- 起始地址：0x0000
- 寄存器数量：12

12个16位寄存器按大端IEEE-754单精度浮点解析为：

`Fx, Fy, Fz, Mx, My, Mz`

其中力的单位为N，力矩的单位为N·m。

## 推荐调用

```python
from sensor_6 import SensorCommunication, SensorController

with SensorCommunication("COM6") as communication:
    sensor = SensorController(communication)

    # 严格读取：失败时抛出异常，适合需要诊断原因的程序。
    wrench = sensor.read_wrench()
    print(wrench.fx, wrench.fy, wrench.fz)
    print(wrench.mx, wrench.my, wrench.mz)

    # 兼容sensor_2风格：失败时返回六个None。
    fx, fy, fz, mx, my, mz = sensor.monitor_6_sensor()
```

仅需元组时：

```python
values = sensor.read_wrench().as_tuple()
```

程序使用线程锁保护每次Modbus事务，同一通信对象可由采集线程与界面安全共享。

## 清零

```python
# 通道1~6依次对应Fx、Fy、Fz、Mx、My、Mz。
sensor.zero_channel(1)
sensor.zero_channel(6)

# 清零全部六个通道。
sensor.zero_all()
```

兼容`sensor_2`命名的`reset_channel(1)`和`reset_all()`作用相同。

清零使用Modbus功能码16，向参数地址`0x4604`写入大端Float。单通道写入
`1.0~6.0`，全部通道写入`255.0`（字节`43 7F 00 00`）。

通道对应关系：

| 通道 | 分量 | 写入Float |
|---:|---|---:|
| 1 | Fx | 1.0 |
| 2 | Fy | 2.0 |
| 3 | Fz | 3.0 |
| 4 | Mx | 4.0 |
| 5 | My | 5.0 |
| 6 | Mz | 6.0 |
| 全部 | Fx~Mz | 255.0 |

上述单通道和全部清零均已在COM6实机验证。清零会把当前载荷作为新的零点，
调用前应确保传感器处于希望定义为零载荷的稳定状态。
