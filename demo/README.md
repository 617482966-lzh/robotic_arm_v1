# HC1机械臂JSON通信示例使用教程

`demo` 只作为协议示例和独立调试程序，不被正式主程序调用。正式机械臂客户端位于 `robot_client`。

## 1. 运行环境

- 项目目录：`C:\Users\www61\OneDrive\JLU\pycharm_code\robotic_arm_v1`
- Python：`C:\Users\www61\anaconda3\envs\common\python.exe`
- 默认机械臂地址：`192.168.1.4:9760`

所有命令均在项目根目录的PowerShell中执行。

## 2. 运动前检查

示教器需要满足：

1. 机械臂处于自动模式。
2. 当前无报警。
3. 机械臂已经停止。
4. 远程指令列表已经清空。
5. 预计运动路径内没有人员、线缆或障碍物。

读取状态：

```powershell
C:\Users\www61\anaconda3\envs\common\python.exe demo\json_write.py read_status
```

正常状态应包含：

```text
curMode = 2
isMoving = 0
curAlarm = 0
```

读取当前世界坐标：

```powershell
C:\Users\www61\anaconda3\envs\common\python.exe demo\json_write.py read_pose
```

## 3. 沿末端姿态方向移动

正向移动10 mm，物理速度1 mm/s：

```powershell
C:\Users\www61\anaconda3\envs\common\python.exe demo\json_write.py tool_x 10 --speed 1
```

反向移动10 mm：

```powershell
C:\Users\www61\anaconda3\envs\common\python.exe demo\json_write.py tool_x -10 --speed 1
```

当前允许距离为 `-60～60 mm`，不能输入0。速度由action51设置，单位为mm/s，当前示例最低允许1 mm/s。

### 3.1 自由路径关节掩码

action4的 `ckStatus` 会根据实际发生变化的关节自动生成：

| 关节 | 位索引 | 掩码 |
|---|---:|---:|
| J1 | bit0 | `0x01` |
| J2 | bit1 | `0x02` |
| J3 | bit2 | `0x04` |
| J4 | bit3 | `0x08` |
| J5 | bit4 | `0x10` |
| J6 | bit5 | `0x20` |

例如只让J6增加5°：

```powershell
C:\Users\www61\anaconda3\envs\common\python.exe demo\json_write.py free_by --dj6 5 --speed 10
```

程序读取当前J1～J6，只改变J6目标，并自动发送 `ckStatus=0x20`。J6是第六轴但使用bit5，不能写成 `0x40`。多轴同时变化时使用按位或，例如J1+J6为 `0x01 | 0x20 = 0x21`。六轴全动为 `0x3F`。

参数说明：

| 参数 | 含义 |
|---|---|
| `tool_x` | 保持末端姿态，沿当前姿态定义的世界XZ方向运动 |
| `distance_mm` | 三维空间距离；正值沿正向，负值沿反向 |
| `--speed` | action51物理线速度，单位mm/s |

## 4. 方向计算

本机械臂的运动方向只由世界姿态中的俯仰角 `Ry`（协议字段V、`world-4`）决定：

```text
ux = cos(Ry)
uy = 0
uz = -sin(Ry)
```

该向量已经归一化。输入空间距离 `L` 后：

```text
ΔX = L × ux
ΔY = 0
ΔZ = L × uz
```

例如 `Ry=45°`、`L=60 mm`：

```text
ΔX = +42.426 mm
ΔY =   0.000 mm
ΔZ = -42.426 mm
sqrt(ΔX²+ΔY²+ΔZ²) = 60 mm
```

U、V、W目标值保持为起始值。action10使用 `ckStatus=0x07`，只启用XYZ轴，UVW姿态轴不参与控制器重新规划。

## 5. 通信指令关系

一次方向运动发送：

1. action51：`isUse=1`，设置物理速度mm/s。
2. action10：发送绝对目标XYZ；不携带speed字段；`ckStatus=0x07`。

本项目机械臂为六轴，所有运动报文只发送 `m0～m5`。自由路径中对应J1～J6，世界位姿中对应X、Y、Z、U、V、W；不发送 `m6`、`m7`，action17也不发送 `m6_p`、`m7_p`。

当前HC1固件可能回复：

```text
cmdReply: ["AddRCC", "err"]
```

实测该组合仍会正常执行，因此不能只凭这个字段判断运动失败；应继续读取 `isMoving`、世界坐标、报警和最终位置。

## 6. Python调用

也可以直接调用类方法：

```python
from demo.json_write import HC1JsonRobot

robot = HC1JsonRobot("192.168.1.4", 9760)
try:
    robot.connect()
    state = robot.read_status_pose()
    if not state or state["curMode"] != 2:
        raise RuntimeError("机械臂不在自动模式")
    if state["isMoving"] or state["curAlarm"]:
        raise RuntimeError(f"状态不允许运动: {state}")

    result = robot.move_along_tool_x(
        distance_mm=10,
        speed_mm_s=1,
    )
    print(result)
finally:
    robot.disconnect()
```

返回字典包含起点、单位方向、ΔXYZ、目标位姿、距离、速度和控制器回复。

## 7. 停止与异常处理

停止按键命令：

```powershell
C:\Users\www61\anaconda3\envs\common\python.exe demo\json_write.py stop
```

注意：当前CLI的 `stop` 使用actionStop，会进入停止模式并可能产生报警。实机测试中 `stopButton` 对远程路径不一定能立即停止，因此运动过程中必须有人守在示教器旁。

出现以下任一情况时不要继续发送运动：

- `curAlarm` 不为0。
- `isMoving` 长时间不归零。
- XYZ方向与计算结果不一致。
- 姿态角发生明显变化。
- TCP连接被控制器重置。

## 8. 已验证结果

修正方向后的±60 mm、1 mm/s在线测试结果：

| 指令 | 实际距离 | 目标误差 | 最大直线偏差 | 最大姿态偏差 |
|---|---:|---:|---:|---:|
| +60 mm | 60.001014 mm | 0.001412 mm | 0.004359 mm | 0.001156° |
| -60 mm | 60.000323 mm | 0.002775 mm | 0.004429 mm | 0.001018° |

正负往返后距初始点约0.00420 mm，结束时自动、静止、无报警、远程列表为0。
