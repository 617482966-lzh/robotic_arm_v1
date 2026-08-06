# 机械臂控制与力传感器数据采集项目

当前主要程序是六维力传感器版本：

- 源码入口：`apps/sensor_6_control_v3/main.py`
- PyInstaller 配置：`apps/sensor_6_control_v3/sensor_6_control_v3.spec`
- EXE 发布目录：`release/sensor_6_control_v3/`

## 常用文档

- [EXE 打包与发布说明](EXE打包与发布说明.md)
- [项目文件夹说明](项目文件夹说明.md)
- [机械臂通信与速度标定记录](机械臂通信与速度标定记录.md)
- [跨电脑继续开发说明](CONVERSATION_HANDOFF.md)
- [V3 程序说明](apps/sensor_6_control_v3/README.md)
- [六维力传感器说明](sensor_6/README.md)

## 源码启动

在项目根目录运行：

```powershell
& "C:\Users\www61\anaconda3\envs\common\python.exe" "apps\sensor_6_control_v3\main.py"
```

程序启动只初始化界面并扫描串口，不会自动连接机械臂，也不会自动发送运动指令。

