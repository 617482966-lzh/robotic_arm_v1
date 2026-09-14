# 应用程序目录

每套可独立启动的程序放在一个单独子目录中。子目录内集中保存：

- `main.py`：程序入口与业务逻辑；
- `main_window*.py`：PySide6 界面控制代码；
- `main_window*.ui`：Qt Designer 界面文件；
- `picture/`：该程序使用的图标和图片。

机械臂与传感器通信代码继续由项目根目录统一维护：

- `robot_client/`
- `sensor_2/`
- `sensor_6/`

当前应用：

- `sensor_2_control_v2/`：原机械臂控制与二维力传感器数据采集界面。
- `sensor_6_control_v3/`：机械臂控制与六维力/力矩传感器数据采集界面。
- `sensor_6_control_v4/`：在 V3 基础上增加四组输入重置、紧凑布局和
  剪切启动时序修复的独立版本，配置保存为自己的 `sensor_6_control_v4.ini`。
