# 机械臂控制与二维力传感器采集界面 V2

本目录是原控制界面的完整归档。主程序、界面控制代码、Qt Designer
界面文件和程序图标均保存在一起。

## 启动

在本目录打开终端并运行：

```powershell
C:\Users\www61\anaconda3\envs\common\python.exe main.py
```

也可以从项目根目录运行：

```powershell
C:\Users\www61\anaconda3\envs\common\python.exe apps\sensor_2_control_v2\main.py
```

## 目录内容

- `main.py`：主程序入口、线程控制、试验控制和数据保存；
- `main_window_2.py`、`main_window_2.ui`：当前 V2 界面；
- `main_window.py`、`main_window.ui`：V2 界面复用的基础界面；
- `picture/`：窗口与界面使用的吉林大学图标。

本应用继续调用项目根目录中的 `robot_client/` 和 `sensor_2/`，因此通信
代码只维护一份。以后新增其他控制程序时，在 `apps/` 下新建独立目录，
并把该程序的入口、界面代码和 `.ui` 文件放在同一目录即可。
