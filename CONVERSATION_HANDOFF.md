# Codex 会话交接文档

更新时间：2026-07-19（Asia/Shanghai）

## 会话定位

- Codex 任务名称：`机械臂控制与力传感采集系统：开发与标定记录`
- Codex 任务 ID：`019f5fa7-d2a8-72a3-8883-f53fba803e7d`
- 项目仓库：`https://github.com/617482966-lzh/robotic_arm_v1.git`
- 当前主分支：`main`
- Python 3.12 环境：`C:\Users\www61\anaconda3\envs\common`

在另一台电脑登录同一个 Codex/OpenAI 账号后，优先在任务列表中搜索上述任务名称。若任务未同步，打开克隆后的项目，将本文件作为新任务的首条上下文即可继续。

## 项目结构与边界

- `main.py`：程序入口、机械臂和传感器线程、20 Hz 试验采样、停止条件、XLSX 导出。
- `main_window_2.py` / `main_window_2.ui`：当前正式界面和交互逻辑。
- `main_window.py`：V2 界面复用的基础窗口、图表和传感器通用功能。
- `robot_client/`：正式机械臂 JSON 通信和控制实现。
- `sensor_2/`：已完善的传感器协议，原则上不修改。
- `demo/`：机械臂通信示例与使用教程，不作为主程序函数库。
- `picture/jlu.png`：窗口图标和右侧校徽。
- `requirements.txt`：部署依赖。
- `机械臂通信与速度标定记录.md`：协议、标定、功能和界面修改的完整技术记录。

## 已实现的主要功能

1. 机械臂 IP/端口连接、断开、使能、回零和急停。
2. 世界坐标 XYZ/Rx/Ry/Rz 和 J1-J6 实时读取，断开后停止刷新。
3. 世界坐标增量/全局运动、关节增量/全局运动，自动计算六轴 `ckStatus` 掩码。
4. 世界坐标物理线速度和 J1-J6 角速度标定，关节输入上限 20 deg/s。
5. `move_along_tool_x` 及探针贯入方向相关实机调试记录。
6. 贯入试验和剪切试验：20 Hz 采集、力/扭矩上限停止、实时曲线和 XLSX 保存。
7. P1-P5 关节角/世界坐标位置记忆，使用 `QSettings` 跨软件重启持久化。
8. F11 全屏、明暗主题切换、右下角退出按钮。
9. 吉林大学图标作为窗口图标，并在右侧栏居中显示。

## 关键控制约定

- `action51`：世界坐标运动前启用物理线速度；关节运动前使用 `isUse=0` 关闭物理速度。
- `action10`：执行世界坐标目标/增量运动时不再额外输入 speed，速度由 `action51` 控制。
- `action4`：关节自由路径运动使用标定后的 speed 百分比，并根据实际运动轴生成 `ckStatus`。
- 六轴机械臂只有 J1-J6，不存在 M6/M7。
- 达到贯入力或剪切扭矩上限时采用已验证的 `actionStop` 安全停止流程。

## 当前界面状态

- 默认暗色主题，可切换完整明亮主题；两张 Matplotlib 图表同步换色。
- 右下角有“明亮主题/暗色主题”和“退出”按钮。
- 末端位姿控制与关节角控制使用相同的三列、两行及紧凑速度行排布。
- 机械臂连接按钮为 100×32 px，状态区域紧随按钮。
- 试验保存路径默认使用系统桌面位置。
- 位置记忆的保存、调用、清除按钮尺寸一致。

## 数据与安全注意事项

- `data/cut_motor_20250528_212313_01.xlsx` 和 `data/pen_motor_20250528_212209_01.xlsx` 曾被误删，现已恢复并提交。
- 实机运动前确认示教器模式、工作空间、急停状态和人员安全。
- 在线标定允许范围曾约定为世界坐标 Z/向量小范围运动及关节受限运动；未经用户明确确认不要扩大运动范围。
- 不要使用 `git reset --hard` 覆盖未提交的实验数据或界面修改。

## 新电脑部署

```powershell
git clone https://github.com/617482966-lzh/robotic_arm_v1.git
cd robotic_arm_v1
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python main.py
```

若 Win10 不显示窗口，从终端运行 `python main.py` 查看错误；重点检查 64 位 Python、Microsoft Visual C++ 2015-2022 x64 运行库和 PySide6 Qt 平台插件。

## 继续会话的推荐提示词

> 请先完整阅读 `CONVERSATION_HANDOFF.md`、`机械臂通信与速度标定记录.md`、`main.py`、`main_window_2.py`、`main_window_2.ui` 和 `robot_client/`。这是上一台电脑延续的机械臂控制项目。保持 `sensor_2` 不变，`demo` 仅作示例，正式机械臂控制只放在 `robot_client`。先检查 Git 状态和当前程序，再继续我的下一项修改。
