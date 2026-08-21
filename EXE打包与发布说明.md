# EXE 打包与发布说明

本文档用于在 Windows 10/11 x64 上重新生成
`release/sensor_6_control_v3/sensor_6_control_v3.exe`。

## 1. 当前打包目标

打包入口为：

```text
apps/sensor_6_control_v3/main.py
```

打包配置为：

```text
apps/sensor_6_control_v3/sensor_6_control_v3.spec
```

该配置会自动收集以下项目代码：

- `apps/sensor_6_control_v3/main.py`
- `apps/sensor_6_control_v3/main_window_3.py`
- `robot_client/robot_control.py`
- `robot_client/hc1_json.py`
- `sensor_6/communication.py`
- `sensor_6/controller.py`

同时打包以下界面资源：

- `apps/sensor_6_control_v3/main_window_3.ui`
- `apps/sensor_6_control_v3/picture/jlu.png`
- `apps/sensor_6_control_v3/picture/jlu.ico`

`demo/`、`sensor_2/`、标定脚本、PDF 和历史试验数据不会作为 V3
程序入口运行。

## 2. 推荐构建环境

- Windows 10 或 Windows 11，64 位
- Python 3.12，64 位
- 推荐使用 Anaconda/Miniconda
- 当前验证环境：
  `C:\Users\www61\anaconda3\envs\common\python.exe`

不要使用 32 位 Python 生成 64 位程序。打包时使用的 Python
架构决定 EXE 架构。

## 3. 创建独立打包环境

首次在新电脑打包时，打开 Anaconda Prompt 或 PowerShell：

```powershell
conda create -n robotic_arm_pack python=3.12 -y
conda activate robotic_arm_pack
```

进入项目根目录：

```powershell
Set-Location "C:\Users\www61\OneDrive\JLU\pycharm_code\robotic_arm_v1"
```

### 日常构建安装

安装兼容版本的运行库和固定版本的 PyInstaller：

```powershell
python -m pip install --upgrade pip
python -m pip install -r requirements-build.txt
```

### 严格复现当前发布环境

需要尽可能复现 2026-08-06 的构建结果时，使用锁定文件：

```powershell
python -m pip install --upgrade pip
python -m pip install -r requirements-packaging-lock.txt
```

锁定文件只包含本项目打包链路使用的依赖，不包含当前 Conda
环境中的其他无关科学计算包。

## 4. 执行打包

必须在项目根目录执行。使用当前已激活环境时：

```powershell
python -m PyInstaller `
  --clean `
  --noconfirm `
  --distpath "release" `
  --workpath "$env:TEMP\pyinstaller_sensor_6_control_v3_work" `
  "apps\sensor_6_control_v3\sensor_6_control_v3.spec"
```

使用本项目当前指定解释器时：

```powershell
& "C:\Users\www61\anaconda3\envs\common\python.exe" -m PyInstaller `
  --clean `
  --noconfirm `
  --distpath "release" `
  --workpath "$env:TEMP\pyinstaller_sensor_6_control_v3_work" `
  "apps\sensor_6_control_v3\sensor_6_control_v3.spec"
```

参数说明：

- `--clean`：清除 PyInstaller 分析缓存，避免旧模块残留。
- `--noconfirm`：允许覆盖已有的同名发布目录。
- `--distpath release`：把正式成品放在项目根目录的 `release/`。
- `--workpath`：把临时构建文件放在系统临时目录，不污染源码。
- `.spec`：统一控制入口、图标、UI 文件、隐藏模块和 Conda DLL。

看到以下内容表示构建阶段完成：

```text
Build complete! The results are available in: ...\release
```

这只表示文件生成成功，仍然需要进行启动检查。

## 5. 成品位置和发布方式

生成结果：

```text
release/
└─ sensor_6_control_v3/
   ├─ sensor_6_control_v3.exe
   ├─ sensor_6_control_v3.ini  （首次运行后生成）
   ├─ 使用说明.txt
   └─ _internal/
```

发布到其他电脑时必须复制整个 `sensor_6_control_v3` 文件夹。
`_internal` 中包含 PySide6、Matplotlib、Python、串口模块和底层 DLL，
不能只复制 EXE，也不能修改 `_internal` 的相对位置。

V3 的 P1～P5 位置记忆和九个试验参数保存在 EXE 同级的
`sensor_6_control_v3.ini`。迁移到其他电脑时应一起复制该文件。
PyInstaller 的 `--noconfirm` 会重建同名发布目录，因此再次打包前应先
备份已有 INI，打包完成后再放回 EXE 同级目录。

目标电脑不需要安装 Python，但仍需要：

- Windows 10/11 x64；
- 可用的机械臂网络；
- 正确的 USB 转串口驱动；
- 六维力传感器对应 COM 端口。

## 6. 构建后检查

### 普通界面检查

双击：

```text
release/sensor_6_control_v3/sensor_6_control_v3.exe
```

确认：

1. 软件图标和 JLU 图片正常显示；
2. 主界面、两个堆叠页面和图表正常显示；
3. 不出现 `Unhandled exception in script`；
4. 串口列表能够刷新；
5. 不连接硬件时程序仍可正常关闭。

### 离线启动检查

以下命令使用 Qt 无屏幕模式，不连接机械臂，不发送运动命令：

```powershell
$env:ROBOTIC_ARM_GUI_CHILD = "1"
$env:QT_QPA_PLATFORM = "offscreen"
$env:QT_OPENGL = "software"
$process = Start-Process `
  -FilePath ".\release\sensor_6_control_v3\sensor_6_control_v3.exe" `
  -PassThru
Start-Sleep -Seconds 10
if ($process.HasExited) {
    Write-Error "EXE 启动失败，退出码：$($process.ExitCode)"
} else {
    Write-Host "EXE 启动检查通过"
    Stop-Process -Id $process.Id
}
Remove-Item Env:ROBOTIC_ARM_GUI_CHILD
Remove-Item Env:QT_QPA_PLATFORM
Remove-Item Env:QT_OPENGL
```

离线检查只证明模块、DLL、UI 和主窗口能够载入，不能代替机械臂及
传感器的现场连接测试。

## 7. 常见问题

### 出现 `Unhandled exception in script`

优先检查：

1. 是否使用项目中的最新 `.spec` 文件打包；
2. 是否复制了完整发布文件夹；
3. `_internal` 中是否存在 `ffi-8.dll`、`libcrypto-3-x64.dll`、
   `libssl-3-x64.dll` 等 Conda DLL；
4. 打包环境是否为 Python 3.12 x64。

需要查看完整异常时，可暂时把 `.spec` 中：

```python
console=False
```

改为：

```python
console=True
```

重新打包并从 PowerShell 启动 EXE。问题定位后应恢复
`console=False`，再生成正式版。

### Win10 上双击无界面

程序已经包含硬件 OpenGL 失败后切换软件渲染的逻辑。仍无法启动时：

```powershell
$env:QT_OPENGL = "software"
.\release\sensor_6_control_v3\sensor_6_control_v3.exe
```

如果这样可以启动，通常是显卡驱动或远程桌面环境的 OpenGL
兼容问题。

### 修改代码后 EXE 没变化

Python 源码修改不会自动进入旧 EXE。每次修改下列内容后都需要重新打包：

- `apps/sensor_6_control_v3/`
- `robot_client/`
- `sensor_6/`
- UI、图标或依赖版本

建议始终保留 `--clean` 参数。

## 8. 维护打包配置

新增 Python 模块并被正常 `import` 时，PyInstaller 通常会自动收集。
如果代码使用运行时动态导入，需要把模块加入 `.spec` 的
`hiddenimports`。

新增 UI、图片或配置文件时，需要把文件加入 `.spec` 的 `datas`。

新增 Conda 底层 DLL 时，需要把 DLL 名称加入 `.spec` 的
`required_dlls`。完成修改后必须重新执行构建和启动检查。

## 9. 2026-08-14 当前正式构建

- 入口：`apps/sensor_6_control_v3/main.py`；
- Python：3.12.12；PyInstaller：6.21.0；
- EXE：`release/sensor_6_control_v3/sensor_6_control_v3.exe`；
- 大小：13,078,825 字节；
- SHA256：`6F60DA17E3E2EF1D00F7EBCBEB72A459F15AD83017EEE2EEF431C0B3D0C89E1C`；
- 已验证发布 UI 与源码一致、B～G 铲挖点存在，并通过窗口启动测试。

若 OneDrive 给旧 `_internal` 目录附加只读重解析属性，PyInstaller 的
`--noconfirm` 可能在删除旧目录时报 `WinError 5`。此时先输出到新的临时
`distpath`，再用 PowerShell 将完整 EXE 和 `_internal` 覆盖到正式目录；
覆盖前后都应备份并恢复 `sensor_6_control_v3.ini`。
