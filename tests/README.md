# V4 无硬件回归检查

从项目根目录使用已安装运行依赖的 Python 3.12 运行：

```powershell
python -B -m unittest discover -s tests -v
```

- `test_v4_force_safety.py`：正负三轴阈值、采集先于 GUI、急停优先级、
  初次握手、开关重启、使能恢复及断开时保留待发送急停；
- `test_v4_window_controls.py`：XYZ 输入边界、默认保护开关、图表的隐藏、
  重入、重置与历史保留；界面测试使用临时 INI；
- `test_v4_wrench_plot.py`：六通道峰值、全点保留、局部重绘、坐标更新，
  以及图例/坐标刻度的像素回归。

测试使用模拟机械臂和传感器，不访问真实硬件。Qt 采用 offscreen 平台，
不会弹出控制界面；也不会改动应用目录中的位置记忆和试验参数。

绘图性能基准另行运行：

```powershell
python -B tools/benchmark_v4_wrench_plot.py --frames 30
```
