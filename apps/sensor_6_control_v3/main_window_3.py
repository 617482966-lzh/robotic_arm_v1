# -*- coding: utf-8 -*-
"""第三版机械臂与六维力传感器控制界面。

该模块独立实现 V3 图表、试验、位置记忆、连接状态和末端/关节控制，
不再依赖旧版 :mod:`main_window`。界面层只采集参数并发出信号，
不直接访问机械臂 socket。
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime

from PySide6.QtCore import QSettings, QStandardPaths, Qt, Signal, Slot
from PySide6.QtGui import QIcon, QKeySequence, QPixmap, QShortcut
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QApplication, QFileDialog, QMainWindow, QWidget

import matplotlib

matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

matplotlib.rcParams["font.sans-serif"] = [
    "Microsoft YaHei",
    "SimHei",
    "DejaVu Sans",
]
matplotlib.rcParams["axes.unicode_minus"] = False


RESOURCE_DIR = os.path.abspath(
    getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
)
APPLICATION_DIR = os.path.abspath(
    os.path.dirname(sys.executable)
    if getattr(sys, "frozen", False)
    else os.path.dirname(os.path.abspath(__file__))
)
SETTINGS_PATH = os.path.join(APPLICATION_DIR, "sensor_6_control_v3.ini")
UI_PATH = os.path.join(RESOURCE_DIR, "main_window_3.ui")
LOGO_PATH = os.path.join(
    RESOURCE_DIR,
    "picture",
    "jlu.png",
)
WINDOWS_ICON_PATH = os.path.join(
    RESOURCE_DIR,
    "picture",
    "jlu.ico",
)
ICON_PATH = WINDOWS_ICON_PATH if os.path.isfile(WINDOWS_ICON_PATH) else LOGO_PATH

WORLD_AXES = ("X", "Y", "Z", "Rx", "Ry", "Rz")
JOINT_AXES = ("J1", "J2", "J3", "J4", "J5", "J6")
SHOVEL_POINTS = ("B", "C", "D", "E", "F", "G")
TEST_PARAMETER_WIDGETS = (
    "testDispSpeed",
    "testDispDist",
    "testForceMax",
    "testAngSpeed",
    "testAngDist",
    "testTorqueMax",
    "testShovelSpeed",
) + tuple(
    f"testShovel{point}_{axis}"
    for point in SHOVEL_POINTS
    for axis in ("X", "Y", "Z")
)


@dataclass(frozen=True, slots=True)
class CartesianMoveRequest:
    """末端世界坐标运动请求。

    ``Rx/Ry/Rz`` 对应机器人 JSON 协议中的 ``U/V/W``。
    ``speed_mm_s`` 保留 UI 的物理速度语义，具体协议转换由机械臂工作线程完成。
    """

    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float
    speed_mm_s: float

    @property
    def pose(self) -> tuple[float, float, float, float, float, float]:
        return self.x, self.y, self.z, self.rx, self.ry, self.rz


@dataclass(frozen=True, slots=True)
class JointMoveRequest:
    """六关节运动请求，角度单位 deg，速度单位 deg/s。"""

    j1: float
    j2: float
    j3: float
    j4: float
    j5: float
    j6: float
    speed_deg_s: float

    @property
    def joints(self) -> tuple[float, float, float, float, float, float]:
        return self.j1, self.j2, self.j3, self.j4, self.j5, self.j6


@dataclass(frozen=True, slots=True)
class PositionMemory:
    coordinate_type: str
    values: tuple[float, float, float, float, float, float]


DARK_STYLE = """
QMainWindow { background-color: #1c1c2e; }
QWidget { background-color: #1c1c2e; color: #c8ccd4; font-size: 13px; }
QGroupBox { color: #d0d4dc; border: 1px solid #3a3a52; border-radius: 5px; margin-top: 16px; padding-top: 18px; font-weight: bold; font-size: 13px; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 8px; color: #14a3a8; }
QLabel { color: #c8ccd4; background: transparent; }
QPushButton { background-color: #2d2d46; color: #c8ccd4; border: 1px solid #3a3a52; border-radius: 4px; padding: 5px 14px; min-height: 26px; font-size: 13px; }
QPushButton:hover { background-color: #3a3a5a; border-color: #14a3a8; }
QPushButton:pressed { background-color: #14a3a8; color: #ffffff; }
QPushButton:disabled { background-color: #1e1e32; color: #5a5a6e; border-color: #2a2a40; }
QPushButton#robotBtnConnect, QPushButton#sensorBtnConnect { background-color: #1a6b3c; border-color: #28a745; }
QPushButton#robotBtnConnect:hover, QPushButton#sensorBtnConnect:hover { background-color: #218838; }
QPushButton#robotBtnDisconnect, QPushButton#sensorBtnDisconnect { background-color: #6b2a2a; border-color: #dc3545; }
QPushButton#robotBtnDisconnect:hover, QPushButton#sensorBtnDisconnect:hover { background-color: #8b2020; }
QPushButton#robotBtnStop { background-color: #8b2020; border-color: #dc3545; font-weight: bold; min-height: 36px; font-size: 15px; }
QPushButton#robotBtnStop:hover { background-color: #a02828; }
QPushButton#sensorBtnZeroAll { background-color: #6b5a1a; border-color: #ffc107; font-weight: bold; }
QPushButton#sensorBtnZeroAll:hover { background-color: #8b7820; }
QPushButton#robotBtnEnable { background-color: #1a5a3c; border-color: #28a745; min-height: 34px; font-size: 14px; font-weight: bold; }
QPushButton#robotBtnEnable:checked { background-color: #28a745; }
QPushButton#robotBtnHome { background-color: #806a12; border-color: #ffc107; color: #fff7cc; font-size: 14px; font-weight: bold; }
QPushButton#robotBtnHome:hover { background-color: #a38716; }
QPushButton#testDispStart { background-color: #1a5a5a; border-color: #14a3a8; font-weight: bold; }
QPushButton#testShearStart { background-color: #5a3a1a; border-color: #e67e22; font-weight: bold; }
QPushButton#testShovelStart { background-color: #514071; border-color: #8e72d4; font-weight: bold; }
QLineEdit { background-color: #12122a; color: #e0e0e0; border: 1px solid #3a3a52; border-radius: 4px; padding: 4px 8px; font-size: 13px; }
QLineEdit:focus { border-color: #14a3a8; }
QDoubleSpinBox, QSpinBox { background-color: #12122a; color: #e0e0e0; border: 1px solid #3a3a52; border-radius: 4px; padding: 4px 6px; font-size: 13px; }
QDoubleSpinBox:focus, QSpinBox:focus { border-color: #14a3a8; }
QComboBox { background-color: #12122a; color: #e0e0e0; border: 1px solid #3a3a52; border-radius: 4px; padding: 4px 8px; font-size: 13px; }
QComboBox:hover { border-color: #14a3a8; }
QComboBox QAbstractItemView { background-color: #1c1c2e; color: #c8ccd4; selection-background-color: #14a3a8; selection-color: #ffffff; border: 1px solid #3a3a52; }
QSlider::groove:horizontal { background: #2d2d46; height: 6px; border-radius: 3px; }
QSlider::handle:horizontal { background: #14a3a8; width: 16px; margin: -5px 0; border-radius: 8px; }
QSlider::sub-page:horizontal { background: #14a3a8; border-radius: 3px; }
QScrollArea { border: none; background: transparent; }
QScrollBar:vertical { background: #1c1c2e; width: 8px; border-radius: 4px; }
QScrollBar::handle:vertical { background: #3a3a52; border-radius: 4px; min-height: 30px; }
QSplitter::handle { background: #2a2a40; width: 3px; }
QFrame#plotPlaceholder1, QFrame#plotPlaceholder2, QFrame#plotPlaceholder3 { background-color: #1c1c2e; border: 1px solid #3a3a52; border-radius: 4px; }
"""

LIGHT_ON = "background-color:#00c853;border-radius:6px;min-width:12px;max-width:12px;min-height:12px;max-height:12px;"
LIGHT_OFF = "background-color:#ff5252;border-radius:6px;min-width:12px;max-width:12px;min-height:12px;max-height:12px;"


EXTRA_STYLE = """
QGroupBox#robotPoseGroup, QGroupBox#robotJointGroup {
    border-color: #356274;
}
QGroupBox#robotJointGroup {
    margin-top: 12px;
    padding-top: 4px;
}
QGroupBox#sensorConnGroup {
    margin-top: 12px;
    padding-top: 4px;
}
QPushButton#robotBtnPTP_zengliang,
QPushButton#robotJointBtnPTP_zengliang {
    background-color: #145c63;
    border-color: #16a6ad;
    color: #f4ffff;
    font-weight: bold;
}
QPushButton#robotBtnPTP_zengliang:hover,
QPushButton#robotJointBtnPTP_zengliang:hover {
    background-color: #19777f;
}
QPushButton#robotBtnPTP_global,
QPushButton#robotJointBtnPTP_global {
    background-color: #304c78;
    border-color: #4f7fbd;
    color: #f6f9ff;
    font-weight: bold;
}
QPushButton#robotBtnPTP_global:hover,
QPushButton#robotJointBtnPTP_global:hover {
    background-color: #3c6096;
}
QLabel#poseValue_X, QLabel#poseValue_Y, QLabel#poseValue_Z,
QLabel#poseValue_Rx, QLabel#poseValue_Ry, QLabel#poseValue_Rz,
QLabel#jointValue_J1, QLabel#jointValue_J2, QLabel#jointValue_J3,
QLabel#jointValue_J4, QLabel#jointValue_J5, QLabel#jointValue_J6 {
    color: #49d5dc;
    font-family: Consolas, "Microsoft YaHei";
    font-weight: bold;
}
QFrame#jointSectionLine {
    color: #3a3a52;
    background-color: #3a3a52;
    max-height: 1px;
}
QWidget#robotActionContainer {
    background: transparent;
}
QPushButton#plotExperimentPageBtn:checked,
QPushButton#plotWrenchPageBtn:checked,
QPushButton#testStandardPageBtn:checked,
QPushButton#testShovelPageBtn:checked {
    background-color: #145c63;
    border-color: #16a6ad;
    color: #f4ffff;
    font-weight: bold;
}
"""

LIGHT_STYLE = """
QMainWindow, QWidget { background-color: #eef2f6; color: #263442; font-size: 13px; }
QGroupBox { background-color: #f8fafc; color: #263442; border: 1px solid #aebdca; border-radius: 5px; margin-top: 16px; padding-top: 18px; font-weight: bold; font-size: 13px; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 8px; color: #087f8c; }
QLabel { color: #334454; background: transparent; }
QPushButton { background-color: #e1e8ef; color: #263442; border: 1px solid #aebdca; border-radius: 4px; padding: 5px 14px; min-height: 26px; font-size: 13px; }
QPushButton:hover { background-color: #d2e4ea; border-color: #0796a3; }
QPushButton:pressed { background-color: #0796a3; color: white; }
QPushButton:disabled { background-color: #edf1f4; color: #9aa7b2; border-color: #d2dae1; }
QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox { background-color: #ffffff; color: #1e2b36; border: 1px solid #aebdca; border-radius: 4px; padding: 4px 6px; }
QLineEdit:focus, QDoubleSpinBox:focus, QSpinBox:focus, QComboBox:hover { border-color: #0796a3; }
QComboBox QAbstractItemView { background-color: #ffffff; color: #263442; selection-background-color: #0796a3; selection-color: white; }
QSlider::groove:horizontal { background: #cbd5df; height: 6px; border-radius: 3px; }
QSlider::handle:horizontal { background: #0796a3; width: 16px; margin: -5px 0; border-radius: 8px; }
QSlider::sub-page:horizontal { background: #0796a3; border-radius: 3px; }
QScrollArea { border: none; background: transparent; }
QScrollBar:vertical { background: #e4eaf0; width: 8px; border-radius: 4px; }
QScrollBar::handle:vertical { background: #aebdca; border-radius: 4px; min-height: 30px; }
QSplitter::handle { background: #c4ced8; width: 3px; }
QFrame#plotPlaceholder1, QFrame#plotPlaceholder2 { background-color: #ffffff; border: 1px solid #b7c4cf; }
QPushButton#robotBtnConnect, QPushButton#sensorBtnConnect { background-color: #218c4e; border-color: #18723e; color: white; }
QPushButton#robotBtnDisconnect, QPushButton#sensorBtnDisconnect { background-color: #b64242; border-color: #962f2f; color: white; }
QPushButton#robotBtnEnable { background-color: #278b58; border-color: #187443; color: white; font-weight: bold; }
QPushButton#robotBtnHome { background-color: #d6a514; border-color: #a87e00; color: #302600; font-weight: bold; }
QPushButton#robotBtnStop { background-color: #bd3434; border-color: #962424; color: white; font-weight: bold; }
QPushButton#sensorBtnZeroAll { background-color: #d6a514; border-color: #a87e00; color: #302600; font-weight: bold; }
QPushButton#robotBtnPTP_zengliang, QPushButton#robotJointBtnPTP_zengliang { background-color: #148c91; border-color: #08767b; color: white; font-weight: bold; }
QPushButton#robotBtnPTP_global, QPushButton#robotJointBtnPTP_global { background-color: #4676ae; border-color: #315e91; color: white; font-weight: bold; }
QPushButton#testDispStart { background-color: #148c91; border-color: #08767b; color: white; font-weight: bold; }
QPushButton#testShearStart { background-color: #c87116; border-color: #a85a0a; color: white; font-weight: bold; }
QPushButton#testShovelStart { background-color: #7557a8; border-color: #60438f; color: white; font-weight: bold; }
QPushButton#appExitBtn { background-color: #b64242; border-color: #962f2f; color: white; font-weight: bold; }
QPushButton#themeToggleBtn { background-color: #526579; border-color: #3e5062; color: white; font-weight: bold; }
QLabel#poseValue_X, QLabel#poseValue_Y, QLabel#poseValue_Z,
QLabel#poseValue_Rx, QLabel#poseValue_Ry, QLabel#poseValue_Rz,
QLabel#jointValue_J1, QLabel#jointValue_J2, QLabel#jointValue_J3,
QLabel#jointValue_J4, QLabel#jointValue_J5, QLabel#jointValue_J6 { color: #087f8c; font-family: Consolas, "Microsoft YaHei"; font-weight: bold; }
QGroupBox#robotPoseGroup, QGroupBox#robotJointGroup { border-color: #6f9eaa; }
QGroupBox#robotJointGroup, QGroupBox#sensorConnGroup { margin-top: 12px; padding-top: 4px; }
QFrame#jointSectionLine { color: #bdc8d2; background-color: #bdc8d2; max-height: 1px; }
QWidget#robotActionContainer { background: transparent; }
QPushButton#plotExperimentPageBtn:checked,
QPushButton#plotWrenchPageBtn:checked,
QPushButton#testStandardPageBtn:checked,
QPushButton#testShovelPageBtn:checked { background-color: #148c91; border-color: #08767b; color: white; font-weight: bold; }
"""


class MainWindow(QMainWindow):
    """加载 ``main_window_3.ui`` 的六维传感器 V3 主窗口。"""

    robot_connect_requested = Signal(str, int)
    robot_disconnect_requested = Signal()
    sensor_connect_requested = Signal(str)  # 仅提交串口，通信参数由sensor_6统一管理
    sensor_disconnect_requested = Signal()
    robot_enable_changed = Signal(bool)
    stop_requested = Signal()
    home_requested = Signal()
    zero_all_requested = Signal()
    sensor_zero_channel_requested = Signal(int)
    disp_test_start = Signal()
    shear_test_start = Signal()
    shovel_test_start = Signal()
    test_save_requested = Signal(str, str)
    test_reset_requested = Signal(str)
    sensor_refresh_requested = Signal()
    plot_disp_show = Signal()
    plot_disp_close = Signal()
    plot_disp_reset = Signal()
    plot_ang_show = Signal()
    plot_ang_close = Signal()
    plot_ang_reset = Signal()
    plot_wrench_show = Signal()
    plot_wrench_close = Signal()
    plot_wrench_reset = Signal()
    world_increment_requested = Signal(object)  # CartesianMoveRequest
    world_absolute_requested = Signal(object)   # CartesianMoveRequest
    joint_increment_requested = Signal(object)  # JointMoveRequest
    joint_absolute_requested = Signal(object)   # JointMoveRequest
    memory_type_changed = Signal(str)
    memory_recall_requested = Signal(int, object)  # PositionMemory

    def __init__(self):
        self._dark_theme = True
        QMainWindow.__init__(self)
        self._setup_ui_from_file()
        self._embed_matplotlib()
        self._apply_style()
        self._connect_signals()
        self._init_state()

    def _find_child(self, name):
        return self.findChild(QWidget, name)

    def _embed_matplotlib(self):
        """创建贯入、剪切和六维力三组 V3 实时图表。"""
        def style_axes(axes):
            axes.set_facecolor("#12122a")
            axes.grid(True, alpha=0.25, color="#3a3a52")
            axes.tick_params(colors="#a0a4b0")
            for spine in axes.spines.values():
                spine.set_color("#3a3a52")

        def install_canvas(figure, placeholder_name):
            canvas = FigureCanvas(figure)
            canvas.setStyleSheet("background-color:#1c1c2e;")
            placeholder = self._find_child(placeholder_name)
            if placeholder:
                parent_layout = placeholder.parentWidget().layout()
                index = parent_layout.indexOf(placeholder)
                if index >= 0:
                    parent_layout.removeWidget(placeholder)
                    placeholder.hide()
                    parent_layout.insertWidget(index, canvas)
            return canvas

        self.figure1 = Figure(figsize=(8, 4), dpi=100, facecolor="#1c1c2e")
        self.ax1 = self.figure1.add_subplot(1, 1, 1)
        style_axes(self.ax1)
        self.ax1.set_title(
            "末端位移 — 力  End-effector Displacement vs Force",
            color="#c8ccd4",
            fontsize=12,
            fontweight="bold",
        )
        self.ax1.set_xlabel("位移 Displacement (mm)", color="#a0a4b0")
        self.ax1.set_ylabel("力 Force (N)", color="#a0a4b0")
        self.line1, = self.ax1.plot(
            [], [], "#14a3a8", linewidth=1.5, label="位移-力"
        )
        self.ax1.legend(
            loc="upper left",
            facecolor="#1c1c2e",
            edgecolor="#3a3a52",
            labelcolor="#c8ccd4",
            fontsize=9,
        )
        self.figure1.tight_layout(pad=2.5)
        self.canvas1 = install_canvas(self.figure1, "plotPlaceholder1")

        self.figure2 = Figure(figsize=(8, 4), dpi=100, facecolor="#1c1c2e")
        self.ax2 = self.figure2.add_subplot(1, 1, 1)
        style_axes(self.ax2)
        self.ax2.set_title(
            "角位移 — 扭矩  Angular Displacement vs Torque",
            color="#c8ccd4",
            fontsize=12,
            fontweight="bold",
        )
        self.ax2.set_xlabel("角位移 Angular (°)", color="#a0a4b0")
        self.ax2.set_ylabel("扭矩 Torque (N·m)", color="#a0a4b0")
        self.line2, = self.ax2.plot(
            [], [], "#e67e22", linewidth=1.5, label="角位移-扭矩"
        )
        self.ax2.legend(
            loc="upper left",
            facecolor="#1c1c2e",
            edgecolor="#3a3a52",
            labelcolor="#c8ccd4",
            fontsize=9,
        )
        self.figure2.tight_layout(pad=2.5)
        self.canvas2 = install_canvas(self.figure2, "plotPlaceholder2")

        self.figure3 = Figure(figsize=(8, 7), dpi=100, facecolor="#1c1c2e")
        self.ax3_force = self.figure3.add_subplot(2, 1, 1)
        self.ax3_torque = self.figure3.add_subplot(
            2, 1, 2, sharex=self.ax3_force
        )
        self.ax3_force.set_title(
            "六维力实时曲线  Six-axis Force/Torque",
            color="#c8ccd4",
            fontsize=12,
            fontweight="bold",
        )
        self.ax3_force.set_ylabel("力 Force (N)", color="#a0a4b0")
        self.ax3_torque.set_ylabel("力矩 Torque (N·m)", color="#a0a4b0")
        self.ax3_torque.set_xlabel("时间 Time (s)", color="#a0a4b0")
        for axes in (self.ax3_force, self.ax3_torque):
            style_axes(axes)

        self.wrench_lines = {}
        for name, color in zip(
            ("Fx", "Fy", "Fz"),
            ("#ef5350", "#66bb6a", "#42a5f5"),
        ):
            self.wrench_lines[name], = self.ax3_force.plot(
                [], [], color, linewidth=1.3, label=name
            )
        for name, color in zip(
            ("Tx", "Ty", "Tz"),
            ("#ab47bc", "#ffa726", "#26c6da"),
        ):
            self.wrench_lines[name], = self.ax3_torque.plot(
                [], [], color, linewidth=1.3, label=name
            )
        for axes in (self.ax3_force, self.ax3_torque):
            axes.legend(
                loc="upper left",
                ncol=3,
                facecolor="#1c1c2e",
                edgecolor="#3a3a52",
                labelcolor="#c8ccd4",
                fontsize=9,
            )
        self.figure3.tight_layout(pad=2.0)
        self.canvas3 = install_canvas(self.figure3, "plotPlaceholder3")

        self._disp_data = []
        self._ang_data = []
        self._wrench_data = []
        self._wrench_time_origin = time.monotonic()

    def _setup_ui_from_file(self):
        loader = QUiLoader()
        widget = loader.load(UI_PATH, self)
        if widget is None:
            raise RuntimeError(f"无法加载 UI 文件: {UI_PATH}")
        self.setCentralWidget(widget)
        self.setWindowTitle("机械臂控制与六维力传感数据采集系统 - V3")
        icon = QIcon(ICON_PATH)
        self.setWindowIcon(icon)
        app = QApplication.instance()
        if app:
            app.setWindowIcon(icon)

        logo_label = self._find_child("jluLogoLabel")
        if logo_label:
            logo_label.setPixmap(
                QPixmap(LOGO_PATH).scaled(
                    150,
                    150,
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
            )
        self.resize(1920, 1080)
        self.setMinimumSize(1400, 800)

        splitter = self._find_child("mainSplitter")
        if splitter:
            splitter.setChildrenCollapsible(False)
            splitter.setStretchFactor(0, 0)
            splitter.setStretchFactor(1, 1)
            splitter.setStretchFactor(2, 0)
            # 右栏需要容纳铲挖 B～G 的三轴输入；保持左栏完整显示，
            # 其余空间仍优先分配给中间曲线区。
            splitter.setSizes([540, 970, 410])

        self._fullscreen_shortcut = QShortcut(QKeySequence("F11"), self)
        self._fullscreen_shortcut.activated.connect(self.toggle_fullscreen)
        self._exit_fullscreen_shortcut = QShortcut(QKeySequence("Escape"), self)
        self._exit_fullscreen_shortcut.activated.connect(self.exit_fullscreen)

    @Slot()
    def toggle_fullscreen(self):
        """F11在全屏与最大化窗口之间切换。"""
        if self.isFullScreen():
            self.showMaximized()
        else:
            self.showFullScreen()

    @Slot()
    def exit_fullscreen(self):
        if self.isFullScreen():
            self.showMaximized()

    def _apply_style(self):
        self.setStyleSheet(DARK_STYLE + EXTRA_STYLE)

    def _apply_plot_theme(self):
        if not hasattr(self, "ax1"):
            return
        colors = (
            {
                "figure": "#1c1c2e", "axes": "#12122a", "text": "#c8ccd4",
                "muted": "#a0a4b0", "grid": "#3a3a52", "legend": "#1c1c2e",
            }
            if self._dark_theme else
            {
                "figure": "#eef2f6", "axes": "#ffffff", "text": "#263442",
                "muted": "#526474", "grid": "#c6d0da", "legend": "#f8fafc",
            }
        )
        for figure, axes_list, canvas in (
            (self.figure1, (self.ax1,), self.canvas1),
            (self.figure2, (self.ax2,), self.canvas2),
            (self.figure3, (self.ax3_force, self.ax3_torque), self.canvas3),
        ):
            figure.set_facecolor(colors["figure"])
            for axes in axes_list:
                axes.set_facecolor(colors["axes"])
                axes.title.set_color(colors["text"])
                axes.xaxis.label.set_color(colors["muted"])
                axes.yaxis.label.set_color(colors["muted"])
                axes.tick_params(colors=colors["muted"])
                axes.grid(True, alpha=0.35, color=colors["grid"])
                for spine in axes.spines.values():
                    spine.set_color(colors["grid"])
                legend = axes.get_legend()
                if legend:
                    legend.get_frame().set_facecolor(colors["legend"])
                    legend.get_frame().set_edgecolor(colors["grid"])
                    for text in legend.get_texts():
                        text.set_color(colors["text"])
            canvas.setStyleSheet(f"background-color:{colors['figure']};")
            canvas.draw_idle()

    @Slot()
    def toggle_theme(self):
        self._dark_theme = not self._dark_theme
        self.setStyleSheet(
            DARK_STYLE + EXTRA_STYLE if self._dark_theme else LIGHT_STYLE
        )
        button = self._find_child("themeToggleBtn")
        if button:
            button.setText("明亮主题" if self._dark_theme else "暗色主题")
        self._apply_plot_theme()

    def _connect_common_signals(self):
        """绑定 V3 使用的连接、试验、图表和公共动作控件。"""
        direct_bindings = (
            ("robotBtnConnect", self._on_robot_connect),
            ("robotBtnDisconnect", self._on_robot_disconnect),
            ("sensorBtnConnect", self._on_sensor_connect),
            ("sensorBtnDisconnect", self._on_sensor_disconnect),
            ("robotBtnHome", self.home_requested.emit),
            ("testDispStart", self.disp_test_start.emit),
            ("testShearStart", self.shear_test_start.emit),
            ("testShovelStart", self.shovel_test_start.emit),
            ("sensorRefreshBtn", self.sensor_refresh_requested.emit),
        )
        for object_name, callback in direct_bindings:
            button = self._find_child(object_name)
            if button:
                button.clicked.connect(callback)

        enable_button = self._find_child("robotBtnEnable")
        if enable_button:
            enable_button.toggled.connect(self.robot_enable_changed.emit)

        stop_button = self._find_child("robotBtnStop")
        if stop_button:
            stop_button.clicked.connect(self.stop_requested.emit)
            stop_button.clicked.connect(lambda: self.set_robot_enabled(False))

        zero_all = self._find_child("sensorBtnZeroAll")
        if zero_all:
            zero_all.clicked.connect(self.zero_all_requested.emit)
        for channel, axis in enumerate(("Fx", "Fy", "Fz", "Tx", "Ty", "Tz"), 1):
            button = self._find_child(f"sensorBtnZero{axis}")
            if button:
                button.clicked.connect(
                    lambda _checked=False, index=channel:
                    self.sensor_zero_channel_requested.emit(index)
                )

        plot_bindings = (
            ("plotDisp", self.plot_disp_show, self.plot_disp_close, self.plot_disp_reset),
            ("plotAng", self.plot_ang_show, self.plot_ang_close, self.plot_ang_reset),
            (
                "plotWrench",
                self.plot_wrench_show,
                self.plot_wrench_close,
                self.plot_wrench_reset,
            ),
        )
        for prefix, show_signal, close_signal, reset_signal in plot_bindings:
            for suffix, signal in zip(
                ("Show", "Close", "Reset"),
                (show_signal, close_signal, reset_signal),
            ):
                button = self._find_child(f"{prefix}{suffix}")
                if button:
                    button.clicked.connect(signal.emit)

        for prefix in ("testDisp", "testShear", "testShovel"):
            browse_button = self._find_child(f"{prefix}SaveBrowse")
            if browse_button:
                browse_button.clicked.connect(
                    lambda _checked=False, kind=prefix:
                    self._on_test_browse(kind)
                )
            save_button = self._find_child(f"{prefix}Save")
            if save_button:
                save_button.clicked.connect(
                    lambda _checked=False, kind=prefix:
                    self._on_test_save(kind)
                )
            reset_button = self._find_child(f"{prefix}Reset")
            if reset_button:
                reset_button.clicked.connect(
                    lambda _checked=False, kind=prefix:
                    self.test_reset_requested.emit(kind)
                )

    def _connect_signals(self):
        self._connect_common_signals()

        self._bind_stacked_navigation(
            "plotStackedWidget",
            ("plotExperimentPageBtn", "plotWrenchPageBtn"),
        )
        self._bind_stacked_navigation(
            "testStackedWidget",
            ("testStandardPageBtn", "testShovelPageBtn"),
        )

        theme_button = self._find_child("themeToggleBtn")
        if theme_button:
            theme_button.clicked.connect(self.toggle_theme)
        exit_button = self._find_child("appExitBtn")
        if exit_button:
            exit_button.clicked.connect(self.close)

        self._bind_speed_pair(
            "robotSpeedSlider_zengliang",
            "robotSpeedSpin_zengliang",
        )
        self._bind_speed_pair(
            "robotSpeedSlider_global",
            "robotSpeedSpin_global",
        )
        self._bind_speed_pair(
            "robotJointSpeedSlider_zengliang",
            "robotJointSpeedSpin_zengliang",
        )
        self._bind_speed_pair(
            "robotJointSpeedSlider_global",
            "robotJointSpeedSpin_global",
        )

        bindings = {
            "robotBtnPTP_zengliang": self._emit_world_increment,
            "robotBtnPTP_global": self._emit_world_absolute,
            "robotJointBtnPTP_zengliang": self._emit_joint_increment,
            "robotJointBtnPTP_global": self._emit_joint_absolute,
        }
        for object_name, callback in bindings.items():
            button = self._find_child(object_name)
            if button:
                button.clicked.connect(callback)

        memory_type = self._find_child("memoryTypeCombo")
        if memory_type:
            memory_type.currentIndexChanged.connect(self._on_memory_type_changed)
        for index in range(1, 6):
            clear_button = self._find_child(f"memClear_{index}")
            save_button = self._find_child(f"memSave_{index}")
            recall_button = self._find_child(f"memRecall_{index}")
            if clear_button:
                clear_button.setText("清除")
                clear_button.clicked.connect(
                    lambda _checked=False, slot=index: self._clear_memory_slot(slot)
                )
            if save_button:
                save_button.clicked.connect(
                    lambda _checked=False, slot=index: self._save_memory_slot(slot)
                )
            if recall_button:
                recall_button.clicked.connect(
                    lambda _checked=False, slot=index: self._recall_memory_slot(slot)
                )

    def _bind_stacked_navigation(self, stack_name, button_names):
        stack = self._find_child(stack_name)
        buttons = tuple(self._find_child(name) for name in button_names)
        if stack is None or any(button is None for button in buttons):
            raise RuntimeError(f"堆叠页面导航控件缺失: {stack_name}")

        def activate(index):
            stack.setCurrentIndex(index)
            for button_index, button in enumerate(buttons):
                button.setChecked(button_index == index)

        for index, button in enumerate(buttons):
            button.clicked.connect(
                lambda _checked=False, page_index=index: activate(page_index)
            )
        activate(0)

    def _init_state(self):
        self._current_pose = (0.0,) * 6
        self._current_joints = (0.0,) * 6
        self._memory_slots = {
            1: PositionMemory("joint", (0.0, 16.0, -25.0, 0.0, 9.0, 0.0))
        }
        self._memory_settings = QSettings(
            SETTINGS_PATH,
            QSettings.Format.IniFormat,
        )
        self._migrate_legacy_registry_settings()
        for obsolete_name in ("testShovelDepth", "testShovelAngle"):
            self._memory_settings.remove(
                self._test_parameter_key(obsolete_name)
            )
        self._memory_settings.sync()
        self._load_memory_slots()
        self._last_plot_draw = [0.0, 0.0, 0.0]
        self._plot_draw_interval = 0.10
        for name in ("robotStatusLight", "sensorStatusLight"):
            light = self._find_child(name)
            if light:
                light.setStyleSheet(LIGHT_OFF)
        desktop_path = QStandardPaths.writableLocation(
            QStandardPaths.StandardLocation.DesktopLocation
        ) or os.path.join(os.path.expanduser("~"), "Desktop")
        for prefix in ("testDisp", "testShear", "testShovel"):
            path_widget = self._find_child(f"{prefix}SavePath")
            if path_widget:
                path_widget.setText(desktop_path)
        self._load_test_parameters()
        self._bind_test_parameter_memory()
        self._configure_control_geometry()
        self.update_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.update_joint_angles(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._refresh_all_memory_labels()

        tooltips = {
            "robotBtnPTP_zengliang": "按当前末端世界坐标计算六维相对增量",
            "robotBtnPTP_global": "移动到输入的末端世界坐标绝对目标",
            "robotJointBtnPTP_zengliang": "按当前关节角计算 J1-J6 相对增量",
            "robotJointBtnPTP_global": "移动到输入的 J1-J6 绝对关节角",
        }
        for object_name, text in tooltips.items():
            widget = self._find_child(object_name)
            if widget:
                widget.setToolTip(text)

    def _configure_control_geometry(self):
        """统一两行实时值、关节输入单位和四个发送按钮的尺寸。"""
        connection_group = self._find_child("robotConnGroup")
        if connection_group:
            connection_group.setGeometry(4, 4, 511, 108)
            rows = sorted(
                connection_group.findChildren(QWidget, "layoutWidget"),
                key=lambda widget: widget.y(),
            )
            if len(rows) >= 2:
                rows[0].setGeometry(11, 18, 489, 34)
                rows[1].setGeometry(11, 60, 489, 40)

        # 连接区增高 15 px；末端位姿区采用与关节角区相同的 335 px 节奏。
        pose_group = self._find_child("robotPoseGroup")
        joint_group = self._find_child("robotJointGroup")
        memory_group = self._find_child("robotMemGroup")
        action_container = self._find_child("robotActionContainer")
        if pose_group:
            pose_group.setGeometry(4, 117, 511, 335)
            current_xyz = self._find_child("poseCurrentRowXYZ")
            current_rotation = self._find_child("poseCurrentRowRotation")
            if current_xyz:
                current_xyz.setGeometry(10, 17, 491, 35)
            if current_rotation:
                current_rotation.setGeometry(10, 56, 491, 35)

            # 旧 UI 的多个输入行使用了重复的 layoutWidget 名称。在部分 Qt
            # 版本中调整固定宽度后会把三列控件归入同一个窄容器。将输入控件
            # 直接放到 GroupBox，坐标严格复用关节角控制的三列排布。
            columns = {"X": 0, "Y": 1, "Z": 2,
                       "Rx": 0, "Ry": 1, "Rz": 2}
            column_label_x = (9, 175, 340)
            column_input_x = (41, 207, 372)
            column_unit_x = (131, 296, 462)
            row_y = {
                "zengliang": {"X": 95, "Y": 95, "Z": 95,
                               "Rx": 134, "Ry": 134, "Rz": 134},
                "global": {"X": 216, "Y": 216, "Z": 216,
                            "Rx": 255, "Ry": 255, "Rz": 255},
            }
            for suffix in ("zengliang", "global"):
                for axis in WORLD_AXES:
                    spin = self._find_child(f"robotTarget{axis}_{suffix}")
                    label_suffix = "" if suffix == "zengliang" else "_7"
                    unit_suffix = "" if suffix == "zengliang" else "_3"
                    axis_label = self._find_child(f"poseLabel_{axis}{label_suffix}")
                    unit_label = self._find_child(f"poseUnit_{axis}{unit_suffix}")
                    column = columns[axis]
                    y = row_y[suffix][axis]
                    if axis_label:
                        axis_label.setParent(pose_group)
                        axis_label.setGeometry(column_label_x[column], y, 28, 35)
                        axis_label.show()
                    if spin:
                        spin.setParent(pose_group)
                        spin.setGeometry(column_input_x[column], y + 3, 86, 29)
                        spin.show()
                    if unit_label:
                        unit_label.setParent(pose_group)
                        unit_label.setGeometry(column_unit_x[column], y, 40, 35)
                        unit_label.show()

                speed_y = 173 if suffix == "zengliang" else 294
                speed_label_name = "speedLabel" if suffix == "zengliang" else "speedLabel_3"
                speed_label = self._find_child(speed_label_name)
                speed_slider = self._find_child(f"robotSpeedSlider_{suffix}")
                speed_spin = self._find_child(f"robotSpeedSpin_{suffix}")
                for widget, geometry in (
                    (speed_label, (9, speed_y, 130, 32)),
                    (speed_slider, (143, speed_y + 6, 158, 20)),
                    (speed_spin, (305, speed_y + 1, 77, 29)),
                ):
                    if widget:
                        widget.setParent(pose_group)
                        widget.setGeometry(*geometry)
                        widget.show()
                if speed_slider:
                    speed_slider.setFixedSize(158, 20)
                if speed_spin:
                    speed_spin.setFixedSize(77, 29)
                send_button = self._find_child(f"robotBtnPTP_{suffix}")
                if send_button:
                    send_button.setParent(pose_group)
                    send_button.setGeometry(386, speed_y, 116, 32)
                    send_button.show()

            # 隐藏已经清空的旧绝对定位容器，实时显示的两个容器继续保留。
            for child in pose_group.findChildren(
                QWidget, options=Qt.FindChildOption.FindDirectChildrenOnly
            ):
                if child.objectName() not in (
                    "poseCurrentRowXYZ", "poseCurrentRowRotation"
                ) and child.layout() is not None:
                    child.hide()

        if joint_group:
            joint_group.setGeometry(4, 457, 511, 335)
        if memory_group:
            memory_group.setGeometry(4, 797, 511, 270)
        if action_container:
            action_container.setGeometry(10, 1072, 495, 46)
        scroll_widget = self._find_child("robotScrollWidget")
        if scroll_widget:
            scroll_widget.setMinimumHeight(1125)

        sensor_connection_group = self._find_child("sensorConnGroup")
        if sensor_connection_group and sensor_connection_group.layout():
            sensor_connection_group.layout().setContentsMargins(9, 0, 9, 9)
            sensor_connection_group.layout().setSpacing(4)

        ip_edit = self._find_child("robotIpEdit")
        port_edit = self._find_child("robotPortEdit")
        if ip_edit:
            ip_edit.setMinimumWidth(155)
            ip_edit.setMaximumWidth(175)
        if port_edit:
            port_edit.setMinimumWidth(80)
            port_edit.setMaximumWidth(90)
        for button_name in ("robotBtnConnect", "robotBtnDisconnect"):
            button = self._find_child(button_name)
            if button:
                button.setFixedWidth(100)
                button.setFixedHeight(32)
        status_label = self._find_child("robotStatusLabel")
        if status_label:
            status_label.setFixedWidth(52)

        for axis in WORLD_AXES:
            for suffix in ("zengliang", "global"):
                input_widget = self._find_child(f"robotTarget{axis}_{suffix}")
                if input_widget:
                    input_widget.setFixedWidth(86)

        joint_group = self._find_child("robotJointGroup")
        if joint_group and joint_group.layout():
            joint_group.layout().setContentsMargins(8, 0, 8, 8)

        for button_name in ("robotBtnEnable", "robotBtnHome", "robotBtnStop"):
            button = self._find_child(button_name)
            if button:
                button.setFixedHeight(36)

        for axis in WORLD_AXES:
            name_label = self._find_child(f"poseLabel_{axis}_2")
            value_label = self._find_child(f"poseValue_{axis}")
            if name_label:
                name_label.setFixedWidth(40)
            if value_label:
                value_label.setMinimumWidth(92)
                value_label.setAlignment(
                    Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
                )

            for unit_name in (f"poseUnit_{axis}", f"poseUnit_{axis}_3"):
                unit_label = self._find_child(unit_name)
                if unit_label:
                    unit_label.setFixedWidth(40)
                    unit_label.setAlignment(
                        Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
                    )

        for axis in JOINT_AXES:
            current_label = self._find_child(f"jointLabel_{axis}_current")
            current_value = self._find_child(f"jointValue_{axis}")
            if current_label:
                current_label.setFixedWidth(40)
            if current_value:
                current_value.setMinimumWidth(92)
                current_value.setAlignment(
                    Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
                )

            for suffix in ("zengliang", "global"):
                axis_label = self._find_child(f"jointLabel_{axis}_{suffix}")
                input_widget = self._find_child(
                    f"robotJointTarget{axis}_{suffix}"
                )
                unit_label = self._find_child(f"jointUnit_{axis}_{suffix}")
                if axis_label:
                    axis_label.setFixedWidth(28)
                if input_widget:
                    input_widget.setMinimumWidth(82)
                if unit_label:
                    unit_label.setFixedWidth(40)
                    unit_label.setAlignment(
                        Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
                    )

        for title_name in ("jointCurrentTitle", "jointCurrentTitleSpacer"):
            title = self._find_child(title_name)
            if title:
                title.setFixedWidth(44)

        for speed_label_name in (
            "jointSpeedLabel_zengliang",
            "jointSpeedLabel_global",
        ):
            speed_label = self._find_child(speed_label_name)
            if speed_label:
                speed_label.setFixedWidth(130)

        for speed_label_name in ("speedLabel", "speedLabel_3"):
            speed_label = self._find_child(speed_label_name)
            if speed_label:
                speed_label.setFixedWidth(130)

        memory_type_label = self._find_child("memoryTypeLabel")
        memory_type_hint = self._find_child("memoryTypeHint")
        if memory_type_label:
            memory_type_label.setFixedWidth(80)
        if memory_type_hint:
            memory_type_hint.setFixedWidth(120)

        for index in range(1, 6):
            value_label = self._find_child(f"memVal_{index}")
            if value_label:
                value_label.setMinimumWidth(180)
            for prefix in ("memSave", "memRecall", "memClear"):
                button = self._find_child(f"{prefix}_{index}")
                if button:
                    button.setFixedSize(64, 32)

        for browse_name in (
            "testDispSaveBrowse", "testShearSaveBrowse", "testShovelSaveBrowse",
        ):
            browse_button = self._find_child(browse_name)
            if browse_button:
                browse_button.setFixedWidth(40)

        # 试验区采用统一列宽，避免全屏及高DPI下标签、单位和输入框错位。
        for label_name in (
            "testDispSpeedLabel", "testDispDistLabel", "testForceMaxLabel",
            "testDispSaveLabel", "testDispNameLabel", "testAngSpeedLabel",
            "testAngDistLabel", "testTorqueMaxLabel", "testShearSaveLabel",
            "testShearNameLabel", "testShovelSpeedLabel",
            "testShovelSaveLabel",
            "testShovelNameLabel",
        ):
            label = self._find_child(label_name)
            if label:
                label.setFixedWidth(82)

        for input_name in (
            "testDispSpeed", "testDispDist", "testForceMax",
            "testAngSpeed", "testAngDist", "testTorqueMax",
            "testShovelSpeed",
        ):
            input_widget = self._find_child(input_name)
            if input_widget:
                input_widget.setMinimumWidth(110)

        for unit_name in (
            "testDispSpeedUnit", "testDispDistUnit", "testForceMaxUnit",
            "testAngSpeedUnit", "testAngDistUnit", "testTorqueMaxUnit",
            "testShovelSpeedUnit",
        ):
            unit = self._find_child(unit_name)
            if unit:
                unit.setFixedWidth(48)

        for point in SHOVEL_POINTS:
            point_label = self._find_child(f"testShovel{point}Label")
            point_unit = self._find_child(f"testShovel{point}Unit")
            if point_label:
                point_label.setFixedWidth(62)
                point_label.setAlignment(
                    Qt.AlignmentFlag.AlignRight
                    | Qt.AlignmentFlag.AlignVCenter
                )
            if point_unit:
                point_unit.setFixedWidth(28)
                point_unit.setAlignment(
                    Qt.AlignmentFlag.AlignLeft
                    | Qt.AlignmentFlag.AlignVCenter
                )
            for axis in ("X", "Y", "Z"):
                axis_label = self._find_child(
                    f"testShovel{point}_{axis}Label"
                )
                value = self._find_child(f"testShovel{point}_{axis}")
                if axis_label:
                    axis_label.setFixedWidth(12)
                    axis_label.setAlignment(Qt.AlignCenter)
                if value:
                    value.setFixedWidth(72)

        for button_name in (
            "testShovelStart", "testShovelSave", "testShovelReset",
        ):
            button = self._find_child(button_name)
            if button:
                button.setFixedSize(110, 34)

        for path_name in (
            "testDispSavePath", "testShearSavePath", "testShovelSavePath",
        ):
            path_widget = self._find_child(path_name)
            if path_widget:
                path_widget.setMinimumWidth(180)

        for object_name in (
            "robotBtnPTP_zengliang",
            "robotBtnPTP_global",
            "robotJointBtnPTP_zengliang",
            "robotJointBtnPTP_global",
        ):
            button = self._find_child(object_name)
            if button:
                button.setFixedSize(116, 32)

    @staticmethod
    def _ensure_finite(values, description):
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f"{description}包含无效数值")

    def _bind_speed_pair(self, slider_name, spin_name):
        slider = self._find_child(slider_name)
        spin = self._find_child(spin_name)
        if not slider or not spin:
            raise RuntimeError(f"速度控件缺失: {slider_name} / {spin_name}")
        slider.valueChanged.connect(spin.setValue)
        spin.valueChanged.connect(slider.setValue)

    def _read_spin_values(self, name_template, axes):
        values = []
        for axis in axes:
            object_name = name_template.format(axis=axis)
            widget = self._find_child(object_name)
            if widget is None or not hasattr(widget, "value"):
                raise RuntimeError(f"运动输入控件缺失: {object_name}")
            values.append(float(widget.value()))
        self._ensure_finite(values, "运动参数")
        return tuple(values)

    def get_world_increment_request(self):
        values = self._read_spin_values(
            "robotTarget{axis}_zengliang",
            WORLD_AXES,
        )
        speed = float(self._find_child("robotSpeedSpin_zengliang").value())
        return CartesianMoveRequest(*values, speed)

    def get_world_absolute_request(self):
        values = self._read_spin_values(
            "robotTarget{axis}_global",
            WORLD_AXES,
        )
        speed = float(self._find_child("robotSpeedSpin_global").value())
        return CartesianMoveRequest(*values, speed)

    def get_joint_increment_request(self):
        values = self._read_spin_values(
            "robotJointTarget{axis}_zengliang",
            JOINT_AXES,
        )
        speed = float(self._find_child("robotJointSpeedSpin_zengliang").value())
        return JointMoveRequest(*values, speed)

    def get_joint_absolute_request(self):
        values = self._read_spin_values(
            "robotJointTarget{axis}_global",
            JOINT_AXES,
        )
        speed = float(self._find_child("robotJointSpeedSpin_global").value())
        return JointMoveRequest(*values, speed)

    @Slot()
    def _emit_world_increment(self):
        request = self.get_world_increment_request()
        self.world_increment_requested.emit(request)
        self.statusBar().showMessage("末端增量运动请求已提交", 2500)

    @Slot()
    def _emit_world_absolute(self):
        request = self.get_world_absolute_request()
        self.world_absolute_requested.emit(request)
        self.statusBar().showMessage("末端绝对运动请求已提交", 2500)

    @Slot()
    def _emit_joint_increment(self):
        request = self.get_joint_increment_request()
        self.joint_increment_requested.emit(request)
        self.statusBar().showMessage("关节增量运动请求已提交", 2500)

    @Slot()
    def _emit_joint_absolute(self):
        request = self.get_joint_absolute_request()
        self.joint_absolute_requested.emit(request)
        self.statusBar().showMessage("关节绝对运动请求已提交", 2500)

    def _on_robot_connect(self):
        """只发出连接请求，连接灯由真实连接结果更新。"""
        ip_edit = self._find_child("robotIpEdit")
        port_edit = self._find_child("robotPortEdit")
        ip = ip_edit.text().strip() if ip_edit else "192.168.1.4"
        try:
            port = int(port_edit.text().strip()) if port_edit else 9760
        except ValueError:
            port = 9760
        self.robot_connect_requested.emit(ip, port)

    def _on_sensor_connect(self):
        """只提交串口号，具体通信参数由sensor_6管理。"""
        port_widget = self._find_child("sensorComPort")
        port = port_widget.text().strip() if port_widget else "COM3"
        self.sensor_connect_requested.emit(port)

    def _on_robot_disconnect(self):
        self.robot_disconnect_requested.emit()
        self._set_robot_connected(False)

    def _on_sensor_disconnect(self):
        self.sensor_disconnect_requested.emit()
        self._set_sensor_connected(False)

    def _on_test_browse(self, prefix):
        path_edit = self._find_child(f"{prefix}SavePath")
        current = path_edit.text() if path_edit else os.path.expanduser("~")
        selected = QFileDialog.getExistingDirectory(
            self,
            "选择保存目录",
            current,
        )
        if selected and path_edit:
            path_edit.setText(selected)

    def _on_test_save(self, prefix):
        path_edit = self._find_child(f"{prefix}SavePath")
        name_edit = self._find_child(f"{prefix}FileName")
        save_dir = path_edit.text() if path_edit else os.path.expanduser("~")
        name = name_edit.text().strip() if name_edit else "01"
        if not name:
            name = "01"
        suffix = name.zfill(2) if name.isdigit() else name
        fixed_prefix = {
            "testDisp": "pen_motor",
            "testShear": "cut_motor",
            "testShovel": "shovel_motor",
        }.get(prefix, "test_motor")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(
            save_dir,
            f"{fixed_prefix}_{timestamp}_{suffix}.xlsx",
        )
        self.test_save_requested.emit(prefix, filepath)

    def _set_robot_connected(self, state):
        for object_name, enabled in (
            ("robotBtnConnect", not state),
            ("robotBtnDisconnect", state),
        ):
            widget = self._find_child(object_name)
            if widget:
                widget.setEnabled(enabled)
        light = self._find_child("robotStatusLight")
        label = self._find_child("robotStatusLabel")
        if light:
            light.setStyleSheet(LIGHT_ON if state else LIGHT_OFF)
        if label:
            label.setText("已连接" if state else "未连接")
            label.setStyleSheet(
                "color:#00c853;" if state else "color:#ff5252;"
            )

    def _set_sensor_connected(self, state):
        for object_name, enabled in (
            ("sensorBtnConnect", not state),
            ("sensorBtnDisconnect", state),
        ):
            widget = self._find_child(object_name)
            if widget:
                widget.setEnabled(enabled)
        light = self._find_child("sensorStatusLight")
        label = self._find_child("sensorStatusLabel")
        if light:
            light.setStyleSheet(LIGHT_ON if state else LIGHT_OFF)
        if label:
            label.setText("已连接" if state else "未连接")
            label.setStyleSheet(
                "color:#00c853;" if state else "color:#ff5252;"
            )

    def set_robot_enabled(self, state):
        button = self._find_child("robotBtnEnable")
        if button:
            button.setChecked(bool(state))
            button.setText("已使能" if state else "使能")

    def update_pose(self, x, y, z, rx, ry, rz):
        self._current_pose = tuple(float(v) for v in (x, y, z, rx, ry, rz))
        for axis, value in zip(WORLD_AXES, self._current_pose):
            label = self._find_child(f"poseValue_{axis}")
            if label:
                label.setText(f"{float(value):.3f}")
                label.setToolTip(f"{axis} = {float(value):.6f}")

    def update_joint_angles(self, j1, j2, j3, j4, j5, j6):
        self._current_joints = tuple(float(v) for v in (j1, j2, j3, j4, j5, j6))
        for axis, value in zip(JOINT_AXES, self._current_joints):
            label = self._find_child(f"jointValue_{axis}")
            if label:
                label.setText(f"{float(value):.3f}")
                label.setToolTip(f"{axis} = {float(value):.6f} deg")

    def clear_robot_realtime_display(self):
        """断开连接后停止展示最后一次机械臂位姿。"""
        self._current_pose = (0.0,) * 6
        self._current_joints = (0.0,) * 6
        for axis in WORLD_AXES:
            label = self._find_child(f"poseValue_{axis}")
            if label:
                label.setText("--")
                label.setToolTip("机械臂未连接")
        for axis in JOINT_AXES:
            label = self._find_child(f"jointValue_{axis}")
            if label:
                label.setText("--")
                label.setToolTip("机械臂未连接")

    def update_force_torque(self, fx, fy, fz, tx, ty, tz):
        values = {
            "Fx": fx,
            "Fy": fy,
            "Fz": fz,
            "Tx": tx,
            "Ty": ty,
            "Tz": tz,
        }
        for axis, value in values.items():
            label = self._find_child(f"sensorWrenchVal_{axis}")
            if label:
                label.setText(f"{float(value):.3f}")

    def clear_force_torque(self):
        for axis in ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz"):
            label = self._find_child(f"sensorWrenchVal_{axis}")
            if label:
                label.setText("--")

    def get_test_params_disp(self):
        speed = self._find_child("testDispSpeed")
        distance = self._find_child("testDispDist")
        force_max = self._find_child("testForceMax")
        return (
            speed.value() if speed else 1.0,
            distance.value() if distance else 10.0,
            force_max.value() if force_max else 10.0,
        )

    def get_test_params_shear(self):
        speed = self._find_child("testAngSpeed")
        angle = self._find_child("testAngDist")
        torque_max = self._find_child("testTorqueMax")
        return (
            speed.value() if speed else 5.0,
            angle.value() if angle else 30.0,
            torque_max.value() if torque_max else 5.0,
        )

    def get_test_params_shovel(self):
        speed = self._find_child("testShovelSpeed")
        points = []
        for point in SHOVEL_POINTS:
            points.append(tuple(
                self._find_child(f"testShovel{point}_{axis}").value()
                for axis in ("X", "Y", "Z")
            ))
        return speed.value() if speed else 10.0, tuple(points)

    def add_disp_force(self, displacement, force):
        self._disp_data.append((float(displacement), float(force)))
        if len(self._disp_data) > 5000:
            self._disp_data = self._disp_data[-5000:]
        self._refresh_plot(0)

    def add_ang_torque(self, angular, torque):
        self._ang_data.append((float(angular), float(torque)))
        if len(self._ang_data) > 5000:
            self._ang_data = self._ang_data[-5000:]
        self._refresh_plot(1)

    def add_wrench_sample(self, fx, fy, fz, tx, ty, tz, timestamp=None):
        if timestamp is None:
            timestamp = time.monotonic() - self._wrench_time_origin
        self._wrench_data.append(
            (
                float(timestamp),
                float(fx),
                float(fy),
                float(fz),
                float(tx),
                float(ty),
                float(tz),
            )
        )
        if len(self._wrench_data) > 5000:
            self._wrench_data = self._wrench_data[-5000:]
        self._refresh_wrench_plot()

    def _refresh_wrench_plot(self, force=False):
        now = time.monotonic()
        if (
            not force
            and now - self._last_plot_draw[2] < self._plot_draw_interval
        ):
            return
        self._last_plot_draw[2] = now
        if self._wrench_data:
            columns = tuple(zip(*self._wrench_data))
            times = columns[0]
            for index, name in enumerate(
                ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz"),
                1,
            ):
                self.wrench_lines[name].set_data(times, columns[index])
        else:
            for line in self.wrench_lines.values():
                line.set_data([], [])
        for axes in (self.ax3_force, self.ax3_torque):
            axes.relim()
            axes.autoscale_view()
        if self.canvas3.isVisible():
            self.canvas3.draw_idle()

    def _refresh_plot(self, ax_index, force=False):
        now = time.monotonic()
        if (
            not force
            and now - self._last_plot_draw[ax_index]
            < self._plot_draw_interval
        ):
            return
        self._last_plot_draw[ax_index] = now
        data = self._disp_data if ax_index == 0 else self._ang_data
        line = self.line1 if ax_index == 0 else self.line2
        axes = self.ax1 if ax_index == 0 else self.ax2
        canvas = self.canvas1 if ax_index == 0 else self.canvas2
        if data:
            x_values, y_values = zip(*data)
        else:
            x_values, y_values = (), ()
        line.set_data(x_values, y_values)
        axes.relim()
        axes.autoscale_view()
        if canvas.isVisible():
            canvas.draw_idle()

    def set_wrench_plot_visible(self, visible):
        self.canvas3.setVisible(bool(visible))

    def show_wrench_plot_page(self):
        """切到六维力页面并确保两幅实时曲线可见。"""
        stack = self._find_child("plotStackedWidget")
        experiment_button = self._find_child("plotExperimentPageBtn")
        wrench_button = self._find_child("plotWrenchPageBtn")
        if stack:
            stack.setCurrentIndex(1)
        if experiment_button:
            experiment_button.setChecked(False)
        if wrench_button:
            wrench_button.setChecked(True)
        self.set_wrench_plot_visible(True)

    def reset_wrench_plot(self):
        self._wrench_data.clear()
        self._wrench_time_origin = time.monotonic()
        self._refresh_wrench_plot(force=True)

    def get_memory_coordinate_type(self):
        combo = self._find_child("memoryTypeCombo")
        return "world" if combo and combo.currentIndex() == 1 else "joint"

    @Slot(int)
    def _on_memory_type_changed(self, _index):
        self.memory_type_changed.emit(self.get_memory_coordinate_type())

    @staticmethod
    def _format_memory_number(value):
        return f"{float(value):.3f}".rstrip("0").rstrip(".")

    def _refresh_memory_label(self, index):
        label = self._find_child(f"memVal_{index}")
        if not label:
            return
        memory = self._memory_slots.get(index)
        if memory is None:
            label.setText("--")
            label.setToolTip("未保存位置")
            return
        prefix = "J" if memory.coordinate_type == "joint" else "W"
        values = ", ".join(self._format_memory_number(v) for v in memory.values)
        label.setText(f"{prefix} [{values}]")
        label.setToolTip(
            ("关节角: " if memory.coordinate_type == "joint" else "世界坐标: ")
            + values
        )

    def _refresh_all_memory_labels(self):
        for index in range(1, 6):
            self._refresh_memory_label(index)

    def _save_memory_slot(self, index):
        coordinate_type = self.get_memory_coordinate_type()
        values = self._current_joints if coordinate_type == "joint" else self._current_pose
        self._memory_slots[index] = PositionMemory(coordinate_type, values)
        self._save_memory_slots()
        self._refresh_memory_label(index)
        description = "关节角" if coordinate_type == "joint" else "世界坐标"
        self.statusBar().showMessage(f"P{index} 已保存{description}", 2500)

    def _recall_memory_slot(self, index):
        memory = self._memory_slots.get(index)
        if memory is None:
            self.statusBar().showMessage(f"P{index} 尚未保存位置", 2500)
            return
        combo = self._find_child("memoryTypeCombo")
        if combo:
            combo.setCurrentIndex(0 if memory.coordinate_type == "joint" else 1)
        template = (
            "robotJointTarget{axis}_global"
            if memory.coordinate_type == "joint"
            else "robotTarget{axis}_global"
        )
        axes = JOINT_AXES if memory.coordinate_type == "joint" else WORLD_AXES
        for axis, value in zip(axes, memory.values):
            widget = self._find_child(template.format(axis=axis))
            if widget:
                widget.setValue(value)
        self.memory_recall_requested.emit(index, memory)
        self.statusBar().showMessage(f"P{index} 已载入目标输入框", 2500)

    def _clear_memory_slot(self, index):
        self._memory_slots.pop(index, None)
        self._save_memory_slots()
        self._refresh_memory_label(index)
        self.statusBar().showMessage(f"P{index} 已清除", 2500)

    def _load_memory_slots(self):
        """从本机设置恢复 P1-P5；数据异常时保留 P1 的出厂默认值。"""
        raw = self._memory_settings.value("position_memory/slots", "")
        if not raw:
            return
        try:
            saved = json.loads(str(raw))
            restored = {}
            for slot_text, item in saved.items():
                slot = int(slot_text)
                coordinate_type = str(item["coordinate_type"])
                values = tuple(float(value) for value in item["values"])
                if slot not in range(1, 6):
                    continue
                if coordinate_type not in ("joint", "world") or len(values) != 6:
                    continue
                self._ensure_finite(values, "位置记忆")
                restored[slot] = PositionMemory(coordinate_type, values)
            self._memory_slots = restored
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return

    def _migrate_legacy_registry_settings(self):
        """首次运行时把旧版注册表记忆复制到 V3 独立 INI 文件。"""
        marker = "meta/legacy_registry_imported"
        if self._memory_settings.value(marker, False, type=bool):
            return

        legacy = QSettings("JLU", "RoboticArmV1")
        for key in legacy.allKeys():
            if not key.startswith(("position_memory/", "test_parameters/")):
                continue
            if not self._memory_settings.contains(key):
                self._memory_settings.setValue(key, legacy.value(key))
        self._memory_settings.setValue(marker, True)
        self._memory_settings.sync()

    def _save_memory_slots(self):
        payload = {
            str(slot): {
                "coordinate_type": memory.coordinate_type,
                "values": list(memory.values),
            }
            for slot, memory in sorted(self._memory_slots.items())
        }
        self._memory_settings.setValue(
            "position_memory/slots",
            json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
        )
        self._memory_settings.sync()

    @staticmethod
    def _test_parameter_key(object_name):
        return f"test_parameters/{object_name}"

    def _load_test_parameters(self):
        """恢复三种试验输入参数，异常值交由控件范围自动约束。"""
        for object_name in TEST_PARAMETER_WIDGETS:
            widget = self._find_child(object_name)
            if widget is None:
                continue
            saved = self._memory_settings.value(
                self._test_parameter_key(object_name),
                widget.value(),
            )
            try:
                widget.setValue(float(saved))
            except (TypeError, ValueError):
                continue

    def _bind_test_parameter_memory(self):
        """参数一经修改即保存，意外退出时也能保留最近一次输入。"""
        for object_name in TEST_PARAMETER_WIDGETS:
            widget = self._find_child(object_name)
            if widget is None:
                continue
            widget.valueChanged.connect(
                lambda value, name=object_name: self._save_test_parameter(
                    name, value
                )
            )

    def _save_test_parameter(self, object_name, value):
        self._memory_settings.setValue(
            self._test_parameter_key(object_name),
            float(value),
        )
        self._memory_settings.sync()

    def _save_test_parameters(self):
        for object_name in TEST_PARAMETER_WIDGETS:
            widget = self._find_child(object_name)
            if widget is not None:
                self._memory_settings.setValue(
                    self._test_parameter_key(object_name),
                    float(widget.value()),
                )
        self._memory_settings.sync()

    def closeEvent(self, event):
        self._save_test_parameters()
        self._save_memory_slots()
        super().closeEvent(event)

    def get_robot_speed(self):
        """兼容旧调用；第二版默认返回末端绝对运动速度。"""
        spin = self._find_child("robotSpeedSpin_global")
        return float(spin.value()) if spin else 5.0

    def set_motion_controls_enabled(self, enabled):
        for object_name in (
            "robotBtnPTP_zengliang",
            "robotBtnPTP_global",
            "robotJointBtnPTP_zengliang",
            "robotJointBtnPTP_global",
        ):
            button = self._find_child(object_name)
            if button:
                button.setEnabled(bool(enabled))

    def set_plot_visible(self, ax_index, visible):
        canvas = self.canvas1 if ax_index == 0 else self.canvas2
        canvas.setVisible(bool(visible))

    def reset_plot(self, ax_index):
        if ax_index == 0:
            self._disp_data.clear()
        else:
            self._ang_data.clear()
        # 绕过10 FPS节流，重置操作必须立即反映到界面。
        self._refresh_plot(ax_index, force=True)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
