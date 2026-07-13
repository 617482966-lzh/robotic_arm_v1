# -*- coding: utf-8 -*-
"""机械臂控制与力传感数据采集 GUI - PySide6 + Matplotlib + .ui"""

import os
from datetime import datetime

from PySide6.QtWidgets import (
    QMainWindow, QApplication, QWidget, QFileDialog,
)
from PySide6.QtCore import Qt, Signal, Slot
from PySide6.QtUiTools import QUiLoader

import matplotlib
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

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
QPushButton#saveBtnSave { background-color: #1a5a5a; border-color: #14a3a8; min-height: 30px; font-size: 14px; font-weight: bold; }
QPushButton#saveBtnSave:hover { background-color: #1a7a7a; }
QPushButton#robotBtnEnable { background-color: #1a5a3c; border-color: #28a745; min-height: 34px; font-size: 14px; font-weight: bold; }
QPushButton#robotBtnEnable:checked { background-color: #28a745; }
QPushButton#testDispStart { background-color: #1a5a5a; border-color: #14a3a8; font-weight: bold; }
QPushButton#testDispStart:hover { background-color: #1a7a7a; }
QPushButton#testShearStart { background-color: #5a3a1a; border-color: #e67e22; font-weight: bold; }
QPushButton#testShearStart:hover { background-color: #7a5a1a; }
QLineEdit { background-color: #12122a; color: #e0e0e0; border: 1px solid #3a3a52; border-radius: 4px; padding: 4px 8px; font-size: 13px; }
QLineEdit:focus { border-color: #14a3a8; }
QDoubleSpinBox, QSpinBox { background-color: #12122a; color: #e0e0e0; border: 1px solid #3a3a52; border-radius: 4px; padding: 4px 6px; font-size: 13px; }
QDoubleSpinBox:focus, QSpinBox:focus { border-color: #14a3a8; }
QComboBox { background-color: #12122a; color: #e0e0e0; border: 1px solid #3a3a52; border-radius: 4px; padding: 4px 8px; font-size: 13px; }
QComboBox:hover { border-color: #14a3a8; }
QComboBox::drop-down { border: none; width: 22px; }
QComboBox QAbstractItemView { background-color: #1c1c2e; color: #c8ccd4; selection-background-color: #14a3a8; selection-color: #ffffff; border: 1px solid #3a3a52; }
QSlider::groove:horizontal { background: #2d2d46; height: 6px; border-radius: 3px; }
QSlider::handle:horizontal { background: #14a3a8; width: 16px; margin: -5px 0; border-radius: 8px; }
QSlider::handle:horizontal:hover { background: #1ac5cc; }
QSlider::sub-page:horizontal { background: #14a3a8; border-radius: 3px; }
QRadioButton { color: #a0a4b0; spacing: 4px; }
QRadioButton::indicator { width: 14px; height: 14px; border-radius: 7px; border: 2px solid #3a3a52; background: #12122a; }
QRadioButton::indicator:checked { background: #14a3a8; border-color: #14a3a8; }
QScrollArea { border: none; background: transparent; }
QScrollBar:vertical { background: #1c1c2e; width: 8px; border-radius: 4px; }
QScrollBar::handle:vertical { background: #3a3a52; border-radius: 4px; min-height: 30px; }
QScrollBar::handle:vertical:hover { background: #14a3a8; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QSplitter::handle { background: #2a2a40; width: 3px; }
QFrame#saveBar { background-color: #252538; border-radius: 4px; padding: 6px; }
QFrame#plotPlaceholder1 { background-color: #1c1c2e; border: 1px solid #3a3a52; border-radius: 4px; }
"""

LIGHT_ON = "background-color:#00c853;border-radius:6px;min-width:12px;max-width:12px;min-height:12px;max-height:12px;"
LIGHT_OFF = "background-color:#ff5252;border-radius:6px;min-width:12px;max-width:12px;min-height:12px;max-height:12px;"

UI_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "main_window.ui")


class MainWindow(QMainWindow):
    # ---- Signals ----
    robot_connect_requested = Signal(str, int)       # ip, port
    robot_disconnect_requested = Signal()
    sensor_connect_requested = Signal(str, int, int) # port, baudrate, slave_addr
    sensor_disconnect_requested = Signal()
    jog_cmd = Signal(str, str)                       # axis, direction
    jog_stop_cmd = Signal()
    robot_move_cmd = Signal(str, float)              # axis, target_value
    speed_changed = Signal(float)
    ang_speed_changed = Signal(float)
    step_changed = Signal(float)
    robot_enable_changed = Signal(bool)
    stop_requested = Signal()
    home_requested = Signal()
    ptp_requested = Signal()
    save_position_cmd = Signal(int)
    recall_position_cmd = Signal(int)
    clear_position_cmd = Signal(int)
    zero_all_requested = Signal()
    zero_pressure_requested = Signal()
    zero_torque_requested = Signal()
    disp_test_start = Signal()                       # 贯入试验启动
    shear_test_start = Signal()                      # 剪切试验启动
    save_data_requested = Signal(str)
    sensor_refresh_requested = Signal()
    plot_disp_show = Signal()
    plot_disp_close = Signal()
    plot_disp_reset = Signal()
    plot_ang_show = Signal()
    plot_ang_close = Signal()
    plot_ang_reset = Signal()

    def __init__(self):
        super().__init__()
        self._setup_ui_from_file()
        self._embed_matplotlib()
        self._apply_style()
        self._connect_signals()
        self._init_state()

    def _setup_ui_from_file(self):
        loader = QUiLoader()
        widget = loader.load(UI_PATH, self)
        if widget is None:
            raise RuntimeError(f"无法加载 UI 文件: {UI_PATH}")
        self.setCentralWidget(widget)
        self.setWindowTitle("机械臂控制与力传感数据采集系统")
        self.resize(1920, 1080)
        self.setMinimumSize(1400, 800)

    def _find_child(self, name):
        return self.findChild(QWidget, name)

    def _embed_matplotlib(self):
        # Figure 1: displacement-force (in plotDispGroup)
        self.figure1 = Figure(figsize=(8, 4), dpi=100, facecolor="#1c1c2e")
        self.ax1 = self.figure1.add_subplot(1, 1, 1)
        self.ax1.set_facecolor("#12122a")
        self.ax1.set_title("末端位移 — 力  End-effector Displacement vs Force",
                           color="#c8ccd4", fontsize=12, fontweight="bold")
        self.ax1.set_xlabel("位移 Displacement (mm)", color="#a0a4b0")
        self.ax1.set_ylabel("力 Force (N)", color="#a0a4b0")
        self.ax1.grid(True, alpha=0.25, color="#3a3a52")
        self.ax1.tick_params(colors="#a0a4b0")
        for spine in self.ax1.spines.values():
            spine.set_color("#3a3a52")
        self.line1, = self.ax1.plot([], [], "#14a3a8", linewidth=1.5, label="位移-力")
        self.ax1.legend(loc="upper left", facecolor="#1c1c2e",
                        edgecolor="#3a3a52", labelcolor="#c8ccd4", fontsize=9)
        self.figure1.tight_layout(pad=2.5)

        self.canvas1 = FigureCanvas(self.figure1)
        self.canvas1.setStyleSheet("background-color:#1c1c2e;")
        p1 = self._find_child("plotPlaceholder1")
        if p1:
            parent_layout = p1.parentWidget().layout()
            idx = parent_layout.indexOf(p1)
            if idx >= 0:
                parent_layout.removeWidget(p1)
                p1.hide()
                parent_layout.insertWidget(idx, self.canvas1)

        # Figure 2: angular-torque (in plotAngGroup)
        self.figure2 = Figure(figsize=(8, 4), dpi=100, facecolor="#1c1c2e")
        self.ax2 = self.figure2.add_subplot(1, 1, 1)
        self.ax2.set_facecolor("#12122a")
        self.ax2.set_title("角位移 — 扭矩  Angular Displacement vs Torque",
                           color="#c8ccd4", fontsize=12, fontweight="bold")
        self.ax2.set_xlabel("角位移 Angular (°)", color="#a0a4b0")
        self.ax2.set_ylabel("扭矩 Torque (N·m)", color="#a0a4b0")
        self.ax2.grid(True, alpha=0.25, color="#3a3a52")
        self.ax2.tick_params(colors="#a0a4b0")
        for spine in self.ax2.spines.values():
            spine.set_color("#3a3a52")
        self.line2, = self.ax2.plot([], [], "#e67e22", linewidth=1.5, label="角位移-扭矩")
        self.ax2.legend(loc="upper left", facecolor="#1c1c2e",
                        edgecolor="#3a3a52", labelcolor="#c8ccd4", fontsize=9)
        self.figure2.tight_layout(pad=2.5)

        self.canvas2 = FigureCanvas(self.figure2)
        self.canvas2.setStyleSheet("background-color:#1c1c2e;")
        p2 = self._find_child("plotPlaceholder2")
        if p2:
            parent_layout = p2.parentWidget().layout()
            idx = parent_layout.indexOf(p2)
            if idx >= 0:
                parent_layout.removeWidget(p2)
                p2.hide()
                parent_layout.insertWidget(idx, self.canvas2)

        self._disp_data = []
        self._ang_data = []
    def _apply_style(self):
        self.setStyleSheet(DARK_STYLE)

    def _connect_signals(self):
        # Robot connection
        btn = self._find_child("robotBtnConnect")
        if btn: btn.clicked.connect(self._on_robot_connect)
        btn = self._find_child("robotBtnDisconnect")
        if btn: btn.clicked.connect(self._on_robot_disconnect)

        # Sensor connection (serial)
        btn = self._find_child("sensorBtnConnect")
        if btn: btn.clicked.connect(self._on_sensor_connect)
        btn = self._find_child("sensorBtnDisconnect")
        if btn: btn.clicked.connect(self._on_sensor_disconnect)

        # Speed sync (1-100 mm/s)
        slider = self._find_child("robotSpeedSlider")
        spin = self._find_child("robotSpeedSpin")
        if slider and spin:
            slider.valueChanged.connect(spin.setValue)
            spin.valueChanged.connect(slider.setValue)
            spin.valueChanged.connect(lambda v: self.speed_changed.emit(float(v)))

        ang_slider = self._find_child("robotAngSpeedSlider")
        ang_spin = self._find_child("robotAngSpeedSpin")
        if ang_slider and ang_spin:
            ang_slider.valueChanged.connect(ang_spin.setValue)
            ang_spin.valueChanged.connect(ang_slider.setValue)
            ang_spin.valueChanged.connect(lambda v: self.ang_speed_changed.emit(float(v)))

        # Step
        combo = self._find_child("robotStepCombo")
        if combo:
            combo.currentTextChanged.connect(self._on_step_changed)

        # Jog buttons + Target move buttons (merged in one layout)
        for axis in ["X", "Y", "Z", "Rx", "Ry", "Rz"]:
            for dir_suffix, direction in [("neg", "-"), ("pos", "+")]:
                btn = self._find_child(f"jogBtn_{axis}_{dir_suffix}")
                if btn:
                    btn.pressed.connect(lambda a=axis, d=direction: self.jog_cmd.emit(a, d))
                    btn.released.connect(self.jog_stop_cmd.emit)
                    if dir_suffix == "neg":
                        btn.setText("\u2212")
                        btn.setStyleSheet("QPushButton{font-size:15px;font-weight:bold;color:#f44336;}QPushButton:hover{background-color:#3a3a5a;}")
                    else:
                        btn.setStyleSheet("QPushButton{font-size:15px;font-weight:bold;color:#4caf50;}QPushButton:hover{background-color:#3a3a5a;}")
            # Move button for target position
            btn_move = self._find_child(f"robotMove{axis}")
            if btn_move:
                btn_move.clicked.connect(lambda c, a=axis: self._on_robot_move(a))

        # Memory
        for i in range(1, 6):
            bs = self._find_child(f"memSave_{i}")
            br = self._find_child(f"memRecall_{i}")
            bc = self._find_child(f"memClear_{i}")
            if bs: bs.clicked.connect(lambda c, idx=i: self.save_position_cmd.emit(idx))
            if br: br.clicked.connect(lambda c, idx=i: self.recall_position_cmd.emit(idx))
            if bc:
                bc.clicked.connect(lambda c, idx=i: self.clear_position_cmd.emit(idx))
                bc.setText("\u00d7")

        # Actions
        btn_enable = self._find_child("robotBtnEnable")
        if btn_enable: btn_enable.toggled.connect(self.robot_enable_changed.emit)
        btn_home = self._find_child("robotBtnHome")
        if btn_home: btn_home.clicked.connect(self.home_requested.emit)
        btn_ptp = self._find_child("robotBtnPTP")
        if btn_ptp: btn_ptp.clicked.connect(self.ptp_requested.emit)
        btn_stop = self._find_child("robotBtnStop")
        if btn_stop:
            btn_stop.clicked.connect(self.stop_requested.emit)
            btn_stop.clicked.connect(lambda: self.set_robot_enabled(False))

        # Sensor zero buttons
        btn_zero = self._find_child("sensorBtnZeroAll")
        if btn_zero: btn_zero.clicked.connect(self.zero_all_requested.emit)
        btn_zp = self._find_child("sensorBtnZeroPressure")
        if btn_zp: btn_zp.clicked.connect(self.zero_pressure_requested.emit)
        btn_zt = self._find_child("sensorBtnZeroTorque")
        if btn_zt: btn_zt.clicked.connect(self.zero_torque_requested.emit)

        # Test section
        btn = self._find_child("testDispStart")
        if btn: btn.clicked.connect(self.disp_test_start.emit)
        btn = self._find_child("testShearStart")
        if btn: btn.clicked.connect(self.shear_test_start.emit)

        # Refresh button
        btn_refresh = self._find_child("sensorRefreshBtn")
        if btn_refresh: btn_refresh.clicked.connect(self.sensor_refresh_requested.emit)

        # Plot control buttons
        for prefix, sigs in [("plotDisp", (self.plot_disp_show, self.plot_disp_close, self.plot_disp_reset)),
                            ("plotAng", (self.plot_ang_show, self.plot_ang_close, self.plot_ang_reset))]:
            for suffix, sig in zip(["Show", "Close", "Reset"], sigs):
                btn = self._find_child(f"{prefix}{suffix}")
                if btn: btn.clicked.connect(sig.emit)

        # Test save/reset/browse
        for prefix in ["testDisp", "testShear"]:
            btn_browse = self._find_child(f"{prefix}SaveBrowse")
            if btn_browse: btn_browse.clicked.connect(lambda c, p=prefix: self._on_test_browse(p))
            btn_save = self._find_child(f"{prefix}Save")
            if btn_save: btn_save.clicked.connect(lambda c, p=prefix: self._on_test_save(p))
            btn_reset = self._find_child(f"{prefix}Reset")
            if btn_reset: btn_reset.clicked.connect(lambda c, p=prefix: print(f"{p} reset clicked"))

    def _init_state(self):
        for name in ["robotStatusLight", "sensorStatusLight"]:
            w = self._find_child(name)
            if w: w.setStyleSheet(LIGHT_OFF)
        # Init test save paths
        for prefix in ["testDisp", "testShear"]:
            sp = self._find_child(f"{prefix}SavePath")
            if sp: sp.setText(os.path.expanduser("~") + "\\Documents")

    # ---- Connection callbacks ----
    def _on_robot_connect(self):
        ip_edit = self._find_child("robotIpEdit")
        port_edit = self._find_child("robotPortEdit")
        ip = ip_edit.text().strip() if ip_edit else "192.168.1.4"
        try: port = int(port_edit.text().strip()) if port_edit else 9760
        except ValueError: port = 502
        self.robot_connect_requested.emit(ip, port)
        self._set_robot_connected(True)

    def _on_robot_disconnect(self):
        self.robot_disconnect_requested.emit()
        self._set_robot_connected(False)

    def _on_sensor_connect(self):
        port_w = self._find_child("sensorComPort")
        port = port_w.text().strip() if port_w else "COM3"
        self.sensor_connect_requested.emit(port, 57600, 1)
        self._set_sensor_connected(True)

    def _on_sensor_disconnect(self):
        self.sensor_disconnect_requested.emit()
        self._set_sensor_connected(False)

    def _on_step_changed(self, text):
        self.step_changed.emit(0.0 if text == "连续" else float(text))

    def _on_robot_move(self, axis):
        """Target position move"""
        spin = self._find_child(f"robotTarget{axis}")
        if spin:
            target = spin.value()
            self.robot_move_cmd.emit(axis, target)

    def _on_test_browse(self, prefix):
        path_edit = self._find_child(f"{prefix}SavePath")
        current = path_edit.text() if path_edit else os.path.expanduser("~")
        d = QFileDialog.getExistingDirectory(self, "选择保存目录", current)
        if d and path_edit: path_edit.setText(d)

    def _on_test_save(self, prefix):
        path_edit = self._find_child(f"{prefix}SavePath")
        name_edit = self._find_child(f"{prefix}FileName")
        save_dir = path_edit.text() if path_edit else os.path.expanduser("~")
        name = name_edit.text().strip() if name_edit else "01"
        if not name: name = "01"
        filepath = os.path.join(save_dir, f"{name}.xlsx")
        self.save_data_requested.emit(filepath)

    # ---- UI state ----
    def _set_robot_connected(self, state):
        for wname, enable in [("robotBtnConnect", not state), ("robotBtnDisconnect", state)]:
            w = self._find_child(wname)
            if w: w.setEnabled(enable)
        light = self._find_child("robotStatusLight")
        label = self._find_child("robotStatusLabel")
        if light: light.setStyleSheet(LIGHT_ON if state else LIGHT_OFF)
        if label:
            label.setText("已连接" if state else "未连接")
            label.setStyleSheet("color:#00c853;" if state else "color:#ff5252;")

    def _set_sensor_connected(self, state):
        for wname, enable in [("sensorBtnConnect", not state), ("sensorBtnDisconnect", state)]:
            w = self._find_child(wname)
            if w: w.setEnabled(enable)
        light = self._find_child("sensorStatusLight")
        label = self._find_child("sensorStatusLabel")
        if light: light.setStyleSheet(LIGHT_ON if state else LIGHT_OFF)
        if label:
            label.setText("已连接" if state else "未连接")
            label.setStyleSheet("color:#00c853;" if state else "color:#ff5252;")

    def set_robot_enabled(self, state):
        btn = self._find_child("robotBtnEnable")
        if btn:
            btn.setChecked(state)
            btn.setText("已使能" if state else "使能")

    # ---- Robot pose update ----
    def update_pose(self, x, y, z, rx, ry, rz):
        for axis, val in zip(["X","Y","Z","Rx","Ry","Rz"], [x,y,z,rx,ry,rz]):
            spin = self._find_child(f"poseSpin_{axis}")
            if spin: spin.setValue(val)

    # ---- Sensor data update (Fz, Tz only) ----
    def update_force_torque(self, fz, tz):
        lbl = self._find_child("sensorForceVal_Fz")
        if lbl: lbl.setText(f"{fz:.3f}")
        lbl = self._find_child("sensorTorqueVal_Tz")
        if lbl: lbl.setText(f"{tz:.3f}")

    # ---- Memory ----
    def set_memory_slot(self, index, text):
        lbl = self._find_child(f"memVal_{index}")
        if lbl: lbl.setText(text)

    # ---- Getters ----
    def get_robot_speed(self):
        spin = self._find_child("robotSpeedSpin")
        return float(spin.value()) if spin else 50.0

    def get_robot_step(self):
        combo = self._find_child("robotStepCombo")
        if combo:
            t = combo.currentText()
            return 0.0 if t == "连续" else float(t)
        return 1.0

    def get_robot_coord(self):
        rb = self._find_child("robotRbBase")
        return "base" if (rb and rb.isChecked()) else "tool"

    def get_test_params_disp(self):
        """Returns (speed, distance, max_force) for penetration test"""
        speed = self._find_child("testDispSpeed")
        dist = self._find_child("testDispDist")
        force_max = self._find_child("testForceMax")
        return (
            speed.value() if speed else 1.0,
            dist.value() if dist else 10.0,
            force_max.value() if force_max else 50.0,
        )

    def get_test_params_shear(self):
        """Returns (ang_speed, ang_dist, max_torque) for shear test"""
        ang_speed = self._find_child("testAngSpeed")
        ang_dist = self._find_child("testAngDist")
        torque_max = self._find_child("testTorqueMax")
        return (
            ang_speed.value() if ang_speed else 5.0,
            ang_dist.value() if ang_dist else 30.0,
            torque_max.value() if torque_max else 5.0,
        )

    # ---- Plot data ----
    def add_disp_force(self, displacement, force):
        self._disp_data.append((displacement, force))
        if len(self._disp_data) > 5000:
            self._disp_data = self._disp_data[-5000:]
        xs = [d[0] for d in self._disp_data]
        ys = [d[1] for d in self._disp_data]
        self.line1.set_data(xs, ys)
        self.ax1.relim(); self.ax1.autoscale_view()
        self.canvas1.draw_idle()

    def add_ang_torque(self, angular, torque):
        self._ang_data.append((angular, torque))
        if len(self._ang_data) > 5000:
            self._ang_data = self._ang_data[-5000:]
        xs = [d[0] for d in self._ang_data]
        ys = [d[1] for d in self._ang_data]
        self.line2.set_data(xs, ys)
        self.ax2.relim(); self.ax2.autoscale_view()
        self.canvas2.draw_idle()

    # ---- Plot control methods ----
    def set_plot_visible(self, ax_index, visible):
        """预留接口：显示/关闭按钮暂不对图表进行控制"""
        pass

    def reset_plot(self, ax_index):
        if ax_index == 0:
            self._disp_data.clear()
            self.line1.set_data([], [])
            self.ax1.relim()
            self.ax1.autoscale_view()
        else:
            self._ang_data.clear()
            self.line2.set_data([], [])
            self.ax2.relim()
            self.ax2.autoscale_view()
        self.canvas1.draw_idle()
    def clear_plots(self):
        self._disp_data.clear()
        self._ang_data.clear()
        self.line1.set_data([], [])
        self.line2.set_data([], [])
        self.ax1.relim(); self.ax1.autoscale_view()
        self.ax2.relim(); self.ax2.autoscale_view()
        self.canvas1.draw_idle()
        self.canvas2.draw_idle()

    def save_figure(self, filepath):
        self.figure1.savefig(filepath, dpi=150, bbox_inches="tight",
                            facecolor="#1c1c2e", edgecolor="none")


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MainWindow()
    w.show()
    sys.exit(app.exec())
