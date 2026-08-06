# -*- coding: utf-8 -*-
"""机械臂控制与六维力传感数据采集 V3 主程序。"""

import itertools
import os
import queue
import subprocess
import sys
import tempfile
import threading
import time
import ctypes
from pathlib import Path

# 归档应用可以直接从本目录启动；通信模块仍由项目根目录统一维护。
IS_FROZEN = bool(getattr(sys, "frozen", False))
BUNDLE_DIR = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
APP_DIR = (
    Path(sys.executable).resolve().parent
    if IS_FROZEN
    else Path(__file__).resolve().parent
)
PROJECT_ROOT = BUNDLE_DIR if IS_FROZEN else APP_DIR.parents[1]
for module_path in (str(PROJECT_ROOT), str(BUNDLE_DIR), str(APP_DIR)):
    if module_path not in sys.path:
        sys.path.insert(0, module_path)

import serial.tools.list_ports
from openpyxl import Workbook

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt, QThread, QTimer, Signal

from main_window_3 import MainWindow

from sensor_6.communication import SensorCommunication
from sensor_6.controller import SensorController
from robot_client.robot_control import BorunteRobot as RobotComm


SAMPLE_INTERVAL_MS = 50  # 20 Hz
GUI_READY_TIMEOUT_SECONDS = 10.0
GUI_CHILD_ENV = "ROBOTIC_ARM_GUI_CHILD"
GUI_READY_FILE_ENV = "ROBOTIC_ARM_GUI_READY_FILE"
WINDOWS_APP_USER_MODEL_ID = "JLU.RoboticArm.ControlSystem.V3"


def configure_windows_app_identity():
    """在创建QApplication前设置Windows任务栏应用身份。

    直接通过python.exe启动时，Windows默认会沿用Python宿主图标。独立的
    AppUserModelID使任务栏使用本窗口设置的吉林大学图标进行分组和显示。
    """
    if sys.platform != "win32":
        return
    try:
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(
            WINDOWS_APP_USER_MODEL_ID
        )
    except (AttributeError, OSError):
        # 老版本Windows或受限会话中失败时，Qt窗口图标仍然可以正常使用。
        pass


def find_serial_a_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "SERIAL-A" in p.description or "SERIAL-A" in p.device:
            return f"{p.device} - {p.description}"
    return None


def extract_port_name(full_text):
    if not full_text:
        return full_text
    return full_text.split(" - ")[0].strip()


class SensorWorker(QThread):
    data_ready = Signal(float, float, float, float, float, float)
    error_occurred = Signal(str)

    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.controller = controller
        self._stop_event = threading.Event()
        self._last_error = None
        self._last_error_at = 0.0

    def run(self):
        next_sample = time.monotonic()
        while not self._stop_event.is_set():
            try:
                values = self.controller.monitor_6_sensor()
                if all(value is not None for value in values):
                    self.data_ready.emit(*values)
                else:
                    error = getattr(self.controller, "last_error", None)
                    if error is not None:
                        self._emit_error_limited(str(error))
            except Exception as e:
                self._emit_error_limited(str(e))
            next_sample += SAMPLE_INTERVAL_MS / 1000.0
            delay = next_sample - time.monotonic()
            if delay > 0:
                self._stop_event.wait(delay)
            else:
                next_sample = time.monotonic()

    def _emit_error_limited(self, message):
        """相同错误最多每秒通知一次，避免断线时堵塞Qt事件队列。"""
        now = time.monotonic()
        if message != self._last_error or now - self._last_error_at >= 1.0:
            self._last_error = message
            self._last_error_at = now
            self.error_occurred.emit(message)

    def request_stop(self):
        self._stop_event.set()

    def stop(self, timeout_ms=2000):
        self.request_stop()
        if QThread.currentThread() is not self:
            return self.wait(timeout_ms)
        return True


class RobotWorker(QThread):
    """独占机械臂 socket 的后台工作线程。"""

    connected = Signal(str, int)
    disconnected = Signal()
    pose_ready = Signal(object)
    joints_ready = Signal(object)
    command_done = Signal(str)
    error_occurred = Signal(str)

    def __init__(self, host, port, poll_interval=0.05):
        super().__init__()
        self.host = host
        self.port = int(port)
        self.poll_interval = float(poll_interval)
        self.robot = None
        self._stop_event = threading.Event()
        self._commands = queue.PriorityQueue()
        self._sequence = itertools.count()

    def submit(self, command, *args, urgent=False):
        if urgent and command in {"stop", "safe_stop", "disconnect"}:
            while True:
                try:
                    self._commands.get_nowait()
                except queue.Empty:
                    break
        priority = 0 if urgent else 10
        self._commands.put((priority, next(self._sequence), command, args))

    def stop(self, wait=False):
        self._stop_event.set()
        self.submit("disconnect", urgent=True)
        if wait and QThread.currentThread() is not self:
            self.wait(3000)

    def _execute(self, command, args):
        actions = {
            "world_increment": self.robot.move_world_increment,
            "world_absolute": self.robot.move_world_absolute,
            "tool_vector_line": self.robot.move_tool_vector_interpolated,
            "move_along_tool_x": self.robot.move_along_tool_x,
            "joint_increment": self.robot.move_joints_increment,
            "joint_absolute": self.robot.move_joints_absolute,
            "enable": self.robot.enable,
            "disable": self.robot.disable,
            "home": self.robot.home,
            "stop": self.robot.emergency_stop,
            "safe_stop": self.robot.safe_stop_and_clear,
        }
        if command == "disconnect":
            self._stop_event.set()
            return
        action = actions.get(command)
        if action is None:
            raise ValueError(f"未知机械臂命令: {command}")
        result = action(*args)
        if result is None or result is False:
            raise RuntimeError(f"机械臂未确认命令: {command}")
        self.command_done.emit(command)

    def run(self):
        try:
            self.robot = RobotComm(self.host, self.port)
            self.robot.connect()
            self.robot.configure_calibrated_speed_control()
            self.connected.emit(self.host, self.port)
            next_poll = 0.0
            while not self._stop_event.is_set():
                command_executed = False
                try:
                    _, _, command, args = self._commands.get_nowait()
                    try:
                        self._execute(command, args)
                    except Exception as exc:
                        self.error_occurred.emit(str(exc))
                    command_executed = True
                except queue.Empty:
                    pass
                now = time.monotonic()
                if now >= next_poll:
                    pose, joints = self.robot.read_realtime_state()
                    self.pose_ready.emit(pose)
                    self.joints_ready.emit(joints)
                    next_poll = now + self.poll_interval
                else:
                    if not command_executed or self._commands.empty():
                        self._stop_event.wait(min(0.01, next_poll - now))
        except Exception as exc:
            if not self._stop_event.is_set():
                self.error_occurred.emit(str(exc))
        finally:
            if self.robot:
                self.robot.disconnect()
            self.robot = None
            self.disconnected.emit()


class XlsxExportWorker(QThread):
    """Write test rows without blocking the GUI thread."""

    saved = Signal(str)
    failed = Signal(str)

    def __init__(self, filepath, headers, rows, parent=None):
        super().__init__(parent)
        self.filepath = str(filepath)
        self.headers = tuple(headers)
        self.rows = tuple(tuple(row) for row in rows)

    def run(self):
        workbook = None
        try:
            path = Path(self.filepath)
            path.parent.mkdir(parents=True, exist_ok=True)
            workbook = Workbook(write_only=True)
            sheet = workbook.create_sheet("试验数据")
            sheet.append(self.headers)
            for row in self.rows:
                sheet.append(row)
            workbook.save(path)
            workbook.close()
            workbook = None
            self.saved.emit(str(path))
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            if workbook is not None:
                workbook.close()


class AppController:

    def __init__(self):
        configure_windows_app_identity()
        self.app = QApplication(sys.argv)
        self.app.setAttribute(Qt.AA_Use96Dpi)
        self.app.setStyle("Fusion")

        self.window = MainWindow()
        self.sensor_comm = None
        self.sensor_ctrl = None
        self.robot_worker = None
        self.worker = None
        self._latest_pose = None
        self._latest_joints = None
        self._latest_wrench = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._active_test = None
        self._test_data = {
            "testDisp": [],
            "testShear": [],
            "testShovel": [],
        }
        self._export_workers = set()

        self._sample_timer = QTimer(self.window)
        self._sample_timer.setInterval(SAMPLE_INTERVAL_MS)
        self._sample_timer.timeout.connect(self._sample_active_test)

        self._connect_signals()
        self._auto_detect_port()
        self.window.set_motion_controls_enabled(False)
        self.window.clear_robot_realtime_display()
        self.app.aboutToQuit.connect(self._shutdown)
        self.window.showMaximized()

    def _auto_detect_port(self):
        port = find_serial_a_port()
        if port:
            com_edit = self.window._find_child("sensorComPort")
            if com_edit:
                com_edit.setText(port)
            self.window.statusBar().showMessage(f"检测到传感器串口: {port}", 5000)
        else:
            self.window.statusBar().showMessage("未检测到 SERIAL-A 串口，请手动输入", 8000)

    def _connect_signals(self):
        self.window.sensor_connect_requested.connect(self._on_sensor_connect)
        self.window.sensor_disconnect_requested.connect(self._on_sensor_disconnect)
        self.window.sensor_refresh_requested.connect(self._on_sensor_refresh)

        self.window.zero_all_requested.connect(self._on_zero_all)
        self.window.sensor_zero_channel_requested.connect(self._on_zero_channel)

        self.window.robot_connect_requested.connect(self._on_robot_connect)
        self.window.robot_disconnect_requested.connect(self._on_robot_disconnect)
        self.window.world_increment_requested.connect(
            lambda request: self._submit_robot("world_increment", request.pose, request.speed_mm_s)
        )
        self.window.world_absolute_requested.connect(
            lambda request: self._submit_robot("world_absolute", request.pose, request.speed_mm_s)
        )
        self.window.joint_increment_requested.connect(
            lambda request: self._submit_robot("joint_increment", request.joints, request.speed_deg_s)
        )
        self.window.joint_absolute_requested.connect(
            lambda request: self._submit_robot("joint_absolute", request.joints, request.speed_deg_s)
        )
        self.window.home_requested.connect(self._on_robot_home)
        self.window.stop_requested.connect(self._on_robot_stop)
        self.window.robot_enable_changed.connect(self._on_robot_enable)

        self.window.plot_disp_show.connect(lambda: self.window.set_plot_visible(0, True))
        self.window.plot_disp_close.connect(lambda: self.window.set_plot_visible(0, False))
        self.window.plot_disp_reset.connect(lambda: self.window.reset_plot(0))
        self.window.plot_ang_show.connect(lambda: self.window.set_plot_visible(1, True))
        self.window.plot_ang_close.connect(lambda: self.window.set_plot_visible(1, False))
        self.window.plot_ang_reset.connect(lambda: self.window.reset_plot(1))
        self.window.plot_wrench_show.connect(
            lambda: self.window.set_wrench_plot_visible(True)
        )
        self.window.plot_wrench_close.connect(
            lambda: self.window.set_wrench_plot_visible(False)
        )
        self.window.plot_wrench_reset.connect(self.window.reset_wrench_plot)

        self.window.disp_test_start.connect(self._on_disp_test_start)
        self.window.shear_test_start.connect(self._on_shear_test_start)
        self.window.shovel_test_start.connect(self._on_shovel_test_start)
        self.window.test_save_requested.connect(self._save_test_data)
        self.window.test_reset_requested.connect(self._reset_test_data)

    def _on_disp_test_start(self):
        """Start penetration along the current tool-X vector."""
        if not self._can_start_test("贯入"):
            return
        speed, distance, max_force = map(float, self.window.get_test_params_disp())
        direction = RobotComm.penetration_axis_in_world(self._latest_pose[4])
        self._begin_test("testDisp", distance, max_force, direction=direction)
        self._submit_robot("move_along_tool_x", distance, speed)
        self.window.statusBar().showMessage(
            f"贯入试验已启动：{distance:g} mm，{speed:g} mm/s，力阈值{max_force:g} N", 5000
        )

    def _on_shear_test_start(self):
        """Start a J6-only shear test."""
        if not self._can_start_test("剪切"):
            return
        speed, angle, max_torque = map(float, self.window.get_test_params_shear())
        if abs(angle) < 1e-9:
            self.window.statusBar().showMessage("剪切角位移不能为0", 5000)
            return
        self._begin_test("testShear", angle, max_torque)
        self._submit_robot("joint_increment", (0, 0, 0, 0, 0, angle), speed)
        self.window.statusBar().showMessage(
            f"剪切试验已启动：J6 {angle:g} deg，{speed:g} deg/s，扭矩阈值{max_torque:g} N·m", 5000
        )

    def _on_shovel_test_start(self):
        """铲挖页先保留参数入口，轨迹明确前不发送机械臂运动。"""
        speed, max_force = map(float, self.window.get_test_params_shovel())
        self.window.statusBar().showMessage(
            "铲挖试验页面已就绪，但尚未定义运动轨迹、位移距离和结束位置；"
            f"当前参数为{speed:g} mm/s、{max_force:g} N，本次未发送机械臂指令。",
            12000,
        )

    def _can_start_test(self, label):
        if self._active_test is not None:
            self.window.statusBar().showMessage("已有试验正在运行，请先等待或停止", 5000)
            return False
        if not self.robot_worker or self._latest_pose is None or self._latest_joints is None:
            self.window.statusBar().showMessage(f"{label}试验需要先连接机械臂", 5000)
            return False
        return True

    def _begin_test(self, kind, target, limit, direction=None):
        self._test_data[kind].clear()
        self.window.reset_plot(0 if kind == "testDisp" else 1)
        start_pose = tuple(self._latest_pose)
        target_xyz = None
        if direction is not None:
            target_xyz = tuple(
                start_pose[index] + float(target) * direction[index]
                for index in range(3)
            )
        self._active_test = {
            "kind": kind,
            "started": time.monotonic(),
            "target": abs(float(target)),
            "sign": 1.0 if float(target) >= 0 else -1.0,
            "limit": abs(float(limit)),
            "start_pose": start_pose,
            "target_xyz": target_xyz,
            "start_joints": tuple(self._latest_joints),
            "direction": direction,
            "base_wrench": self._latest_wrench if self.worker else (0.0,) * 6,
            "previous_j6": float(self._latest_joints[5]),
            "accumulated_j6": 0.0,
        }
        self._sample_active_test()
        self._sample_timer.start()

    @staticmethod
    def _shortest_angle_delta(current, previous):
        return (float(current) - float(previous) + 180.0) % 360.0 - 180.0

    def _sample_active_test(self):
        state = self._active_test
        if state is None or self._latest_pose is None or self._latest_joints is None:
            return
        pose = tuple(self._latest_pose)
        joints = tuple(self._latest_joints)
        relative_wrench = tuple(
            self._latest_wrench[index] - state["base_wrench"][index]
            for index in range(6)
        ) if self.worker else (0.0,) * 6
        fz = relative_wrench[2]
        tz = relative_wrench[5]
        elapsed_ms = (time.monotonic() - state["started"]) * 1000.0

        if state["kind"] == "testDisp":
            delta = tuple(pose[i] - state["start_pose"][i] for i in range(3))
            projection = sum(delta[i] * state["direction"][i] for i in range(3))
            depth = max(0.0, state["sign"] * projection)
            angle = 0.0
            self.window.add_disp_force(depth, fz)
            remaining = sum(
                (pose[i] - state["target_xyz"][i]) ** 2 for i in range(3)
            ) ** 0.5
            reached = remaining <= 0.3 or depth >= state["target"] - 0.2
            # 保存和曲线使用起点置零后的相对力；安全上限必须与界面显示的
            # 原始Fz比较，否则存在起始预载荷时会延迟甚至完全不触发停止。
            overloaded = abs(self._latest_wrench[2]) >= state["limit"]
            reason = "达到目标世界坐标" if reached else "达到最大力"
        else:
            step = self._shortest_angle_delta(joints[5], state["previous_j6"])
            state["accumulated_j6"] += step
            state["previous_j6"] = joints[5]
            progress = max(0.0, state["sign"] * state["accumulated_j6"])
            angle = state["sign"] * progress
            depth = 0.0
            self.window.add_ang_torque(angle, tz)
            reached = progress >= state["target"] - 0.1
            overloaded = abs(self._latest_wrench[5]) >= state["limit"]
            reason = "达到目标角位移" if reached else "达到最大扭矩"

        self._test_data[state["kind"]].append((
            round(elapsed_ms, 3), round(depth, 6), round(angle, 6),
            *(round(value, 6) for value in relative_wrench),
            *pose, *joints,
        ))
        if (reached or overloaded) and not state.get("stop_requested"):
            self._finish_active_test(reason, send_stop=True)

    def _finish_active_test(self, reason, send_stop):
        if self._active_test is None:
            return
        if send_stop:
            if self._active_test.get("stop_requested"):
                return
            self._active_test["stop_requested"] = True
            self._active_test["stop_reason"] = reason
            self._submit_robot("safe_stop", urgent=True)
            self.window.statusBar().showMessage(
                "正在使用actionPause停止并清空运动队列…", 10000
            )
            return
        kind = self._active_test["kind"]
        clear_on_stop = self._active_test.get("clear_on_stop", False)
        self._active_test = None
        self._sample_timer.stop()
        if clear_on_stop:
            self._test_data[kind].clear()
            self.window.reset_plot(0 if kind == "testDisp" else 1)
        label = {
            "testDisp": "贯入",
            "testShear": "剪切",
            "testShovel": "铲挖",
        }.get(kind, "试验")
        self.window.statusBar().showMessage(
            f"{label}试验结束：{reason}；机械臂保持使能", 12000
        )

    def _reset_test_data(self, kind):
        if self._active_test and self._active_test["kind"] == kind:
            self._active_test["clear_on_stop"] = True
            self._finish_active_test("用户重置", send_stop=True)
            return
        self._test_data[kind].clear()
        if kind == "testDisp":
            self.window.reset_plot(0)
        elif kind == "testShear":
            self.window.reset_plot(1)
        else:
            self.window.reset_wrench_plot()
        self.window.statusBar().showMessage("试验数据和曲线已清空", 3000)

    def _save_test_data(self, kind, filepath):
        rows = tuple(self._test_data.get(kind, ()))
        if not rows:
            self.window.statusBar().showMessage("没有可保存的试验数据", 5000)
            return
        headers, rows = self._format_test_export(kind, rows)
        exporter = XlsxExportWorker(filepath, headers, rows, self.window)
        self._export_workers.add(exporter)
        exporter.saved.connect(lambda path, w=exporter: self._on_export_finished(w, path, None))
        exporter.failed.connect(lambda error, w=exporter: self._on_export_finished(w, None, error))
        exporter.start()
        self.window.statusBar().showMessage("正在后台保存XLSX…", 3000)

    @staticmethod
    def _format_test_export(kind, rows):
        """按试验类型生成包含全部六维数据的XLSX表头和行。"""
        common_headers = (
            "X/mm", "Y/mm", "Z/mm", "U/deg", "V/deg", "W/deg",
            "J1/deg", "J2/deg", "J3/deg", "J4/deg", "J5/deg", "J6/deg",
        )
        wrench_headers = (
            "Fx/N", "Fy/N", "Fz/N",
            "Tx/N·m", "Ty/N·m", "Tz/N·m",
        )
        if kind == "testDisp":
            headers = (
                "时间戳/ms", "贯入深度/mm", *wrench_headers,
                *common_headers,
            )
            rows = tuple(
                (row[0], row[1], *row[3:9], *row[9:])
                for row in rows
            )
        elif kind == "testShear":
            headers = (
                "时间戳/ms", "角位移/deg", *wrench_headers,
                *common_headers,
            )
            rows = tuple(
                (row[0], row[2], *row[3:9], *row[9:])
                for row in rows
            )
        else:
            headers = (
                "时间戳/ms", *wrench_headers, *common_headers,
            )
            rows = tuple(
                (row[0], *row[3:9], *row[9:])
                for row in rows
            )
        return headers, rows

    def _on_export_finished(self, worker, path, error):
        self._export_workers.discard(worker)
        worker.deleteLater()
        if error:
            self.window.statusBar().showMessage(f"保存失败：{error}", 8000)
        else:
            self.window.statusBar().showMessage(f"已保存：{path}", 8000)

    def _on_sensor_connect(self, port):
        actual_port = extract_port_name(port)
        try:
            # 避免重复点击连接后遗留旧线程或继续占用同一个COM口。
            if self.worker or self.sensor_comm:
                self._on_sensor_disconnect()
            self.sensor_comm = SensorCommunication(actual_port)
            self.sensor_ctrl = SensorController(self.sensor_comm)

            # COM口能够打开不代表传感器已连接。先进行最多三次六维只读握手，
            # 只有功能码04成功返回12个输入寄存器后才启动20 Hz采集线程。
            initial_data = None
            for attempt in range(3):
                initial_data = self.sensor_ctrl.monitor_6_sensor()
                if all(value is not None for value in initial_data):
                    break
                if attempt < 2:
                    time.sleep(0.05)
            else:
                detail = self.sensor_ctrl.last_error or "未收到Modbus响应"
                raise RuntimeError(
                    f"{actual_port}可以打开，但传感器模块无响应；"
                    f"请检查供电、A/B接线和转换器，详细信息: {detail}"
                )

            self.worker = SensorWorker(self.sensor_ctrl)
            self.worker.data_ready.connect(self._on_data_ready)
            self.worker.error_occurred.connect(self._on_worker_error)
            self.worker.start()

            self._on_data_ready(*initial_data)

            self.window._set_sensor_connected(True)
            self.window.statusBar().showMessage(f"传感器已连接: {actual_port}", 5000)
        except Exception as e:
            if self.sensor_comm:
                try:
                    self.sensor_comm.close()
                except Exception:
                    pass
            self.worker = None
            self.sensor_ctrl = None
            self.sensor_comm = None
            self._latest_wrench = (0.0,) * 6
            self.window.clear_force_torque()
            self.window._set_sensor_connected(False)
            self.window.statusBar().showMessage(
                f"传感器连接失败（{actual_port}）: {e}", 12000
            )

    def _on_sensor_disconnect(self):
        try:
            if self.worker:
                sensor_worker = self.worker
                sensor_worker.request_stop()
                if self.sensor_comm:
                    self.sensor_comm.close()
                if not sensor_worker.stop():
                    self.window.statusBar().showMessage(
                        "传感器读取线程未能及时退出，请检查串口驱动", 8000
                    )
                self.worker = None
            elif self.sensor_comm:
                self.sensor_comm.close()
            self.sensor_comm = None
            self.sensor_ctrl = None
        except Exception:
            pass
        self._latest_wrench = (0.0,) * 6
        self.window.clear_force_torque()
        self.window._set_sensor_connected(False)
        self.window.statusBar().showMessage("传感器已断开", 5000)

    def _on_data_ready(self, fx, fy, fz, tx, ty, tz):
        self._latest_wrench = tuple(
            float(value) for value in (fx, fy, fz, tx, ty, tz)
        )
        self.window.update_force_torque(*self._latest_wrench)
        self.window.add_wrench_sample(*self._latest_wrench)
        # 直接在20 Hz传感器数据到达时检查阈值，避免再等待GUI采样定时器，
        # 最坏可减少约50 ms的停止判定延迟。
        state = self._active_test
        if state and not state.get("stop_requested"):
            if state["kind"] == "testDisp":
                value = abs(self._latest_wrench[2])
                if value >= state["limit"]:
                    self._finish_active_test("达到最大力", send_stop=True)
            else:
                value = abs(self._latest_wrench[5])
                if value >= state["limit"]:
                    self._finish_active_test("达到最大扭矩", send_stop=True)

    def _on_worker_error(self, msg):
        self.window.statusBar().showMessage(f"传感器错误: {msg}", 5000)

    def _on_sensor_refresh(self):
        ports = list(serial.tools.list_ports.comports())
        if ports:
            port_list = [f"{p.device} - {p.description}" for p in ports]
            self.window.statusBar().showMessage(f"可用串口: {', '.join(port_list)}", 10000)
            self._auto_detect_port()
        else:
            self.window.statusBar().showMessage("未检测到任何串口", 5000)

    def _on_zero_all(self):
        if self.sensor_ctrl:
            try:
                self.sensor_ctrl.reset_all()
                self.window.statusBar().showMessage("全部置零完成", 3000)
            except Exception as e:
                self.window.statusBar().showMessage(f"置零失败: {e}", 5000)
        else:
            self.window.statusBar().showMessage("传感器未连接", 3000)

    def _on_zero_channel(self, channel):
        if self.sensor_ctrl:
            try:
                self.sensor_ctrl.zero_channel(channel)
                channel_name = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")[channel - 1]
                self.window.statusBar().showMessage(
                    f"{channel_name}通道置零完成", 3000
                )
            except Exception as e:
                self.window.statusBar().showMessage(f"通道置零失败: {e}", 5000)
        else:
            self.window.statusBar().showMessage("传感器未连接", 3000)

    def _on_robot_connect(self, ip, port):
        if self.robot_worker:
            self.window.statusBar().showMessage("机械臂正在连接或已经连接", 3000)
            return
        self.window.statusBar().showMessage(f"正在连接机械臂: {ip}:{port}", 5000)
        worker = RobotWorker(ip, port)
        self.robot_worker = worker
        worker.connected.connect(self._on_robot_connected)
        worker.disconnected.connect(self._on_robot_worker_disconnected)
        worker.pose_ready.connect(self._update_robot_pose)
        worker.joints_ready.connect(self._update_robot_joints)
        worker.command_done.connect(self._on_robot_command_done)
        worker.error_occurred.connect(self._on_robot_error)
        worker.start()

    def _update_robot_pose(self, values):
        self._latest_pose = tuple(float(value) for value in values)
        self.window.update_pose(*values)

    def _update_robot_joints(self, values):
        self._latest_joints = tuple(float(value) for value in values)
        self.window.update_joint_angles(*values)

    def _on_robot_connected(self, ip, port):
        self.window._set_robot_connected(True)
        self.window.set_motion_controls_enabled(True)
        self.window.statusBar().showMessage(f"机械臂已连接: {ip}:{port}", 5000)

    def _on_robot_disconnect(self):
        if self.robot_worker:
            self.robot_worker.stop()
        else:
            self._on_robot_worker_disconnected()

    def _on_robot_worker_disconnected(self):
        if self._active_test:
            self._finish_active_test("机械臂连接断开", send_stop=False)
        self.robot_worker = None
        self._latest_pose = None
        self._latest_joints = None
        self.window._set_robot_connected(False)
        self.window.set_motion_controls_enabled(False)
        self.window.clear_robot_realtime_display()
        self.window.statusBar().showMessage("机械臂已断开，实时显示已关闭", 5000)

    def _submit_robot(self, command, *args, urgent=False):
        if not self.robot_worker:
            self.window.statusBar().showMessage("请先连接机械臂", 3000)
            return
        self.robot_worker.submit(command, *args, urgent=urgent)

    def _on_robot_home(self):
        self._submit_robot("home", 10.0)

    def _on_robot_stop(self):
        if self._active_test:
            self._finish_active_test("用户停止", send_stop=True)
        else:
            self._submit_robot("stop", urgent=True)

    def _on_robot_enable(self, state):
        self._submit_robot("enable" if state else "disable")

    def _on_robot_command_done(self, command):
        if command == "safe_stop" and self._active_test:
            reason = self._active_test.get("stop_reason", "停止完成")
            self._finish_active_test(reason, send_stop=False)
        names = {
            "world_increment": "末端增量运动",
            "world_absolute": "末端目标运动",
            "joint_increment": "关节增量运动",
            "joint_absolute": "关节目标运动",
            "enable": "使能",
            "disable": "取消使能",
            "home": "回零",
            "stop": "急停",
            "safe_stop": "试验安全停止",
        }
        self.window.statusBar().showMessage(f"{names.get(command, command)}指令已发送", 3000)

    def _on_robot_error(self, message):
        if self._active_test:
            self._finish_active_test("机械臂指令失败", send_stop=False)
        self.window.statusBar().showMessage(f"机械臂通信错误: {message}", 8000)

    def _shutdown(self):
        self._sample_timer.stop()
        if self.robot_worker:
            self.robot_worker.stop(wait=True)
        if self.worker:
            self.worker.request_stop()
            if self.sensor_comm:
                try:
                    self.sensor_comm.close()
                except Exception:
                    pass
            self.worker.stop()
        for exporter in tuple(self._export_workers):
            exporter.wait(3000)

    def run(self):
        return self.app.exec()


def _stop_child(process):
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def _run_gui_child(env, ready_file, timeout):
    command = (
        [sys.executable]
        if IS_FROZEN
        else [sys.executable, str(Path(__file__).resolve())]
    )
    process = subprocess.Popen(
        command,
        cwd=str(APP_DIR),
        env=env,
    )
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            if ready_file.exists():
                ready_file.unlink(missing_ok=True)
                return process.wait(), True
            return_code = process.poll()
            if return_code is not None:
                return return_code, False
            time.sleep(0.1)
        _stop_child(process)
        return None, False
    except KeyboardInterrupt:
        _stop_child(process)
        raise


def _run_with_opengl_fallback():
    ready_file = Path(tempfile.gettempdir()) / f"robotic_arm_gui_{os.getpid()}.ready"
    ready_file.unlink(missing_ok=True)

    child_env = os.environ.copy()
    child_env[GUI_CHILD_ENV] = "1"
    child_env[GUI_READY_FILE_ENV] = str(ready_file)

    try:
        return_code, ready = _run_gui_child(
            child_env, ready_file, GUI_READY_TIMEOUT_SECONDS
        )
        if ready:
            return return_code

        if return_code is None:
            reason = "timed out"
        else:
            reason = f"exited before showing the window (code {return_code})"
        print(
            f"Qt hardware OpenGL initialization {reason}; "
            "restarting with software OpenGL.",
            flush=True,
        )
        child_env["QT_OPENGL"] = "software"
        return_code, _ = _run_gui_child(
            child_env, ready_file, GUI_READY_TIMEOUT_SECONDS
        )
        if return_code is None:
            print("Qt software OpenGL initialization also timed out.", flush=True)
            return 1
        return return_code
    finally:
        ready_file.unlink(missing_ok=True)


if __name__ == "__main__":
    if os.environ.get(GUI_CHILD_ENV) == "1":
        ctrl = AppController()
        ready_file_path = os.environ.get(GUI_READY_FILE_ENV)
        if ready_file_path:
            Path(ready_file_path).touch()
        sys.exit(ctrl.run())
    sys.exit(_run_with_opengl_fallback())
