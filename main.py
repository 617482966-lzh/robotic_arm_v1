# -*- coding: utf-8 -*-
"""机械臂控制与力传感数据采集 - 主程序入口 + 传感器控制器"""

import sys
import itertools
import queue
import threading
import time
from pathlib import Path
import serial.tools.list_ports
from openpyxl import Workbook

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt, QThread, QTimer, Signal

from main_window_2 import MainWindow

from sensor_2.communication import SensorCommunication
from sensor_2.controller import SensorController
from robot_client.robot_control import BorunteRobot as RobotComm


SAMPLE_INTERVAL_MS = 50  # 20 Hz


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
    data_ready = Signal(float, float)
    error_occurred = Signal(str)

    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.controller = controller
        self._running = False

    def run(self):
        self._running = True
        while self._running:
            try:
                fz, tz = self.controller.monitor_2_sensor()
                if fz is not None and tz is not None:
                    self.data_ready.emit(fz, tz)
            except Exception as e:
                self.error_occurred.emit(str(e))
            self.msleep(50)

    def stop(self):
        self._running = False
        self.wait(2000)


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
                try:
                    _, _, command, args = self._commands.get_nowait()
                    try:
                        self._execute(command, args)
                    except Exception as exc:
                        self.error_occurred.emit(str(exc))
                    continue
                except queue.Empty:
                    pass
                now = time.monotonic()
                if now >= next_poll:
                    pose, joints = self.robot.read_realtime_state()
                    self.pose_ready.emit(pose)
                    self.joints_ready.emit(joints)
                    next_poll = now + self.poll_interval
                else:
                    time.sleep(min(0.02, next_poll - now))
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
        try:
            path = Path(self.filepath)
            path.parent.mkdir(parents=True, exist_ok=True)
            workbook = Workbook(write_only=True)
            sheet = workbook.create_sheet("试验数据")
            sheet.append(self.headers)
            for row in self.rows:
                sheet.append(row)
            workbook.save(path)
            self.saved.emit(str(path))
        except Exception as exc:
            self.failed.emit(str(exc))


class AppController:

    def __init__(self):
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
        self._latest_fz = 0.0
        self._latest_tz = 0.0
        self._active_test = None
        self._test_data = {"testDisp": [], "testShear": []}
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
        self.window.zero_pressure_requested.connect(self._on_zero_pressure)
        self.window.zero_torque_requested.connect(self._on_zero_torque)

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

        self.window.disp_test_start.connect(self._on_disp_test_start)
        self.window.shear_test_start.connect(self._on_shear_test_start)
        self.window.test_save_requested.connect(self._save_test_data)
        self.window.test_reset_requested.connect(self._reset_test_data)

    def _on_disp_test_start(self):
        """Start penetration along the current tool-X vector."""
        if not self._can_start_test("贯入"):
            return
        speed, distance, max_force = map(float, self.window.get_test_params_disp())
        direction = RobotComm.penetration_axis_in_world()
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
        self._begin_test("testShear", angle, max_torque)
        self._submit_robot("joint_increment", (0, 0, 0, 0, 0, angle), speed)
        self.window.statusBar().showMessage(
            f"剪切试验已启动：J6 {angle:g} deg，{speed:g} deg/s，扭矩阈值{max_torque:g} N·m", 5000
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
            "base_fz": self._latest_fz if self.worker else 0.0,
            "base_tz": self._latest_tz if self.worker else 0.0,
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
        fz = self._latest_fz - state["base_fz"] if self.worker else 0.0
        tz = self._latest_tz - state["base_tz"] if self.worker else 0.0
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
            overloaded = abs(self._latest_fz) >= state["limit"]
            reason = "达到目标世界坐标" if reached else "达到最大力"
        else:
            step = self._shortest_angle_delta(joints[5], state["previous_j6"])
            state["accumulated_j6"] += step
            state["previous_j6"] = joints[5]
            angle = max(0.0, state["sign"] * state["accumulated_j6"])
            depth = 0.0
            self.window.add_ang_torque(angle, tz)
            reached = angle >= state["target"] - 0.1
            overloaded = abs(self._latest_tz) >= state["limit"]
            reason = "达到目标角位移" if reached else "达到最大扭矩"

        self._test_data[state["kind"]].append((
            round(elapsed_ms, 3), round(depth, 6), round(angle, 6),
            round(fz, 6), round(tz, 6),
            round(self._latest_fz if self.worker else 0.0, 6),
            round(self._latest_tz if self.worker else 0.0, 6),
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
                "正在actionStop立即停止；结束后请在示教器重新切回自动模式…", 10000
            )
            return
        kind = self._active_test["kind"]
        clear_on_stop = self._active_test.get("clear_on_stop", False)
        self._active_test = None
        self._sample_timer.stop()
        if clear_on_stop:
            self._test_data[kind].clear()
            self.window.reset_plot(0 if kind == "testDisp" else 1)
        label = "贯入" if kind == "testDisp" else "剪切"
        self.window.statusBar().showMessage(
            f"{label}试验结束：{reason}；请确认示教器已重新切回自动模式", 12000
        )

    def _reset_test_data(self, kind):
        if self._active_test and self._active_test["kind"] == kind:
            self._active_test["clear_on_stop"] = True
            self._finish_active_test("用户重置", send_stop=True)
            return
        self._test_data[kind].clear()
        self.window.reset_plot(0 if kind == "testDisp" else 1)
        self.window.statusBar().showMessage("试验数据和曲线已清空", 3000)

    def _save_test_data(self, kind, filepath):
        rows = tuple(self._test_data.get(kind, ()))
        if not rows:
            self.window.statusBar().showMessage("没有可保存的试验数据", 5000)
            return
        common_headers = (
            "X/mm", "Y/mm", "Z/mm", "U/deg", "V/deg", "W/deg",
            "J1/deg", "J2/deg", "J3/deg", "J4/deg", "J5/deg", "J6/deg",
        )
        if kind == "testDisp":
            headers = (
                "时间戳/ms", "贯入深度/mm", "拉压力/N", "扭矩/N·m",
                "原始拉压力/N", "原始扭矩/N·m", *common_headers,
            )
            rows = tuple(
                (row[0], row[1], row[3], row[4], row[5], row[6], *row[7:])
                for row in rows
            )
        else:
            headers = (
                "时间戳/ms", "角位移/deg", "扭矩/N·m", "拉压力/N",
                "原始扭矩/N·m", "原始拉压力/N", *common_headers,
            )
            rows = tuple(
                (row[0], row[2], row[4], row[3], row[6], row[5], *row[7:])
                for row in rows
            )
        exporter = XlsxExportWorker(filepath, headers, rows, self.window)
        self._export_workers.add(exporter)
        exporter.saved.connect(lambda path, w=exporter: self._on_export_finished(w, path, None))
        exporter.failed.connect(lambda error, w=exporter: self._on_export_finished(w, None, error))
        exporter.start()
        self.window.statusBar().showMessage("正在后台保存XLSX…", 3000)

    def _on_export_finished(self, worker, path, error):
        self._export_workers.discard(worker)
        worker.deleteLater()
        if error:
            self.window.statusBar().showMessage(f"保存失败：{error}", 8000)
        else:
            self.window.statusBar().showMessage(f"已保存：{path}", 8000)

    def _on_sensor_connect(self, port, baudrate, slave_addr):
        try:
            actual_port = extract_port_name(port)
            self.sensor_comm = SensorCommunication(actual_port, slave_address=slave_addr)
            self.sensor_comm.setup_communication()
            self.sensor_comm.instrument.serial.baudrate = baudrate
            self.sensor_ctrl = SensorController(self.sensor_comm)

            self.worker = SensorWorker(self.sensor_ctrl)
            self.worker.data_ready.connect(self._on_data_ready)
            self.worker.error_occurred.connect(self._on_worker_error)
            self.worker.start()

            self.window._set_sensor_connected(True)
            self.window.statusBar().showMessage(f"传感器已连接: {actual_port}", 5000)
        except Exception as e:
            self.window.statusBar().showMessage(f"连接失败: {e}", 8000)

    def _on_sensor_disconnect(self):
        try:
            if self.worker:
                self.worker.stop()
                self.worker = None
            if self.sensor_comm and self.sensor_comm.instrument:
                self.sensor_comm.instrument.serial.close()
            self.sensor_comm = None
            self.sensor_ctrl = None
        except Exception:
            pass
        self.window._set_sensor_connected(False)
        self.window.statusBar().showMessage("传感器已断开", 5000)

    def _on_data_ready(self, fz, tz):
        self._latest_fz = float(fz)
        self._latest_tz = float(tz)
        self.window.update_force_torque(fz, tz)
        # 直接在20 Hz传感器数据到达时检查阈值，避免再等待GUI采样定时器，
        # 最坏可减少约50 ms的停止判定延迟。
        state = self._active_test
        if state and not state.get("stop_requested"):
            if state["kind"] == "testDisp":
                value = abs(self._latest_fz)
                if value >= state["limit"]:
                    self._finish_active_test("达到最大力", send_stop=True)
            else:
                value = abs(self._latest_tz)
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

    def _on_zero_pressure(self):
        if self.sensor_ctrl:
            try:
                self.sensor_ctrl.reset_pulling_pressure()
                self.window.statusBar().showMessage("压力置零完成", 3000)
            except Exception as e:
                self.window.statusBar().showMessage(f"压力置零失败: {e}", 5000)
        else:
            self.window.statusBar().showMessage("传感器未连接", 3000)

    def _on_zero_torque(self):
        if self.sensor_ctrl:
            try:
                self.sensor_ctrl.reset_torque()
                self.window.statusBar().showMessage("扭矩置零完成", 3000)
            except Exception as e:
                self.window.statusBar().showMessage(f"扭矩置零失败: {e}", 5000)
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
            self.worker.stop()
        for exporter in tuple(self._export_workers):
            exporter.wait(3000)

    def run(self):
        return self.app.exec()


if __name__ == "__main__":
    ctrl = AppController()
    sys.exit(ctrl.run())
