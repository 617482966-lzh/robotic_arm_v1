# -*- coding: utf-8 -*-
"""机械臂控制与力传感数据采集 - 主程序入口 + 传感器控制器"""

import sys
import itertools
import queue
import threading
import time
import serial.tools.list_ports

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt, QThread, Signal

from main_window_2 import MainWindow

from sensor_2.communication import SensorCommunication
from sensor_2.controller import SensorController
from robot_client.robot_control import BorunteRobot as RobotComm


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

    def __init__(self, host, port, poll_interval=0.12):
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
            "joint_increment": self.robot.move_joints_increment,
            "joint_absolute": self.robot.move_joints_absolute,
            "enable": self.robot.enable,
            "disable": self.robot.disable,
            "home": self.robot.home,
            "stop": self.robot.emergency_stop,
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

        self._connect_signals()
        self._auto_detect_port()
        self.window.set_motion_controls_enabled(False)
        self.window.clear_robot_realtime_display()
        self.app.aboutToQuit.connect(self._shutdown)
        self.window.show()

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
        self.window.shear_test_start.connect(lambda: None)
        self.window.save_data_requested.connect(lambda p: print(f"Save: {p}"))

    def _on_disp_test_start(self):
        """沿当前末端局部+X方向执行定姿态直线插补。"""
        speed_widget = self.window._find_child("testDispSpeed")
        distance_widget = self.window._find_child("testDispDist")
        if speed_widget is None or distance_widget is None:
            self.window.statusBar().showMessage("找不到直线插补参数控件", 5000)
            return
        speed = float(speed_widget.value())
        distance = float(distance_widget.value())
        self._submit_robot("tool_vector_line", distance, speed, 5.0)
        self.window.statusBar().showMessage(
            f"已提交末端方向直线插补：{distance:+.1f} mm，{speed:.1f} mm/s，5 Hz",
            5000,
        )

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
        self.window.update_force_torque(fz, tz)

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
        self.window.update_pose(*values)

    def _update_robot_joints(self, values):
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
        self.robot_worker = None
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
        self._submit_robot("stop", urgent=True)

    def _on_robot_enable(self, state):
        self._submit_robot("enable" if state else "disable")

    def _on_robot_command_done(self, command):
        names = {
            "world_increment": "末端增量运动",
            "world_absolute": "末端目标运动",
            "joint_increment": "关节增量运动",
            "joint_absolute": "关节目标运动",
            "enable": "使能",
            "disable": "取消使能",
            "home": "回零",
            "stop": "急停",
        }
        self.window.statusBar().showMessage(f"{names.get(command, command)}指令已发送", 3000)

    def _on_robot_error(self, message):
        self.window.statusBar().showMessage(f"机械臂通信错误: {message}", 8000)

    def _shutdown(self):
        if self.robot_worker:
            self.robot_worker.stop(wait=True)
        if self.worker:
            self.worker.stop()

    def run(self):
        return self.app.exec()


if __name__ == "__main__":
    ctrl = AppController()
    sys.exit(ctrl.run())
