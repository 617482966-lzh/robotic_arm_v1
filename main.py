# -*- coding: utf-8 -*-
"""机械臂控制与力传感数据采集 - 主程序入口 + 传感器控制器"""

import sys
import os
import serial.tools.list_ports
from datetime import datetime

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt, QThread, Signal

from main_window import MainWindow

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


class AppController:

    def __init__(self):
        self.app = QApplication(sys.argv)
        self.app.setAttribute(Qt.AA_Use96Dpi)
        self.app.setStyle("Fusion")

        self.window = MainWindow()
        self.sensor_comm = None
        self.sensor_ctrl = None
        self.robot_comm = None
        self.worker = None

        self._connect_signals()
        self._auto_detect_port()
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
        self.window.jog_cmd.connect(self._on_robot_jog)
        self.window.jog_stop_cmd.connect(lambda: None)
        self.window.robot_move_cmd.connect(self._on_robot_move)
        self.window.speed_changed.connect(self._on_robot_speed)
        self.window.ang_speed_changed.connect(lambda v: print(f"Ang speed: {v}"))
        self.window.ptp_requested.connect(self._on_robot_ptp)
        self.window.home_requested.connect(self._on_robot_home)
        self.window.stop_requested.connect(self._on_robot_stop)
        self.window.robot_enable_changed.connect(self._on_robot_enable)

        self.window.plot_disp_show.connect(lambda: self.window.set_plot_visible(0, True))
        self.window.plot_disp_close.connect(lambda: self.window.set_plot_visible(0, False))
        self.window.plot_disp_reset.connect(lambda: self.window.reset_plot(0))
        self.window.plot_ang_show.connect(lambda: self.window.set_plot_visible(1, True))
        self.window.plot_ang_close.connect(lambda: self.window.set_plot_visible(1, False))
        self.window.plot_ang_reset.connect(lambda: self.window.reset_plot(1))

        self.window.disp_test_start.connect(lambda: None)
        self.window.shear_test_start.connect(lambda: None)
        self.window.save_data_requested.connect(lambda p: print(f"Save: {p}"))

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
        try:
            self.robot_comm = RobotComm(ip, port)
            self.robot_comm.connect()
            self.window._set_robot_connected(True)
            self.window.statusBar().showMessage(f"机械臂已连接: {ip}:{port}", 5000)
        except Exception as e:
            self.window.statusBar().showMessage(f"机械臂连接失败: {e}", 8000)

    def _on_robot_disconnect(self):
        if self.robot_comm:
            self.robot_comm.disconnect()
            self.robot_comm = None
        self.window._set_robot_connected(False)
        self.window.statusBar().showMessage("机械臂已断开", 5000)

    def _on_robot_jog(self, axis, direction):
        if self.robot_comm:
            step = self.window.get_robot_step()
            speed = self.window.get_robot_speed()
            if step == 0.0:
                step = 1.0
            try:
                self.robot_comm.move_jog(axis, direction, step, speed)
                self.window.statusBar().showMessage(f"Jog {axis}{direction} {step}", 2000)
            except Exception as e:
                self.window.statusBar().showMessage(f"Jog 失败: {e}", 5000)

    def _on_robot_move(self, axis, target):
        if self.robot_comm:
            try:
                speed = self.window.get_robot_speed()
                self.robot_comm.move_jog(axis, "+" if target >= 0 else "-", abs(target), speed)
                self.window.statusBar().showMessage(f"移动 {axis} -> {target}", 2000)
            except Exception as e:
                self.window.statusBar().showMessage(f"移动失败: {e}", 5000)

    def _on_robot_speed(self, speed):
        if self.robot_comm:
            try:
                self.robot_comm.set_speed(speed)
            except:
                pass

    def _on_robot_ptp(self):
        if self.robot_comm:
            try:
                x = self.window._find_child("robotTargetX")
                y = self.window._find_child("robotTargetY")
                z = self.window._find_child("robotTargetZ")
                rx = self.window._find_child("robotTargetRx")
                ry = self.window._find_child("robotTargetRy")
                rz = self.window._find_child("robotTargetRz")
                speed = self.window.get_robot_speed()
                if all([x, y, z, rx, ry, rz]):
                    self.robot_comm.move_to(
                        x.value(), y.value(), z.value(),
                        rx.value(), ry.value(), rz.value(), speed
                    )
                    self.window.statusBar().showMessage("PTP 移动已发送", 2000)
            except Exception as e:
                self.window.statusBar().showMessage(f"PTP 失败: {e}", 5000)

    def _on_robot_home(self):
        if self.robot_comm:
            try:
                self.robot_comm.home()
                self.window.statusBar().showMessage("回零指令已发送", 2000)
            except Exception as e:
                self.window.statusBar().showMessage(f"回零失败: {e}", 5000)

    def _on_robot_stop(self):
        if self.robot_comm:
            try:
                self.robot_comm.stop()
                self.window.statusBar().showMessage("急停指令已发送", 2000)
            except Exception as e:
                self.window.statusBar().showMessage(f"急停失败: {e}", 5000)

    def _on_robot_enable(self, state):
        if self.robot_comm:
            try:
                if state:
                    self.robot_comm.enable()
                else:
                    self.robot_comm.disable()
            except:
                pass

    def run(self):
        return self.app.exec()


if __name__ == "__main__":
    ctrl = AppController()
    sys.exit(ctrl.run())
