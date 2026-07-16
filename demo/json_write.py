# -*- coding: utf-8 -*-
"""
HC1 / 伯朗特机器人 JSON 远程控制模块 — 基于 HCRemoteCommand 协议
端口: 9760  |  文档: BRT-C-RC-HC1-TI1-V1.4

功能:
  1. 读取当前关节角 (axis-0~axis-5) 与末端世界坐标 (world-0~world-5)
  2. 物理速度控制 (action=51), 单位 mm/s, 一直执行
  3. 自由路径 (action=4), 输入六个关节角
  4. 姿势直线 (action=10), 输入世界坐标系
  5. 急停 (actionStop), 停止所有运动
  6. 解除所有报警 (clearAlarm)
  7. 保持姿态，沿末端当前局部 +X 方向移动指定三维距离

快速使用（在项目根目录执行）:
  读取状态:
    C:\\Users\\www61\\anaconda3\\envs\\common\\python.exe demo\\json_write.py read_status
  沿姿态向量正向移动 10 mm，速度 1 mm/s:
    C:\\Users\\www61\\anaconda3\\envs\\common\\python.exe demo\\json_write.py tool_x 10 --speed 1
  沿反方向返回 10 mm:
    C:\\Users\\www61\\anaconda3\\envs\\common\\python.exe demo\\json_write.py tool_x -10 --speed 1

完整说明见 demo/README.md。实机运动前必须确认自动模式、无报警、静止、
远程指令列表为空，并确保运动方向上没有人员或障碍物。
"""

import socket
import json
import time
import threading
import math

# 默认连接参数
HOST = "192.168.1.4"
PORT = 9760

DSID_MONITOR = "www.hc-system.com.RemoteMonitor"
DSID_COMMAND = "www.hc-system.com.HCRemoteCommand"

class HC1JsonRobot:
    """HC1 机器人 JSON 远程控制客户端"""

    def __init__(self, host=HOST, port=PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.pack_id = 8000
        self._lock = threading.Lock()

    # ---- 连接管理 ----

    def connect(self):
        """建立 TCP 连接"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(3.0)
        self.sock.connect((self.host, self.port))
        print(f"[OK] 已连接 {self.host}:{self.port}")

    def disconnect(self):
        """关闭 TCP 连接"""
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            print("[OK] 已断开连接")

    def is_connected(self):
        return self.sock is not None

    # ---- 底层通信 ----

    def _next_pack_id(self):
        self.pack_id += 1
        return str(self.pack_id)

    def _recv_json(self, timeout=3.0):
        """接收并解析 JSON 回复"""
        self.sock.settimeout(timeout)
        data = b""
        while True:
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
                data += chunk
                text = data.decode("utf-8", errors="ignore").strip()
                try:
                    return json.loads(text)
                except json.JSONDecodeError:
                    continue
            except socket.timeout:
                break
        if data:
            print("[WARN] 收到数据但无法解析 JSON:")
            print(data.decode("utf-8", errors="ignore"))
        return None

    def _send_json(self, req, show=False, timeout=3.0):
        """发送 JSON 请求并接收回复"""
        with self._lock:
            raw = json.dumps(req, ensure_ascii=False) + "\r\n"
            if show:
                print(">>> SEND")
                print(json.dumps(req, ensure_ascii=False, indent=2))
            self.sock.sendall(raw.encode("utf-8"))
            rep = self._recv_json(timeout=timeout)
            if show:
                print("<<< RECV")
                print(rep)
            return rep

    # ---- Query: 读取状态 ----

    def query(self, addrs, show=False):
        """RemoteMonitor query"""
        req = {
            "dsID": DSID_MONITOR,
            "reqType": "query",
            "packID": self._next_pack_id(),
            "queryAddr": list(addrs)
        }
        return self._send_json(req, show=show)

    # ---- 功能1: 读取当前关节角 + 末端世界坐标 ----

    def read_joints(self):
        """读取六个关节角 (axis-0 ~ axis-5)
        返回: {"j1": float, "j2": ..., "j6": ...} 或 None
        """
        addrs = [f"axis-{i}" for i in range(6)]
        rep = self.query(addrs)
        if not rep or "queryData" not in rep:
            return None
        q = rep["queryData"]
        if len(q) < 6:
            return None
        try:
            return {
                "j1": float(q[0]), "j2": float(q[1]), "j3": float(q[2]),
                "j4": float(q[3]), "j5": float(q[4]), "j6": float(q[5]),
            }
        except Exception as e:
            print(f"[FAIL] 关节角解析失败: {e}")
            return None

    def read_world_pose(self):
        """读取末端世界坐标系 (world-0 ~ world-5)
        返回: {"x": float, "y": ..., "z": ..., "u": ..., "v": ..., "w": ...} 或 None
        """
        addrs = [f"world-{i}" for i in range(6)]
        rep = self.query(addrs)
        if not rep or "queryData" not in rep:
            return None
        q = rep["queryData"]
        if len(q) < 6:
            return None
        try:
            return {
                "x": float(q[0]), "y": float(q[1]), "z": float(q[2]),
                "u": float(q[3]), "v": float(q[4]), "w": float(q[5]),
            }
        except Exception as e:
            print(f"[FAIL] 世界坐标解析失败: {e}")
            return None

    def read_status_pose(self):
        """读取当前模式、运动状态、报警 + 末端世界坐标
        返回: {"curMode": int, "isMoving": int, "curAlarm": int,
                "x","y","z","u","v","w": float} 或 None
        """
        addrs = ["curMode", "isMoving", "curAlarm"] + [f"world-{i}" for i in range(6)]
        rep = self.query(addrs)
        if not rep or "queryData" not in rep:
            return None
        q = rep["queryData"]
        if len(q) < 9:
            return None
        try:
            return {
                "curMode": int(float(q[0])),
                "isMoving": int(float(q[1])),
                "curAlarm": int(float(q[2])),
                "x": float(q[3]), "y": float(q[4]), "z": float(q[5]),
                "u": float(q[6]), "v": float(q[7]), "w": float(q[8]),
            }
        except Exception as e:
            print(f"[FAIL] 状态解析失败: {e}")
            return None

    # ---- Command: 控制命令 ----

    def command(self, cmd_data, show=True):
        """RemoteMonitor command"""
        req = {
            "dsID": DSID_MONITOR,
            "reqType": "command",
            "packID": self._next_pack_id(),
            "cmdData": list(cmd_data)
        }
        return self._send_json(req, show=show)

    # ---- 功能5: 急停 ----

    def emergency_stop(self):
        """立即停止所有运动 (使能断开, 重新上使能后从头发起)"""
        return self.command(["actionStop"])

    def stop_button(self):
        """停止按键 (等同急停, 同时清除报警)"""
        return self.command(["stopButton"])

    # ---- 功能6: 解除所有报警 ----

    def clear_alarm(self):
        """清除所有报警 (d1=1 首行, d2=1 仅报警执行)"""
        return self.command(["clearAlarm", "1", "1"])

    def clear_alarm_and_run(self):
        """清除报警后运行下一条指令"""
        return self.command(["clearAlarmRunNext"])

    def clear_alarm_continue(self):
        """清除报警并继续自动运行"""
        return self.command(["clearAlarmContinue"])

    # ---- 启动 / 暂停 / 使能 ----

    def start(self):
        """启动按键"""
        return self.command(["startButton"])

    def pause(self):
        """暂停当前动作 (启动从当前步继续)"""
        return self.command(["actionPause"])

    def enable(self):
        """切换到运行模式"""
        return self.command(["run", "1", "1"])

    def disable(self):
        """切换到停止模式"""
        return self.command(["stop", "1", "1"])

    def home(self):
        """回零"""
        return self.command(["home", "1", "1"])

    def set_global_speed(self, speed_pct):
        """设置全局速度百分比 (0-100, 精度0.1%)"""
        return self.command(["modifyGSPD", str(int(speed_pct * 10))])

    # ---- AddRCC: 远程教导指令 ----

    def add_rcc(self, instructions, empty=True, show=True):
        """发送远程教导指令集
        instructions: 指令对象列表
        empty: True=清空列表后添加, False=追加
        """
        req = {
            "dsID": DSID_COMMAND,
            "reqType": "AddRCC",
            "packID": self._next_pack_id(),
            "emptyList": "1" if empty else "0",
            "instructions": instructions
        }
        return self._send_json(req, show=show)

    # ---- 功能2: 物理速度 (action=51) ----

    def set_physical_speed(self, speed_mm_s, one_shot=False):
        """设置物理速度 (action=51), 一直执行直到被覆盖
        speed_mm_s: 速度, 单位 mm/s (例如 100 = 100 mm/s = 0.1 m/s)
        one_shot: True=执行一次, False=一直执行

        协议说明: speed 字段取整数, 如 1234 表示 1234 mm/s = 1.234 m/s
        """
        inst = {
            "oneshot": "1" if one_shot else "0",
            "action": "51",
            "isUse": "1",
            "speed": str(int(speed_mm_s)),
        }
        return self.add_rcc([inst], empty=False)

    def disable_physical_speed(self):
        """禁用物理速度"""
        inst = {
            "oneshot": "0",
            "action": "51",
            "isUse": "0",
            "speed": "0",
        }
        return self.add_rcc([inst], empty=False)

    # ---- 功能3: 自由路径 (action=4) ----

    @staticmethod
    def joint_mask_from_deltas(deltas, tolerance=1e-6):
        """根据J1～J6的变化量生成六轴ckStatus掩码。"""
        values = tuple(float(value) for value in deltas)
        if len(values) != 6:
            raise ValueError("关节变化量必须包含J1～J6六个值")
        mask = 0
        for index, value in enumerate(values):
            if abs(value) > tolerance:
                mask |= 1 << index
        if mask == 0:
            raise ValueError("六个关节均无变化，不发送自由路径指令")
        return mask

    def _build_free_path_inst(self, j1, j2, j3, j4, j5, j6,
                              speed_pct=50.0, ck_status=0x3F,
                              one_shot=True, tool=0, coord=0, smooth=0):
        """构建自由路径指令 (action=4)

        六轴自由路径: m0-m5 对应关节角 J1-J6 (deg)
        ck_status: 轴掩码, bit0=J1 ... bit5=J6，0x3F = 六轴
        """
        return {
            "oneshot": "1" if one_shot else "0",
            "action": "4",
            "m0": f"{j1:.3f}", "m1": f"{j2:.3f}", "m2": f"{j3:.3f}",
            "m3": f"{j4:.3f}", "m4": f"{j5:.3f}", "m5": f"{j6:.3f}",
            "ckStatus": f"0x{ck_status:X}" if ck_status >= 0 else "0X3F",
            "speed": str(int(speed_pct)),
            "tool": str(tool), "coord": str(coord),
            "smooth": str(smooth),
        }

    def move_free_path(self, j1, j2, j3, j4, j5, j6,
                       speed_pct=50.0, ck_status=None,
                       one_shot=True, show=True):
        """自由路径运动 (action=4)

        参数:
            j1~j6: 关节角 1~6 (deg)
            speed_pct: 速度百分比 0-100
            ck_status: 默认None，自动比较当前角度与目标角度生成掩码
            one_shot: True=执行一次后删除, False=一直循环
        """
        targets = tuple(float(value) for value in (j1, j2, j3, j4, j5, j6))
        if ck_status is None:
            current = self.read_joints()
            if current is None:
                raise RuntimeError("无法读取当前关节角，不能自动生成ckStatus")
            deltas = tuple(
                targets[index] - float(current[f"j{index + 1}"])
                for index in range(6)
            )
            ck_status = self.joint_mask_from_deltas(deltas)
        inst = self._build_free_path_inst(
            *targets,
            speed_pct=speed_pct, ck_status=ck_status,
            one_shot=one_shot
        )
        return self.add_rcc([inst], empty=True, show=show)

    def move_free_path_dict(self, joints, speed_pct=50.0, ck_status=None, one_shot=True):
        """自由路径 (从字典传参)
        joints: {"j1": ..., "j2": ..., ..., "j6": ...}
        """
        j = [joints.get(f"j{i+1}", 0) for i in range(6)]
        return self.move_free_path(*j, speed_pct=speed_pct,
                                    ck_status=ck_status, one_shot=one_shot)

    def move_free_path_by(self, dj1=0.0, dj2=0.0, dj3=0.0,
                            dj4=0.0, dj5=0.0, dj6=0.0,
                            speed_pct=50.0, ck_status=None,
                            one_shot=True, show=True):
        """自由路径增量运动 — 在当前关节角基础上增减

        读取当前位置, 加上增量后发送 action=4 自由路径指令。

        参数:
            dj1~dj6: 关节角增量 (deg)
            speed_pct: 速度百分比
            ck_status: 默认None，根据非零关节增量自动生成
            one_shot: True=执行一次后删除, False=循环
            show: 打印发送的 JSON
        返回:
            True=成功发送, False=读取关节角失败
        """
        cur = self.read_joints()
        if cur is None:
            print("[FAIL] 无法读取当前关节角, 增量运动取消")
            return False
        target = {
            "j1": cur["j1"] + dj1, "j2": cur["j2"] + dj2,
            "j3": cur["j3"] + dj3, "j4": cur["j4"] + dj4,
            "j5": cur["j5"] + dj5, "j6": cur["j6"] + dj6,
        }
        if ck_status is None:
            ck_status = self.joint_mask_from_deltas(
                (dj1, dj2, dj3, dj4, dj5, dj6)
            )
        print(f"自由路径增量: "
              f"J1 {cur['j1']:.3f}->{target['j1']:.3f}  "
              f"J2 {cur['j2']:.3f}->{target['j2']:.3f}  "
              f"J3 {cur['j3']:.3f}->{target['j3']:.3f}  "
              f"J4 {cur['j4']:.3f}->{target['j4']:.3f}  "
              f"J5 {cur['j5']:.3f}->{target['j5']:.3f}  "
              f"J6 {cur['j6']:.3f}->{target['j6']:.3f}")
        self.move_free_path(
            target["j1"], target["j2"], target["j3"],
            target["j4"], target["j5"], target["j6"],
            speed_pct=speed_pct, ck_status=ck_status,
            one_shot=one_shot, show=show
        )
        return True

        """自由路径 (从字典传参)
        joints: {"j1": ..., "j2": ..., ..., "j6": ...} 或列表 [j1,...,j6]
        """
        if isinstance(joints, dict):
            j = [joints.get(f"j{i+1}", 0) for i in range(6)]
        else:
            j = list(joints)[:6]
        return self.move_free_path(*j, speed_pct=speed_pct,
                                    ck_status=ck_status, one_shot=one_shot)

    # ---- 功能4: 姿势直线 (action=10) ----

    def _build_pose_line_inst(self, x, y, z, u, v, w,
                              speed_pct=50.0, ck_status=0x3F,
                              one_shot=True, tool=0, coord=0, smooth=0):
        """构建姿势直线指令 (action=10)

        六轴姿势直线: m0-m5 对应世界坐标 X,Y,Z,U,V,W
        """
        return {
            "oneshot": "1" if one_shot else "0",
            "action": "10",
            "m0": f"{x:.3f}", "m1": f"{y:.3f}", "m2": f"{z:.3f}",
            "m3": f"{u:.3f}", "m4": f"{v:.3f}", "m5": f"{w:.3f}",
            "ckStatus": f"0x{ck_status:X}" if ck_status >= 0 else "0X3F",
            "speed": str(int(speed_pct)),
            "tool": str(tool), "coord": str(coord),
            "smooth": str(smooth),
        }

    def move_pose_line(self, x, y, z, u, v, w,
                       speed_pct=50.0, ck_status=0x3F,
                       one_shot=True, show=True):
        """姿势直线运动 (action=10)

        参数:
            x,y,z: 世界坐标 (mm)
            u,v,w: 世界姿态 (deg)
            speed_pct: 速度百分比
            ck_status: 轴掩码
            one_shot: True=执行一次, False=一直循环
        """
        inst = self._build_pose_line_inst(
            x, y, z, u, v, w,
            speed_pct=speed_pct, ck_status=ck_status,
            one_shot=one_shot
        )
        return self.add_rcc([inst], empty=True, show=show)

    def move_pose_line_dict(self, pose, speed_pct=50.0, ck_status=0x3F, one_shot=True):
        """姿势直线 (从字典传参)
        pose: {"x": ..., "y": ..., "z": ..., "u": ..., "v": ..., "w": ...}
        """
        return self.move_pose_line(
            pose.get("x", 0), pose.get("y", 0), pose.get("z", 0),
            pose.get("u", 0), pose.get("v", 0), pose.get("w", 0),
            speed_pct=speed_pct, ck_status=ck_status, one_shot=one_shot
        )

    def move_pose_line_by(self, dx=0.0, dy=0.0, dz=0.0,
                            du=0.0, dv=0.0, dw=0.0,
                            speed_pct=50.0, ck_status=0x3F,
                            one_shot=True, show=True):
        """姿势直线增量运动 — 在当前世界坐标基础上增减

        读取当前位置, 加上增量后发送 action=10 姿势直线指令。

        参数:
            dx,dy,dz: 世界坐标增量 (mm)
            du,dv,dw: 世界姿态增量 (deg)
            speed_pct: 速度百分比
            ck_status: 轴掩码
            one_shot: True=执行一次后删除, False=循环
            show: 打印发送的 JSON
        返回:
            True=成功发送, False=读取坐标失败
        """
        cur = self.read_world_pose()
        if cur is None:
            print("[FAIL] 无法读取当前世界坐标, 增量运动取消")
            return False
        target = {
            "x": cur["x"] + dx, "y": cur["y"] + dy, "z": cur["z"] + dz,
            "u": cur["u"] + du, "v": cur["v"] + dv, "w": cur["w"] + dw,
        }
        print(f"姿势直线增量: "
              f"X {cur['x']:.3f}->{target['x']:.3f}  "
              f"Y {cur['y']:.3f}->{target['y']:.3f}  "
              f"Z {cur['z']:.3f}->{target['z']:.3f}  "
              f"U {cur['u']:.3f}->{target['u']:.3f}  "
              f"V {cur['v']:.3f}->{target['v']:.3f}  "
              f"W {cur['w']:.3f}->{target['w']:.3f}")
        self.move_pose_line(
            target["x"], target["y"], target["z"],
            target["u"], target["v"], target["w"],
            speed_pct=speed_pct, ck_status=ck_status,
            one_shot=one_shot, show=show
        )
        return True

    @staticmethod
    def tool_x_axis_in_world(u_deg, v_deg, w_deg):
        """标准Rz·Ry·Rx旋转下，工具坐标系+X轴的世界方向。"""
        ry = math.radians(float(v_deg))
        rz = math.radians(float(w_deg))
        return math.cos(rz) * math.cos(ry), math.sin(rz) * math.cos(ry), -math.sin(ry)

    @staticmethod
    def penetration_axis_in_world():
        """贯入试验固定沿世界坐标系Z负方向。"""
        return 0.0, 0.0, -1.0

    def move_along_tool_x(self, distance_mm, speed_mm_s=1.0, show=True):
        """保持U/V/W不变，沿末端当前局部+X方向移动指定空间距离。

        正距离沿局部+X，负距离沿局部-X。为配合当前实机安全范围，
        距离限制为±60 mm。action10使用ckStatus=0x07，只使能XYZ，
        因此姿态轴不会被控制器重新规划。
        """
        distance = float(distance_mm)
        speed = float(speed_mm_s)
        if not 0 < abs(distance) <= 60.0:
            raise ValueError("移动距离必须在-60~60 mm内且不能为0")
        if speed < 1.0:
            raise ValueError("物理速度不能低于1 mm/s")

        current = self.read_world_pose()
        if current is None:
            raise RuntimeError("无法读取当前末端世界坐标")
        direction = self.penetration_axis_in_world()
        delta = tuple(distance * value for value in direction)
        target = (
            current["x"] + delta[0],
            current["y"] + delta[1],
            current["z"] + delta[2],
            current["u"], current["v"], current["w"],
        )
        move = self._build_pose_line_inst(
            *target,
            speed_pct=int(round(speed)),
            ck_status=0x07,
            one_shot=True,
            smooth=9,
        )
        # action51提供mm/s物理速度；action10不再携带speed，避免覆盖。
        move.pop("speed", None)
        speed_inst = {
            "oneshot": "0", "action": "51", "isUse": "1",
            "speed": str(int(round(speed))),
        }
        reply = self.add_rcc([speed_inst, move], empty=True, show=show)
        result = {
            "start": tuple(current[key] for key in ("x", "y", "z", "u", "v", "w")),
            "direction": direction,
            "delta_xyz": delta,
            "target": target,
            "distance_mm": distance,
            "speed_mm_s": speed,
            "reply": reply,
        }
        print(
            "末端方向移动: "
            f"距离={distance:+.3f} mm, "
            f"方向=({direction[0]:+.6f}, {direction[1]:+.6f}, {direction[2]:+.6f}), "
            f"ΔXYZ=({delta[0]:+.3f}, {delta[1]:+.3f}, {delta[2]:+.3f})"
        )
        return result

        """姿势直线 (从字典传参)
        pose: {"x": ..., "y": ..., "z": ..., "u": ..., "v": ..., "w": ...}
        """
        return self.move_pose_line(
            pose.get("x", 0), pose.get("y", 0), pose.get("z", 0),
            pose.get("u", 0), pose.get("v", 0), pose.get("w", 0),
            speed_pct=speed_pct, ck_status=ck_status, one_shot=one_shot
        )

    # ---- 监控辅助 ----

    def wait_for_idle(self, timeout=30.0, interval=0.2):
        """等待机器人停止运动"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            status = self.read_status_pose()
            if status and status["isMoving"] == 0:
                return True
            time.sleep(interval)
        print("[WARN] 等待超时")
        return False

    def check_alarm(self):
        """检查是否有报警, 返回报警码 (0=无报警)"""
        rep = self.query(["curAlarm"])
        if rep and "queryData" in rep:
            return int(float(rep["queryData"][0]))
        return -1


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="HC1 JSON 远程控制 CLI")
    sub = parser.add_subparsers(dest="cmd", help="可用命令")
    # 连接管理
    p_conn = sub.add_parser("connect", help="连接机器人")
    p_conn.add_argument("--host", default="192.168.1.4", help="IP 地址")
    p_conn.add_argument("--port", type=int, default=9760, help="端口")
    sub.add_parser("disconnect", help="断开连接")
    # 读取
    sub.add_parser("read_joints", help="读取当前关节角")
    sub.add_parser("read_pose", help="读取末端世界坐标")
    sub.add_parser("read_status", help="读取完整状态")
    # 物理速度
    p_speed = sub.add_parser("speed", help="设置物理速度 (action=51)")
    p_speed.add_argument("speed_mm_s", type=float, help="速度 (mm/s)")
    p_speed.add_argument("--oneshot", type=int, choices=[0, 1], default=0, help="1=执行一次, 0=一直执行")
    sub.add_parser("speed_off", help="禁用物理速度")
    # 自由路径
    p_free = sub.add_parser("free", help="自由路径 (action=4)")
    for a in ["j1", "j2", "j3", "j4", "j5", "j6"]:
        p_free.add_argument(f"--{a}", type=float, default=0.0, help=f"关节角 {a[1]} (deg)")
    p_free.add_argument("--speed", type=float, default=50.0, help="速度百分比")
    p_free.add_argument("--oneshot", type=int, choices=[0, 1], default=1, help="1=执行一次, 0=循环")
    # 自由路径增量
    p_fby = sub.add_parser("free_by", help="自由路径增量")
    for a in ["dj1", "dj2", "dj3", "dj4", "dj5", "dj6"]:
        p_fby.add_argument(f"--{a}", type=float, default=0.0, help=f"关节角 {a[1]} 增量 (deg)")
    p_fby.add_argument("--speed", type=float, default=50.0, help="速度百分比")
    p_fby.add_argument("--oneshot", type=int, choices=[0, 1], default=1, help="1=执行一次, 0=循环")
    # 姿势直线
    p_pose = sub.add_parser("pose", help="姿势直线 (action=10)")
    for a in ["x", "y", "z", "u", "v", "w"]:
        p_pose.add_argument(f"--{a}", type=float, default=0.0, help=f"世界坐标 {a.upper()} (mm/deg)")
    p_pose.add_argument("--speed", type=float, default=50.0, help="速度百分比")
    p_pose.add_argument("--oneshot", type=int, choices=[0, 1], default=1, help="1=执行一次, 0=循环")
    # 姿势直线增量
    p_pby = sub.add_parser("pose_by", help="姿势直线增量")
    for a in ["dx", "dy", "dz", "du", "dv", "dw"]:
        p_pby.add_argument(f"--{a}", type=float, default=0.0, help=f"世界坐标增量 {a[1:]} (mm/deg)")
    p_pby.add_argument("--speed", type=float, default=50.0, help="速度百分比")
    p_pby.add_argument("--oneshot", type=int, choices=[0, 1], default=1, help="1=执行一次, 0=循环")
    # 保持姿态，沿末端局部X方向移动指定三维距离
    p_tool = sub.add_parser("tool_x", help="保持姿态，沿末端当前局部X方向移动")
    p_tool.add_argument("distance_mm", type=float, help="空间距离(mm)，正值+X，负值-X，范围±60")
    p_tool.add_argument("--speed", type=float, default=1.0, help="物理速度(mm/s)，默认1")
    # 控制命令
    sub.add_parser("stop", help="急停")
    sub.add_parser("clear_alarm", help="清除报警")
    sub.add_parser("home", help="回零")
    p_enable = sub.add_parser("enable", help="使能控制")
    p_enable.add_argument("state", choices=["on", "off"], help="on=使能, off=禁用")
    p_global = sub.add_parser("global_speed", help="全局速度百分比")
    p_global.add_argument("pct", type=float, help="速度百分比 0-100")
    args = parser.parse_args()
    robot = HC1JsonRobot()
    if args.cmd == "connect":
        robot.host = args.host
        robot.port = args.port
        robot.connect()
    elif args.cmd is None or not robot.sock:
        robot.connect()
    try:
        if args.cmd == "read_joints":
            j = robot.read_joints()
            if j: print(j)
        elif args.cmd == "read_pose":
            p = robot.read_world_pose()
            if p: print(p)
        elif args.cmd == "read_status":
            s = robot.read_status_pose()
            if s: print(s)
        elif args.cmd == "speed":
            robot.set_physical_speed(args.speed_mm_s, one_shot=bool(args.oneshot))
        elif args.cmd == "speed_off":
            robot.disable_physical_speed()
        elif args.cmd == "free":
            robot.move_free_path(args.j1, args.j2, args.j3, args.j4, args.j5, args.j6,
                                 speed_pct=args.speed, one_shot=bool(args.oneshot))
        elif args.cmd == "free_by":
            robot.move_free_path_by(args.dj1, args.dj2, args.dj3, args.dj4, args.dj5, args.dj6,
                                    speed_pct=args.speed, one_shot=bool(args.oneshot))
        elif args.cmd == "pose":
            robot.move_pose_line(args.x, args.y, args.z, args.u, args.v, args.w,
                                 speed_pct=args.speed, one_shot=bool(args.oneshot))
        elif args.cmd == "pose_by":
            robot.move_pose_line_by(args.dx, args.dy, args.dz, args.du, args.dv, args.dw,
                                    speed_pct=args.speed, one_shot=bool(args.oneshot))
        elif args.cmd == "tool_x":
            robot.move_along_tool_x(args.distance_mm, speed_mm_s=args.speed)
        elif args.cmd == "stop":
            robot.emergency_stop()
        elif args.cmd == "clear_alarm":
            robot.clear_alarm()
        elif args.cmd == "home":
            robot.home()
        elif args.cmd == "enable":
            if args.state == "on": robot.enable()
            else: robot.disable()
        elif args.cmd == "global_speed":
            robot.set_global_speed(args.pct)
        elif args.cmd == "disconnect":
            robot.disconnect()
            exit(0)
    finally:
        if args.cmd != "disconnect":
            robot.disconnect()
