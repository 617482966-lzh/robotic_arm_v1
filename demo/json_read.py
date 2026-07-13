# -*- coding: utf-8 -*-
"""
HC1 / 伯朗特机器人 JSON 获取末端位姿

功能：
1. 连接机器人控制器
2. 使用 JSON RemoteMonitor 查询当前状态
3. 使用 JSON RemoteMonitor 查询末端世界坐标 world-0 ~ world-5
4. 不发送任何运动控制指令

查询地址来自文档：
axis-n  : 轴位置，0:J1,1:J2,2:J3,3:J4,4:J5,5:J6
world-n : 世界坐标轴位置，0:X,1:Y,2:Z,3:U,4:V,5:W,6:M7,7:M8
"""

import socket
import json
import time


HOST = "192.168.1.4"
PORT = 9760

DSID_MONITOR = "www.hc-system.com.RemoteMonitor"


class HC1JsonReader:
    def __init__(self, host=HOST, port=PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.pack_id = 1000

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(3.0)
        self.sock.connect((self.host, self.port))
        print(f"[OK] 已连接 {self.host}:{self.port}")

    def disconnect(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            print("[OK] 已断开连接")

    def recv_json(self, timeout=3.0):
        """
        接收 JSON 回复。
        机器人通常以 \\r\\n 结尾，但这里采用累积解析方式，兼容无换行情况。
        """
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
                    # 可能还没收完整，继续接收
                    continue

            except socket.timeout:
                break

        if data:
            print("[WARN] 收到数据但 JSON 解析失败:")
            print(data.decode("utf-8", errors="ignore"))

        return None

    def query(self, addrs, show_raw=False):
        """
        JSON RemoteMonitor query 查询。
        """
        self.pack_id += 1

        req = {
            "dsID": DSID_MONITOR,
            "reqType": "query",
            "packID": str(self.pack_id),
            "queryAddr": list(addrs)
        }

        raw = json.dumps(req, ensure_ascii=False) + "\r\n"

        if show_raw:
            print("\n>>> SEND")
            print(json.dumps(req, ensure_ascii=False, indent=2))

        self.sock.sendall(raw.encode("utf-8"))

        rep = self.recv_json(timeout=3.0)

        if show_raw:
            print("<<< RECV")
            print(rep)

        return rep

    def read_status(self):
        """
        读取当前模式、运动状态、报警状态。
        """
        addrs = [
            "curMode",
            "isMoving",
            "curAlarm",
            "axisNum",
            "origin",
        ]

        rep = self.query(addrs)

        if not rep or "queryData" not in rep:
            return None

        q = rep["queryData"]

        status = {}

        try:
            status["curMode"] = int(float(q[0])) if len(q) > 0 else None
            status["isMoving"] = int(float(q[1])) if len(q) > 1 else None
            status["curAlarm"] = int(float(q[2])) if len(q) > 2 else None
            status["axisNum"] = int(float(q[3])) if len(q) > 3 else None
            status["origin"] = q[4] if len(q) > 4 else None
        except Exception:
            status["raw"] = q

        return status

    def read_axes(self):
        """
        读取六轴关节角。
        axis-0~axis-5:
            0:J1, 1:J2, 2:J3, 3:J4, 4:J5, 5:J6
        """
        addrs = [
            "axis-0",
            "axis-1",
            "axis-2",
            "axis-3",
            "axis-4",
            "axis-5",
        ]

        rep = self.query(addrs)

        if not rep or "queryData" not in rep:
            return None

        q = rep["queryData"]

        if len(q) < 6:
            return None

        try:
            return {
                "j1": float(q[0]),
                "j2": float(q[1]),
                "j3": float(q[2]),
                "j4": float(q[3]),
                "j5": float(q[4]),
                "j6": float(q[5]),
            }
        except Exception:
            return {
                "raw": q
            }

    def read_world_pose(self):
        """
        读取末端世界坐标。

        文档：
        world-n：世界坐标轴位置
            n 从 0 开始
            0:X
            1:Y
            2:Z
            3:U
            4:V
            5:W
            6:M7
            7:M8

        这里先读取 world-0~world-5。
        """
        addrs = [
            "world-0",
            "world-1",
            "world-2",
            "world-3",
            "world-4",
            "world-5",
        ]

        rep = self.query(addrs, show_raw=True)

        if not rep or "queryData" not in rep:
            print("[FAIL] world 位姿读取失败")
            return None

        q = rep["queryData"]

        if len(q) < 6:
            print("[FAIL] world queryData 长度不足:", q)
            return None

        try:
            pose = {
                "x": float(q[0]),
                "y": float(q[1]),
                "z": float(q[2]),
                "u": float(q[3]),
                "v": float(q[4]),
                "w": float(q[5]),
            }
            return pose

        except Exception as e:
            print("[FAIL] world 位姿数值解析失败:", e)
            print("queryData =", q)
            return None

    def read_world_pose_8axis(self):
        """
        如果你想把 M7、M8 也读出来，用这个函数。
        """
        addrs = [
            "world-0",
            "world-1",
            "world-2",
            "world-3",
            "world-4",
            "world-5",
            "world-6",
            "world-7",
        ]

        rep = self.query(addrs)

        if not rep or "queryData" not in rep:
            return None

        q = rep["queryData"]

        try:
            data = {}
            names = ["x", "y", "z", "u", "v", "w", "m7", "m8"]

            for name, value in zip(names, q):
                data[name] = float(value)

            return data

        except Exception:
            return {
                "raw": q
            }

    def print_all(self):
        print("\n================ JSON 当前信息 ================")

        status = self.read_status()
        axes = self.read_axes()
        pose = self.read_world_pose()

        print("\n[状态]")
        if status:
            for k, v in status.items():
                print(f"{k:<10} = {v}")
        else:
            print("读取失败")

        print("\n[六轴关节角 axis-n / deg]")
        if axes and "raw" not in axes:
            print(f"J1 = {axes['j1']:.6f}")
            print(f"J2 = {axes['j2']:.6f}")
            print(f"J3 = {axes['j3']:.6f}")
            print(f"J4 = {axes['j4']:.6f}")
            print(f"J5 = {axes['j5']:.6f}")
            print(f"J6 = {axes['j6']:.6f}")
        elif axes:
            print(axes["raw"])
        else:
            print("读取失败")

        print("\n[末端世界坐标 world-n]")
        if pose:
            print(f"X = {pose['x']:.3f} mm")
            print(f"Y = {pose['y']:.3f} mm")
            print(f"Z = {pose['z']:.3f} mm")
            print(f"U = {pose['u']:.3f} deg")
            print(f"V = {pose['v']:.3f} deg")
            print(f"W = {pose['w']:.3f} deg")
        else:
            print("读取失败")

        print("\n================================================")


def main():
    robot = HC1JsonReader(HOST, PORT)

    try:
        robot.connect()

        # 只读取一次，不控制运动
        robot.print_all()

        print("\n[DONE] JSON 读取完成，没有发送任何运动控制指令。")

    except ConnectionRefusedError:
        print(f"[FAIL] 无法连接 {HOST}:{PORT}")

    except socket.timeout:
        print("[FAIL] 通信超时")

    except KeyboardInterrupt:
        print("\n[STOP] 用户中断")

    except Exception as e:
        print("[FAIL] 程序异常:", e)

    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()