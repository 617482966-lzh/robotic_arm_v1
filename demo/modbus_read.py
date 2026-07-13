# -*- coding: utf-8 -*-
"""
HC1 机械臂当前位置读取程序

功能：
1. 连接 HC1 控制器
2. 读取 JSON 状态：curMode、isMoving、curAlarm、axis-0~axis-5
3. 读取 Modbus TCP 末端位姿：X、Y、Z、Rx、Ry、Rz

注意：
- 本程序只读取信息，不控制机械臂运动。
- 不发送 clearAlarm。
- 不发送 stop。
- 不发送 AddRCC。
"""

import socket
import struct
import json
import time


HOST = "192.168.1.4"
PORT = 9760

UNIT_ID = 1

# 已通过实测确认：
# 0x091C -> X
# 0x091E -> Y
# 0x0920 -> Z
# 0x0922 -> Rx
# 0x0924 -> Ry
# 0x0926 -> Rz
POSE_START_ADDR = 0x091C

# 6 个 32 位数，每个数占 2 个 16 位寄存器，共 12 个寄存器
POSE_REG_QTY = 12


class HC1PositionReader:
    def __init__(self, host=HOST, port=PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.pack_id = 1000
        self.tid = 2000

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

    def recv_exact(self, n, timeout=3.0):
        self.sock.settimeout(timeout)
        data = b""
        while len(data) < n:
            chunk = self.sock.recv(n - len(data))
            if not chunk:
                break
            data += chunk
        return data

    # =========================================================
    # JSON 状态查询
    # =========================================================
    def json_query(self, query_addrs):
        self.pack_id += 1

        req = {
            "dsID": "www.hc-system.com.RemoteMonitor",
            "reqType": "query",
            "packID": str(self.pack_id),
            "queryAddr": list(query_addrs)
        }

        raw = json.dumps(req, ensure_ascii=False) + "\r\n"

        self.sock.sendall(raw.encode("utf-8"))

        data = self.sock.recv(4096)
        text = data.decode("utf-8", errors="ignore").strip()

        try:
            return json.loads(text)
        except Exception as e:
            print("[WARN] JSON 解析失败:", e)
            print("原始返回:", text)
            return None

    def read_json_status_and_axes(self):
        addrs = [
            "curMode",
            "isMoving",
            "curAlarm",
            "axis-0",
            "axis-1",
            "axis-2",
            "axis-3",
            "axis-4",
            "axis-5",
        ]

        rep = self.json_query(addrs)

        if not rep or "queryData" not in rep:
            print("[FAIL] JSON 状态读取失败")
            return None

        q = rep["queryData"]

        if len(q) < 9:
            print("[FAIL] queryData 长度不足:", q)
            return None

        status = {
            "curMode": int(float(q[0])),
            "isMoving": int(float(q[1])),
            "curAlarm": int(float(q[2])),
            "axis": [
                float(q[3]),
                float(q[4]),
                float(q[5]),
                float(q[6]),
                float(q[7]),
                float(q[8]),
            ]
        }

        return status

    # =========================================================
    # Modbus TCP 读取保持寄存器
    # =========================================================
    def modbus_read_holding_registers(self, start_addr, qty):
        self.tid += 1

        function_code = 0x03

        # PDU: 功能码 + 起始地址 + 数量
        pdu = struct.pack(">BHH", function_code, start_addr, qty)

        # MBAP: tid, protocol, length, unit_id
        # length = unit_id 1字节 + PDU长度
        mbap = struct.pack(">HHHB", self.tid, 0, len(pdu) + 1, UNIT_ID)

        req = mbap + pdu

        self.sock.sendall(req)

        header = self.recv_exact(7, timeout=3.0)

        if len(header) < 7:
            print("[FAIL] Modbus 响应头长度不足")
            return None

        r_tid, r_proto, r_length, r_unit = struct.unpack(">HHHB", header)

        body_len = r_length - 1
        body = self.recv_exact(body_len, timeout=3.0)

        if len(body) < body_len:
            print("[FAIL] Modbus 响应 body 长度不足")
            return None

        if len(body) < 2:
            print("[FAIL] Modbus body 太短")
            return None

        func = body[0]

        if func & 0x80:
            err_code = body[1] if len(body) > 1 else None
            print(f"[FAIL] Modbus 异常响应, func=0x{func:02X}, err_code={err_code}")
            return None

        if func != function_code:
            print(f"[FAIL] 功能码不匹配，期望 0x03，实际 0x{func:02X}")
            return None

        byte_count = body[1]
        data = body[2:2 + byte_count]

        if len(data) != byte_count:
            print("[FAIL] Modbus 数据长度不匹配")
            return None

        regs = []
        for i in range(0, len(data), 2):
            reg = struct.unpack(">H", data[i:i + 2])[0]
            regs.append(reg)

        return regs

    # =========================================================
    # 寄存器转 32 位有符号数
    # =========================================================
    @staticmethod
    def u32_to_s32(v):
        if v >= 0x80000000:
            v -= 0x100000000
        return v

    @staticmethod
    def regs_to_s32_scaled_values(regs, scale=1000.0):
        """
        每两个 16 位寄存器合成一个 32 位有符号整数。
        实测为高字在前 HL。

        例如：
        0x0008 0xE27A -> 0x0008E27A -> 582266 -> 582.266
        """
        vals = []

        for i in range(0, len(regs) - 1, 2):
            hi = regs[i]
            lo = regs[i + 1]

            raw = (hi << 16) | lo

            signed = HC1PositionReader.u32_to_s32(raw)

            vals.append(signed / scale)

        return vals

    def read_tcp_pose_modbus(self):
        regs = self.modbus_read_holding_registers(POSE_START_ADDR, POSE_REG_QTY)

        if regs is None:
            return None

        vals = self.regs_to_s32_scaled_values(regs, scale=1000.0)

        if len(vals) < 6:
            print("[FAIL] 解析出的位姿数量不足")
            return None

        pose = {
            "x": vals[0],
            "y": vals[1],
            "z": vals[2],
            "rx": vals[3],
            "ry": vals[4],
            "rz": vals[5],
        }

        return pose

    # =========================================================
    # 总读取函数
    # =========================================================
    def read_all(self):
        status = self.read_json_status_and_axes()
        pose = self.read_tcp_pose_modbus()

        print("\n================ 当前机械臂信息 ================")

        if status:
            print("\n[控制器状态]")
            print(f"curMode  = {status['curMode']}")
            print(f"isMoving = {status['isMoving']}")
            print(f"curAlarm = {status['curAlarm']}")

            print("\n[六轴关节角 deg]")
            for i, a in enumerate(status["axis"]):
                print(f"axis-{i} = {a:.6f}")

        else:
            print("\n[控制器状态]")
            print("读取失败")

        if pose:
            print("\n[末端位姿]")
            print(f"X  = {pose['x']:.3f} mm")
            print(f"Y  = {pose['y']:.3f} mm")
            print(f"Z  = {pose['z']:.3f} mm")
            print(f"Rx = {pose['rx']:.3f} deg")
            print(f"Ry = {pose['ry']:.3f} deg")
            print(f"Rz = {pose['rz']:.3f} deg")
        else:
            print("\n[末端位姿]")
            print("读取失败")

        print("\n================================================")


def main():
    reader = HC1PositionReader(HOST, PORT)

    try:
        reader.connect()

        # 只读取一次
        reader.read_all()

        print("\n[DONE] 本程序只读取当前位置，没有发送任何运动控制指令。")

    except ConnectionRefusedError:
        print(f"[FAIL] 无法连接 {HOST}:{PORT}")

    except socket.timeout:
        print("[FAIL] 通信超时")

    except KeyboardInterrupt:
        print("\n[STOP] 用户中断")

    except Exception as e:
        print("[FAIL] 程序异常:", e)

    finally:
        reader.disconnect()


if __name__ == "__main__":
    main()