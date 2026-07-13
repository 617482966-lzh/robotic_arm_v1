# -*- coding: utf-8 -*-
"""
Borunte HC1 6-axis robot - Remote Communication Protocol (JSON/TCP) Python client
Port: 9760  |  Doc: BRT-C-RC-HC1-TI1-V1.4
JSON details aligned with demo/ (\r\n delimiter, lowercase "oneshot", hex ckStatus)

Capabilities:
  Query:     world coords, joint positions, IO, mode, alarms
  Command:   start/stop, speed, IO, clear alarm
  AddRCC:    posture line/curve, free path, absolute speed
  Monitor:   sync/async end-effector world position
  High-level: move_to, move_jog, home, enable, disable, set_speed, get_position
"""

import json
import socket
import time
import threading


class BorunteRobot:

    def __init__(self, host: str, port: int = 9760):
        self.host = host
        self.port = port
        self.sock = None
        self._lock = threading.Lock()
        self._heartbeat_running = False

    def connect(self, timeout: float = 5.0, retries: int = 3) -> bool:
        for attempt in range(retries):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(timeout)
                self.sock.connect((self.host, self.port))
                print(f"[OK] Connected to {self.host}:{self.port}")
                self._start_heartbeat()
                return True
            except Exception as e:
                if attempt < retries - 1:
                    print(f"[RETRY] ({attempt+1}/{retries}) {e}")
                    time.sleep(1)
                else:
                    print(f"[FAIL] Connection failed after {retries} attempts: {e}")
                    self.sock = None
                    return False

    def disconnect(self):
        self._heartbeat_running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            print("[OK] Disconnected")

    def is_connected(self) -> bool:
        return self.sock is not None

    def _send_frame(self, frame: dict) -> dict | None:
        if not self.sock:
            print("[FAIL] Not connected")
            return None
        with self._lock:
            try:
                data = (json.dumps(frame, ensure_ascii=False) + "\r\n").encode("utf-8")
                self.sock.sendall(data)
                buf = b""
                while True:
                    chunk = self.sock.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                    try:
                        return json.loads(buf.decode("utf-8"))
                    except json.JSONDecodeError:
                        continue
            except socket.timeout:
                print("[WARN] Receive timeout")
                return None
            except Exception as e:
                print(f"[FAIL] Communication error: {e}")
                return None

    def heartbeat(self):
        return self._send_frame({
            "dsID": "www.hc-system.com.RemoteMonitor",
            "reqType": "heartbreak",
        })

    def _start_heartbeat(self):
        self._heartbeat_running = True
        def _loop():
            while self._heartbeat_running and self.sock:
                self.heartbeat()
                time.sleep(8)
        t = threading.Thread(target=_loop, daemon=True)
        t.start()

    def query(self, addrs: list[str], pack_id: str = "0") -> dict | None:
        frame = {
            "dsID": "www.hc-system.com.RemoteMonitor",
            "reqType": "query",
            "packID": pack_id,
            "queryAddr": addrs,
        }
        return self._send_frame(frame)

    def read_world_position(self) -> list[float] | None:
        reply = self.query([f"world-{i}" for i in range(8)])
        if reply and "queryData" in reply:
            return [float(v) for v in reply["queryData"]]
        return None

    def read_joint_position(self) -> list[float] | None:
        reply = self.query([f"axis-{i}" for i in range(8)])
        if reply and "queryData" in reply:
            return [float(v) for v in reply["queryData"]]
        return None

    def is_moving(self) -> bool:
        reply = self.query(["isMoving"])
        if reply and "queryData" in reply:
            return reply["queryData"][0] == "1"
        return False

    def current_mode(self) -> str:
        modes = {"1": "manual", "2": "auto", "3": "stop",
                 "7": "auto-running", "8": "step", "9": "single-cycle"}
        reply = self.query(["curMode"])
        if reply and "queryData" in reply:
            return modes.get(reply["queryData"][0], reply["queryData"][0])
        return "?"

    def monitor_position(self, interval: float = 0.05, callback=None,
                         stop_on_idle: bool = True) -> list[list[float]]:
        trajectory = []
        print(f"[MONITOR] Starting (interval={interval*1000:.0f}ms)")
        idle = 0
        try:
            while True:
                pos = self.read_world_position()
                if pos is None:
                    time.sleep(interval)
                    continue
                trajectory.append(pos)
                if callback:
                    callback(pos)
                if stop_on_idle:
                    if not self.is_moving() and len(trajectory) >= 2:
                        prev = trajectory[-2]
                        if (abs(pos[0] - prev[0]) < 0.01 and
                            abs(pos[1] - prev[1]) < 0.01 and
                            abs(pos[2] - prev[2]) < 0.01):
                            idle += 1
                        else:
                            idle = 0
                    if idle >= 3:
                        print(f"[MONITOR] Movement stopped, {len(trajectory)} points")
                        break
                time.sleep(interval)
        except KeyboardInterrupt:
            print(f"\n[MONITOR] Interrupted, {len(trajectory)} points")
        return trajectory

    def monitor_position_async(self, interval: float = 0.05, callback=None):
        stop_event = threading.Event()
        def _loop():
            while not stop_event.is_set():
                pos = self.read_world_position()
                if pos and callback:
                    callback(pos)
                time.sleep(interval)
        t = threading.Thread(target=_loop, daemon=True)
        t.start()
        return stop_event.set

    def command(self, cmd: str, *args: str, pack_id: str = "0") -> dict | None:
        frame = {
            "dsID": "www.hc-system.com.RemoteMonitor",
            "reqType": "command",
            "packID": pack_id,
            "cmdData": [cmd] + list(args),
        }
        return self._send_frame(frame)

    def stop(self):         return self.command("actionStop")
    def start(self):        return self.command("startButton")
    def pause(self):        return self.command("actionPause")
    def single_cycle(self): return self.command("actionSingleCycle")

    def set_speed_pct(self, pct: float):
        return self.command("modifyGSPD", str(int(pct * 10)))

    def add_rcc(self, instructions: list[dict], clear_first: bool = True,
                pack_id: str = "0") -> dict | None:
        frame = {
            "dsID": "www.hc-system.com.HCRemoteCommand",
            "reqType": "AddRCC",
            "emptyList": "1" if clear_first else "0",
            "packID": pack_id,
            "instructions": instructions,
        }
        return self._send_frame(frame)

    @staticmethod
    def _fmt(v: float) -> str:
        return f"{v:.3f}"

    def _make_move_inst(self, action: str, m: tuple, ck_status: int,
                        speed: str, tool: int, coord: int, smooth: int,
                        one_shot: bool = False, **extra) -> dict:
        inst = {
            "oneshot":  "1" if one_shot else "0",
            "action":   action,
            "m0": self._fmt(m[0]), "m1": self._fmt(m[1]),
            "m2": self._fmt(m[2]), "m3": self._fmt(m[3]),
            "m4": self._fmt(m[4]), "m5": self._fmt(m[5]),
            "m6": self._fmt(m[6]), "m7": self._fmt(m[7]),
            "ckStatus": "0x{:X}".format(ck_status) if ck_status >= 0 else "0X3F",
            "speed":    speed,
            "tool":     str(tool),
            "coord":    str(coord),
            "smooth":   str(smooth),
        }
        inst.update(extra)
        return inst

    def move_linear(self,
                    x: float, y: float, z: float,
                    u: float = 0, v: float = 0, w: float = 0,
                    m7: float = 0, m8: float = 0,
                    speed_pct: float = 50.0,
                    tool: int = 0, coord: int = 0,
                    smooth: int = 5, one_shot: bool = False,
                    ck_status: int = 63) -> dict | None:
        inst = self._make_move_inst(
            action="10", m=(x, y, z, u, v, w, m7, m8),
            ck_status=ck_status,
            speed=str(int(speed_pct)),
            tool=tool, coord=coord, smooth=smooth,
            one_shot=one_shot,
        )
        return self.add_rcc([inst], clear_first=True)

    def move_linear_mms(self,
                        x: float, y: float, z: float,
                        u: float = 0, v: float = 0, w: float = 0,
                        m7: float = 0, m8: float = 0,
                        speed_mm_s: float = 10.0,
                        tool: int = 0, coord: int = 0,
                        smooth: int = 5,
                        ck_status: int = 63) -> dict | None:
        instructions = [
            {
                "oneshot": "1",
                "action":  "51",
                "isUse":   "1",
                "speed":   str(int(speed_mm_s)),
            },
            self._make_move_inst(
                action="10", m=(x, y, z, u, v, w, m7, m8),
                ck_status=ck_status,
                speed=str(int(50)),
                tool=tool, coord=coord, smooth=smooth,
                one_shot=False,
            ),
        ]
        return self.add_rcc(instructions, clear_first=True)

    # ---- High-level convenience methods ----

    def get_position(self) -> dict | None:
        pos = self.read_world_position()
        if pos and len(pos) >= 6:
            return {"x": pos[0], "y": pos[1], "z": pos[2],
                    "rx": pos[3], "ry": pos[4], "rz": pos[5]}
        return None

    def move_to(self, x: float, y: float, z: float,
                rx: float, ry: float, rz: float,
                speed_pct: float = 50) -> dict | None:
        return self.move_linear(x, y, z, rx, ry, rz, speed_pct=speed_pct)

    def move_jog(self, axis: str, direction: str,
                 step: float, speed_pct: float = 50) -> bool:
        pos = self.get_position()
        if not pos:
            return False
        axis_map = {"X": "x", "Y": "y", "Z": "z",
                    "Rx": "rx", "Ry": "ry", "Rz": "rz"}
        ak = axis_map.get(axis)
        if not ak:
            return False
        pos[ak] += step if direction == "+" else -step
        self.move_to(pos["x"], pos["y"], pos["z"],
                     pos["rx"], pos["ry"], pos["rz"], speed_pct)
        return True

    def home(self) -> dict | None:
        return self.command("home", "1", "1")

    def enable(self) -> dict | None:
        return self.command("run", "1", "1")

    def disable(self) -> dict | None:
        return self.command("stop", "1", "1")

    def set_speed(self, pct: float) -> dict | None:
        return self.set_speed_pct(pct)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Borunte HC1 Robot Remote Control")
    parser.add_argument("--host", default="192.168.1.100", help="Robot IP address")
    parser.add_argument("--port", type=int, default=9760)
    parser.add_argument("--demo", choices=["z78", "u12", "monitor"], default="u12")
    args = parser.parse_args()

    robot = BorunteRobot(args.host, args.port)
    try:
        if not robot.connect():
            exit(1)
        if args.demo == "u12":
            print("=" * 55)
            print("  End-effector U +12 deg (rotate about world X), 1 mm/s")
            print("=" * 55)
            pos = robot.read_world_position()
            if pos:
                print(f"Start: X={pos[0]:.3f} Y={pos[1]:.3f} Z={pos[2]:.3f}  U={pos[3]:.3f} V={pos[4]:.3f} W={pos[5]:.3f}")
            else:
                print("[WARN] Cannot read current position")
                pos = [0.0] * 8
            target = pos.copy()
            target[3] += 12.0
            def print_pos(p):
                print(f"  [RT] X={p[0]:8.3f} Y={p[1]:8.3f} Z={p[2]:8.3f}  U={p[3]:8.3f} V={p[4]:8.3f} W={p[5]:8.3f}")
            stop_monitor = robot.monitor_position_async(interval=0.1, callback=print_pos)
            print(f"\nTarget: U={target[3]:.3f}  Speed: 1 mm/s")
            reply = robot.move_linear_mms(x=target[0], y=target[1], z=target[2],
                u=target[3], v=target[4], w=target[5], m7=target[6], m8=target[7],
                speed_mm_s=1.0, ck_status=8)
            print(f"Reply: {reply}")
            time.sleep(0.5)
            while robot.is_moving():
                time.sleep(0.2)
            stop_monitor()
            print("\nDone.")
        elif args.demo == "z78":
            print("Demo: Move Z- 78 mm")
            pos = robot.read_world_position()
            if pos: print(f"  Start: X={pos[0]:.3f} Y={pos[1]:.3f} Z={pos[2]:.3f}")
            robot.move_linear(x=0, y=0, z=-78, speed_pct=30, ck_status=7)
            time.sleep(0.5)
            while robot.is_moving(): time.sleep(0.2)
            pos = robot.read_world_position()
            if pos: print(f"  End:   X={pos[0]:.3f} Y={pos[1]:.3f} Z={pos[2]:.3f}")
        elif args.demo == "monitor":
            print("Real-time position monitor, Ctrl+C to stop")
            robot.monitor_position(interval=0.1,
                callback=lambda p: print(f"  X={p[0]:7.3f} Y={p[1]:7.3f} Z={p[2]:7.3f}  U={p[3]:7.3f} V={p[4]:7.3f} W={p[5]:7.3f}"),
                stop_on_idle=False)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        robot.disconnect()
