# -*- coding: utf-8 -*-
"""
Mock Borunte HC1 robot server for local testing.
Listens on 127.0.0.1:9760, simulates the remote JSON protocol responses.
"""

import json
import socket
import threading
import time
import math


class MockRobot:
    """Simulates robot state and responds to HC1 remote protocol queries."""

    def __init__(self):
        # Current world position [X, Y, Z, U, V, W, M7, M8]
        self.world = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Current joint angles [J1..J8]
        self.joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.mode = "7"       # 7 = auto-running
        self.moving = False
        self.alarm = "0"
        self.speed_pct = 500  # 50.0%
        self._motion_thread = None
        self._lock = threading.Lock()

    def handle_frame(self, frame: dict) -> dict | None:
        """Process one JSON frame, return response or None."""
        dsid  = frame.get("dsID", "")
        rtype = frame.get("reqType", frame.get("cmdType", ""))

        # ---- Heartbeat ----
        if rtype == "heartbreak":
            return {"dsID": dsid, "reqType": "heartbreak"}

        # ---- Query ----
        if rtype == "query":
            return self._handle_query(frame)

        # ---- Command ----
        if rtype == "command":
            return self._handle_command(frame)

        # ---- Remote Teach (AddRCC) ----
        if rtype == "AddRCC":
            return self._handle_add_rcc(frame)

        # Unknown - echo back
        return {"dsID": dsid, "reqType": rtype, "cmdReply": ["unknown"]}

    # ==================================================================
    # Query handlers
    # ==================================================================

    def _handle_query(self, frame: dict) -> dict:
        addrs = frame.get("queryAddr", [])
        data = []
        for addr in addrs:
            data.append(self._query_one(addr))

        # v2.0.2.1+: reply uses same reqType as request
        return {
            "dsID": frame.get("dsID", "www.hc-system.com.RemoteMonitor"),
            "reqType": "query",
            "packID": frame.get("packID", "0"),
            "queryData": data,
        }

    def _query_one(self, addr: str) -> str:
        """Return string value for a single query address."""
        with self._lock:
            # World coordinates
            if addr.startswith("world-"):
                idx = int(addr.split("-")[1])
                if 0 <= idx < 8:
                    return f"{self.world[idx]:.3f}"
                return "0.000"

            # Joint positions
            if addr.startswith("axis-"):
                idx = int(addr.split("-")[1])
                if 0 <= idx < 8:
                    return f"{self.joints[idx]:.3f}"
                return "0.000"

            # Simple single-value queries
            simple = {
                "version":   "HC-RX-7.8.02-mock",
                "curMold":   "default",
                "curMode":   self.mode,
                "axisNum":   "6",
                "isMoving":  "1" if self.moving else "0",
                "curAlarm":  self.alarm,
                "origin":    "1",
                "machineName": "MOCK-001",
            }
            if addr in simple:
                return simple[addr]

            # Tool / Coord
            if addr == "toolCoord":
                return '["0","0","0"]'

            # Remote cmd length
            if addr == "RemoteCmdLen":
                return "0"

            # Counters, IO, M values, etc
            if addr.startswith("counter-"):
                return '["0","0","0"]'
            if addr == "counterList":
                return "[]"
            if addr.startswith("input-") or addr.startswith("output-"):
                return "0"
            if addr.startswith("M-"):
                return "0"
            if addr == "boardIONum":
                return "1"
            if addr.startswith("curTorque-"):
                return "0"
            if addr.startswith("curSpeed-"):
                return "0"
            if addr == "curCycle" or addr == "lastCycle":
                return "0"
            if addr.startswith("Addr-"):
                return "0"
            if addr == "curAccount":
                return "0"
            if addr == "moldList":
                return '["default"]'

            return "0"

    # ==================================================================
    # Command handlers
    # ==================================================================

    def _handle_command(self, frame: dict) -> dict:
        cmd_data = frame.get("cmdData", [])
        cmd = cmd_data[0] if cmd_data else ""

        reply = ["ok"]
        with self._lock:
            if cmd == "actionStop":
                self.moving = False
                self.mode = "3"
            elif cmd == "startButton":
                self.mode = "7"
            elif cmd == "actionPause":
                self.moving = False
            elif cmd == "actionSingleCycle":
                self.mode = "9"
            elif cmd == "modifyGSPD" and len(cmd_data) > 1:
                self.speed_pct = int(cmd_data[1])
            elif cmd == "clearAlarm":
                self.alarm = "0"
            elif cmd == "clearAlarmRunNext":
                self.alarm = "0"
            elif cmd == "clearAlarmContinue":
                self.alarm = "0"

        return {
            "dsID": frame.get("dsID", "www.hc-system.com.RemoteMonitor"),
            "reqType": "command",
            "packID": frame.get("packID", "0"),
            "cmdReply": reply,
        }

    # ==================================================================
    # AddRCC handler - simulates motion
    # ==================================================================

    def _handle_add_rcc(self, frame: dict) -> dict:
        instructions = frame.get("instructions", [])
        empty = frame.get("emptyList", "0")

        # Process instructions: find the last motion instruction and animate to it
        targets = []
        use_abs_speed = False
        abs_speed_mm_s = 10.0

        for inst in instructions:
            action = inst.get("action", "")
            if action == "51":
                use_abs_speed = True
                abs_speed_mm_s = float(inst.get("speed", "10"))
            elif action in ("4", "10", "17"):
                m = [float(inst.get(f"m{i}", "0")) for i in range(8)]
                ck_text = str(inst.get("ckStatus", "63"))
                ck = int(ck_text, 0)
                targets.append((action, m, ck, use_abs_speed, abs_speed_mm_s))

        if targets:
            # Take the last motion target
            _, m, ck, use_spd, spd = targets[-1]
            # Start animation in background
            threading.Thread(target=self._animate_motion,
                             args=(m, ck, use_spd, spd), daemon=True).start()

        return {
            "dsID": frame.get("dsID", "HCRemoteCommand"),
            "reqType": "AddRCC",
            "packID": frame.get("packID", "0"),
            "cmdReply": ["AddRCC", "ok"],
        }

    def _animate_motion(self, target: list[float], ck: int,
                        use_abs_speed: bool, abs_speed: float):
        """Simulate motion from current position to target."""
        with self._lock:
            start = self.world.copy()
            self.moving = True

        # Determine which axes move (ck bitmask, bit1~bit8)
        active = [bool(ck & (1 << i)) for i in range(8)]

        # Calculate distance (only on active axes)
        dist = 0.0
        for i in range(8):
            if active[i]:
                d = target[i] - start[i]
                dist += d * d
        dist = math.sqrt(dist)

        if dist < 0.001:
            with self._lock:
                self.moving = False
            return

        # Simulate motion: use absolute speed if set, else ~50mm/s default
        speed = abs_speed if use_abs_speed else 50.0  # mm/s
        duration = dist / speed
        duration = max(duration, 0.5)  # at least 0.5s to be observable
        steps = max(int(duration / 0.05), 10)
        dt = duration / steps

        for step in range(1, steps + 1):
            t = step / steps
            with self._lock:
                for i in range(8):
                    if active[i]:
                        self.world[i] = start[i] + (target[i] - start[i]) * t
                        # Rough IK → joint approximation for display
                        self.joints[i] = self.world[i] * 0.1
            time.sleep(dt)

        # Snap to exact target
        with self._lock:
            for i in range(8):
                if active[i]:
                    self.world[i] = target[i]
            self.moving = False


# ======================================================================
# TCP server
# ======================================================================

class MockRobotServer:
    """TCP server that accepts JSON frames and routes them to MockRobot."""

    def __init__(self, host: str = "127.0.0.1", port: int = 9760):
        self.host = host
        self.port = port
        self.robot = MockRobot()
        self._running = False

    def start(self):
        self._running = True
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Allow quick restart on Windows
        server.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 0)
        server.bind((self.host, self.port))
        server.listen(5)
        print(f"[MOCK] Robot server listening on {self.host}:{self.port}")
        print(f"[MOCK] Press Ctrl+C to stop")

        try:
            while self._running:
                conn, addr = server.accept()
                print(f"[MOCK] Client connected: {addr}")
                t = threading.Thread(target=self._handle_client,
                                     args=(conn, addr), daemon=True)
                t.start()
        except KeyboardInterrupt:
            print("\n[MOCK] Shutting down...")
        finally:
            server.close()
            print("[MOCK] Server stopped")

    def _handle_client(self, conn: socket.socket, addr):
        conn.settimeout(30.0)
        buf = b""
        try:
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk
                # Try to parse complete JSON objects separated by newlines
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        frame = json.loads(line.decode("utf-8"))
                    except json.JSONDecodeError:
                        # Try parsing without newline delimiter
                        try:
                            frame = json.loads(buf.decode("utf-8"))
                            buf = b""
                        except json.JSONDecodeError:
                            continue

                    reply = self.robot.handle_frame(frame)
                    if reply:
                        resp = (json.dumps(reply, ensure_ascii=False) + "\n").encode("utf-8")
                        conn.sendall(resp)

                    # Log
                    rtype = frame.get("reqType", frame.get("cmdType", "?"))
                    print(f"  [{rtype}] → {reply.get('cmdReply', reply.get('queryData', 'ok'))}")
        except Exception as e:
            print(f"[MOCK] Client error: {e}")
        finally:
            conn.close()
            print(f"[MOCK] Client disconnected: {addr}")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Mock Borunte HC1 Robot Server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9760)
    args = parser.parse_args()

    server = MockRobotServer(args.host, args.port)
    server.start()
