# -*- coding: utf-8 -*-
"""
HC1 机器人交互式控制程序
在命令行中输入命令控制机器人，输入 help 查看所有命令，输入 quit 退出。
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from json_write import HC1JsonRobot

HELP = """
====================================================
  HC1 机器人远程控制 - 交互式命令行
====================================================
连接管理:
  connect [host] [port]    连接机器人 (默认 192.168.1.4:9760)
  disconnect                断开连接
  status                    显示连接状态

速度说明: 运动命令中的 [pct] 参数是速度百分比 (0-100)，对应物理速度(action=51)
         的比例。例如物理速度 100mm/s + 百分比 50 = 实际 50mm/s
读取:
  joints                    读取当前关节角 (J1~J6, deg)
  pose                      读取末端世界坐标 (X,Y,Z,U,V,W)
  state                     读取完整状态 (模式/运动/报警 + 坐标)
运动 - 自由路径 (关节角, action=4):
  free j1 j2 j3 j4 j5 j6 [pct]     移动到指定关节角 (pct: 速度百分比 0-100)
  free_by dj1 dj2 dj3 dj4 dj5 dj6 [pct]  在当前关节角上增减 (pct: 速度百分比)
运动 - 姿势直线 (世界坐标, action=10):
  pose_move x y z u v w [pct]    移动到指定世界坐标 (pct: 速度百分比)
  pose_by dx dy dz du dv dw [pct]  在当前坐标上增减 (pct: 速度百分比)
速度控制:
  speed <mm/s>              物理速度, 单位毫米/秒 (action=51, 一直执行)
                            实际运动速度 = 物理速度 x 百分比 x 全局倍率
  speed_off                 禁用物理速度，回退到全局速度
  global <pct>              全局速度倍率 (0-100)，影响所有运动的参考速度
控制:
  stop                      急停
  clear                     清除所有报警
  home                      回零
  enable on|off             使能/禁用
  wait [timeout]           等待运动完成 (默认30s)
其他:
  help                      显示此帮助
  quit / exit               退出程序
====================================================
"""

def parse_floats(args, count):
    vals = []
    for a in args[:count]:
        try:
            vals.append(float(a))
        except (ValueError, IndexError):
            vals.append(0.0)
    return vals

def parse_speed(args, idx, default=30.0):
    if idx < len(args):
        try:
            return float(args[idx])
        except ValueError:
            pass
    return default

class RobotShell:
    def __init__(self):
        self.robot = HC1JsonRobot()
        self.running = True
        self.commands = {
            "connect":    self.cmd_connect,
            "disconnect": self.cmd_disconnect,
            "status":     self.cmd_status,
            "joints":     self.cmd_joints,
            "pose":       self.cmd_pose,
            "state":      self.cmd_state,
            "free":       self.cmd_free,
            "free_by":    self.cmd_free_by,
            "pose_move":  self.cmd_pose_move,
            "pose_by":    self.cmd_pose_by,
            "speed":      self.cmd_speed,
            "speed_off":  self.cmd_speed_off,
            "global":     self.cmd_global,
            "stop":       self.cmd_stop,
            "clear":      self.cmd_clear,
            "home":       self.cmd_home,
            "enable":     self.cmd_enable,
            "wait":       self.cmd_wait,
            "help":       self.cmd_help,
            "quit":       self.cmd_quit,
            "exit":       self.cmd_quit,
        }

    def require_connected(self):
        if not self.robot.is_connected():
            print("未连接，请先 connect")
            return False
        return True

    def run(self):
        print(HELP)
        self._auto_connect()
        while self.running:
            try:
                line = input(">>> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            args = line.split()
            cmd = args[0].lower()
            if cmd in self.commands:
                try:
                    self.commands[cmd](args[1:])
                except Exception as e:
                    print(f"[错误] {e}")
            else:
                print(f"未知命令: {cmd}，输入 help 查看帮助")

    def _auto_connect(self):
        host = self.robot.host
        port = self.robot.port
        try:
            self.robot.connect()
            print(f"[自检] 已连接 {host}:{port}")
            self._self_check()
        except ConnectionRefusedError:
            print(f"[自检] 连接被拒绝: {host}:{port}")
            print(f"       请确认: 1) 机器人IP正确  2) 机器人已开机  3) 端口 {port} 已开放")
        except OSError as e:
            print(f"[自检] 网络错误: {e}")
            print(f"       请确认: 1) IP地址可达  2) 防火墙未拦截  3) 网线已连接")
        except Exception as e:
            print(f"[自检] 连接失败: {e}")

    def _self_check(self):
        import time
        time.sleep(0.3)
        ok = True
        try:
            s = self.robot.read_status_pose()
            if s:
                mode_map = {0:"无", 1:"手动", 2:"自动", 3:"停止", 7:"自动运行中", 8:"单步", 9:"单循环"}
                print(f"[自检] 状态: 模式={mode_map.get(s['curMode'], s['curMode'])}  运动={'是' if s['isMoving'] else '否'}  报警={s['curAlarm']}")
                if s['curAlarm'] != 0:
                    print(f"[自检] 注意: 当前存在报警号 {s['curAlarm']}，请用 clear 命令清除")
                    ok = False
            else:
                print("[自检] 无法读取状态，请检查通讯协议是否匹配")
                ok = False
        except Exception as e:
            print(f"[自检] 状态查询异常: {e}")
            ok = False
        try:
            j = self.robot.read_joints()
            if j:
                print(f"[自检] 关节: J1={j['j1']:.4f} J2={j['j2']:.4f} J3={j['j3']:.4f} J4={j['j4']:.4f} J5={j['j5']:.4f} J6={j['j6']:.4f}")
            else:
                print("[自检] 无法读取关节角")
                ok = False
        except Exception as e:
            print(f"[自检] 关节查询异常: {e}")
            ok = False
        try:
            p = self.robot.read_world_pose()
            if p:
                print(f"[自检] 坐标: X={p['x']:.3f} Y={p['y']:.3f} Z={p['z']:.3f} U={p['u']:.3f} V={p['v']:.3f} W={p['w']:.3f}")
            else:
                print("[自检] 无法读取世界坐标")
                ok = False
        except Exception as e:
            print(f"[自检] 坐标查询异常: {e}")
            ok = False
        if ok:
            print("[自检] 全部通过，可以输入命令")
        else:
            print("[自检] 部分检查未通过，请根据上述提示排查")

    def cmd_connect(self, args):
        host = args[0] if len(args) > 0 else "192.168.1.4"
        port = int(args[1]) if len(args) > 1 else 9760
        self.robot.host = host
        self.robot.port = port
        self.robot.connect()
        print(f"已连接 {host}:{port}")

    def cmd_disconnect(self, args):
        self.robot.disconnect()

    def cmd_status(self, args):
        if self.robot.is_connected():
            print(f"已连接 {self.robot.host}:{self.robot.port}")
        else:
            print("未连接")

    def cmd_joints(self, args):
        if not self.require_connected(): return
        j = self.robot.read_joints()
        if j:
            print(f"J1={j['j1']:.4f}  J2={j['j2']:.4f}  J3={j['j3']:.4f}")
            print(f"J4={j['j4']:.4f}  J5={j['j5']:.4f}  J6={j['j6']:.4f}")
        else:
            print("读取失败")

    def cmd_pose(self, args):
        if not self.require_connected(): return
        p = self.robot.read_world_pose()
        if p:
            print(f"X={p['x']:.3f}  Y={p['y']:.3f}  Z={p['z']:.3f}")
            print(f"U={p['u']:.3f}  V={p['v']:.3f}  W={p['w']:.3f}")
        else:
            print("读取失败")

    def cmd_state(self, args):
        if not self.require_connected(): return
        s = self.robot.read_status_pose()
        if s:
            mode_map = {0:"无", 1:"手动", 2:"自动", 3:"停止", 7:"自动运行中", 8:"单步", 9:"单循环"}
            print(f"模式={mode_map.get(s['curMode'], s['curMode'])}  运动={'是' if s['isMoving'] else '否'}  报警={s['curAlarm']}")
            print(f"X={s['x']:.3f}  Y={s['y']:.3f}  Z={s['z']:.3f}  U={s['u']:.3f}  V={s['v']:.3f}  W={s['w']:.3f}")
        else:
            print("读取失败")

    def cmd_free(self, args):
        if not self.require_connected(): return
        vals = parse_floats(args, 6)
        speed = parse_speed(args, 6, 30.0)
        self.robot.move_free_path(*vals, speed_pct=speed)
        print(f"自由路径: J1={vals[0]:.2f} J2={vals[1]:.2f} J3={vals[2]:.2f} J4={vals[3]:.2f} J5={vals[4]:.2f} J6={vals[5]:.2f} 速度={speed}%")

    def cmd_free_by(self, args):
        if not self.require_connected(): return
        vals = parse_floats(args, 6)
        speed = parse_speed(args, 6, 30.0)
        self.robot.move_free_path_by(*vals, speed_pct=speed)
        print(f"自由路径增量: dJ1={vals[0]:.2f} dJ2={vals[1]:.2f} dJ3={vals[2]:.2f} dJ4={vals[3]:.2f} dJ5={vals[4]:.2f} dJ6={vals[5]:.2f} 速度={speed}%")

    def cmd_pose_move(self, args):
        if not self.require_connected(): return
        vals = parse_floats(args, 6)
        speed = parse_speed(args, 6, 30.0)
        self.robot.move_pose_line(*vals, speed_pct=speed)
        print(f"姿势直线: X={vals[0]:.2f} Y={vals[1]:.2f} Z={vals[2]:.2f} U={vals[3]:.2f} V={vals[4]:.2f} W={vals[5]:.2f} 速度={speed}%")

    def cmd_pose_by(self, args):
        if not self.require_connected(): return
        vals = parse_floats(args, 6)
        speed = parse_speed(args, 6, 30.0)
        self.robot.move_pose_line_by(*vals, speed_pct=speed)
        print(f"姿势直线增量: dX={vals[0]:.2f} dY={vals[1]:.2f} dZ={vals[2]:.2f} dU={vals[3]:.2f} dV={vals[4]:.2f} dW={vals[5]:.2f} 速度={speed}%")

    def cmd_speed(self, args):
        if not self.require_connected(): return
        v = float(args[0]) if args else 100.0
        self.robot.set_physical_speed(v)
        print(f"物理速度: {int(v)} mm/s (一直执行)")

    def cmd_speed_off(self, args):
        if not self.require_connected(): return
        self.robot.disable_physical_speed()
        print("物理速度已禁用")

    def cmd_global(self, args):
        if not self.require_connected(): return
        v = float(args[0]) if args else 50.0
        self.robot.set_global_speed(v)
        print(f"全局速度: {v}%")

    def cmd_stop(self, args):
        if not self.require_connected(): return
        self.robot.emergency_stop()
        print("急停已发送")

    def cmd_clear(self, args):
        if not self.require_connected(): return
        self.robot.clear_alarm()
        print("报警已清除")

    def cmd_home(self, args):
        if not self.require_connected(): return
        self.robot.home()
        print("回零指令已发送")

    def cmd_enable(self, args):
        if not self.require_connected(): return
        state = args[0].lower() if args else "on"
        if state in ("on", "1", "true"):
            self.robot.enable()
            print("已使能")
        else:
            self.robot.disable()
            print("已禁用")

    def cmd_wait(self, args):
        if not self.require_connected(): return
        t = float(args[0]) if args else 30.0
        print(f"等待运动完成 (超时 {t}s)...")
        ok = self.robot.wait_for_idle(timeout=t)
        print("运动完成" if ok else "等待超时")

    def cmd_help(self, args):
        print(HELP)

    def cmd_quit(self, args):
        print("退出...")
        if self.robot.is_connected():
            self.robot.disconnect()
        self.running = False


if __name__ == "__main__":
    RobotShell().run()
