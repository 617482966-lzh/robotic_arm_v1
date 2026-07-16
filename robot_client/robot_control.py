# -*- coding: utf-8 -*-
"""机械臂业务客户端。

底层 JSON/TCP 实现在 ``robot_client.hc1_json`` 中；``demo`` 目录仅作为厂商
示例保留，不参与正式程序运行。
"""

from __future__ import annotations

import math
import time

from robot_client.hc1_json import HC1JsonRobot

GLOBAL_SPEED_PERCENT = 10.0
DEFAULT_INTERPOLATION_FREQUENCY_HZ = 5.0
MAX_INTERPOLATION_SEGMENTS = 10
MAX_REQUESTED_JOINT_SPEED_DEG_S = 20.0
# 2026-07-16 实机双向标定：(实测 deg/s, action4.speed 百分比)。
# 标定条件：全局速度10%，加/减速0.250 s，S比例30%，滤波128 ms。
JOINT_SPEED_CALIBRATIONS = (
    ((1.163669, 5), (2.378422, 10), (4.756821, 20), (7.133593, 30),
     (11.886208, 50), (16.641205, 70), (23.648726, 100)),
    ((1.307187, 5), (2.671718, 10), (5.343965, 20), (8.014062, 30),
     (13.353887, 50), (18.684826, 70), (25.741579, 100)),
    ((1.850862, 5), (3.699920, 10), (7.395409, 20), (11.092476, 30),
     (18.469153, 50), (25.140991, 70), (32.048416, 100)),
    ((1.690345, 5), (3.380226, 10), (6.756338, 20), (10.130300, 30),
     (16.889749, 50), (23.560099, 70), (31.616944, 100)),
    ((2.996666, 5), (5.991215, 10), (11.962755, 20), (17.732809, 30),
     (29.118150, 50), (36.659746, 70), (42.918165, 100)),
    ((2.947982, 5), (5.893505, 10), (11.716466, 20), (17.051939, 30),
     (27.750129, 50), (34.367871, 70), (38.711236, 100)),
)


class RobotError(RuntimeError):
    pass


class BorunteRobot(HC1JsonRobot):
    """与 V2 界面对应的 HC1 六轴机械臂客户端。"""

    def __init__(self, host: str, port: int = 9760):
        super().__init__(host, port)

    def connect(self, timeout: float = 5.0) -> bool:
        # demo 客户端固定使用 5 秒；临时覆盖便于界面快速报告连接失败。
        super().connect()
        if not self.is_connected():
            raise RobotError(f"无法连接机械臂 {self.host}:{self.port}")
        self.sock.settimeout(timeout)
        return True

    def read_realtime_state(self):
        addresses = [f"world-{i}" for i in range(6)] + [f"axis-{i}" for i in range(6)]
        reply = self.query(addresses, show=False)
        values = reply.get("queryData", ()) if isinstance(reply, dict) else ()
        if len(values) < 12:
            raise RobotError("读取机械臂实时位姿失败")
        values = tuple(float(value) for value in values[:12])
        return (
            values[:6],
            values[6:12],
        )

    def configure_calibrated_speed_control(self):
        """固定使用逐轴标定时的10%全局速度。"""
        reply = self.set_global_speed(GLOBAL_SPEED_PERCENT)
        if not reply:
            raise RobotError("设置机械臂全局速度10%失败")
        return reply

    @staticmethod
    def _require_add_rcc_ok(reply, operation: str):
        values = reply.get("cmdReply", []) if isinstance(reply, dict) else []
        if not values or str(values[-1]).lower() != "ok":
            raise RobotError(f"{operation}失败，控制器回复: {reply}")
        return reply

    @staticmethod
    def _physical_speed_instruction(speed_mm_s: float) -> dict:
        if speed_mm_s <= 0:
            raise ValueError("世界坐标运动速度必须大于 0 mm/s")
        return {
            "oneshot": "0", "action": "51", "isUse": "1",
            "speed": str(int(round(speed_mm_s))),
        }

    @staticmethod
    def _disable_physical_speed_instruction() -> dict:
        return {
            "oneshot": "0", "action": "51", "isUse": "0", "speed": "0",
        }

    def move_world_absolute(self, values, speed_mm_s: float):
        values = tuple(float(v) for v in values)
        if len(values) != 6:
            raise ValueError("世界坐标目标必须包含 6 个值")
        speed = int(round(float(speed_mm_s)))
        move = self._build_pose_line_inst(
            *values, speed_pct=speed, ck_status=0x3F, one_shot=True
        )
        # action=51 已启用物理速度，action=10 不再携带 speed，避免覆盖。
        move.pop("speed", None)
        return self.add_rcc(
            [self._physical_speed_instruction(speed_mm_s), move],
            empty=True,
            show=False,
        )

    def move_world_increment(self, increments, speed_mm_s: float):
        increments = tuple(float(value) for value in increments)
        if len(increments) != 6:
            raise ValueError("世界坐标增量必须包含 X/Y/Z/Rx/Ry/Rz 六个值")
        ck_status = self.joint_mask_from_deltas(increments)
        pose = self.read_world_pose()
        if pose is None:
            raise RobotError("无法读取当前世界坐标")
        current = (pose["x"], pose["y"], pose["z"], pose["u"], pose["v"], pose["w"])
        target = tuple(a + b for a, b in zip(current, increments))
        speed = int(round(float(speed_mm_s)))
        move = self._build_pose_line_inst(
            *target, speed_pct=speed, ck_status=ck_status, one_shot=True
        )
        move.pop("speed", None)
        return self.add_rcc(
            [self._physical_speed_instruction(speed_mm_s), move],
            empty=True,
            show=False,
        )

    def move_along_tool_x(self, distance_mm: float, speed_mm_s: float):
        """沿当前末端工具X向量运动，世界姿态角保持为启动时的值。

        该接口只向控制器发送一个最终世界坐标目标，便于试验期间用
        ``stopButton`` 在力阈值或扭矩阈值到达时无报警停止。
        """
        distance = float(distance_mm)
        speed = float(speed_mm_s)
        if distance == 0:
            raise ValueError("末端向量运动距离不能为0 mm")
        if speed <= 0:
            raise ValueError("末端向量运动速度必须大于0 mm/s")
        pose = self.read_world_pose()
        if pose is None:
            raise RobotError("无法读取末端向量运动起始位姿")
        start = (pose["x"], pose["y"], pose["z"], pose["u"], pose["v"], pose["w"])
        direction = self.penetration_axis_in_world()
        target = (
            start[0] + distance * direction[0],
            start[1] + distance * direction[1],
            start[2] + distance * direction[2],
            start[3], start[4], start[5],
        )
        move = self._build_pose_line_inst(
            *target, speed_pct=int(round(speed)), ck_status=0x07,
            one_shot=True, smooth=9,
        )
        move.pop("speed", None)
        reply = self.add_rcc(
            [self._physical_speed_instruction(speed), move], empty=True, show=False
        )
        return {"reply": reply, "start": start, "target": target, "direction": direction}

    @staticmethod
    def penetration_axis_in_world():
        """贯入试验固定沿世界坐标系Z负方向运动。"""
        return 0.0, 0.0, -1.0

    @staticmethod
    def tool_x_axis_in_world(rx_deg: float, ry_deg: float, rz_deg: float):
        """标准Rz·Ry·Rx旋转下，工具坐标系+X轴的世界方向。"""
        ry = math.radians(float(ry_deg))
        rz = math.radians(float(rz_deg))
        return (
            math.cos(rz) * math.cos(ry),
            math.sin(rz) * math.cos(ry),
            -math.sin(ry),
        )

    def safe_stop_and_clear(self, timeout: float = 2.0, interval: float = 0.01):
        """用actionStop立即停止，确认静止后清空远程列表并清除报警。

        实机确认stopButton/actionPause不能中止正在执行的远程action10；
        actionStop会进入模式3，下一次试验前需在示教器切回自动模式。
        """
        deadline = time.monotonic() + float(timeout)
        stop_reply = self.command(["actionStop"], show=False)
        state = None
        while time.monotonic() < deadline:
            state = self.read_status_pose()
            if state is not None and not state["isMoving"]:
                break
            time.sleep(float(interval))
        else:
            raise RobotError(
                f"actionStop在{timeout:.1f}s内未能确认机械臂停止，回复: {stop_reply}"
            )
        clear_reply = self.add_rcc([], empty=True, show=False)
        button_reply = self.stop_button()
        alarm_reply = self.clear_alarm()
        final_state = self.read_status_pose()
        return {
            "actionStop": stop_reply, "clear": clear_reply,
            "stopButton": button_reply, "clearAlarm": alarm_reply,
            "stopped_state": state, "final_state": final_state,
        }

    @staticmethod
    def _canonical_angle(angle_deg: float) -> float:
        """统一为[-180, 180)，避免+180/-180等价角触发长路径旋转。"""
        value = (float(angle_deg) + 180.0) % 360.0 - 180.0
        return 0.0 if abs(value) < 1e-12 else value

    def move_tool_vector_interpolated(
        self,
        distance_mm: float,
        speed_mm_s: float,
        frequency_hz: float = DEFAULT_INTERPOLATION_FREQUENCY_HZ,
    ):
        """沿当前末端局部+X方向进行定姿态直线插补。

        ``distance_mm`` 可正可负，当前现场调试范围限制为±10 mm。所有action10
        插补点保持起始Rx/Ry/Rz不变，并一次性写入控制器队列，避免受TCP往返
        延迟影响插补节拍。
        """
        distance = float(distance_mm)
        speed = float(speed_mm_s)
        frequency = float(frequency_hz)
        if not 0 < abs(distance) <= 10.0:
            raise ValueError("直线插补距离必须在-10~10 mm内且不能为0")
        if speed < 1.0:
            raise ValueError("直线插补速度不能低于1 mm/s")
        if not 2.0 <= frequency <= 50.0:
            raise ValueError("直线插补频率必须在2~50 Hz范围内")

        pose = self.read_status_pose()
        if pose is None:
            raise RobotError("无法读取直线插补起始位姿")
        if pose["curAlarm"] != 0:
            raise RobotError(f"机械臂存在报警 {pose['curAlarm']}，禁止直线插补")
        if pose["curMode"] not in (2, 7):
            raise RobotError(f"机械臂不在自动模式，当前模式={pose['curMode']}")
        if pose["isMoving"]:
            raise RobotError("机械臂正在运动，禁止启动新的直线插补")
        start = (
            pose["x"], pose["y"], pose["z"],
            pose["u"], pose["v"], pose["w"],
        )
        command_angles = tuple(self._canonical_angle(value) for value in start[3:])
        direction = self.tool_x_axis_in_world(start[3], start[4], start[5])
        duration = abs(distance) / speed
        requested_segments = max(1, int(math.ceil(duration * frequency)))
        segment_count = min(MAX_INTERPOLATION_SEGMENTS, requested_segments)
        instructions = [self._physical_speed_instruction(speed)]
        for index in range(1, segment_count + 1):
            travelled = distance * index / segment_count
            target = (
                start[0] + direction[0] * travelled,
                start[1] + direction[1] * travelled,
                start[2] + direction[2] * travelled,
                command_angles[0], command_angles[1], command_angles[2],
            )
            move = self._build_pose_line_inst(
                *target, speed_pct=int(round(speed)), ck_status=0x07,
                one_shot=True, smooth=9,
            )
            move.pop("speed", None)
            instructions.append(move)
        reply = self.add_rcc(instructions, empty=True, show=False)
        # 当前HC1固件对“action51 + action10”组合会回复AddRCC/err，
        # 但指令实际正常执行；实机必须结合isMoving和最终位姿判断。
        if not isinstance(reply, dict):
            raise RobotError(f"action10直线插补无有效回复: {reply}")
        return {
            "reply": reply,
            "start": start,
            "direction": direction,
            "distance_mm": distance,
            "speed_mm_s": speed,
            "frequency_hz": frequency,
            "effective_frequency_hz": segment_count / duration,
            "segments": segment_count,
            "segments_limited": segment_count < requested_segments,
        }

    def move_tool_vector_curve(self, distance_mm: float, speed_percent: float = 10.0):
        """用“action10入口点 + action17”生成姿势曲线，供对比试验。"""
        distance = float(distance_mm)
        speed = float(speed_percent)
        if not 0 < abs(distance) <= 10.0:
            raise ValueError("姿势曲线距离必须在-10~10 mm内且不能为0")
        if not 1.0 <= speed <= 100.0:
            raise ValueError("action17速度百分比必须在1~100范围内")
        state = self.read_status_pose()
        if state is None:
            raise RobotError("无法读取姿势曲线起始位姿")
        if state["curAlarm"] != 0 or state["curMode"] not in (2, 7) or state["isMoving"]:
            raise RobotError(
                f"姿势曲线启动条件不满足: mode={state['curMode']}, "
                f"moving={state['isMoving']}, alarm={state['curAlarm']}"
            )
        start = (
            state["x"], state["y"], state["z"],
            *(self._canonical_angle(state[key]) for key in ("u", "v", "w")),
        )
        direction = self.tool_x_axis_in_world(*start[3:])
        target = (
            start[0] + direction[0] * distance,
            start[1] + direction[1] * distance,
            start[2] + direction[2] * distance,
            start[3], start[4], start[5],
        )
        curve = self._build_pose_curve_inst(
            start, target, speed_pct=int(round(speed)), ck_status=0x07, smooth=9,
        )
        entry = self._build_pose_line_inst(
            *start, speed_pct=int(round(speed)), ck_status=0x07,
            one_shot=True, smooth=9,
        )
        reply = self.add_rcc(
            [entry, curve],
            empty=True,
            show=False,
        )
        self._require_add_rcc_ok(reply, "action17姿势曲线")
        return {"reply": reply, "start": start, "target": target, "direction": direction}

    @staticmethod
    def joint_speed_to_action4_percent(speed_deg_s: float, ck_status: int) -> float:
        """按参与运动的关节实测表反插值成安全的action4.speed。"""
        desired = float(speed_deg_s)
        if not 1 <= desired <= MAX_REQUESTED_JOINT_SPEED_DEG_S:
            raise ValueError(
                f"关节速度必须在1~{MAX_REQUESTED_JOINT_SPEED_DEG_S:.0f} deg/s范围内"
            )
        active_axes = [axis for axis in range(6) if ck_status & (1 << axis)]
        if not active_axes:
            raise ValueError("没有检测到需要运动的关节")

        def inverse(points):
            if desired <= points[0][0]:
                return points[0][1] * desired / points[0][0]
            for (v0, p0), (v1, p1) in zip(points, points[1:]):
                if desired <= v1:
                    return p0 + (desired - v0) * (p1 - p0) / (v1 - v0)
            return 100.0

        # action4只有一个公共speed。多轴同时运动时取各轴所需百分比的最小值，
        # 保证任何活动轴都不会超过用户输入速度；单轴运动则使用该轴专属曲线。
        raw = min(inverse(JOINT_SPEED_CALIBRATIONS[axis]) for axis in active_axes)
        return max(0.1, min(100.0, round(raw, 1)))

    def move_joints_absolute(self, values, speed_deg_s: float):
        values = tuple(float(v) for v in values)
        if len(values) != 6:
            raise ValueError("关节目标必须包含 6 个值")
        joints = self.read_joints()
        if joints is None:
            raise RobotError("无法读取当前关节角，不能自动生成ckStatus")
        current = tuple(float(joints[f"j{i}"]) for i in range(1, 7))
        ck_status = self.joint_mask_from_deltas(
            tuple(target - actual for target, actual in zip(values, current))
        )
        speed = self.joint_speed_to_action4_percent(speed_deg_s, ck_status)
        move = self._build_free_path_inst(
            *values, speed_pct=speed, ck_status=ck_status, one_shot=True
        )
        return self.add_rcc(
            [self._disable_physical_speed_instruction(), move],
            empty=True,
            show=False,
        )

    def move_joints_increment(self, increments, speed_deg_s: float):
        increments = tuple(float(value) for value in increments)
        if len(increments) != 6:
            raise ValueError("关节增量必须包含 6 个值")
        joints = self.read_joints()
        if joints is None:
            raise RobotError("无法读取当前关节角")
        current = tuple(joints[f"j{i}"] for i in range(1, 7))
        target = tuple(a + b for a, b in zip(current, increments))
        ck_status = self.joint_mask_from_deltas(increments)
        speed = self.joint_speed_to_action4_percent(speed_deg_s, ck_status)
        move = self._build_free_path_inst(
            *target, speed_pct=speed, ck_status=ck_status, one_shot=True
        )
        return self.add_rcc(
            [self._disable_physical_speed_instruction(), move],
            empty=True,
            show=False,
        )

    def home(self, speed_deg_s: float = 10.0):
        """按需求以六关节零位作为回零目标。"""
        return self.move_joints_absolute((0, 0, 0, 0, 0, 0), speed_deg_s)

    def emergency_stop(self):
        """停止当前动作并清除相关报警，避免actionStop切入报警状态。"""
        return self.stop_button()

    # 兼容旧调用名称。
    stop = emergency_stop
