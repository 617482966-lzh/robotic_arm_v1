# -*- coding: utf-8 -*-
"""机械臂业务客户端。

底层 JSON/TCP 实现在 ``robot_client.hc1_json`` 中；``demo`` 目录仅作为厂商
示例保留，不参与正式程序运行。
"""

from __future__ import annotations

import math

from robot_client.hc1_json import HC1JsonRobot

GLOBAL_SPEED_PERCENT = 5.0
DEFAULT_INTERPOLATION_FREQUENCY_HZ = 5.0
MAX_INTERPOLATION_SEGMENTS = 10
MAX_CALIBRATED_JOINT_SPEED_DEG_S = 10.6374256987
J1_SPEED_CALIBRATION = (
    (1.1517574354, 10),
    (2.3315252973, 20),
    (3.4269934617, 30),
    (5.5855354942, 50),
    (7.6638072551, 70),
    (10.6374256987, 100),
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
        pose = self.read_world_pose()
        joints = self.read_joints()
        if pose is None or joints is None:
            raise RobotError("读取机械臂实时位姿失败")
        return (
            (pose["x"], pose["y"], pose["z"], pose["u"], pose["v"], pose["w"]),
            tuple(joints[f"j{i}"] for i in range(1, 7)),
        )

    def configure_calibrated_speed_control(self):
        """固定使用标定时的5%全局速度。"""
        reply = self.set_global_speed(GLOBAL_SPEED_PERCENT)
        if not reply:
            raise RobotError("设置机械臂全局速度5%失败")
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
        pose = self.read_world_pose()
        if pose is None:
            raise RobotError("无法读取当前世界坐标")
        current = (pose["x"], pose["y"], pose["z"], pose["u"], pose["v"], pose["w"])
        target = tuple(a + float(b) for a, b in zip(current, increments))
        return self.move_world_absolute(target, speed_mm_s)

    @staticmethod
    def tool_x_axis_in_world(rx_deg: float, ry_deg: float, rz_deg: float):
        """返回本机械臂姿态在世界XZ平面的前向单位向量。

        方向只由Ry决定，约定X为正向、正Ry对应Z负向；Rx/Rz仅保持姿态。
        """
        ry = math.radians(float(ry_deg))
        vector = (
            math.cos(ry),
            0.0,
            -math.sin(ry),
        )
        norm = math.sqrt(sum(component * component for component in vector))
        if norm <= 1e-12:
            raise RobotError("无法根据当前姿态计算末端方向")
        return tuple(component / norm for component in vector)

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
    def joint_speed_to_action4_percent(speed_deg_s: float) -> int:
        """按J1实测表将期望deg/s反插值成action4.speed。"""
        desired = float(speed_deg_s)
        if not 1 <= desired <= MAX_CALIBRATED_JOINT_SPEED_DEG_S:
            raise ValueError(
                f"关节速度必须在1~{MAX_CALIBRATED_JOINT_SPEED_DEG_S:.2f} deg/s范围内"
            )
        points = J1_SPEED_CALIBRATION
        if desired <= points[0][0]:
            return max(1, round(points[0][1] * desired / points[0][0]))
        for (v0, p0), (v1, p1) in zip(points, points[1:]):
            if desired <= v1:
                raw = p0 + (desired - v0) * (p1 - p0) / (v1 - v0)
                return max(1, min(100, round(raw)))
        return 100

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
        speed = self.joint_speed_to_action4_percent(speed_deg_s)
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
        speed = self.joint_speed_to_action4_percent(speed_deg_s)
        move = self._build_free_path_inst(
            *target, speed_pct=speed, ck_status=ck_status, one_shot=True
        )
        return self.add_rcc(
            [self._disable_physical_speed_instruction(), move],
            empty=True,
            show=False,
        )

    def home(self, speed_mm_s: float = 10.0):
        """按需求以六关节零位作为回零目标。"""
        return self.move_joints_absolute((0, 0, 0, 0, 0, 0), speed_mm_s)

    def emergency_stop(self):
        """停止当前动作并清除相关报警，避免actionStop切入报警状态。"""
        return self.stop_button()

    # 兼容旧调用名称。
    stop = emergency_stop
