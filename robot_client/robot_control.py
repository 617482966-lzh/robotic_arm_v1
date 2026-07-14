# -*- coding: utf-8 -*-
"""机械臂业务客户端。

底层 JSON/TCP 格式直接复用 ``demo/json_write.py``，本模块只提供界面需要的
六轴读取、世界坐标运动、关节运动、使能、回零和急停接口。
"""

from __future__ import annotations

from demo.json_write import HC1JsonRobot

GLOBAL_SPEED_PERCENT = 5.0
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
        speed = self.joint_speed_to_action4_percent(speed_deg_s)
        move = self._build_free_path_inst(
            *values, speed_pct=speed, ck_status=0x3F, one_shot=True
        )
        return self.add_rcc(
            [self._disable_physical_speed_instruction(), move],
            empty=True,
            show=False,
        )

    def move_joints_increment(self, increments, speed_deg_s: float):
        joints = self.read_joints()
        if joints is None:
            raise RobotError("无法读取当前关节角")
        current = tuple(joints[f"j{i}"] for i in range(1, 7))
        target = tuple(a + float(b) for a, b in zip(current, increments))
        return self.move_joints_absolute(target, speed_deg_s)

    def home(self, speed_mm_s: float = 10.0):
        """按需求以六关节零位作为回零目标。"""
        return self.move_joints_absolute((0, 0, 0, 0, 0, 0), speed_mm_s)

    def emergency_stop(self):
        return super().emergency_stop()

    # 兼容旧调用名称。
    stop = emergency_stop
