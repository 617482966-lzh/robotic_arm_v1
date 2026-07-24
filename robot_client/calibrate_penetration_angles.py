# -*- coding: utf-8 -*-
"""实机标定不同Ry角度下的末端向量贯入速度。

默认在0~90°范围内每隔1°，沿贯入试验按钮使用的当前末端向量
``(sin(Ry), 0, -cos(Ry))``正向100 mm并原路返回。
程序仅供现场标定使用；任何模式、报警、距离或回位异常都会终止序列。
"""

from __future__ import annotations

import csv
import math
import sys
import time
from datetime import datetime
from pathlib import Path

from robot_client.robot_control import BorunteRobot


HOST = "192.168.1.4"
PORT = 9760
REQUESTED_SPEED_MM_S = 10.0
DISTANCE_MM = 100.0
GLOBAL_SPEED_PERCENT = 5.0
START_ANGLE = 0
END_ANGLE = 90
ANGLE_STEP = 1


def log(message: str) -> None:
    print(f"{datetime.now():%H:%M:%S} {message}", flush=True)


def require_safe(state: dict) -> None:
    if state["curMode"] not in (2, 7):
        raise RuntimeError(f"机械臂不在自动模式: {state}")
    if state["curAlarm"] != 0:
        raise RuntimeError(f"机械臂存在报警: {state}")


def wait_motion(robot: BorunteRobot, timeout: float):
    deadline = time.perf_counter() + timeout
    moving_at = None
    samples = []
    while time.perf_counter() < deadline:
        timestamp = time.perf_counter()
        state = robot.read_status_pose()
        require_safe(state)
        samples.append((timestamp, state))
        if state["isMoving"] and moving_at is None:
            moving_at = timestamp
        if moving_at is not None and not state["isMoving"]:
            return moving_at, timestamp, state, samples
        time.sleep(0.025)
    raise TimeoutError(f"等待运动完成超时（{timeout:.1f}s）")


def set_ry(robot: BorunteRobot, angle_deg: float) -> dict:
    state = robot.read_status_pose()
    require_safe(state)
    instruction = robot._build_pose_line_inst(
        state["x"], state["y"], state["z"],
        state["u"], float(angle_deg), state["w"],
        speed_pct=100, ck_status=0x10, one_shot=True, smooth=9,
    )
    reply = robot.add_rcc(
        [robot._disable_physical_speed_instruction(), instruction],
        empty=True,
        show=False,
    )
    _, _, final, _ = wait_motion(robot, timeout=45.0)
    if abs(final["v"] - float(angle_deg)) > 0.15:
        raise RuntimeError(
            f"Ry目标偏差过大: target={angle_deg}, actual={final['v']}"
        )
    return {"reply": reply, "state": final}


def _steady_speed(samples, start, direction) -> float:
    moving = [(stamp, state) for stamp, state in samples if state["isMoving"]]
    lower = int(len(moving) * 0.2)
    upper = int(len(moving) * 0.8)
    points = moving[lower:max(upper, lower + 2)]
    timestamps = [item[0] for item in points]
    projections = [
        sum(
            (item[1][key] - start[index]) * direction[index]
            for index, key in enumerate(("x", "y", "z"))
        )
        for item in points
    ]
    mean_t = sum(timestamps) / len(timestamps)
    mean_p = sum(projections) / len(projections)
    denominator = sum((value - mean_t) ** 2 for value in timestamps)
    if denominator <= 0:
        raise RuntimeError("稳态速度采样点时间跨度无效")
    return abs(
        sum(
            (stamp - mean_t) * (position - mean_p)
            for stamp, position in zip(timestamps, projections)
        )
        / denominator
    )


def vector_segment(robot: BorunteRobot, signed_distance: float) -> dict:
    initial = robot.read_status_pose()
    require_safe(initial)
    direction = robot.penetration_axis_in_world(initial["v"])
    start = tuple(initial[key] for key in ("x", "y", "z"))
    target_xyz = tuple(
        start[index] + signed_distance * direction[index] for index in range(3)
    )
    target = (
        *target_xyz, initial["u"], initial["v"], initial["w"],
    )
    move = robot._build_pose_line_inst(
        *target, speed_pct=int(REQUESTED_SPEED_MM_S),
        ck_status=0x07, one_shot=True, smooth=9,
    )
    move.pop("speed", None)
    reply = robot.add_rcc(
        [robot._physical_speed_instruction(REQUESTED_SPEED_MM_S), move],
        empty=True,
        show=False,
    )
    began, ended, final, samples = wait_motion(
        robot, abs(signed_distance) / REQUESTED_SPEED_MM_S + 8.0
    )
    end = tuple(final[key] for key in ("x", "y", "z"))
    actual_distance = math.dist(start, end)
    target_error = math.dist(target_xyz, end)
    duration = ended - began
    if target_error > 0.25 or abs(actual_distance - abs(signed_distance)) > 0.3:
        raise RuntimeError(
            f"向量运动偏差过大: distance={actual_distance:.4f}, "
            f"target_error={target_error:.4f}"
        )
    return {
        "reply": reply,
        "start": start,
        "end": end,
        "actual_distance": actual_distance,
        "target_error": target_error,
        "duration": duration,
        "average_speed": actual_distance / duration,
        "steady_speed": _steady_speed(samples, start, direction),
    }


def main() -> int:
    output_dir = Path(__file__).resolve().parents[1] / "data"
    output_dir.mkdir(exist_ok=True)
    output_path = output_dir / f"penetration_angle_speed_{datetime.now():%Y%m%d_%H%M%S}.csv"
    robot = BorunteRobot(HOST, PORT)
    original_ry = None
    completed = False
    try:
        robot.connect(timeout=3.0)
        initial = robot.read_status_pose()
        require_safe(initial)
        if initial["isMoving"]:
            raise RuntimeError("机械臂正在运动，不能启动标定")
        original_ry = initial["v"]
        robot.set_global_speed(GLOBAL_SPEED_PERCENT)
        log(
            f"START output={output_path} original_Ry={original_ry:.6f} "
            f"speed={REQUESTED_SPEED_MM_S} distance={DISTANCE_MM}"
        )
        with output_path.open("w", newline="", encoding="utf-8-sig") as stream:
            columns = [
                "Ry/deg", "方向", "设定速度/mm/s", "实际距离/mm",
                "运动时间/s", "全程平均速度/mm/s", "稳态速度/mm/s",
                "目标误差/mm", "起点X/mm", "起点Y/mm", "起点Z/mm",
                "终点X/mm", "终点Y/mm", "终点Z/mm",
            ]
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            for angle in range(START_ANGLE, END_ANGLE + 1, ANGLE_STEP):
                orientation = set_ry(robot, angle)["state"]
                time.sleep(0.2)
                outward = vector_segment(robot, DISTANCE_MM)
                time.sleep(0.3)
                backward = vector_segment(robot, -DISTANCE_MM)
                return_error = math.dist(outward["start"], backward["end"])
                if return_error > 0.4:
                    raise RuntimeError(
                        f"Ry={angle}°往返回位误差过大: {return_error:.4f} mm"
                    )
                for label, result in (("正向", outward), ("反向", backward)):
                    writer.writerow({
                        "Ry/deg": angle,
                        "方向": label,
                        "设定速度/mm/s": REQUESTED_SPEED_MM_S,
                        "实际距离/mm": f"{result['actual_distance']:.6f}",
                        "运动时间/s": f"{result['duration']:.6f}",
                        "全程平均速度/mm/s": f"{result['average_speed']:.6f}",
                        "稳态速度/mm/s": f"{result['steady_speed']:.6f}",
                        "目标误差/mm": f"{result['target_error']:.6f}",
                        "起点X/mm": f"{result['start'][0]:.6f}",
                        "起点Y/mm": f"{result['start'][1]:.6f}",
                        "起点Z/mm": f"{result['start'][2]:.6f}",
                        "终点X/mm": f"{result['end'][0]:.6f}",
                        "终点Y/mm": f"{result['end'][1]:.6f}",
                        "终点Z/mm": f"{result['end'][2]:.6f}",
                    })
                stream.flush()
                log(
                    f"ANGLE {angle:02d}/90 "
                    f"out={outward['steady_speed']:.4f} "
                    f"back={backward['steady_speed']:.4f} mm/s "
                    f"return_error={return_error:.4f} mm"
                )
        completed = True
        return 0
    except Exception as exc:
        log(f"FAILED {type(exc).__name__}: {exc}")
        try:
            robot.safe_stop_and_clear()
        except Exception as stop_exc:
            log(f"STOP_FAILED {stop_exc}")
        return 1
    finally:
        if completed and original_ry is not None:
            try:
                set_ry(robot, original_ry)
                log(f"RESTORED Ry={original_ry:.6f}")
            except Exception as exc:
                log(f"RESTORE_FAILED {exc}")
        robot.disconnect()
        log("DONE")


if __name__ == "__main__":
    sys.exit(main())
