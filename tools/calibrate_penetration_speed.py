"""按末端Ry角度标定世界Z方向贯入速度。

这是现场只读测量加运动测试工具。每个角度先沿世界-Z运动指定距离，再沿
世界+Z返回；姿态调整只启用Ry轴。发生模式变化、报警或运动超时时立即停止。
"""

from __future__ import annotations

import argparse
import json
import math
import time

from robot_client.robot_control import BorunteRobot


def wait_motion(robot: BorunteRobot, timeout: float) -> dict:
    deadline = time.perf_counter() + timeout
    began = False
    while time.perf_counter() < deadline:
        state = robot.read_status_pose()
        if state["curAlarm"] != 0 or state["curMode"] not in (2, 7):
            raise RuntimeError(
                f"机械臂状态异常: mode={state['curMode']}, alarm={state['curAlarm']}"
            )
        began = began or bool(state["isMoving"])
        if began and not state["isMoving"]:
            return state
        time.sleep(0.02)
    state = robot.read_status_pose()
    if not began and not state["isMoving"]:
        return state
    robot.safe_stop_and_clear()
    raise TimeoutError(f"运动在{timeout:g}s内未结束，已执行保护停止")


def set_ry(robot: BorunteRobot, angle_deg: float) -> dict:
    state = robot.read_status_pose()
    if state["curAlarm"] != 0 or state["curMode"] not in (2, 7):
        raise RuntimeError(
            f"姿态调整前状态异常: mode={state['curMode']}, alarm={state['curAlarm']}"
        )
    instruction = robot._build_pose_line_inst(
        state["x"],
        state["y"],
        state["z"],
        state["u"],
        angle_deg,
        state["w"],
        speed_pct=20,
        ck_status=0x10,
        one_shot=True,
        smooth=9,
    )
    robot.add_rcc(
        [robot._disable_physical_speed_instruction(), instruction],
        empty=True,
        show=False,
    )
    final = wait_motion(robot, timeout=45.0)
    if abs(final["v"] - angle_deg) > 0.05:
        raise RuntimeError(
            f"Ry未到位: target={angle_deg:.3f}, actual={final['v']:.3f}"
        )
    time.sleep(0.1)
    return final


def measure_segment(
    robot: BorunteRobot, distance_mm: float, requested_speed_mm_s: float
) -> dict:
    start = robot.read_status_pose()
    start_xyz = (start["x"], start["y"], start["z"])
    command_time = time.perf_counter()
    robot.move_along_tool_x(distance_mm, requested_speed_mm_s)
    samples = []
    began = None
    ended = None
    deadline = command_time + 10.0
    while time.perf_counter() < deadline:
        now = time.perf_counter()
        state = robot.read_status_pose()
        if state["curAlarm"] != 0 or state["curMode"] not in (2, 7):
            raise RuntimeError(
                f"贯入期间状态异常: mode={state['curMode']}, alarm={state['curAlarm']}"
            )
        samples.append((now, state["x"], state["y"], state["z"], state["isMoving"]))
        if state["isMoving"] and began is None:
            began = now
        if began is not None and not state["isMoving"]:
            ended = now
            break
        time.sleep(0.015)
    if ended is None:
        robot.safe_stop_and_clear()
        raise TimeoutError("贯入段超时，已执行保护停止")

    final = robot.read_status_pose()
    final_xyz = (final["x"], final["y"], final["z"])
    distance = math.dist(start_xyz, final_xyz)
    duration = ended - began
    moving = [sample for sample in samples if sample[4]]
    steady_speed = None
    if len(moving) >= 6:
        central = moving[int(len(moving) * 0.2) : max(3, int(len(moving) * 0.8))]
        times = [sample[0] for sample in central]
        z_values = [sample[3] for sample in central]
        mean_t = sum(times) / len(times)
        mean_z = sum(z_values) / len(z_values)
        denominator = sum((value - mean_t) ** 2 for value in times)
        if denominator:
            steady_speed = abs(
                sum(
                    (sample_time - mean_t) * (z - mean_z)
                    for sample_time, z in zip(times, z_values)
                )
                / denominator
            )
    return {
        "command_mm": distance_mm,
        "actual_distance_mm": distance,
        "duration_s": duration,
        "average_mm_s": distance / duration,
        "central_mm_s": steady_speed,
        "dx_mm": final_xyz[0] - start_xyz[0],
        "dy_mm": final_xyz[1] - start_xyz[1],
        "dz_mm": final_xyz[2] - start_xyz[2],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.1.4")
    parser.add_argument("--port", type=int, default=9760)
    parser.add_argument("--start-angle", type=int, required=True)
    parser.add_argument("--end-angle", type=int, required=True)
    parser.add_argument("--distance", type=float, default=5.0)
    parser.add_argument("--speed", type=float, default=10.0)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    if not 0 <= args.start_angle <= args.end_angle <= 90:
        parser.error("角度范围必须满足0 <= start <= end <= 90")

    robot = BorunteRobot(args.host, args.port)
    try:
        robot.connect(timeout=3.0)
        robot.configure_calibrated_speed_control()
        for angle in range(args.start_angle, args.end_angle + 1):
            pose = set_ry(robot, float(angle))
            forward = measure_segment(robot, args.distance, args.speed)
            time.sleep(0.1)
            backward = measure_segment(robot, -args.distance, args.speed)
            row = {
                "ry_target_deg": angle,
                "ry_actual_deg": pose["v"],
                "requested_mm_s": args.speed,
                "forward": forward,
                "backward": backward,
                "bidirectional_average_mm_s": (
                    forward["average_mm_s"] + backward["average_mm_s"]
                )
                / 2.0,
                "bidirectional_central_mm_s": (
                    forward["central_mm_s"] + backward["central_mm_s"]
                )
                / 2.0,
            }
            if args.compact:
                print(
                    "CAL,"
                    f"{angle},"
                    f"{pose['v']:.6f},"
                    f"{row['bidirectional_average_mm_s']:.6f},"
                    f"{row['bidirectional_central_mm_s']:.6f},"
                    f"{forward['dx_mm']:.6f},"
                    f"{backward['dx_mm']:.6f}",
                    flush=True,
                )
            else:
                print(json.dumps(row, ensure_ascii=False), flush=True)
    finally:
        robot.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
