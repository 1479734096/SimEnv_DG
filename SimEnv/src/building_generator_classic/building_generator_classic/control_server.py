"""Gazebo Classic 门/电梯控制服务。

auto.sh 在 Gazebo 启动后运行本节点，并传入本次场景生成的
door_config.yaml 和 elevator_config.yaml。

本节点对外提供两个 ROS 服务：
- /set_door_state：开关指定动态门
- /call_elevator：把指定电梯移动到目标楼层

内部实现上，它先让 control_runtime.py 更新逻辑状态，再调用 Gazebo 的
/gazebo/set_model_state 或 /gazebo/set_link_state 移动实际模型。
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from building_generator_classic.control_runtime import BuildingControlRuntime
import yaml

DEFAULT_DOOR_ANIMATION_RATE_HZ = 10.0
DEFAULT_ELEVATOR_PASSENGER_MODEL = "a1_gazebo"


def main(argv: list[str] | None = None) -> int:
    import rospy
    from gazebo_msgs.srv import GetModelState, SetLinkState, SetModelState
    from building_generator_interfaces.srv import (
        CallElevator,
        CallElevatorResponse,
        SetDoorState,
        SetDoorStateResponse,
    )

    # rospy.myargv 会剥离 ROS 自己的 remap 参数，只留下本节点关心的命令行参数。
    args = _build_parser().parse_args(rospy.myargv(argv=argv)[1:])
    runtime = _load_runtime(args.door_config, args.elevator_config)

    rospy.init_node("building_generator_classic_control")
    # 等 Gazebo 服务可用后再注册本节点服务；否则第一次请求可能无法应用到模型。
    rospy.wait_for_service("/gazebo/set_model_state")
    rospy.wait_for_service("/gazebo/set_link_state")
    rospy.wait_for_service("/gazebo/get_model_state")
    set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    set_link_state = rospy.ServiceProxy("/gazebo/set_link_state", SetLinkState)
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    rospy.Service(
        "call_elevator",
        CallElevator,
        lambda request: _handle_call_elevator(
            runtime,
            request,
            CallElevatorResponse,
            set_model_state,
            get_model_state,
        ),
    )
    rospy.Service(
        "set_door_state",
        SetDoorState,
        lambda request: _handle_set_door_state(runtime, request, SetDoorStateResponse, set_model_state, set_link_state),
    )
    rospy.loginfo("building_generator_classic_control ready")
    rospy.spin()
    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Building generator Gazebo Classic control server")
    parser.add_argument("--door-config", required=True)
    parser.add_argument("--elevator-config", required=True)
    return parser


def _load_runtime(door_config_path: str, elevator_config_path: str) -> BuildingControlRuntime:
    """读取场景生成阶段写出的门/电梯配置，构造运行时状态机。"""
    door_config = yaml.safe_load(Path(door_config_path).read_text()) or {}
    elevator_config = yaml.safe_load(Path(elevator_config_path).read_text()) or {}
    return BuildingControlRuntime(
        door_specs=door_config.get("doors", []),
        elevator_specs=elevator_config.get("elevators", []),
    )


def _handle_call_elevator(
    runtime: BuildingControlRuntime,
    request,
    response_type,
    set_model_state,
    get_model_state=None,
):
    """ROS /call_elevator 服务处理函数。"""
    result = runtime.call_elevator(
        request.elevator_id,
        request.target_floor,
        request.open_doors,
    )
    passenger_state = None
    passenger_lookup_success = False
    passenger_inside_elevator = False
    passenger_before_xyz = None
    if (
        result.get("accepted")
        and result.get("source_pose")
        and result.get("target_pose")
        and result.get("car_size")
        and get_model_state is not None
    ):
        try:
            candidate = get_model_state(DEFAULT_ELEVATOR_PASSENGER_MODEL, "world")
            passenger_lookup_success = bool(getattr(candidate, "success", True))
            if passenger_lookup_success:
                passenger_before_xyz = [
                    float(candidate.pose.position.x),
                    float(candidate.pose.position.y),
                    float(candidate.pose.position.z),
                ]
                passenger_inside_elevator = _passenger_is_inside_elevator(
                    candidate.pose,
                    result["source_pose"],
                    result["car_size"],
                )
                if passenger_inside_elevator:
                    passenger_state = candidate
        except Exception as exc:
            import rospy

            rospy.logwarn("读取电梯乘客状态失败，轿厢仍按原逻辑移动: %s", exc)

    # 逻辑请求通过后，把电梯轿厢模型移动到目标楼层。
    transport_applied = False
    passenger_requested_after_z = None
    if result.get("accepted") and result.get("target_pose"):
        _apply_model_pose(set_model_state, result["model_name"], result["target_pose"])
        if passenger_state is not None:
            z_offset = float(result["target_pose"][2]) - float(result["source_pose"][2])
            passenger_requested_after_z = float(passenger_state.pose.position.z) + z_offset
            _apply_model_z_offset(
                set_model_state,
                DEFAULT_ELEVATOR_PASSENGER_MODEL,
                passenger_state,
                z_offset,
            )
            transport_applied = True
            import rospy

            rospy.loginfo(
                "电梯已同步运输乘客模型: model=%s z_offset=%.3f",
                DEFAULT_ELEVATOR_PASSENGER_MODEL,
                z_offset,
            )
    if result.get("accepted"):
        import rospy

        rospy.loginfo(
            "%s",
            json.dumps(
                {
                    "event": "elevator_transport",
                    "elevator_id": request.elevator_id,
                    "target_floor": int(request.target_floor),
                    "source_z": (
                        float(result["source_pose"][2])
                        if result.get("source_pose")
                        else None
                    ),
                    "target_z": (
                        float(result["target_pose"][2])
                        if result.get("target_pose")
                        else None
                    ),
                    "passenger_model": DEFAULT_ELEVATOR_PASSENGER_MODEL,
                    "passenger_lookup_success": passenger_lookup_success,
                    "passenger_inside_elevator": passenger_inside_elevator,
                    "passenger_before_xyz": passenger_before_xyz,
                    "passenger_requested_after_z": passenger_requested_after_z,
                    "transport_applied": transport_applied,
                },
                ensure_ascii=False,
                sort_keys=True,
            ),
        )
    return response_type(
        accepted=bool(result["accepted"]),
        current_floor=int(result["current_floor"]),
        state=str(result["state"]),
        message=str(result["message"]),
    )


def _passenger_is_inside_elevator(
    passenger_pose,
    elevator_pose: list[float],
    car_size: list[float],
    margin: float = 0.12,
) -> bool:
    """按轿厢局部坐标判断机器人是否位于轿厢内部。"""
    if len(elevator_pose) < 6 or len(car_size) < 3:
        return False
    yaw = float(elevator_pose[5])
    dx = float(passenger_pose.position.x) - float(elevator_pose[0])
    dy = float(passenger_pose.position.y) - float(elevator_pose[1])
    dz = float(passenger_pose.position.z) - float(elevator_pose[2])
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    horizontal_margin = max(0.0, float(margin))
    vertical_margin = 0.25
    return bool(
        abs(local_x) <= float(car_size[0]) / 2.0 + horizontal_margin
        and abs(local_y) <= float(car_size[1]) / 2.0 + horizontal_margin
        and -float(car_size[2]) / 2.0 - vertical_margin
        <= dz
        <= float(car_size[2]) / 2.0 + vertical_margin
    )


def _apply_model_z_offset(set_model_state, model_name: str, source_state, z_offset: float) -> None:
    """保留乘客姿态与速度，只施加与轿厢一致的竖直位移。"""
    from gazebo_msgs.msg import ModelState

    model_state = ModelState()
    model_state.model_name = model_name
    model_state.reference_frame = "world"
    model_state.pose = source_state.pose
    model_state.pose.position.z += float(z_offset)
    model_state.twist = source_state.twist
    set_model_state(model_state)


def _handle_set_door_state(runtime: BuildingControlRuntime, request, response_type, set_model_state, set_link_state):
    """ROS /set_door_state 服务处理函数。"""
    result = runtime.set_door_state(request.door_id, request.open)
    if result.get("accepted") and result.get("panel_poses"):
        # 门模型的 base pose 固定在门洞位置；真正“开合”的是左右门板 link。
        _apply_model_pose(set_model_state, result["model_name"], result["model_pose"])
        motion_duration = float(result.get("motion_duration", 0.0) or 0.0)
        if motion_duration > 0.0:
            _animate_panel_poses(
                set_link_state,
                result["model_name"],
                result["model_pose"],
                result.get("start_panel_poses", {}) or {},
                result["panel_poses"],
                motion_duration,
            )
        else:
            _apply_panel_poses(
                set_link_state,
                result["model_name"],
                result["model_pose"],
                result["panel_poses"],
            )
    elif result.get("accepted") and result.get("target_pose"):
        _apply_model_pose(set_model_state, result["model_name"], result["target_pose"])
    return response_type(
        accepted=bool(result["accepted"]),
        state=str(result["state"]),
        message=str(result["message"]),
    )


def _apply_panel_poses(
    set_link_state,
    model_name: str,
    model_pose: list[float],
    panel_poses: dict[str, list[float] | None],
) -> None:
    """把左右门板局部位姿应用到 Gazebo link。"""
    for link_name, pose_values in panel_poses.items():
        if pose_values:
            _apply_link_pose(
                set_link_state,
                model_name,
                link_name,
                _compose_world_pose(model_pose, pose_values),
            )


def _animate_panel_poses(
    set_link_state,
    model_name: str,
    model_pose: list[float],
    start_panel_poses: dict[str, list[float] | None],
    target_panel_poses: dict[str, list[float] | None],
    duration: float,
) -> None:
    """按固定频率插值门板位姿，实现电梯门等慢速开合动画。"""
    import rospy

    duration = max(0.0, float(duration))
    if duration <= 0.0:
        _apply_panel_poses(set_link_state, model_name, model_pose, target_panel_poses)
        return

    step_count = max(1, int(math.ceil(duration * DEFAULT_DOOR_ANIMATION_RATE_HZ)))
    step_period = duration / step_count
    link_names = sorted(target_panel_poses.keys())
    start_time = time.monotonic()
    rospy.loginfo("Animating %s door panels over %.1f seconds", model_name, duration)

    for step in range(step_count + 1):
        if rospy.is_shutdown():
            return
        ratio = step / step_count
        current_poses: dict[str, list[float]] = {}
        for link_name in link_names:
            target_pose = target_panel_poses.get(link_name)
            if not target_pose:
                continue
            start_pose = start_panel_poses.get(link_name) or target_pose
            current_poses[link_name] = _interpolate_pose(start_pose, target_pose, ratio)
        _apply_panel_poses(set_link_state, model_name, model_pose, current_poses)

        next_time = start_time + (step + 1) * step_period
        sleep_time = next_time - time.monotonic()
        if step < step_count and sleep_time > 0.0:
            rospy.sleep(sleep_time)


def _interpolate_pose(start_pose: list[float], target_pose: list[float], ratio: float) -> list[float]:
    ratio = max(0.0, min(1.0, float(ratio)))
    return [
        float(start_value) + (float(target_value) - float(start_value)) * ratio
        for start_value, target_value in zip(start_pose, target_pose)
    ]


def _apply_model_pose(set_model_state, model_name: str, pose_values: list[float]) -> None:
    """调用 /gazebo/set_model_state 移动整个 Gazebo model。"""
    from gazebo_msgs.msg import ModelState
    from geometry_msgs.msg import Pose
    from tf.transformations import quaternion_from_euler

    model_state = ModelState()
    model_state.model_name = model_name
    model_state.reference_frame = "world"
    model_state.pose = Pose()
    model_state.pose.position.x = float(pose_values[0])
    model_state.pose.position.y = float(pose_values[1])
    model_state.pose.position.z = float(pose_values[2])
    qx, qy, qz, qw = quaternion_from_euler(
        float(pose_values[3]),
        float(pose_values[4]),
        float(pose_values[5]),
    )
    model_state.pose.orientation.x = qx
    model_state.pose.orientation.y = qy
    model_state.pose.orientation.z = qz
    model_state.pose.orientation.w = qw
    set_model_state(model_state)


def _apply_link_pose(set_link_state, model_name: str, link_name: str, pose_values: list[float]) -> None:
    """调用 /gazebo/set_link_state 移动某个 model 内的 link。"""
    from gazebo_msgs.msg import LinkState
    from geometry_msgs.msg import Pose
    from tf.transformations import quaternion_from_euler

    link_state = LinkState()
    link_state.link_name = f"{model_name}::{link_name}"
    link_state.reference_frame = "world"
    link_state.pose = Pose()
    link_state.pose.position.x = float(pose_values[0])
    link_state.pose.position.y = float(pose_values[1])
    link_state.pose.position.z = float(pose_values[2])
    qx, qy, qz, qw = quaternion_from_euler(
        float(pose_values[3]),
        float(pose_values[4]),
        float(pose_values[5]),
    )
    link_state.pose.orientation.x = qx
    link_state.pose.orientation.y = qy
    link_state.pose.orientation.z = qz
    link_state.pose.orientation.w = qw
    set_link_state(link_state)


def _compose_world_pose(base_pose: list[float], local_pose: list[float]) -> list[float]:
    """把门板局部 pose 合成为 Gazebo world pose。"""
    yaw = float(base_pose[5])
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    local_x = float(local_pose[0])
    local_y = float(local_pose[1])
    return [
        float(base_pose[0]) + local_x * cos_yaw - local_y * sin_yaw,
        float(base_pose[1]) + local_x * sin_yaw + local_y * cos_yaw,
        float(base_pose[2]) + float(local_pose[2]),
        float(base_pose[3]) + float(local_pose[3]),
        float(base_pose[4]) + float(local_pose[4]),
        float(base_pose[5]) + float(local_pose[5]),
    ]

__all__ = ["main"]
