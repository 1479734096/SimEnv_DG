"""公开门/电梯服务、运行时空间注册与过门几何的统一封装。

公开场景文件只提供允许调用的 ID 和电梯服务楼层；门中心、朝向与轿厢等待点
必须由合法传感器在运行时注册到 ``doors/elevators``。服务请求前再次校验公开
集合，空间对象存在不代表 ID 自动获得调用权限。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from threading import Lock, Thread

from danger_search.interface_policy import require_allowed_service

try:
    import rospy
    from building_generator_interfaces.srv import CallElevator, SetDoorState
except ImportError:
    rospy = None
    CallElevator = SetDoorState = None


class AsyncServiceCall:
    """在线程中执行一次 ROS 服务请求并暴露线程安全的轮询结果。

    门动画和电梯运输可能远慢于 10 Hz 主状态机，任务对象只轮询 ``done/ok``，
    等待期间仍可持续停车、处理传感器和执行安全检查。
    """

    def __init__(self, name: str, fn):
        self.name = name
        self._fn = fn
        self._lock = Lock()
        self._thread: Thread | None = None
        self._done = False
        self._ok = False
        self._message = ""

    def start(self) -> None:
        with self._lock:
            if self._thread is not None:
                return
            self._thread = Thread(target=self._run, daemon=True)
            self._thread.start()

    def done(self) -> bool:
        with self._lock:
            return self._done

    def ok(self) -> bool:
        with self._lock:
            return self._ok

    def message(self) -> str:
        with self._lock:
            return self._message

    def _run(self) -> None:
        ok = False
        message = ""
        try:
            result = self._fn()
            ok = bool(getattr(result, "accepted", True))
            message = str(getattr(result, "message", ""))
        except Exception as exc:
            ok = False
            message = str(exc)
        with self._lock:
            self._ok = ok
            self._message = message
            self._done = True


@dataclass
class DoorInfo:
    """由传感器注册的门空间几何；``door_id`` 仍须属于公开 ID 集。"""
    door_id: str
    kind: str
    floor_index: int
    x: float
    y: float
    yaw: float
    width: float = 1.0

    def __post_init__(self) -> None:
        self.door_id = str(self.door_id).strip()
        self.kind = str(self.kind).strip()
        self.floor_index = int(self.floor_index)
        self.x = float(self.x)
        self.y = float(self.y)
        self.yaw = float(self.yaw)
        self.width = float(self.width)
        if (
            not self.door_id
            or not self.kind
            or not all(
                math.isfinite(value)
                for value in (self.x, self.y, self.yaw, self.width)
            )
            or self.width <= 0.0
        ):
            raise ValueError("运行时门 ID、类型、位置、朝向或宽度无效")

@dataclass
class ElevatorInfo:
    """由传感器注册的各楼层电梯厅姿态，不包含服务权限。"""
    elevator_id: str
    lobby_poses: dict[int, tuple[float, float, float]]  # floor -> (x, y, yaw)

    def __post_init__(self) -> None:
        self.elevator_id = str(self.elevator_id).strip()
        resolved: dict[int, tuple[float, float, float]] = {}
        for floor, pose in self.lobby_poses.items():
            if len(pose) != 3:
                raise ValueError("电梯厅姿态必须是三维 XY-yaw")
            values = tuple(float(value) for value in pose)
            if not all(math.isfinite(value) for value in values):
                raise ValueError("电梯厅姿态必须是有限数")
            resolved[int(floor)] = values
        if not self.elevator_id or not resolved:
            raise ValueError("运行时电梯 ID 和厅姿态不能为空")
        self.lobby_poses = resolved


class BuildingControlClient:
    """只封装初始文档允许的门/电梯服务。

    公开 scene info 不提供空间坐标，因此本类不读取布局元数据，也不会自行
    创建可导航的门或电梯目标。后续只有传感器发现并提供空间几何时才可接入。
    """

    def __init__(self, service_timeout: float = 5.0):
        self.service_timeout = float(service_timeout)
        if not math.isfinite(self.service_timeout) or self.service_timeout <= 0.0:
            raise ValueError("服务连接超时必须是正有限数")
        self.doors: dict[str, DoorInfo] = {}
        self.elevators: dict[str, ElevatorInfo] = {}
        self.public_door_ids: set[str] = set()
        self.public_elevators: dict[str, set[int]] = {}
        self.public_elevator_door_ids_by_floor: dict[int, tuple[str, ...]] = {}

        self._set_door = None
        self._call_elevator = None

    def set_public_features(
        self,
        *,
        door_ids: set[str] | frozenset[str],
        elevators: dict[str, set[int] | frozenset[int]],
        elevator_door_ids_by_floor: dict[int, list[str] | tuple[str, ...]] | None = None,
    ) -> None:
        """登记公开 ID 和服务楼层，不把公开信息错误扩展为坐标真值。"""
        self.public_door_ids = {str(door_id) for door_id in door_ids}
        self.public_elevators = {
            str(elevator_id): {int(floor) for floor in floors}
            for elevator_id, floors in elevators.items()
        }
        self.public_elevator_door_ids_by_floor = {
            int(floor): tuple(str(door_id) for door_id in door_ids)
            for floor, door_ids in (elevator_door_ids_by_floor or {}).items()
        }

    def elevator_door_id(self, floor: int) -> str:
        """返回公开 schema 中该楼层唯一的电梯门 ID。"""
        door_ids = self.public_elevator_door_ids_by_floor.get(int(floor), ())
        if len(door_ids) != 1:
            raise ValueError(
                f"楼层 {int(floor)} 必须恰好声明一个电梯门，实际为 {len(door_ids)}"
            )
        return door_ids[0]

    def start(self, *, connect_elevator: bool = True) -> None:
        """连接初始文档允许的服务；失败时保留未就绪状态供任务显式处理。"""
        if rospy is None:
            raise RuntimeError("没有 rospy 环境")
        try:
            service = require_allowed_service("/set_door_state", purpose="门服务客户端")
            rospy.wait_for_service(service, timeout=self.service_timeout)
            self._set_door = rospy.ServiceProxy(service, SetDoorState)
        except Exception as exc:
            rospy.logwarn("开关门服务初始化失败/超时：%s", exc)
        if connect_elevator:
            try:
                service = require_allowed_service("/call_elevator", purpose="电梯服务客户端")
                rospy.wait_for_service(service, timeout=self.service_timeout)
                self._call_elevator = rospy.ServiceProxy(service, CallElevator)
            except Exception as exc:
                rospy.logwarn("电梯服务初始化失败/超时：%s", exc)

        if self._set_door is not None or self._call_elevator is not None:
            rospy.loginfo("门/电梯服务客户端初始化成功。")

    def services_ready(self) -> bool:
        return self._set_door is not None and self._call_elevator is not None

    def door_service_ready(self) -> bool:
        return self._set_door is not None

    def has_controllable_doors(self) -> bool:
        return bool(self.doors)

    def controllable_doors(self, *, floor_index: int | None = None, include_elevators: bool = True) -> list[DoorInfo]:
        doors = []
        for door in self.doors.values():
            if floor_index is not None and door.floor_index != floor_index:
                continue
            if door.kind == "elevator" and not include_elevators:
                continue
            doors.append(door)
        return doors

    def is_elevator_door(self, door_id: str) -> bool:
        door = self.doors.get(door_id)
        return door is not None and door.kind == "elevator"

    def open_door(self, door_id: str) -> AsyncServiceCall:
        """校验公开门 ID 后异步请求打开；不根据响应直接修改地图。"""
        if door_id not in self.public_door_ids:
            raise ValueError(f"门 ID 未在公开场景信息中声明: {door_id}")
        def call():
            if self._set_door is None:
                raise RuntimeError("/set_door_state 尚未连接")
            return self._set_door(door_id=door_id, open=True)
        task = AsyncServiceCall(f"open_door:{door_id}", call)
        task.start()
        return task

    def close_door(self, door_id: str) -> AsyncServiceCall:
        """校验公开门 ID 后异步请求关闭。"""
        if door_id not in self.public_door_ids:
            raise ValueError(f"门 ID 未在公开场景信息中声明: {door_id}")
        def call():
            if self._set_door is None:
                raise RuntimeError("/set_door_state 尚未连接")
            return self._set_door(door_id=door_id, open=False)
        task = AsyncServiceCall(f"close_door:{door_id}", call)
        task.start()
        return task

    def call_elevator(self, elevator_id: str, floor: int, open_doors: bool = False) -> AsyncServiceCall:
        """仅对公开电梯及其 ``served_floors`` 发起异步呼叫。"""
        floors = self.public_elevators.get(elevator_id)
        if not floors or int(floor) not in floors:
            raise ValueError(f"电梯 {elevator_id} 不服务公开楼层 {floor}")
        def call():
            if self._call_elevator is None:
                raise RuntimeError("/call_elevator 尚未连接")
            return self._call_elevator(
                elevator_id=elevator_id,
                target_floor=int(floor),
                open_doors=bool(open_doors),
            )
        task = AsyncServiceCall(f"call_elevator:{elevator_id}:{floor}", call)
        task.start()
        return task

    def get_door_path_points(
        self,
        door_id: str,
        robot_pose: tuple[float, float],
        pass_offset: float = 0.8,
        commit_distance: float = 1.8,
    ) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
        """由传感器注册门法向计算等待点、通过点和门后深入点。

        法向有两个方向，离机器人更近的一侧作为等待侧，另一侧作为通过侧；
        commit 点继续沿相同方向深入，供探索器建立“已经完整过门”的空间上下文。
        """
        if door_id not in self.doors:
            raise ValueError(f"Unknown door_id: {door_id}")
        door = self.doors[door_id]
        try:
            robot_x, robot_y = float(robot_pose[0]), float(robot_pose[1])
            offset = float(pass_offset)
            commit = float(commit_distance)
        except (IndexError, TypeError, ValueError):
            raise ValueError("机器人位置和过门距离必须是二维有限数") from None
        if not all(
            math.isfinite(value) for value in (robot_x, robot_y, offset, commit)
        ):
            raise ValueError("机器人位置和过门距离必须是二维有限数")
        offset = max(offset, 0.05)
        commit = max(commit, offset)
        dx = math.cos(door.yaw) * offset
        dy = math.sin(door.yaw) * offset

        p1 = (door.x + dx, door.y + dy)
        p2 = (door.x - dx, door.y - dy)

        d1 = math.hypot(robot_x - p1[0], robot_y - p1[1])
        d2 = math.hypot(robot_x - p2[0], robot_y - p2[1])

        if d1 < d2:
            wait_point, pass_point = p1, p2
        else:
            wait_point, pass_point = p2, p1
        pass_dir_x = pass_point[0] - door.x
        pass_dir_y = pass_point[1] - door.y
        norm = math.hypot(pass_dir_x, pass_dir_y)
        if norm <= 1e-6:
            commit_point = pass_point
        else:
            scale = commit / norm
            commit_point = (door.x + pass_dir_x * scale, door.y + pass_dir_y * scale)
        return wait_point, pass_point, commit_point

    def get_door_points(self, door_id: str, robot_pose: tuple[float, float], offset: float = 0.8) -> tuple[tuple[float, float], tuple[float, float]]:
        """计算过门的等待点和通过点。"""
        wait_point, pass_point, _ = self.get_door_path_points(
            door_id,
            robot_pose,
            pass_offset=offset,
            commit_distance=offset,
        )
        return wait_point, pass_point

    def get_elevator_points(self, elevator_id: str, floor: int, offset_out: float = 0.8, offset_in: float = 0.95) -> tuple[tuple[float, float], tuple[float, float]]:
        """计算乘电梯的等待点（厅外）和轿厢中心点。"""
        if elevator_id not in self.elevators:
            raise ValueError(f"Unknown elevator_id: {elevator_id}")
        elevator = self.elevators[elevator_id]
        if floor not in elevator.lobby_poses:
            raise ValueError(f"Elevator {elevator_id} does not serve floor {floor}")

        ex, ey, eyaw = elevator.lobby_poses[floor]
        try:
            offset_out = float(offset_out)
            offset_in = float(offset_in)
        except (TypeError, ValueError):
            raise ValueError("电梯内外偏移必须是有限数") from None
        if not all(
            math.isfinite(value) and value >= 0.0
            for value in (offset_out, offset_in)
        ):
            raise ValueError("电梯内外偏移必须是非负有限数")
        # 电梯门法向量朝外（朝向大厅）
        dx_out = math.cos(eyaw) * offset_out
        dy_out = math.sin(eyaw) * offset_out
        dx_in = math.cos(eyaw) * offset_in
        dy_in = math.sin(eyaw) * offset_in

        wait_point = (ex + dx_out, ey + dy_out)
        cabin_center = (ex - dx_in, ey - dy_in)
        return wait_point, cabin_center
