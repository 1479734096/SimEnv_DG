"""门/电梯客户端控制封装。"""
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
    """异步服务调用，避免阻塞。"""

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
    door_id: str
    kind: str
    floor_index: int
    x: float
    y: float
    yaw: float
    width: float = 1.0

@dataclass
class ElevatorInfo:
    elevator_id: str
    lobby_poses: dict[int, tuple[float, float, float]]  # floor -> (x, y, yaw)


class BuildingControlClient:
    """只封装初始文档允许的门/电梯服务。

    公开 scene info 不提供空间坐标，因此本类不读取布局元数据，也不会自行
    创建可导航的门或电梯目标。后续只有传感器发现并提供空间几何时才可接入。
    """

    def __init__(self, service_timeout: float = 5.0):
        self.service_timeout = float(service_timeout)
        self.doors: dict[str, DoorInfo] = {}
        self.elevators: dict[str, ElevatorInfo] = {}
        self.public_door_ids: set[str] = set()
        self.public_elevators: dict[str, set[int]] = {}

        self._set_door = None
        self._call_elevator = None

    def set_public_features(
        self,
        *,
        door_ids: set[str] | frozenset[str],
        elevators: dict[str, set[int] | frozenset[int]],
    ) -> None:
        """登记公开 ID，不把公开信息错误扩展为坐标真值。"""
        self.public_door_ids = {str(door_id) for door_id in door_ids}
        self.public_elevators = {
            str(elevator_id): {int(floor) for floor in floors}
            for elevator_id, floors in elevators.items()
        }

    def start(self, *, connect_elevator: bool = True) -> None:
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
        if self.public_door_ids and door_id not in self.public_door_ids:
            raise ValueError(f"门 ID 未在公开场景信息中声明: {door_id}")
        def call():
            if self._set_door is None:
                raise RuntimeError("/set_door_state 尚未连接")
            return self._set_door(door_id=door_id, open=True)
        task = AsyncServiceCall(f"open_door:{door_id}", call)
        task.start()
        return task

    def close_door(self, door_id: str) -> AsyncServiceCall:
        if self.public_door_ids and door_id not in self.public_door_ids:
            raise ValueError(f"门 ID 未在公开场景信息中声明: {door_id}")
        def call():
            if self._set_door is None:
                raise RuntimeError("/set_door_state 尚未连接")
            return self._set_door(door_id=door_id, open=False)
        task = AsyncServiceCall(f"close_door:{door_id}", call)
        task.start()
        return task

    def call_elevator(self, elevator_id: str, floor: int, open_doors: bool = False) -> AsyncServiceCall:
        floors = self.public_elevators.get(elevator_id)
        if floors is not None and int(floor) not in floors:
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
        """计算过门的等待点、通过点和门后深入点。"""
        if door_id not in self.doors:
            raise ValueError(f"Unknown door_id: {door_id}")
        door = self.doors[door_id]
        offset = max(float(pass_offset), 0.05)
        commit = max(float(commit_distance), offset)
        dx = math.cos(door.yaw) * offset
        dy = math.sin(door.yaw) * offset

        p1 = (door.x + dx, door.y + dy)
        p2 = (door.x - dx, door.y - dy)

        d1 = math.hypot(robot_pose[0] - p1[0], robot_pose[1] - p1[1])
        d2 = math.hypot(robot_pose[0] - p2[0], robot_pose[1] - p2[1])

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
        # 电梯门法向量朝外（朝向大厅）
        dx_out = math.cos(eyaw) * offset_out
        dy_out = math.sin(eyaw) * offset_out
        dx_in = math.cos(eyaw) * offset_in
        dy_in = math.sin(eyaw) * offset_in

        wait_point = (ex + dx_out, ey + dy_out)
        cabin_center = (ex - dx_in, ey - dy_in)
        return wait_point, cabin_center
