"""门/电梯客户端控制封装。"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from threading import Lock, Thread

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

@dataclass
class ElevatorInfo:
    elevator_id: str
    lobby_poses: dict[int, tuple[float, float, float]]  # floor -> (x, y, yaw)


class BuildingControlClient:
    def __init__(self, metadata_path: str = "generated_building/layout_metadata.json", service_timeout: float = 5.0):
        self.metadata_path = metadata_path
        self.service_timeout = float(service_timeout)
        self.doors: dict[str, DoorInfo] = {}
        self.elevators: dict[str, ElevatorInfo] = {}

        self._set_door = None
        self._call_elevator = None
        self._load_metadata()

    def _load_metadata(self) -> None:
        path = Path(self.metadata_path)
        if not path.is_file():
            if rospy:
                rospy.logwarn("找不到场景元数据文件: %s", self.metadata_path)
            return

        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        for door_spec in data.get("door_specs", []):
            if not door_spec.get("dynamic", False):
                continue
            door_id = door_spec["id"]
            pose = door_spec.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.doors[door_id] = DoorInfo(
                door_id=door_id,
                kind=door_spec.get("kind", ""),
                floor_index=int(door_spec.get("floor_index", 0)),
                x=float(pose[0]),
                y=float(pose[1]),
                yaw=float(pose[5]),
            )

        for el_spec in data.get("elevator_specs", []):
            el_id = el_spec["id"]
            lobby_poses = {}
            for floor_str, pose in el_spec.get("lobby_positions", {}).items():
                lobby_poses[int(floor_str)] = (float(pose[0]), float(pose[1]), float(pose[5]))
            self.elevators[el_id] = ElevatorInfo(elevator_id=el_id, lobby_poses=lobby_poses)

    def start(self, *, connect_elevator: bool = True) -> None:
        if rospy is None:
            raise RuntimeError("没有 rospy 环境")
        try:
            rospy.wait_for_service("/set_door_state", timeout=self.service_timeout)
            self._set_door = rospy.ServiceProxy("/set_door_state", SetDoorState)
        except Exception as exc:
            rospy.logwarn("开关门服务初始化失败/超时：%s", exc)
        if connect_elevator:
            try:
                rospy.wait_for_service("/call_elevator", timeout=self.service_timeout)
                self._call_elevator = rospy.ServiceProxy("/call_elevator", CallElevator)
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
        def call():
            if self._set_door is None:
                raise RuntimeError("/set_door_state 尚未连接")
            return self._set_door(door_id=door_id, open=True)
        task = AsyncServiceCall(f"open_door:{door_id}", call)
        task.start()
        return task

    def close_door(self, door_id: str) -> AsyncServiceCall:
        def call():
            if self._set_door is None:
                raise RuntimeError("/set_door_state 尚未连接")
            return self._set_door(door_id=door_id, open=False)
        task = AsyncServiceCall(f"close_door:{door_id}", call)
        task.start()
        return task

    def call_elevator(self, elevator_id: str, floor: int, open_doors: bool = False) -> AsyncServiceCall:
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

    def get_door_points(self, door_id: str, robot_pose: tuple[float, float], offset: float = 0.8) -> tuple[tuple[float, float], tuple[float, float]]:
        """计算过门的等待点和通过点。"""
        if door_id not in self.doors:
            raise ValueError(f"Unknown door_id: {door_id}")
        door = self.doors[door_id]
        dx = math.cos(door.yaw) * offset
        dy = math.sin(door.yaw) * offset

        p1 = (door.x + dx, door.y + dy)
        p2 = (door.x - dx, door.y - dy)

        # 判断 robot_pose 离 p1 近还是 p2 近
        d1 = math.hypot(robot_pose[0] - p1[0], robot_pose[1] - p1[1])
        d2 = math.hypot(robot_pose[0] - p2[0], robot_pose[1] - p2[1])

        if d1 < d2:
            return p1, p2
        else:
            return p2, p1

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
