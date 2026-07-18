"""旧版多楼层服务兼容封装；正式运行链使用 ``BuildingControlClient``。

该模块保留给历史测试和配置，不应接入正式启动链：构造参数中的 lobby/inside
坐标不是公开 schema 字段，不能作为生产导航目标。服务名仍经过接口白名单，
但缺少 ``BuildingFeatureProvider`` 的公开 ID/served floors 双重约束。
"""
from __future__ import annotations

from dataclasses import dataclass
from threading import Lock, Thread
import time

try:
    import rospy
    from building_generator_interfaces.srv import CallElevator, SetDoorState
except ImportError:  # pragma: no cover - 离线环境可能没有 ROS
    rospy = None
    CallElevator = SetDoorState = None

from danger_search.interface_policy import require_allowed_service


@dataclass(frozen=True)
class ElevatorTargets:
    lobby_xy: tuple[float, float]
    inside_xy: tuple[float, float]


class AsyncServiceCall:
    """简单异步服务调用，避免 Timer 回调被门动画长时间阻塞。"""

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
        except Exception as exc:  # 服务异常必须显式反馈给状态机
            ok = False
            message = str(exc)
        with self._lock:
            self._ok = ok
            self._message = message
            self._done = True


class FloorManager:
    """提供旧配置兼容能力，不得作为正式几何信息或服务权限来源。"""

    def __init__(
        self,
        *,
        elevator_id: str = "elevator_main",
        lobby_xy: tuple[float, float] = (1.0, 2.6),
        inside_xy: tuple[float, float] = (2.6, 2.6),
        service_timeout: float = 5.0,
    ):
        self.elevator_id = elevator_id
        self.targets = ElevatorTargets(lobby_xy=lobby_xy, inside_xy=inside_xy)
        self.service_timeout = float(service_timeout)
        self._set_door_service = require_allowed_service(
            "/set_door_state",
            purpose="旧 FloorManager 门服务",
        )
        self._call_elevator_service = require_allowed_service(
            "/call_elevator",
            purpose="旧 FloorManager 电梯服务",
        )
        self._set_door = None
        self._call_elevator = None

    def start(self) -> None:
        if rospy is None:
            raise RuntimeError("当前环境没有 rospy，无法使用门/电梯服务")
        try:
            rospy.wait_for_service(
                self._set_door_service,
                timeout=self.service_timeout,
            )
            rospy.wait_for_service(
                self._call_elevator_service,
                timeout=self.service_timeout,
            )
            self._set_door = rospy.ServiceProxy(
                self._set_door_service,
                SetDoorState,
            )
            self._call_elevator = rospy.ServiceProxy(
                self._call_elevator_service,
                CallElevator,
            )
        except Exception as exc:
            rospy.logwarn("门/电梯服务暂不可用：%s", exc)

    def set_door_async(self, floor_index: int, open_state: bool) -> AsyncServiceCall:
        door_id = f"elevator_floor_{int(floor_index)}"

        def call():
            if self._set_door is None:
                raise RuntimeError("/set_door_state 服务未连接")
            return self._set_door(door_id=door_id, open=bool(open_state))

        task = AsyncServiceCall(f"set_door:{door_id}:{open_state}", call)
        task.start()
        return task

    def call_elevator_async(self, target_floor: int, open_doors: bool = False) -> AsyncServiceCall:
        def call():
            if self._call_elevator is None:
                raise RuntimeError("/call_elevator 服务未连接")
            return self._call_elevator(
                elevator_id=self.elevator_id,
                target_floor=int(target_floor),
                open_doors=bool(open_doors),
            )

        task = AsyncServiceCall(f"call_elevator:{target_floor}", call)
        task.start()
        return task

    def services_ready(self) -> bool:
        return self._set_door is not None and self._call_elevator is not None
