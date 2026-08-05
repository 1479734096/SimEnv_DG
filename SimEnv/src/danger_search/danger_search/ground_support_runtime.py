"""GroundSupport 证据反馈的并发队列与任务层运行状态。

安全判定只属于 ``GroundSupportSafetyGate``。本模块不判定支撑是否安全，只把支撑
线程输出串行交给主线程，并保存任务层重试和已登记禁区的有界运行状态。
"""
from __future__ import annotations

from dataclasses import dataclass
from threading import Lock

from danger_search.ground_support_gate import (
    CLEAR,
    DROP_STOPPING,
    OBSERVATION_HOLD,
    SAFETY_BLOCKED,
    SAFETY_STOP_TIMEOUT,
    SPEED_OUT_OF_ENVELOPE_STOPPING,
    STALE_DATA_HOLD,
    SUPPORT_PATH_UNAVAILABLE,
)


ACTION_CLEAR = "clear"
ACTION_HOLD = "hold"
ACTION_NEGATIVE_OBSTACLE = "negative_obstacle"
ACTION_TERMINAL = "terminal"
ACTION_PATH_REPLAN = "path_replan"


@dataclass(frozen=True)
class GroundSupportMapUpdate:
    negative_added: int
    negative_cleared_by_support: int

    @property
    def changed(self) -> bool:
        return self.negative_added > 0 or self.negative_cleared_by_support > 0


@dataclass(frozen=True)
class GateStateTransition:
    old_state: str
    new_state: str

    @property
    def changed(self) -> bool:
        return self.old_state != self.new_state


def classify_gate_action(state: str, reason: str) -> str:
    """把 Gate 状态映射为任务层动作，不改变 Gate 的锁存语义。"""
    if state == CLEAR:
        return ACTION_CLEAR
    if state in {
        OBSERVATION_HOLD,
        STALE_DATA_HOLD,
        DROP_STOPPING,
        SPEED_OUT_OF_ENVELOPE_STOPPING,
    }:
        return ACTION_HOLD
    if state == SAFETY_BLOCKED and reason == "negative_obstacle":
        return ACTION_NEGATIVE_OBSTACLE
    if state in {SAFETY_BLOCKED, SAFETY_STOP_TIMEOUT} or (
        state == SUPPORT_PATH_UNAVAILABLE
        and reason in {"support_speed_out_of_envelope", "localization_speed_abnormal"}
    ):
        return ACTION_TERMINAL
    if state == SUPPORT_PATH_UNAVAILABLE:
        return ACTION_PATH_REPLAN
    return ACTION_HOLD


class GroundSupportFeedbackRuntime:
    """持有 Gate 之外的反馈协调状态；并发写入只发生在证据队列。"""

    def __init__(self) -> None:
        self._lock = Lock()
        self._pending_negative: dict[int, set[tuple[int, int]]] = {}
        self._pending_supported: dict[int, set[tuple[int, int]]] = {}
        self.last_gate_state = CLEAR
        self.path_retry_key: tuple[object, ...] | None = None
        self.path_retry_count = 0
        self.negative_replan_count = 0
        self.negative_obstacle_marks: list[tuple[int, float, float, float]] = []

    def queue_map_updates(self, result: dict, *, default_floor: int) -> None:
        """从支撑线程登记有限格坐标；畸形负载不进入正式地图。"""
        negative = _grid_cells(result.get("confirmed_negative_cells"))
        supported = _grid_cells(result.get("confirmed_supported_cells"))
        if not negative and not supported:
            return
        try:
            floor = int(result.get("floor", default_floor))
        except (TypeError, ValueError, OverflowError):
            return
        with self._lock:
            if negative:
                self._pending_negative.setdefault(floor, set()).update(negative)
            if supported:
                self._pending_supported.setdefault(floor, set()).update(supported)

    def apply_map_updates(self, mapper) -> GroundSupportMapUpdate:
        """由主线程排空队列；同批次显式正支撑最后应用。"""
        with self._lock:
            negative = self._pending_negative
            supported = self._pending_supported
            self._pending_negative = {}
            self._pending_supported = {}
        added = sum(
            int(mapper.mark_negative_cells(floor, cells))
            for floor, cells in negative.items()
        )
        cleared = sum(
            int(mapper.clear_negative_cells_with_support(floor, cells))
            for floor, cells in supported.items()
        )
        return GroundSupportMapUpdate(added, cleared)

    def observe_gate_state(self, state: str) -> GateStateTransition:
        transition = GateStateTransition(self.last_gate_state, str(state))
        self.last_gate_state = str(state)
        return transition

    def synchronize_gate_state(self, state: str) -> None:
        self.last_gate_state = str(state)

    def record_path_retry(self, retry_key: tuple[object, ...]) -> int:
        if retry_key == self.path_retry_key:
            self.path_retry_count += 1
        else:
            self.path_retry_key = retry_key
            self.path_retry_count = 1
        return self.path_retry_count

    def can_replan_negative_obstacle(self, maximum: int) -> bool:
        return self.negative_replan_count < max(0, int(maximum))

    def record_negative_obstacle(
        self,
        *,
        floor: int,
        x: float,
        y: float,
        radius: float,
    ) -> int:
        self.negative_obstacle_marks.append(
            (int(floor), float(x), float(y), float(radius))
        )
        self.negative_replan_count += 1
        return self.negative_replan_count

    def black_marks_for_floor(
        self,
        floor: int,
    ) -> tuple[tuple[int, float, float, float], ...]:
        return tuple(
            mark for mark in self.negative_obstacle_marks if mark[0] == int(floor)
        )


def _grid_cells(payload) -> set[tuple[int, int]]:
    cells: set[tuple[int, int]] = set()
    if not isinstance(payload, (list, tuple)):
        return cells
    for item in payload:
        if not isinstance(item, (list, tuple)) or len(item) < 2:
            continue
        try:
            gx = int(item[0])
            gy = int(item[1])
        except (TypeError, ValueError, OverflowError):
            continue
        cells.add((gx, gy))
    return cells
