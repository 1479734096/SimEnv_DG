"""导航子任务可见的最小显式上下文。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class NavigationTaskContext:
    controller: object
    mapper: object
    building_client: object
    current_floor: int
    door_pass_max_linear: float | None
    open_door_geometry_refresh_enabled: bool
    elevator_geometry_refresh_enabled: bool
    elevator_geometry_refresh_standoff: float
    elevator_opening_confirmation_timeout: float
    elevator_stuck_retry_limit: int
    exit_first_max_linear: float | None
    exit_cabin_backoff_enabled: bool
    clear_opened_door_area_fn: Callable[[str], None]
    refresh_open_door_geometry_fn: Callable[..., str]
    refresh_elevator_geometry_fn: Callable[..., str]
    record_failure_fn: Callable[[str], None]
    log_structured_fn: Callable[[dict[str, object]], None]
    begin_floor_transition_fn: Callable[..., None]
    confirm_floor_transition_fn: Callable[[int], None]
    begin_door_open_confirmation_fn: Callable[[str], None]
    confirm_door_open_fn: Callable[..., bool]
    finish_door_open_confirmation_fn: Callable[[str], None]
    elevator_visual_preparation_pending_fn: Callable[[str], bool]
    recover_with_rotation_clearance_fn: Callable[[str], bool]
    elevator_start_retreat_target_fn: Callable[..., object]
    elevator_approach_trace_candidates_fn: Callable[..., object]

    def clear_opened_door_area(self, door_id: str) -> None:
        self.clear_opened_door_area_fn(door_id)

    def refresh_open_door_geometry(self, *args, **kwargs) -> str:
        return self.refresh_open_door_geometry_fn(*args, **kwargs)

    def refresh_elevator_geometry(self, *args, **kwargs) -> str:
        return self.refresh_elevator_geometry_fn(*args, **kwargs)

    def record_failure(self, reason: str) -> None:
        self.record_failure_fn(str(reason))

    def log_structured(self, payload: dict[str, object]) -> None:
        self.log_structured_fn(payload)

    def begin_floor_transition(self, **kwargs) -> None:
        self.begin_floor_transition_fn(**kwargs)

    def confirm_floor_transition(self, target_floor: int) -> None:
        self.confirm_floor_transition_fn(int(target_floor))

    def begin_elevator_door_open_confirmation(self, door_id: str) -> None:
        self.begin_door_open_confirmation_fn(door_id)

    def confirm_elevator_door_open(self, *args, **kwargs) -> bool:
        return bool(self.confirm_door_open_fn(*args, **kwargs))

    def finish_elevator_door_open_confirmation(self, door_id: str) -> None:
        self.finish_door_open_confirmation_fn(door_id)

    def elevator_visual_preparation_pending(self, door_id: str) -> bool:
        return bool(self.elevator_visual_preparation_pending_fn(door_id))

    def recover_with_rotation_clearance(self, source: str) -> bool:
        return bool(self.recover_with_rotation_clearance_fn(source))

    def elevator_start_retreat_target(self, *args, **kwargs):
        return self.elevator_start_retreat_target_fn(*args, **kwargs)

    def elevator_approach_trace_candidates(self, *args, **kwargs):
        return self.elevator_approach_trace_candidates_fn(*args, **kwargs)
