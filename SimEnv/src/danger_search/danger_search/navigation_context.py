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
    planning_constraints: object | None
    door_pass_max_linear: float | None
    open_door_geometry_refresh_enabled: bool
    elevator_geometry_refresh_enabled: bool
    elevator_geometry_refresh_standoff: float
    elevator_opening_confirmation_timeout: float
    elevator_local_crossing_geometry_enabled: bool
    elevator_local_crossing_timeout: float
    elevator_local_crossing_max_center_correction: float
    elevator_stuck_retry_limit: int
    exit_first_max_linear: float | None
    exit_cabin_backoff_enabled: bool
    clear_opened_door_area_fn: Callable[[str], None]
    clear_elevator_crossing_area_fn: Callable[[object], None]
    refresh_open_door_geometry_fn: Callable[..., str]
    refresh_elevator_geometry_fn: Callable[..., str]
    refresh_elevator_crossing_geometry_fn: Callable[..., object]
    elevator_crossing_obstacle_fn: Callable[..., object]
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
    plan_path_fn: Callable[..., object]
    segment_is_traversable_fn: Callable[..., bool]
    elevator_async_target_door_close_enabled: bool = True
    relocalize_landmark_fn: Callable[..., object] | None = None
    latest_cloud_stamp_fn: Callable[[], float | None] | None = None
    elevator_opening_confirmation_retry_limit: int = 1
    elevator_opening_confirmation_retry_timeout: float = 10.0
    elevator_opening_confirmation_sensor_refresh: float = 0.5
    entrance_geometry_correction_fn: Callable[..., object] | None = None
    elevator_wait_position_tolerance: float = 0.30
    elevator_wait_heading_tolerance: float = 0.15
    elevator_wait_lateral_tolerance: float = 0.15
    elevator_lateral_search_extra_overshoot: float = 0.15
    elevator_lateral_search_speed: float = 0.15
    pose_stamp: float | None = None
    observe_post_exit_door_fn: Callable[..., object] | None = None
    freeze_landmark_anchor_fn: Callable[[str, str], bool] | None = None
    current_landmark_frames_fn: Callable[[], list] | None = None
    correct_post_exit_elevator_drift_fn: Callable[..., str] | None = None

    def clear_opened_door_area(self, door_id: str) -> None:
        self.clear_opened_door_area_fn(door_id)

    def clear_elevator_crossing_area(self, geometry) -> None:
        self.clear_elevator_crossing_area_fn(geometry)

    def refresh_open_door_geometry(self, *args, **kwargs) -> str:
        return self.refresh_open_door_geometry_fn(*args, **kwargs)

    def refresh_elevator_geometry(self, *args, **kwargs) -> str:
        return self.refresh_elevator_geometry_fn(*args, **kwargs)

    def refresh_elevator_crossing_geometry(self, *args, **kwargs):
        return self.refresh_elevator_crossing_geometry_fn(*args, **kwargs)

    def elevator_crossing_obstacle(self, *args, **kwargs):
        return self.elevator_crossing_obstacle_fn(*args, **kwargs)

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

    def plan_path(self, *args, **kwargs):
        return self.plan_path_fn(*args, **kwargs)

    def segment_is_traversable(self, *args, **kwargs) -> bool:
        return bool(self.segment_is_traversable_fn(*args, **kwargs))

    def relocalize_landmark(self, *args, **kwargs):
        if self.relocalize_landmark_fn is None:
            return None
        return self.relocalize_landmark_fn(*args, **kwargs)

    def latest_cloud_stamp(self) -> float | None:
        return self.latest_cloud_stamp_fn() if self.latest_cloud_stamp_fn else None

    def correct_entrance_geometry(self, *args, **kwargs):
        """请求固定主入口几何纠偏；未接入时保持任务兼容。"""
        if self.entrance_geometry_correction_fn is None:
            return None
        return self.entrance_geometry_correction_fn(*args, **kwargs)

    def observe_post_exit_door(self, *args, **kwargs):
        if self.observe_post_exit_door_fn is None:
            return None
        return self.observe_post_exit_door_fn(*args, **kwargs)

    def freeze_landmark_anchor(self, landmark_id: str, door_id: str) -> bool:
        if self.freeze_landmark_anchor_fn is None:
            return False
        return bool(self.freeze_landmark_anchor_fn(landmark_id, door_id))

    def current_landmark_frames(self) -> list:
        if self.current_landmark_frames_fn is None:
            return []
        return self.current_landmark_frames_fn()

    def correct_post_exit_elevator_drift(self, elevator_id: str, floor: int) -> str:
        if self.correct_post_exit_elevator_drift_fn is None:
            return "done"
        return str(self.correct_post_exit_elevator_drift_fn(elevator_id, floor))

