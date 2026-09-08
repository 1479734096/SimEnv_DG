"""楼层切换期间的点云写入守卫。

该状态机只回答“当前楼层作用域能否消费传感器帧”，不更新任务状态、不调用
控制器，也不记录日志。调用方依据返回的冻结事件执行停车、日志和终局策略。
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Iterable


FLOOR_ALIGNMENT_REQUIRED_ODOM_SAMPLES = 2


@dataclass(frozen=True)
class FloorCoordinateModel:
    """统一解释定位 Z 与逻辑楼层坐标，不读取任务状态。"""

    vertical_reference: str
    floor_height: float

    def __post_init__(self) -> None:
        if self.vertical_reference not in {"absolute", "floor_local"}:
            raise ValueError("楼层坐标参考只能是 absolute 或 floor_local")
        if not math.isfinite(self.floor_height) or self.floor_height <= 0.0:
            raise ValueError("楼层高度必须为正有限数")

    def z_offset(self, floor_index: int) -> float:
        if self.vertical_reference == "absolute":
            return 0.0
        return int(floor_index) * self.floor_height

    def logical_xyz(
        self,
        xyz: tuple[float, float, float],
        floor_index: int,
    ) -> tuple[float, float, float]:
        return (
            float(xyz[0]),
            float(xyz[1]),
            float(xyz[2]) + self.z_offset(floor_index),
        )

    def logical_points(
        self,
        points: list[tuple[float, float, float]],
        floor_index: int,
    ) -> list[tuple[float, float, float]]:
        offset = self.z_offset(floor_index)
        if offset == 0.0:
            return points
        return [
            (float(x), float(y), float(z) + offset)
            for x, y, z in points
        ]

    def estimate_floor(
        self,
        xyz: tuple[float, float, float],
        *,
        current_floor: int,
    ) -> int:
        if self.vertical_reference == "floor_local":
            return int(current_floor)
        height = float(xyz[2])
        if not math.isfinite(height):
            raise ValueError("楼层估计高度必须为有限数")
        return max(0, int(round(height / self.floor_height)))


@dataclass(frozen=True)
class FloorTransitionSnapshot:
    entered_at: float
    enter_odom_stamp: float | None
    source_floor: int
    enter_estimated_floor: int
    target_floor: int | None
    enter_phase: str | None
    last_mismatch_odom_stamp: float | None
    alignment_floor: int | None = None
    alignment_sample_count: int = 0
    last_alignment_odom_stamp: float | None = None
    task_exit_at: float | None = None
    timeout_reported: bool = False
    skipped_cloud_count: int = 0
    first_skipped_at: float | None = None
    last_skipped_at: float | None = None
    first_skipped_cloud_stamp: float | None = None
    last_skipped_cloud_stamp: float | None = None
    last_skip_log_key: tuple[int, int, str | None] | None = None
    explicit_transition: bool = False
    confirmed_floor: int | None = None
    confirmation_odom_stamp: float | None = None


@dataclass(frozen=True)
class FloorGuardDecision:
    blocked: bool
    entered: bool = False
    exited: FloorTransitionSnapshot | None = None


class FloorTransitionRuntime:
    """拥有唯一换层守卫快照。"""

    def __init__(self) -> None:
        self._guard: FloorTransitionSnapshot | None = None

    @property
    def snapshot(self) -> FloorTransitionSnapshot | None:
        return self._guard

    @property
    def active(self) -> bool:
        return self._guard is not None

    def begin_explicit(
        self,
        *,
        source_floor: int,
        target_floor: int,
        phase: str,
        now: float,
        odom_stamp: float | None,
    ) -> FloorTransitionSnapshot | None:
        source_floor = int(source_floor)
        target_floor = int(target_floor)
        if source_floor == target_floor:
            return None
        existing = self._guard
        if existing is not None:
            if (
                existing.explicit_transition
                and existing.source_floor == source_floor
                and existing.target_floor == target_floor
            ):
                return existing
            raise RuntimeError("已有未完成的楼层过渡，拒绝开始新的电梯运输")
        stamp = _finite_stamp(odom_stamp)
        self._guard = FloorTransitionSnapshot(
            entered_at=float(now),
            enter_odom_stamp=stamp,
            source_floor=source_floor,
            enter_estimated_floor=source_floor,
            target_floor=target_floor,
            enter_phase=str(phase),
            last_mismatch_odom_stamp=stamp,
            explicit_transition=True,
        )
        return self._guard

    def confirm(self, target_floor: int, *, odom_stamp: float | None) -> FloorTransitionSnapshot:
        guard = self._guard
        target_floor = int(target_floor)
        if guard is None or not guard.explicit_transition or guard.target_floor != target_floor:
            raise RuntimeError("没有与目标楼层匹配的显式电梯过渡")
        self._guard = replace(
            guard,
            confirmed_floor=target_floor,
            confirmation_odom_stamp=_finite_stamp(odom_stamp),
            alignment_floor=target_floor,
            alignment_sample_count=0,
            last_alignment_odom_stamp=None,
        )
        return self._guard

    def evaluate(
        self,
        *,
        current_floor: int,
        estimated_floor: int,
        is_elevator_task: bool,
        task_target_floor: int | None,
        task_phase: str | None,
        odom_stamp: float | None,
        now: float,
    ) -> FloorGuardDecision:
        current_floor = int(current_floor)
        estimated_floor = int(estimated_floor)
        stamp = _finite_stamp(odom_stamp)
        guard = self._guard
        entered = False
        if guard is None and not (is_elevator_task and estimated_floor != current_floor):
            return FloorGuardDecision(False)
        if guard is None:
            guard = FloorTransitionSnapshot(
                entered_at=float(now),
                enter_odom_stamp=stamp,
                source_floor=current_floor,
                enter_estimated_floor=estimated_floor,
                target_floor=(int(task_target_floor) if task_target_floor is not None else None),
                enter_phase=task_phase,
                last_mismatch_odom_stamp=stamp,
            )
            self._guard = guard
            entered = True
        if not is_elevator_task and guard.task_exit_at is None:
            guard = replace(guard, task_exit_at=float(now))
        if guard.explicit_transition:
            if guard.confirmed_floor != current_floor:
                self._guard = guard
                return FloorGuardDecision(True, entered)
            fresh = stamp is not None and (
                guard.confirmation_odom_stamp is None or stamp > guard.confirmation_odom_stamp
            )
            if fresh and (guard.last_alignment_odom_stamp is None or stamp > guard.last_alignment_odom_stamp):
                guard = replace(
                    guard,
                    alignment_sample_count=guard.alignment_sample_count + 1,
                    last_alignment_odom_stamp=stamp,
                )
            if guard.alignment_sample_count < FLOOR_ALIGNMENT_REQUIRED_ODOM_SAMPLES:
                self._guard = guard
                return FloorGuardDecision(True, entered)
        elif estimated_floor != current_floor:
            guard = replace(
                guard,
                alignment_floor=None,
                alignment_sample_count=0,
                last_alignment_odom_stamp=None,
                last_mismatch_odom_stamp=(
                    stamp
                    if stamp is not None
                    and (guard.last_mismatch_odom_stamp is None or stamp > guard.last_mismatch_odom_stamp)
                    else guard.last_mismatch_odom_stamp
                ),
            )
            self._guard = guard
            return FloorGuardDecision(True, entered)
        else:
            freshness_floor = guard.last_mismatch_odom_stamp
            if freshness_floor is None:
                freshness_floor = guard.enter_odom_stamp
            fresh = stamp is not None and (freshness_floor is None or stamp > freshness_floor)
            if guard.alignment_floor != current_floor:
                guard = replace(
                    guard,
                    alignment_floor=current_floor,
                    alignment_sample_count=0,
                    last_alignment_odom_stamp=None,
                )
            if fresh and (guard.last_alignment_odom_stamp is None or stamp > guard.last_alignment_odom_stamp):
                guard = replace(
                    guard,
                    alignment_sample_count=guard.alignment_sample_count + 1,
                    last_alignment_odom_stamp=stamp,
                )
            if guard.alignment_sample_count < FLOOR_ALIGNMENT_REQUIRED_ODOM_SAMPLES:
                self._guard = guard
                return FloorGuardDecision(True, entered)
        self._guard = None
        return FloorGuardDecision(False, entered, guard)

    def record_skipped(
        self,
        *,
        cloud_stamp: float | None,
        current_floor: int,
        estimated_floor: int,
        task_phase: str | None,
        now: float,
    ) -> tuple[FloorTransitionSnapshot | None, bool]:
        guard = self._guard
        if guard is None:
            return None, False
        first = guard.first_skipped_at is None
        key = (int(current_floor), int(estimated_floor), task_phase)
        should_log = key != guard.last_skip_log_key
        guard = replace(
            guard,
            skipped_cloud_count=guard.skipped_cloud_count + 1,
            first_skipped_at=float(now) if first else guard.first_skipped_at,
            first_skipped_cloud_stamp=cloud_stamp if first else guard.first_skipped_cloud_stamp,
            last_skipped_at=float(now),
            last_skipped_cloud_stamp=cloud_stamp,
            last_skip_log_key=key if should_log else guard.last_skip_log_key,
        )
        self._guard = guard
        return guard, should_log

    def mark_timeout_reported(self) -> FloorTransitionSnapshot | None:
        guard = self._guard
        if guard is None or guard.timeout_reported:
            return guard
        self._guard = replace(guard, timeout_reported=True)
        return self._guard


def _finite_stamp(value: float | None) -> float | None:
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def estimate_floor_height_from_cloud(
    cloud_points: Iterable[tuple[float, float, float]],
    current_floor: int,
    nominal_height: float = 2.6,
    min_valid_height: float = 2.5,
    max_valid_height: float = 2.7,
) -> float | None:
    """从下试点云中检测穿透空洞打在下层地坪的回波，估计两层间的实际层高。

    若测得高差在 [min_valid_height, max_valid_height] 物理门限内（默认 [2.5m, 2.7m]）
    则返回实测值；若超出门限（例如仅探测到半层楼梯平台 1.3m 或其它噪声）
    则返回 None（由调用方安全回退至标称层高）。
    """
    if int(current_floor) < 1 or not cloud_points:
        return None

    z_values = [float(p[2]) for p in cloud_points if len(p) > 2 and math.isfinite(p[2])]
    if len(z_values) < 50:
        return None

    bin_size = 0.05
    bins: dict[int, int] = {}
    for z in z_values:
        idx = int(math.floor(z / bin_size))
        bins[idx] = bins.get(idx, 0) + 1

    # 寻找水平聚集平面峰值
    min_points = 20
    peaks = [
        (idx * bin_size + bin_size / 2.0, count)
        for idx, count in bins.items()
        if count >= min_points
    ]
    if len(peaks) < 2:
        return None

    # 当前层的主要水平地坪（通常是最高且密度大的一组）
    z_high = max(peaks, key=lambda x: x[1])[0]

    # 寻找比当前层地表显著更低（至少低 1.8m 以上）的穿透下层候选
    low_candidates = [p for p in peaks if (z_high - p[0]) >= 1.8]
    if not low_candidates:
        return None

    best_low = max(low_candidates, key=lambda x: x[1])
    diff = float(z_high - best_low[0])

    # 严格校验物理合理性门限 [min_valid_height, max_valid_height]
    if min_valid_height <= diff <= max_valid_height:
        return diff
    return None

