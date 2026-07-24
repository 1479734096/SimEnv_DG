"""使用传感器派生电梯门板地标记录平移漂移候选。

本模块只计算和返回诊断结果，不发布控制命令、不修改定位或地图。调用方必须
显式开启，并在任务结束后使用独立真值评估候选是否具备校正价值。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from statistics import median
from typing import Iterable

from .rgbd_door_detector import DoorPanelObservation


def _angle_difference(left: float, right: float) -> float:
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


@dataclass(frozen=True)
class LandmarkDriftConfig:
    enabled: bool = False
    minimum_width: float = 0.65
    maximum_width: float = 1.80
    maximum_distance: float = 4.0
    maximum_yaw_difference: float = 0.35
    maximum_width_difference: float = 0.25
    required_confirmations: int = 3
    maximum_confirmation_gap: float = 0.4
    maximum_residual_spread: float = 0.15

    def __post_init__(self) -> None:
        finite_positive = (
            self.minimum_width,
            self.maximum_width,
            self.maximum_distance,
            self.maximum_yaw_difference,
            self.maximum_width_difference,
            self.maximum_confirmation_gap,
            self.maximum_residual_spread,
        )
        if (
            not all(math.isfinite(value) and value > 0.0 for value in finite_positive)
            or self.minimum_width >= self.maximum_width
            or self.maximum_yaw_difference > math.pi
            or self.required_confirmations < 2
        ):
            raise ValueError("地标漂移诊断参数必须为有限正值且范围有效")


@dataclass(frozen=True)
class ElevatorLandmarkAnchor:
    stamp: float
    floor: int
    world_xy: tuple[float, float]
    outward_yaw: float
    width: float
    source: str


@dataclass(frozen=True)
class LandmarkDriftEvaluation:
    status: str
    stamp: float | None = None
    sequence_start_stamp: float | None = None
    candidate_count: int = 0
    residual_xy: tuple[float, float] | None = None
    residual_norm: float | None = None
    normal_residual: float | None = None
    tangent_residual: float | None = None
    confirmed: bool = False
    confirmations: int = 0
    observation: DoorPanelObservation | None = None


class LandmarkDriftMonitor:
    """冻结首个电梯地标并对后续关闭门板做三帧只读确认。"""

    def __init__(self, config: LandmarkDriftConfig):
        self.config = config
        self.anchor: ElevatorLandmarkAnchor | None = None
        self._sequence: list[
            tuple[float, tuple[float, float], DoorPanelObservation]
        ] = []
        self._last_stamp: float | None = None

    def freeze_anchor(
        self,
        *,
        stamp: float | None,
        floor: int,
        world_xy: tuple[float, float],
        outward_yaw: float,
        width: float,
        source: str,
    ) -> bool:
        """只冻结第一次有效的 0 层地标，后续语义几何刷新不得覆盖。"""
        values = (
            stamp,
            world_xy[0],
            world_xy[1],
            outward_yaw,
            width,
        )
        if (
            not self.config.enabled
            or self.anchor is not None
            or int(floor) != 0
            or stamp is None
            or not all(math.isfinite(float(value)) for value in values)
            or float(width) <= 0.0
        ):
            return False
        self.anchor = ElevatorLandmarkAnchor(
            stamp=float(stamp),
            floor=int(floor),
            world_xy=(float(world_xy[0]), float(world_xy[1])),
            outward_yaw=float(outward_yaw),
            width=float(width),
            source=str(source),
        )
        return True

    def reset_sequence(self) -> None:
        self._sequence = []
        self._last_stamp = None

    def observe(
        self,
        observations: Iterable[DoorPanelObservation],
        *,
        stamp: float | None,
        floor: int,
        ordinary_exploration: bool,
        elevator_count: int,
    ) -> LandmarkDriftEvaluation:
        """返回当前帧诊断；任何质量缺失都只清确认序列。"""
        if not self.config.enabled:
            return LandmarkDriftEvaluation(status="disabled")
        anchor = self.anchor
        if anchor is None:
            self.reset_sequence()
            return LandmarkDriftEvaluation(status="anchor_unavailable")
        if (
            stamp is None
            or not math.isfinite(float(stamp))
            or int(floor) <= anchor.floor
            or not ordinary_exploration
            or int(elevator_count) != 1
        ):
            self.reset_sequence()
            return LandmarkDriftEvaluation(status="context_rejected", stamp=stamp)
        current_stamp = float(stamp)
        if self._last_stamp is not None and current_stamp <= self._last_stamp:
            self.reset_sequence()
            return LandmarkDriftEvaluation(status="nonmonotonic_stamp", stamp=current_stamp)

        candidates = [
            observation
            for observation in observations
            if self._eligible(observation, anchor)
        ]
        if not candidates:
            self.reset_sequence()
            return LandmarkDriftEvaluation(
                status="no_eligible_observation",
                stamp=current_stamp,
            )

        # 门板检测器可能同时给出左右门板。诊断阶段使用离冻结点最近者，但完整
        # 候选数量会进入日志；离线门槛必须据此排除门板身份切换。
        candidate = min(
            candidates,
            key=lambda item: math.hypot(
                float(item.world_xy[0]) - anchor.world_xy[0],
                float(item.world_xy[1]) - anchor.world_xy[1],
            ),
        )
        residual = (
            float(candidate.world_xy[0]) - anchor.world_xy[0],
            float(candidate.world_xy[1]) - anchor.world_xy[1],
        )
        if (
            self._last_stamp is None
            or current_stamp - self._last_stamp > self.config.maximum_confirmation_gap
        ):
            self._sequence = []
        self._last_stamp = current_stamp
        self._sequence.append((current_stamp, residual, candidate))
        self._sequence = self._sequence[-self.config.required_confirmations :]

        spread = self._maximum_pairwise_spread(self._sequence)
        if spread > self.config.maximum_residual_spread:
            self._sequence = [(current_stamp, residual, candidate)]

        residuals = [item[1] for item in self._sequence]
        median_residual = (
            median(item[0] for item in residuals),
            median(item[1] for item in residuals),
        )
        normal = (
            math.cos(anchor.outward_yaw),
            math.sin(anchor.outward_yaw),
        )
        tangent = (-normal[1], normal[0])
        confirmations = len(self._sequence)
        return LandmarkDriftEvaluation(
            status="confirmed" if confirmations >= self.config.required_confirmations else "candidate",
            stamp=current_stamp,
            sequence_start_stamp=self._sequence[0][0],
            candidate_count=len(candidates),
            residual_xy=median_residual,
            residual_norm=math.hypot(*median_residual),
            normal_residual=(
                median_residual[0] * normal[0] + median_residual[1] * normal[1]
            ),
            tangent_residual=(
                median_residual[0] * tangent[0] + median_residual[1] * tangent[1]
            ),
            confirmed=confirmations >= self.config.required_confirmations,
            confirmations=confirmations,
            observation=candidate,
        )

    def _eligible(
        self,
        observation: DoorPanelObservation,
        anchor: ElevatorLandmarkAnchor,
    ) -> bool:
        values = (
            observation.distance,
            observation.world_xy[0],
            observation.world_xy[1],
            observation.outward_yaw,
            observation.width,
            observation.plane_rmse,
        )
        return (
            str(observation.panel_state) == "closed"
            and all(math.isfinite(float(value)) for value in values)
            and self.config.minimum_width
            <= float(observation.width)
            <= self.config.maximum_width
            and float(observation.distance) <= self.config.maximum_distance
            and _angle_difference(
                float(observation.outward_yaw),
                anchor.outward_yaw,
            )
            <= self.config.maximum_yaw_difference
            and abs(float(observation.width) - anchor.width)
            <= self.config.maximum_width_difference
        )

    @staticmethod
    def _maximum_pairwise_spread(
        sequence: list[tuple[float, tuple[float, float], DoorPanelObservation]],
    ) -> float:
        maximum = 0.0
        for index, (_, left, _) in enumerate(sequence):
            for _, right, _ in sequence[index + 1 :]:
                maximum = max(
                    maximum,
                    math.hypot(left[0] - right[0], left[1] - right[1]),
                )
        return maximum
