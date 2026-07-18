"""把单帧危险源候选融合成可输出的稳定目标。

``RedSphereDetector`` 每帧产生带置信度、拟合误差和 world 坐标的候选；本模块
按空间半径关联轨迹，并同时要求命中次数、时间窗口、位置方差、置信度和球拟合
误差通过阈值。结果写出前再做一次按质量排序的 NMS，降低同一红球产生多轨迹
时的重复报告概率。
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Iterable, Sequence

import numpy as np
from .time_utils import get_time


TENTATIVE = "Tentative"
CONFIRMED = "Confirmed"
REJECTED = "Rejected"


@dataclass(frozen=True)
class DetectionCandidate:
    """单帧检测候选。

    position 使用 Gazebo world/odom 数值坐标，confidence 为 0 到 1。
    """

    position: tuple[float, float, float]
    confidence: float
    rmse: float = 0.0
    radius: float | None = None
    stamp: float | None = None

    @classmethod
    def from_sequence(
        cls,
        position: Sequence[float],
        confidence: float = 1.0,
        rmse: float = 0.0,
        radius: float | None = None,
        stamp: float | None = None,
    ) -> "DetectionCandidate":
        if len(position) != 3:
            raise ValueError(f"检测候选位置必须是三维坐标: {position}")
        return cls(
            position=(float(position[0]), float(position[1]), float(position[2])),
            confidence=float(confidence),
            rmse=float(rmse),
            radius=radius,
            stamp=stamp,
        )


@dataclass
class Track:
    """一个空间目标的观测历史和生命周期状态。

    ``TENTATIVE`` 尚未满足确认门槛，``CONFIRMED`` 可参与最终输出，长时间未确认
    的轨迹会成为 ``REJECTED`` 且不再参与后续关联。
    """
    track_id: int
    mean_position: np.ndarray
    first_seen: float
    last_seen: float
    hit_times: list[float] = field(default_factory=list)
    positions: list[np.ndarray] = field(default_factory=list)
    confidences: list[float] = field(default_factory=list)
    rmses: list[float] = field(default_factory=list)
    status: str = TENTATIVE

    def update(self, candidate: DetectionCandidate, now: float) -> None:
        """追加一次命中，并用全部历史观测均值更新轨迹位置。"""
        point = np.array(candidate.position, dtype=float)
        self.positions.append(point)
        self.confidences.append(float(candidate.confidence))
        self.rmses.append(float(candidate.rmse))
        self.hit_times.append(now)
        self.last_seen = now
        self.mean_position = np.mean(np.vstack(self.positions), axis=0)

    def recent_hit_count(self, now: float, window: float) -> int:
        cutoff = now - window
        return sum(1 for hit_time in self.hit_times if hit_time >= cutoff)

    def position_std(self) -> float:
        if len(self.positions) <= 1:
            return 0.0
        values = np.vstack(self.positions)
        return float(np.max(np.std(values, axis=0)))

    def average_confidence(self) -> float:
        if not self.confidences:
            return 0.0
        return float(sum(self.confidences) / len(self.confidences))

    def average_rmse(self) -> float:
        if not self.rmses:
            return 0.0
        return float(sum(self.rmses) / len(self.rmses))


class DetectionTracker:
    """管理全部目标轨迹、关联新候选并输出空间去重后的确认位置。"""

    def __init__(
        self,
        *,
        association_radius: float = 0.45,
        confirmation_window: float = 10.0,
        min_hits: int = 4,
        max_position_std: float = 0.25,
        min_confidence: float = 0.75,
        max_average_rmse: float = 0.04,
        nms_radius: float = 0.9,
        stale_after: float = 30.0,
    ):
        self.association_radius = float(association_radius)
        self.confirmation_window = float(confirmation_window)
        self.min_hits = int(min_hits)
        self.max_position_std = float(max_position_std)
        self.min_confidence = float(min_confidence)
        self.max_average_rmse = float(max_average_rmse)
        self.nms_radius = float(nms_radius)
        self.stale_after = float(stale_after)
        self._tracks: list[Track] = []
        self._next_track_id = 1

    @property
    def tracks(self) -> list[Track]:
        return list(self._tracks)

    def update(self, candidates: Iterable[DetectionCandidate], now: float | None = None) -> None:
        """依次关联一帧候选，再统一刷新所有轨迹状态。"""
        timestamp = float(now if now is not None else get_time())
        for candidate in candidates:
            self._update_one(candidate, timestamp)
        self._refresh_status(timestamp)

    def confirmed_positions(self, now: float | None = None) -> list[list[float]]:
        """返回质量优先、经 NMS 去重的确认目标坐标。

        NMS 不删除内部轨迹，只决定本次输出集合，因此诊断仍能看到被近邻高质量
        轨迹抑制的观测历史。
        """
        timestamp = float(now if now is not None else get_time())
        self._refresh_status(timestamp)
        confirmed = [track for track in self._tracks if track.status == CONFIRMED]
        confirmed.sort(
            key=lambda item: (item.average_confidence(), len(item.hit_times)),
            reverse=True,
        )

        selected: list[Track] = []
        for track in confirmed:
            if all(_distance(track.mean_position, other.mean_position) > self.nms_radius for other in selected):
                selected.append(track)
        return [track.mean_position.astype(float).tolist() for track in selected]

    def _update_one(self, candidate: DetectionCandidate, now: float) -> None:
        matched = self._nearest_track(candidate.position)
        if matched is None:
            point = np.array(candidate.position, dtype=float)
            track = Track(
                track_id=self._next_track_id,
                mean_position=point,
                first_seen=now,
                last_seen=now,
            )
            self._next_track_id += 1
            track.update(candidate, now)
            self._tracks.append(track)
            return
        matched.update(candidate, now)

    def _nearest_track(self, position: Sequence[float]) -> Track | None:
        point = np.array(position, dtype=float)
        nearest: Track | None = None
        nearest_distance = math.inf
        for track in self._tracks:
            if track.status == REJECTED:
                continue
            distance = _distance(point, track.mean_position)
            if distance < nearest_distance:
                nearest = track
                nearest_distance = distance
        if nearest is None or nearest_distance > self.association_radius:
            return None
        return nearest

    def _refresh_status(self, now: float) -> None:
        for track in self._tracks:
            if track.status == REJECTED:
                continue
            if now - track.last_seen > self.stale_after and track.status != CONFIRMED:
                track.status = REJECTED
                continue
            if (
                track.recent_hit_count(now, self.confirmation_window) >= self.min_hits
                and track.position_std() <= self.max_position_std
                and track.average_confidence() >= self.min_confidence
                and track.average_rmse() <= self.max_average_rmse
            ):
                track.status = CONFIRMED


def _distance(a: np.ndarray, b: np.ndarray) -> float:
    diff = a - b
    return float(np.linalg.norm(diff))
