"""把单帧危险源候选融合成可输出的稳定目标。

``RedSphereDetector`` 每帧产生带置信度、拟合误差和 world 坐标的候选；本模块
按空间半径关联轨迹，并同时要求命中次数、时间窗口、位置方差、置信度和球拟合
误差通过阈值。结果写出前再做一次按质量排序的 NMS，降低同一红球产生多轨迹
时的重复报告概率。
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Callable, Iterable, Sequence

import numpy as np
from .time_utils import get_time


TENTATIVE = "Tentative"
CONFIRMED = "Confirmed"
REJECTED = "Rejected"


@dataclass(frozen=True)
class DetectionCandidate:
    """单帧检测候选。

    position 使用第 2 层定位建立的逻辑 world 数值坐标，confidence 为 0 到 1。
    """

    position: tuple[float, float, float]
    confidence: float
    rmse: float = 0.0
    radius: float | None = None
    stamp: float | None = None
    distance: float | None = None

    def __post_init__(self) -> None:
        if len(self.position) != 3:
            raise ValueError("检测候选位置必须是三维坐标")
        position = tuple(float(value) for value in self.position)
        confidence = float(self.confidence)
        rmse = float(self.rmse)
        radius = None if self.radius is None else float(self.radius)
        stamp = None if self.stamp is None else float(self.stamp)
        distance = None if self.distance is None else float(self.distance)
        if (
            not all(math.isfinite(value) for value in position)
            or not math.isfinite(confidence)
            or not 0.0 <= confidence <= 1.0
            or not math.isfinite(rmse)
            or rmse < 0.0
            or radius is not None
            and (not math.isfinite(radius) or radius <= 0.0)
            or stamp is not None
            and (not math.isfinite(stamp) or stamp < 0.0)
            or distance is not None
            and (not math.isfinite(distance) or distance <= 0.0)
        ):
            raise ValueError("检测候选的坐标、质量、半径、时间戳或距离无效")
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "confidence", confidence)
        object.__setattr__(self, "rmse", rmse)
        object.__setattr__(self, "radius", radius)
        object.__setattr__(self, "stamp", stamp)
        object.__setattr__(self, "distance", distance)

    @classmethod
    def from_sequence(
        cls,
        position: Sequence[float],
        confidence: float = 1.0,
        rmse: float = 0.0,
        radius: float | None = None,
        stamp: float | None = None,
        distance: float | None = None,
    ) -> "DetectionCandidate":
        if len(position) != 3:
            raise ValueError(f"检测候选位置必须是三维坐标: {position}")
        return cls(
            position=(float(position[0]), float(position[1]), float(position[2])),
            confidence=float(confidence),
            rmse=float(rmse),
            radius=None if radius is None else float(radius),
            stamp=None if stamp is None else float(stamp),
            distance=None if distance is None else float(distance),
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
    observation_stamps: list[float] = field(default_factory=list)
    positions: list[np.ndarray] = field(default_factory=list)
    confidences: list[float] = field(default_factory=list)
    rmses: list[float] = field(default_factory=list)
    distances: list[float] = field(default_factory=list)
    status: str = TENTATIVE
    inspected: bool = False

    def update(self, candidate: DetectionCandidate, now: float) -> None:
        """追加一次命中，并用距离与误差加权更新轨迹质心位置。"""
        point = np.array(candidate.position, dtype=float)
        self.positions.append(point)
        self.confidences.append(float(candidate.confidence))
        self.rmses.append(float(candidate.rmse))
        obs_dist = float(candidate.distance) if candidate.distance is not None else 1.0
        self.distances.append(obs_dist)
        self.hit_times.append(now)
        self.observation_stamps.append(
            float(candidate.stamp) if candidate.stamp is not None else float(now)
        )
        self.last_seen = now
        self.mean_position = self._calculate_weighted_position()

    def translate_planar(self, dx: float, dy: float) -> None:
        """将同一定位连续段的轨迹观测整体平移到新坐标系。"""
        offset = np.array((float(dx), float(dy)), dtype=float)
        self.mean_position = np.array(self.mean_position, dtype=float, copy=True)
        self.mean_position[:2] += offset
        translated_positions: list[np.ndarray] = []
        for position in self.positions:
            translated = np.array(position, dtype=float, copy=True)
            translated[:2] += offset
            translated_positions.append(translated)
        self.positions = translated_positions

    def transform_planar(self, dx: float, dy: float, yaw: float) -> None:
        """将轨迹观测整体变换到新的世界系平面坐标。"""
        cosine = math.cos(float(yaw))
        sine = math.sin(float(yaw))
        rotation = np.asarray(((cosine, -sine), (sine, cosine)), dtype=float)
        offset = np.asarray((float(dx), float(dy)), dtype=float)
        if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(offset)):
            raise ValueError("检测轨迹 SE(2) 变换量必须为有限数")

        def transform(position: np.ndarray) -> np.ndarray:
            result = np.array(position, dtype=float, copy=True)
            result[:2] = rotation @ result[:2] + offset
            return result

        self.mean_position = transform(self.mean_position)
        self.positions = [transform(position) for position in self.positions]

    def _effective_indices(self) -> list[int]:
        """获取用于质心和方差计算的最优近场子集索引。

        当存在近距离高质量观测（<= 1.8m）时，彻底剔除较远的历史粗检点（> min_dist + 0.6m），
        并仅采信最新的若干近场帧，避免远距离历史误差拉偏近场收敛位置。
        """
        if not self.positions:
            return []
        min_dist = min(self.distances) if self.distances else 1.0
        if min_dist <= 1.8:
            cutoff = min_dist + 0.6
            indices = [i for i, d in enumerate(self.distances) if d <= cutoff]
            if len(indices) > 6:
                indices = indices[-6:]
            return indices
        return list(range(len(self.positions)))

    def _calculate_weighted_position(self) -> np.ndarray:
        if not self.positions:
            return self.mean_position
        indices = self._effective_indices()
        weights = []
        for idx in indices:
            conf = self.confidences[idx]
            rmse = self.rmses[idx]
            dist = self.distances[idx]
            safe_dist = max(dist, 0.4)
            safe_rmse = max(rmse, 0.005)
            safe_conf = max(conf, 0.01)
            w = safe_conf / ((safe_dist ** 2) * safe_rmse)
            weights.append(w)
        weights_arr = np.array(weights, dtype=float)
        total_w = float(np.sum(weights_arr))
        selected_positions = [self.positions[i] for i in indices]
        if total_w > 0.0 and np.isfinite(total_w):
            pts = np.vstack(selected_positions)
            return np.sum(pts * weights_arr[:, np.newaxis], axis=0) / total_w
        return np.mean(np.vstack(selected_positions), axis=0)

    def recent_hit_count(self, now: float, window: float) -> int:
        cutoff = now - window
        return sum(1 for hit_time in self.hit_times if hit_time >= cutoff)

    def position_std(self) -> float:
        indices = self._effective_indices()
        if len(indices) <= 1:
            return 0.0
        values = np.vstack([self.positions[i] for i in indices])
        return float(np.max(np.std(values, axis=0)))

    def average_confidence(self) -> float:
        if not self.confidences:
            return 0.0
        return float(sum(self.confidences) / len(self.confidences))

    def average_rmse(self) -> float:
        if not self.rmses:
            return 0.0
        return float(sum(self.rmses) / len(self.rmses))


@dataclass(frozen=True)
class InspectionTarget:
    """供任务仲裁使用的危险源核验目标快照。"""

    track_id: int
    position: tuple[float, float, float]
    confidence: float
    last_seen: float


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
        nms_radius: float = 0.50,
        stale_after: float = 30.0,
        obstacle_min_hits: int = 2,
        obstacle_min_confidence: float = 0.90,
        obstacle_max_average_rmse: float = 0.025,
        obstacle_max_age: float = 0.75,
        obstacle_min_observation_span: float = 0.08,
        inspection_max_attempts: int = 3,
        tier1_max_rmse: float = 0.005,
        tier1_min_hits: int = 2,
        tier1_min_confidence: float = 0.70,
        tier2_max_rmse: float = 0.015,
        tier2_min_hits: int = 2,
        tier2_min_confidence: float = 0.70,
    ):
        self.association_radius = float(association_radius)
        self.confirmation_window = float(confirmation_window)
        self.min_hits = int(min_hits)
        self.max_position_std = float(max_position_std)
        self.min_confidence = float(min_confidence)
        self.max_average_rmse = float(max_average_rmse)
        self.nms_radius = float(nms_radius)
        self.stale_after = float(stale_after)
        self.obstacle_min_hits = int(obstacle_min_hits)
        self.obstacle_min_confidence = float(obstacle_min_confidence)
        self.obstacle_max_average_rmse = float(obstacle_max_average_rmse)
        self.obstacle_max_age = float(obstacle_max_age)
        self.obstacle_min_observation_span = float(
            obstacle_min_observation_span
        )
        self.tier1_max_rmse = float(tier1_max_rmse)
        self.tier1_min_hits = int(tier1_min_hits)
        self.tier1_min_confidence = float(tier1_min_confidence)
        self.tier2_max_rmse = float(tier2_max_rmse)
        self.tier2_min_hits = int(tier2_min_hits)
        self.tier2_min_confidence = float(tier2_min_confidence)
        if (
            isinstance(inspection_max_attempts, bool)
            or not isinstance(inspection_max_attempts, int)
            or inspection_max_attempts <= 0
        ):
            raise ValueError("危险源核验尝试上限必须为正整数")
        self.inspection_max_attempts = inspection_max_attempts
        finite_nonnegative = (
            self.max_position_std,
            self.max_average_rmse,
            self.nms_radius,
            self.obstacle_max_average_rmse,
            self.obstacle_min_observation_span,
            self.tier1_max_rmse,
            self.tier2_max_rmse,
        )
        if (
            not math.isfinite(self.association_radius)
            or self.association_radius <= 0.0
            or not math.isfinite(self.confirmation_window)
            or self.confirmation_window <= 0.0
            or self.min_hits <= 0
            or self.obstacle_min_hits <= 0
            or self.tier1_min_hits <= 0
            or self.tier2_min_hits <= 0
            or not all(
                math.isfinite(value) and value >= 0.0
                for value in finite_nonnegative
            )
            or not math.isfinite(self.min_confidence)
            or not 0.0 <= self.min_confidence <= 1.0
            or not math.isfinite(self.obstacle_min_confidence)
            or not 0.0 <= self.obstacle_min_confidence <= 1.0
            or not math.isfinite(self.tier1_min_confidence)
            or not 0.0 <= self.tier1_min_confidence <= 1.0
            or not math.isfinite(self.tier2_min_confidence)
            or not 0.0 <= self.tier2_min_confidence <= 1.0
            or not math.isfinite(self.stale_after)
            or self.stale_after <= 0.0
            or not math.isfinite(self.obstacle_max_age)
            or self.obstacle_max_age <= 0.0
        ):
            raise ValueError("检测跟踪关联、确认、质量或过期参数无效")
        self._tracks: list[Track] = []
        self._next_track_id = 1
        self._last_update_time: float | None = None
        self._inspection_in_progress: set[int] = set()
        self._inspection_retry_after: dict[int, float] = {}
        self._inspection_attempts: dict[int, int] = {}

    @property
    def tracks(self) -> list[Track]:
        return list(self._tracks)

    def update(self, candidates: Iterable[DetectionCandidate], now: float | None = None) -> None:
        """依次关联一帧候选，再统一刷新所有轨迹状态。"""
        timestamp = float(now if now is not None else get_time())
        if not math.isfinite(timestamp):
            raise ValueError("检测跟踪时间必须是有限数")
        if self._last_update_time is not None and timestamp < self._last_update_time:
            raise ValueError("检测跟踪时间不能倒退")
        self._last_update_time = timestamp
        updated_track_ids: set[int] = set()
        for candidate in candidates:
            if not isinstance(candidate, DetectionCandidate):
                raise TypeError("检测跟踪只接受 DetectionCandidate")
            updated_track_ids.add(
                self._update_one(candidate, timestamp, updated_track_ids)
            )
        self._refresh_status(timestamp)

    def confirmed_tracks(self, now: float | None = None) -> list[Track]:
        """返回质量优先、经 NMS 去重后的确认轨迹。

        NMS 不删除内部轨迹，只决定本次输出集合，因此诊断仍能看到被近邻高质量
        轨迹抑制的观测历史。
        """
        timestamp = float(now if now is not None else get_time())
        if not math.isfinite(timestamp):
            raise ValueError("确认目标查询时间必须是有限数")
        if self._last_update_time is not None and timestamp < self._last_update_time:
            raise ValueError("确认目标查询时间不能早于最后观测")
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
        return selected

    def obstacle_tracks(self, now: float | None = None) -> list[Track]:
        """返回可写入语义 OCC 的确认轨迹与高质量双帧预保护轨迹。

        未确认轨迹只有在两帧独立观测、拟合残差和置信度都明显强于最终输出
        门槛时才临时阻挡路径；停止刷新后会在短年龄上限内自动撤销，不能把
        单帧误检或旧候选永久写入地图。
        """
        timestamp = float(now if now is not None else get_time())
        if not math.isfinite(timestamp):
            raise ValueError("危险源障碍查询时间必须是有限数")
        if self._last_update_time is not None and timestamp < self._last_update_time:
            raise ValueError("危险源障碍查询时间不能早于最后观测")
        self._refresh_status(timestamp)

        eligible: list[Track] = []
        for track in self._tracks:
            if track.status == CONFIRMED:
                eligible.append(track)
            elif (
                track.status == TENTATIVE
                and self._is_protective_tentative(track, timestamp)
            ):
                eligible.append(track)
        # 确认轨迹优先于临时预保护轨迹，避免高置信度的短期候选抑制最终结果。
        eligible.sort(
            key=lambda item: (
                item.status == CONFIRMED,
                item.average_confidence(),
                len(item.hit_times),
            ),
            reverse=True,
        )

        selected: list[Track] = []
        for track in eligible:
            if all(
                _distance(track.mean_position, other.mean_position) > self.nms_radius
                for other in selected
            ):
                selected.append(track)
        return selected

    def _is_protective_tentative(self, track: Track, now: float) -> bool:
        """判定未确认轨迹是否足以在短时间内保护机器人不压过目标。"""
        if now - track.last_seen > self.obstacle_max_age:
            return False
        recent_hits = [
            stamp for stamp in track.hit_times
            if stamp >= now - self.confirmation_window
        ]
        if len(recent_hits) < self.obstacle_min_hits:
            return False
        if (
            recent_hits[-1] - recent_hits[0]
            < self.obstacle_min_observation_span
        ):
            return False
        return (
            track.average_confidence() >= self.obstacle_min_confidence
            and track.average_rmse() <= self.obstacle_max_average_rmse
            and track.position_std() <= self.max_position_std
        )

    def confirmed_positions(self, now: float | None = None) -> list[list[float]]:
        """返回质量优先、经 NMS 去重的确认目标坐标。"""
        return [
            track.mean_position.astype(float).tolist()
            for track in self.confirmed_tracks(now=now)
        ]

    def translate_planar(self, dx: float, dy: float) -> int:
        """原子平移全部历史轨迹，保持纠偏后的输出与后续观测同坐标。"""
        offset_x = float(dx)
        offset_y = float(dy)
        if not math.isfinite(offset_x) or not math.isfinite(offset_y):
            raise ValueError("检测轨迹平移量必须为有限数")
        for track in self._tracks:
            track.translate_planar(offset_x, offset_y)
        return len(self._tracks)

    def transform_planar(self, dx: float, dy: float, yaw: float) -> int:
        """原子变换全部历史轨迹，保持输出与后续观测同坐标。"""
        values = (float(dx), float(dy), float(yaw))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("检测轨迹 SE(2) 变换量必须为有限数")
        for track in self._tracks:
            track.transform_planar(*values)
        return len(self._tracks)

    def get_pending_inspection_target(
        self,
        robot_xy: tuple[float, float],
        min_observe_dist: float = 2.0,
        min_confidence: float = 0.70,
    ) -> tuple[float, float] | None:
        """获取尚未走近核验的高质量远距离目标世界平面坐标。"""
        candidates = []
        for track in self._tracks:
            if not track.inspected and track.status in (TENTATIVE, CONFIRMED):
                dist = math.hypot(
                    float(track.mean_position[0]) - float(robot_xy[0]),
                    float(track.mean_position[1]) - float(robot_xy[1]),
                )
                if dist >= min_observe_dist and track.average_confidence() >= min_confidence:
                    candidates.append(
                        (dist, (float(track.mean_position[0]), float(track.mean_position[1])))
                    )
        if candidates:
            candidates.sort(key=lambda item: item[0])
            return candidates[0][1]
        return None

    def select_pending_inspection_target(
        self,
        robot_xy: tuple[float, float],
        *,
        now: float | None = None,
        min_observe_dist: float = 2.0,
        min_confidence: float = 0.70,
        floor_filter: Callable[[Track], bool] | None = None,
    ) -> InspectionTarget | None:
        """选择可抢占探索路径的当前楼层确认危险源。

        普通 ``get_pending_inspection_target`` 保留旧语义供兼容调用；主动抢占
        必须额外要求确认、未过期、未在核验中且通过楼层过滤，避免旧楼层或
        单帧候选反复打断探索。
        """
        timestamp = float(now if now is not None else get_time())
        if not math.isfinite(timestamp):
            raise ValueError("危险源核验查询时间必须是有限数")
        if self._last_update_time is not None and timestamp < self._last_update_time:
            raise ValueError("危险源核验查询时间不能早于最后观测")
        self._refresh_status(timestamp)

        candidates: list[tuple[float, float, Track]] = []
        for track in self._tracks:
            if (
                track.status != CONFIRMED
                or track.inspected
                or track.track_id in self._inspection_in_progress
                or timestamp < self._inspection_retry_after.get(track.track_id, -math.inf)
                or timestamp - track.last_seen > self.stale_after
                or self._inspection_attempts.get(track.track_id, 0)
                >= self.inspection_max_attempts
            ):
                continue
            if floor_filter is not None and not floor_filter(track):
                continue
            distance = math.hypot(
                float(track.mean_position[0]) - float(robot_xy[0]),
                float(track.mean_position[1]) - float(robot_xy[1]),
            )
            confidence = track.average_confidence()
            if distance < float(min_observe_dist) or confidence < float(min_confidence):
                continue
            candidates.append((distance, -confidence, track))

        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[0], item[1], item[2].track_id))
        _, _, track = candidates[0]
        return InspectionTarget(
            track_id=int(track.track_id),
            position=tuple(float(value) for value in track.mean_position),
            confidence=float(track.average_confidence()),
            last_seen=float(track.last_seen),
        )

    def inspection_attempt_count(self, track_id: int) -> int:
        """返回目标已消耗的核验尝试数，供任务日志说明失败降级。"""
        return self._inspection_attempts.get(int(track_id), 0)

    def mark_inspection_started(self, track_id: int) -> bool:
        """记录已成功装载接近路径，但不把目标标记为已核验。"""
        parsed_id = int(track_id)
        track = next((item for item in self._tracks if item.track_id == parsed_id), None)
        if (
            track is None or track.inspected
            or parsed_id in self._inspection_in_progress
            or self._inspection_attempts.get(parsed_id, 0) >= self.inspection_max_attempts
        ):
            return False
        self._inspection_attempts[parsed_id] = self._inspection_attempts.get(parsed_id, 0) + 1
        self._inspection_in_progress.add(parsed_id)
        return True

    def mark_inspection_verified(
        self,
        track_id: int,
        *,
        floor_filter: Callable[[Track], bool] | None = None,
    ) -> bool:
        """在近场证据成立后标记实际目标轨迹已核验。"""
        parsed_id = int(track_id)
        target = next((item for item in self._tracks if item.track_id == parsed_id), None)
        if target is None:
            self._inspection_in_progress.discard(parsed_id)
            return False
        if floor_filter is not None and not floor_filter(target):
            self._inspection_in_progress.discard(parsed_id)
            return False
        target.inspected = True
        self._inspection_in_progress.discard(parsed_id)
        self._inspection_retry_after.pop(parsed_id, None)
        return True

    def mark_inspection_failed(
        self,
        track_id: int,
        *,
        cooldown: float = 5.0,
        now: float | None = None,
    ) -> None:
        """记录规划或执行失败；冷却后可有限重试，不撤销正式确认资格。"""
        parsed_id = int(track_id)
        timestamp = float(now if now is not None else get_time())
        if not math.isfinite(timestamp) or not math.isfinite(float(cooldown)):
            raise ValueError("危险源核验失败冷却参数必须为有限数")
        if parsed_id not in self._inspection_in_progress:
            self._inspection_attempts[parsed_id] = self._inspection_attempts.get(parsed_id, 0) + 1
        self._inspection_in_progress.discard(parsed_id)
        self._inspection_retry_after[parsed_id] = timestamp + max(0.0, float(cooldown))

    def has_verification_evidence(
        self,
        track_id: int,
        *,
        since: float,
        now: float | None = None,
        min_hits: int = 2,
        min_observation_span: float = 0.08,
    ) -> bool:
        """确认目标在核验阶段仍有足够的新鲜多帧观测。"""
        timestamp = float(now if now is not None else get_time())
        if (
            not math.isfinite(timestamp) or not math.isfinite(float(since))
            or not math.isfinite(float(min_observation_span))
            or float(min_observation_span) <= 0.0
            or timestamp < float(since)
        ):
            return False
        track = next((item for item in self._tracks if item.track_id == int(track_id)), None)
        if track is None or track.status != CONFIRMED or track.inspected:
            return False
        if timestamp - track.last_seen > self.stale_after:
            return False
        # 处理时刻与采集时刻都必须属于核验窗口，重复图像时间戳只计一次。
        fresh_stamps = sorted({
            stamp for hit_time, stamp in zip(track.hit_times, track.observation_stamps)
            if float(since) < hit_time <= timestamp
            and float(since) < stamp <= timestamp
        })
        return (
            len(fresh_stamps) >= max(2, int(min_hits))
            and fresh_stamps[-1] - fresh_stamps[0] >= float(min_observation_span)
        )

    def has_recent_observation(
        self,
        track_id: int,
        *,
        min_hits: int = 2,
        window: float = 0.5,
        now: float | None = None,
    ) -> bool:
        """检查指定轨迹在最近 window 时间窗口内是否至少有 min_hits 次观测命中。"""
        timestamp = float(now if now is not None else get_time())
        parsed_id = int(track_id)
        track = next((item for item in self._tracks if item.track_id == parsed_id), None)
        if track is None:
            return False
        recent_hits = [
            stamp for stamp in track.hit_times
            if stamp >= timestamp - float(window)
        ]
        return len(recent_hits) >= int(min_hits)

    def mark_inspected(
        self,
        target_xy: tuple[float, float],
        radius: float = 1.2,
    ) -> None:
        """兼容旧调用；新流程应在核验成功后使用 ``mark_inspection_verified``。"""
        for track in self._tracks:
            dist = math.hypot(
                float(track.mean_position[0]) - float(target_xy[0]),
                float(track.mean_position[1]) - float(target_xy[1]),
            )
            if dist <= radius:
                track.inspected = True

    def _update_one(
        self,
        candidate: DetectionCandidate,
        now: float,
        excluded_track_ids: set[int],
    ) -> int:
        matched = self._nearest_track(candidate.position, excluded_track_ids)
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
            return track.track_id
        matched.update(candidate, now)
        return matched.track_id

    def _nearest_track(
        self,
        position: Sequence[float],
        excluded_track_ids: set[int],
    ) -> Track | None:
        point = np.array(position, dtype=float)
        nearest: Track | None = None
        nearest_distance = math.inf
        for track in self._tracks:
            if track.status == REJECTED or track.track_id in excluded_track_ids:
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
            recent_hits = track.recent_hit_count(now, self.confirmation_window)
            avg_rmse = track.average_rmse()
            avg_conf = track.average_confidence()
            pos_std = track.position_std()

            # Tier 1: 极低残差 (<= 5mm) 快速通道，按配置分级确认（默认 2 帧）
            tier1_ok = (
                recent_hits >= self.tier1_min_hits
                and avg_rmse <= self.tier1_max_rmse
                and avg_conf >= self.tier1_min_confidence
            )

            # Tier 2: 较低残差 (<= 15mm) 快速通道，2 帧确认
            tier2_ok = (
                recent_hits >= self.tier2_min_hits
                and avg_rmse <= self.tier2_max_rmse
                and avg_conf >= self.tier2_min_confidence
                and pos_std <= self.max_position_std
            )

            # Tier 3: 标准基线保守通道，4 帧确认
            tier3_ok = (
                recent_hits >= self.min_hits
                and avg_rmse <= self.max_average_rmse
                and avg_conf >= self.min_confidence
                and pos_std <= self.max_position_std
            )

            if tier1_ok or tier2_ok or tier3_ok:
                track.status = CONFIRMED


def _distance(a: np.ndarray, b: np.ndarray) -> float:
    diff = a - b
    return float(np.linalg.norm(diff))
