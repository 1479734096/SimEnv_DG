"""定位时间匹配与健康状态的无副作用决策层。"""
from __future__ import annotations

from dataclasses import dataclass
import math

from danger_search.pose_provider import (
    POSE_EXPIRED,
    POSE_MATCHED,
    POSE_PENDING,
    PoseLookup,
    PoseSample,
)


@dataclass(frozen=True)
class LocalizationHealthAssessment:
    """一次定位健康检查的决策结果。

    该对象只描述应采取的动作；停车、日志和任务状态转换仍由主状态机执行。
    """

    allows_progress: bool
    recovered: bool
    stale_started: bool
    failure_started: bool
    elapsed: float
    pose_stamp: float | None
    pose_age: float
    observed_period: float
    stale_limit: float
    unavailable_duration: float


class LocalizationRuntime:
    """集中管理定位健康、传感器位姿匹配和诊断统计。"""

    def __init__(
        self,
        pose_provider,
        *,
        stale_minimum: float,
        unavailable_timeout: float,
        start_time: float,
    ) -> None:
        stale_minimum = float(stale_minimum)
        unavailable_timeout = float(unavailable_timeout)
        start_time = float(start_time)
        if not math.isfinite(stale_minimum) or stale_minimum <= 0.0:
            raise ValueError("定位 stale 最小阈值必须为正有限数")
        if (
            not math.isfinite(unavailable_timeout)
            or unavailable_timeout <= stale_minimum
        ):
            raise ValueError("定位不可用超时必须大于 stale 最小阈值")
        if not math.isfinite(start_time):
            raise ValueError("任务开始时间必须为有限数")

        self.pose_provider = pose_provider
        self.stale_minimum = stale_minimum
        self.unavailable_timeout = unavailable_timeout
        self.start_time = start_time
        self.unavailable_since: float | None = None
        self.stale_reported = False
        self.failure_reported = False
        self._pending_stamps: dict[str, float | None] = {}
        self._lookup_stats: dict[str, dict[str, int]] = {
            "cloud": {POSE_MATCHED: 0, POSE_PENDING: 0, POSE_EXPIRED: 0},
            "rgbd": {POSE_MATCHED: 0, POSE_PENDING: 0, POSE_EXPIRED: 0},
        }

    def restore_health_state(
        self,
        *,
        unavailable_since: float | None,
        stale_reported: bool,
        failure_reported: bool,
    ) -> None:
        """兼容轻量测试和旧检查点中的健康状态。"""
        self.unavailable_since = (
            None if unavailable_since is None else float(unavailable_since)
        )
        self.stale_reported = bool(stale_reported)
        self.failure_reported = bool(failure_reported)

    def assess_health(
        self,
        pose_sample: PoseSample | None,
        *,
        now: float,
    ) -> LocalizationHealthAssessment:
        """按现有 10 Hz 定位语义判断是否允许任务继续推进。"""
        current_time = float(now)
        period_reader = getattr(self.pose_provider, "observed_period", None)
        observed_period = float(
            period_reader(default=0.1) if callable(period_reader) else 0.1
        )
        stale_limit = max(self.stale_minimum, 4.0 * observed_period)
        stale_limit = min(
            stale_limit,
            max(self.stale_minimum, 0.5 * self.unavailable_timeout),
        )
        elapsed = max(0.0, current_time - self.start_time)
        sample_stamp = getattr(pose_sample, "stamp", None)
        if sample_stamp is None or not math.isfinite(float(sample_stamp)):
            pose_stamp = None
            age = math.inf
        else:
            pose_stamp = float(sample_stamp)
            age = max(0.0, current_time - pose_stamp)

        if pose_sample is not None and age <= stale_limit:
            unavailable_since = self.unavailable_since
            unavailable_duration = (
                max(0.0, current_time - unavailable_since)
                if unavailable_since is not None
                else 0.0
            )
            self.unavailable_since = None
            recovered = unavailable_since is not None
            self.stale_reported = False
            return LocalizationHealthAssessment(
                allows_progress=True,
                recovered=recovered,
                stale_started=False,
                failure_started=False,
                elapsed=elapsed,
                pose_stamp=pose_stamp,
                pose_age=age,
                observed_period=observed_period,
                stale_limit=stale_limit,
                unavailable_duration=unavailable_duration,
            )

        if self.unavailable_since is None:
            self.unavailable_since = current_time
        unavailable_duration = max(
            0.0,
            current_time - float(self.unavailable_since),
        )
        stale_started = not self.stale_reported
        self.stale_reported = True
        failure_started = (
            unavailable_duration >= self.unavailable_timeout
            and not self.failure_reported
        )
        if failure_started:
            self.failure_reported = True
        return LocalizationHealthAssessment(
            allows_progress=False,
            recovered=False,
            stale_started=stale_started,
            failure_started=failure_started,
            elapsed=elapsed,
            pose_stamp=pose_stamp,
            pose_age=age,
            observed_period=observed_period,
            stale_limit=stale_limit,
            unavailable_duration=unavailable_duration,
        )

    def lookup(
        self,
        stamp: float | None,
        *,
        allow_legacy_at: bool = True,
    ) -> PoseLookup:
        return time_matched_pose_lookup(
            self.pose_provider,
            stamp,
            allow_legacy_at=allow_legacy_at,
        )

    def record_lookup(
        self,
        stream: str,
        lookup: PoseLookup,
        *,
        stamp: float | None = None,
    ) -> bool:
        """累计诊断并抑制同一待匹配帧的重复计数。

        返回 ``True`` 表示本次应记录对应结构化事件。
        """
        stream = str(stream)
        if (
            lookup.status == POSE_PENDING
            and stream in self._pending_stamps
            and self._pending_stamps[stream] == stamp
        ):
            return False
        if lookup.status == POSE_PENDING:
            self._pending_stamps[stream] = stamp

        stream_stats = self._lookup_stats.setdefault(
            stream,
            {POSE_MATCHED: 0, POSE_PENDING: 0, POSE_EXPIRED: 0},
        )
        stream_stats[lookup.status] = int(stream_stats.get(lookup.status, 0)) + 1
        return True

    def lookup_stats(self) -> dict[str, dict[str, int]]:
        return {
            stream: dict(values)
            for stream, values in self._lookup_stats.items()
        }


def time_matched_pose_lookup(
    provider,
    stamp: float | None,
    *,
    allow_legacy_at: bool = True,
) -> PoseLookup:
    """适配生产 PoseProvider 与历史轻量测试替身。"""
    if provider is None:
        return PoseLookup(POSE_MATCHED, None, None)
    lookup = getattr(provider, "lookup", None)
    if callable(lookup):
        return lookup(stamp)
    at = getattr(provider, "at", None)
    if not allow_legacy_at or not callable(at):
        return PoseLookup(POSE_MATCHED, None, None)
    sample = at(stamp)
    return PoseLookup(
        POSE_MATCHED if sample is not None else POSE_EXPIRED,
        sample,
        None,
    )
