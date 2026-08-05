"""任务结果、状态话题与结构化日志的输出边界。"""
from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
from typing import Iterable, Mapping

try:
    import rospy
    from std_msgs.msg import String
    from nav_msgs.msg import OccupancyGrid
except ImportError:  # pragma: no cover - 离线单测不要求 ROS
    rospy = None
    String = None
    OccupancyGrid = None

from danger_search.result_writer import ResultWriter


def atomic_write_json(path: Path, payload: Mapping[str, object]) -> None:
    """原子写入 JSON，避免 runner 读取半截结果。"""
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(dict(payload), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


@dataclass(frozen=True)
class StatusReport:
    """一次状态发布所需的不可变业务摘要和只读地图引用。"""

    payload: Mapping[str, object]
    grid: object
    floor_index: int
    floor_height: float


class MissionReportingRuntime:
    """唯一持有结果文件、状态 publisher 和地图诊断 publisher 的边缘对象。"""

    def __init__(
        self,
        *,
        result_file: str | Path,
        mission_summary_file: str | Path | None,
        status_topic: str,
        map_topic: str,
    ) -> None:
        self._result_writer = ResultWriter(result_file)
        self._summary_path = (
            Path(mission_summary_file) if mission_summary_file else None
        )
        self._status_topic = str(status_topic)
        self._map_topic = str(map_topic)
        self._status_publisher = None
        self._map_publisher = None

    @property
    def status_enabled(self) -> bool:
        return self._status_publisher is not None and String is not None

    def start_ros(self) -> None:
        if rospy is None or String is None:
            raise RuntimeError("当前环境没有 ROS，无法启动任务报告边缘")
        self._status_publisher = rospy.Publisher(
            self._status_topic, String, queue_size=1
        )
        self._map_publisher = (
            rospy.Publisher(
                self._map_topic, OccupancyGrid, queue_size=1, latch=True
            )
            if OccupancyGrid is not None
            else None
        )

    def publish_status(self, report: StatusReport) -> None:
        if self._status_publisher is None or String is None:
            return
        grid = report.grid
        if self._map_publisher is not None:
            message = OccupancyGrid()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "odom"
            message.info.resolution = grid.resolution
            message.info.width = grid.size_x
            message.info.height = grid.size_y
            message.info.origin.position.x = grid.origin_x
            message.info.origin.position.y = grid.origin_y
            message.info.origin.position.z = (
                int(report.floor_index) * float(report.floor_height)
            )
            message.info.origin.orientation.w = 1.0
            message.data = grid.data.flatten().tolist()
            self._map_publisher.publish(message)
        self._status_publisher.publish(
            String(
                data=json.dumps(
                    dict(report.payload),
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
        )

    def write_result(
        self,
        *,
        elapsed: float,
        positions: Iterable[Iterable[float]],
        summary: Mapping[str, object] | None,
    ) -> Path:
        path = self._result_writer.write(float(elapsed), positions)
        if self._summary_path is not None and summary is not None:
            self._summary_path.parent.mkdir(parents=True, exist_ok=True)
            payload = dict(summary)
            payload["result_file"] = str(path)
            atomic_write_json(self._summary_path, payload)
        return path

    @staticmethod
    def log_structured(payload: Mapping[str, object]) -> None:
        logger = getattr(rospy, "loginfo", None) if rospy is not None else None
        if callable(logger):
            logger(
                "%s",
                json.dumps(dict(payload), ensure_ascii=False, sort_keys=True),
            )

    @staticmethod
    def loginfo(message: str, *args) -> None:
        logger = getattr(rospy, "loginfo", None) if rospy is not None else None
        if callable(logger):
            logger(message, *args)

    @staticmethod
    def logwarn(message: str, *args) -> None:
        logger = getattr(rospy, "logwarn", None) if rospy is not None else None
        if callable(logger):
            logger(message, *args)

    @staticmethod
    def logwarn_throttle(period: float, message: str, *args) -> None:
        logger = (
            getattr(rospy, "logwarn_throttle", None)
            if rospy is not None
            else None
        )
        if callable(logger):
            logger(float(period), message, *args)

    @staticmethod
    def logerr(message: str, *args) -> None:
        logger = getattr(rospy, "logerr", None) if rospy is not None else None
        if callable(logger):
            logger(message, *args)
