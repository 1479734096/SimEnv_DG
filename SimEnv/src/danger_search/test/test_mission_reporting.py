"""任务诊断输出坐标系合同测试。"""
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from danger_search.mission_reporting import MissionReportingRuntime, StatusReport


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _OccupancyGrid:
    def __init__(self):
        self.header = SimpleNamespace()
        self.info = SimpleNamespace(
            origin=SimpleNamespace(
                position=SimpleNamespace(),
                orientation=SimpleNamespace(),
            )
        )
        self.data = []


class _String:
    def __init__(self, *, data):
        self.data = data


def test_map_diagnostic_uses_configured_global_frame(tmp_path):
    runtime = MissionReportingRuntime(
        result_file=tmp_path / "detected_danger.json",
        mission_summary_file=None,
        status_topic="/danger_search/status",
        map_topic="/danger_search/map",
        frame_id="danger_search_relative",
    )
    runtime._status_publisher = _Publisher()
    runtime._map_publisher = _Publisher()
    grid = SimpleNamespace(
        resolution=0.1,
        size_x=2,
        size_y=2,
        origin_x=-1.0,
        origin_y=-1.0,
        data=np.zeros((2, 2), dtype=np.int8),
    )

    with patch("danger_search.mission_reporting.OccupancyGrid", _OccupancyGrid), patch(
        "danger_search.mission_reporting.String", _String
    ), patch(
        "danger_search.mission_reporting.rospy",
        SimpleNamespace(Time=SimpleNamespace(now=lambda: object())),
    ):
        runtime.publish_status(
            StatusReport(payload={}, grid=grid, floor_index=0, floor_height=2.6)
        )

    assert runtime._map_publisher.messages[0].header.frame_id == "danger_search_relative"
