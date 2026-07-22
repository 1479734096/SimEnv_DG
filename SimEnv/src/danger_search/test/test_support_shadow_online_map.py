"""支撑稀疏层向在线负障地图输出的回归测试。"""
from __future__ import annotations

import unittest

import numpy as np

from danger_search.support_shadow import ShadowSupportConfig, ShadowSupportGrid


class ShadowSupportOnlineMapTest(unittest.TestCase):
    def setUp(self):
        self.config = ShadowSupportConfig(
            resolution=0.1,
            origin_x=-1.0,
            origin_y=-1.0,
            cloud_stride=1,
            cloud_max_points=100,
            minimum_cell_hits=2,
            required_consecutive_frames=2,
            minimum_alert_cells=3,
            sweep_half_width=0.25,
            minimum_frame_points=1,
            threshold_widths=(0.25,),
        )
        self.grid = ShadowSupportGrid(self.config)
        self.sensor_origin = np.asarray((0.0, 0.0, 0.4), dtype=float)
        self.sensor_rotation = np.eye(3, dtype=float)
        self.path = [np.asarray((0.0, 0.0)), np.asarray((0.8, 0.0))]

    def _process(self, stamp: float, points):
        return self.grid.process_frame(
            stamp=stamp,
            floor=0,
            robot_xy=np.asarray((0.0, 0.0)),
            robot_yaw=0.0,
            sensor_origin=self.sensor_origin,
            sensor_rotation=self.sensor_rotation,
            points=np.asarray(points, dtype=float),
            path=self.path,
            command_linear=0.3,
            ambiguous=False,
            local_only=True,
        )

    @staticmethod
    def _lower_points():
        # 从 0.4m 高的传感器射向 -0.6m 表面，分别穿过本层前三个相邻格。
        return [
            point
            for x in (0.40, 0.65, 0.90)
            for point in ((x, 0.0, -0.6), (x, 0.0, -0.6))
        ]

    def test_confirmed_lower_returns_and_support_can_replace_each_other(self):
        first = self._process(1.0, self._lower_points())
        second = self._process(1.1, self._lower_points())

        self.assertEqual(first["confirmed_negative_cells"], [])
        negative = {tuple(cell) for cell in second["confirmed_negative_cells"]}
        self.assertGreaterEqual(len(negative), 3)

        support_points = []
        for gx, gy in negative:
            x = self.config.origin_x + (gx + 0.5) * self.config.resolution
            y = self.config.origin_y + (gy + 0.5) * self.config.resolution
            support_points.extend(((x, y, 0.0), (x, y, 0.0)))
        first_support = self._process(1.2, support_points)
        self.assertEqual(first_support["confirmed_supported_cells"], [])
        supported = self._process(1.3, support_points)
        self.assertTrue(
            negative.issubset(
                {tuple(cell) for cell in supported["confirmed_supported_cells"]}
            )
        )

        self._process(1.4, self._lower_points())
        lower_again = self._process(1.5, self._lower_points())
        self.assertTrue(
            negative.issubset(
                {tuple(cell) for cell in lower_again["confirmed_negative_cells"]}
            )
        )


if __name__ == "__main__":
    unittest.main()
