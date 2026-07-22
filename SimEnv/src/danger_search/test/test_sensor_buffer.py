#!/usr/bin/env python3
"""传感器缓存的接口与点云过滤测试。"""
import math
import unittest
from unittest.mock import patch

from danger_search.sensor_buffer import SensorBuffer


def _buffer(**overrides):
    values = {
        "cloud_topic": "/livox/Pointcloud2",
        "rgb_topic": "/real_sense/rgb/image_raw",
        "depth_topic": "/real_sense/depth/image_raw",
        "camera_info_topic": "/real_sense/depth/camera_info",
    }
    values.update(overrides)
    return SensorBuffer(**values)


class SensorBufferTest(unittest.TestCase):
    def test_only_accepts_public_sensor_inputs(self):
        self.assertEqual(_buffer().cloud_topic, "/livox/Pointcloud2")
        self.assertEqual(
            _buffer(
                cloud_topic="/danger_search/localization/cloud_registered"
            ).cloud_topic,
            "/danger_search/localization/cloud_registered",
        )
        with self.assertRaises(ValueError):
            _buffer(cloud_topic="/cmd_vel")
        with self.assertRaises(ValueError):
            _buffer(cloud_topic="/ground_truth/base_w")

    def test_sync_configuration_must_be_bounded(self):
        with self.assertRaises(ValueError):
            _buffer(sync_slop=math.inf)
        with self.assertRaises(ValueError):
            _buffer(sync_queue_size=0)

    def test_cloud_reader_filters_non_finite_points_and_honors_limit(self):
        reader = type(
            "Reader",
            (),
            {
                "read_points": staticmethod(
                    lambda *_args, **_kwargs: iter(
                        [(1.0, 2.0, 3.0), (math.inf, 0.0, 0.0), (4.0, 5.0, 6.0)]
                    )
                )
            },
        )
        with patch("danger_search.sensor_buffer.pc2", reader):
            self.assertEqual(
                _buffer().read_cloud_xyz(object(), max_points=2),
                [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)],
            )

    def test_cloud_reader_rejects_non_positive_limit(self):
        with self.assertRaises(ValueError):
            _buffer().read_cloud_xyz(object(), max_points=0)


if __name__ == "__main__":
    unittest.main()
