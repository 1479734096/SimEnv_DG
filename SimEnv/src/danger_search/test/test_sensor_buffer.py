#!/usr/bin/env python3
"""传感器缓存的接口与点云过滤测试。"""
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from danger_search.sensor_buffer import SensorBuffer, message_stamp_seconds


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
    @staticmethod
    def _message(stamp=1.0, frame_id="laser_livox"):
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(to_sec=lambda: stamp),
                frame_id=frame_id,
            )
        )

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

    def test_cloud_generation_only_advances_for_valid_new_messages(self):
        buffer = _buffer()
        valid = self._message()
        buffer._on_cloud(valid)
        first = buffer.snapshot()
        buffer._on_cloud(self._message(stamp=0.0))
        rejected = buffer.snapshot()
        buffer._on_cloud(self._message(stamp=2.0, frame_id="unexpected"))
        second_rejected = buffer.snapshot()

        self.assertEqual(first.cloud_generation, 1)
        self.assertIs(first.cloud_msg, valid)
        self.assertEqual(rejected.cloud_generation, 1)
        self.assertEqual(second_rejected.cloud_generation, 1)
        self.assertEqual(buffer.rejected_cloud_count, 2)

    def test_rgbd_bundle_requires_matching_frame_and_bounded_time_window(self):
        buffer = _buffer(sync_slop=0.08)
        valid = [
            self._message(1.00, "real_sense_optical_frame"),
            self._message(1.03, "real_sense_optical_frame"),
            self._message(1.02, "real_sense_optical_frame"),
        ]
        buffer._on_image_bundle(*valid)
        accepted = buffer.snapshot()
        buffer._on_image_bundle(
            self._message(2.00, "real_sense_optical_frame"),
            self._message(2.20, "real_sense_optical_frame"),
            self._message(2.00, "real_sense_optical_frame"),
        )

        self.assertEqual(accepted.image_generation, 1)
        self.assertEqual(accepted.image_bundle.generation, 1)
        self.assertEqual(buffer.snapshot().image_generation, 1)
        self.assertEqual(buffer.rejected_image_bundle_count, 1)

    def test_timestamp_contract_rejects_missing_zero_and_non_finite_values(self):
        self.assertIsNone(message_stamp_seconds(object()))
        self.assertIsNone(message_stamp_seconds(self._message(0.0)))
        self.assertIsNone(message_stamp_seconds(self._message(math.nan)))
        self.assertEqual(message_stamp_seconds(self._message(1.25)), 1.25)


if __name__ == "__main__":
    unittest.main()
