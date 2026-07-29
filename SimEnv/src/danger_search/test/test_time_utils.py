import unittest
from unittest.mock import patch

from danger_search import time_utils


class _RosTime:
    @staticmethod
    def now():
        return _RosTime()

    @staticmethod
    def to_sec():
        return 12.5


class _RosRuntime:
    Time = _RosTime

    @staticmethod
    def is_shutdown():
        return False


class _BrokenRosTime:
    @staticmethod
    def now():
        raise RuntimeError("clock unavailable")


class _BrokenRosRuntime:
    Time = _BrokenRosTime


class TimeUtilsTest(unittest.TestCase):
    def test_uses_ros_time_when_runtime_is_available(self):
        with patch.object(time_utils, "rospy", _RosRuntime):
            self.assertEqual(time_utils.get_time(), 12.5)

    def test_falls_back_to_monotonic_wall_time_without_ros(self):
        with patch.object(time_utils, "rospy", None), patch.object(
            time_utils.time,
            "monotonic",
            return_value=34.5,
        ):
            self.assertEqual(time_utils.get_time(), 34.5)

    def test_ros_failure_never_falls_back_to_wall_time(self):
        with patch.object(time_utils, "rospy", _BrokenRosRuntime), patch.object(
            time_utils.time,
            "monotonic",
            return_value=34.5,
        ):
            with self.assertRaisesRegex(RuntimeError, "拒绝切换到墙钟"):
                time_utils.get_time()


if __name__ == "__main__":
    unittest.main()
