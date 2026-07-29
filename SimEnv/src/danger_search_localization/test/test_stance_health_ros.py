#!/usr/bin/env python3
"""用合成公开传感器验证定位 ready/health 的失效安全闭环。"""

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Imu, PointCloud
from std_msgs.msg import Bool


class StanceHealthRosTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._health = None
        self._ready = None
        self._health_was_true = False
        self._ready_was_true = False
        self._health_false_after_true = False
        self._ready_false_after_true = False
        self._imu_publisher = rospy.Publisher("/trunk_imu", Imu, queue_size=5)
        self._scan_publisher = rospy.Publisher("/scan", PointCloud, queue_size=2)
        self._health_subscriber = rospy.Subscriber(
            "/danger_search/localization/health",
            Bool,
            self._on_health,
            queue_size=5,
        )
        self._ready_subscriber = rospy.Subscriber(
            "/danger_search/localization/ready",
            Bool,
            self._on_ready,
            queue_size=5,
        )
        self._floor_points = self._make_floor_points()

    def _on_health(self, message):
        with self._lock:
            value = bool(message.data)
            if value:
                self._health_was_true = True
            elif self._health_was_true:
                self._health_false_after_true = True
            self._health = value

    def _on_ready(self, message):
        with self._lock:
            value = bool(message.data)
            if value:
                self._ready_was_true = True
            elif self._ready_was_true:
                self._ready_false_after_true = True
            self._ready = value

    @staticmethod
    def _make_floor_points():
        """生成 base 系 z=-0.31 m 的楼面，再逆变换到 Livox 系。"""
        pitch = 0.785
        cosine = math.cos(pitch)
        sine = math.sin(pitch)
        translation_x = 0.2
        translation_z = 0.08
        points = []
        for ix in range(-10, 11):
            for iy in range(-10, 11):
                base_x = 0.2 * ix
                base_y = 0.2 * iy
                base_z = -0.31
                delta_x = base_x - translation_x
                delta_z = base_z - translation_z
                sensor_x = cosine * delta_x - sine * delta_z
                sensor_y = base_y
                sensor_z = sine * delta_x + cosine * delta_z
                points.append(Point32(sensor_x, sensor_y, sensor_z))
        return points

    def _publish_frame(self, *, fallen):
        stamp = rospy.Time.now()
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        if fallen:
            imu.orientation.x = math.sin(math.pi / 4.0)
            imu.orientation.w = math.cos(math.pi / 4.0)
        else:
            imu.orientation.w = 1.0
        imu.linear_acceleration.z = 9.81
        self._imu_publisher.publish(imu)
        time.sleep(0.02)

        cloud = PointCloud()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "laser_livox"
        cloud.points = self._floor_points
        self._scan_publisher.publish(cloud)
        time.sleep(0.08)

    def _connections_ready(self):
        return (
            self._imu_publisher.get_num_connections() > 0
            and self._scan_publisher.get_num_connections() > 0
        )

    def _wait_for_connections(self):
        deadline = time.monotonic() + 8.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._connections_ready():
                return
            time.sleep(0.05)
        self.fail("定位节点未在时限内订阅合成 /scan 与 /trunk_imu")

    def test_health_and_ready_revoke_after_fall(self):
        self._wait_for_connections()
        for _ in range(20):
            self._publish_frame(fallen=False)
            with self._lock:
                if self._health_was_true and self._ready_was_true:
                    break
        with self._lock:
            self.assertTrue(self._health_was_true)
            self.assertTrue(self._ready_was_true)

        for _ in range(10):
            self._publish_frame(fallen=True)
            with self._lock:
                if self._health_false_after_true and self._ready_false_after_true:
                    break
        with self._lock:
            self.assertTrue(self._health_false_after_true)
            self.assertTrue(self._ready_false_after_true)
            self.assertFalse(self._health)
            self.assertFalse(self._ready)


if __name__ == "__main__":
    rospy.init_node("stance_health_ros_test")
    rostest.rosrun(
        "danger_search_localization",
        "stance_health_ros_test",
        StanceHealthRosTest,
    )
