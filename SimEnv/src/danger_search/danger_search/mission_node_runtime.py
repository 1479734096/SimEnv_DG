"""ROS 节点生命周期与回调边缘。

该模块只负责把 ROS 消息转换为业务事件、启动组件和管理定时器/线程；任务状态
转换仍由 MissionManager 决定。
"""
from __future__ import annotations

import math
from threading import Event, Thread
import time

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool
except ImportError:  # pragma: no cover - 离线单测不要求 ROS
    rospy = None
    PoseStamped = None
    Bool = None

from danger_search.ground_support_gate import SAFETY_STOP_TIMEOUT
from danger_search.interface_policy import require_internal_input_topic
from danger_search.mission_config import ros_parameter
from danger_search.time_utils import get_time


LOCALIZATION_READY_TOPIC = "/danger_search/localization/ready"
LOCALIZATION_HEALTH_TOPIC = "/danger_search/localization/health"


def wait_for_localization_ready(
    *,
    subscriber_factory=None,
    shutdown_check=None,
    poll_interval: float = 0.1,
) -> bool:
    """等待定位锁存就绪；等待期间不创建任务控制 publisher。"""
    if rospy is None or Bool is None:
        raise RuntimeError("当前环境没有 ROS Bool 消息，无法等待定位就绪")
    topic = require_internal_input_topic(
        LOCALIZATION_READY_TOPIC,
        purpose="定位站立就绪输入",
    )
    interval = float(poll_interval)
    if not math.isfinite(interval) or interval <= 0.0:
        raise ValueError("定位就绪轮询周期必须为正有限数")
    ready = Event()

    def on_ready(message) -> None:
        if bool(getattr(message, "data", False)):
            ready.set()

    subscribe = subscriber_factory or rospy.Subscriber
    is_shutdown = shutdown_check or rospy.is_shutdown
    subscriber = subscribe(topic, Bool, on_ready, queue_size=1)
    try:
        while not ready.is_set():
            if is_shutdown():
                return False
            ready.wait(interval)
        return True
    finally:
        unregister = getattr(subscriber, "unregister", None)
        if callable(unregister):
            unregister()


class MissionNodeRuntime:
    """持有一次 MissionManager 对应的全部 ROS 资源。"""

    def __init__(self, manager) -> None:
        if rospy is None:
            raise RuntimeError("当前环境没有 rospy，无法启动任务节点")
        self._manager = manager
        self._wall_stop = Event()
        self._wall_thread: Thread | None = None
        self._timer = None
        self._ground_support_timer = None
        self._goal_subscriber = None
        self._health_subscriber = None

    def start(self) -> None:
        manager = self._manager
        manager.pose_provider.start()
        manager.sensor_buffer.start()
        manager.controller.start(pose_provider=manager.pose_provider)
        if manager.building_services_required():
            manager.building_client.start(connect_elevator=manager.max_floor > 1)
        manager.start_elevator_visual_preparation()
        manager.reporting.start_ros()
        if manager.manual_goal_enabled and PoseStamped is not None:
            self._goal_subscriber = rospy.Subscriber(
                manager.goal_topic,
                PoseStamped,
                self._on_goal,
            )
        health_topic = require_internal_input_topic(
            LOCALIZATION_HEALTH_TOPIC,
            purpose="定位持续姿态健康输入",
        )
        self._health_subscriber = rospy.Subscriber(
            health_topic,
            Bool,
            self._on_localization_health,
            queue_size=1,
        )
        self._timer = rospy.Timer(rospy.Duration(0.1), self._on_timer)
        if manager.ground_support_gate is not None:
            self._ground_support_timer = rospy.Timer(
                rospy.Duration(manager.ground_support_assessment_period),
                self._on_ground_support_timer,
            )
        self._wall_thread = Thread(
            target=self._ground_support_wall_watch,
            name="mission-wall-watch",
            daemon=True,
        )
        self._wall_thread.start()
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("danger_search 已启动，等待传感器数据")

    def shutdown(self) -> None:
        self._wall_stop.set()
        self._manager.handle_shutdown()

    def request_shutdown(self, reason: str) -> None:
        rospy.signal_shutdown(str(reason))

    def _on_goal(self, message) -> None:
        self._manager.accept_manual_goal(
            float(message.pose.position.x),
            float(message.pose.position.y),
        )

    def _on_localization_health(self, message) -> None:
        if message is None:
            return
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        stamp_value = (
            float(stamp.to_sec())
            if stamp is not None and hasattr(stamp, "to_sec")
            else float(get_time())
        )
        if math.isfinite(stamp_value):
            self._manager.accept_localization_health(
                bool(getattr(message, "data", False)),
                stamp_value,
            )

    def _on_timer(self, _event) -> None:
        try:
            self._manager.tick()
        except Exception as exc:  # 业务异常必须在 ROS 边缘转成显式失败
            rospy.logerr("danger_search 状态机异常：%s", exc)
            self._manager.handle_unexpected_exception(exc)

    def _on_ground_support_timer(self, event) -> None:
        try:
            self._manager.tick_ground_support(event)
        except Exception as exc:
            rospy.logerr("地面支撑评估异常：%s", exc)
            self._manager.handle_unexpected_exception(exc)

    def _ground_support_wall_watch(self) -> None:
        manager = self._manager
        while not self._wall_stop.wait(0.05):
            tick_timed_out, tick_reason = manager.check_tick_wall_timeout(
                time.monotonic()
            )
            if tick_timed_out:
                manager.handle_tick_wall_timeout(tick_reason)
                self.request_shutdown(tick_reason)
                return
            gate = manager.ground_support_gate
            if gate is None:
                continue
            timed_out, reason = manager.check_ground_support_wall_timeout(
                time.monotonic()
            )
            if not timed_out:
                continue
            manager.handle_ground_support_wall_timeout(reason)
            self.request_shutdown(reason)
            return


def main() -> int:
    """正式 ROS 可执行入口。"""
    if rospy is None:
        raise RuntimeError("当前环境没有 rospy，无法运行 danger_search_node")
    rospy.init_node("danger_search")
    runtime_profile = str(
        ros_parameter("runtime/profile", "formal")
    ).strip().lower()
    if runtime_profile == "formal" and not bool(
        ros_parameter("controller_ready_ack", False)
    ):
        rospy.logfatal(
            "正式任务拒绝启动：尚未确认 junior_ctrl 已进入 RL /cmd_vel 模式"
        )
        return 2
    if bool(ros_parameter("wait_for_localization_ready", True)):
        rospy.loginfo("等待定位站立证据与世界系初始化完成")
        if not wait_for_localization_ready():
            return 1
        rospy.loginfo("定位已就绪，开始创建任务编排器并计入任务预算")

    # 延迟导入避免配置模块与状态机模块在离线测试中形成 ROS 初始化副作用。
    from danger_search.mission_manager import MissionManager

    manager = MissionManager()
    runtime = MissionNodeRuntime(manager)
    manager.attach_node_runtime(runtime)
    runtime.start()
    rospy.spin()
    return 0
