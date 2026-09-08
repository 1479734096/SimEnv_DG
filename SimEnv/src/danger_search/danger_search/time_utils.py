"""统一任务计时来源。

Gazebo 运行时严格使用 ROS ``/clock``，否则仿真暂停或实时率下降会错误消耗
任务预算；没有 ROS 或尚未 ``init_node`` 的离线进程使用单调墙钟。ROS 节点
初始化后读取失败必须显式报错，不能在一次任务中悄悄切换时间域。
"""
import math
import time

try:
    import rospy
except ImportError:
    rospy = None

def get_time() -> float:
    """返回当前任务时间；ROS 运行时失败时拒绝混入墙钟（shutdown 时降级到墙钟）。"""
    core = getattr(rospy, "core", None) if rospy is not None else None
    ros_initialized = (
        bool(core.is_initialized())
        if core is not None and callable(getattr(core, "is_initialized", None))
        else rospy is not None
    )
    if ros_initialized:
        if rospy is not None and bool(rospy.is_shutdown()):
            return time.time()
        try:
            value = float(rospy.Time.now().to_sec())
        except Exception as exc:
            if rospy is not None and bool(rospy.is_shutdown()):
                return time.time()
            raise RuntimeError("ROS 任务时间不可用，拒绝切换到墙钟") from exc
        if not math.isfinite(value) or value < 0.0:
            raise RuntimeError(f"ROS 任务时间非法: {value!r}")
        return value
    return time.monotonic()
