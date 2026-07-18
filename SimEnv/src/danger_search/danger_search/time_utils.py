"""统一任务计时来源。

Gazebo 运行时必须使用 ROS ``/clock``，否则仿真暂停或实时率下降会错误消耗
任务预算；不含 ROS 的离线测试则退回系统墙钟。调用方只依赖 ``get_time``，
因此状态机、控制器和测试不会各自选择不同的时间基准。
"""
import time

try:
    import rospy
except ImportError:
    rospy = None

def get_time() -> float:
    """返回当前任务时间，优先取 ROS 仿真时间，离线环境取 Unix 时间。

    ROS 初始化早期或关闭过程中读取时间可能抛出异常，此时回退墙钟是为了让
    清理和结果写出仍可完成；正常仿真循环不会经过该回退路径。
    """
    if rospy is not None and not rospy.is_shutdown():
        try:
            return rospy.Time.now().to_sec()
        except Exception:
            pass
    return time.time()
