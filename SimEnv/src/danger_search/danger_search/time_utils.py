"""时间工具，自动使用 ROS 仿真时间。"""
import time

try:
    import rospy
except ImportError:
    rospy = None

def get_time() -> float:
    if rospy is not None and not rospy.is_shutdown():
        try:
            return rospy.Time.now().to_sec()
        except Exception:
            pass
    return time.time()
