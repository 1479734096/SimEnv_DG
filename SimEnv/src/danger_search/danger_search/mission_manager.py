"""danger_search 主任务状态机。"""
from __future__ import annotations

import json
import math
import os
import time

import numpy as np

try:
    import rospy
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - 离线环境可能没有 ROS
    rospy = None
    String = None

from danger_search.detection_tracker import DetectionTracker
from danger_search.floor_manager import FloorManager
from danger_search.frontier_explorer import Frontier, FrontierExplorer, astar_path
from danger_search.local_controller import DONE, IDLE, STUCK, PathFollower
from danger_search.multi_floor_mapper import MultiFloorGridMap
from danger_search.red_sphere_detector import RedSphereDetector
from danger_search.result_writer import ResultWriter
from danger_search.sensor_buffer import SensorBuffer, odom_pose_xy_yaw, odom_xyz


INIT = "INIT"
WAIT_CONTROLLER_READY = "WAIT_CONTROLLER_READY"
EXPLORE_FLOOR = "EXPLORE_FLOOR"
CHANGE_FLOOR = "CHANGE_FLOOR"
RETURN_HOME = "RETURN_HOME"
WRITE_RESULT = "WRITE_RESULT"
FINISH = "FINISH"


class MissionManager:
    """Timer 驱动的主状态机，避免在主循环里阻塞等待传感器。"""

    def __init__(self):
        if rospy is None:
            raise RuntimeError("当前环境没有 rospy，无法运行 mission_manager")
        self.state = INIT
        self.start_time = time.time()
        self.floor_start_time = self.start_time
        self.home_pose: tuple[float, float, float] | None = None
        self.current_floor = 0
        self.target_floor: int | None = None
        self.return_after_floor_change = False
        self.current_frontier: Frontier | None = None
        self._last_image_stamp: float | None = None
        self._service_task = None
        self._change_phase: str | None = None
        self._no_frontier_scan_until = 0.0
        self._last_status_publish = 0.0
        self._timer = None
        self._finished = False

        self._load_params()
        self._status_publisher = None
        self.sensor_buffer = SensorBuffer(
            odom_topic=self.odom_topic,
            cloud_topic=self.cloud_topic,
            rgb_topic=self.rgb_topic,
            depth_topic=self.depth_topic,
            camera_info_topic=self.camera_info_topic,
        )
        self.mapper = MultiFloorGridMap(**self.map_params)
        self.explorer = FrontierExplorer(**self.frontier_params)
        self.controller = PathFollower(cmd_vel_topic=self.cmd_vel_topic, **self.controller_params)
        self.detector = RedSphereDetector(**self.detection_params)
        self.tracker = DetectionTracker(**self.tracker_params)
        self.floor_manager = FloorManager(**self.floor_params)
        self.result_writer = ResultWriter(self.result_file)

    def start(self) -> None:
        self.sensor_buffer.start()
        self.controller.start()
        self.floor_manager.start()
        self._status_publisher = rospy.Publisher(self.status_topic, String, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(0.1), self._on_timer)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("danger_search 已启动，等待传感器数据")

    def shutdown(self) -> None:
        self.controller.stop()
        if not self._finished:
            self._write_result()

    def _load_params(self) -> None:
        self.cmd_vel_topic = _param("topics/cmd_vel", "/cmd_vel")
        self.odom_topic = _param("topics/odom", "/Odometry_gazebo")
        self.cloud_topic = _param("topics/cloud", "/livox/Pointcloud2")
        self.rgb_topic = _param("topics/rgb", "/real_sense/rgb/image_raw")
        self.depth_topic = _param("topics/depth", "/real_sense/depth/image_raw")
        self.camera_info_topic = _param("topics/camera_info", "/real_sense/rgb/camera_info")
        self.status_topic = _param("topics/status", "/danger_search/status")

        self.startup_delay = float(_param("mission/startup_delay", 2.0))
        self.max_runtime = float(_param("mission/max_runtime", 590.0))
        self.max_floor = int(_param("mission/max_floor", 1))
        self.floor_timeout = float(_param("mission/floor_timeout", 180.0))
        self.home_tolerance = float(_param("mission/home_tolerance", 0.7))
        self.explore_coverage_ratio = float(_param("mission/explore_coverage_ratio", 0.82))
        self.min_frontier_count = int(_param("mission/min_frontier_count", 1))
        self.no_frontier_scan_duration = float(_param("mission/no_frontier_scan_duration", 8.0))
        self.status_publish_period = float(_param("mission/status_publish_period", 1.0))
        self.result_file = str(_param("result_file", os.path.join(os.getcwd(), "results", "detected_danger.json")))

        self.map_params = {
            "resolution": float(_param("map/resolution", 0.10)),
            "width": float(_param("map/width", 28.0)),
            "length": float(_param("map/length", 48.0)),
            "origin_x": float(_param("map/origin_x", -14.0)),
            "origin_y": float(_param("map/origin_y", -6.0)),
            "floor_height": float(_param("map/floor_height", 2.6)),
            "obstacle_min_z": float(_param("map/obstacle_min_z", 0.15)),
            "obstacle_max_z": float(_param("map/obstacle_max_z", 1.40)),
            "raycast_max_range": float(_param("map/raycast_max_range", 12.0)),
            "visited_radius": float(_param("map/visited_radius", 0.45)),
        }
        self.cloud_stride = int(_param("map/cloud_stride", 3))
        self.cloud_max_points = int(_param("map/cloud_max_points", 18000))
        self.frontier_params = {
            "min_cluster_size": int(_param("frontier/min_cluster_size", 8)),
            "blacklist_ttl": float(_param("frontier/blacklist_ttl", 60.0)),
            "fail_limit": int(_param("frontier/fail_limit", 3)),
            "information_gain_weight": float(_param("frontier/information_gain_weight", 2.0)),
            "path_cost_weight": float(_param("frontier/path_cost_weight", 1.0)),
            "yaw_cost_weight": float(_param("frontier/yaw_cost_weight", 0.3)),
            "revisit_penalty_weight": float(_param("frontier/revisit_penalty_weight", 2.0)),
        }
        self.controller_params = {
            "max_linear": float(_param("controller/max_linear", 0.35)),
            "max_angular": float(_param("controller/max_angular", 0.9)),
            "goal_tolerance": float(_param("controller/goal_tolerance", 0.35)),
            "waypoint_tolerance": float(_param("controller/waypoint_tolerance", 0.25)),
            "pure_pursuit_lookahead": float(_param("controller/pure_pursuit_lookahead", 0.65)),
            "reverse_speed": float(_param("controller/reverse_speed", -0.18)),
            "recover_duration": float(_param("controller/recover_duration", 1.2)),
            "no_progress_timeout": float(_param("frontier/no_progress_timeout", 5.0)),
        }
        self.detection_params = {
            "min_area": int(_param("detection/min_area", 35)),
            "min_circularity": float(_param("detection/min_circularity", 0.55)),
            "min_aspect": float(_param("detection/min_aspect", 0.65)),
            "max_aspect": float(_param("detection/max_aspect", 1.55)),
            "min_radius": float(_param("detection/min_radius", 0.10)),
            "max_radius": float(_param("detection/max_radius", 0.22)),
            "max_rmse": float(_param("detection/max_rmse", 0.035)),
            "ransac_iterations": int(_param("detection/ransac_iterations", 80)),
            "ransac_sample_size": int(_param("detection/ransac_sample_size", 4)),
            "min_depth": float(_param("detection/min_depth", 0.35)),
            "max_depth": float(_param("detection/max_depth", 8.0)),
        }
        self.depth_frame_is_optical = bool(_param("detection/depth_frame_is_optical", True))
        self.tracker_params = {
            "association_radius": float(_param("tracker/association_radius", 0.45)),
            "confirmation_window": float(_param("tracker/confirmation_window", 10.0)),
            "min_hits": int(_param("tracker/min_hits", 4)),
            "max_position_std": float(_param("tracker/max_position_std", 0.25)),
            "min_confidence": float(_param("tracker/min_confidence", 0.75)),
            "max_average_rmse": float(_param("tracker/max_average_rmse", 0.04)),
            "nms_radius": float(_param("tracker/nms_radius", 0.9)),
            "stale_after": float(_param("tracker/stale_after", 30.0)),
        }
        self.floor_params = {
            "elevator_id": str(_param("floor/elevator_id", "elevator_main")),
            "lobby_xy": tuple(float(v) for v in _param("floor/elevator_lobby_xy", [1.0, 2.6])),
            "inside_xy": tuple(float(v) for v in _param("floor/elevator_inside_xy", [2.6, 2.6])),
            "service_timeout": float(_param("floor/service_timeout", 5.0)),
        }

    def _on_timer(self, _event) -> None:
        try:
            self.tick()
        except Exception as exc:
            rospy.logerr("danger_search 状态机异常：%s", exc)
            self.controller.stop()

    def tick(self) -> None:
        self._publish_status()
        if self.state == FINISH:
            self.controller.stop()
            return
        if self._elapsed() >= self.max_runtime:
            rospy.logwarn("接近时间预算，写出当前检测结果")
            self.state = WRITE_RESULT
        if self.state == WRITE_RESULT:
            self._write_result()
            self.state = FINISH
            return

        snapshot = self.sensor_buffer.snapshot()
        pose = odom_pose_xy_yaw(snapshot.odom_msg)
        xyz = odom_xyz(snapshot.odom_msg)
        if self.state == INIT:
            if pose is None or xyz is None:
                return
            self.home_pose = pose
            self.current_floor = self.mapper.estimate_floor_from_z(xyz[2])
            self.floor_start_time = time.time()
            self.state = WAIT_CONTROLLER_READY
            rospy.loginfo("记录起点：x=%.2f y=%.2f floor=%d", pose[0], pose[1], self.current_floor)
            return

        if pose is None or xyz is None:
            self.controller.stop()
            return

        self._update_mapping_and_detection(snapshot, pose, xyz)

        if self.state == WAIT_CONTROLLER_READY:
            self.controller.stop()
            if self._elapsed() >= self.startup_delay:
                self.state = EXPLORE_FLOOR
            return
        if self.state == EXPLORE_FLOOR:
            self._tick_explore(pose)
        elif self.state == CHANGE_FLOOR:
            self._tick_change_floor(pose)
        elif self.state == RETURN_HOME:
            self._tick_return_home(pose)

    def _update_mapping_and_detection(self, snapshot, pose, xyz) -> None:
        points = self.sensor_buffer.read_cloud_xyz(
            snapshot.cloud_msg,
            max_points=self.cloud_max_points,
            stride=self.cloud_stride,
        )
        self.mapper.update_from_cloud(floor_index=self.current_floor, robot_xyz=xyz, points=points)

        bundle = snapshot.image_bundle
        if bundle is None:
            return
        stamp = bundle.rgb_msg.header.stamp.to_sec() if hasattr(bundle.rgb_msg.header.stamp, "to_sec") else None
        if stamp is not None and self._last_image_stamp == stamp:
            return
        self._last_image_stamp = stamp
        camera_frame = bundle.rgb_msg.header.frame_id
        camera_to_world = self.sensor_buffer.camera_to_target_matrix(camera_frame, bundle.rgb_msg.header.stamp)
        if camera_to_world is not None and self.depth_frame_is_optical:
            camera_to_world = camera_to_world @ _ros_link_from_optical_matrix()
        candidates = self.detector.detect_messages(
            bundle.rgb_msg,
            bundle.depth_msg,
            bundle.camera_info_msg,
            camera_to_world,
            require_world_transform=True,
        )
        if candidates:
            self.tracker.update(candidates, now=time.time())

    def _tick_explore(self, pose: tuple[float, float, float]) -> None:
        grid = self.mapper.floor(self.current_floor)
        frontiers = self.explorer.extract_frontiers(grid)
        floor_elapsed = time.time() - self.floor_start_time
        if (
            floor_elapsed >= self.floor_timeout
            or (grid.coverage_ratio() >= self.explore_coverage_ratio and len(frontiers) <= self.min_frontier_count)
        ):
            self._finish_current_floor()
            return

        if not self.controller.has_path():
            frontier, path = self.explorer.select_frontier(grid, pose)
            if frontier is None or not path:
                if self._tick_no_frontier_scan(frontiers):
                    self._finish_current_floor()
                return
            self.current_frontier = frontier
            self.controller.set_path(path)
            self._no_frontier_scan_until = 0.0
            rospy.loginfo("选择前沿：floor=%d score=%.2f target=(%.2f, %.2f)",
                          self.current_floor, frontier.score, frontier.centroid[0], frontier.centroid[1])

        status = self.controller.tick(pose)
        if status == STUCK:
            if self.current_frontier is not None:
                self.explorer.record_failure(grid, self.current_frontier.centroid)
                self.mapper.mark_black(self.current_floor, self.current_frontier.centroid[0], self.current_frontier.centroid[1], 0.6)
            self.controller.recover()
            self.current_frontier = None
        elif status in (DONE, IDLE):
            self.current_frontier = None

    def _tick_no_frontier_scan(self, frontiers: list[Frontier]) -> bool:
        now = time.time()
        if frontiers:
            self._no_frontier_scan_until = 0.0
            return True
        if self._no_frontier_scan_until <= 0.0:
            self._no_frontier_scan_until = now + self.no_frontier_scan_duration
            rospy.loginfo("当前无前沿，原地扫描 %.1f 秒后再判断", self.no_frontier_scan_duration)
        if now < self._no_frontier_scan_until:
            self.controller.rotate_in_place(0.45)
            return False
        self.controller.stop()
        self._no_frontier_scan_until = 0.0
        return True

    def _finish_current_floor(self) -> None:
        self.controller.clear_path()
        if self.current_floor + 1 < self.max_floor:
            self._begin_floor_change(self.current_floor + 1, return_after=False)
        else:
            self.state = RETURN_HOME

    def _tick_return_home(self, pose: tuple[float, float, float]) -> None:
        if self.home_pose is None:
            self.state = WRITE_RESULT
            return
        if self.current_floor != 0 and self.max_floor > 1:
            self._begin_floor_change(0, return_after=True)
            return
        distance = math.hypot(self.home_pose[0] - pose[0], self.home_pose[1] - pose[1])
        if distance <= self.home_tolerance:
            self.state = WRITE_RESULT
            return
        if not self.controller.has_path():
            grid = self.mapper.floor(self.current_floor)
            path = astar_path(grid, (pose[0], pose[1]), (self.home_pose[0], self.home_pose[1]))
            if not path:
                path = [(pose[0], pose[1]), (self.home_pose[0], self.home_pose[1])]
            self.controller.set_path(path)
        status = self.controller.tick(pose)
        if status == STUCK:
            self.controller.recover()

    def _begin_floor_change(self, target_floor: int, *, return_after: bool) -> None:
        if not self.floor_manager.services_ready():
            rospy.logwarn("门/电梯服务不可用，停止探索并写出已有结果")
            self.state = WRITE_RESULT
            return
        self.target_floor = int(target_floor)
        self.return_after_floor_change = bool(return_after)
        self._change_phase = "GO_LOBBY"
        self._service_task = None
        self.controller.clear_path()
        self.state = CHANGE_FLOOR
        rospy.loginfo("准备换层：%d -> %d", self.current_floor, self.target_floor)

    def _tick_change_floor(self, pose: tuple[float, float, float]) -> None:
        if self.target_floor is None or self._change_phase is None:
            self.state = RETURN_HOME if self.return_after_floor_change else EXPLORE_FLOOR
            return

        if self._change_phase == "GO_LOBBY":
            if self._navigate_to(pose, self.floor_manager.targets.lobby_xy):
                self._change_phase = "OPEN_CURRENT"
            return
        if self._change_phase == "OPEN_CURRENT":
            if self._run_service_once(lambda: self.floor_manager.set_door_async(self.current_floor, True)):
                self._change_phase = "ENTER"
            return
        if self._change_phase == "ENTER":
            if self._navigate_to(pose, self.floor_manager.targets.inside_xy):
                self._change_phase = "CLOSE_CURRENT"
            return
        if self._change_phase == "CLOSE_CURRENT":
            if self._run_service_once(lambda: self.floor_manager.set_door_async(self.current_floor, False)):
                self._change_phase = "CALL_ELEVATOR"
            return
        if self._change_phase == "CALL_ELEVATOR":
            if self._run_service_once(lambda: self.floor_manager.call_elevator_async(self.target_floor, False)):
                self._change_phase = "OPEN_TARGET"
            return
        if self._change_phase == "OPEN_TARGET":
            if self._run_service_once(lambda: self.floor_manager.set_door_async(self.target_floor, True)):
                self.current_floor = self.target_floor
                self.floor_start_time = time.time()
                self.controller.clear_path()
                self._change_phase = "EXIT"
            return
        if self._change_phase == "EXIT":
            if self._navigate_to(pose, self.floor_manager.targets.lobby_xy):
                self._service_task = None
                self._change_phase = None
                self.target_floor = None
                self.state = RETURN_HOME if self.return_after_floor_change else EXPLORE_FLOOR

    def _navigate_to(self, pose: tuple[float, float, float], target_xy: tuple[float, float]) -> bool:
        if math.hypot(target_xy[0] - pose[0], target_xy[1] - pose[1]) <= self.controller.goal_tolerance:
            self.controller.clear_path()
            return True
        if not self.controller.has_path():
            grid = self.mapper.floor(self.current_floor)
            path = astar_path(grid, (pose[0], pose[1]), target_xy)
            if not path:
                path = [(pose[0], pose[1]), target_xy]
            self.controller.set_path(path)
        status = self.controller.tick(pose)
        if status == STUCK:
            self.controller.recover()
        return status == DONE

    def _run_service_once(self, task_factory) -> bool:
        if self._service_task is None:
            self._service_task = task_factory()
            return False
        if not self._service_task.done():
            self.controller.stop()
            return False
        ok = self._service_task.ok()
        message = self._service_task.message()
        if not ok:
            rospy.logwarn("换层服务失败：%s", message)
            self.state = WRITE_RESULT
            return False
        self._service_task = None
        return True

    def _write_result(self) -> None:
        positions = self.tracker.confirmed_positions(now=time.time())
        path = self.result_writer.write(self._elapsed(), positions)
        self.controller.stop()
        self._finished = True
        rospy.loginfo("已写出危险源检测结果：%s，数量=%d", path, len(positions))

    def _publish_status(self) -> None:
        if self._status_publisher is None or String is None:
            return
        now = time.time()
        if now - self._last_status_publish < self.status_publish_period:
            return
        self._last_status_publish = now
        confirmed_count = len(self.tracker.confirmed_positions(now=now))
        payload = {
            "state": self.state,
            "floor": self.current_floor,
            "elapsed": round(self._elapsed(), 2),
            "confirmed_count": confirmed_count,
            "track_count": len(self.tracker.tracks),
            "has_path": self.controller.has_path(),
            "change_phase": self._change_phase,
        }
        self._status_publisher.publish(String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))

    def _elapsed(self) -> float:
        return time.time() - self.start_time


def _param(path: str, default):
    private_name = "~" + path
    global_name = "/" + path
    if rospy.has_param(private_name):
        return rospy.get_param(private_name)
    if rospy.has_param(global_name):
        return rospy.get_param(global_name)
    return default


def _ros_link_from_optical_matrix() -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=float,
    )
    return matrix


def main() -> int:
    if rospy is None:
        raise RuntimeError("当前环境没有 rospy，无法运行 danger_search_node")
    rospy.init_node("danger_search")
    manager = MissionManager()
    manager.start()
    rospy.spin()
    return 0
