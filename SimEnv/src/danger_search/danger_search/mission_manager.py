"""danger_search 主任务状态机。"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time

import numpy as np

try:
    import rospy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid
except ImportError:  # pragma: no cover - 离线环境可能没有 ROS
    rospy = None
    String = None
    PoseStamped = None
    OccupancyGrid = None

from danger_search.detection_tracker import DetectionTracker
from danger_search.building_control_client import BuildingControlClient
from danger_search.navigation_tasks import NavigationTask, TakeElevator, OpenDoorAndPass, NavigateToPose, FAILED
from danger_search.frontier import Frontier, FrontierExplorer
from danger_search.local_semantic_detector import DoorCandidate, LocalSemanticDetector
from danger_search.navigator import DONE, IDLE, STUCK, PathFollower
from danger_search.planner import astar_path
from danger_search.mapper import MultiFloorGridMap
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
PROBE_LATERAL_OFFSETS = (0.0, -1.0, 1.0, -2.0, 2.0)


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
        self._probe_attempt = 0
        self._timer = None
        self._finished = False
        self._shutdown_requested = False
        self._return_home_success = False
        self._elevator_success: bool | None = None
        self._timeout_triggered = False
        self._failure_reasons: list[str] = []
        self._last_pose: tuple[float, float, float] | None = None
        self._max_home_distance = 0.0
        self.manual_goal: tuple[float, float] | None = None
        self.manual_mode = False
        self.current_task: NavigationTask | None = None
        self.opened_doors: set[str] = set()
        self.failed_doors: set[str] = set()
        self.active_door: str | None = None
        self._latest_cloud_points: list[tuple[float, float, float]] = []

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
        self.local_detector = LocalSemanticDetector(**self.local_detection_params)
        self.tracker = DetectionTracker(**self.tracker_params)
        metadata_file = os.path.abspath(os.path.join(os.path.dirname(self.result_file), "../generated_building/layout_metadata.json"))
        self.building_client = BuildingControlClient(
            metadata_path=metadata_file,
            service_timeout=self.floor_params.get("service_timeout", 5.0),
        )
        self.result_writer = ResultWriter(self.result_file)

    def start(self) -> None:
        self.sensor_buffer.start()
        self.controller.start()
        if self.max_floor > 1 or self.building_client.has_controllable_doors():
            self.building_client.start(connect_elevator=self.max_floor > 1)
        self._status_publisher = rospy.Publisher(self.status_topic, String, queue_size=1)
        if OccupancyGrid is not None:
            self._map_publisher = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=1, latch=True)
        else:
            self._map_publisher = None
        if PoseStamped is not None:
            self._goal_subscriber = rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_callback)
        self._timer = rospy.Timer(rospy.Duration(0.1), self._on_timer)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("danger_search 已启动，等待传感器数据")

    def _goal_callback(self, msg) -> None:
        self.manual_mode = True
        self.manual_goal = (msg.pose.position.x, msg.pose.position.y)
        self.current_task = NavigateToPose(self.manual_goal)

        self.controller.clear_path()
        self.current_frontier = None
        rospy.loginfo("收到新的手动导航目标点: (%.2f, %.2f)，已切换到手动目标模式！", self.manual_goal[0], self.manual_goal[1])

    def shutdown(self) -> None:
        self.controller.stop(repeat=5)
        if not self._finished:
            self._write_result()

    def _load_params(self) -> None:
        self.cmd_vel_topic = _param("topics/cmd_vel", "/cmd_vel")
        self.odom_topic = _param("topics/odom", "/Odometry_gazebo")
        self.cloud_topic = _param("topics/cloud", "/livox/Pointcloud2")
        self.rgb_topic = _param("topics/rgb", "/real_sense/rgb/image_raw")
        self.depth_topic = _param("topics/depth", "/real_sense/depth/image_raw")
        self.camera_info_topic = _param("topics/camera_info", "/real_sense/rgb/camera_info")
        self.status_topic = _param("topics/status", "/nav/status")
        self.goal_topic = _param("topics/goal", "/nav/goal")
        self.map_topic = _param("topics/map", "/nav/map_debug")

        self.startup_delay = float(_param("mission/startup_delay", 2.0))
        self.max_runtime = float(_param("mission/max_runtime", 590.0))
        self.max_floor = int(_param("mission/max_floor", 1))
        self.floor_timeout = float(_param("mission/floor_timeout", 180.0))
        self.min_floor_explore_time = float(_param("mission/min_floor_explore_time", 20.0))
        self.min_floor_travel_distance = float(_param("mission/min_floor_travel_distance", 6.0))
        self.home_tolerance = float(_param("mission/home_tolerance", 0.7))
        self.explore_coverage_ratio = float(_param("mission/explore_coverage_ratio", 0.82))
        self.min_frontier_count = int(_param("mission/min_frontier_count", 1))
        self.no_frontier_scan_duration = float(_param("mission/no_frontier_scan_duration", 8.0))
        self.probe_distance = float(_param("mission/probe_distance", 1.2))
        self.probe_yaw = float(_param("mission/probe_yaw", math.pi / 2.0))
        self.status_publish_period = float(_param("mission/status_publish_period", 1.0))
        self.result_file = str(_param("result_file", os.path.join(os.getcwd(), "results", "detected_danger.json")))
        self.mission_summary_file = str(_param("mission_summary_file", ""))

        self.map_params = {
            "resolution": float(_param("map/resolution", 0.10)),
            "width": float(_param("map/width", 28.0)),
            "length": float(_param("map/length", 48.0)),
            "origin_x": float(_param("map/origin_x", -14.0)),
            "origin_y": float(_param("map/origin_y", -6.0)),
            "floor_height": float(_param("map/floor_height", 2.6)),
            "obstacle_min_z": float(_param("map/obstacle_min_z", 0.15)),
            "obstacle_max_z": float(_param("map/obstacle_max_z", 1.40)),
            "obstacle_inflation_radius": float(_param("map/obstacle_inflation_radius", 0.25)),
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
            "max_linear": float(_param("controller/max_linear", 0.60)),
            "max_lateral": float(_param("controller/max_lateral", 0.25)),
            "max_angular": float(_param("controller/max_angular", 0.8)),
            "min_linear_fraction": float(_param("controller/min_linear_fraction", 0.15)),
            "goal_tolerance": float(_param("controller/goal_tolerance", 0.35)),
            "waypoint_tolerance": float(_param("controller/waypoint_tolerance", 0.25)),
            "pure_pursuit_lookahead": float(_param("controller/pure_pursuit_lookahead", 0.65)),
            "reverse_speed": float(_param("controller/reverse_speed", -0.18)),
            "recover_duration": float(_param("controller/recover_duration", 0.6)),
            "no_progress_timeout": float(_param("frontier/no_progress_timeout", 12.0)),
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
        self.local_detection_params = {
            "max_distance": float(_param("local_semantic/max_door_distance", 3.0)),
            "fov_half_angle": float(_param("local_semantic/door_fov_half_angle", 0.85)),
            "evidence_radius": float(_param("local_semantic/door_evidence_radius", 0.75)),
            "min_evidence_points": int(_param("local_semantic/min_door_evidence_points", 6)),
            "min_evidence_height": float(_param("local_semantic/min_door_evidence_height", 0.15)),
            "max_evidence_height": float(_param("local_semantic/max_door_evidence_height", 1.80)),
            "use_truth_proximity_detector": bool(_param("local_semantic/use_truth_proximity_detector", False)),
        }
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
            self._record_failure(f"状态机异常: {exc}")
            self.controller.stop()

    def tick(self) -> None:
        self._publish_status()
        if self.state == FINISH:
            self.controller.stop()
            self._request_shutdown("danger_search mission finished")
            return
        if self._elapsed() >= self.max_runtime:
            rospy.logwarn("接近时间预算，写出当前检测结果")
            self._timeout_triggered = True
            self._record_failure("探索超时")
            self.state = WRITE_RESULT
        if self.state == WRITE_RESULT:
            self._write_result()
            self.state = FINISH
            self._request_shutdown("danger_search mission finished")
            return

        snapshot = self.sensor_buffer.snapshot()
        pose = odom_pose_xy_yaw(snapshot.odom_msg)
        xyz = odom_xyz(snapshot.odom_msg)
        if pose is None or xyz is None:
            self.controller.stop()
            return

        self._update_pose_progress(pose)
        self._update_mapping_and_detection(snapshot, pose, xyz)

        if self.current_task is not None:
            status = self.current_task.tick(self, pose)
            if status in (DONE, FAILED):
                if isinstance(self.current_task, OpenDoorAndPass):
                    if status == DONE:
                        self.opened_doors.add(self.current_task.door_id)
                    else:
                        self.failed_doors.add(self.current_task.door_id)
                    self.active_door = None
                if status == FAILED:
                    rospy.logwarn("当前任务 %s 失败，放弃该门/任务", self.current_task.name)

                if isinstance(self.current_task, TakeElevator) and status == DONE:
                    self.current_floor = self.current_task.target_floor
                    self.controller.clear_path()
                    self.floor_start_time = time.time()

                self.current_task = None

                if self.state == CHANGE_FLOOR:
                    self.state = RETURN_HOME if self.return_after_floor_change else EXPLORE_FLOOR
            return

        if self.state == INIT:
            if pose is None or xyz is None:
                return
            self.home_pose = pose
            self.current_floor = self.mapper.estimate_floor_from_z(xyz[2])
            self.floor_start_time = time.time()
            self.state = WAIT_CONTROLLER_READY
            rospy.loginfo("记录起点：x=%.2f y=%.2f floor=%d", pose[0], pose[1], self.current_floor)
            return

        if self.state == WAIT_CONTROLLER_READY:
            self.controller.stop()
            if self._elapsed() >= self.startup_delay:
                self.state = EXPLORE_FLOOR
            return
        if self.state == EXPLORE_FLOOR:
            self._tick_explore(pose)

        elif self.state == RETURN_HOME:
            self._tick_return_home(pose)

    def _update_mapping_and_detection(self, snapshot, pose, xyz) -> None:
        points = self.sensor_buffer.read_cloud_xyz(
            snapshot.cloud_msg,
            max_points=self.cloud_max_points,
            stride=self.cloud_stride,
        )
        self._latest_cloud_points = points
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
        if self.manual_mode:
            if self.current_task is None:
                self.controller.stop()
            return

        frontiers = self.explorer.extract_frontiers(grid)
        floor_elapsed = time.time() - self.floor_start_time
        if (
            floor_elapsed >= self.floor_timeout
            or (
                floor_elapsed >= self.min_floor_explore_time
                and self._max_home_distance >= self.min_floor_travel_distance
                and grid.coverage_ratio() >= self.explore_coverage_ratio
                and len(frontiers) <= self.min_frontier_count
            )
        ):
            self._finish_current_floor()
            return

        candidate = self.local_detector.detect_controllable_door(
            robot_pose=pose,
            floor_index=self.current_floor,
            cloud_points=self._latest_cloud_points,
            building_client=self.building_client,
            floor_height=self.mapper.floor_height,
            allow_elevator=False,
        )
        if self._should_open_door(candidate):
            assert candidate is not None
            self.controller.clear_path()
            self.current_frontier = None
            self.active_door = candidate.door_id
            self.current_task = OpenDoorAndPass(candidate.door_id)
            rospy.loginfo(
                "局部检测到可控动态门：door_id=%s distance=%.2f source=%s evidence=%d",
                candidate.door_id,
                candidate.distance,
                candidate.source,
                candidate.evidence_count,
            )
            return

        if not self.controller.has_path():
            frontier, path = self.explorer.select_frontier(grid, pose)
            if frontier is None or not path:
                if not frontiers and self._try_probe_unknown_space(pose):
                    return
                if self._tick_no_frontier_scan(frontiers):
                    if floor_elapsed >= self.min_floor_explore_time:
                        self._finish_current_floor()
                    else:
                        rospy.loginfo(
                            "楼层探索时间 %.1f 秒，小于最小值 %.1f 秒，继续扫描",
                            floor_elapsed,
                            self.min_floor_explore_time,
                        )
                return
            self.current_frontier = frontier
            self.controller.set_path(path)
            self._no_frontier_scan_until = 0.0
            self._probe_attempt = 0
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

    def _try_probe_unknown_space(self, pose: tuple[float, float, float]) -> bool:
        grid = self.mapper.floor(self.current_floor)
        base_pose = (pose[0], pose[1], self.probe_yaw)
        for _ in range(12):
            target = probe_target_for_attempt(base_pose, self.probe_distance, self._probe_attempt)
            self._probe_attempt += 1
            target_cell = grid.world_to_grid(target[0], target[1])
            if target_cell is None:
                continue
            cell_value = int(grid.data[target_cell[1], target_cell[0]])
            if cell_value in (80, 100):
                continue
            if segment_crosses_blocked_cell(grid, (pose[0], pose[1]), target):
                continue
            self.current_frontier = None
            self.controller.set_path([(pose[0], pose[1]), target])
            rospy.loginfo("无前沿，执行短距离探测目标：target=(%.2f, %.2f)", target[0], target[1])
            return True
        return False

    def _tick_no_frontier_scan(self, frontiers: list[Frontier]) -> bool:
        now = time.time()
        if self._no_frontier_scan_until <= 0.0:
            self._no_frontier_scan_until = now + self.no_frontier_scan_duration
            if frontiers:
                rospy.loginfo("当前无可达前沿，原地扫描 %.1f 秒后再判断", self.no_frontier_scan_duration)
            else:
                rospy.loginfo("当前无前沿，原地扫描 %.1f 秒后再判断", self.no_frontier_scan_duration)
        if now < self._no_frontier_scan_until:
            self.controller.rotate_in_place(0.45)
            return False
        self.controller.stop()
        self._no_frontier_scan_until = 0.0
        if frontiers:
            self._record_failure("无可达前沿")
            rospy.logwarn("扫描后仍无可达前沿，结束当前楼层")
        return True

    def _should_open_door(self, candidate: DoorCandidate | None) -> bool:
        if candidate is None:
            return False
        if self.state != EXPLORE_FLOOR:
            return False
        if self.current_task is not None:
            return False
        if candidate.kind != "dynamic_door":
            return False
        if candidate.door_id in self.opened_doors:
            return False
        if candidate.door_id in self.failed_doors:
            return False
        if candidate.door_id == self.active_door:
            return False
        if self.building_client.is_elevator_door(candidate.door_id):
            return False
        if candidate.distance >= self.local_detector.max_distance:
            return False
        if not candidate.visible:
            return False
        if not self.building_client.door_service_ready():
            self._record_failure("门服务不可用")
            return False
        return True

    def _finish_current_floor(self) -> None:
        self.controller.clear_path()
        if self.current_floor + 1 < self.max_floor:
            self._begin_floor_change(self.current_floor + 1, return_after=False)
        else:
            self.state = RETURN_HOME

    def _tick_return_home(self, pose: tuple[float, float, float]) -> None:
        if self.home_pose is None:
            self._record_failure("缺少返航起点")
            self.state = WRITE_RESULT
            return
        if self.current_floor != 0 and self.max_floor > 1:
            self._begin_floor_change(0, return_after=True)
            return
        distance = math.hypot(self.home_pose[0] - pose[0], self.home_pose[1] - pose[1])
        if distance <= self.home_tolerance:
            self._return_home_success = True
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
        if not self.building_client.services_ready():
            rospy.logwarn("门/电梯服务不可用，停止探索并写出已有结果")
            self._elevator_success = False
            self._record_failure("门/电梯服务不可用")
            self.state = WRITE_RESULT
            return
        if self._elevator_success is None:
            self._elevator_success = True
        self.target_floor = int(target_floor)
        self.return_after_floor_change = bool(return_after)
        self.controller.clear_path()
        self.state = CHANGE_FLOOR
        self.current_task = TakeElevator(self.current_floor, target_floor, elevator_id=self.floor_params.get("elevator_id", "elevator_main"))
        rospy.loginfo("准备换层：%d -> %d", self.current_floor, self.target_floor)

    def _write_result(self) -> None:
        positions = self.tracker.confirmed_positions(now=time.time())
        path = self.result_writer.write(self._elapsed(), positions)
        self.controller.stop(repeat=5)
        self._finished = True
        self._write_mission_summary(path, len(positions))
        rospy.loginfo("已写出危险源检测结果：%s，数量=%d", path, len(positions))

    def _publish_status(self) -> None:
        if self._status_publisher is None or String is None:
            return
        now = time.time()
        if now - self._last_status_publish < self.status_publish_period:
            return
        self._last_status_publish = now

        grid = self.mapper.floor(self.current_floor)
        if self._map_publisher is not None:
            msg = OccupancyGrid()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "odom"
            msg.info.resolution = grid.resolution
            msg.info.width = grid.size_x
            msg.info.height = grid.size_y
            msg.info.origin.position.x = grid.origin_x
            msg.info.origin.position.y = grid.origin_y
            msg.info.origin.position.z = self.current_floor * self.mapper.floor_height
            msg.info.origin.orientation.w = 1.0
            msg.data = grid.data.flatten().tolist()
            self._map_publisher.publish(msg)

        confirmed_count = len(self.tracker.confirmed_positions(now=now))
        payload = {
            "state": self.state,
            "floor": self.current_floor,
            "elapsed": round(self._elapsed(), 2),
            "confirmed_count": confirmed_count,
            "track_count": len(self.tracker.tracks),
            "current_task": self.current_task.name if self.current_task else None,
            "task_phase": getattr(self.current_task, "phase", None) if self.current_task else None,
            "coverage_ratio": round(grid.coverage_ratio(), 3),
            "current_goal": [round(self.manual_goal[0], 2), round(self.manual_goal[1], 2)] if self.manual_goal else None,
            "path_length": round(self.controller._last_goal_distance, 2) if self.controller.has_path() else 0.0,
            "is_stuck": self.controller._recover_until > now,
        }
        self._status_publisher.publish(String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))

    def _elapsed(self) -> float:
        return time.time() - self.start_time

    def _record_failure(self, reason: str) -> None:
        if reason and reason not in self._failure_reasons:
            self._failure_reasons.append(reason)

    def _update_pose_progress(self, pose: tuple[float, float, float]) -> None:
        self._last_pose = pose
        if self.home_pose is None:
            return
        self._max_home_distance = max(
            self._max_home_distance,
            math.hypot(pose[0] - self.home_pose[0], pose[1] - self.home_pose[1]),
        )

    def _request_shutdown(self, reason: str) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        rospy.signal_shutdown(reason)

    def _write_mission_summary(self, result_path: Path, detected_count: int) -> None:
        if not self.mission_summary_file:
            return
        summary_path = Path(self.mission_summary_file)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "exploration_time": round(self._elapsed(), 2),
            "state": FINISH if self._finished else self.state,
            "current_floor": self.current_floor,
            "max_floor": self.max_floor,
            "return_home_success": self._return_home_success,
            "elevator_success": self._elevator_success,
            "timeout_triggered": self._timeout_triggered,
            "failure_reason": list(self._failure_reasons),
            "detected_count": int(detected_count),
            "track_count": len(self.tracker.tracks),
            "confirmed_count": int(detected_count),
            "final_pose": list(self._last_pose) if self._last_pose is not None else None,
            "max_home_distance": round(self._max_home_distance, 3),
            "controller_debug": self.controller.command_debug_summary(),
            "result_file": str(result_path),
        }
        summary_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _param(path: str, default):
    node_name = rospy.get_name().rstrip("/")
    candidate_names = [
        f"{node_name}/{path}",
        "~" + path,
        "/" + path,
    ]
    for name in candidate_names:
        if rospy.has_param(name):
            return rospy.get_param(name)
    return default


def probe_target_for_attempt(
    base_pose: tuple[float, float, float],
    probe_distance: float,
    attempt: int,
) -> tuple[float, float]:
    """按固定走廊扫描序列生成短距离探测目标，避免无前沿时只在起点附近抖动。"""
    distance = max(float(probe_distance), 0.05)
    attempt_index = max(int(attempt), 0)
    lane_index = attempt_index % len(PROBE_LATERAL_OFFSETS)
    forward_step = attempt_index // len(PROBE_LATERAL_OFFSETS) + 1
    forward = (math.cos(base_pose[2]), math.sin(base_pose[2]))
    lateral = (math.cos(base_pose[2] + math.pi / 2.0), math.sin(base_pose[2] + math.pi / 2.0))
    lateral_offset = PROBE_LATERAL_OFFSETS[lane_index] * distance
    forward_distance = forward_step * distance
    return (
        base_pose[0] + forward_distance * forward[0] + lateral_offset * lateral[0],
        base_pose[1] + forward_distance * forward[1] + lateral_offset * lateral[1],
    )


def segment_crosses_blocked_cell(
    grid,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
) -> bool:
    start_cell = grid.world_to_grid(start_xy[0], start_xy[1])
    end_cell = grid.world_to_grid(end_xy[0], end_xy[1])
    if start_cell is None or end_cell is None:
        return True
    distance = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
    step_count = max(1, int(math.ceil(distance / max(grid.resolution * 0.5, 1e-6))))
    for index in range(1, step_count + 1):
        ratio = index / step_count
        x = start_xy[0] + (end_xy[0] - start_xy[0]) * ratio
        y = start_xy[1] + (end_xy[1] - start_xy[1]) * ratio
        cell = grid.world_to_grid(x, y)
        if cell is None:
            return True
        gx, gy = cell
        if int(grid.data[gy, gx]) in (80, 100):
            return True
    return False


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
