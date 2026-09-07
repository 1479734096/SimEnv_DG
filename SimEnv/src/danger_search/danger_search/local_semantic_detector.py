"""从局部点云提取门洞和电梯轿厢几何。

正式路径只使用运行期间的局部传感器观测，不读取 metadata。门洞由同一深度
平面上的左右竖直结构确定；电梯候选还要求门后同时存在两侧壁和后壁，减少把
普通长走廊误认作轿厢。只有显式诊断配置才允许使用已有空间门信息做邻近回退。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from danger_search.building_control_client import BuildingControlClient, DoorInfo


@dataclass(frozen=True)
class DoorCandidate:
    """已关联到公开服务 ID 的局部门候选及其证据来源。"""
    door_id: str
    distance: float
    visible: bool
    kind: str
    floor_index: int
    world_xy: tuple[float, float]
    source: str
    evidence_count: int = 0


@dataclass(frozen=True)
class OpeningObservation:
    """由局部点云直接观测到的开放门洞几何。"""

    distance: float
    world_xy: tuple[float, float]
    yaw: float
    width: float
    evidence_count: int


class LocalSemanticDetector:
    """用轻量几何规则检测机器人附近的门洞与轿厢结构。

    本类只做观测解释，不调用门/电梯服务。门洞跨帧确认由独立运行时负责，
    ``MissionManager`` 只绑定公开 ID 并创建任务，防止单帧候选直接触发环境交互。
    """

    # 左右门框分别量化到该横向格，二者之差的最大欠估误差也是一个格宽。
    opening_width_resolution = 0.05

    def __init__(
        self,
        *,
        max_distance: float = 3.0,
        fov_half_angle: float = 0.85,
        evidence_radius: float = 0.75,
        min_evidence_points: int = 6,
        min_evidence_height: float = 0.15,
        max_evidence_height: float = 1.80,
        opening_min_width: float = 0.65,
        opening_max_width: float = 2.50,
        opening_plane_depth: float = 0.30,
        opening_min_side_height_span: float = 0.25,
        use_truth_proximity_detector: bool = False,
    ):
        self.max_distance = float(max_distance)
        self.fov_half_angle = float(fov_half_angle)
        self.evidence_radius = float(evidence_radius)
        self.min_evidence_points = int(min_evidence_points)
        self.min_evidence_height = float(min_evidence_height)
        self.max_evidence_height = float(max_evidence_height)
        self.opening_min_width = float(opening_min_width)
        self.opening_max_width = float(opening_max_width)
        self.opening_plane_depth = float(opening_plane_depth)
        self.opening_min_side_height_span = float(opening_min_side_height_span)
        self.use_truth_proximity_detector = bool(use_truth_proximity_detector)
        if (
            not all(
                math.isfinite(value) and value > 0.0
                for value in (
                    self.max_distance,
                    self.evidence_radius,
                    self.max_evidence_height,
                    self.opening_min_width,
                    self.opening_max_width,
                    self.opening_plane_depth,
                    self.opening_min_side_height_span,
                )
            )
            or not math.isfinite(self.fov_half_angle)
            or not 0.0 < self.fov_half_angle <= math.pi
            or self.min_evidence_points <= 0
            or not math.isfinite(self.min_evidence_height)
            or self.min_evidence_height < 0.0
            or self.min_evidence_height >= self.max_evidence_height
            or self.opening_min_width >= self.opening_max_width
        ):
            raise ValueError("本地语义距离、视场、证据高度或门洞参数无效")

    def detect_opening_ahead(
        self,
        *,
        robot_pose: tuple[float, float, float],
        floor_index: int,
        cloud_points: Iterable[tuple[float, float, float]],
        floor_height: float = 2.6,
        minimum_distance: float = 0.15,
        maximum_distance: float | None = None,
        min_side_height_span: float | None = None,
        min_evidence_points: int | None = None,
        expected_center_xy: tuple[float, float] | None = None,
        maximum_center_error: float | None = None,
        min_opening_width: float | None = None,
        max_opening_width: float | None = None,
    ) -> OpeningObservation | None:
        """从前方两侧竖直结构之间确认门洞，不依赖门坐标真值。"""
        minimum_distance = float(minimum_distance)
        maximum_distance = (
            self.max_distance
            if maximum_distance is None
            else min(self.max_distance, float(maximum_distance))
        )
        if (
            not math.isfinite(minimum_distance)
            or not math.isfinite(maximum_distance)
            or minimum_distance < 0.0
            or maximum_distance <= minimum_distance
        ):
            raise ValueError("门洞检测距离窗口必须为有限递增范围")
        if expected_center_xy is not None and (
            not all(math.isfinite(value) for value in expected_center_xy)
            or maximum_center_error is None
            or not math.isfinite(maximum_center_error)
            or maximum_center_error <= 0.0
        ):
            raise ValueError("门洞预期中心必须有限且位置容差必须为正有限数")
        heading = (math.cos(robot_pose[2]), math.sin(robot_pose[2]))
        lateral_axis = (-heading[1], heading[0])
        floor_z = floor_index * floor_height
        samples: list[tuple[float, float, float]] = []
        for px, py, pz in cloud_points:
            z_rel = pz - floor_z
            if not self.min_evidence_height <= z_rel <= self.max_evidence_height:
                continue
            dx = px - robot_pose[0]
            dy = py - robot_pose[1]
            forward = dx * heading[0] + dy * heading[1]
            lateral = dx * lateral_axis[0] + dy * lateral_axis[1]
            if not minimum_distance <= forward <= maximum_distance:
                continue
            if abs(lateral) > self.opening_max_width:
                continue
            samples.append((forward, lateral, z_rel))
        if not samples:
            return None

        # 门板和门框会在相近前向距离形成左右两组竖直回波。按小窗口搜索
        # 最近一组，中心若仍有结构点则内缘间距会收缩并自然拒绝。
        search_step = max(0.05, self.opening_plane_depth / 3.0)
        candidate_distances = sorted(
            {
                round(forward / search_step) * search_step
                for forward, _lateral, _height in samples
            }
        )
        half_depth = max(0.05, self.opening_plane_depth * 0.5)
        parsed_min_pts = (
            int(min_evidence_points)
            if min_evidence_points is not None
            else self.min_evidence_points
        )
        minimum_side_points = max(1, parsed_min_pts)
        side_height_threshold = (
            float(min_side_height_span)
            if min_side_height_span is not None
            else self.opening_min_side_height_span
        )
        observations: list[OpeningObservation] = []
        for distance in candidate_distances:
            plane = [sample for sample in samples if abs(sample[0] - distance) <= half_depth]
            lateral_bin_width = self.opening_width_resolution
            lateral_bins: dict[int, list[tuple[float, float, float]]] = {}
            for sample in plane:
                bucket = int(round(sample[1] / lateral_bin_width))
                lateral_bins.setdefault(bucket, []).append(sample)
            vertical_bins: list[tuple[float, list[tuple[float, float, float]]]] = []
            for bucket, bucket_samples in lateral_bins.items():
                if len(bucket_samples) < minimum_side_points:
                    continue
                height_span = max(sample[2] for sample in bucket_samples) - min(
                    sample[2] for sample in bucket_samples
                )
                if height_span < side_height_threshold:
                    continue
                vertical_bins.append((bucket * lateral_bin_width, bucket_samples))
            left = [item for item in vertical_bins if item[0] > 0.05]
            right = [item for item in vertical_bins if item[0] < -0.05]
            if not left or not right:
                continue
            left_inner, left_evidence = min(left, key=lambda item: item[0])
            right_inner, right_evidence = max(right, key=lambda item: item[0])
            eff_min_width = (
                float(min_opening_width)
                if min_opening_width is not None
                else self.opening_min_width
            )
            eff_max_width = (
                float(max_opening_width)
                if max_opening_width is not None
                else self.opening_max_width
            )
            if not eff_min_width <= width <= eff_max_width:
                continue
            lateral_center = 0.5 * (left_inner + right_inner)
            edge_evidence = left_evidence + right_evidence
            plane_distance = sum(sample[0] for sample in edge_evidence) / len(
                edge_evidence
            )
            world_xy = (
                robot_pose[0]
                + plane_distance * heading[0]
                + lateral_center * lateral_axis[0],
                robot_pose[1]
                + plane_distance * heading[1]
                + lateral_center * lateral_axis[1],
            )
            # 先筛身份再选最近候选，否则近处无关开口会遮住真正入口。
            if expected_center_xy is not None and math.hypot(
                world_xy[0] - expected_center_xy[0],
                world_xy[1] - expected_center_xy[1],
            ) > maximum_center_error:
                continue
            left_forward = sum(sample[0] for sample in left_evidence) / len(left_evidence)
            right_forward = sum(sample[0] for sample in right_evidence) / len(right_evidence)
            d_forward = left_forward - right_forward
            d_lateral = left_inner - right_inner
            opening_yaw_offset = math.atan2(-d_forward, d_lateral)
            opening_yaw = math.atan2(
                math.sin(float(robot_pose[2]) + opening_yaw_offset),
                math.cos(float(robot_pose[2]) + opening_yaw_offset),
            )
            observations.append(
                OpeningObservation(
                    distance=plane_distance,
                    world_xy=world_xy,
                    yaw=opening_yaw,
                    width=width,
                    evidence_count=len(edge_evidence),
                )
            )
        if not observations:
            return None
        return min(observations, key=lambda observation: observation.distance)

    def detect_openings_ahead(
        self,
        *,
        robot_pose: tuple[float, float, float],
        floor_index: int,
        cloud_points: Iterable[tuple[float, float, float]],
        floor_height: float = 2.6,
        minimum_distance: float = 0.15,
        maximum_distance: float | None = None,
    ) -> list[OpeningObservation]:
        """返回前方所有有效门洞观测列表，按距离由近及远排序。"""
        minimum_distance = float(minimum_distance)
        maximum_distance = (
            self.max_distance
            if maximum_distance is None
            else min(self.max_distance, float(maximum_distance))
        )
        if (
            not math.isfinite(minimum_distance)
            or not math.isfinite(maximum_distance)
            or minimum_distance < 0.0
            or maximum_distance <= minimum_distance
        ):
            raise ValueError("门洞检测距离窗口必须为有限递增范围")
        heading = (math.cos(robot_pose[2]), math.sin(robot_pose[2]))
        lateral_axis = (-heading[1], heading[0])
        floor_z = floor_index * floor_height
        samples: list[tuple[float, float, float]] = []
        for px, py, pz in cloud_points:
            z_rel = pz - floor_z
            if not self.min_evidence_height <= z_rel <= self.max_evidence_height:
                continue
            dx = px - robot_pose[0]
            dy = py - robot_pose[1]
            forward = dx * heading[0] + dy * heading[1]
            lateral = dx * lateral_axis[0] + dy * lateral_axis[1]
            if not minimum_distance <= forward <= maximum_distance:
                continue
            if abs(lateral) > self.opening_max_width:
                continue
            samples.append((forward, lateral, z_rel))
        if not samples:
            return []

        search_step = max(0.05, self.opening_plane_depth / 3.0)
        candidate_distances = sorted(
            {
                round(forward / search_step) * search_step
                for forward, _lateral, _height in samples
            }
        )
        half_depth = max(0.05, self.opening_plane_depth * 0.5)
        minimum_side_points = max(3, self.min_evidence_points)
        observations: list[OpeningObservation] = []
        for distance in candidate_distances:
            plane = [sample for sample in samples if abs(sample[0] - distance) <= half_depth]
            lateral_bin_width = self.opening_width_resolution
            lateral_bins: dict[int, list[tuple[float, float, float]]] = {}
            for sample in plane:
                bucket = int(round(sample[1] / lateral_bin_width))
                lateral_bins.setdefault(bucket, []).append(sample)
            vertical_bins: list[tuple[float, list[tuple[float, float, float]]]] = []
            for bucket, bucket_samples in lateral_bins.items():
                if len(bucket_samples) < minimum_side_points:
                    continue
                height_span = max(sample[2] for sample in bucket_samples) - min(
                    sample[2] for sample in bucket_samples
                )
                if height_span < self.opening_min_side_height_span:
                    continue
                vertical_bins.append((bucket * lateral_bin_width, bucket_samples))
            left = [item for item in vertical_bins if item[0] > 0.05]
            right = [item for item in vertical_bins if item[0] < -0.05]
            if not left or not right:
                continue
            left_inner, left_evidence = min(left, key=lambda item: item[0])
            right_inner, right_evidence = max(right, key=lambda item: item[0])
            width = left_inner - right_inner
            if not self.opening_min_width <= width <= self.opening_max_width:
                continue
            lateral_center = 0.5 * (left_inner + right_inner)
            edge_evidence = left_evidence + right_evidence
            plane_distance = sum(sample[0] for sample in edge_evidence) / len(
                edge_evidence
            )
            world_xy = (
                robot_pose[0]
                + plane_distance * heading[0]
                + lateral_center * lateral_axis[0],
                robot_pose[1]
                + plane_distance * heading[1]
                + lateral_center * lateral_axis[1],
            )
            left_forward = sum(sample[0] for sample in left_evidence) / len(left_evidence)
            right_forward = sum(sample[0] for sample in right_evidence) / len(right_evidence)
            d_forward = left_forward - right_forward
            d_lateral = left_inner - right_inner
            opening_yaw_offset = math.atan2(-d_forward, d_lateral)
            opening_yaw = math.atan2(
                math.sin(float(robot_pose[2]) + opening_yaw_offset),
                math.cos(float(robot_pose[2]) + opening_yaw_offset),
            )
            observations.append(
                OpeningObservation(
                    distance=plane_distance,
                    world_xy=world_xy,
                    yaw=opening_yaw,
                    width=width,
                    evidence_count=len(edge_evidence),
                )
            )
        return sorted(observations, key=lambda observation: observation.distance)

    def detect_elevator_opening_ahead(
        self,
        *,
        robot_pose: tuple[float, float, float],
        floor_index: int,
        cloud_points: Iterable[tuple[float, float, float]],
        floor_height: float = 2.6,
        minimum_cabin_depth: float = 0.8,
        maximum_cabin_depth: float = 2.8,
    ) -> OpeningObservation | None:
        """确认门洞后方存在受侧壁和后壁约束的小型轿厢空间。"""
        points = list(cloud_points)
        opening = self.detect_opening_ahead(
            robot_pose=robot_pose,
            floor_index=floor_index,
            cloud_points=points,
            floor_height=floor_height,
        )
        if opening is None:
            return None

        heading = (math.cos(opening.yaw), math.sin(opening.yaw))
        lateral_axis = (-heading[1], heading[0])
        floor_z = floor_index * floor_height
        behind: list[tuple[float, float, float]] = []
        for px, py, pz in points:
            height = pz - floor_z
            if not self.min_evidence_height <= height <= self.max_evidence_height:
                continue
            dx = px - opening.world_xy[0]
            dy = py - opening.world_xy[1]
            forward = dx * heading[0] + dy * heading[1]
            lateral = dx * lateral_axis[0] + dy * lateral_axis[1]
            if 0.15 <= forward <= maximum_cabin_depth + 0.25:
                behind.append((forward, lateral, height))
        if not behind:
            return None

        depth_bin = 0.10
        depth_bins: dict[int, list[tuple[float, float, float]]] = {}
        for sample in behind:
            depth_bins.setdefault(int(round(sample[0] / depth_bin)), []).append(sample)
        back_candidates: list[tuple[float, list[tuple[float, float, float]]]] = []
        minimum_back_points = max(8, 2 * self.min_evidence_points)
        minimum_back_span = max(0.55, 0.45 * opening.width)
        for bucket, samples in depth_bins.items():
            depth = bucket * depth_bin
            if not minimum_cabin_depth <= depth <= maximum_cabin_depth:
                continue
            if len(samples) < minimum_back_points:
                continue
            span = max(sample[1] for sample in samples) - min(sample[1] for sample in samples)
            if span >= minimum_back_span:
                back_candidates.append((depth, samples))
        if not back_candidates:
            return None
        back_depth, back_evidence = min(back_candidates, key=lambda item: item[0])

        expected_half_width = max(0.25, 0.5 * opening.width)
        side_tolerance = max(0.12, min(0.25, 0.20 * opening.width))
        left_depths = [
            sample[0]
            for sample in behind
            if abs(sample[1] - expected_half_width) <= side_tolerance
            and sample[0] <= back_depth + depth_bin
        ]
        right_depths = [
            sample[0]
            for sample in behind
            if abs(sample[1] + expected_half_width) <= side_tolerance
            and sample[0] <= back_depth + depth_bin
        ]
        minimum_side_points = max(6, self.min_evidence_points)
        if len(left_depths) < minimum_side_points or len(right_depths) < minimum_side_points:
            return None
        if (
            max(left_depths) - min(left_depths) < 0.55
            or max(right_depths) - min(right_depths) < 0.55
        ):
            return None

        return OpeningObservation(
            distance=opening.distance,
            world_xy=opening.world_xy,
            yaw=opening.yaw,
            width=opening.width,
            evidence_count=(
                opening.evidence_count
                + len(back_evidence)
                + len(left_depths)
                + len(right_depths)
            ),
        )

    def detect_elevator_opening_nearby(
        self,
        *,
        robot_pose: tuple[float, float, float],
        floor_index: int,
        cloud_points: Iterable[tuple[float, float, float]],
        floor_height: float = 2.6,
        heading_count: int = 24,
    ) -> OpeningObservation | None:
        """利用 Livox 全向点云搜索近邻轿厢，不要求机身先朝向门洞。"""
        points = list(cloud_points)
        if heading_count < 4:
            raise ValueError("全向电梯检测至少需要四个朝向假设")
        observations: list[OpeningObservation] = []
        for index in range(int(heading_count)):
            yaw = -math.pi + 2.0 * math.pi * index / float(heading_count)
            observation = self.detect_elevator_opening_ahead(
                robot_pose=(robot_pose[0], robot_pose[1], yaw),
                floor_index=floor_index,
                cloud_points=points,
                floor_height=floor_height,
            )
            if observation is not None:
                observations.append(observation)
        if not observations:
            return None
        return min(observations, key=lambda observation: observation.distance)

    def detect_controllable_door(
        self,
        *,
        robot_pose: tuple[float, float, float],
        floor_index: int,
        cloud_points: Iterable[tuple[float, float, float]],
        building_client: BuildingControlClient,
        floor_height: float = 2.6,
        allow_elevator: bool = False,
    ) -> DoorCandidate | None:
        """在已注册的可控门中选择当前局部点云证据最接近的一扇。

        正式模式下空间门注册本身必须来自传感器；``use_truth_proximity_detector``
        仅供诊断，主状态机在 formal profile 会禁止启用该回退。
        """
        candidates: list[DoorCandidate] = []
        points = list(cloud_points)
        for door in building_client.controllable_doors(
            floor_index=floor_index,
            include_elevators=allow_elevator,
        ):
            if door.kind == "elevator" and not allow_elevator:
                continue
            distance = math.hypot(door.x - robot_pose[0], door.y - robot_pose[1])
            if distance > self.max_distance:
                continue
            visible = self._in_sensor_view(robot_pose, door)
            if not visible:
                continue
            evidence_count = self._count_local_structure_points(door, points, floor_index, floor_height)
            if evidence_count >= self.min_evidence_points:
                candidates.append(
                    DoorCandidate(
                        door_id=door.door_id,
                        distance=distance,
                        visible=True,
                        kind="dynamic_door" if door.kind != "elevator" else "elevator",
                        floor_index=door.floor_index,
                        world_xy=(door.x, door.y),
                        source="local_pointcloud",
                        evidence_count=evidence_count,
                    )
                )
            elif self.use_truth_proximity_detector:
                candidates.append(
                    DoorCandidate(
                        door_id=door.door_id,
                        distance=distance,
                        visible=True,
                        kind="dynamic_door" if door.kind != "elevator" else "elevator",
                        floor_index=door.floor_index,
                        world_xy=(door.x, door.y),
                        source="truth_proximity_debug",
                        evidence_count=evidence_count,
                    )
                )
        if not candidates:
            return None
        return min(candidates, key=lambda candidate: candidate.distance)

    def _in_sensor_view(self, robot_pose: tuple[float, float, float], door: DoorInfo) -> bool:
        dx = door.x - robot_pose[0]
        dy = door.y - robot_pose[1]
        bearing = math.atan2(dy, dx)
        return abs(_angle_diff(bearing, robot_pose[2])) <= self.fov_half_angle

    def _count_local_structure_points(
        self,
        door: DoorInfo,
        points: list[tuple[float, float, float]],
        floor_index: int,
        floor_height: float,
    ) -> int:
        count = 0
        floor_z = floor_index * floor_height
        for px, py, pz in points:
            if math.hypot(px - door.x, py - door.y) > self.evidence_radius:
                continue
            z_rel = pz - floor_z
            if self.min_evidence_height <= z_rel <= self.max_evidence_height:
                count += 1
        return count


def _angle_diff(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi
