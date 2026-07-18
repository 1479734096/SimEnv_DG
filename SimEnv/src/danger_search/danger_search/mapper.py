"""由配准点云增量维护多楼层 2.5D 占据栅格。

每个楼层使用相同 world XY 原点和分辨率，但持有独立二维数组。点云 Z 只用于
相对当前楼板筛选障碍高度，因此地图不表达完整三维几何。栅格状态含义为：
``UNKNOWN`` 未观测、``FREE`` 可通行、``VISITED`` 机器人已覆盖、``BLACK``
任务层临时/永久禁区、``OCC`` 实体障碍。

更新一帧时先写全部障碍命中，再执行点射线与可视扇区清空。这一顺序以及
“遇到 OCC/BLACK 即停止”的规则可防止稀疏回波、点顺序或墙后点把静态墙擦除。
动态门打开后的局部清理必须显式调用 ``clear_dynamic_door_strip``。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


UNKNOWN = -1
FREE = 0
VISITED = 30
BLACK = 80
OCC = 100


@dataclass
class _AngleBinObservation:
    point_count: int = 0
    nearest_obstacle_distance: float = math.inf


@dataclass(frozen=True)
class MapperDebugSnapshot:
    """一帧建图结束后的只读诊断摘要，不参与规划决策。"""
    floor_index: int
    point_ray_count: int
    sector_ray_count: int
    max_clearing_length: float
    blocked_stop_count: int
    occupied_to_free_count: int
    dynamic_door_cleared_count: int
    suspicious_clearing_prevented_count: int
    nearest_obstacle_distance: float | None
    known_bbox_fill_ratio: float
    traversable_known_ratio: float
    visited_traversable_ratio: float


@dataclass
class _MapperFrameStats:
    floor_index: int
    point_ray_count: int = 0
    sector_ray_count: int = 0
    max_clearing_length: float = 0.0
    blocked_stop_count: int = 0
    occupied_to_free_count: int = 0
    dynamic_door_cleared_count: int = 0
    suspicious_clearing_prevented_count: int = 0
    nearest_obstacle_distance: float = math.inf


@dataclass
class FloorGrid:
    """单层 world XY 与离散栅格之间的映射及基础标记操作。

    数组按 ``data[gy, gx]`` 存储；world 坐标落在格子中心时通过
    ``grid_to_world`` 返回。所有可通行判断统一只接受 FREE/VISITED，UNKNOWN、
    BLACK 和 OCC 均 fail-closed。
    """
    floor_index: int
    resolution: float
    width: float
    length: float
    origin_x: float
    origin_y: float

    def __post_init__(self) -> None:
        self.size_x = int(math.ceil(self.width / self.resolution))
        self.size_y = int(math.ceil(self.length / self.resolution))
        self.data = np.full((self.size_y, self.size_x), UNKNOWN, dtype=np.int16)

    def world_to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
            return gx, gy
        return None

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        return (
            self.origin_x + (gx + 0.5) * self.resolution,
            self.origin_y + (gy + 0.5) * self.resolution,
        )

    def is_traversable(self, gx: int, gy: int) -> bool:
        if not (0 <= gx < self.size_x and 0 <= gy < self.size_y):
            return False
        return int(self.data[gy, gx]) in (FREE, VISITED)

    def mark_free(self, gx: int, gy: int) -> None:
        """仅把 UNKNOWN 变为 FREE，绝不覆盖已知障碍或任务禁区。"""
        if 0 <= gx < self.size_x and 0 <= gy < self.size_y and self.data[gy, gx] == UNKNOWN:
            self.data[gy, gx] = FREE

    def mark_occupied(self, gx: int, gy: int) -> None:
        if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
            self.data[gy, gx] = OCC

    def mark_occupied_radius(self, x: float, y: float, radius: float) -> None:
        """把障碍命中按机器人安全余量膨胀为圆形 OCC 区域。"""
        center = self.world_to_grid(x, y)
        if center is None:
            return
        if radius <= 0.0:
            self.mark_occupied(*center)
            return
        cx, cy = center
        cells = int(math.ceil(radius / self.resolution))
        for gy in range(cy - cells, cy + cells + 1):
            for gx in range(cx - cells, cx + cells + 1):
                if not (0 <= gx < self.size_x and 0 <= gy < self.size_y):
                    continue
                wx, wy = self.grid_to_world(gx, gy)
                if math.hypot(wx - x, wy - y) <= radius:
                    self.data[gy, gx] = OCC

    def mark_visited_radius(self, x: float, y: float, radius: float) -> None:
        """记录机身实际覆盖区域；实体 OCC 始终拥有更高优先级。"""
        center = self.world_to_grid(x, y)
        if center is None:
            return
        cx, cy = center
        cells = int(math.ceil(radius / self.resolution))
        for gy in range(cy - cells, cy + cells + 1):
            for gx in range(cx - cells, cx + cells + 1):
                if not (0 <= gx < self.size_x and 0 <= gy < self.size_y):
                    continue
                wx, wy = self.grid_to_world(gx, gy)
                if math.hypot(wx - x, wy - y) <= radius and self.data[gy, gx] != OCC:
                    self.data[gy, gx] = VISITED

    def coverage_ratio(self) -> float:
        # 不使用真值：以当前观测到的地图包围盒为分母，衡量内部未知空洞是否已填充。
        return self.known_bbox_fill_ratio()

    def traversable_known_ratio(self) -> float:
        known = np.count_nonzero(self.data != UNKNOWN)
        if known == 0:
            return 0.0
        visited_or_free = np.count_nonzero((self.data == VISITED) | (self.data == FREE))
        return float(visited_or_free / known)

    def visited_traversable_ratio(self) -> float:
        traversable = np.count_nonzero((self.data == VISITED) | (self.data == FREE))
        if traversable == 0:
            return 0.0
        visited = np.count_nonzero(self.data == VISITED)
        return float(visited / traversable)

    def known_bbox_fill_ratio(self) -> float:
        known_cells = np.argwhere(self.data != UNKNOWN)
        if known_cells.size == 0:
            return 0.0
        min_y, min_x = known_cells.min(axis=0)
        max_y, max_x = known_cells.max(axis=0)
        bbox_cells = (int(max_x) - int(min_x) + 1) * (int(max_y) - int(min_y) + 1)
        return float(len(known_cells) / bbox_cells)

    def known_area(self) -> float:
        return float(np.count_nonzero(self.data != UNKNOWN) * self.resolution * self.resolution)

    def visited_area(self) -> float:
        return float(np.count_nonzero(self.data == VISITED) * self.resolution * self.resolution)

    def known_bbox_extent(self) -> tuple[float, float]:
        known_cells = np.argwhere(self.data != UNKNOWN)
        if known_cells.size == 0:
            return (0.0, 0.0)
        min_y, min_x = known_cells.min(axis=0)
        max_y, max_x = known_cells.max(axis=0)
        return (
            float((int(max_x) - int(min_x) + 1) * self.resolution),
            float((int(max_y) - int(min_y) + 1) * self.resolution),
        )


class MultiFloorGridMap:
    """按需创建楼层栅格，并把已配准 world 点云融合到指定楼层。

    本类不知道门 ID、房间或电梯语义。``MissionManager`` 负责在楼层转换未确认
    时阻断更新，并只在服务成功且传感器确认开门后调用动态门清理接口。
    """

    def __init__(
        self,
        *,
        resolution: float = 0.10,
        width: float = 28.0,
        length: float = 48.0,
        origin_x: float = -14.0,
        origin_y: float = -6.0,
        floor_height: float = 2.6,
        obstacle_min_z: float = 0.15,
        obstacle_max_z: float = 1.40,
        obstacle_inflation_radius: float = 0.25,
        raycast_max_range: float = 12.0,
        point_free_clear_max_range: float = 1.5,
        visited_radius: float = 0.45,
        free_clear_enabled: bool = True,
        free_clear_max_range: float = 8.0,
        free_clear_angle_step: float = 0.1745,
        free_clear_obstacle_margin: float = 0.25,
        free_clear_min_points_per_bin: int = 2,
    ):
        self.resolution = float(resolution)
        self.width = float(width)
        self.length = float(length)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.floor_height = float(floor_height)
        self.obstacle_min_z = float(obstacle_min_z)
        self.obstacle_max_z = float(obstacle_max_z)
        self.obstacle_inflation_radius = float(obstacle_inflation_radius)
        self.raycast_max_range = float(raycast_max_range)
        self.point_free_clear_max_range = max(0.0, float(point_free_clear_max_range))
        self.visited_radius = float(visited_radius)
        self.free_clear_enabled = bool(free_clear_enabled)
        self.free_clear_max_range = float(free_clear_max_range)
        self.free_clear_angle_step = float(free_clear_angle_step)
        self.free_clear_obstacle_margin = float(free_clear_obstacle_margin)
        self.free_clear_min_points_per_bin = max(1, int(free_clear_min_points_per_bin))
        self._floors: dict[int, FloorGrid] = {}
        self.last_debug_snapshot: MapperDebugSnapshot | None = None
        self._pending_occupied_to_free_count = 0
        self._pending_dynamic_door_cleared_count = 0

    def floor(self, floor_index: int) -> FloorGrid:
        """取得指定楼层地图；第一次访问时按统一几何参数惰性创建。"""
        if floor_index not in self._floors:
            self._floors[floor_index] = FloorGrid(
                floor_index=floor_index,
                resolution=self.resolution,
                width=self.width,
                length=self.length,
                origin_x=self.origin_x,
                origin_y=self.origin_y,
            )
        return self._floors[floor_index]

    def estimate_floor_from_z(self, z: float) -> int:
        """按楼层高度把绝对 Z 量化为非负楼层编号。"""
        return max(0, int(round(float(z) / self.floor_height)))

    def update_from_cloud(
        self,
        *,
        floor_index: int,
        robot_xyz: tuple[float, float, float],
        points: Iterable[tuple[float, float, float]],
    ) -> None:
        """融合一帧已变换到 world 的点云。

        处理阶段为：标记机器人访问区、收集有效回波和角度桶、批量写障碍、
        对每个回波做保守自由射线、按足够观测的扇区补充自由空间，最后保存
        诊断统计。无障碍远点的自由射线受 ``point_free_clear_max_range`` 限制。
        """
        grid = self.floor(floor_index)
        stats = _MapperFrameStats(
            floor_index=floor_index,
            occupied_to_free_count=self._pending_occupied_to_free_count,
            dynamic_door_cleared_count=self._pending_dynamic_door_cleared_count,
        )
        self._pending_occupied_to_free_count = 0
        self._pending_dynamic_door_cleared_count = 0
        robot_cell = grid.world_to_grid(robot_xyz[0], robot_xyz[1])
        if robot_cell is None:
            self._store_debug_snapshot(grid, stats)
            return
        grid.mark_visited_radius(robot_xyz[0], robot_xyz[1], self.visited_radius)
        floor_base_z = floor_index * self.floor_height
        angle_bins: dict[int, _AngleBinObservation] = {}
        observations: list[
            tuple[float, float, float, float, bool, tuple[int, int]]
        ] = []
        for point in points:
            px, py, pz = point
            distance = math.hypot(px - robot_xyz[0], py - robot_xyz[1])
            if distance > self.raycast_max_range:
                continue
            z_rel = pz - floor_base_z
            is_obstacle_point = self.obstacle_min_z <= z_rel <= self.obstacle_max_z
            if is_obstacle_point and distance < stats.nearest_obstacle_distance:
                stats.nearest_obstacle_distance = distance
            target_cell = grid.world_to_grid(px, py)
            if target_cell is None:
                continue
            self._record_angle_bin(
                angle_bins,
                robot_xyz,
                px,
                py,
                distance,
                is_obstacle_point=is_obstacle_point,
            )
            observations.append((px, py, pz, distance, is_obstacle_point, target_cell))

        # 先写入同帧全部障碍，再清自由空间，避免点顺序导致早到的 miss 穿过晚到的墙命中。
        for px, py, _pz, _distance, is_obstacle_point, _target_cell in observations:
            if is_obstacle_point:
                grid.mark_occupied_radius(px, py, self.obstacle_inflation_radius)

        for px, py, _pz, distance, is_obstacle_point, target_cell in observations:
            ray_target = target_cell
            if not is_obstacle_point and distance > self.point_free_clear_max_range:
                if self.point_free_clear_max_range <= 0.0:
                    stats.suspicious_clearing_prevented_count += 1
                    continue
                scale = self.point_free_clear_max_range / max(distance, 1e-9)
                capped_x = robot_xyz[0] + (px - robot_xyz[0]) * scale
                capped_y = robot_xyz[1] + (py - robot_xyz[1]) * scale
                capped_cell = grid.world_to_grid(capped_x, capped_y)
                if capped_cell is None:
                    stats.suspicious_clearing_prevented_count += 1
                    continue
                ray_target = capped_cell
                stats.suspicious_clearing_prevented_count += 1
            clear_length, stopped_at_blocked = self._raycast_free(
                grid,
                robot_cell,
                ray_target,
                stop_at_blocked=True,
            )
            stats.point_ray_count += 1
            stats.max_clearing_length = max(stats.max_clearing_length, clear_length)
            if stopped_at_blocked:
                stats.blocked_stop_count += 1
                stats.suspicious_clearing_prevented_count += 1
        if self.free_clear_enabled:
            self._clear_visible_sectors(grid, robot_xyz, robot_cell, angle_bins, stats)
        self._store_debug_snapshot(grid, stats)

    def latest_debug_snapshot(self) -> MapperDebugSnapshot | None:
        return self.last_debug_snapshot

    def _store_debug_snapshot(self, grid: FloorGrid, stats: _MapperFrameStats) -> None:
        nearest_obstacle_distance = (
            stats.nearest_obstacle_distance if math.isfinite(stats.nearest_obstacle_distance) else None
        )
        self.last_debug_snapshot = MapperDebugSnapshot(
            floor_index=stats.floor_index,
            point_ray_count=stats.point_ray_count,
            sector_ray_count=stats.sector_ray_count,
            max_clearing_length=stats.max_clearing_length,
            blocked_stop_count=stats.blocked_stop_count,
            occupied_to_free_count=stats.occupied_to_free_count,
            dynamic_door_cleared_count=stats.dynamic_door_cleared_count,
            suspicious_clearing_prevented_count=stats.suspicious_clearing_prevented_count,
            nearest_obstacle_distance=nearest_obstacle_distance,
            known_bbox_fill_ratio=grid.known_bbox_fill_ratio(),
            traversable_known_ratio=grid.traversable_known_ratio(),
            visited_traversable_ratio=grid.visited_traversable_ratio(),
        )

    def mark_black(self, floor_index: int, x: float, y: float, radius: float) -> None:
        """标记任务层禁止规划区域，但不覆盖已经确认的实体 OCC。"""
        grid = self.floor(floor_index)
        center = grid.world_to_grid(x, y)
        if center is None:
            return
        cx, cy = center
        cells = int(math.ceil(radius / grid.resolution))
        for gy in range(cy - cells, cy + cells + 1):
            for gx in range(cx - cells, cx + cells + 1):
                if 0 <= gx < grid.size_x and 0 <= gy < grid.size_y and grid.data[gy, gx] != OCC:
                    grid.data[gy, gx] = BLACK

    def clear_dynamic_door_strip(
        self,
        floor_index: int,
        x: float,
        y: float,
        yaw: float,
        width: float,
        depth: float,
    ) -> None:
        """按门洞方向清理动态障碍残留，避免圆形清理擦掉门侧静态墙。

        仅矩形门板条带中的 OCC/BLACK 会恢复为 FREE；门框两侧和门洞之外不受
        影响。该方法是动态环境修改地图的唯一显式入口。
        """
        grid = self.floor(floor_index)
        center = grid.world_to_grid(x, y)
        if center is None:
            return
        cx, cy = center
        half_width = max(0.0, float(width)) * 0.5
        half_depth = max(0.0, float(depth)) * 0.5
        if half_width <= 0.0 or half_depth <= 0.0:
            return
        normal_x = math.cos(yaw)
        normal_y = math.sin(yaw)
        tangent_x = -normal_y
        tangent_y = normal_x
        search_radius = math.hypot(half_width, half_depth)
        cells = int(math.ceil(search_radius / grid.resolution))
        cleared_count = 0
        for gy in range(cy - cells, cy + cells + 1):
            for gx in range(cx - cells, cx + cells + 1):
                if not (0 <= gx < grid.size_x and 0 <= gy < grid.size_y):
                    continue
                if int(grid.data[gy, gx]) != OCC:
                    continue
                wx, wy = grid.grid_to_world(gx, gy)
                dx = wx - x
                dy = wy - y
                along_normal = dx * normal_x + dy * normal_y
                along_tangent = dx * tangent_x + dy * tangent_y
                if abs(along_normal) <= half_depth and abs(along_tangent) <= half_width:
                    grid.data[gy, gx] = FREE
                    cleared_count += 1
        self._pending_occupied_to_free_count += cleared_count
        self._pending_dynamic_door_cleared_count += cleared_count

    def _raycast_free(
        self,
        grid: FloorGrid,
        start: tuple[int, int],
        end: tuple[int, int],
        *,
        stop_at_blocked: bool = True,
    ) -> tuple[float, bool]:
        last_cleared = start
        for gx, gy in _bresenham(start[0], start[1], end[0], end[1])[:-1]:
            if stop_at_blocked and int(grid.data[gy, gx]) in (OCC, BLACK):
                return _grid_distance(grid, start, last_cleared), True
            grid.mark_free(gx, gy)
            last_cleared = (gx, gy)
        return _grid_distance(grid, start, last_cleared), False

    def _record_angle_bin(
        self,
        angle_bins: dict[int, _AngleBinObservation],
        robot_xyz: tuple[float, float, float],
        px: float,
        py: float,
        distance: float,
        *,
        is_obstacle_point: bool,
    ) -> None:
        if self.free_clear_angle_step <= 0.0:
            return
        angle = math.atan2(py - robot_xyz[1], px - robot_xyz[0])
        bucket = int(math.floor((angle + math.pi) / self.free_clear_angle_step))
        observation = angle_bins.setdefault(bucket, _AngleBinObservation())
        observation.point_count += 1
        if is_obstacle_point and distance < observation.nearest_obstacle_distance:
            observation.nearest_obstacle_distance = distance

    def _clear_visible_sectors(
        self,
        grid: FloorGrid,
        robot_xyz: tuple[float, float, float],
        robot_cell: tuple[int, int],
        angle_bins: dict[int, _AngleBinObservation],
        stats: _MapperFrameStats,
    ) -> None:
        if self.free_clear_angle_step <= 0.0 or self.free_clear_max_range <= 0.0:
            return
        # 无障碍点的方向只清到保守近距离，防止因点云稀疏把墙后误标 free。
        no_obstacle_clear_distance = min(1.5, self.free_clear_max_range)
        for bucket, observation in angle_bins.items():
            if observation.point_count < self.free_clear_min_points_per_bin:
                stats.suspicious_clearing_prevented_count += 1
                continue
            if math.isfinite(observation.nearest_obstacle_distance):
                clear_distance = min(
                    observation.nearest_obstacle_distance - self.free_clear_obstacle_margin,
                    self.free_clear_max_range,
                )
            else:
                clear_distance = no_obstacle_clear_distance
                if no_obstacle_clear_distance < self.free_clear_max_range:
                    stats.suspicious_clearing_prevented_count += 1
            if clear_distance <= 0.0:
                continue
            angle = -math.pi + (bucket + 0.5) * self.free_clear_angle_step
            target_x = robot_xyz[0] + clear_distance * math.cos(angle)
            target_y = robot_xyz[1] + clear_distance * math.sin(angle)
            target_cell = grid.world_to_grid(target_x, target_y)
            if target_cell is None:
                continue
            clear_length, stopped_at_blocked = self._clear_free_until_blocked(grid, robot_cell, target_cell)
            stats.sector_ray_count += 1
            stats.max_clearing_length = max(stats.max_clearing_length, clear_length)
            if stopped_at_blocked:
                stats.blocked_stop_count += 1
                stats.suspicious_clearing_prevented_count += 1

    def _clear_free_until_blocked(
        self,
        grid: FloorGrid,
        start: tuple[int, int],
        end: tuple[int, int],
    ) -> tuple[float, bool]:
        last_cleared = start
        for gx, gy in _bresenham(start[0], start[1], end[0], end[1])[1:]:
            cell_value = int(grid.data[gy, gx])
            if cell_value in (OCC, BLACK):
                return _grid_distance(grid, start, last_cleared), True
            grid.mark_free(gx, gy)
            last_cleared = (gx, gy)
        return _grid_distance(grid, start, last_cleared), False


def _grid_distance(grid: FloorGrid, start: tuple[int, int], end: tuple[int, int]) -> float:
    return math.hypot(end[0] - start[0], end[1] - start[1]) * grid.resolution


def _bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    points: list[tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return points
