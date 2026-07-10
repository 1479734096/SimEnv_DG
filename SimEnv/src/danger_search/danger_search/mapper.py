"""多楼层 2.5D 栅格建图。"""
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


@dataclass
class FloorGrid:
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
        if 0 <= gx < self.size_x and 0 <= gy < self.size_y and self.data[gy, gx] == UNKNOWN:
            self.data[gy, gx] = FREE

    def mark_occupied(self, gx: int, gy: int) -> None:
        if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
            self.data[gy, gx] = OCC

    def mark_occupied_radius(self, x: float, y: float, radius: float) -> None:
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
        known = np.count_nonzero(self.data != UNKNOWN)
        if known == 0:
            return 0.0
        visited_or_free = np.count_nonzero((self.data == VISITED) | (self.data == FREE))
        return float(visited_or_free / known)

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
    """为每个楼层维护一张占据栅格。"""

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
        self.visited_radius = float(visited_radius)
        self.free_clear_enabled = bool(free_clear_enabled)
        self.free_clear_max_range = float(free_clear_max_range)
        self.free_clear_angle_step = float(free_clear_angle_step)
        self.free_clear_obstacle_margin = float(free_clear_obstacle_margin)
        self.free_clear_min_points_per_bin = max(1, int(free_clear_min_points_per_bin))
        self._floors: dict[int, FloorGrid] = {}

    def floor(self, floor_index: int) -> FloorGrid:
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
        return max(0, int(round(float(z) / self.floor_height)))

    def update_from_cloud(
        self,
        *,
        floor_index: int,
        robot_xyz: tuple[float, float, float],
        points: Iterable[tuple[float, float, float]],
    ) -> None:
        grid = self.floor(floor_index)
        robot_cell = grid.world_to_grid(robot_xyz[0], robot_xyz[1])
        if robot_cell is None:
            return
        grid.mark_visited_radius(robot_xyz[0], robot_xyz[1], self.visited_radius)
        floor_base_z = floor_index * self.floor_height
        angle_bins: dict[int, _AngleBinObservation] = {}
        for point in points:
            px, py, pz = point
            distance = math.hypot(px - robot_xyz[0], py - robot_xyz[1])
            if distance > self.raycast_max_range:
                continue
            z_rel = pz - floor_base_z
            is_obstacle_point = self.obstacle_min_z <= z_rel <= self.obstacle_max_z
            target_cell = grid.world_to_grid(px, py)
            if target_cell is None:
                continue
            self._raycast_free(grid, robot_cell, target_cell, stop_at_blocked=True)
            self._record_angle_bin(
                angle_bins,
                robot_xyz,
                px,
                py,
                distance,
                is_obstacle_point=is_obstacle_point,
            )
            if is_obstacle_point:
                grid.mark_occupied_radius(px, py, self.obstacle_inflation_radius)
        if self.free_clear_enabled:
            self._clear_visible_sectors(grid, robot_xyz, robot_cell, angle_bins)

    def mark_black(self, floor_index: int, x: float, y: float, radius: float) -> None:
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
        """按门洞方向清理动态障碍残留，避免圆形清理擦掉门侧静态墙。"""
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

    def _raycast_free(
        self,
        grid: FloorGrid,
        start: tuple[int, int],
        end: tuple[int, int],
        *,
        stop_at_blocked: bool = True,
    ) -> None:
        for gx, gy in _bresenham(start[0], start[1], end[0], end[1])[:-1]:
            if stop_at_blocked and int(grid.data[gy, gx]) in (OCC, BLACK):
                break
            grid.mark_free(gx, gy)

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
    ) -> None:
        if self.free_clear_angle_step <= 0.0 or self.free_clear_max_range <= 0.0:
            return
        for bucket, observation in angle_bins.items():
            if observation.point_count < self.free_clear_min_points_per_bin:
                continue
            if math.isfinite(observation.nearest_obstacle_distance):
                clear_distance = min(
                    observation.nearest_obstacle_distance - self.free_clear_obstacle_margin,
                    self.free_clear_max_range,
                )
            else:
                clear_distance = self.free_clear_max_range
            if clear_distance <= 0.0:
                continue
            angle = -math.pi + (bucket + 0.5) * self.free_clear_angle_step
            target_x = robot_xyz[0] + clear_distance * math.cos(angle)
            target_y = robot_xyz[1] + clear_distance * math.sin(angle)
            target_cell = grid.world_to_grid(target_x, target_y)
            if target_cell is None:
                continue
            self._clear_free_until_blocked(grid, robot_cell, target_cell)

    def _clear_free_until_blocked(
        self,
        grid: FloorGrid,
        start: tuple[int, int],
        end: tuple[int, int],
    ) -> None:
        for gx, gy in _bresenham(start[0], start[1], end[0], end[1])[1:]:
            cell_value = int(grid.data[gy, gx])
            if cell_value in (OCC, BLACK):
                break
            grid.mark_free(gx, gy)


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
