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
        raycast_max_range: float = 12.0,
        visited_radius: float = 0.45,
    ):
        self.resolution = float(resolution)
        self.width = float(width)
        self.length = float(length)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.floor_height = float(floor_height)
        self.obstacle_min_z = float(obstacle_min_z)
        self.obstacle_max_z = float(obstacle_max_z)
        self.raycast_max_range = float(raycast_max_range)
        self.visited_radius = float(visited_radius)
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
        for point in points:
            px, py, pz = point
            if math.hypot(px - robot_xyz[0], py - robot_xyz[1]) > self.raycast_max_range:
                continue
            z_rel = pz - floor_base_z
            target_cell = grid.world_to_grid(px, py)
            if target_cell is None:
                continue
            self._raycast_free(grid, robot_cell, target_cell)
            if self.obstacle_min_z <= z_rel <= self.obstacle_max_z:
                grid.mark_occupied(*target_cell)

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

    def _raycast_free(self, grid: FloorGrid, start: tuple[int, int], end: tuple[int, int]) -> None:
        for gx, gy in _bresenham(start[0], start[1], end[0], end[1])[:-1]:
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
