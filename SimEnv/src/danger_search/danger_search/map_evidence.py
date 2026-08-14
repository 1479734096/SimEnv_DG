"""正式地图中的正障碍证据账本。

本模块只管理障碍核心、膨胀所有权和有界命中/反证窗口。它不读取 ROS、不修改
任务状态，也不决定控制命令；调用方根据返回的受影响格刷新派生规划视图。
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterable


LOW_CANDIDATE = "LOW_CANDIDATE"
LOW_CONFIRMED = "LOW_CONFIRMED"
HIGH_HARD = "HIGH_HARD"
LEGACY_HARD = "LEGACY_HARD"
BLOCKING_OBSTACLE_KINDS = frozenset(
    {LEGACY_HARD, HIGH_HARD, LOW_CONFIRMED}
)
_POSE_SAMPLE_LIMIT = 8


@dataclass
class ObstacleCoreRecord:
    cell: tuple[int, int]
    inflated_cells: set[tuple[int, int]]
    kind: str
    floor_index: int
    map_epoch: int
    localization_epoch: int
    hit_generations: set[int] = field(default_factory=set)
    hit_world_poses: list[tuple[float, float]] = field(default_factory=list)
    counter_generations: set[int] = field(default_factory=set)
    counter_world_poses: list[tuple[float, float]] = field(default_factory=list)
    first_hit_generation: int | None = None
    last_hit_generation: int | None = None
    hit_generation_count: int = 0
    last_counter_generation: int | None = None
    counter_generation_count: int = 0
    protected_hit_generations: set[int] = field(default_factory=set)
    protected_hit_pose_samples: list[tuple[int, float, float]] = field(
        default_factory=list
    )
    protected_counter_generations: set[int] = field(default_factory=set)
    protected_counter_world_poses: list[tuple[float, float]] = field(default_factory=list)
    last_protected_hit_generation: int | None = None
    last_protected_counter_generation: int | None = None
    protected_trail_confirmed: bool = False


class ObstacleEvidenceLedger:
    """单楼层、有界、来源可追溯的障碍核心账本。"""

    def __init__(
        self,
        *,
        floor_index: int,
        size_x: int,
        size_y: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
    ) -> None:
        self.floor_index = int(floor_index)
        self.size_x = int(size_x)
        self.size_y = int(size_y)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.cores: dict[tuple[int, int], ObstacleCoreRecord] = {}
        self.inflation_owners: dict[
            tuple[int, int], set[tuple[int, int]]
        ] = {}

    def copy(self) -> "ObstacleEvidenceLedger":
        result = ObstacleEvidenceLedger(
            floor_index=self.floor_index,
            size_x=self.size_x,
            size_y=self.size_y,
            resolution=self.resolution,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
        )
        result.cores = {
            cell: ObstacleCoreRecord(
                cell=record.cell,
                inflated_cells=set(record.inflated_cells),
                kind=record.kind,
                floor_index=record.floor_index,
                map_epoch=record.map_epoch,
                localization_epoch=record.localization_epoch,
                hit_generations=set(record.hit_generations),
                hit_world_poses=list(record.hit_world_poses),
                counter_generations=set(record.counter_generations),
                counter_world_poses=list(record.counter_world_poses),
                first_hit_generation=record.first_hit_generation,
                last_hit_generation=record.last_hit_generation,
                hit_generation_count=record.hit_generation_count,
                last_counter_generation=record.last_counter_generation,
                counter_generation_count=record.counter_generation_count,
                protected_hit_generations=set(record.protected_hit_generations),
                protected_hit_pose_samples=list(record.protected_hit_pose_samples),
                protected_counter_generations=set(
                    record.protected_counter_generations
                ),
                protected_counter_world_poses=list(
                    record.protected_counter_world_poses
                ),
                last_protected_hit_generation=(
                    record.last_protected_hit_generation
                ),
                last_protected_counter_generation=(
                    record.last_protected_counter_generation
                ),
                protected_trail_confirmed=record.protected_trail_confirmed,
            )
            for cell, record in self.cores.items()
        }
        result.inflation_owners = {
            cell: set(owners) for cell, owners in self.inflation_owners.items()
        }
        return result

    def blocking_at(self, cell: tuple[int, int]) -> bool:
        return any(
            owner in self.cores
            and self.cores[owner].kind in BLOCKING_OBSTACLE_KINDS
            for owner in self.inflation_owners.get(cell, ())
        )

    def protected_trail_blocking_at(self, cell: tuple[int, int]) -> bool:
        """仅返回已获得近场确认权的阻挡来源。"""
        return any(
            owner in self.cores
            and self.cores[owner].kind in BLOCKING_OBSTACLE_KINDS
            and self.cores[owner].protected_trail_confirmed
            for owner in self.inflation_owners.get(cell, ())
        )

    def record_observation(
        self,
        x: float,
        y: float,
        radius: float,
        *,
        kind: str,
        cloud_generation: int,
        robot_xy: tuple[float, float],
        map_epoch: int,
        localization_epoch: int,
        low_confirm_generations: int,
        low_confirm_window_generations: int,
        pose_separation: float,
    ) -> set[tuple[int, int]]:
        center = self.world_to_grid(x, y)
        if center is None:
            return set()
        if kind not in {LOW_CANDIDATE, HIGH_HARD, LEGACY_HARD}:
            raise ValueError("未知障碍证据类型")
        affected = self._inflated_cells(x, y, radius)
        core = self.cores.get(center)
        scope = (self.floor_index, int(map_epoch), int(localization_epoch))
        if core is not None and (
            core.floor_index,
            core.map_epoch,
            core.localization_epoch,
        ) != scope:
            # 跨 epoch 证据不得合并；旧核心保持阻挡，等待上层地图重建事务。
            return set()
        if core is None:
            core = ObstacleCoreRecord(
                cell=center,
                inflated_cells=set(),
                kind=(HIGH_HARD if kind == HIGH_HARD else kind),
                floor_index=self.floor_index,
                map_epoch=int(map_epoch),
                localization_epoch=int(localization_epoch),
                protected_trail_confirmed=(kind == LEGACY_HARD),
            )
            self.cores[center] = core
        if kind in {HIGH_HARD, LEGACY_HARD}:
            core.kind = kind
            if kind == LEGACY_HARD:
                core.protected_trail_confirmed = True
        elif core.kind not in BLOCKING_OBSTACLE_KINDS:
            core.kind = LOW_CANDIDATE

        generation = int(cloud_generation)
        if generation > 0 and generation != core.last_hit_generation:
            core.hit_generations.add(generation)
            minimum = generation - int(low_confirm_window_generations) + 1
            core.hit_generations = {
                item for item in core.hit_generations if item >= minimum
            }
            core.hit_generation_count = min(
                int(low_confirm_window_generations),
                core.hit_generation_count + 1,
            )
            core.first_hit_generation = (
                generation
                if core.first_hit_generation is None
                else min(core.first_hit_generation, generation)
            )
            core.last_hit_generation = generation
            _append_separated_pose(
                core.hit_world_poses,
                robot_xy,
                pose_separation,
            )
            core.counter_generations.clear()
            core.counter_world_poses.clear()
            core.counter_generation_count = 0
            core.last_counter_generation = None
            core.protected_counter_generations.clear()
            core.protected_counter_world_poses.clear()
            core.last_protected_counter_generation = None
            if (
                core.kind == LOW_CANDIDATE
                and len(core.hit_generations) >= int(low_confirm_generations)
                and len(core.hit_world_poses)
                >= min(2, int(low_confirm_generations))
            ):
                core.kind = LOW_CONFIRMED

        changed = set(core.inflated_cells) | affected
        for cell in affected:
            self.inflation_owners.setdefault(cell, set()).add(center)
        core.inflated_cells.update(affected)
        return changed

    def record_protected_trail_observation(
        self,
        x: float,
        y: float,
        *,
        cloud_generation: int,
        robot_xy: tuple[float, float],
        confirm_generations: int,
        confirm_positions: int,
        generation_window: int,
        pose_separation: float,
    ) -> set[tuple[int, int]]:
        """为已存在的障碍核心累计覆盖历史足迹所需的近场证据。"""
        center = self.world_to_grid(x, y)
        core = self.cores.get(center) if center is not None else None
        generation = int(cloud_generation)
        if core is None or generation <= 0:
            return set()
        if generation == core.last_protected_hit_generation:
            return set()

        minimum = generation - int(generation_window) + 1
        core.protected_hit_generations.add(generation)
        core.protected_hit_generations = {
            item for item in core.protected_hit_generations if item >= minimum
        }
        core.protected_hit_pose_samples = [
            sample
            for sample in core.protected_hit_pose_samples
            if sample[0] >= minimum
        ]
        core.last_protected_hit_generation = generation
        _upsert_generation_pose(
            core.protected_hit_pose_samples,
            generation,
            robot_xy,
            pose_separation,
        )
        core.protected_counter_generations.clear()
        core.protected_counter_world_poses.clear()
        core.last_protected_counter_generation = None
        was_confirmed = core.protected_trail_confirmed
        if not core.protected_trail_confirmed and (
            len(core.protected_hit_generations) >= int(confirm_generations)
            and len(core.protected_hit_pose_samples) >= int(confirm_positions)
        ):
            core.protected_trail_confirmed = True
        return set(core.inflated_cells) if core.protected_trail_confirmed != was_confirmed else set()

    def remove_core(self, cell: tuple[int, int]) -> set[tuple[int, int]]:
        core = self.cores.pop(tuple(cell), None)
        if core is None:
            return set()
        for inflated in core.inflated_cells:
            owners = self.inflation_owners.get(inflated)
            if owners is None:
                continue
            owners.discard(core.cell)
            if not owners:
                self.inflation_owners.pop(inflated, None)
        return set(core.inflated_cells)

    def record_counter_ray(
        self,
        start: tuple[int, int],
        end: tuple[int, int],
        *,
        hit_cells: frozenset[tuple[int, int]],
        cloud_generation: int,
        robot_xy: tuple[float, float],
        map_epoch: int,
        localization_epoch: int,
        candidate_generations: int,
        candidate_positions: int,
        confirmed_generations: int,
        confirmed_positions: int,
        pose_separation: float,
    ) -> set[tuple[int, int]]:
        ray = _bresenham(start[0], start[1], end[0], end[1])
        if len(ray) < 3:
            return set()
        index_by_cell = {cell: index for index, cell in enumerate(ray)}
        removed_cells: set[tuple[int, int]] = set()
        for cell, core in tuple(self.cores.items()):
            low_lifecycle = core.kind in {LOW_CANDIDATE, LOW_CONFIRMED}
            protected_lifecycle = core.protected_trail_confirmed
            if not low_lifecycle and not protected_lifecycle:
                continue
            if cell in hit_cells or cell not in index_by_cell:
                continue
            # 终点必须严格位于核心之后，不能把止于核心的射线当作反证。
            if index_by_cell[cell] >= len(ray) - 1:
                continue
            if (
                core.map_epoch != int(map_epoch)
                or core.localization_epoch != int(localization_epoch)
                or core.floor_index != self.floor_index
            ):
                continue
            generation = int(cloud_generation)
            if generation <= 0:
                continue
            if (
                protected_lifecycle
                and generation != core.last_protected_counter_generation
            ):
                core.protected_counter_generations.add(generation)
                if len(core.protected_counter_generations) > int(confirmed_generations):
                    core.protected_counter_generations = set(
                        sorted(core.protected_counter_generations)[
                            -int(confirmed_generations):
                        ]
                    )
                core.last_protected_counter_generation = generation
                _append_separated_pose(
                    core.protected_counter_world_poses,
                    robot_xy,
                    pose_separation,
                )
                if (
                    len(core.protected_counter_generations)
                    >= int(confirmed_generations)
                    and len(core.protected_counter_world_poses)
                    >= int(confirmed_positions)
                ):
                    core.protected_trail_confirmed = False
                    core.protected_hit_generations.clear()
                    core.protected_hit_pose_samples.clear()
                    core.last_protected_hit_generation = None
                    removed_cells.update(core.inflated_cells)

            if low_lifecycle and generation != core.last_counter_generation:
                core.counter_generations.add(generation)
                limit = max(int(candidate_generations), int(confirmed_generations))
                if len(core.counter_generations) > limit:
                    core.counter_generations = set(
                        sorted(core.counter_generations)[-limit:]
                    )
                core.counter_generation_count = min(
                    limit, core.counter_generation_count + 1
                )
                core.last_counter_generation = generation
                _append_separated_pose(
                    core.counter_world_poses,
                    robot_xy,
                    pose_separation,
                )
                if core.kind == LOW_CANDIDATE:
                    removable = (
                        len(core.counter_generations) >= int(candidate_generations)
                        and len(core.counter_world_poses) >= int(candidate_positions)
                    )
                else:
                    removable = (
                        len(core.counter_generations) >= int(confirmed_generations)
                        and len(core.counter_world_poses) >= int(confirmed_positions)
                    )
                if removable:
                    removed_cells.update(self.remove_core(cell))
        return removed_cells

    def world_to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        gx = int(math.floor((float(x) - self.origin_x) / self.resolution))
        gy = int(math.floor((float(y) - self.origin_y) / self.resolution))
        if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
            return gx, gy
        return None

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        return (
            self.origin_x + (int(gx) + 0.5) * self.resolution,
            self.origin_y + (int(gy) + 0.5) * self.resolution,
        )

    def _inflated_cells(
        self, x: float, y: float, radius: float
    ) -> set[tuple[int, int]]:
        center = self.world_to_grid(x, y)
        if center is None:
            return set()
        if radius <= 0.0:
            return {center}
        cells = int(math.ceil(float(radius) / self.resolution))
        result = set()
        for gy in range(center[1] - cells, center[1] + cells + 1):
            for gx in range(center[0] - cells, center[0] + cells + 1):
                if not (0 <= gx < self.size_x and 0 <= gy < self.size_y):
                    continue
                wx, wy = self.grid_to_world(gx, gy)
                if math.hypot(wx - float(x), wy - float(y)) <= float(radius):
                    result.add((gx, gy))
        return result


def _append_separated_pose(
    poses: list[tuple[float, float]],
    pose: tuple[float, float],
    minimum_separation: float,
) -> None:
    candidate = (float(pose[0]), float(pose[1]))
    if any(
        math.hypot(candidate[0] - old[0], candidate[1] - old[1])
        < float(minimum_separation)
        for old in poses
    ):
        return
    poses.append(candidate)
    if len(poses) > _POSE_SAMPLE_LIMIT:
        del poses[:-_POSE_SAMPLE_LIMIT]


def _upsert_generation_pose(
    samples: list[tuple[int, float, float]],
    generation: int,
    pose: tuple[float, float],
    separation: float,
) -> None:
    """在代次窗口内刷新同一位姿，或登记一个空间独立的新位姿。"""
    px, py = float(pose[0]), float(pose[1])
    for index, (_old_generation, old_x, old_y) in enumerate(samples):
        if math.hypot(px - old_x, py - old_y) < float(separation):
            samples[index] = (int(generation), px, py)
            return
    samples.append((int(generation), px, py))
    if len(samples) > _POSE_SAMPLE_LIMIT:
        del samples[:-_POSE_SAMPLE_LIMIT]


def _bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    points: list[tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    x, y = x0, y0
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            return points
        twice = 2 * error
        if twice >= dy:
            error += dy
            x += sx
        if twice <= dx:
            error += dx
            y += sy
