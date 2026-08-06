"""由配准点云增量维护多楼层 2.5D 占据栅格。

每个楼层使用相同 world XY 原点和分辨率，但持有独立二维数组。点云 Z 只用于
相对当前楼板筛选障碍高度，因此地图不表达完整三维几何。栅格状态含义为：
``UNKNOWN`` 未观测、``FREE`` 可通行、``VISITED`` 机器人已覆盖、``BLACK``
任务层临时/永久禁区、``NEGATIVE`` 动态负障、``OCC`` 实体障碍。

更新一帧时先写全部障碍命中，再执行点射线与可视扇区清空。这一顺序以及
“遇到 OCC/BLACK 即停止”的规则可防止稀疏回波、点顺序或墙后点把静态墙擦除。
动态门打开后的局部清理必须显式调用 ``clear_dynamic_door_strip``。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np

from danger_search.map_evidence import (
    BLOCKING_OBSTACLE_KINDS,
    HIGH_HARD,
    LEGACY_HARD,
    LOW_CANDIDATE,
    LOW_CONFIRMED,
    ObstacleEvidenceLedger,
)


UNKNOWN = -1
FREE = 0
VISITED = 30
BLACK = 80
NEGATIVE = 90
OCC = 100


@dataclass(frozen=True)
class NegativeClusterRecord:
    cluster_id: str
    floor: int
    frame_id: str
    map_epoch: int
    localization_epoch: int
    stamp: float
    generation: int
    grid_origin: tuple[float, float]
    grid_resolution: float
    source_cells: frozenset[tuple[int, int]]


@dataclass(frozen=True)
class NegativeClusterCommit:
    committed: bool
    reason_code: str
    cluster_ids: tuple[str, ...] = ()
    revision: int = 0


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
    low_candidate_core_count: int = 0
    low_confirmed_core_count: int = 0
    hard_core_count: int = 0
    low_core_removed_count: int = 0
    low_core_removed_total: int = 0


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
    low_core_removed_count: int = 0


def _valid_negative_cell(cell, grid: "FloorGrid") -> bool:
    if not _valid_negative_cell_structure(cell):
        return False
    gx, gy = cell
    return bool(0 <= gx < grid.size_x and 0 <= gy < grid.size_y)


def _valid_negative_cell_structure(cell) -> bool:
    return bool(
        isinstance(cell, (tuple, list))
        and len(cell) == 2
        and all(
            not isinstance(value, bool) and isinstance(value, int)
            for value in cell
        )
    )


def _negative_record_structure_valid(record) -> bool:
    try:
        return bool(
            isinstance(record, NegativeClusterRecord)
            and isinstance(record.cluster_id, str)
            and bool(record.cluster_id)
            and record.frame_id == "odom"
            and not isinstance(record.floor, bool)
            and isinstance(record.floor, int)
            and record.floor >= 0
            and not isinstance(record.map_epoch, bool)
            and isinstance(record.map_epoch, int)
            and record.map_epoch > 0
            and not isinstance(record.localization_epoch, bool)
            and isinstance(record.localization_epoch, int)
            and record.localization_epoch > 0
            and not isinstance(record.generation, bool)
            and isinstance(record.generation, int)
            and record.generation > 0
            and math.isfinite(float(record.stamp))
            and bool(record.source_cells)
            and all(
                _valid_negative_cell_structure(cell)
                for cell in record.source_cells
            )
            and math.isfinite(float(record.grid_resolution))
            and record.grid_resolution > 0.0
            and len(record.grid_origin) == 2
            and all(
                math.isfinite(float(value)) for value in record.grid_origin
            )
        )
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False


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
        self.resolution = float(self.resolution)
        self.width = float(self.width)
        self.length = float(self.length)
        self.origin_x = float(self.origin_x)
        self.origin_y = float(self.origin_y)
        if (
            not all(
                math.isfinite(value)
                for value in (
                    self.resolution,
                    self.width,
                    self.length,
                    self.origin_x,
                    self.origin_y,
                )
            )
            or self.resolution <= 0.0
            or self.width <= 0.0
            or self.length <= 0.0
        ):
            raise ValueError("栅格分辨率和尺寸必须为正有限数，原点必须为有限数")
        self.size_x = int(math.ceil(self.width / self.resolution))
        self.size_y = int(math.ceil(self.length / self.resolution))
        self.base = np.full((self.size_y, self.size_x), UNKNOWN, dtype=np.int16)
        self._derived = np.full((self.size_y, self.size_x), UNKNOWN, dtype=np.int16)
        self.obstacle_evidence = ObstacleEvidenceLedger(
            floor_index=self.floor_index,
            size_x=self.size_x,
            size_y=self.size_y,
            resolution=self.resolution,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
        )
        self.evidence_scope = (self.floor_index, 1, 1)
        self.black_owners: dict[tuple[int, int], set[str]] = {}
        # NEGATIVE 可能暂时被 OCC/BLACK 覆盖；单独保存来源，动态门清除 OCC 时
        # 才能恢复负障，而直接楼面证据也只能移除这一来源。
        self.negative_cells: set[tuple[int, int]] = set()
        self.negative_cluster_records: dict[str, NegativeClusterRecord] = {}
        self.negative_cell_owners: dict[tuple[int, int], set[str]] = {}
        self.negative_first_state: dict[tuple[int, int], int] = {}
        self.negative_cluster_confirmations: dict[
            str, frozenset[tuple[int, int]]
        ] = {}

    @property
    def data(self) -> np.ndarray:
        view = self._derived.view()
        view.flags.writeable = False
        return view

    @property
    def occ_cores(self):
        return self.obstacle_evidence.cores

    @property
    def inflation_owners(self):
        return self.obstacle_evidence.inflation_owners

    def _in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= int(gx) < self.size_x and 0 <= int(gy) < self.size_y

    def _refresh_cell(self, gx: int, gy: int) -> None:
        cell = (int(gx), int(gy))
        if not self._in_bounds(*cell):
            return
        if self.obstacle_evidence.blocking_at(cell):
            state = OCC
        elif self.black_owners.get(cell):
            state = BLACK
        elif self.negative_cell_owners.get(cell):
            state = NEGATIVE
        else:
            state = int(self.base[cell[1], cell[0]])
        self._derived[cell[1], cell[0]] = state

    def _refresh_cells(self, cells: Iterable[tuple[int, int]]) -> None:
        for gx, gy in cells:
            self._refresh_cell(gx, gy)

    def set_base_state(self, gx: int, gy: int, state: int) -> None:
        if int(state) not in (UNKNOWN, FREE, VISITED):
            raise ValueError("基础层只允许 UNKNOWN、FREE 或 VISITED")
        if self._in_bounds(gx, gy):
            self.base[int(gy), int(gx)] = int(state)
            self._refresh_cell(gx, gy)

    def fill_base(self, state: int) -> None:
        if int(state) not in (UNKNOWN, FREE, VISITED):
            raise ValueError("基础层只允许 UNKNOWN、FREE 或 VISITED")
        self.base[:, :] = int(state)
        for gy in range(self.size_y):
            for gx in range(self.size_x):
                self._refresh_cell(gx, gy)

    def load_legacy_data(self, values) -> None:
        data = np.asarray(values, dtype=np.int16)
        if data.shape != (self.size_y, self.size_x):
            raise ValueError("旧栅格快照尺寸不匹配")
        self.base[:, :] = UNKNOWN
        self.obstacle_evidence = ObstacleEvidenceLedger(
            floor_index=self.floor_index,
            size_x=self.size_x,
            size_y=self.size_y,
            resolution=self.resolution,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
        )
        self.black_owners.clear()
        self.negative_cells.clear()
        self.negative_cell_owners.clear()
        for gy in range(self.size_y):
            for gx in range(self.size_x):
                state = int(data[gy, gx])
                if state in (UNKNOWN, FREE, VISITED):
                    self.base[gy, gx] = state
                elif state == OCC:
                    wx, wy = self.grid_to_world(gx, gy)
                    self.record_obstacle_observation(
                        wx,
                        wy,
                        0.0,
                        kind=LEGACY_HARD,
                        cloud_generation=0,
                        robot_xy=(wx, wy),
                        map_epoch=1,
                        localization_epoch=1,
                        low_confirm_generations=1,
                        low_confirm_window_generations=1,
                        pose_separation=0.0,
                    )
                elif state == BLACK:
                    self.black_owners[(gx, gy)] = {"legacy_snapshot"}
                elif state == NEGATIVE:
                    self.negative_cells.add((gx, gy))
                    self.negative_cell_owners[(gx, gy)] = {"legacy_snapshot"}
        self._derived[:, :] = UNKNOWN
        self._refresh_cells(
            (gx, gy)
            for gy in range(self.size_y)
            for gx in range(self.size_x)
        )

    def copy(self) -> "FloorGrid":
        result = FloorGrid(
            self.floor_index,
            self.resolution,
            self.width,
            self.length,
            self.origin_x,
            self.origin_y,
        )
        result.base = np.array(self.base, copy=True)
        result._derived = np.array(self._derived, copy=True)
        result.obstacle_evidence = self.obstacle_evidence.copy()
        result.evidence_scope = tuple(self.evidence_scope)
        result.black_owners = {
            cell: set(owners) for cell, owners in self.black_owners.items()
        }
        result.negative_cells = set(self.negative_cells)
        result.negative_cluster_records = dict(self.negative_cluster_records)
        result.negative_cell_owners = {
            cell: set(owners) for cell, owners in self.negative_cell_owners.items()
        }
        result.negative_first_state = dict(self.negative_first_state)
        result.negative_cluster_confirmations = dict(
            self.negative_cluster_confirmations
        )
        return result

    def set_snapshot_cell(
        self,
        gx: int,
        gy: int,
        state: int,
        *,
        owner: str = "snapshot",
    ) -> None:
        if not self._in_bounds(gx, gy):
            return
        cell = (int(gx), int(gy))
        if state == BLACK:
            self.black_owners.setdefault(cell, set()).add(str(owner))
        elif state in (UNKNOWN, FREE, VISITED):
            self.base[gy, gx] = int(state)
            self.black_owners.pop(cell, None)
            if state in (FREE, VISITED):
                for obstacle_owner in tuple(
                    self.inflation_owners.get(cell, ())
                ):
                    self.remove_obstacle_core(obstacle_owner)
        else:
            raise ValueError("规划快照只允许基础状态或 BLACK")
        self._refresh_cell(*cell)

    def activate_evidence_scope(
        self, *, map_epoch: int, localization_epoch: int
    ) -> None:
        requested = (
            self.floor_index,
            int(map_epoch),
            int(localization_epoch),
        )
        if requested == self.evidence_scope:
            return
        affected = set(self.inflation_owners) | set(self.negative_cell_owners)
        self.obstacle_evidence = ObstacleEvidenceLedger(
            floor_index=self.floor_index,
            size_x=self.size_x,
            size_y=self.size_y,
            resolution=self.resolution,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
        )
        self.negative_cells.clear()
        self.negative_cluster_records.clear()
        self.negative_cell_owners.clear()
        self.negative_first_state.clear()
        self.negative_cluster_confirmations.clear()
        self.evidence_scope = requested
        self._refresh_cells(affected)

    def world_to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        if not math.isfinite(float(x)) or not math.isfinite(float(y)):
            return None
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
        if self._in_bounds(gx, gy) and int(self.base[gy, gx]) == UNKNOWN:
            self.set_base_state(gx, gy, FREE)

    def mark_occupied(self, gx: int, gy: int) -> None:
        if not self._in_bounds(gx, gy):
            return
        wx, wy = self.grid_to_world(gx, gy)
        self.record_obstacle_observation(
            wx,
            wy,
            0.0,
            kind=LEGACY_HARD,
            cloud_generation=0,
            robot_xy=(wx, wy),
            map_epoch=1,
            localization_epoch=1,
            low_confirm_generations=1,
            low_confirm_window_generations=1,
            pose_separation=0.0,
        )

    def mark_occupied_radius(self, x: float, y: float, radius: float) -> None:
        """把障碍命中按机器人安全余量膨胀为圆形 OCC 区域。"""
        self.record_obstacle_observation(
            x,
            y,
            radius,
            kind=LEGACY_HARD,
            cloud_generation=0,
            robot_xy=(x, y),
            map_epoch=1,
            localization_epoch=1,
            low_confirm_generations=1,
            low_confirm_window_generations=1,
            pose_separation=0.0,
        )

    def record_obstacle_observation(self, x, y, radius, **kwargs) -> None:
        changed = self.obstacle_evidence.record_observation(
            x, y, radius, **kwargs
        )
        self._refresh_cells(changed)

    def remove_obstacle_core(self, cell: tuple[int, int]) -> bool:
        existed = tuple(cell) in self.occ_cores
        changed = self.obstacle_evidence.remove_core(tuple(cell))
        self._refresh_cells(changed)
        return existed

    def record_counter_ray(self, start, end, **kwargs) -> int:
        before = len(self.occ_cores)
        changed = self.obstacle_evidence.record_counter_ray(
            start, end, **kwargs
        )
        self._refresh_cells(changed)
        return max(0, before - len(self.occ_cores))

    def mark_black(self, gx: int, gy: int, *, owner: str) -> None:
        if self._in_bounds(gx, gy):
            self.black_owners.setdefault((gx, gy), set()).add(str(owner))
            self._refresh_cell(gx, gy)

    def clear_black(
        self, gx: int, gy: int, *, owner: str | None = None
    ) -> None:
        cell = (int(gx), int(gy))
        owners = self.black_owners.get(cell)
        if owners is None:
            return
        if owner is None:
            self.black_owners.pop(cell, None)
        else:
            owners.discard(str(owner))
            if not owners:
                self.black_owners.pop(cell, None)
        self._refresh_cell(*cell)

    def mark_visited_radius(self, x: float, y: float, radius: float) -> None:
        """记录机身实际覆盖区域；实体 OCC 与安全 BLACK 始终拥有更高优先级。"""
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
                if math.hypot(wx - x, wy - y) <= radius:
                    self.set_base_state(gx, gy, VISITED)

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
        negative_minimum_drop: float = 0.30,
        evidence_lifecycle_enabled: bool = True,
        low_height_max_z: float = 0.30,
        low_confirm_generations: int = 3,
        low_confirm_window_generations: int = 20,
        low_candidate_counter_generations: int = 3,
        low_candidate_counter_positions: int = 2,
        low_confirmed_counter_generations: int = 6,
        low_confirmed_counter_positions: int = 3,
        counter_pose_separation: float = 0.15,
        low_near_exempt_radius: float = 0.0,
        low_candidate_max_range: float = 0.0,
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
        self.point_free_clear_max_range = float(point_free_clear_max_range)
        self.visited_radius = float(visited_radius)
        self.free_clear_enabled = bool(free_clear_enabled)
        self.free_clear_max_range = float(free_clear_max_range)
        self.free_clear_angle_step = float(free_clear_angle_step)
        self.free_clear_obstacle_margin = float(free_clear_obstacle_margin)
        self.free_clear_min_points_per_bin = max(1, int(free_clear_min_points_per_bin))
        self.negative_minimum_drop = float(negative_minimum_drop)
        self.evidence_lifecycle_enabled = bool(evidence_lifecycle_enabled)
        self.low_height_max_z = float(low_height_max_z)
        self.low_confirm_generations = int(low_confirm_generations)
        self.low_confirm_window_generations = int(low_confirm_window_generations)
        self.low_candidate_counter_generations = int(
            low_candidate_counter_generations
        )
        self.low_candidate_counter_positions = int(
            low_candidate_counter_positions
        )
        self.low_confirmed_counter_generations = int(
            low_confirmed_counter_generations
        )
        self.low_confirmed_counter_positions = int(
            low_confirmed_counter_positions
        )
        self.counter_pose_separation = float(counter_pose_separation)
        self.low_near_exempt_radius = max(0.0, float(low_near_exempt_radius))
        self.low_candidate_max_range = max(0.0, float(low_candidate_max_range))
        self._negative_cluster_revision = 0
        self._low_core_removed_total = 0
        finite_parameters = (
            self.resolution,
            self.width,
            self.length,
            self.origin_x,
            self.origin_y,
            self.floor_height,
            self.obstacle_min_z,
            self.obstacle_max_z,
            self.obstacle_inflation_radius,
            self.raycast_max_range,
            self.point_free_clear_max_range,
            self.visited_radius,
            self.free_clear_max_range,
            self.free_clear_angle_step,
            self.free_clear_obstacle_margin,
            self.negative_minimum_drop,
            self.low_height_max_z,
            self.counter_pose_separation,
            self.low_near_exempt_radius,
            self.low_candidate_max_range,
        )
        if not all(math.isfinite(value) for value in finite_parameters):
            raise ValueError("建图参数必须为有限数")
        if (
            self.resolution <= 0.0
            or self.width <= 0.0
            or self.length <= 0.0
            or self.floor_height <= 0.0
            or self.obstacle_max_z < self.obstacle_min_z
            or self.obstacle_inflation_radius < 0.0
            or self.raycast_max_range < 0.0
            or self.visited_radius < 0.0
            or self.free_clear_max_range < 0.0
            or self.free_clear_obstacle_margin < 0.0
            or self.negative_minimum_drop <= 0.0
            or self.low_height_max_z < self.obstacle_min_z
            or self.low_height_max_z > self.obstacle_max_z
            or self.low_confirm_generations <= 0
            or self.low_confirm_window_generations < self.low_confirm_generations
            or self.low_candidate_counter_generations <= 0
            or self.low_candidate_counter_positions <= 0
            or self.low_confirmed_counter_generations
            < self.low_candidate_counter_generations
            or self.low_confirmed_counter_positions
            < self.low_candidate_counter_positions
            or self.counter_pose_separation < 0.0
        ):
            raise ValueError("建图尺寸、楼高、障碍范围或距离参数无效")
        self.point_free_clear_max_range = max(0.0, self.point_free_clear_max_range)
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
        height = float(z)
        if not math.isfinite(height):
            raise ValueError("楼层估计高度必须为有限数")
        return max(0, int(round(height / self.floor_height)))

    def update_from_cloud(
        self,
        *,
        floor_index: int,
        robot_xyz: tuple[float, float, float],
        points: Iterable[tuple[float, float, float]],
        cloud_generation: int = 0,
        map_epoch: int = 1,
        localization_epoch: int = 1,
    ) -> None:
        """融合一帧已变换到 world 的点云。

        处理阶段为：标记机器人访问区、收集有效回波和角度桶、批量写障碍、
        对每个回波做保守自由射线、按足够观测的扇区补充自由空间，最后保存
        诊断统计。无障碍远点的自由射线受 ``point_free_clear_max_range`` 限制。
        """
        grid = self.floor(floor_index)
        grid.activate_evidence_scope(
            map_epoch=int(map_epoch),
            localization_epoch=int(localization_epoch),
        )
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
            try:
                px, py, pz = (float(value) for value in point)
            except (TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in (px, py, pz)):
                continue
            distance = math.hypot(px - robot_xyz[0], py - robot_xyz[1])
            if distance > self.raycast_max_range:
                continue
            z_rel = pz - floor_base_z
            is_obstacle_point = self.obstacle_min_z <= z_rel <= self.obstacle_max_z
            is_lower_return = z_rel < -self.negative_minimum_drop
            if is_obstacle_point and distance < stats.nearest_obstacle_distance:
                stats.nearest_obstacle_distance = distance
            target_cell = grid.world_to_grid(px, py)
            if target_cell is None:
                continue
            # 明确落到本层楼板以下的回波由支撑层投影为负障。它既不能产生
            # 普通自由射线，也不能让该方向被视为“无障碍扇区”。
            if is_lower_return:
                stats.suspicious_clearing_prevented_count += 1
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

        obstacle_hit_cells = frozenset(
            target_cell
            for _px, _py, _pz, _distance, is_obstacle_point, target_cell
            in observations
            if is_obstacle_point
        )
        # 先写入同帧全部障碍，再清自由空间，避免点顺序导致早到的 miss 穿过晚到的墙命中。
        for px, py, _pz, _distance, is_obstacle_point, _target_cell in observations:
            if is_obstacle_point:
                if not self.evidence_lifecycle_enabled or int(cloud_generation) <= 0:
                    grid.mark_occupied_radius(
                        px, py, self.obstacle_inflation_radius
                    )
                else:
                    nominal_z_rel = float(_pz) - floor_base_z
                    if nominal_z_rel < self.low_height_max_z:
                        # 低回波候选记账的距离语境：
                        # - 近距（< low_near_exempt_radius）：来源是自腿/紧贴物体等物理
                        #   稳定的伪影（bag 取证：机身前上方 Mid360 每帧上千自腿低回波点
                        #   命中站位格，3 代次+2 位姿秒确认成 LOW_CONFIRMED，直接导致
                        #   start_cell_obstacle_core）；由 0.5m stop envelope 执行层物理兜底，
                        #   不记账不会造成穿越。视线遮挡语义保留（该点仍在
                        #   obstacle_hit_cells 中截断自由射线）。
                        # - 远距（> low_candidate_max_range）：点稀疏、高度混叠，多帧
                        #   "稳定命中"不可靠，只信高回波（HIGH_HARD 不受限）。
                        if (
                            self.low_near_exempt_radius > 0.0
                            and _distance < self.low_near_exempt_radius
                        ):
                            continue
                        if (
                            self.low_candidate_max_range > 0.0
                            and _distance > self.low_candidate_max_range
                        ):
                            continue
                        kind = LOW_CANDIDATE
                    else:
                        kind = HIGH_HARD
                    grid.record_obstacle_observation(
                        px,
                        py,
                        self.obstacle_inflation_radius,
                        kind=kind,
                        cloud_generation=int(cloud_generation),
                        robot_xy=(robot_xyz[0], robot_xyz[1]),
                        map_epoch=int(map_epoch),
                        localization_epoch=int(localization_epoch),
                        low_confirm_generations=self.low_confirm_generations,
                        low_confirm_window_generations=(
                            self.low_confirm_window_generations
                        ),
                        pose_separation=self.counter_pose_separation,
                    )

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
            if self.evidence_lifecycle_enabled and not is_obstacle_point:
                removed = grid.record_counter_ray(
                    robot_cell,
                    ray_target,
                    hit_cells=obstacle_hit_cells,
                    cloud_generation=int(cloud_generation),
                    robot_xy=(robot_xyz[0], robot_xyz[1]),
                    map_epoch=int(map_epoch),
                    localization_epoch=int(localization_epoch),
                    candidate_generations=(
                        self.low_candidate_counter_generations
                    ),
                    candidate_positions=self.low_candidate_counter_positions,
                    confirmed_generations=(
                        self.low_confirmed_counter_generations
                    ),
                    confirmed_positions=self.low_confirmed_counter_positions,
                    pose_separation=self.counter_pose_separation,
                )
                stats.low_core_removed_count += removed
                self._low_core_removed_total += removed
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
            low_candidate_core_count=sum(
                core.kind == LOW_CANDIDATE for core in grid.occ_cores.values()
            ),
            low_confirmed_core_count=sum(
                core.kind == LOW_CONFIRMED for core in grid.occ_cores.values()
            ),
            hard_core_count=sum(
                core.kind in {HIGH_HARD, LEGACY_HARD}
                for core in grid.occ_cores.values()
            ),
            low_core_removed_count=int(stats.low_core_removed_count),
            low_core_removed_total=int(self._low_core_removed_total),
        )

    def persist_negative_clusters_atomic(
        self,
        records: Iterable[NegativeClusterRecord],
        *,
        allow_new: bool = True,
    ) -> NegativeClusterCommit:
        """整批校验后提交 DROP owner；任一记录异常时零修改。"""
        frozen = tuple(records)
        if not frozen:
            return NegativeClusterCommit(False, "drop_source_cluster_missing")
        if any(not _negative_record_structure_valid(record) for record in frozen):
            return NegativeClusterCommit(False, "drop_cluster_scope_invalid")
        batch_scope = (
            frozen[0].floor,
            frozen[0].frame_id,
            frozen[0].map_epoch,
            frozen[0].localization_epoch,
            frozen[0].grid_origin,
            frozen[0].grid_resolution,
        )
        ids = tuple(record.cluster_id for record in frozen)
        if len(set(ids)) != len(ids):
            return NegativeClusterCommit(False, "drop_cluster_id_conflict")
        for record in frozen:
            if (
                (
                    record.floor,
                    record.frame_id,
                    record.map_epoch,
                    record.localization_epoch,
                    record.grid_origin,
                    record.grid_resolution,
                ) != batch_scope
            ):
                return NegativeClusterCommit(False, "drop_cluster_scope_invalid")
        floor = frozen[0].floor
        expected_size_x = int(math.ceil(self.width / self.resolution))
        expected_size_y = int(math.ceil(self.length / self.resolution))
        if (
            abs(frozen[0].grid_resolution - self.resolution) > 1e-9
            or frozen[0].grid_origin != (self.origin_x, self.origin_y)
            or any(
                not (
                    0 <= cell[0] < expected_size_x
                    and 0 <= cell[1] < expected_size_y
                )
                for record in frozen
                for cell in record.source_cells
            )
        ):
            return NegativeClusterCommit(False, "drop_cluster_scope_invalid")
        grid = self.floor(floor)
        for record in frozen:
            if (
                abs(record.grid_resolution - grid.resolution) > 1e-9
                or record.grid_origin != (grid.origin_x, grid.origin_y)
                or any(
                    not _valid_negative_cell(cell, grid)
                    for cell in record.source_cells
                )
            ):
                return NegativeClusterCommit(False, "drop_cluster_scope_invalid")
            existing = grid.negative_cluster_records.get(record.cluster_id)
            if existing is not None and existing != record:
                return NegativeClusterCommit(False, "drop_cluster_id_conflict")
        additions = tuple(
            record for record in frozen
            if record.cluster_id not in grid.negative_cluster_records
        )
        if not additions:
            return NegativeClusterCommit(
                True, "already_committed", ids, self._negative_cluster_revision
            )
        if not allow_new:
            return NegativeClusterCommit(
                False,
                "negative_obstacle_replan_exhausted",
                ids,
                self._negative_cluster_revision,
            )
        for record in additions:
            grid.negative_cluster_records[record.cluster_id] = record
            grid.negative_cluster_confirmations[record.cluster_id] = frozenset()
            for cell in record.source_cells:
                owners = grid.negative_cell_owners.setdefault(cell, set())
                owners.add(record.cluster_id)
                grid.negative_cells.add(cell)
                grid._refresh_cell(*cell)
        self._negative_cluster_revision += 1
        return NegativeClusterCommit(
            True, "committed", ids, self._negative_cluster_revision
        )

    def clear_negative_clusters_with_support(
        self,
        *,
        floor: int,
        cells: Iterable[tuple[int, int]],
        map_epoch: int,
        localization_epoch: int,
        generation: int,
        grid_origin: tuple[float, float],
        grid_resolution: float,
    ) -> tuple[str, ...]:
        """逐帧累积更新正支撑；完整覆盖来源簇后才原子撤销。"""
        if (
            isinstance(floor, bool)
            or not isinstance(floor, int)
            or floor < 0
        ):
            return ()
        grid = self._floors.get(floor)
        if grid is None:
            return ()
        if (
            tuple(float(value) for value in grid_origin)
            != (grid.origin_x, grid.origin_y)
            or abs(float(grid_resolution) - grid.resolution) > 1e-9
        ):
            return ()
        confirmed = frozenset((int(gx), int(gy)) for gx, gy in cells)
        removable = []
        for record in tuple(grid.negative_cluster_records.values()):
            if (
                record.map_epoch != int(map_epoch)
                or record.localization_epoch != int(localization_epoch)
                or record.grid_origin != tuple(grid_origin)
                or abs(record.grid_resolution - float(grid_resolution)) > 1e-9
                or int(generation) <= record.generation
            ):
                continue
            matched = confirmed & record.source_cells
            if not matched:
                continue
            accumulated = (
                grid.negative_cluster_confirmations.get(
                    record.cluster_id, frozenset()
                ) | matched
            )
            if record.source_cells <= accumulated:
                removable.append(record)
            else:
                grid.negative_cluster_confirmations[record.cluster_id] = frozenset(
                    accumulated
                )
        for record in removable:
            grid.negative_cluster_records.pop(record.cluster_id, None)
            grid.negative_cluster_confirmations.pop(record.cluster_id, None)
            for cell in record.source_cells:
                owners = grid.negative_cell_owners.get(cell, set())
                owners.discard(record.cluster_id)
                if owners:
                    continue
                grid.negative_cell_owners.pop(cell, None)
                grid.negative_cells.discard(cell)
                grid.negative_first_state.pop(cell, None)
                grid._refresh_cell(*cell)
        if removable:
            self._negative_cluster_revision += 1
        return tuple(sorted(record.cluster_id for record in removable))

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

        仅矩形门板条带中的 OCC 会恢复；BLACK 是任务/安全禁区，不能由开门
        动作清除。若格子仍有动态负障来源则恢复为 NEGATIVE，而不是 FREE。
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
        strip_cells: set[tuple[int, int]] = set()
        for gy in range(cy - cells, cy + cells + 1):
            for gx in range(cx - cells, cx + cells + 1):
                if not (0 <= gx < grid.size_x and 0 <= gy < grid.size_y):
                    continue
                wx, wy = grid.grid_to_world(gx, gy)
                dx = wx - x
                dy = wy - y
                along_normal = dx * normal_x + dy * normal_y
                along_tangent = dx * tangent_x + dy * tangent_y
                if abs(along_normal) <= half_depth and abs(along_tangent) <= half_width:
                    strip_cells.add((gx, gy))
        occupied_before_cells = {
            (gx, gy)
            for gx, gy in strip_cells
            if int(grid.data[gy, gx]) == OCC
        }
        # 只移除核心本身落在门板条带内的 owner。按派生 OCC 格匿名擦除会把
        # 门侧墙体的膨胀所有权一并清掉，破坏来源账本。
        removable_cores = []
        for core_cell in tuple(grid.occ_cores):
            wx, wy = grid.grid_to_world(*core_cell)
            dx = wx - x
            dy = wy - y
            along_normal = dx * normal_x + dy * normal_y
            along_tangent = dx * tangent_x + dy * tangent_y
            if abs(along_normal) <= half_depth and abs(along_tangent) <= half_width:
                removable_cores.append(core_cell)
        for core_cell in removable_cores:
            grid.remove_obstacle_core(core_cell)
        for gx, gy in occupied_before_cells:
            if int(grid.data[gy, gx]) not in (OCC, BLACK, NEGATIVE):
                grid.set_base_state(gx, gy, FREE)
        occupied_after = sum(
            int(grid.data[gy, gx]) == OCC for gx, gy in strip_cells
        )
        cleared_count = max(0, len(occupied_before_cells) - occupied_after)
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
            if stop_at_blocked and int(grid.data[gy, gx]) in (OCC, BLACK, NEGATIVE):
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
            if cell_value in (OCC, BLACK, NEGATIVE):
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
