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

from dataclasses import dataclass, replace
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
    protected_trail_cell_count: int = 0
    protected_trail_confirmed_core_count: int = 0


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


def compute_door_strip_cells(
    x: float,
    y: float,
    yaw: float,
    width: float,
    depth: float,
    grid: "FloorGrid",
) -> set[tuple[int, int]]:
    """计算门板矩形条带覆盖的栅格格子。

    Args:
        x, y: 门中心世界坐标
        yaw: 门法向角度（rad）
        width: 门宽（沿切向）
        depth: 门深（沿法向）
        grid: 目标栅格

    Returns:
        门板条带覆盖的格子集合 {(gx, gy), ...}
    """
    center = grid.world_to_grid(x, y)
    if center is None:
        return set()

    cx, cy = center
    half_width = max(0.0, float(width)) * 0.5
    half_depth = max(0.0, float(depth)) * 0.5

    if half_width <= 0.0 or half_depth <= 0.0:
        return set()

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

    return strip_cells


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

    架构说明：
    - observation_layer (原 base): 观测基础层，由传感器直接驱动
      * UNKNOWN: 未观测
      * FREE: 传感器确认可通行
      * VISITED: 机器人已到达
    - planning_layer (原 _derived): 规划约束层，叠加多个约束源
      * 继承 observation_layer 的 FREE/VISITED
      * 叠加 OCC (正障碍)、BLACK (任务禁区)、NEGATIVE (负障碍)
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

        # 观测基础层：由传感器直接驱动 (UNKNOWN/FREE/VISITED)
        self.base = np.full((self.size_y, self.size_x), UNKNOWN, dtype=np.int16)
        self.observation_layer = self.base  # 语义别名

        # 规划约束层：叠加约束后的规划视图 (+ OCC/BLACK/NEGATIVE)
        self._derived = np.full((self.size_y, self.size_x), UNKNOWN, dtype=np.int16)
        self.planning_layer = self._derived  # 语义别名

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
        # 已确认危险源由检测链独立登记为语义 OCC。它不属于点云证据账本，
        # 因而不会被自由射线、动态门清理或 evidence epoch 切换误删。
        self.semantic_occupied_owners: dict[tuple[int, int], set[str]] = {}
        self.semantic_occupied_owner_cells: dict[str, set[tuple[int, int]]] = {}
        self.protected_trail_cells: set[tuple[int, int]] = set()
        self.last_protected_trail_xy: tuple[float, float] | None = None

        # NEGATIVE 可能暂时被 OCC/BLACK 覆盖；单独保存来源，动态门清除 OCC 时
        # 才能恢复负障，而直接楼面证据也只能移除这一来源。
        self.negative_cells: set[tuple[int, int]] = set()
        self.negative_cluster_records: dict[str, NegativeClusterRecord] = {}
        self.negative_cell_owners: dict[tuple[int, int], set[str]] = {}
        self.negative_first_state: dict[tuple[int, int], int] = {}
        self.negative_cluster_confirmations: dict[
            str, frozenset[tuple[int, int]]
        ] = {}
        self.negative_cluster_inflated_cells: dict[
            str, set[tuple[int, int]]
        ] = {}

        # 批量刷新优化：标记脏格子，延迟刷新
        self._dirty_cells: set[tuple[int, int]] = set()

    @property
    def data(self) -> np.ndarray:
        """返回规划约束层的只读视图（向后兼容接口）。"""
        view = self.planning_layer.view()
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

    def _mark_dirty(self, gx: int, gy: int) -> None:
        """标记格子为脏，延迟刷新以减少重复计算。"""
        cell = (int(gx), int(gy))
        if self._in_bounds(*cell):
            self._dirty_cells.add(cell)

    def _flush_dirty_cells(self) -> None:
        """批量刷新所有标记为脏的格子。"""
        for cell in self._dirty_cells:
            self._refresh_cell(*cell)
        self._dirty_cells.clear()

    def _refresh_cell(self, gx: int, gy: int) -> None:
        cell = (int(gx), int(gy))
        if not self._in_bounds(*cell):
            return
        if self.semantic_occupied_owners.get(cell):
            state = OCC
        elif cell in self.protected_trail_cells:
            if self.black_owners.get(cell):
                state = BLACK
            elif self.negative_cell_owners.get(cell):
                state = NEGATIVE
            elif self.obstacle_evidence.protected_trail_blocking_at(cell):
                state = OCC
            else:
                state = int(self.observation_layer[cell[1], cell[0]])
        else:
            if self.obstacle_evidence.blocking_at(cell):
                state = OCC
            elif self.black_owners.get(cell):
                state = BLACK
            elif self.negative_cell_owners.get(cell):
                state = NEGATIVE
            else:
                state = int(self.observation_layer[cell[1], cell[0]])
        self.planning_layer[cell[1], cell[0]] = state

    def _refresh_cells(self, cells: Iterable[tuple[int, int]]) -> None:
        for gx, gy in cells:
            self._refresh_cell(gx, gy)

    def set_base_state(self, gx: int, gy: int, state: int) -> None:
        if int(state) not in (UNKNOWN, FREE, VISITED):
            raise ValueError("基础层只允许 UNKNOWN、FREE 或 VISITED")
        if self._in_bounds(gx, gy):
            self.observation_layer[int(gy), int(gx)] = int(state)
            self._refresh_cell(gx, gy)

    def fill_base(self, state: int) -> None:
        if int(state) not in (UNKNOWN, FREE, VISITED):
            raise ValueError("基础层只允许 UNKNOWN、FREE 或 VISITED")
        self.observation_layer[:, :] = int(state)
        for gy in range(self.size_y):
            for gx in range(self.size_x):
                self._refresh_cell(gx, gy)

    def load_legacy_data(self, values) -> None:
        data = np.asarray(values, dtype=np.int16)
        if data.shape != (self.size_y, self.size_x):
            raise ValueError("旧栅格快照尺寸不匹配")
        self.observation_layer[:, :] = UNKNOWN
        self.obstacle_evidence = ObstacleEvidenceLedger(
            floor_index=self.floor_index,
            size_x=self.size_x,
            size_y=self.size_y,
            resolution=self.resolution,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
        )
        self.black_owners.clear()
        self.semantic_occupied_owners.clear()
        self.semantic_occupied_owner_cells.clear()
        self.protected_trail_cells.clear()
        self.last_protected_trail_xy = None
        self.negative_cells.clear()
        self.negative_cell_owners.clear()
        for gy in range(self.size_y):
            for gx in range(self.size_x):
                state = int(data[gy, gx])
                if state in (UNKNOWN, FREE, VISITED):
                    self.observation_layer[gy, gx] = state
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
        self.planning_layer[:, :] = UNKNOWN
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
        # 直接复制底层数组，然后重建别名
        result.base[:] = self.observation_layer
        result.observation_layer = result.base
        result._derived[:] = self.planning_layer
        result.planning_layer = result._derived

        result.obstacle_evidence = self.obstacle_evidence.copy()
        result.evidence_scope = tuple(self.evidence_scope)
        result.black_owners = {
            cell: set(owners) for cell, owners in self.black_owners.items()
        }
        result.semantic_occupied_owners = {
            cell: set(owners)
            for cell, owners in self.semantic_occupied_owners.items()
        }
        result.semantic_occupied_owner_cells = {
            owner: set(cells)
            for owner, cells in self.semantic_occupied_owner_cells.items()
        }
        result.protected_trail_cells = set(self.protected_trail_cells)
        result.last_protected_trail_xy = self.last_protected_trail_xy
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
            self.observation_layer[gy, gx] = int(state)
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
        affected = (
            set(self.inflation_owners)
            | set(self.negative_cell_owners)
            | set(self.semantic_occupied_owners)
        )
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

    def transform_planar(self, dx: float, dy: float, yaw: float) -> None:
        """将楼层栅格及其证据索引整体变换到新的世界系。"""
        values = (float(dx), float(dy), float(yaw))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("楼层栅格 SE(2) 变换量必须为有限数")
        cosine = math.cos(values[2])
        sine = math.sin(values[2])
        rotation = np.asarray(((cosine, -sine), (sine, cosine)), dtype=float)
        offset = np.asarray(values[:2], dtype=float)

        def transform_cell(x: float, y: float):
            transformed = rotation @ np.asarray((float(x), float(y))) + offset
            return self.world_to_grid(float(transformed[0]), float(transformed[1]))

        def remap(source: np.ndarray) -> np.ndarray:
            target = np.full_like(source, UNKNOWN)
            ys, xs = np.nonzero(source != UNKNOWN)
            for gy, gx in zip(ys.tolist(), xs.tolist()):
                destination = transform_cell(*self.grid_to_world(gx, gy))
                if destination is None:
                    continue
                tx, ty = destination
                state = int(source[gy, gx])
                if state > int(target[ty, tx]):
                    target[ty, tx] = state
            return target

        self.base[:] = remap(self.observation_layer)
        self._derived[:] = remap(self.planning_layer)

        def transform_cells(cells):
            return {
                destination
                for destination in (
                    transform_cell(*self.grid_to_world(*cell)) for cell in cells
                )
                if destination is not None
            }

        self.black_owners = {
            cell: set(owners)
            for cell, owners in self._transform_owner_map(self.black_owners, transform_cells).items()
        }
        transformed_semantic = self._transform_owner_map(
            self.semantic_occupied_owners, transform_cells
        )
        self.semantic_occupied_owners = transformed_semantic
        self.semantic_occupied_owner_cells = self._invert_owner_map(transformed_semantic)
        self.protected_trail_cells = transform_cells(self.protected_trail_cells)
        if self.last_protected_trail_xy is not None:
            self.last_protected_trail_xy = tuple(
                rotation @ np.asarray(self.last_protected_trail_xy) + offset
            )
        self.negative_cells = transform_cells(self.negative_cells)
        self.negative_cell_owners = self._transform_owner_map(
            self.negative_cell_owners, transform_cells
        )
        self.negative_first_state = {
            cell: state
            for cell, state in (
                (destination, self.negative_first_state[source])
                for source in self.negative_first_state
                for destination in (
                    transform_cell(*self.grid_to_world(*source)),
                )
                if destination is not None
            )
        }
        self.negative_cluster_confirmations = {
            cluster_id: frozenset(transform_cells(cells))
            for cluster_id, cells in self.negative_cluster_confirmations.items()
        }
        self.negative_cluster_records = {
            cluster_id: replace(
                record,
                source_cells=frozenset(transform_cells(record.source_cells)),
            )
            for cluster_id, record in self.negative_cluster_records.items()
        }
        self.obstacle_evidence.transform_planar((rotation, offset, transform_cell))
        self._dirty_cells = set()
        self._refresh_cells(
            (gx, gy)
            for gy in range(self.size_y)
            for gx in range(self.size_x)
        )

    @staticmethod
    def _transform_owner_map(source, transform_cells):
        result = {}
        for cell, owners in source.items():
            destination_set = transform_cells((cell,))
            for destination in destination_set:
                result.setdefault(destination, set()).update(owners)
        return result

    @staticmethod
    def _invert_owner_map(source):
        result = {}
        for cell, owners in source.items():
            for owner in owners:
                result.setdefault(owner, set()).add(cell)
        return result

    def is_traversable(self, gx: int, gy: int) -> bool:
        if not (0 <= gx < self.size_x and 0 <= gy < self.size_y):
            return False
        return int(self.data[gy, gx]) in (FREE, VISITED)

    def without_negative_obstacles(self) -> "FloorGrid":
        """为已保证无空洞的跨越生成规划副本，不撤销实体障碍或任务禁区。"""
        result = self.copy()
        cells = set(result.negative_cell_owners) | set(result.negative_cells)
        result.negative_cell_owners.clear()
        result.negative_cells.clear()
        result._refresh_cells(cells)
        return result

    def is_planning_traversable(self, gx: int, gy: int) -> bool:
        """规划层可通行性（考虑所有约束：障碍、黑区、负障碍）。

        等价于 is_traversable，但语义更明确。
        """
        if not self._in_bounds(gx, gy):
            return False
        value = self.planning_layer[gy, gx]
        return value in (FREE, VISITED)

    def is_observation_free(self, gx: int, gy: int) -> bool:
        """观测层可通行性（仅传感器观测，不考虑任务约束）。

        用于判断传感器是否认为此处可通行，忽略 BLACK/NEGATIVE 等任务层约束。
        """
        if not self._in_bounds(gx, gy):
            return False
        value = self.observation_layer[gy, gx]
        return value in (FREE, VISITED)

    def get_constraint_kind(self, gx: int, gy: int):
        """返回阻挡此格子的约束类型。

        Returns:
            "obstacle" - 正障碍（传感器检测到的实体障碍）
            "black" - 黑区（任务层禁区）
            "negative" - 负障碍（楼面支撑约束）
            None - 无约束阻挡
        """
        cell = (int(gx), int(gy))
        if not self._in_bounds(*cell):
            return None

        if self.semantic_occupied_owners.get(cell):
            return "danger_obstacle"
        if self.obstacle_evidence.blocking_at(cell):
            return "obstacle"
        if self.black_owners.get(cell):
            return "black"
        if self.negative_cell_owners.get(cell):
            return "negative"
        return None

    def mark_free(self, gx: int, gy: int) -> None:
        """仅把 UNKNOWN 变为 FREE，绝不覆盖已知障碍或任务禁区。"""
        if self._in_bounds(gx, gy) and int(self.observation_layer[gy, gx]) == UNKNOWN:
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

    def replace_semantic_occupied_radius(
        self,
        x: float,
        y: float,
        radius: float,
        *,
        owner: str,
    ) -> frozenset[tuple[int, int]]:
        """以所有者为单位原子替换一块语义 OCC 圆形覆盖。"""
        center_x = float(x)
        center_y = float(y)
        safe_radius = float(radius)
        owner_id = str(owner).strip()
        if (
            not owner_id
            or not all(
                math.isfinite(value)
                for value in (center_x, center_y, safe_radius)
            )
            or safe_radius < 0.0
        ):
            raise ValueError("语义 OCC 的位置、半径或所有者无效")
        previous = set(self.semantic_occupied_owner_cells.get(owner_id, ()))
        replacement = self._semantic_occupied_cells(
            center_x,
            center_y,
            safe_radius,
        )
        if previous == replacement:
            return frozenset()

        for cell in previous:
            owners = self.semantic_occupied_owners.get(cell)
            if owners is None:
                continue
            owners.discard(owner_id)
            if not owners:
                self.semantic_occupied_owners.pop(cell, None)
        if replacement:
            self.semantic_occupied_owner_cells[owner_id] = set(replacement)
            for cell in replacement:
                self.semantic_occupied_owners.setdefault(cell, set()).add(owner_id)
        else:
            self.semantic_occupied_owner_cells.pop(owner_id, None)
        affected = previous | replacement
        self._refresh_cells(affected)
        return frozenset(affected)

    def clear_semantic_occupied_owner(
        self,
        owner: str,
    ) -> frozenset[tuple[int, int]]:
        """仅撤销指定所有者的语义 OCC，不影响重叠的其他危险源。"""
        owner_id = str(owner).strip()
        if not owner_id:
            return frozenset()
        cells = set(self.semantic_occupied_owner_cells.pop(owner_id, ()))
        for cell in cells:
            owners = self.semantic_occupied_owners.get(cell)
            if owners is None:
                continue
            owners.discard(owner_id)
            if not owners:
                self.semantic_occupied_owners.pop(cell, None)
        self._refresh_cells(cells)
        return frozenset(cells)

    def semantic_occupied_owner_ids(self, *, prefix: str | None = None) -> set[str]:
        """返回语义 OCC 所有者，供运行时精确撤销被 NMS 抑制的危险源。"""
        owners = set(self.semantic_occupied_owner_cells)
        if prefix is None:
            return owners
        return {owner for owner in owners if owner.startswith(str(prefix))}

    def _semantic_occupied_cells(
        self,
        x: float,
        y: float,
        radius: float,
    ) -> set[tuple[int, int]]:
        center = self.world_to_grid(x, y)
        if center is None:
            return set()
        cells = int(math.ceil(float(radius) / self.resolution))
        result: set[tuple[int, int]] = set()
        for gy in range(center[1] - cells, center[1] + cells + 1):
            for gx in range(center[0] - cells, center[0] + cells + 1):
                if not self._in_bounds(gx, gy):
                    continue
                wx, wy = self.grid_to_world(gx, gy)
                if math.hypot(wx - x, wy - y) <= radius + 1e-9:
                    result.add((gx, gy))
        return result

    def clear_region(self, cells: Iterable[tuple[int, int]]) -> int:
        """清除指定格子中的正障碍，恢复为观测层状态或负障碍。

        只移除核心本身落在指定区域内的障碍；若格子仍有负障碍来源则恢复为
        NEGATIVE，否则恢复为观测层状态（通常是 FREE）。BLACK 是任务/安全
        禁区，不会被清除。

        Args:
            cells: 要清理的格子集合 (gx, gy)

        Returns:
            实际清除的 OCC 格子数量
        """
        cells_set = set(cells)

        # 记录清理前的 OCC 状态
        occupied_before = {
            cell for cell in cells_set
            if self._in_bounds(*cell) and int(self.planning_layer[cell[1], cell[0]]) == OCC
        }

        # 只移除核心本身落在区域内的障碍
        removable_cores = [
            core_cell for core_cell in tuple(self.obstacle_evidence.cores)
            if core_cell in cells_set
        ]

        for core_cell in removable_cores:
            self.remove_obstacle_core(core_cell)

        # 恢复清理后非阻挡格子为观测层状态
        for gx, gy in occupied_before:
            current_state = int(self.planning_layer[gy, gx])
            if current_state not in (OCC, BLACK, NEGATIVE):
                restored_state = (
                    VISITED if (gx, gy) in self.protected_trail_cells else FREE
                )
                self.set_base_state(gx, gy, restored_state)

        # 计算实际清除数量
        occupied_after = sum(
            int(self.planning_layer[cell[1], cell[0]]) == OCC
            for cell in cells_set
            if self._in_bounds(*cell)
        )

        return max(0, len(occupied_before) - occupied_after)

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

    def protect_trail_radius(self, x: float, y: float, radius: float) -> None:
        """把机身实际通过的核心区域登记为本层持久 VISITED。"""
        current = (float(x), float(y))
        if self.world_to_grid(*current) is None:
            return
        start = self.last_protected_trail_xy or current
        distance = math.hypot(current[0] - start[0], current[1] - start[1])
        # 仅连接能够由相邻足迹圆连续覆盖的位姿；较大跳变可能来自定位修正或
        # 长时间无点云，不能据此臆造机器人实际走过的永久自由通道。
        if distance > max(2.0 * float(radius), self.resolution) + 1e-9:
            start = current
            distance = 0.0
        samples = max(
            1,
            int(math.ceil(distance / max(self.resolution * 0.5, 1e-9))),
        )
        for index in range(samples + 1):
            ratio = index / samples
            sample = (
                start[0] + (current[0] - start[0]) * ratio,
                start[1] + (current[1] - start[1]) * ratio,
            )
            self._protect_trail_sample(*sample, radius=float(radius))
        self.last_protected_trail_xy = current

    def _protect_trail_sample(self, x: float, y: float, *, radius: float) -> None:
        center = self.world_to_grid(x, y)
        if center is None:
            return
        cells = int(math.ceil(float(radius) / self.resolution))
        for gy in range(center[1] - cells, center[1] + cells + 1):
            for gx in range(center[0] - cells, center[0] + cells + 1):
                if not self._in_bounds(gx, gy):
                    continue
                wx, wy = self.grid_to_world(gx, gy)
                if (
                    math.hypot(wx - float(x), wy - float(y))
                    > float(radius) + 1e-9
                ):
                    continue
                self.protected_trail_cells.add((gx, gy))
                self.observation_layer[gy, gx] = VISITED
                self._refresh_cell(gx, gy)

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
        negative_obstacle_inflation_radius: float = 0.40,
        raycast_max_range: float = 12.0,
        point_free_clear_max_range: float = 8.0,
        visited_radius: float = 0.45,
        free_clear_enabled: bool = True,
        free_clear_max_range: float = 8.0,
        free_clear_angle_step: float = 0.1745,
        free_clear_obstacle_margin: float = 0.25,
        free_clear_min_points_per_bin: int = 2,
        negative_minimum_drop: float = 0.20,
        evidence_lifecycle_enabled: bool = True,
        low_height_max_z: float = 0.30,
        low_confirm_generations: int = 2,
        low_confirm_window_generations: int = 20,
        low_candidate_counter_generations: int = 3,
        low_candidate_counter_positions: int = 2,
        low_confirmed_counter_generations: int = 6,
        low_confirmed_counter_positions: int = 3,
        counter_pose_separation: float = 0.15,
        low_near_exempt_radius: float = 0.0,
        low_candidate_max_range: float = 0.0,
        trail_protection_enabled: bool = False,
        trail_protection_radius: float = 0.25,
        trail_protection_near_range: float = 2.0,
        trail_protection_confirm_generations: int = 3,
        trail_protection_confirm_positions: int = 2,
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
        self.negative_obstacle_inflation_radius = float(negative_obstacle_inflation_radius)
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
        self.trail_protection_enabled = bool(trail_protection_enabled)
        self.trail_protection_radius = float(trail_protection_radius)
        self.trail_protection_near_range = float(trail_protection_near_range)
        self.trail_protection_confirm_generations = int(
            trail_protection_confirm_generations
        )
        self.trail_protection_confirm_positions = int(
            trail_protection_confirm_positions
        )
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
            self.negative_obstacle_inflation_radius,
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
            self.trail_protection_radius,
            self.trail_protection_near_range,
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
            or self.negative_obstacle_inflation_radius < 0.0
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
            or self.trail_protection_radius < 0.0
            or self.trail_protection_near_range <= 0.0
            or self.trail_protection_confirm_generations <= 0
            or self.trail_protection_confirm_positions <= 0
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

    @property
    def instantiated_floor_indices(self) -> tuple[int, ...]:
        """返回当前已创建的楼层，避免语义层为未知楼层凭空建图。"""
        return tuple(sorted(self._floors))

    def transform_planar(self, dx: float, dy: float, yaw: float) -> tuple[int, ...]:
        """同步变换全部已实例化楼层及其世界坐标证据。"""
        values = (float(dx), float(dy), float(yaw))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("地图 SE(2) 变换量必须为有限数")
        for grid in self._floors.values():
            grid.transform_planar(*values)
        return self.instantiated_floor_indices

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
        if self.trail_protection_enabled:
            grid.protect_trail_radius(
                robot_xyz[0], robot_xyz[1], self.trail_protection_radius
            )
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
                    # 方案 2：低高度障碍分类与近身自腿豁免解耦
                    # 1. 严格自腿过滤：
                    # 低高度且在自腿豁免半径内，坚决不在脚下记账，彻底消除起步自绊自锁风险。
                    # （视线遮挡语义保留，该点已在 obstacle_hit_cells 中截断自由射线）
                    if (
                        nominal_z_rel < self.low_height_max_z
                        and self.low_near_exempt_radius > 0.0
                        and _distance < self.low_near_exempt_radius
                    ):
                        continue

                    # 2. 障碍物分类裁决：
                    if nominal_z_rel < self.low_height_max_z:
                        # 生产模式（low_near_exempt_radius > 0）：近身自腿已被过滤，前方视距内的低矮点
                        # 物理上 100% 为真实障碍物（0.24m 托盘），直接判定为 HIGH_HARD 首帧阻挡并膨胀，避免切角踏上托盘；
                        # 未开启豁免（如纯生命周期单测）或远距不可靠回波保持 LOW_CANDIDATE 候选账本。
                        if (
                            self.low_near_exempt_radius > 0.0
                            and (self.point_free_clear_max_range <= 0.0 or _distance <= self.point_free_clear_max_range)
                            and (self.low_candidate_max_range <= 0.0 or _distance <= self.low_candidate_max_range)
                        ):
                            kind = HIGH_HARD
                        else:
                            kind = LOW_CANDIDATE
                    else:
                        if (
                            self.point_free_clear_max_range > 0.0
                            and _distance > self.point_free_clear_max_range
                        ):
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
                    core = grid.occ_cores.get(_target_cell)
                    if (
                        self.trail_protection_enabled
                        and _distance <= self.trail_protection_near_range
                        and core is not None
                        and bool(core.inflated_cells & grid.protected_trail_cells)
                    ):
                        changed = (
                            grid.obstacle_evidence.record_protected_trail_observation(
                                px,
                                py,
                                cloud_generation=int(cloud_generation),
                                robot_xy=(robot_xyz[0], robot_xyz[1]),
                                confirm_generations=(
                                    self.trail_protection_confirm_generations
                                ),
                                confirm_positions=(
                                    self.trail_protection_confirm_positions
                                ),
                                generation_window=self.low_confirm_window_generations,
                                pose_separation=self.counter_pose_separation,
                            )
                        )
                        grid._refresh_cells(changed)

        for px, py, pz, distance, is_obstacle_point, target_cell in observations:
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
                    robot_z=float(robot_xyz[2]),
                    point_z=float(pz),
                    distance=float(distance),
                    floor_base_z=float(floor_base_z),
                    low_height_max_z=float(self.low_height_max_z),
                )
                stats.low_core_removed_count += removed
                self._low_core_removed_total += removed
            clear_length, stopped_at_blocked = self._raycast_free(
                grid,
                robot_cell,
                ray_target,
                stop_at_blocked=True,
                blocked_cells=obstacle_hit_cells,
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
            protected_trail_cell_count=len(grid.protected_trail_cells),
            protected_trail_confirmed_core_count=sum(
                core.protected_trail_confirmed
                for core in grid.occ_cores.values()
            ),
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
        inflation_radius = getattr(self, "negative_obstacle_inflation_radius", 0.40)
        radius_cells = max(0, int(math.ceil(inflation_radius / grid.resolution)))
        for record in additions:
            grid.negative_cluster_records[record.cluster_id] = record
            grid.negative_cluster_confirmations[record.cluster_id] = frozenset()
            inflated_for_record = set()
            for cx, cy in record.source_cells:
                for dx in range(-radius_cells, radius_cells + 1):
                    for dy in range(-radius_cells, radius_cells + 1):
                        if math.hypot(dx * grid.resolution, dy * grid.resolution) <= inflation_radius + 1e-6:
                            nx, ny = cx + dx, cy + dy
                            if grid._in_bounds(nx, ny):
                                cell = (nx, ny)
                                inflated_for_record.add(cell)
                                owners = grid.negative_cell_owners.setdefault(cell, set())
                                owners.add(record.cluster_id)
                                grid.negative_cells.add(cell)
                                grid._refresh_cell(*cell)
            if hasattr(grid, "negative_cluster_inflated_cells"):
                grid.negative_cluster_inflated_cells[record.cluster_id] = inflated_for_record
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
            affected_cells = set(record.source_cells)
            if hasattr(grid, "negative_cluster_inflated_cells"):
                affected_cells.update(
                    grid.negative_cluster_inflated_cells.pop(record.cluster_id, set())
                )
            for cell in affected_cells:
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

        注：此方法是便利包装，内部使用 compute_door_strip_cells + clear_region。
        """
        grid = self.floor(floor_index)
        strip_cells = compute_door_strip_cells(x, y, yaw, width, depth, grid)
        cleared_count = grid.clear_region(strip_cells)

        self._pending_occupied_to_free_count += cleared_count
        self._pending_dynamic_door_cleared_count += cleared_count

    def _raycast_free(
        self,
        grid: FloorGrid,
        start: tuple[int, int],
        end: tuple[int, int],
        *,
        stop_at_blocked: bool = True,
        blocked_cells: frozenset[tuple[int, int]] = frozenset(),
    ) -> tuple[float, bool]:
        last_cleared = start
        for gx, gy in _bresenham(start[0], start[1], end[0], end[1])[:-1]:
            if stop_at_blocked and (
                (gx, gy) in blocked_cells
                or int(grid.data[gy, gx]) in (OCC, BLACK, NEGATIVE)
            ):
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
