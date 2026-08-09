"""在单层占据栅格上执行 A* 路径规划与视线压缩。

规划器只接受 Mapper 判定为 FREE/VISITED 的格子，不对 UNKNOWN 做乐观穿越。
返回状态显式区分“开放集确实耗尽”和“本轮迭代预算用完”：前者允许探索器
累计失败并拉黑目标，后者只表示计算尚未完成，不能据此破坏可达性判断。
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

from danger_search.map_evidence import BLOCKING_OBSTACLE_KINDS, LOW_CONFIRMED
from danger_search.mapper import FREE, OCC, UNKNOWN, VISITED, FloorGrid

# 默认迭代预算：在当前配置尺寸（28x48m, 0.10m 分辨率）下，
# 复杂绕行的全量探索可超过一个 10 Hz Timer 周期。
# 20000 次迭代实测约 0.05~0.08s，既限制单次阻塞时长，
# 又给正常可达目标留足搜索空间。
DEFAULT_MAX_ITERATIONS = 20000

REACHED = "reached"
UNREACHABLE = "unreachable"
BUDGET_EXHAUSTED = "budget_exhausted"
DIAGONAL_COST = math.sqrt(2.0)


@dataclass
class PlanResult:
    """A* 规划结果，携带状态以区分“确实不可达”与“预算耗尽”。

    只有 ``REACHED`` 保证 ``path`` 从起点连续到目标；另外两种状态的路径为空。
    """
    path: list[tuple[float, float]] = field(default_factory=list)
    status: str = UNREACHABLE
    reason_code: str | None = None
    blocker: dict | None = None

    @property
    def reached(self) -> bool:
        return self.status == REACHED

    @property
    def budget_exhausted(self) -> bool:
        return self.status == BUDGET_EXHAUSTED

    @property
    def unreachable(self) -> bool:
        return self.status == UNREACHABLE


@dataclass(frozen=True)
class SupportVetoRegion:
    """一次支撑失败产生的冻结临时约束。"""

    region_id: str
    blocked_cells: frozenset[tuple[int, int]]
    source_cells: frozenset[tuple[int, int]]


@dataclass(frozen=True)
class PlanningConstraints:
    """单次规划使用的只读派生约束，不修改 ``FloorGrid``。"""

    blocked_cells: frozenset[tuple[int, int]] = frozenset()
    support_regions: tuple[SupportVetoRegion, ...] = ()
    revision: int = 0

    @classmethod
    def from_regions(
        cls,
        regions: tuple[SupportVetoRegion, ...],
        *,
        revision: int,
    ) -> "PlanningConstraints":
        return cls(
            blocked_cells=frozenset(
                cell for region in regions for cell in region.blocked_cells
            ),
            support_regions=tuple(regions),
            revision=int(revision),
        )


@dataclass(frozen=True)
class _StartEscapeContext:
    required: bool
    active_cells: frozenset[tuple[int, int]] = frozenset()
    foreign_blocked: frozenset[tuple[int, int]] = frozenset()
    source_sets: tuple[frozenset[tuple[int, int]], ...] = ()
    escape_mask: object | None = None
    failure_reason: str | None = None
    unreachable_reason: str = "start_escape_unreachable"
    blocker: dict | None = None


def _resolve_start_escape(
    grid: FloorGrid,
    start: tuple[int, int],
    *,
    constraints: PlanningConstraints | None,
    formal,
    effective,
) -> _StartEscapeContext:
    """冻结起点的支撑约束与障碍 owner，并构造唯一认证退出域。"""
    support_blocked = bool(
        constraints is not None and start in constraints.blocked_cells
    )
    obstacle_blocked = int(grid.data[start[1], start[0]]) == OCC
    if not support_blocked and not obstacle_blocked:
        return _StartEscapeContext(False, escape_mask=formal)

    active_regions: tuple[SupportVetoRegion, ...] = ()
    active_support_cells: frozenset[tuple[int, int]] = frozenset()
    foreign_support: frozenset[tuple[int, int]] = frozenset()
    if support_blocked and constraints is not None:
        active_regions = tuple(
            region
            for region in constraints.support_regions
            if start in region.blocked_cells
        )
        if not active_regions or any(
            not region.source_cells for region in active_regions
        ):
            return _StartEscapeContext(
                True, failure_reason="start_cell_planning_veto"
            )
        active_ids = {region.region_id for region in active_regions}
        active_support_cells = frozenset(
            cell for region in active_regions for cell in region.blocked_cells
        )
        represented = frozenset(
            cell
            for region in constraints.support_regions
            for cell in region.blocked_cells
        )
        foreign_support = frozenset(
            cell
            for region in constraints.support_regions
            if region.region_id not in active_ids
            for cell in region.blocked_cells
        ) | (constraints.blocked_cells - represented)

    active_owner_ids: frozenset[tuple[int, int]] = frozenset()
    active_obstacle_cells: frozenset[tuple[int, int]] = frozenset()
    foreign_obstacles: set[tuple[int, int]] = set()
    start_core_self = False
    if obstacle_blocked:
        start_core_record = (
            grid.occ_cores.get(start)
            if start in grid.occ_cores
            else None
        )
        if (
            start_core_record is not None
            and start_core_record.kind in BLOCKING_OBSTACLE_KINDS
        ):
            if start_core_record.kind == LOW_CONFIRMED:
                # 起点格本体为 LOW_CONFIRMED 核心（多帧低回波确认）：机器人 pose
                # 当前落在此格 = 物理可通行事实（seed=8127 链 B 取证：机身前上方
                # Mid360 自腿低回波被多帧确认成脚下核心）。本次规划放开起点格本体
                # 及其膨胀邻域中 base 可通行的格子（自腿伪影圈内的物理空地，不
                # 修改正式地图、其余核心照常阻挡），与"膨胀覆盖"同构走逃出流程；
                # escape 失败仍按不可达兜底。
                start_core_self = True
                neighbor_cells = {start}
                record = start_core_record
                neighbor_cells |= {
                    cell
                    for cell in record.inflated_cells
                    if int(grid.observation_layer[cell[1], cell[0]]) in (FREE, VISITED)
                }
                active_obstacle_cells = frozenset(neighbor_cells)
            else:
                # HIGH_HARD/LEGACY_HARD 是可靠实体障碍（贴墙/显式写入），起点为其
                # 本体时保持 fail-closed，不豁免。
                return _StartEscapeContext(
                    True,
                    failure_reason="start_cell_obstacle_core",
                    blocker={"start_cell": start},
                )
        else:
            active_owner_ids = frozenset(
                owner
                for owner in grid.inflation_owners.get(start, ())
                if owner in grid.occ_cores
                and grid.occ_cores[owner].kind in BLOCKING_OBSTACLE_KINDS
            )
            if not active_owner_ids:
                return _StartEscapeContext(
                    True, failure_reason="start_cell_blocked"
                )
        scope = tuple(getattr(grid, "evidence_scope", (grid.floor_index, 1, 1)))
        if any(
            (
                grid.occ_cores[owner].floor_index,
                grid.occ_cores[owner].map_epoch,
                grid.occ_cores[owner].localization_epoch,
            )
            != scope
            for owner in active_owner_ids
        ):
            return _StartEscapeContext(
                True,
                failure_reason="start_obstacle_owner_scope_mismatch",
                blocker={"owner_cells": tuple(sorted(active_owner_ids))},
            )
        if not start_core_self:
            active_obstacle_cells = frozenset(
                cell
                for owner in active_owner_ids
                for cell in grid.occ_cores[owner].inflated_cells
            )
        for owner, record in grid.occ_cores.items():
            if (
                owner not in active_owner_ids
                and not (start_core_self and owner == start)
                and record.kind in BLOCKING_OBSTACLE_KINDS
            ):
                foreign_obstacles.update(record.inflated_cells)

    active_cells = active_support_cells | active_obstacle_cells
    foreign = foreign_support | frozenset(foreign_obstacles)
    escape_mask = formal.copy()
    for gx, gy in active_obstacle_cells:
        if (
            int(grid.observation_layer[gy, gx]) in (FREE, VISITED)
            and not grid.black_owners.get((gx, gy))
            and not grid.negative_cell_owners.get((gx, gy))
            and (gx, gy) not in foreign
        ):
            escape_mask[gy, gx] = True
    if start in foreign or not escape_mask[start[1], start[0]]:
        return _StartEscapeContext(
            True,
            failure_reason=(
                "start_cell_planning_veto"
                if support_blocked and not obstacle_blocked
                else "start_escape_foreign_blocker"
            ),
        )
    source_sets = tuple(region.source_cells for region in active_regions) + tuple(
        frozenset({owner}) for owner in sorted(active_owner_ids)
    )
    if support_blocked and obstacle_blocked:
        unreachable_reason = "start_escape_unreachable"
    elif support_blocked:
        unreachable_reason = "support_veto_escape_unreachable"
    else:
        unreachable_reason = "obstacle_inflation_escape_unreachable"
    blocker = {
        "support_region_ids": tuple(
            sorted(region.region_id for region in active_regions)
        ),
        "obstacle_owner_cells": tuple(sorted(active_owner_ids)),
        "active_cell_count": len(active_cells),
        "start_core_self": start_core_self,
    }
    return _StartEscapeContext(
        True,
        active_cells=active_cells,
        foreign_blocked=foreign,
        source_sets=source_sets,
        escape_mask=escape_mask,
        unreachable_reason=unreachable_reason,
        blocker=blocker,
    )


def reachable_traversable_component(
    grid: FloorGrid,
    start_xy: tuple[float, float],
    *,
    constraints: PlanningConstraints | None = None,
) -> frozenset[tuple[int, int]]:
    """按 A* 相同掩码和起点逃逸规则返回可达正常规划域。"""
    start = grid.world_to_grid(*start_xy)
    if start is None:
        return frozenset()
    formal = _traversable_mask(grid, None)
    effective = _traversable_mask(grid, constraints)
    escape = _resolve_start_escape(
        grid, start, constraints=constraints, formal=formal, effective=effective
    )
    if escape.failure_reason is not None:
        return frozenset()
    if not escape.required and not effective[start[1], start[0]]:
        if int(grid.data[start[1], start[0]]) == UNKNOWN:
            effective[start[1], start[0]] = True
            formal[start[1], start[0]] = True
        else:
            return frozenset()
    start_state = (start, not escape.required)
    queue = [start_state]
    visited = {start_state}
    normal_cells: set[tuple[int, int]] = set()
    while queue:
        current, escaped = queue.pop()
        if escaped:
            normal_cells.add(current)
        gx, gy = current
        for dx, dy in (
            (-1, -1), (0, -1), (1, -1), (-1, 0),
            (1, 0), (-1, 1), (0, 1), (1, 1),
        ):
            neighbor = (gx + dx, gy + dy)
            nx, ny = neighbor
            if not (0 <= nx < grid.size_x and 0 <= ny < grid.size_y):
                continue
            next_escaped = escaped or neighbor not in escape.active_cells
            mask = effective if next_escaped else escape.escape_mask
            if not mask[ny, nx] or (
                not next_escaped and neighbor in escape.foreign_blocked
            ):
                continue
            if not escaped:
                current_world = grid.grid_to_world(*current)
                end_world = grid.grid_to_world(*neighbor)
                vector = (
                    end_world[0] - current_world[0],
                    end_world[1] - current_world[1],
                )
                if not all(
                    _edge_moves_away_from_region(grid, current_world, vector, sources)
                    for sources in escape.source_sets
                ):
                    continue
            if dx and dy:
                corners = ((gx + dx, gy), (gx, gy + dy))
                if any(
                    not _state_cell_traversable(
                        (cx, cy),
                        escaped=escaped,
                        formal=formal,
                        effective=effective,
                        active_cells=escape.active_cells,
                        foreign_blocked=escape.foreign_blocked,
                    )
                    for cx, cy in corners
                ):
                    continue
            state = (neighbor, next_escaped)
            if state not in visited:
                visited.add(state)
                queue.append(state)
    return frozenset(normal_cells)


def astar_plan(
    grid: FloorGrid,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    max_iterations: int | None = DEFAULT_MAX_ITERATIONS,
    *,
    constraints: PlanningConstraints | None = None,
) -> PlanResult:
    """在八邻域栅格上规划并返回带原因的 ``PlanResult``。

    - REACHED：到达目标，path 为完整路径。
    - UNREACHABLE：open_heap 耗尽，确认不可达；上层可黑名单目标。
    - BUDGET_EXHAUSTED：迭代达到上限，可能可达但未搜完；上层不应黑名单，
      应等待地图更新后重试，避免把可达目标误判不可达。
    对角移动要求两个相邻正交格都可通行，避免从障碍角点之间穿过。若请求目标
    本身不可通行，会在有限半径内寻找最近的安全替代格；起点因量化误差落在
    UNKNOWN 时仅在本次规划掩码中开放当前机身格，不写正式地图。
    """
    if max_iterations is not None and (
        isinstance(max_iterations, bool)
        or not isinstance(max_iterations, int)
        or max_iterations <= 0
    ):
        raise ValueError("A* 最大迭代次数必须为正整数或 None")
    start = grid.world_to_grid(start_xy[0], start_xy[1])
    goal = grid.world_to_grid(goal_xy[0], goal_xy[1])
    if start is None or goal is None:
        return PlanResult(status=UNREACHABLE)
    traversable = _traversable_mask(grid, constraints)
    formal = _traversable_mask(grid, None)
    escape = _resolve_start_escape(
        grid,
        start,
        constraints=constraints,
        formal=formal,
        effective=traversable,
    )
    if escape.failure_reason is not None:
        return PlanResult(
            status=UNREACHABLE,
            reason_code=escape.failure_reason,
            blocker=escape.blocker,
        )
    if not escape.required and not traversable[start[1], start[0]]:
        if int(grid.data[start[1], start[0]]) == UNKNOWN:
            # 量化后的当前机身格只在本次快照开放，禁止写正式地图。
            traversable[start[1], start[0]] = True
        else:
            return PlanResult(status=UNREACHABLE, reason_code="start_cell_blocked")
    if not traversable[goal[1], goal[0]]:
        nearest = _nearest_traversable(traversable, goal)
        if nearest is None:
            return PlanResult(status=UNREACHABLE, reason_code="goal_region_blocked")
        goal = nearest

    # 单个状态化 A* 同时处理普通路径和起点约束退出；所有出口共享同一预算。
    size_y, size_x = traversable.shape
    start_escaped = not escape.required
    start_state = (start, start_escaped)
    start_heuristic = _heuristic(start, goal)
    open_heap: list[tuple[float, float, tuple[tuple[int, int], bool]]] = []
    heapq.heappush(open_heap, (start_heuristic, start_heuristic, start_state))
    came_from: dict[
        tuple[tuple[int, int], bool], tuple[tuple[int, int], bool]
    ] = {}
    cost_so_far: dict[tuple[tuple[int, int], bool], float] = {start_state: 0.0}
    closed_set: set[tuple[tuple[int, int], bool]] = set()

    iterations = 0
    while open_heap:
        if max_iterations is not None and iterations >= max_iterations:
            return PlanResult(status=BUDGET_EXHAUSTED)
        iterations += 1
        _, _, current_state = heapq.heappop(open_heap)
        if current_state in closed_set:
            continue
        closed_set.add(current_state)
        current, escaped = current_state
        if current == goal and escaped:
            states = _reconstruct_state_path(came_from, current_state)
            cells = [state[0] for state in states]
            first_normal = next(
                (index for index, state in enumerate(states) if state[1]), 0
            )
            prefix = [grid.grid_to_world(*cell) for cell in cells[: first_normal + 1]]
            tail = simplify_path_line_of_sight(
                grid,
                [grid.grid_to_world(*cell) for cell in cells[first_normal:]],
                constraints=constraints,
                traversable=traversable,
            )
            path = prefix[:-1] + tail
            return PlanResult(path=path, status=REACHED, blocker=escape.blocker)

        gx, gy = current
        for dx, dy in ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)):
            nx, ny = gx + dx, gy + dy
            if not (0 <= nx < size_x and 0 <= ny < size_y):
                continue
            neighbor = (nx, ny)
            next_escaped = escaped or neighbor not in escape.active_cells
            mask = traversable if next_escaped else escape.escape_mask
            if not mask[ny, nx]:
                continue
            if not next_escaped and neighbor in escape.foreign_blocked:
                continue
            if not escaped:
                current_world = grid.grid_to_world(*current)
                neighbor_world = grid.grid_to_world(*neighbor)
                vector = (
                    neighbor_world[0] - current_world[0],
                    neighbor_world[1] - current_world[1],
                )
                if not all(
                    _edge_moves_away_from_region(grid, current_world, vector, sources)
                    for sources in escape.source_sets
                ):
                    continue
            if dx != 0 and dy != 0:
                corners = ((gx + dx, gy), (gx, gy + dy))
                if any(
                    not _state_cell_traversable(
                        (cx, cy),
                        escaped=escaped,
                        formal=formal,
                        effective=traversable,
                        active_cells=escape.active_cells,
                        foreign_blocked=escape.foreign_blocked,
                    )
                    for cx, cy in corners
                ):
                    continue
            neighbor_state = (neighbor, next_escaped)
            if neighbor_state in closed_set:
                continue
            step_cost = DIAGONAL_COST if dx != 0 and dy != 0 else 1.0
            new_cost = cost_so_far[current_state] + step_cost
            if neighbor_state not in cost_so_far or new_cost < cost_so_far[neighbor_state]:
                cost_so_far[neighbor_state] = new_cost
                heuristic = _heuristic(neighbor, goal)
                priority = new_cost + heuristic
                heapq.heappush(open_heap, (priority, heuristic, neighbor_state))
                came_from[neighbor_state] = current_state

    return PlanResult(
        status=UNREACHABLE,
        reason_code=(
            escape.unreachable_reason if escape.required else "open_set_exhausted"
        ),
        blocker=escape.blocker,
    )


def astar_path(
    grid: FloorGrid,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    max_iterations: int | None = DEFAULT_MAX_ITERATIONS,
    *,
    constraints: PlanningConstraints | None = None,
) -> list[tuple[float, float]]:
    """兼容包装：仅返回路径，空列表表示未到达目标（不可达或预算耗尽）。

    需要区分“不可达”与“预算耗尽”的调用方应改用 astar_plan()。
    """
    return astar_plan(
        grid,
        start_xy,
        goal_xy,
        max_iterations=max_iterations,
        constraints=constraints,
    ).path


def path_length(path: list[tuple[float, float]]) -> float:
    """计算 world 坐标折线路径的欧氏总长度。"""
    if len(path) < 2:
        return 0.0
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def simplify_path_line_of_sight(
    grid: FloorGrid,
    path: list[tuple[float, float]],
    *,
    constraints: PlanningConstraints | None = None,
    traversable=None,
) -> list[tuple[float, float]]:
    """在现有 A* 路径上贪心删除有安全视线的中间折点。

    该步骤只改变跟踪点数量，不改变可通行语义；每条跨越线段仍按半栅格间距
    采样，并复查对角切角条件。
    """
    if len(path) <= 2:
        return list(path)

    simplified = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        next_index = anchor + 1
        for candidate in range(len(path) - 1, anchor, -1):
            if _segment_is_traversable(
                grid,
                path[anchor],
                path[candidate],
                constraints=constraints,
                traversable=traversable,
            ):
                next_index = candidate
                break
        simplified.append(path[next_index])
        anchor = next_index
    return simplified


def segment_is_traversable(
    grid: FloorGrid,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    *,
    constraints: PlanningConstraints | None = None,
) -> bool:
    """公开的只读线段合同，与 A* 使用同一派生掩码。"""
    return _segment_is_traversable(
        grid, start_xy, end_xy, constraints=constraints
    )


def _segment_is_traversable(
    grid: FloorGrid,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    *,
    constraints: PlanningConstraints | None = None,
    traversable=None,
) -> bool:
    """检查线段覆盖的全部栅格，并保持 A* 的对角禁止切角语义。"""
    start_cell = grid.world_to_grid(start_xy[0], start_xy[1])
    end_cell = grid.world_to_grid(end_xy[0], end_xy[1])
    if start_cell is None or end_cell is None:
        return False
    mask = traversable if traversable is not None else _traversable_mask(grid, constraints)
    if not mask[start_cell[1], start_cell[0]] or not mask[end_cell[1], end_cell[0]]:
        return False

    distance = math.hypot(
        end_xy[0] - start_xy[0],
        end_xy[1] - start_xy[1],
    )
    step_count = max(
        1,
        int(math.ceil(distance / max(float(grid.resolution) * 0.5, 1e-6))),
    )
    previous = start_cell
    for index in range(1, step_count + 1):
        ratio = index / step_count
        cell = grid.world_to_grid(
            start_xy[0] + (end_xy[0] - start_xy[0]) * ratio,
            start_xy[1] + (end_xy[1] - start_xy[1]) * ratio,
        )
        if cell is None or not mask[cell[1], cell[0]]:
            return False
        dx = cell[0] - previous[0]
        dy = cell[1] - previous[1]
        if dx != 0 and dy != 0:
            step_x = 1 if dx > 0 else -1
            step_y = 1 if dy > 0 else -1
            if not mask[previous[1], previous[0] + step_x]:
                return False
            if not mask[previous[1] + step_y, previous[0]]:
                return False
        previous = cell
    return True


def _nearest_traversable(traversable, goal: tuple[int, int], max_radius: int = 8) -> tuple[int, int] | None:
    gx, gy = goal
    candidates = [
        (nx, ny)
        for ny in range(gy - max_radius, gy + max_radius + 1)
        for nx in range(gx - max_radius, gx + max_radius + 1)
        if 0 <= ny < traversable.shape[0]
        and 0 <= nx < traversable.shape[1]
        and traversable[ny, nx]
    ]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda cell: (
            (cell[0] - gx) ** 2 + (cell[1] - gy) ** 2,
            cell[1],
            cell[0],
        ),
    )


def _traversable_mask(
    grid: FloorGrid,
    constraints: PlanningConstraints | None,
):
    mask = ((grid.data == FREE) | (grid.data == VISITED)).copy()
    if constraints is not None:
        for gx, gy in constraints.blocked_cells:
            if 0 <= gx < grid.size_x and 0 <= gy < grid.size_y:
                mask[gy, gx] = False
    return mask


def _edge_moves_away_from_region(
    grid: FloorGrid,
    start_world: tuple[float, float],
    vector: tuple[float, float],
    source_cells: frozenset[tuple[int, int]],
) -> bool:
    half = 0.5 * float(grid.resolution)
    for cell in source_cells:
        cx, cy = grid.grid_to_world(*cell)
        corners = (
            (cx - half, cy - half), (cx - half, cy + half),
            (cx + half, cy - half), (cx + half, cy + half),
        )
        if min(
            (start_world[0] - qx) * vector[0]
            + (start_world[1] - qy) * vector[1]
            for qx, qy in corners
        ) < -1e-9:
            return False
    return True


def _state_cell_traversable(
    cell: tuple[int, int],
    *,
    escaped: bool,
    formal,
    effective,
    active_cells: frozenset[tuple[int, int]],
    foreign_blocked: frozenset[tuple[int, int]],
) -> bool:
    gx, gy = cell
    if escaped:
        return bool(effective[gy, gx])
    if cell in active_cells:
        return bool(formal[gy, gx]) and cell not in foreign_blocked
    return bool(effective[gy, gx])


def _reconstruct_state_path(
    came_from: dict[
        tuple[tuple[int, int], bool], tuple[tuple[int, int], bool]
    ],
    current: tuple[tuple[int, int], bool],
) -> list[tuple[tuple[int, int], bool]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _reconstruct_path(
    came_from: dict[tuple[int, int], tuple[int, int]],
    current: tuple[int, int],
) -> list[tuple[int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
