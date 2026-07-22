"""在单层占据栅格上执行 A* 路径规划与视线压缩。

规划器只接受 Mapper 判定为 FREE/VISITED 的格子，不对 UNKNOWN 做乐观穿越。
返回状态显式区分“开放集确实耗尽”和“本轮迭代预算用完”：前者允许探索器
累计失败并拉黑目标，后者只表示计算尚未完成，不能据此破坏可达性判断。
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

from danger_search.mapper import FREE, VISITED, FloorGrid

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

    @property
    def reached(self) -> bool:
        return self.status == REACHED

    @property
    def budget_exhausted(self) -> bool:
        return self.status == BUDGET_EXHAUSTED

    @property
    def unreachable(self) -> bool:
        return self.status == UNREACHABLE


def astar_plan(
    grid: FloorGrid,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    max_iterations: int | None = DEFAULT_MAX_ITERATIONS,
) -> PlanResult:
    """在八邻域栅格上规划并返回带原因的 ``PlanResult``。

    - REACHED：到达目标，path 为完整路径。
    - UNREACHABLE：open_heap 耗尽，确认不可达；上层可黑名单目标。
    - BUDGET_EXHAUSTED：迭代达到上限，可能可达但未搜完；上层不应黑名单，
      应等待地图更新后重试，避免把可达目标误判不可达。
    对角移动要求两个相邻正交格都可通行，避免从障碍角点之间穿过。若请求目标
    本身不可通行，会在有限半径内寻找最近的安全替代格；起点因量化误差落在
    UNKNOWN 时只尝试 ``mark_free``，该操作不会覆盖 OCC/BLACK。
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
    if not grid.is_traversable(*start):
        grid.mark_free(*start)
    if not grid.is_traversable(*start):
        # UNKNOWN 可由 mark_free 修正为当前机器人所在自由格；OCC/BLACK 不可
        # 覆盖，也不能作为搜索起点直接穿出阻塞区域。
        return PlanResult(status=UNREACHABLE)
    if not grid.is_traversable(*goal):
        nearest = _nearest_traversable(grid, goal)
        if nearest is None:
            return PlanResult(status=UNREACHABLE)
        goal = nearest

    # 同一次同步规划期间地图不会被其他回调修改。预先生成只读掩码可避免
    # 搜索内数十万次 Python 方法调用，同时保持 FREE/VISITED、UNKNOWN 禁行
    # 和对角禁止切角的既有语义。
    traversable = (grid.data == FREE) | (grid.data == VISITED)
    size_y, size_x = traversable.shape

    # f 相同的节点优先选择 h 更小者，保持 A* 的最优 f 顺序，同时避免开放区
    # 由坐标字典序扩展大量等价节点。
    start_heuristic = _heuristic(start, goal)
    open_heap: list[tuple[float, float, tuple[int, int]]] = []
    heapq.heappush(open_heap, (start_heuristic, start_heuristic, start))
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    cost_so_far: dict[tuple[int, int], float] = {start: 0.0}
    closed_set: set[tuple[int, int]] = set()

    iterations = 0
    while open_heap:
        if max_iterations is not None and iterations >= max_iterations:
            return PlanResult(status=BUDGET_EXHAUSTED)
        iterations += 1
        _, _, current = heapq.heappop(open_heap)
        if current in closed_set:
            continue

        closed_set.add(current)
        if current == goal:
            path = [
                grid.grid_to_world(gx, gy)
                for gx, gy in _reconstruct_path(came_from, current)
            ]
            path = simplify_path_line_of_sight(grid, path)
            return PlanResult(path=path, status=REACHED)

        gx, gy = current
        for dx, dy in ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)):
            nx, ny = gx + dx, gy + dy
            if not (0 <= nx < size_x and 0 <= ny < size_y and traversable[ny, nx]):
                continue
            # 对角线移动时，检查相邻两个正交格是否同时可通行，防止切角
            if dx != 0 and dy != 0:
                if not traversable[gy, gx + dx] or not traversable[gy + dy, gx]:
                    continue
            neighbor = (nx, ny)
            if neighbor in closed_set:
                continue
            step_cost = DIAGONAL_COST if dx != 0 and dy != 0 else 1.0
            new_cost = cost_so_far[current] + step_cost
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                heuristic = _heuristic(neighbor, goal)
                priority = new_cost + heuristic
                heapq.heappush(open_heap, (priority, heuristic, neighbor))
                came_from[neighbor] = current

    return PlanResult(status=UNREACHABLE)


def astar_path(
    grid: FloorGrid,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    max_iterations: int | None = DEFAULT_MAX_ITERATIONS,
) -> list[tuple[float, float]]:
    """兼容包装：仅返回路径，空列表表示未到达目标（不可达或预算耗尽）。

    需要区分“不可达”与“预算耗尽”的调用方应改用 astar_plan()。
    """
    return astar_plan(grid, start_xy, goal_xy, max_iterations=max_iterations).path


def path_length(path: list[tuple[float, float]]) -> float:
    """计算 world 坐标折线路径的欧氏总长度。"""
    if len(path) < 2:
        return 0.0
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def simplify_path_line_of_sight(
    grid: FloorGrid,
    path: list[tuple[float, float]],
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
            if _segment_is_traversable(grid, path[anchor], path[candidate]):
                next_index = candidate
                break
        simplified.append(path[next_index])
        anchor = next_index
    return simplified


def _segment_is_traversable(
    grid: FloorGrid,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
) -> bool:
    """检查线段覆盖的全部栅格，并保持 A* 的对角禁止切角语义。"""
    start_cell = grid.world_to_grid(start_xy[0], start_xy[1])
    end_cell = grid.world_to_grid(end_xy[0], end_xy[1])
    if start_cell is None or end_cell is None:
        return False
    if not grid.is_traversable(*start_cell) or not grid.is_traversable(*end_cell):
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
        if cell is None or not grid.is_traversable(*cell):
            return False
        dx = cell[0] - previous[0]
        dy = cell[1] - previous[1]
        if dx != 0 and dy != 0:
            step_x = 1 if dx > 0 else -1
            step_y = 1 if dy > 0 else -1
            if not grid.is_traversable(previous[0] + step_x, previous[1]):
                return False
            if not grid.is_traversable(previous[0], previous[1] + step_y):
                return False
        previous = cell
    return True


def _nearest_traversable(grid: FloorGrid, goal: tuple[int, int], max_radius: int = 8) -> tuple[int, int] | None:
    gx, gy = goal
    candidates = [
        (nx, ny)
        for ny in range(gy - max_radius, gy + max_radius + 1)
        for nx in range(gx - max_radius, gx + max_radius + 1)
        if grid.is_traversable(nx, ny)
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
