"""GroundSupport 证据反馈的并发队列与任务层运行状态。

安全判定只属于 ``GroundSupportSafetyGate``。本模块不判定支撑是否安全，只把支撑
线程输出串行交给主线程，并保存空间断面约束与序列事务的有界运行状态。
"""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass, replace
from threading import Lock

from danger_search.ground_support_gate import (
    CLEAR,
    DROP_STOPPING,
    OBSERVATION_HOLD,
    SAFETY_BLOCKED,
    SAFETY_STOP_TIMEOUT,
    SPEED_OUT_OF_ENVELOPE_STOPPING,
    STALE_DATA_HOLD,
    SUPPORT_PATH_UNAVAILABLE,
    SupportFailureWitness,
)
from danger_search.planner import PlanningConstraints, SupportVetoRegion


ACTION_CLEAR = "clear"
ACTION_HOLD = "hold"
ACTION_NEGATIVE_OBSTACLE = "negative_obstacle"
ACTION_TERMINAL = "terminal"
ACTION_PATH_REPLAN = "path_replan"


@dataclass(frozen=True)
class SupportConstraintUpdate:
    revoked_region_ids: tuple[str, ...] = ()
    confirmed_batches: tuple[ConfirmedSupportBatch, ...] = ()

    @property
    def changed(self) -> bool:
        return bool(self.revoked_region_ids or self.confirmed_batches)


@dataclass(frozen=True)
class ConfirmedSupportBatch:
    floor: int
    map_epoch: int
    localization_epoch: int
    grid_origin: tuple[float, float]
    grid_resolution: float
    cloud_stamp: float
    cloud_generation: int
    cells: frozenset[tuple[int, int]]


@dataclass(frozen=True)
class GateStateTransition:
    old_state: str
    new_state: str

    @property
    def changed(self) -> bool:
        return self.old_state != self.new_state


@dataclass(frozen=True)
class SupportConstraintRegion:
    region_id: str
    floor: int
    map_epoch: int
    localization_epoch: int
    frame_id: str
    grid_origin: tuple[float, float]
    grid_resolution: float
    source_cells: frozenset[tuple[int, int]]
    blocked_cells: frozenset[tuple[int, int]]
    first_sequence: int
    last_sequence: int
    last_witness_generation: int
    failure_count: int
    last_context_key: tuple[object, ...] | None = None


@dataclass(frozen=True)
class ConstraintInstallResult:
    installed: bool
    reason_code: str
    region_id: str | None = None
    failure_count: int = 0
    revision: int = 0


def classify_gate_action(state: str, reason: str) -> str:
    """把 Gate 状态映射为任务层动作，不改变 Gate 的锁存语义。"""
    if state == CLEAR:
        return ACTION_CLEAR
    if state in {
        OBSERVATION_HOLD,
        STALE_DATA_HOLD,
        DROP_STOPPING,
        SPEED_OUT_OF_ENVELOPE_STOPPING,
    }:
        return ACTION_HOLD
    if state == SAFETY_BLOCKED and reason == "negative_obstacle":
        return ACTION_NEGATIVE_OBSTACLE
    if state in {SAFETY_BLOCKED, SAFETY_STOP_TIMEOUT} or (
        state == SUPPORT_PATH_UNAVAILABLE
        and reason in {"support_speed_out_of_envelope", "localization_speed_abnormal"}
    ):
        return ACTION_TERMINAL
    if state == SUPPORT_PATH_UNAVAILABLE:
        return ACTION_PATH_REPLAN
    return ACTION_HOLD


class GroundSupportFeedbackRuntime:
    """持有 Gate 之外的反馈协调状态；并发写入只发生在证据队列。"""

    def __init__(self) -> None:
        self._lock = Lock()
        self._pending_supported: list[ConfirmedSupportBatch] = []
        self._regions: dict[str, SupportConstraintRegion] = {}
        self._region_confirmations: dict[str, frozenset[tuple[int, int]]] = {}
        self._sequence_fingerprints: OrderedDict[
            tuple[int, int, int, int], tuple[tuple[object, ...], str]
        ] = OrderedDict()
        self._sequence_highwater: dict[tuple[int, int, int], int] = {}
        self._constraint_revision = 0
        self._next_region_sequence = 0
        self.last_gate_state = CLEAR

    def queue_confirmed_support(
        self,
        result: dict,
        *,
        default_floor: int,
        map_epoch: int | None = None,
        localization_epoch: int | None = None,
        grid_origin: tuple[float, float] | None = None,
        grid_resolution: float | None = None,
        cloud_stamp: float | None = None,
        cloud_generation: int | None = None,
    ) -> None:
        """从支撑线程登记有限格坐标；畸形负载不进入正式地图。"""
        supported = _grid_cells(result.get("confirmed_supported_cells"))
        if not supported:
            return
        try:
            floor = int(result.get("floor", default_floor))
            resolved_map_epoch = int(map_epoch)
            resolved_localization_epoch = int(localization_epoch)
            resolved_origin = (float(grid_origin[0]), float(grid_origin[1]))
            resolved_resolution = float(grid_resolution)
            resolved_stamp = float(cloud_stamp)
            resolved_generation = int(cloud_generation)
        except (TypeError, ValueError, OverflowError):
            return
        if (
            resolved_map_epoch <= 0
            or resolved_localization_epoch <= 0
            or resolved_generation <= 0
            or resolved_resolution <= 0.0
        ):
            return
        batch = ConfirmedSupportBatch(
            floor=floor,
            map_epoch=resolved_map_epoch,
            localization_epoch=resolved_localization_epoch,
            grid_origin=resolved_origin,
            grid_resolution=resolved_resolution,
            cloud_stamp=resolved_stamp,
            cloud_generation=resolved_generation,
            cells=frozenset(supported),
        )
        with self._lock:
            self._pending_supported.append(batch)

    def apply_confirmed_support(
        self,
        *,
        floor: int,
        map_epoch: int,
        localization_epoch: int,
        grid_origin: tuple[float, float],
        grid_resolution: float,
    ) -> SupportConstraintUpdate:
        """用晚于失败 witness 的逐格正支撑原子撤销临时约束。

        该反馈绝不修改正式 ``FloorGrid``。正式 NEGATIVE 的生命周期由 Mapper
        自己的来源账本负责，不能由临时 GroundSupport 反馈匿名清除。
        """
        with self._lock:
            pending = self._pending_supported
            self._pending_supported = []
            revoked: list[str] = []
            expected_origin = tuple(float(value) for value in grid_origin)
            expected_resolution = float(grid_resolution)
            for batch in pending:
                if (
                    batch.floor != int(floor)
                    or batch.map_epoch != int(map_epoch)
                    or batch.localization_epoch != int(localization_epoch)
                    or batch.grid_origin != expected_origin
                    or abs(batch.grid_resolution - expected_resolution) > 1e-9
                ):
                    continue
                for region_id, region in tuple(self._regions.items()):
                    if (
                        region.floor != batch.floor
                        or region.map_epoch != batch.map_epoch
                        or region.localization_epoch != batch.localization_epoch
                        or region.grid_origin != batch.grid_origin
                        or abs(region.grid_resolution - batch.grid_resolution) > 1e-9
                        or batch.cloud_generation <= region.last_witness_generation
                    ):
                        continue
                    matched = batch.cells & region.source_cells
                    if not matched:
                        continue
                    confirmed = self._region_confirmations.get(
                        region_id, frozenset()
                    ) | matched
                    if region.source_cells <= confirmed:
                        self._regions.pop(region_id, None)
                        self._region_confirmations.pop(region_id, None)
                        revoked.append(region_id)
                    else:
                        self._region_confirmations[region_id] = frozenset(confirmed)
            if revoked:
                self._constraint_revision += 1
            applied_batches = tuple(
                batch
                for batch in pending
                if batch.floor == int(floor)
                and batch.map_epoch == int(map_epoch)
                and batch.localization_epoch == int(localization_epoch)
                and batch.grid_origin == expected_origin
                and abs(batch.grid_resolution - expected_resolution) <= 1e-9
            )
            return SupportConstraintUpdate(
                tuple(sorted(revoked)),
                applied_batches,
            )

    def observe_gate_state(self, state: str) -> GateStateTransition:
        transition = GateStateTransition(self.last_gate_state, str(state))
        self.last_gate_state = str(state)
        return transition

    def synchronize_gate_state(self, state: str) -> None:
        self.last_gate_state = str(state)

    def install_constraint(
        self,
        witness: SupportFailureWitness,
        *,
        blocked_cells: set[tuple[int, int]] | frozenset[tuple[int, int]],
        context_key: tuple[object, ...] | None = None,
    ) -> ConstraintInstallResult:
        """原子安装或按相接空间断面归并；目标上下文只作诊断。"""
        if (
            witness.frame_id != "odom"
            or witness.sequence <= 0
            or witness.generation <= 0
            or not witness.source_cells
        ):
            return ConstraintInstallResult(False, "support_constraint_witness_invalid")
        source = frozenset((int(x), int(y)) for x, y in witness.source_cells)
        blocked = frozenset((int(x), int(y)) for x, y in blocked_cells)
        if not blocked:
            return ConstraintInstallResult(False, "support_constraint_geometry_missing")
        sequence_key = (
            int(witness.floor), int(witness.map_epoch),
            int(witness.localization_epoch), int(witness.sequence),
        )
        scope_key = sequence_key[:3]
        fingerprint = (
            witness.frame_id,
            round(float(witness.stamp), 9),
            int(witness.generation),
            int(witness.hold_context_id),
            tuple(round(float(value), 9) for value in witness.trigger_command),
            tuple(round(float(value), 9) for value in witness.grid_origin),
            round(float(witness.grid_resolution), 9),
            tuple(sorted(source)),
            tuple(sorted(blocked)),
        )
        with self._lock:
            installed = self._sequence_fingerprints.get(sequence_key)
            if installed is not None:
                installed_fingerprint, installed_id = installed
                self._sequence_fingerprints.move_to_end(sequence_key)
                region = self._regions.get(installed_id)
                if fingerprint != installed_fingerprint:
                    return ConstraintInstallResult(
                        False, "support_constraint_sequence_conflict",
                        revision=self._constraint_revision,
                    )
                if region is None:
                    return ConstraintInstallResult(
                        False, "support_constraint_stale_transition",
                        revision=self._constraint_revision,
                    )
                return ConstraintInstallResult(
                    True, "already_installed", region.region_id,
                    region.failure_count, self._constraint_revision,
                )
            if int(witness.sequence) <= self._sequence_highwater.get(scope_key, 0):
                return ConstraintInstallResult(
                    False, "support_constraint_stale_transition",
                    revision=self._constraint_revision,
                )
            candidates = [
                region for region in self._regions.values()
                if region.floor == int(witness.floor)
                and region.map_epoch == int(witness.map_epoch)
                and region.localization_epoch == int(witness.localization_epoch)
                and region.frame_id == witness.frame_id
                and region.grid_origin == tuple(witness.grid_origin)
                and abs(region.grid_resolution - witness.grid_resolution) <= 1e-9
                and _cells_touch(region.source_cells, source)
            ]
            if len(candidates) > 1:
                candidates.sort(key=lambda item: item.region_id)
            if candidates:
                region = candidates[0]
                updated = replace(
                    region,
                    source_cells=region.source_cells | source,
                    blocked_cells=region.blocked_cells | blocked,
                    last_sequence=int(witness.sequence),
                    last_witness_generation=max(
                        region.last_witness_generation,
                        int(witness.generation),
                    ),
                    failure_count=min(1_000_000, region.failure_count + 1),
                    last_context_key=context_key,
                )
                # 新 failure witness 把整个空间断面的确认代次推进到
                # last_witness_generation。此前累计的任何正支撑都早于这次失败，
                # 不能与后续局部确认拼接后撤销约束。
                self._region_confirmations[region.region_id] = frozenset()
            else:
                if len(self._regions) >= 64:
                    return ConstraintInstallResult(
                        False, "support_constraint_ledger_exhausted",
                        revision=self._constraint_revision,
                    )
                self._next_region_sequence += 1
                region_id = (
                    f"support:{witness.floor}:{witness.map_epoch}:"
                    f"{witness.localization_epoch}:{self._next_region_sequence}"
                )
                updated = SupportConstraintRegion(
                    region_id=region_id,
                    floor=int(witness.floor),
                    map_epoch=int(witness.map_epoch),
                    localization_epoch=int(witness.localization_epoch),
                    frame_id=witness.frame_id,
                    grid_origin=tuple(float(value) for value in witness.grid_origin),
                    grid_resolution=float(witness.grid_resolution),
                    source_cells=source,
                    blocked_cells=blocked,
                    first_sequence=int(witness.sequence),
                    last_sequence=int(witness.sequence),
                    last_witness_generation=int(witness.generation),
                    failure_count=1,
                    last_context_key=context_key,
                )
                self._region_confirmations[region_id] = frozenset()
            if len(updated.source_cells) > 256 or len(updated.blocked_cells) > 4096:
                return ConstraintInstallResult(
                    False, "support_constraint_ledger_exhausted",
                    revision=self._constraint_revision,
                )
            self._regions[updated.region_id] = updated
            self._sequence_fingerprints[sequence_key] = (
                fingerprint, updated.region_id,
            )
            self._sequence_highwater[scope_key] = int(witness.sequence)
            while len(self._sequence_fingerprints) > 512:
                self._sequence_fingerprints.popitem(last=False)
            self._constraint_revision += 1
            return ConstraintInstallResult(
                True, "installed", updated.region_id,
                updated.failure_count, self._constraint_revision,
            )

    def planning_constraints(
        self,
        *,
        floor: int,
        map_epoch: int,
        localization_epoch: int,
    ) -> PlanningConstraints | None:
        with self._lock:
            regions = tuple(
                region for region in self._regions.values()
                if region.floor == int(floor)
                and region.map_epoch == int(map_epoch)
                and region.localization_epoch == int(localization_epoch)
            )
            revision = self._constraint_revision
        if not regions:
            return None
        return PlanningConstraints.from_regions(
            tuple(
                SupportVetoRegion(
                    region.region_id,
                    region.blocked_cells,
                    region.source_cells,
                )
                for region in regions
            ),
            revision=revision,
        )

    def constraint_regions(
        self,
        *,
        floor: int,
        map_epoch: int,
        localization_epoch: int,
    ) -> tuple[SupportConstraintRegion, ...]:
        """返回当前 scope 的冻结断面快照，供诊断与编排决策读取。"""
        with self._lock:
            return tuple(
                sorted(
                    (
                        region
                        for region in self._regions.values()
                        if region.floor == int(floor)
                        and region.map_epoch == int(map_epoch)
                        and region.localization_epoch == int(localization_epoch)
                    ),
                    key=lambda region: region.region_id,
                )
            )

    def invalidate_scope(
        self,
        *,
        floor: int | None = None,
        map_epoch: int | None = None,
        localization_epoch: int | None = None,
    ) -> None:
        with self._lock:
            remove = [
                region_id for region_id, region in self._regions.items()
                if (floor is None or region.floor == int(floor))
                and (map_epoch is None or region.map_epoch != int(map_epoch))
                and (
                    localization_epoch is None
                    or region.localization_epoch != int(localization_epoch)
                )
            ]
            if not remove:
                return
            for region_id in remove:
                self._regions.pop(region_id, None)
                self._region_confirmations.pop(region_id, None)
            self._pending_supported = [
                batch for batch in self._pending_supported
                if not (
                    (floor is None or batch.floor == int(floor))
                    and (map_epoch is None or batch.map_epoch != int(map_epoch))
                    and (
                        localization_epoch is None
                        or batch.localization_epoch != int(localization_epoch)
                    )
                )
            ]
            self._constraint_revision += 1

    def discard_pending_batches(self) -> None:
        """scope 转换事务中丢弃尚未消费的旧证据。"""
        with self._lock:
            self._pending_supported = []

def _grid_cells(payload) -> set[tuple[int, int]]:
    cells: set[tuple[int, int]] = set()
    if not isinstance(payload, (list, tuple)):
        return cells
    for item in payload:
        if not isinstance(item, (list, tuple)) or len(item) < 2:
            continue
        try:
            gx = int(item[0])
            gy = int(item[1])
        except (TypeError, ValueError, OverflowError):
            continue
        cells.add((gx, gy))
    return cells


def _cells_touch(
    left: frozenset[tuple[int, int]],
    right: frozenset[tuple[int, int]],
) -> bool:
    return any(
        abs(ax - bx) <= 1 and abs(ay - by) <= 1
        for ax, ay in left for bx, by in right
    )
