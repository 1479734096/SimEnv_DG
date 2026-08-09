from danger_search.exploration_runtime import (
    ExplorationBudgetInput,
    FrontierArrivalGainRuntime,
    decide_exploration_budget,
)


def _arrive(
    runtime,
    *,
    target,
    baseline,
    known,
    floor=0,
    map_epoch=1,
    expected_gain=100.0,
):
    runtime.begin(
        floor=floor,
        map_epoch=map_epoch,
        target=target,
        known_cells=baseline,
        expected_gain=expected_gain,
        is_real_frontier=True,
    )
    return runtime.arrive(
        floor=floor,
        map_epoch=map_epoch,
        known_cells=known,
    )


def test_a_b_a_low_gain_loop_cools_repeated_region():
    runtime = FrontierArrivalGainRuntime()

    first_a = _arrive(runtime, target=(1.0, 1.0), baseline=100, known=110)
    first_b = _arrive(runtime, target=(4.0, 1.0), baseline=110, known=115)
    second_a = _arrive(runtime, target=(1.0, 1.0), baseline=115, known=120)

    assert first_a.low_gain_count == 1
    assert first_b.low_gain_count == 1
    assert second_a.low_gain_count == 2
    assert second_a.cooldown_center == (1.0, 1.0)


def test_target_drift_within_one_meter_uses_same_region():
    runtime = FrontierArrivalGainRuntime()

    _arrive(runtime, target=(2.0, 3.0), baseline=100, known=105)
    result = _arrive(runtime, target=(2.1, 3.0), baseline=105, known=110)

    assert result.low_gain_count == 2
    assert result.cooldown_center == (2.1, 3.0)


def test_normal_gain_resets_region_count():
    runtime = FrontierArrivalGainRuntime()

    _arrive(runtime, target=(2.0, 3.0), baseline=100, known=105)
    normal = _arrive(runtime, target=(2.1, 3.0), baseline=105, known=140)
    after_reset = _arrive(runtime, target=(2.0, 3.0), baseline=140, known=145)

    assert normal.actual_known_gain == 35
    assert normal.low_gain_count == 0
    assert after_reset.low_gain_count == 1
    assert after_reset.cooldown_center is None


def test_fallback_and_context_change_do_not_leak_arrival_accounting():
    runtime = FrontierArrivalGainRuntime()
    runtime.begin(
        floor=0,
        map_epoch=1,
        target=(1.0, 1.0),
        known_cells=100,
        expected_gain=0.0,
        is_real_frontier=False,
    )
    assert runtime.arrive(floor=0, map_epoch=1, known_cells=101) is None

    _arrive(runtime, target=(1.0, 1.0), baseline=100, known=105)
    changed = _arrive(
        runtime,
        target=(1.0, 1.0),
        baseline=105,
        known=110,
        map_epoch=2,
    )
    assert changed.low_gain_count == 1
    assert changed.cooldown_center is None


def test_last_floor_timeout_low_progress_returns_even_with_many_frontiers():
    decision = decide_exploration_budget(
        ExplorationBudgetInput(
            home_recorded=True,
            mission_elapsed=300.0,
            floor_elapsed=280.0,
            max_runtime=620.0,
            return_time=30.0,
            return_safety_buffer=20.0,
            floor_timeout=280.0,
            floor_finish_time_ratio=0.75,
            known_cells_growth=0.0,
            exploration_progress_window=15.0,
            minimum_progress_cells=30,
            frontier_count=8,
            minimum_remaining_frontiers=2,
            current_floor=1,
            maximum_floor=2,
            next_floor_change_time=float("inf"),
            last_floor_uses_global_budget=True,
        )
    )

    assert decision.action == "RETURN_HOME"
    assert decision.reason_code == "floor_low_progress_return_home"


def test_last_floor_timeout_with_normal_gain_keeps_global_budget():
    decision = decide_exploration_budget(
        ExplorationBudgetInput(
            home_recorded=True,
            mission_elapsed=300.0,
            floor_elapsed=280.0,
            max_runtime=620.0,
            return_time=30.0,
            return_safety_buffer=20.0,
            floor_timeout=280.0,
            floor_finish_time_ratio=0.75,
            known_cells_growth=2.0,
            exploration_progress_window=15.0,
            minimum_progress_cells=30,
            frontier_count=8,
            minimum_remaining_frontiers=2,
            current_floor=1,
            maximum_floor=2,
            next_floor_change_time=float("inf"),
            last_floor_uses_global_budget=True,
        )
    )

    assert decision.action == "CONTINUE"
    assert decision.reason_code == "last_floor_uses_global_budget"
