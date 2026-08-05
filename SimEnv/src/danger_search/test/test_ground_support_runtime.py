from danger_search.ground_support_gate import (
    CLEAR,
    OBSERVATION_HOLD,
    SAFETY_BLOCKED,
    SAFETY_STOP_TIMEOUT,
    SUPPORT_PATH_UNAVAILABLE,
)
from danger_search.ground_support_runtime import (
    ACTION_CLEAR,
    ACTION_HOLD,
    ACTION_NEGATIVE_OBSTACLE,
    ACTION_PATH_REPLAN,
    ACTION_TERMINAL,
    GroundSupportFeedbackRuntime,
    classify_gate_action,
)


class _Mapper:
    def __init__(self):
        self.negative = []
        self.supported = []

    def mark_negative_cells(self, floor, cells):
        self.negative.append((floor, set(cells)))
        return len(cells)

    def clear_negative_cells_with_support(self, floor, cells):
        self.supported.append((floor, set(cells)))
        return len(cells)


def test_map_update_queue_deduplicates_and_applies_support_last():
    runtime = GroundSupportFeedbackRuntime()
    runtime.queue_map_updates(
        {
            "floor": 1,
            "confirmed_negative_cells": [[1, 2], [1, 2], [3, 4]],
            "confirmed_supported_cells": [[1, 2], ["bad"]],
        },
        default_floor=0,
    )
    mapper = _Mapper()

    result = runtime.apply_map_updates(mapper)

    assert result.negative_added == 2
    assert result.negative_cleared_by_support == 1
    assert mapper.negative == [(1, {(1, 2), (3, 4)})]
    assert mapper.supported == [(1, {(1, 2)})]
    assert not runtime.apply_map_updates(mapper).changed


def test_retry_identity_and_negative_marks_are_owned_by_runtime():
    runtime = GroundSupportFeedbackRuntime()
    assert runtime.record_path_retry(("frontier", 1.0, 2.0)) == 1
    assert runtime.record_path_retry(("frontier", 1.0, 2.0)) == 2
    assert runtime.record_path_retry(("task", 1, "PLAN")) == 1
    assert runtime.can_replan_negative_obstacle(1)
    assert runtime.record_negative_obstacle(floor=0, x=1.0, y=2.0, radius=0.4) == 1
    assert not runtime.can_replan_negative_obstacle(1)
    assert runtime.black_marks_for_floor(0) == ((0, 1.0, 2.0, 0.4),)
    assert runtime.black_marks_for_floor(1) == ()


def test_gate_state_classification_keeps_gate_as_single_safety_authority():
    assert classify_gate_action(CLEAR, "") == ACTION_CLEAR
    assert classify_gate_action(OBSERVATION_HOLD, "unknown") == ACTION_HOLD
    assert (
        classify_gate_action(SAFETY_BLOCKED, "negative_obstacle")
        == ACTION_NEGATIVE_OBSTACLE
    )
    assert classify_gate_action(SAFETY_STOP_TIMEOUT, "timeout") == ACTION_TERMINAL
    assert (
        classify_gate_action(
            SUPPORT_PATH_UNAVAILABLE,
            "support_speed_out_of_envelope",
        )
        == ACTION_TERMINAL
    )
    assert (
        classify_gate_action(
            SUPPORT_PATH_UNAVAILABLE,
            "ground_support_path_unavailable",
        )
        == ACTION_PATH_REPLAN
    )


def test_gate_transition_mirror_has_no_independent_transition_rules():
    runtime = GroundSupportFeedbackRuntime()
    first = runtime.observe_gate_state(OBSERVATION_HOLD)
    repeated = runtime.observe_gate_state(OBSERVATION_HOLD)
    reset = runtime.observe_gate_state(CLEAR)
    assert first.changed and first.old_state == CLEAR
    assert not repeated.changed
    assert reset.changed and reset.new_state == CLEAR
