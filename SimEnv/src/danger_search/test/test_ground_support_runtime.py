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
    SupportFailureWitness,
    classify_gate_action,
)


def test_map_update_queue_never_writes_anonymous_negative_cells():
    runtime = GroundSupportFeedbackRuntime()
    runtime.queue_confirmed_support(
        {
            "floor": 1,
            "confirmed_negative_cells": [[1, 2], [1, 2], [3, 4]],
            "confirmed_supported_cells": [[1, 2], ["bad"]],
        },
        default_floor=0,
        map_epoch=1,
        localization_epoch=1,
        grid_origin=(-14.0, -6.0),
        grid_resolution=0.1,
        cloud_stamp=1.0,
        cloud_generation=1,
    )
    apply_kwargs = dict(
        floor=0,
        map_epoch=1,
        localization_epoch=1,
        grid_origin=(-14.0, -6.0),
        grid_resolution=0.1,
    )
    result = runtime.apply_confirmed_support(**apply_kwargs)

    assert not result.changed
    assert not runtime.apply_confirmed_support(**apply_kwargs).revoked_region_ids


def test_support_constraint_identity_is_spatial_not_target_context():
    runtime = GroundSupportFeedbackRuntime()
    first = SupportFailureWitness(
        sequence=1,
        floor=0,
        map_epoch=1,
        localization_epoch=1,
        stamp=1.0,
        generation=1,
        source_cells=((10, 10),),
        hold_context_id=1,
        trigger_command=(0.2, 0.0, 0.0),
        grid_origin=(-14.0, -6.0),
        grid_resolution=0.1,
    )
    shifted = SupportFailureWitness(
        sequence=2,
        floor=0,
        map_epoch=1,
        localization_epoch=1,
        stamp=2.0,
        generation=2,
        source_cells=((11, 10),),
        hold_context_id=2,
        trigger_command=(0.2, 0.0, 0.0),
        grid_origin=(-14.0, -6.0),
        grid_resolution=0.1,
    )
    first_result = runtime.install_constraint(
        first,
        blocked_cells={(9, 9), (10, 10)},
        context_key=("frontier", 1.0, 2.0),
    )
    shifted_result = runtime.install_constraint(
        shifted,
        blocked_cells={(10, 10), (11, 10)},
        context_key=("frontier", 8.0, 9.0),
    )
    assert first_result.installed and shifted_result.installed
    assert shifted_result.region_id == first_result.region_id
    assert shifted_result.failure_count == 2


def test_constraint_sequence_fingerprint_is_idempotent_conflicting_and_stale():
    runtime = GroundSupportFeedbackRuntime()
    witness = SupportFailureWitness(
        sequence=4, floor=0, map_epoch=1, localization_epoch=1,
        stamp=4.0, generation=4, source_cells=((10, 10),),
        hold_context_id=4, trigger_command=(0.2, 0.0, 0.0),
        grid_origin=(-14.0, -6.0), grid_resolution=0.1,
    )
    installed = runtime.install_constraint(witness, blocked_cells={(10, 10)})
    replay = runtime.install_constraint(witness, blocked_cells={(10, 10)})
    conflict = runtime.install_constraint(witness, blocked_cells={(10, 10), (11, 10)})
    assert installed.installed and replay.reason_code == "already_installed"
    assert conflict.reason_code == "support_constraint_sequence_conflict"

    runtime.invalidate_scope(map_epoch=2)
    stale = runtime.install_constraint(witness, blocked_cells={(10, 10)})
    assert not stale.installed
    assert stale.reason_code == "support_constraint_stale_transition"


def test_confirmed_support_batch_cannot_cross_epoch_or_geometry():
    runtime = GroundSupportFeedbackRuntime()
    runtime.queue_confirmed_support(
        {"floor": 0, "confirmed_supported_cells": [[2, 3]]},
        default_floor=0,
        map_epoch=1,
        localization_epoch=1,
        grid_origin=(0.0, 0.0),
        grid_resolution=0.1,
        cloud_stamp=1.0,
        cloud_generation=1,
    )
    result = runtime.apply_confirmed_support(
        floor=0,
        map_epoch=2,
        localization_epoch=1,
        grid_origin=(0.0, 0.0),
        grid_resolution=0.1,
    )
    assert not result.changed


def test_region_is_revoked_only_after_all_cells_are_reconfirmed_by_newer_frames():
    runtime = GroundSupportFeedbackRuntime()
    witness = SupportFailureWitness(
        sequence=1, floor=0, map_epoch=1, localization_epoch=1,
        stamp=1.0, generation=10, source_cells=((2, 3), (3, 3)),
        hold_context_id=1, trigger_command=(0.2, 0.0, 0.0),
        grid_origin=(0.0, 0.0), grid_resolution=0.1,
    )
    installed = runtime.install_constraint(
        witness, blocked_cells={(2, 3), (3, 3)}
    )
    assert installed.installed
    apply_kwargs = dict(
        floor=0, map_epoch=1, localization_epoch=1,
        grid_origin=(0.0, 0.0), grid_resolution=0.1,
    )
    runtime.queue_confirmed_support(
        {"floor": 0, "confirmed_supported_cells": [[2, 3]]},
        default_floor=0, map_epoch=1, localization_epoch=1,
        grid_origin=(0.0, 0.0), grid_resolution=0.1,
        cloud_stamp=2.0, cloud_generation=11,
    )
    assert not runtime.apply_confirmed_support(**apply_kwargs).revoked_region_ids
    assert runtime.planning_constraints(
        floor=0, map_epoch=1, localization_epoch=1
    ) is not None

    runtime.queue_confirmed_support(
        {"floor": 0, "confirmed_supported_cells": [[3, 3]]},
        default_floor=0, map_epoch=1, localization_epoch=1,
        grid_origin=(0.0, 0.0), grid_resolution=0.1,
        cloud_stamp=3.0, cloud_generation=12,
    )
    update = runtime.apply_confirmed_support(**apply_kwargs)
    assert update.revoked_region_ids == (installed.region_id,)
    assert runtime.planning_constraints(
        floor=0, map_epoch=1, localization_epoch=1
    ) is None


def test_support_not_newer_than_failure_cannot_revoke_region():
    runtime = GroundSupportFeedbackRuntime()
    witness = SupportFailureWitness(
        sequence=1, floor=0, map_epoch=1, localization_epoch=1,
        stamp=2.0, generation=5, source_cells=((2, 3),),
        hold_context_id=1, trigger_command=(0.2, 0.0, 0.0),
        grid_origin=(0.0, 0.0), grid_resolution=0.1,
    )
    runtime.install_constraint(witness, blocked_cells={(2, 3)})
    runtime.queue_confirmed_support(
        {"floor": 0, "confirmed_supported_cells": [[2, 3]]},
        default_floor=0, map_epoch=1, localization_epoch=1,
        grid_origin=(0.0, 0.0), grid_resolution=0.1,
        cloud_stamp=2.0, cloud_generation=5,
    )
    update = runtime.apply_confirmed_support(
        floor=0, map_epoch=1, localization_epoch=1,
        grid_origin=(0.0, 0.0), grid_resolution=0.1,
    )
    assert not update.revoked_region_ids
    assert runtime.planning_constraints(
        floor=0, map_epoch=1, localization_epoch=1
    ) is not None


def test_new_failure_resets_all_partial_confirmations_in_merged_region():
    runtime = GroundSupportFeedbackRuntime()
    first = SupportFailureWitness(
        sequence=1, floor=0, map_epoch=1, localization_epoch=1,
        stamp=1.0, generation=10, source_cells=((2, 3), (3, 3)),
        hold_context_id=1, trigger_command=(0.2, 0.0, 0.0),
        grid_origin=(0.0, 0.0), grid_resolution=0.1,
    )
    installed = runtime.install_constraint(
        first, blocked_cells={(2, 3), (3, 3)}
    )
    apply_kwargs = dict(
        floor=0, map_epoch=1, localization_epoch=1,
        grid_origin=(0.0, 0.0), grid_resolution=0.1,
    )
    runtime.queue_confirmed_support(
        {"floor": 0, "confirmed_supported_cells": [[2, 3]]},
        default_floor=0, map_epoch=1, localization_epoch=1,
        grid_origin=(0.0, 0.0), grid_resolution=0.1,
        cloud_stamp=2.0, cloud_generation=11,
    )
    assert not runtime.apply_confirmed_support(**apply_kwargs).revoked_region_ids

    repeated_failure = SupportFailureWitness(
        sequence=2, floor=0, map_epoch=1, localization_epoch=1,
        stamp=3.0, generation=12, source_cells=((3, 3),),
        hold_context_id=2, trigger_command=(0.2, 0.0, 0.0),
        grid_origin=(0.0, 0.0), grid_resolution=0.1,
    )
    merged = runtime.install_constraint(
        repeated_failure, blocked_cells={(2, 3), (3, 3)}
    )
    assert merged.region_id == installed.region_id

    runtime.queue_confirmed_support(
        {"floor": 0, "confirmed_supported_cells": [[3, 3]]},
        default_floor=0, map_epoch=1, localization_epoch=1,
        grid_origin=(0.0, 0.0), grid_resolution=0.1,
        cloud_stamp=4.0, cloud_generation=13,
    )
    assert not runtime.apply_confirmed_support(**apply_kwargs).revoked_region_ids
    runtime.queue_confirmed_support(
        {"floor": 0, "confirmed_supported_cells": [[2, 3]]},
        default_floor=0, map_epoch=1, localization_epoch=1,
        grid_origin=(0.0, 0.0), grid_resolution=0.1,
        cloud_stamp=5.0, cloud_generation=14,
    )
    update = runtime.apply_confirmed_support(**apply_kwargs)
    assert update.revoked_region_ids == (installed.region_id,)


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
