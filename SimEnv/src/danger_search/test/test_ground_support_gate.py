"""生产负障碍安全门的纯逻辑测试。"""
from __future__ import annotations

import math
import unittest

from danger_search.ground_support_gate import (
    CLEAR,
    DROP_STOPPING,
    OBSERVATION_HOLD,
    SAFETY_BLOCKED,
    SAFETY_STOP_TIMEOUT,
    SPEED_OUT_OF_ENVELOPE_STOPPING,
    STALE_DATA_HOLD,
    SUPPORT_PATH_UNAVAILABLE,
    ConservativeStopEnvelope,
    GroundSupportAssessment,
    GroundSupportGateConfig,
    GroundSupportSafetyGate,
)


def _config(**overrides) -> GroundSupportGateConfig:
    values = {
        "enabled": True,
        "validated_max_speed": 0.60,
        "stop_distance_table": ((0.30, 0.06), (0.60, 0.09337727739487302)),
        "drop_evidence_max_age": 0.05,
        "decision_period": 0.10,
        "rl_receive_delay": 0.009,
        "required_margin": 0.02,
        "release_hysteresis": 0.10,
        "release_supported_frames": 2,
        "observation_hold_timeout": 2.0,
        "safety_stop_timeout": 3.0,
        "wall_stall_timeout": 30.0,
        "odometry_linear_deadband": 0.05,
        "odometry_angular_deadband": 0.03,
        "stop_linear_threshold": 0.05,
        "stop_angular_threshold": 0.05,
        "stop_confirmation_samples": 3,
        "localization_speed_anomaly_threshold": 1.0,
        "localization_speed_anomaly_timeout": 2.0,
    }
    values.update(overrides)
    return GroundSupportGateConfig(**values)


def _assessment(**overrides) -> GroundSupportAssessment:
    values = {
        "stamp": 10.0,
        "received_wall_time": 100.0,
        "data_age": 0.02,
        "supported_distance": 0.80,
        "support_boundary_detected": True,
        "current_footprint_supported": True,
        "observation_adequate": True,
        "drop_confirmed": False,
        "drop_intersects_sweep": False,
        "drop_distance": None,
        "drop_direction_world": None,
        "received_control_time": 100.0,
    }
    values.update(overrides)
    if "received_control_time" not in overrides:
        values["received_control_time"] = values["received_wall_time"]
    return GroundSupportAssessment(**values)


class ConservativeStopEnvelopeTest(unittest.TestCase):
    def test_validated_speed_is_derived_from_last_configured_breakpoint(self):
        envelope = ConservativeStopEnvelope(
            ((0.60, 0.0934), (0.80, 0.13)),
            validated_max_speed=None,
        )

        self.assertEqual(envelope.validated_max_speed, 0.80)
        self.assertEqual(envelope.distance_for(0.79), 0.13)
        self.assertIsNone(envelope.distance_for(0.800001))

    def test_uses_ceiling_table_without_interpolation(self):
        envelope = ConservativeStopEnvelope(
            ((0.30, 0.06), (0.60, 0.09337727739487302)),
            validated_max_speed=0.60,
        )

        self.assertEqual(envelope.distance_for(0.0), 0.0)
        self.assertAlmostEqual(envelope.distance_for(0.20), 0.06)
        self.assertAlmostEqual(envelope.distance_for(0.31), 0.09337727739487302)
        self.assertAlmostEqual(envelope.distance_for(0.60), 0.09337727739487302)

    def test_never_extrapolates_above_validated_speed(self):
        envelope = ConservativeStopEnvelope(
            ((0.60, 0.09337727739487302),),
            validated_max_speed=0.60,
        )

        self.assertIsNone(envelope.distance_for(0.600001))
        self.assertIsNone(envelope.distance_for(1.0))

    def test_high_speed_breakpoint_uses_measured_conservative_upper_bound(self):
        envelope = ConservativeStopEnvelope(
            (
                (0.60, 0.09337727739487302),
                (0.651, 0.10195826367975175),
                (0.80, 0.13),
            ),
            validated_max_speed=0.80,
        )

        self.assertAlmostEqual(
            envelope.distance_for(0.6507615808719522),
            0.10195826367975175,
        )
        self.assertAlmostEqual(envelope.distance_for(0.794421670252759), 0.13)
        self.assertIsNone(envelope.distance_for(0.800001))

    def test_rejects_table_that_does_not_cover_validated_speed(self):
        with self.assertRaises(ValueError):
            ConservativeStopEnvelope(((0.50, 0.08),), validated_max_speed=0.60)


class GroundSupportSafetyGateTest(unittest.TestCase):
    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            GroundSupportSafetyGate(_config(decision_period=math.nan))
        with self.assertRaises(ValueError):
            GroundSupportSafetyGate(_config(stop_confirmation_samples=0))
        with self.assertRaises(ValueError):
            GroundSupportSafetyGate(_config(stop_linear_threshold=0.02))

    def test_non_finite_command_is_fail_closed_even_when_disabled(self):
        gate = GroundSupportSafetyGate(_config(enabled=False))

        self.assertEqual(
            gate.filter_command(
                (0.2, 0.0, math.nan),
                pose_yaw=0.0,
                wall_time=100.0,
            ),
            (0.0, 0.0, 0.0),
        )

    def test_invalid_control_context_is_fail_closed(self):
        gate = GroundSupportSafetyGate(_config())

        self.assertEqual(
            gate.filter_command(
                (0.2, 0.0, 0.0),
                pose_yaw=math.nan,
                wall_time=100.0,
            ),
            (0.0, 0.0, 0.0),
        )

    def test_non_finite_odom_stamp_does_not_poison_freshness(self):
        gate = GroundSupportSafetyGate(_config())

        gate.observe_odometry(
            math.nan,
            linear_speed=0.1,
            angular_speed=0.0,
            wall_time=100.0,
        )
        gate.observe_odometry(
            1.0,
            linear_speed=0.1,
            angular_speed=0.0,
            wall_time=100.1,
        )

        self.assertEqual(gate._last_odom_stamp, 1.0)

    def test_missing_assessment_is_fail_closed(self):
        gate = GroundSupportSafetyGate(_config())

        command = gate.filter_command(
            (0.2, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
        )

        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, STALE_DATA_HOLD)
        self.assertEqual(gate.reason_code, "ground_support_missing")

    def test_non_finite_assessment_invalidates_previous_evidence(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(_assessment())
        gate.update_assessment(_assessment(stamp=math.nan))

        command = gate.filter_command(
            (0.2, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
        )

        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, STALE_DATA_HOLD)

    def test_disabled_gate_is_transparent(self):
        gate = GroundSupportSafetyGate(_config(enabled=False))
        gate.update_assessment(_assessment(drop_confirmed=True, drop_intersects_sweep=True))

        command = gate.filter_command((0.42, -0.08, 0.30), pose_yaw=0.0, wall_time=100.0)

        self.assertEqual(command, (0.42, -0.08, 0.30))
        self.assertEqual(gate.state, CLEAR)

    def test_verified_horizon_end_is_not_treated_as_unknown_boundary(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                supported_distance=0.01,
                support_boundary_detected=False,
            )
        )

        command = gate.filter_command(
            (0.30, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
        )

        self.assertEqual(command, (0.30, 0.0, 0.0))
        self.assertEqual(gate.state, CLEAR)

    def test_controlled_threshold_crossing_does_not_hold_for_unknown_support(self):
        gate = GroundSupportSafetyGate(_config())
        gate.set_unknown_support_hold_enabled(False)
        gate.update_assessment(
            _assessment(
                supported_distance=0.05,
                support_boundary_detected=True,
                observation_adequate=False,
            )
        )

        command = gate.filter_command(
            (0.30, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
        )

        self.assertEqual(command, (0.30, 0.0, 0.0))
        self.assertEqual(gate.state, CLEAR)

    def test_controlled_threshold_crossing_still_stops_explicit_drop(self):
        gate = GroundSupportSafetyGate(_config())
        gate.set_unknown_support_hold_enabled(False)
        gate.update_assessment(
            _assessment(
                observation_adequate=True,
                drop_confirmed=True,
                drop_intersects_sweep=True,
                drop_distance=0.30,
                drop_direction_world=(1.0, 0.0),
            )
        )

        command = gate.filter_command(
            (0.30, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
        )

        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, DROP_STOPPING)
        self.assertEqual(gate.reason_code, "negative_obstacle")

    def test_speed_above_envelope_stops_without_extrapolating(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(_assessment())

        command = gate.filter_command((0.61, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)

        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, SPEED_OUT_OF_ENVELOPE_STOPPING)
        self.assertEqual(gate.reason_code, "support_speed_out_of_envelope")
        self.assertIsNone(gate.last_stop_distance)

    def test_single_actual_speed_spike_stops_then_resumes_on_fresh_odom(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(_assessment())
        gate.observe_odometry(1.0, linear_speed=1.01, angular_speed=0.0, wall_time=99.9)

        command = gate.filter_command((0.30, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)

        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, SPEED_OUT_OF_ENVELOPE_STOPPING)
        self.assertFalse(gate.snapshot()["speed_out_of_envelope_confirmed"])

        gate.observe_odometry(1.1, linear_speed=0.40, angular_speed=0.0, wall_time=100.1)

        self.assertEqual(gate.state, CLEAR)
        self.assertEqual(gate.automatic_resume_count, 1)
        self.assertEqual(
            gate.snapshot()["speed_out_of_envelope_trigger_source"],
            "localization_odometry",
        )

    def test_repeated_localization_speed_anomaly_recovers_without_terminal_claim(self):
        gate = GroundSupportSafetyGate(_config(stop_confirmation_samples=1))
        gate.update_assessment(_assessment())
        gate.observe_odometry(1.0, linear_speed=1.01, angular_speed=0.0, wall_time=99.9)
        gate.filter_command((0.30, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)

        gate.observe_odometry(1.1, linear_speed=1.02, angular_speed=0.0, wall_time=100.1)
        self.assertFalse(gate.snapshot()["speed_out_of_envelope_confirmed"])
        gate.observe_odometry(1.2, linear_speed=0.01, angular_speed=0.0, wall_time=100.2)

        self.assertEqual(gate.state, CLEAR)
        self.assertEqual(gate.automatic_resume_count, 1)
        self.assertFalse(gate.terminal)

    def test_persistent_localization_speed_anomaly_fails_after_two_seconds(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(_assessment())
        gate.observe_odometry(
            1.0,
            linear_speed=1.01,
            angular_speed=0.0,
            wall_time=100.0,
        )
        gate.filter_command(
            (0.30, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
            control_time=1.0,
        )

        gate.observe_odometry(
            2.9,
            linear_speed=1.02,
            angular_speed=0.0,
            wall_time=101.9,
        )
        self.assertEqual(gate.state, SPEED_OUT_OF_ENVELOPE_STOPPING)
        gate.observe_odometry(
            3.0,
            linear_speed=1.03,
            angular_speed=0.0,
            wall_time=102.0,
        )

        self.assertEqual(gate.state, SUPPORT_PATH_UNAVAILABLE)
        self.assertEqual(gate.reason_code, "localization_speed_abnormal")
        self.assertTrue(gate.terminal)
        self.assertTrue(gate.snapshot()["speed_out_of_envelope_confirmed"])

    def test_requested_speed_above_envelope_still_becomes_terminal_after_stop(self):
        gate = GroundSupportSafetyGate(_config(stop_confirmation_samples=1))
        gate.update_assessment(_assessment())

        gate.filter_command((0.61, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)
        gate.observe_odometry(
            1.0,
            linear_speed=0.01,
            angular_speed=0.0,
            wall_time=100.1,
        )

        self.assertEqual(gate.state, SUPPORT_PATH_UNAVAILABLE)
        self.assertEqual(gate.reason_code, "support_speed_out_of_envelope")
        self.assertTrue(gate.terminal)

    def test_stop_distance_is_indexed_only_by_requested_command(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(_assessment(supported_distance=0.80))
        gate.observe_odometry(1.0, linear_speed=0.59, angular_speed=0.0, wall_time=99.9)

        command = gate.filter_command(
            (0.30, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
        )

        self.assertEqual(command, (0.30, 0.0, 0.0))
        self.assertAlmostEqual(gate.last_stop_distance, 0.06)
        self.assertEqual(
            gate.snapshot()["stop_envelope_index_source"],
            "requested_command",
        )

    def test_repeated_filter_calls_do_not_confirm_single_odom_spike(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(_assessment())
        gate.observe_odometry(1.0, linear_speed=1.01, angular_speed=0.0, wall_time=99.9)

        gate.filter_command((0.30, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)
        gate.filter_command((0.30, 0.0, 0.0), pose_yaw=0.0, wall_time=100.01)

        self.assertEqual(gate.state, SPEED_OUT_OF_ENVELOPE_STOPPING)
        self.assertEqual(gate.snapshot()["speed_out_of_envelope_sample_count"], 1)
        self.assertFalse(gate.snapshot()["speed_out_of_envelope_confirmed"])

    def test_temporary_hold_does_not_hide_speed_out_of_envelope(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                observation_adequate=False,
                supported_distance=0.10,
            )
        )
        gate.filter_command((0.30, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)
        self.assertEqual(gate.state, OBSERVATION_HOLD)

        command = gate.filter_command(
            (0.61, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.1,
        )

        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, SPEED_OUT_OF_ENVELOPE_STOPPING)
        self.assertEqual(gate.reason_code, "support_speed_out_of_envelope")

    def test_fresh_drop_only_stops_translation_toward_drop(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                drop_confirmed=True,
                drop_intersects_sweep=True,
                drop_distance=0.45,
                drop_direction_world=(1.0, 0.0),
            )
        )

        away = gate.filter_command((-0.20, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)
        toward = gate.filter_command((0.20, 0.0, 0.0), pose_yaw=0.0, wall_time=100.01)

        self.assertEqual(away, (-0.20, 0.0, 0.0))
        self.assertEqual(toward, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, DROP_STOPPING)
        self.assertEqual(gate.reason_code, "negative_obstacle")

    def test_fresh_cloud_can_latch_drop_before_next_control_tick(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                drop_confirmed=True,
                drop_intersects_sweep=True,
                drop_distance=0.45,
                drop_direction_world=(1.0, 0.0),
            )
        )

        latched = gate.evaluate_latest_drop(
            (0.30, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
        )

        self.assertTrue(latched)
        self.assertEqual(gate.state, DROP_STOPPING)
        self.assertEqual(gate.reason_code, "negative_obstacle")

    def test_old_drop_is_not_accepted_as_drop(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                data_age=0.25,
                supported_distance=0.10,
                drop_confirmed=True,
                drop_intersects_sweep=True,
                drop_distance=0.30,
                drop_direction_world=(1.0, 0.0),
            )
        )

        command = gate.filter_command((0.20, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)

        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, STALE_DATA_HOLD)
        self.assertEqual(gate.reason_code, "ground_support_stale")

    def test_assessment_wait_within_decision_period_is_not_double_counted_as_stale(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                data_age=0.01,
                received_wall_time=100.0,
                supported_distance=0.10,
            )
        )

        command = gate.filter_command((0.20, 0.0, 0.0), pose_yaw=0.0, wall_time=100.06)

        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, OBSERVATION_HOLD)

        gate.filter_command((0.0, 0.0, 0.0), pose_yaw=0.0, wall_time=100.11)

        self.assertEqual(gate.state, STALE_DATA_HOLD)

    def test_drop_assessment_remains_usable_within_decision_period(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                data_age=0.02,
                drop_confirmed=True,
                drop_intersects_sweep=True,
                drop_direction_world=(1.0, 0.0),
            )
        )

        latched = gate.evaluate_latest_drop(
            (0.30, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.08,
        )

        self.assertTrue(latched)
        self.assertEqual(gate.state, DROP_STOPPING)

    def test_sensor_freshness_uses_control_time_not_slow_wall_time(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                data_age=0.02,
                received_wall_time=100.0,
                received_control_time=10.0,
                supported_distance=0.10,
            )
        )

        gate.filter_command(
            (0.20, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.5,
            control_time=10.05,
        )

        self.assertEqual(gate.state, OBSERVATION_HOLD)

    def test_unknown_far_inside_supported_corridor_does_not_hold(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                observation_adequate=False,
                supported_distance=0.80,
            )
        )

        command = gate.filter_command((0.30, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)

        self.assertEqual(command, (0.30, 0.0, 0.0))
        self.assertEqual(gate.state, CLEAR)

    def test_unknown_without_confirmed_current_support_does_not_hold(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                data_age=0.20,
                supported_distance=0.0,
                current_footprint_supported=False,
                observation_adequate=False,
            )
        )

        command = gate.filter_command((0.30, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)

        self.assertEqual(command, (0.30, 0.0, 0.0))
        self.assertEqual(gate.state, CLEAR)

    def test_unknown_near_supported_boundary_holds_then_recovers(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                observation_adequate=False,
                supported_distance=0.10,
            )
        )
        held = gate.filter_command((0.30, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)

        gate.update_assessment(
            _assessment(
                stamp=10.1,
                received_wall_time=100.1,
                supported_distance=0.60,
            )
        )
        still_held = gate.filter_command((0.30, 0.0, 0.0), pose_yaw=0.0, wall_time=100.1)
        gate.update_assessment(
            _assessment(
                stamp=10.2,
                received_wall_time=100.2,
                supported_distance=0.60,
            )
        )
        released = gate.filter_command((0.30, 0.0, 0.0), pose_yaw=0.0, wall_time=100.2)

        self.assertEqual(held, (0.0, 0.0, 0.0))
        self.assertEqual(still_held, (0.0, 0.0, 0.0))
        self.assertEqual(released, (0.30, 0.0, 0.0))
        self.assertEqual(gate.state, CLEAR)
        self.assertEqual(gate.snapshot()["automatic_resume_count"], 1)

    def test_fresh_but_inconclusive_unknown_near_boundary_still_holds(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                observation_adequate=True,
                supported_distance=0.10,
                current_footprint_supported=True,
            )
        )

        command = gate.filter_command((0.30, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)

        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, OBSERVATION_HOLD)

    def test_hold_uses_original_motion_intent_when_drop_is_confirmed(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                observation_adequate=False,
                supported_distance=0.10,
            )
        )
        gate.filter_command((0.30, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)
        self.assertEqual(gate.state, OBSERVATION_HOLD)

        gate.update_assessment(
            _assessment(
                stamp=10.1,
                received_wall_time=100.1,
                supported_distance=0.10,
                drop_confirmed=True,
                drop_intersects_sweep=True,
                drop_distance=0.35,
                drop_direction_world=(1.0, 0.0),
            )
        )
        command = gate.filter_command((0.0, 0.0, 0.0), pose_yaw=0.0, wall_time=100.1)

        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, DROP_STOPPING)
        self.assertEqual(gate.reason_code, "negative_obstacle")

    def test_hold_release_distance_uses_original_motion_speed(self):
        gate = GroundSupportSafetyGate(_config(release_supported_frames=1))
        gate.update_assessment(
            _assessment(
                observation_adequate=False,
                supported_distance=0.10,
            )
        )
        gate.filter_command((0.60, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)

        gate.update_assessment(
            _assessment(
                stamp=10.1,
                received_wall_time=100.1,
                supported_distance=0.25,
            )
        )
        command = gate.filter_command((0.0, 0.0, 0.0), pose_yaw=0.0, wall_time=100.1)

        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, OBSERVATION_HOLD)

    def test_hold_release_ignores_localization_speed_for_envelope_index(self):
        gate = GroundSupportSafetyGate(_config(release_supported_frames=2))
        gate.update_assessment(
            _assessment(
                observation_adequate=False,
                supported_distance=0.10,
            )
        )
        gate.filter_command((0.30, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)

        gate.observe_odometry(
            1.0,
            linear_speed=0.59,
            angular_speed=0.0,
            wall_time=100.05,
        )
        for index in range(2):
            gate.update_assessment(
                _assessment(
                    stamp=10.1 + 0.1 * index,
                    received_wall_time=100.1 + 0.1 * index,
                    supported_distance=0.24,
                    support_boundary_detected=True,
                )
            )
            command = gate.filter_command(
                (0.0, 0.0, 0.0),
                pose_yaw=0.0,
                wall_time=100.1 + 0.1 * index,
            )

        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, CLEAR)

    def test_drop_stop_confirms_immediately_after_three_fresh_odom_samples(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                drop_confirmed=True,
                drop_intersects_sweep=True,
                drop_distance=0.45,
                drop_direction_world=(1.0, 0.0),
            )
        )
        gate.filter_command((0.20, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)

        gate.observe_odometry(1.0, linear_speed=0.01, angular_speed=0.01, wall_time=100.1)
        gate.observe_odometry(1.1, linear_speed=0.01, angular_speed=0.01, wall_time=100.2)
        gate.observe_odometry(1.2, linear_speed=0.01, angular_speed=0.01, wall_time=100.3)

        self.assertEqual(gate.state, SAFETY_BLOCKED)
        self.assertEqual(gate.reason_code, "negative_obstacle")

    def test_confirmed_drop_keeps_trigger_point_until_replan_acknowledges_it(self):
        gate = GroundSupportSafetyGate(_config(stop_confirmation_samples=1))
        gate.update_assessment(
            _assessment(
                drop_confirmed=True,
                drop_intersects_sweep=True,
                drop_distance=0.45,
                drop_direction_world=(1.0, 0.0),
                drop_point_world=(4.2, 5.3),
            )
        )
        gate.filter_command((0.20, 0.0, 0.0), pose_yaw=0.0, wall_time=100.0)
        gate.update_assessment(_assessment(stamp=10.1, drop_point_world=(9.0, 9.0)))
        gate.observe_odometry(
            1.0,
            linear_speed=0.01,
            angular_speed=0.01,
            wall_time=100.1,
        )

        self.assertEqual(gate.state, SAFETY_BLOCKED)
        self.assertEqual(gate.latched_drop_point_world, (4.2, 5.3))
        self.assertTrue(gate.reset_negative_obstacle_after_replan())
        self.assertEqual(gate.state, CLEAR)
        self.assertIsNone(gate.latched_drop_point_world)
        self.assertFalse(gate.reset_negative_obstacle_after_replan())

    def test_safety_stop_timeout_uses_sim_time(self):
        gate = GroundSupportSafetyGate(_config())
        gate.update_assessment(
            _assessment(
                drop_confirmed=True,
                drop_intersects_sweep=True,
                drop_distance=0.45,
                drop_direction_world=(1.0, 0.0),
            )
        )
        gate.filter_command(
            (0.20, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
            control_time=10.0,
        )

        gate.check_control_timeout(13.01, wall_time=100.1)

        self.assertEqual(gate.state, SAFETY_STOP_TIMEOUT)
        self.assertEqual(gate.reason_code, "ground_support_safety_stop_timeout")
        self.assertTrue(gate.terminal)

    def test_switching_temporary_hold_reason_does_not_restart_sim_timeout(self):
        gate = GroundSupportSafetyGate(_config(observation_hold_timeout=2.0))
        gate.update_assessment(_assessment(supported_distance=0.10))
        gate.filter_command(
            (0.20, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
            control_time=10.0,
        )
        self.assertEqual(gate.state, OBSERVATION_HOLD)

        gate.update_assessment(
            _assessment(
                stamp=10.1,
                received_wall_time=100.5,
                data_age=0.25,
                supported_distance=0.10,
            )
        )
        gate.filter_command(
            (0.20, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.5,
            control_time=10.5,
        )
        gate.check_control_timeout(12.01, wall_time=100.6)

        self.assertEqual(gate.state, SUPPORT_PATH_UNAVAILABLE)
        self.assertEqual(gate.reason_code, "ground_support_path_unavailable")

    def test_wall_progress_alone_does_not_expire_observation_hold(self):
        gate = GroundSupportSafetyGate(_config(observation_hold_timeout=2.0))
        gate.update_assessment(_assessment(supported_distance=0.10, received_control_time=10.0))
        gate.filter_command(
            (0.20, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
            control_time=10.0,
        )

        gate.check_control_timeout(10.0, wall_time=101.9)

        self.assertEqual(gate.state, OBSERVATION_HOLD)

    def test_frozen_sim_clock_uses_wall_watchdog_not_path_timeout(self):
        gate = GroundSupportSafetyGate(
            _config(safety_stop_timeout=3.0, wall_stall_timeout=30.0)
        )
        gate.update_assessment(_assessment(supported_distance=0.10, received_control_time=10.0))
        gate.filter_command(
            (0.20, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
            control_time=10.0,
        )

        gate.check_wall_timeout(129.99)

        self.assertEqual(gate.state, OBSERVATION_HOLD)

        gate.check_wall_timeout(130.01)

        self.assertEqual(gate.state, SAFETY_STOP_TIMEOUT)
        self.assertEqual(gate.reason_code, "ground_support_clock_or_odom_stalled")

    def test_boundary_margin_stops_before_verified_support_edge(self):
        gate = GroundSupportSafetyGate(_config(boundary_hold_margin=0.30))
        gate.update_assessment(_assessment(supported_distance=0.35))

        command = gate.filter_command(
            (0.30, 0.0, 0.0),
            pose_yaw=0.0,
            wall_time=100.0,
            control_time=10.0,
        )

        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(gate.state, OBSERVATION_HOLD)

    def test_stable_boundary_cells_are_latched_after_three_frames(self):
        gate = GroundSupportSafetyGate(
            _config(boundary_persistence_confirmations=3)
        )
        cells = ((1.0, 2.0), (1.0, 2.1), (1.0, 2.2))
        for index in range(3):
            gate.update_assessment(
                _assessment(
                    stamp=10.0 + 0.1 * index,
                    support_boundary_cells_world=cells,
                )
            )

        self.assertEqual(gate.latched_boundary_cells_world, cells)

    def test_boundary_geometry_jump_resets_confirmation(self):
        gate = GroundSupportSafetyGate(
            _config(
                boundary_persistence_confirmations=2,
                boundary_consistency_radius=0.20,
            )
        )
        gate.update_assessment(
            _assessment(support_boundary_cells_world=((1.0, 2.0),))
        )
        gate.update_assessment(
            _assessment(
                stamp=10.1,
                support_boundary_cells_world=((1.5, 2.0),),
            )
        )

        self.assertEqual(gate.latched_boundary_cells_world, ())
        self.assertEqual(gate.snapshot()["boundary_confirmation_count"], 1)


if __name__ == "__main__":
    unittest.main()
