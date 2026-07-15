"""负障碍诊断工具的单元测试。"""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"


def _load_tool(name: str):
    path = TOOLS_DIR / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


guard = _load_tool("ground_support_kill_guard")
analyzer = _load_tool("analyze_ground_support")

PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from danger_search.support_shadow import (  # noqa: E402
    NO_SUPPORT_EVIDENCE,
    SUPPORTED,
    UNKNOWN_SUPPORT,
    ShadowSupportConfig,
    ShadowSupportGrid,
    SweepWidthComponents,
)


class GroundSupportKillGuardTest(unittest.TestCase):
    def test_boundary_requires_floor_and_all_configured_half_planes(self):
        boundary = guard.KillOnlyBoundary(
            floor=1,
            min_x=1.08,
            min_y=5.95,
        )

        self.assertFalse(boundary.matches(floor=0, x=1.20, y=6.10))
        self.assertFalse(boundary.matches(floor=1, x=1.07, y=6.10))
        self.assertFalse(boundary.matches(floor=1, x=1.20, y=5.94))
        self.assertTrue(boundary.matches(floor=1, x=1.08, y=5.95))

    def test_boundary_can_require_comparable_actual_speed(self):
        boundary = guard.KillOnlyBoundary(floor=0, min_speed=0.45)

        self.assertFalse(boundary.matches(floor=0, x=5.0, y=5.0, speed=0.44))
        self.assertTrue(boundary.matches(floor=0, x=5.0, y=5.0, speed=0.45))

    def test_posture_limits_trigger_on_any_configured_hazard(self):
        limits = guard.PostureLimits(
            minimum_z=0.20,
            maximum_abs_roll=0.55,
            maximum_abs_pitch=0.60,
            active_after_sim=5.0,
        )

        self.assertIsNone(limits.violation(z=0.10, roll=1.0, pitch=1.0, sim_time=4.9))
        self.assertIsNone(limits.violation(z=0.31, roll=0.1, pitch=-0.1, sim_time=5.0))
        self.assertEqual(limits.violation(z=0.19, roll=0.1, pitch=0.1, sim_time=5.0), "minimum_z")
        self.assertEqual(limits.violation(z=0.31, roll=-0.56, pitch=0.1, sim_time=5.0), "maximum_abs_roll")
        self.assertEqual(limits.violation(z=0.31, roll=0.1, pitch=0.61, sim_time=5.0), "maximum_abs_pitch")

    def test_guard_module_has_no_ros_publisher_or_velocity_message(self):
        source = (TOOLS_DIR / "ground_support_kill_guard.py").read_text(encoding="utf-8")

        self.assertNotIn("rospy.Publisher", source)
        self.assertNotIn("Twist", source)
        self.assertNotIn("/cmd_vel", source)

    def test_observation_wrapper_keeps_raw_capture_opt_in_and_uses_guard(self):
        source = (TOOLS_DIR / "run_observation.sh").read_text(encoding="utf-8")

        self.assertIn("GROUND_SUPPORT_CAPTURE=0", source)
        self.assertIn("--ground-support-capture", source)
        self.assertIn("ground_support_kill_guard.py", source)
        self.assertIn("ground_support_telemetry.bag", source)

    def test_scene_manifest_wait_uses_static_request_marker(self):
        source = (TOOLS_DIR / "run_observation.sh").read_text(encoding="utf-8")

        self.assertIn("SCENE_REQUEST_MARKER", source)
        self.assertIn('[ "$manifest" -nt "$SCENE_REQUEST_MARKER" ]', source)
        self.assertNotIn('[ "$manifest" -nt "$BOOTSTRAP_LOG" ]', source)

    def test_raw_capture_contains_required_sensor_and_map_topics(self):
        source = (TOOLS_DIR / "run_observation.sh").read_text(encoding="utf-8")

        for topic in analyzer.RAW_REQUIRED_TOPICS:
            self.assertIn(topic, source)

    def test_time_only_guard_is_not_misreported_as_stop_distance(self):
        with tempfile.TemporaryDirectory() as temporary:
            run_dir = Path(temporary)
            observation = run_dir / "observation"
            observation.mkdir()
            (observation / "ground_support_kill_only_guard.json").write_text(
                json.dumps({"reason": "kill_only_sim_time", "sim_time": 15.0}),
                encoding="utf-8",
            )

            result = analyzer._measure_kill_only_stop(
                run_dir,
                run_dir / "unused.bag",
                alert_time=None,
            )

        self.assertFalse(result["available"])
        self.assertEqual(result["guard_reason"], "kill_only_sim_time")

    def test_rl_status_parser_reads_actual_received_command(self):
        value = (
            "in_rl=1 cmd_source=cmd_vel cmd_valid=1 "
            "cmd=(0.542454,-0.0174374,-0.131284)"
        )

        self.assertEqual(
            analyzer._parse_rl_status_command(value),
            (0.542454, -0.0174374, -0.131284),
        )
        self.assertIsNone(analyzer._parse_rl_status_command("cmd unavailable"))

    def test_shadow_capture_is_opt_in_and_records_rl_receiver(self):
        source = (TOOLS_DIR / "run_observation.sh").read_text(encoding="utf-8")

        self.assertIn("GROUND_SUPPORT_SHADOW=0", source)
        self.assertIn("--ground-support-shadow", source)
        self.assertIn("ground_support_shadow.py", source)
        self.assertIn("/junior_ctrl/rl_status", source)

    def test_gazebo_cli_probes_have_wall_time_bounds(self):
        source = (TOOLS_DIR / "run_observation.sh").read_text(encoding="utf-8")

        self.assertIn('timeout 5 gz model -m "$MODEL_NAME" -p', source)
        self.assertIn("timeout 5 gz camera -l", source)

    def test_shadow_runtime_has_no_publisher_or_forbidden_scene_input(self):
        source = (TOOLS_DIR / "ground_support_shadow.py").read_text(encoding="utf-8")

        self.assertNotIn("rospy.Publisher", source)
        self.assertNotIn("set_model_state", source)
        self.assertNotIn("layout_metadata", source)
        self.assertNotIn("scene_manifest", source)
        self.assertNotIn("danger_truth", source)


class DiagnosticStopLatchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.stop_latch = _load_tool("ground_support_stop_latch")

    def test_stale_inactive_or_stationary_alert_does_not_latch(self):
        state = self.stop_latch.StopLatchState(
            minimum_trigger_speed=0.10,
            maximum_alert_age=0.25,
            stop_speed_threshold=0.02,
            stop_hold_seconds=0.50,
            stop_position_tolerance=0.01,
        )

        self.assertFalse(
            state.consider_alert(
                now=10.0,
                alert={"active": False, "receive_sim_time": 10.0},
                speed=0.30,
                position=(0.0, 0.0),
            )
        )
        self.assertFalse(
            state.consider_alert(
                now=10.0,
                alert={"active": True, "receive_sim_time": 9.70},
                speed=0.30,
                position=(0.0, 0.0),
            )
        )
        self.assertFalse(
            state.consider_alert(
                now=10.0,
                alert={"active": True, "receive_sim_time": 10.0},
                speed=0.09,
                position=(0.0, 0.0),
            )
        )

    def test_latched_output_never_leaks_later_nonzero_command(self):
        state = self.stop_latch.StopLatchState(
            minimum_trigger_speed=0.10,
            maximum_alert_age=0.25,
            stop_speed_threshold=0.02,
            stop_hold_seconds=0.50,
            stop_position_tolerance=0.01,
        )
        self.assertEqual(state.output_command((0.4, 0.1, -0.2)), (0.4, 0.1, -0.2))
        self.assertTrue(
            state.consider_alert(
                now=10.0,
                alert={"active": True, "receive_sim_time": 9.90},
                speed=0.30,
                position=(1.0, 2.0),
            )
        )
        self.assertEqual(state.output_command((0.6, -0.2, 1.2)), (0.0, 0.0, 0.0))

    def test_stop_requires_low_speed_and_stable_position_for_full_hold(self):
        state = self.stop_latch.StopLatchState(
            minimum_trigger_speed=0.10,
            maximum_alert_age=0.25,
            stop_speed_threshold=0.02,
            stop_hold_seconds=0.50,
            stop_position_tolerance=0.01,
        )
        state.consider_alert(
            now=1.0,
            alert={"active": True, "receive_sim_time": 1.0},
            speed=0.30,
            position=(0.0, 0.0),
        )

        self.assertFalse(state.observe_motion(now=1.20, speed=0.01, position=(0.10, 0.0)))
        self.assertFalse(state.observe_motion(now=1.55, speed=0.01, position=(0.105, 0.0)))
        self.assertTrue(state.observe_motion(now=1.72, speed=0.01, position=(0.106, 0.0)))

    def test_wrapper_remaps_production_command_through_test_only_gate(self):
        source = (TOOLS_DIR / "run_observation.sh").read_text(encoding="utf-8")

        self.assertIn("--diagnostic-stop-latch", source)
        self.assertIn("/danger_search/diagnostic_cmd_vel_raw", source)
        self.assertIn("ground_support_stop_latch.py", source)


class SweepWidthComponentsTest(unittest.TestCase):
    def test_physical_half_width_combines_all_uncertainty_components(self):
        components = SweepWidthComponents(
            collision_half_width=0.097,
            foot_lateral_extent=0.137,
            lateral_tracking_error=0.063,
            turning_sweep=0.122,
            cloud_odom_registration_error=0.023,
            grid_quantization_error=0.071,
            safety_margin=0.020,
        )

        self.assertAlmostEqual(components.physical_half_width(), 0.436)
        self.assertEqual(components.neighbor_widths(delta=0.02), (0.416, 0.436, 0.456))


class SnapshotReplaySemanticsTest(unittest.TestCase):
    def test_latest_cloud_and_latest_odom_are_snapshotted_on_tick(self):
        events = [
            analyzer.ReplayEvent(0.01, "odom", SimpleNamespace(name="odom_a")),
            analyzer.ReplayEvent(0.02, "cloud", SimpleNamespace(stamp=1.0, name="cloud_a")),
            analyzer.ReplayEvent(0.08, "odom", SimpleNamespace(name="odom_b")),
            analyzer.ReplayEvent(0.11, "cloud", SimpleNamespace(stamp=2.0, name="cloud_b")),
            analyzer.ReplayEvent(0.18, "odom", SimpleNamespace(name="odom_c")),
            analyzer.ReplayEvent(0.21, "cloud", SimpleNamespace(stamp=3.0, name="cloud_c")),
        ]

        decisions = analyzer.replay_snapshot_decisions(
            events,
            tick_times=[0.10, 0.20, 0.30],
            floor_at=lambda _time: 1,
            ambiguous_at=lambda time: 0.15 <= time < 0.25,
            cloud_stamp=lambda cloud: cloud.stamp,
        )

        self.assertEqual(
            [(item.tick_time, item.odom.name, item.cloud.name, item.action) for item in decisions],
            [
                (0.10, "odom_b", "cloud_a", "update"),
                (0.20, "odom_c", "cloud_b", "skip_ambiguous"),
                (0.30, "odom_c", "cloud_c", "update"),
            ],
        )

    def test_skipped_cloud_stamp_is_consumed_and_not_replayed(self):
        cloud = SimpleNamespace(stamp=5.0, name="cloud")
        decisions = analyzer.replay_snapshot_decisions(
            [
                analyzer.ReplayEvent(0.01, "odom", SimpleNamespace(name="odom")),
                analyzer.ReplayEvent(0.02, "cloud", cloud),
            ],
            tick_times=[0.10, 0.20],
            floor_at=lambda _time: 0,
            ambiguous_at=lambda time: time < 0.15,
            cloud_stamp=lambda message: message.stamp,
        )

        self.assertEqual([item.action for item in decisions], ["skip_ambiguous"])

    def test_cloud_sampling_matches_production_stride_and_limit(self):
        points = [(float(index), 0.0, 0.0) for index in range(12)]

        sampled = analyzer.sample_cloud_points(points, stride=3, max_points=3)

        self.assertEqual(sampled, [points[0], points[3], points[6]])

    def test_zero_cloud_limit_reads_no_points(self):
        points = [(1.0, 2.0, 3.0)]

        self.assertEqual(
            analyzer.sample_cloud_points(points, stride=1, max_points=0),
            [],
        )

    def test_support_sweep_horizon_includes_front_reach_and_is_bounded(self):
        self.assertAlmostEqual(analyzer.support_sweep_horizon(0.0), 0.50)
        self.assertAlmostEqual(analyzer.support_sweep_horizon(0.35), 0.65)
        self.assertAlmostEqual(analyzer.support_sweep_horizon(-2.0), 0.80)

    def test_map_snapshot_precedes_update_at_equal_tick_time(self):
        decision = analyzer.SnapshotDecision(
            tick_time=1.0,
            odom=object(),
            cloud=object(),
            cloud_stamp=1.0,
            floor=0,
            action="update",
        )

        timeline = analyzer.merge_replay_timeline(
            [decision],
            [(1.0, object())],
        )

        self.assertEqual([item[1] for item in timeline], ["map", "update"])

    def test_map_comparison_marks_non_identical_replay_as_approximate(self):
        actual = np.array([[0, 30], [-1, 100]], dtype=np.int16)
        replayed = actual.copy()
        self.assertTrue(analyzer.compare_map_arrays(actual, replayed).exact)

        replayed[0, 0] = -1
        comparison = analyzer.compare_map_arrays(actual, replayed)
        self.assertFalse(comparison.exact)
        self.assertEqual(comparison.different_cells, 1)


class SupportObservationClassificationTest(unittest.TestCase):
    def test_non_visible_conditions_never_become_no_support(self):
        cases = [
            analyzer.SupportObservation(ever_observed=False),
            analyzer.SupportObservation(ever_observed=True, inside_fov=False),
            analyzer.SupportObservation(ever_observed=True, inside_fov=True, occluded=True),
            analyzer.SupportObservation(
                ever_observed=True,
                inside_fov=True,
                occluded=False,
                sufficient_points=False,
            ),
        ]

        results = [analyzer.classify_support_observation(case) for case in cases]

        self.assertEqual([item.state for item in results], ["UNKNOWN_SUPPORT"] * 4)
        self.assertEqual(
            [item.reason for item in results],
            ["never_observed", "outside_fov", "occluded", "insufficient_points"],
        )

    def test_confirmed_missing_support_requires_visible_sufficient_evidence(self):
        supported = analyzer.classify_support_observation(
            analyzer.SupportObservation(
                ever_observed=True,
                inside_fov=True,
                sufficient_points=True,
                support_point_count=4,
            )
        )
        missing = analyzer.classify_support_observation(
            analyzer.SupportObservation(
                ever_observed=True,
                inside_fov=True,
                sufficient_points=True,
                support_point_count=0,
                lower_return_count=3,
            )
        )

        self.assertEqual(supported.state, "SUPPORTED")
        self.assertEqual(missing.state, "NO_SUPPORT_EVIDENCE")

    def test_scene_threshold_is_not_promoted_to_production_default(self):
        result = analyzer.select_scene_candidate_confirmation(
            positive_runs=[4, 5],
            false_positive_runs=[1, 2],
            neighbor_thresholds_stable=True,
            independent_normal_scene_count=0,
        )

        self.assertEqual(result.minimum_frames, 3)
        self.assertTrue(result.scene_only)
        self.assertFalse(result.production_ready)

    def test_lower_return_projects_to_expected_floor_crossing(self):
        projected = analyzer.project_lower_returns_to_floor(
            sensor_origin=np.array([0.0, 0.0, 1.0]),
            points=np.array([[2.0, 0.0, -1.0], [0.0, 1.0, 1.2]]),
            floor_z=0.0,
            minimum_drop=0.2,
        )

        np.testing.assert_allclose(projected, np.array([[1.0, 0.0, 0.0]]))

    def test_cell_with_no_positive_ray_stays_unknown(self):
        result = analyzer.classify_support_cell(
            support_hits=0,
            lower_crossing_hits=0,
            inside_fov=True,
            occluded=False,
            minimum_hits=2,
        )

        self.assertEqual(result.state, "UNKNOWN_SUPPORT")
        self.assertEqual(result.reason, "insufficient_points")

    def test_confirmed_lower_crossing_is_no_support_but_occlusion_is_not(self):
        missing = analyzer.classify_support_cell(
            support_hits=0,
            lower_crossing_hits=3,
            inside_fov=True,
            occluded=False,
            minimum_hits=2,
        )
        occluded = analyzer.classify_support_cell(
            support_hits=0,
            lower_crossing_hits=0,
            inside_fov=True,
            occluded=True,
            minimum_hits=2,
        )

        self.assertEqual(missing.state, "NO_SUPPORT_EVIDENCE")
        self.assertEqual(occluded.state, "UNKNOWN_SUPPORT")
        self.assertEqual(occluded.reason, "occluded")


def _shadow_config(**overrides):
    values = {
        "resolution": 0.1,
        "origin_x": 0.0,
        "origin_y": 0.0,
        "floor_height": 2.6,
        "cloud_stride": 1,
        "cloud_max_points": 1000,
        "support_band": 0.08,
        "minimum_drop": 0.30,
        "minimum_cell_hits": 2,
        "required_consecutive_frames": 2,
        "minimum_alert_cells": 1,
        "sweep_half_width": 0.12,
        "snapshot_period": 0.5,
        "threshold_widths": (0.12, 0.18),
        "threshold_hit_counts": (1, 2, 3),
        "threshold_consecutive_frames": (1, 2, 3),
    }
    values.update(overrides)
    return ShadowSupportConfig(**values)


class ShadowSupportGridTest(unittest.TestCase):
    def setUp(self):
        self.sensor_origin = np.array([0.0, 0.0, 0.40])
        self.sensor_rotation = np.eye(3)
        self.robot_xy = np.array([0.0, 0.0])
        self.path = [np.array([0.0, 0.0]), np.array([1.0, 0.0])]

    def _process(self, grid, stamp, points, *, ambiguous=False):
        return grid.process_frame(
            stamp=stamp,
            floor=0,
            robot_xy=self.robot_xy,
            robot_yaw=0.0,
            sensor_origin=self.sensor_origin,
            sensor_rotation=self.sensor_rotation,
            points=np.asarray(points, dtype=float).reshape((-1, 3)),
            path=self.path,
            command_linear=0.2,
            ambiguous=ambiguous,
        )

    def test_direct_floor_return_marks_supported(self):
        grid = ShadowSupportGrid(_shadow_config())

        result = self._process(
            grid,
            1.0,
            [[0.45, 0.02, 0.0], [0.46, 0.03, 0.01]],
        )
        cell = grid.cell_snapshot(0, (4, 0))

        self.assertEqual(result["action"], "update")
        self.assertEqual(cell["state"], SUPPORTED)
        self.assertEqual(cell["support_frames"], 1)
        self.assertEqual(
            grid.sparse_grid_snapshot()["0"][0]["state"],
            SUPPORTED,
        )

    def test_lower_surface_requires_enough_hits_and_consecutive_frames(self):
        grid = ShadowSupportGrid(_shadow_config())
        # 两条射线都在 x=0.4 左右穿过当前楼面，随后命中明确更低表面。
        points = [[0.9, 0.01, -0.4], [0.92, 0.02, -0.42]]

        first = self._process(grid, 1.0, points)
        first_cell = grid.cell_snapshot(0, (4, 0))
        second = self._process(grid, 1.1, points)
        second_cell = grid.cell_snapshot(0, (4, 0))

        self.assertEqual(first_cell["state"], UNKNOWN_SUPPORT)
        self.assertEqual(first_cell["lower_streak"], 1)
        self.assertEqual(second_cell["state"], NO_SUPPORT_EVIDENCE)
        self.assertEqual(second_cell["lower_streak"], 2)
        self.assertTrue(second["hypothetical_alert"]["active"])

    def test_no_return_and_single_lower_hit_stay_unknown(self):
        grid = ShadowSupportGrid(_shadow_config())

        empty = self._process(grid, 1.0, [])
        sparse = self._process(grid, 1.1, [[0.9, 0.0, -0.4]])

        self.assertFalse(empty["hypothetical_alert"]["active"])
        self.assertFalse(sparse["hypothetical_alert"]["active"])
        self.assertEqual(grid.cell_state(0, (4, 0)), UNKNOWN_SUPPORT)

    def test_support_evidence_wins_over_conflicting_lower_return(self):
        grid = ShadowSupportGrid(_shadow_config(required_consecutive_frames=1))
        self._process(
            grid,
            1.0,
            [[0.45, 0.02, 0.0], [0.46, 0.03, 0.01]],
        )

        self._process(
            grid,
            1.1,
            [[0.9, 0.01, -0.4], [0.92, 0.02, -0.42]],
        )

        self.assertEqual(grid.cell_state(0, (4, 0)), SUPPORTED)

    def test_ambiguous_cloud_is_consumed_and_never_replayed(self):
        grid = ShadowSupportGrid(_shadow_config(required_consecutive_frames=1))
        points = [[0.9, 0.01, -0.4], [0.92, 0.02, -0.42]]

        skipped = self._process(grid, 1.0, points, ambiguous=True)
        duplicate = self._process(grid, 1.0, points, ambiguous=False)
        fresh = self._process(grid, 1.1, points, ambiguous=False)

        self.assertEqual(skipped["action"], "skip_floor_transition_ambiguous")
        self.assertEqual(duplicate["action"], "skip_duplicate_cloud")
        self.assertEqual(fresh["action"], "update")
        self.assertEqual(grid.skipped_ambiguous_count, 1)
        self.assertEqual(grid.updated_frame_count, 1)

    def test_non_elevator_absence_does_not_create_negative_evidence(self):
        grid = ShadowSupportGrid(_shadow_config())

        for stamp in (1.0, 1.1, 1.2, 1.3):
            self._process(grid, stamp, [])

        self.assertEqual(grid.no_support_cell_count(0), 0)

    def test_threshold_sensitivity_is_reported_without_changing_default_state(self):
        grid = ShadowSupportGrid(_shadow_config())
        points = [[0.9, 0.01, -0.4], [0.92, 0.02, -0.42]]

        result = self._process(grid, 1.0, points)

        self.assertIn("w0.12_h1_c1", result["threshold_sensitivity"])
        self.assertIn("w0.18_h3_c3", result["threshold_sensitivity"])
        self.assertEqual(grid.cell_state(0, (4, 0)), UNKNOWN_SUPPORT)


if __name__ == "__main__":
    unittest.main()
