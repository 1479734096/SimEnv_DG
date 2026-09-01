import ast
from pathlib import Path
import unittest
from types import SimpleNamespace

from danger_search.floor_transition_runtime import (
    FloorCoordinateModel,
    FloorTransitionRuntime,
)
from danger_search.elevator_observation_runtime import ElevatorObservationRuntime
from danger_search.elevator_registration_runtime import ElevatorRegistrationRuntime
from danger_search.exploration_runtime import ExplorationRuntime
from danger_search.map_update_runtime import ImageDetectionResult, MapUpdateRuntime
from danger_search.detection_tracker import (
    REJECTED,
    DetectionCandidate,
    DetectionTracker,
)
from danger_search.mapper import MultiFloorGridMap, OCC
from danger_search.navigation_context import NavigationTaskContext
from danger_search.perception_runtime import (
    CloudInput,
    CloudPreparation,
    ImageInput,
    ImagePreparation,
    INPUT_INVALID,
    INPUT_MATCHED,
    INPUT_POSE_PENDING,
    PerceptionRuntime,
)
from danger_search.perception_mapping_runtime import PerceptionMappingRuntime
from danger_search.pose_provider import POSE_MATCHED, POSE_PENDING
from danger_search.return_home_runtime import ReturnHomeRuntime


class PlanningConstraintBoundaryTest(unittest.TestCase):
    def test_runtime_wall_thread_is_unconditional_and_checks_tick_liveness_first(self):
        path = Path(__file__).resolve().parents[1] / "danger_search" / "mission_node_runtime.py"
        source = path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(path))
        start_method = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef) and node.name == "start"
        )
        gate_if = next(
            node
            for node in start_method.body
            if isinstance(node, ast.If)
            and isinstance(node.test, ast.Compare)
            and any(
                isinstance(part, ast.Attribute)
                and part.attr == "ground_support_gate"
                for part in ast.walk(node.test)
            )
        )
        self.assertFalse(
            any(
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "Thread"
                for node in ast.walk(gate_if)
            )
        )
        wall_method = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef)
            and node.name == "_ground_support_wall_watch"
        )
        tick_index = source.index("check_tick_wall_timeout")
        support_index = source.index("check_ground_support_wall_timeout")
        self.assertLess(tick_index, support_index)
        wall_calls = {
            node.func.attr
            for node in ast.walk(wall_method)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
        }
        self.assertIn("check_tick_wall_timeout", wall_calls)

    def test_all_production_astar_callers_supply_frozen_constraints(self):
        package = Path(__file__).resolve().parents[1] / "danger_search"
        callers = (
            "mission_manager.py",
            "return_home_runtime.py",
            "exploration_runtime.py",
            "door_navigation_runtime.py",
            "frontier.py",
            "return_home_planner.py",
        )
        missing = []
        for name in callers:
            path = package / name
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                if not isinstance(node.func, ast.Name) or node.func.id != "astar_plan":
                    continue
                if not any(keyword.arg == "constraints" for keyword in node.keywords):
                    missing.append(f"{name}:{node.lineno}")
        self.assertEqual(missing, [])

    def test_ground_support_feedback_has_no_formal_map_mutation_api(self):
        package = Path(__file__).resolve().parents[1] / "danger_search"
        sources = "\n".join(
            (package / name).read_text(encoding="utf-8")
            for name in (
                "ground_support_runtime.py",
                "ground_support_coordinator.py",
            )
        )
        self.assertNotIn("clear_negative_cells_with_support", sources)
        self.assertNotIn("mark_negative_cells", sources)
        self.assertNotIn("apply_map_updates", sources)

        mapper_source = (package / "mapper.py").read_text(encoding="utf-8")
        self.assertNotIn("def mark_negative(", mapper_source)
        self.assertNotIn("def clear_negative_with_support(", mapper_source)


class _Stamp:
    def __init__(self, value):
        self._value = value

    def to_sec(self):
        return self._value


def _cloud(stamp):
    return SimpleNamespace(header=SimpleNamespace(stamp=_Stamp(stamp)))


class PerceptionRuntimeTest(unittest.TestCase):
    def test_cloud_cursor_requires_explicit_consumption(self):
        runtime = PerceptionRuntime(SimpleNamespace())
        snapshot = SimpleNamespace(cloud_msg=_cloud(1.0), cloud_generation=7)

        first = runtime.cloud_input(snapshot)
        self.assertEqual(first.generation, 7)
        self.assertEqual(first.stamp, 1.0)
        self.assertEqual(runtime.cloud_input(snapshot).generation, 7)

        runtime.consume_cloud(7)
        self.assertIsNone(runtime.cloud_input(snapshot))

    def test_invalid_generation_is_rejected_at_commit(self):
        runtime = PerceptionRuntime(SimpleNamespace())
        with self.assertRaisesRegex(ValueError, "generation"):
            runtime.consume_cloud(0)

    def test_cloud_preparation_freezes_lookup_without_consuming_frame(self):
        runtime = PerceptionRuntime(SimpleNamespace())
        snapshot = SimpleNamespace(cloud_msg=_cloud(1.0), cloud_generation=7)
        fallback = object()
        lookup = SimpleNamespace(status=POSE_PENDING, sample=None)
        localization = SimpleNamespace(lookup=lambda *_args, **_kwargs: lookup)

        prepared = runtime.prepare_cloud(
            snapshot,
            localization_runtime=localization,
            fallback_pose_sample=fallback,
        )

        self.assertEqual(prepared.status, INPUT_POSE_PENDING)
        self.assertIs(prepared.pose_sample, fallback)
        self.assertEqual(runtime.cloud_input(snapshot).generation, 7)

    def test_matched_cloud_uses_time_matched_pose(self):
        runtime = PerceptionRuntime(SimpleNamespace())
        snapshot = SimpleNamespace(cloud_msg=_cloud(1.0), cloud_generation=7)
        matched = object()
        lookup = SimpleNamespace(status=POSE_MATCHED, sample=matched)

        prepared = runtime.prepare_cloud(
            snapshot,
            localization_runtime=SimpleNamespace(
                lookup=lambda *_args, **_kwargs: lookup
            ),
            fallback_pose_sample=object(),
        )

        self.assertEqual(prepared.status, INPUT_MATCHED)
        self.assertIs(prepared.pose_sample, matched)

    def test_unsynchronized_images_are_rejected_before_pose_lookup(self):
        runtime = PerceptionRuntime(SimpleNamespace(sync_slop=0.08))
        bundle = SimpleNamespace(
            rgb_msg=_cloud(1.0),
            depth_msg=_cloud(1.2),
            camera_info_msg=_cloud(1.0),
        )
        snapshot = SimpleNamespace(image_bundle=bundle, image_generation=3)

        prepared = runtime.prepare_image(
            snapshot,
            localization_runtime=SimpleNamespace(
                lookup=lambda *_args, **_kwargs: self.fail("不得查询位姿")
            ),
        )

        self.assertEqual(prepared.status, INPUT_INVALID)
        self.assertEqual(runtime.image_input(snapshot).generation, 3)


class FloorTransitionRuntimeTest(unittest.TestCase):
    def test_absolute_coordinates_preserve_points_and_estimate_floor(self):
        model = FloorCoordinateModel("absolute", 2.6)
        points = [(1.0, 2.0, 0.4)]

        self.assertEqual(
            model.logical_xyz((1.0, 2.0, 2.9), 1),
            (1.0, 2.0, 2.9),
        )
        self.assertIs(model.logical_points(points, 1), points)
        self.assertEqual(
            model.estimate_floor((1.0, 2.0, 2.9), current_floor=0),
            1,
        )

    def test_floor_local_coordinates_apply_exactly_one_offset(self):
        model = FloorCoordinateModel("floor_local", 2.6)

        self.assertEqual(
            model.logical_xyz((1.0, 2.0, 0.3), 1),
            (1.0, 2.0, 2.9),
        )
        self.assertEqual(
            model.logical_points([(1.0, 2.0, 0.3)], 1),
            [(1.0, 2.0, 2.9)],
        )
        self.assertEqual(
            model.estimate_floor((1.0, 2.0, 0.3), current_floor=1),
            1,
        )

    def test_floor_coordinate_contract_rejects_invalid_values(self):
        with self.assertRaises(ValueError):
            FloorCoordinateModel("unknown", 2.6)
        with self.assertRaises(ValueError):
            FloorCoordinateModel("absolute", 0.0)

    def test_explicit_transition_requires_confirmation_and_two_fresh_samples(self):
        runtime = FloorTransitionRuntime()
        runtime.begin_explicit(
            source_floor=0,
            target_floor=1,
            phase="MOVE",
            now=1.0,
            odom_stamp=10.0,
        )
        blocked = runtime.evaluate(
            current_floor=0,
            estimated_floor=0,
            is_elevator_task=True,
            task_target_floor=1,
            task_phase="MOVE",
            odom_stamp=11.0,
            now=2.0,
        )
        self.assertTrue(blocked.blocked)

        runtime.confirm(1, odom_stamp=12.0)
        first = runtime.evaluate(
            current_floor=1,
            estimated_floor=1,
            is_elevator_task=True,
            task_target_floor=1,
            task_phase="EXIT",
            odom_stamp=13.0,
            now=3.0,
        )
        second = runtime.evaluate(
            current_floor=1,
            estimated_floor=1,
            is_elevator_task=False,
            task_target_floor=None,
            task_phase=None,
            odom_stamp=14.0,
            now=4.0,
        )
        self.assertTrue(first.blocked)
        self.assertFalse(second.blocked)
        self.assertIsNotNone(second.exited)
        self.assertFalse(runtime.active)

    def test_mismatch_alignment_resets_before_releasing(self):
        runtime = FloorTransitionRuntime()
        first = runtime.evaluate(
            current_floor=0,
            estimated_floor=1,
            is_elevator_task=True,
            task_target_floor=1,
            task_phase="WAIT",
            odom_stamp=1.0,
            now=0.0,
        )
        self.assertTrue(first.entered)
        self.assertTrue(first.blocked)
        self.assertTrue(
            runtime.evaluate(
                current_floor=0,
                estimated_floor=0,
                is_elevator_task=True,
                task_target_floor=1,
                task_phase="WAIT",
                odom_stamp=2.0,
                now=1.0,
            ).blocked
        )
        self.assertFalse(
            runtime.evaluate(
                current_floor=0,
                estimated_floor=0,
                is_elevator_task=True,
                task_target_floor=1,
                task_phase="WAIT",
                odom_stamp=3.0,
                now=2.0,
            ).blocked
        )


class MapUpdateRuntimeTest(unittest.TestCase):
    def test_cloud_update_returns_obstacle_summary_and_updates_map(self):
        runtime = MapUpdateRuntime.__new__(MapUpdateRuntime)
        runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_min_z=0.15,
            obstacle_max_z=1.4,
        )
        result = runtime.update_cloud(
            floor_index=0,
            cloud_generation=1,
            map_epoch=1,
            localization_epoch=1,
            robot_xyz=(0.0, 0.0, 0.3),
            robot_pose=(0.0, 0.0, 0.0),
            points=[(0.4, 0.0, 0.4), (0.8, 0.7, 0.4)],
            opened_door_exclusion=None,
        )
        self.assertEqual(len(result.points), 2)
        self.assertAlmostEqual(result.forward_obstacle.distance, 0.4)
        self.assertAlmostEqual(result.radial_obstacle_distance, 0.4)

    def test_confirmed_nms_tracks_create_and_reproject_semantic_occupancy(self):
        runtime = MapUpdateRuntime.__new__(MapUpdateRuntime)
        runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
            obstacle_inflation_radius=0.25,
        )
        runtime.detector = SimpleNamespace(target_radius=0.15)
        runtime.tracker = DetectionTracker(
            association_radius=0.2,
            min_hits=2,
            nms_radius=0.9,
        )
        runtime.mapper.floor(0)
        runtime.mapper.floor(1)

        runtime.tracker.update(
            [
                DetectionCandidate.from_sequence(
                    [0.0, 0.0, 0.15], confidence=0.9, distance=2.0
                ),
                DetectionCandidate.from_sequence(
                    [0.5, 0.0, 0.15], confidence=0.8, distance=2.0
                ),
                DetectionCandidate.from_sequence(
                    [0.0, 0.0, 2.75], confidence=0.95, distance=2.0
                ),
                DetectionCandidate.from_sequence(
                    [2.0, 0.0, 0.15], confidence=0.9, distance=2.0
                ),
            ],
            now=1.0,
        )
        tentative = runtime.synchronize_confirmed_danger_obstacles(now=1.0)
        self.assertEqual(tentative.selected_track_ids, ())
        self.assertEqual(runtime.mapper.floor(0).semantic_occupied_owner_cells, {})

        runtime.tracker.update(
            [
                DetectionCandidate.from_sequence(
                    [0.0, 0.0, 0.15], confidence=0.9, distance=2.0
                ),
                DetectionCandidate.from_sequence(
                    [0.5, 0.0, 0.15], confidence=0.8, distance=2.0
                ),
                DetectionCandidate.from_sequence(
                    [0.0, 0.0, 2.75], confidence=0.95, distance=2.0
                ),
            ],
            now=2.0,
        )

        first = runtime.synchronize_confirmed_danger_obstacles(now=2.0)
        floor_zero = runtime.mapper.floor(0)
        floor_one = runtime.mapper.floor(1)
        zero_cell = floor_zero.world_to_grid(0.0, 0.0)
        one_cell = floor_one.world_to_grid(0.0, 0.0)
        self.assertIsNotNone(zero_cell)
        self.assertIsNotNone(one_cell)
        self.assertEqual(first.selected_track_ids, (3, 1))
        self.assertEqual(first.instantiated_floor_indices, (0, 1))
        self.assertIn("danger:1", floor_zero.semantic_occupied_owners[zero_cell])
        self.assertNotIn("danger:2", floor_zero.semantic_occupied_owner_cells)
        self.assertIn("danger:3", floor_one.semantic_occupied_owners[one_cell])

        unchanged_overlap = runtime.synchronize_confirmed_danger_obstacles(
            now=2.0,
            robot_floor=0,
            robot_xy=(0.0, 0.0),
        )
        self.assertEqual(unchanged_overlap.changed_owner_ids, ())
        self.assertEqual(unchanged_overlap.overlap_owner_ids, ("danger:1",))

        corrected = runtime.apply_planar_correction(
            0.6,
            -0.6,
            now=2.0,
            robot_floor=0,
            robot_xy=(2.0, 2.0),
        )
        moved_zero = floor_zero.world_to_grid(0.6, -0.6)
        moved_one = floor_one.world_to_grid(0.6, -0.6)
        self.assertIsNotNone(moved_zero)
        self.assertIsNotNone(moved_one)
        self.assertEqual(corrected.translated_track_count, 4)
        self.assertEqual(int(floor_zero.data[moved_zero[1], moved_zero[0]]), OCC)
        self.assertEqual(int(floor_one.data[moved_one[1], moved_one[0]]), OCC)
        self.assertNotIn("danger:1", floor_zero.semantic_occupied_owners.get(zero_cell, set()))
        self.assertAlmostEqual(runtime.tracker.tracks[0].positions[0][0], 0.6)
        self.assertAlmostEqual(runtime.tracker.tracks[0].positions[0][1], -0.6)

        runtime.tracker.update([], now=40.0)
        rejected = runtime.synchronize_confirmed_danger_obstacles(now=40.0)
        self.assertEqual(runtime.tracker.tracks[3].status, REJECTED)
        self.assertNotIn("danger:4", rejected.changed_owner_ids)
        self.assertNotIn("danger:4", floor_zero.semantic_occupied_owner_cells)

    def test_high_quality_tentative_track_creates_then_expires_semantic_obstacle(self):
        """近场双帧红球先保护路径；失去新帧后必须自行撤销。"""
        runtime = MapUpdateRuntime.__new__(MapUpdateRuntime)
        runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=6.0,
            length=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
            obstacle_inflation_radius=0.25,
        )
        runtime.detector = SimpleNamespace(target_radius=0.15)
        runtime.tracker = DetectionTracker(
            association_radius=0.2,
            min_hits=4,
            obstacle_min_hits=2,
            obstacle_min_confidence=0.90,
            obstacle_max_average_rmse=0.025,
            obstacle_max_age=0.75,
            obstacle_min_observation_span=0.08,
        )
        floor = runtime.mapper.floor(0)
        candidate = DetectionCandidate.from_sequence(
            [0.0, 0.0, 0.15], confidence=0.95, rmse=0.01, distance=1.0
        )
        runtime.tracker.update([candidate], now=1.0)
        runtime.tracker.update([candidate], now=1.1)

        protected = runtime.synchronize_confirmed_danger_obstacles(now=1.1)
        cell = floor.world_to_grid(0.0, 0.0)
        self.assertIsNotNone(cell)
        self.assertEqual(protected.selected_track_ids, (1,))
        self.assertEqual(protected.protective_track_ids, (1,))
        self.assertIn("danger:1", floor.semantic_occupied_owners[cell])

        expired = runtime.synchronize_confirmed_danger_obstacles(now=1.86)
        self.assertEqual(expired.selected_track_ids, ())
        self.assertEqual(expired.protective_track_ids, ())
        self.assertIn("danger:1", expired.changed_owner_ids)
        self.assertNotIn("danger:1", floor.semantic_occupied_owner_cells)


class PerceptionMappingRuntimeTest(unittest.TestCase):
    def test_cloud_cache_updates_existing_points_without_mapping(self):
        consumed = []
        perception = SimpleNamespace(
            read_cloud_xyz=lambda *_args, **_kwargs: [(0.4, 0.0, 0.4)],
            consume_cloud=consumed.append,
        )
        map_runtime = SimpleNamespace(
            last_cloud_stamp=None,
            latest_cloud_points=[(9.0, 9.0, 9.0)],
        )
        runtime = PerceptionMappingRuntime(
            perception=perception,
            map_runtime=map_runtime,
            floor_coordinates=FloorCoordinateModel("floor_local", 2.6),
            pose_provider=None,
        )
        sample = SimpleNamespace(
            xyz=(0.0, 0.0, 0.3),
            xy_yaw=(0.0, 0.0, 0.0),
        )
        preparation = CloudPreparation(
            INPUT_MATCHED,
            CloudInput(object(), 7, 1.0),
            SimpleNamespace(sample=sample),
            sample,
        )

        runtime.cache_cloud(
            preparation,
            floor_index=1,
            cloud_max_points=100,
            cloud_stride=1,
        )

        self.assertEqual(consumed, [7])
        self.assertEqual(map_runtime.last_cloud_stamp, 1.0)
        self.assertEqual(map_runtime.latest_cloud_points, [(0.4, 0.0, 3.0)])

    def test_cloud_commit_owns_decode_cursor_and_derived_obstacle_state(self):
        consumed = []
        perception = SimpleNamespace(
            read_cloud_xyz=lambda *_args, **_kwargs: [(0.4, 0.0, 0.4)],
            consume_cloud=consumed.append,
        )
        map_runtime = MapUpdateRuntime.__new__(MapUpdateRuntime)
        map_runtime.mapper = MultiFloorGridMap(
            resolution=0.1,
            width=4.0,
            length=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            obstacle_min_z=0.15,
            obstacle_max_z=1.4,
        )
        runtime = PerceptionMappingRuntime(
            perception=perception,
            map_runtime=map_runtime,
            floor_coordinates=FloorCoordinateModel("absolute", 2.6),
            pose_provider=None,
        )
        sample = SimpleNamespace(
            xyz=(0.0, 0.0, 0.3),
            xy_yaw=(0.0, 0.0, 0.0),
        )
        preparation = CloudPreparation(
            INPUT_MATCHED,
            CloudInput(object(), 7, 1.0),
            SimpleNamespace(sample=sample),
            sample,
        )

        committed = runtime.commit_cloud(
            preparation,
            floor_index=0,
            map_epoch=1,
            localization_epoch=1,
            cloud_max_points=100,
            cloud_stride=1,
            opened_door_exclusion=None,
        )

        self.assertEqual(consumed, [7])
        self.assertEqual(map_runtime.last_cloud_stamp, 1.0)
        self.assertEqual(map_runtime.latest_cloud_points, [(0.4, 0.0, 0.4)])
        self.assertAlmostEqual(committed.map_result.forward_obstacle.distance, 0.4)

    def test_image_commit_propagates_floor_and_time_matched_robot_xy(self):
        consumed = []
        received = []
        perception = SimpleNamespace(consume_image=consumed.append)
        map_runtime = SimpleNamespace(
            last_image_stamp=None,
            update_images=lambda bundle, camera_to_world, **kwargs: (
                received.append((bundle, camera_to_world, kwargs))
                or ImageDetectionResult(())
            ),
        )
        pose_provider = SimpleNamespace(
            sensor_to_world_matrix=lambda *_args, **_kwargs: (
                (1.0, 0.0, 0.0, 0.0),
                (0.0, 1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            )
        )
        runtime = PerceptionMappingRuntime(
            perception=perception,
            map_runtime=map_runtime,
            floor_coordinates=FloorCoordinateModel("absolute", 2.6),
            pose_provider=pose_provider,
        )
        sample = SimpleNamespace(
            xyz=(1.0, 2.0, 0.3),
            xy_yaw=(1.0, 2.0, 0.1),
        )
        bundle = SimpleNamespace()
        preparation = ImagePreparation(
            INPUT_MATCHED,
            ImageInput(bundle, 9, (3.0, 3.0, 3.0)),
            SimpleNamespace(sample=sample),
        )

        runtime.commit_image(
            preparation,
            floor_index=1,
            camera_mount_xyz=(0.0, 0.0, 0.0),
            camera_mount_rpy=(0.0, 0.0, 0.0),
            depth_frame_is_optical=False,
            now=3.0,
        )

        self.assertEqual(consumed, [9])
        self.assertEqual(map_runtime.last_image_stamp, 3.0)
        self.assertEqual(len(received), 1)
        self.assertEqual(received[0][2]["floor_index"], 1)
        self.assertEqual(received[0][2]["robot_xy"], (1.0, 2.0))
        self.assertEqual(received[0][2]["now"], 3.0)

    def test_unmatched_cloud_cannot_commit(self):
        runtime = PerceptionMappingRuntime(
            perception=SimpleNamespace(),
            map_runtime=SimpleNamespace(),
            floor_coordinates=FloorCoordinateModel("absolute", 2.6),
            pose_provider=None,
        )
        with self.assertRaisesRegex(ValueError, "已匹配"):
            runtime.commit_cloud(
                CloudPreparation(INPUT_INVALID, None, None, object()),
                floor_index=0,
                map_epoch=1,
                localization_epoch=1,
                cloud_max_points=100,
                cloud_stride=1,
                opened_door_exclusion=None,
            )


class MissionStateOwnerTest(unittest.TestCase):
    def test_exploration_progress_is_bounded_and_reports_growth(self):
        runtime = ExplorationRuntime()
        runtime.record_progress(
            now=1.0,
            known_cells=10,
            free_or_visited_cells=8,
            confirmed_dangers=0,
            retention_seconds=10.0,
        )
        runtime.record_progress(
            now=3.0,
            known_cells=16,
            free_or_visited_cells=12,
            confirmed_dangers=2,
            retention_seconds=10.0,
        )
        growth = runtime.recent_progress(now=3.0, window_seconds=5.0)
        self.assertEqual(growth["known_cells_growth"], 3.0)
        self.assertEqual(growth["free_or_visited_growth"], 2.0)
        self.assertEqual(growth["danger_growth"], 1.0)

        runtime.record_progress(
            now=20.0,
            known_cells=20,
            free_or_visited_cells=15,
            confirmed_dangers=2,
            retention_seconds=10.0,
        )
        self.assertEqual(len(runtime.progress_history), 1)

    def test_return_feedback_reset_keeps_task_outcome_outside_runtime(self):
        runtime = ReturnHomeRuntime(
            stuck_anchor=(1.0, 2.0),
            stuck_count=3,
            failed_segments=[(0.0, 0.0, 1.0, 1.0)],
            candidate_reject_count=2,
            retreat_target=(0.5, 0.5),
        )
        runtime.reset_feedback()
        self.assertIsNone(runtime.stuck_anchor)
        self.assertEqual(runtime.stuck_count, 0)
        self.assertEqual(runtime.failed_segments, [])
        self.assertEqual(runtime.candidate_reject_count, 0)
        self.assertIsNone(runtime.retreat_target)

    def test_elevator_observation_reset_is_local_to_refresh_family(self):
        registration = ElevatorRegistrationRuntime(
            visual_confirmations=4,
            refresh_context=("elevator", 0, "door"),
            refresh_confirmations=3,
            refresh_rejections={"shape": 2},
        )
        runtime = ElevatorObservationRuntime(registration=registration)
        runtime.registration.reset_refresh()
        self.assertEqual(runtime.registration.visual_confirmations, 4)
        self.assertIsNone(runtime.registration.refresh_context)
        self.assertEqual(runtime.registration.refresh_confirmations, 0)
        self.assertEqual(runtime.registration.refresh_rejections, {})

    def test_navigation_context_preserves_task_status_values(self):
        context = NavigationTaskContext(
            controller=object(),
            mapper=object(),
            building_client=object(),
            current_floor=0,
            planning_constraints=None,
            door_pass_max_linear=None,
            open_door_geometry_refresh_enabled=True,
            elevator_geometry_refresh_enabled=True,
            elevator_geometry_refresh_standoff=1.2,
            elevator_opening_confirmation_timeout=8.0,
            elevator_local_crossing_geometry_enabled=True,
            elevator_local_crossing_timeout=4.0,
            elevator_local_crossing_max_center_correction=0.8,
            elevator_stuck_retry_limit=4,
            exit_first_max_linear=None,
            exit_cabin_backoff_enabled=False,
            clear_opened_door_area_fn=lambda _door: None,
            clear_elevator_crossing_area_fn=lambda _geometry: None,
            refresh_open_door_geometry_fn=lambda *_args: "running",
            refresh_elevator_geometry_fn=lambda *_args: "idle",
            refresh_elevator_crossing_geometry_fn=lambda **_kwargs: None,
            elevator_crossing_obstacle_fn=lambda *_args, **_kwargs: None,
            record_failure_fn=lambda _reason: None,
            log_structured_fn=lambda _payload: None,
            begin_floor_transition_fn=lambda **_kwargs: None,
            confirm_floor_transition_fn=lambda _floor: None,
            begin_door_open_confirmation_fn=lambda _door: None,
            confirm_door_open_fn=lambda *_args: True,
            finish_door_open_confirmation_fn=lambda _door: None,
            elevator_visual_preparation_pending_fn=lambda _door: False,
            recover_with_rotation_clearance_fn=lambda _reason: True,
            elevator_start_retreat_target_fn=lambda *_args: None,
            elevator_approach_trace_candidates_fn=lambda *_args, **_kwargs: (),
            plan_path_fn=lambda *_args, **_kwargs: None,
            segment_is_traversable_fn=lambda *_args, **_kwargs: True,
        )
        self.assertEqual(context.refresh_open_door_geometry("door", (0, 0, 0)), "running")
        self.assertEqual(
            context.refresh_elevator_geometry("elevator", 0, "door", (0, 0, 0)),
            "idle",
        )


if __name__ == "__main__":
    unittest.main()
