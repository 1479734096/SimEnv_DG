import unittest
from types import SimpleNamespace

from danger_search.elevator_observation_runtime import (
    ElevatorObservationRuntime,
    ElevatorVisualScanConfig,
)


class ElevatorObservationRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.runtime = ElevatorObservationRuntime()
        self.runtime.visual_prepare_task = SimpleNamespace(
            done=lambda: True,
            ok=lambda: True,
        )
        elevator = SimpleNamespace(elevator_id="lift")
        entrance = SimpleNamespace(door_id="main", kind="main_entrance")
        self.features = SimpleNamespace(
            elevator_features={"lift": elevator},
            doors={"main": entrance},
        )
        self.client = SimpleNamespace(elevators={})
        self.detector = SimpleNamespace(
            diagnostics=lambda: {"frames": 4},
            reset_diagnostics=lambda: None,
        )
        self.calls = []
        self.controller = SimpleNamespace(
            clear_path=lambda reason=None: self.calls.append(("clear", reason)),
            rotate_in_place=lambda speed: self.calls.append(("rotate", speed)),
            stop=lambda reason=None: self.calls.append(("stop", reason)),
        )
        self.config = ElevatorVisualScanConfig(18.0, 0.7, 5.8)

    def tick(self, now, pose):
        return self.runtime.tick_visual_scan(
            now=now,
            pose=pose,
            current_floor=0,
            current_task_present=False,
            building_features=self.features,
            building_client=self.client,
            rgbd_detector=self.detector,
            controller=self.controller,
            last_door_id="main",
            last_door_xy=(0.0, 0.0),
            post_door_context_satisfied=True,
            config=self.config,
        )

    def test_scan_keeps_observing_until_timeout(self):
        first = self.tick(10.0, (0.0, 2.0, 0.0))
        final = self.tick(28.1, (0.0, 2.0, 1.0))

        self.assertTrue(first.handled)
        self.assertTrue(first.clear_exploration_target)
        self.assertFalse(final.handled)
        self.assertTrue(self.runtime.visual_scan_completed)
        self.assertEqual(sum(name == "clear" for name, _ in self.calls), 1)
        self.assertEqual(final.structured_event["result"], "landmark_not_found")
        self.assertEqual(final.structured_event["detector"], {"frames": 4})

    def test_registered_landmark_finishes_without_restarting_scan(self):
        self.tick(10.0, (0.0, 2.0, 0.0))
        self.client.elevators["lift"] = SimpleNamespace(lobby_poses={0: (1, 2, 3)})

        result = self.tick(11.0, (0.0, 2.0, 0.1))

        self.assertFalse(result.handled)
        self.assertTrue(self.runtime.visual_scan_completed)
        self.assertEqual(result.structured_event["result"], "landmark_found")
        self.assertIn(("stop", "elevator_visual_landmark_found"), self.calls)

    def test_visual_confirmation_does_not_require_livox_dependencies(self):
        self.runtime.door_open_confirmation = {
            "door_id": "lift_0",
            "baseline": None,
            "last_image_stamp": 2.0,
            "visual_confirmations": 2,
            "last_visual_change": None,
            "last_cloud_stamp": None,
            "livox_confirmations": 0,
            "logged": False,
        }
        client = SimpleNamespace(doors={})
        local_detector = SimpleNamespace(
            detect_opening_ahead=lambda **_kwargs: self.fail(
                "RGB-D 已确认时不得读取 Livox 依赖"
            )
        )

        result = self.runtime.confirm_door_open(
            door_id="lift_0",
            pose=(0.0, 0.0, 0.0),
            cloud_stamp=3.0,
            cloud_points=(),
            floor_index=0,
            floor_height=None,
            floor_transition_explicit=True,
            geometry_tolerance=0.25,
            building_client=client,
            local_detector=local_detector,
        )

        self.assertTrue(result.confirmed)
        self.assertEqual(
            result.structured_event["event"],
            "elevator_door_open_sensor_confirmed",
        )


if __name__ == "__main__":
    unittest.main()
