import math
import unittest
from types import SimpleNamespace

from danger_search.elevator_registration_runtime import (
    ElevatorRegistrationConfig,
    ElevatorRegistrationRuntime,
)
from danger_search.navigator import IDLE
from danger_search.rgbd_door_detector import DoorPanelObservation


def _observation(stamp: float, x: float = 2.0) -> DoorPanelObservation:
    return DoorPanelObservation(
        stamp=stamp,
        distance=1.0,
        world_xy=(x, 2.0),
        outward_yaw=math.pi,
        width=0.8,
        height=1.8,
        plane_rmse=0.01,
        depth_inlier_count=400,
        bbox=(0, 0, 10, 10),
        median_depth=1.0,
        center_dark_fraction=0.9,
    )


class ElevatorRegistrationRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.runtime = ElevatorRegistrationRuntime()
        self.feature = SimpleNamespace(
            elevator_id="lift",
            served_floors=frozenset({0, 1}),
        )
        self.features = SimpleNamespace(elevator_features={"lift": self.feature})
        self.client = SimpleNamespace(
            elevators={},
            doors={},
            elevator_door_id=lambda floor: f"lift_{floor}",
        )
        self.config = ElevatorRegistrationConfig(
            visual_min_width=0.65,
            visual_max_width=1.8,
            visual_max_distance=4.0,
            visual_confirmations=3,
            visual_confirmation_max_gap=0.4,
            geometry_tolerance=0.25,
            door_pass_offset=0.8,
            livox_enabled=False,
            refresh_timeout=1.0,
            refresh_max_shift=0.75,
            refresh_max_yaw_change=0.35,
        )

    def register_frame(self, stamp: float):
        self.runtime.latest_rgbd_door_stamp = stamp
        self.runtime.latest_rgbd_door_observations = [_observation(stamp)]
        return self.runtime.try_register(
            pose=(0.0, 0.0, 0.0),
            current_floor=0,
            max_floor=2,
            current_task_present=False,
            building_features=self.features,
            building_client=self.client,
            local_detector=SimpleNamespace(),
            cloud_stamp=None,
            cloud_points=(),
            floor_height=2.6,
            entrance_xy=None,
            config=self.config,
        )

    def test_three_consistent_rgbd_frames_register_public_geometry(self):
        self.assertFalse(self.register_frame(1.0).registered)
        self.assertFalse(self.register_frame(1.2).registered)
        result = self.register_frame(1.4)

        self.assertTrue(result.registered)
        self.assertEqual(result.active_elevator_id, "lift")
        self.assertEqual(set(self.client.elevators["lift"].lobby_poses), {0, 1})
        self.assertEqual(self.client.doors["lift_0"].kind, "elevator")
        self.assertEqual(self.client.doors["lift_1"].kind, "elevator")

    def test_refresh_timeout_preserves_registered_geometry(self):
        self.register_frame(1.0)
        self.register_frame(1.2)
        self.register_frame(1.4)
        original_x = self.client.doors["lift_0"].x

        first = self.runtime.refresh_geometry(
            now=10.0,
            elevator_id="lift",
            floor_index=0,
            door_id="lift_0",
            pose=(0.0, 0.0, 0.0),
            building_features=self.features,
            building_client=self.client,
            config=self.config,
        )
        final = self.runtime.refresh_geometry(
            now=11.1,
            elevator_id="lift",
            floor_index=0,
            door_id="lift_0",
            pose=(0.0, 0.0, 0.0),
            building_features=self.features,
            building_client=self.client,
            config=self.config,
        )

        self.assertEqual(first.status, "running")
        self.assertEqual(final.status, IDLE)
        self.assertEqual(self.client.doors["lift_0"].x, original_x)


if __name__ == "__main__":
    unittest.main()
