from __future__ import annotations

import json
import unittest
from types import SimpleNamespace
from unittest.mock import ANY, patch

from building_generator_classic.control_server import (
    _handle_call_elevator,
    _passenger_is_inside_elevator,
)


class ElevatorPassengerTest(unittest.TestCase):
    def test_detects_passenger_inside_rotated_elevator(self) -> None:
        elevator_pose = [2.76, 2.6, 1.03, 0.0, 0.0, -1.5707963267948966]
        car_size = [1.9, 2.1, 2.3]
        passenger_pose = SimpleNamespace(
            position=SimpleNamespace(x=2.27, y=2.61, z=0.32),
        )

        self.assertTrue(
            _passenger_is_inside_elevator(passenger_pose, elevator_pose, car_size)
        )

    def test_rejects_robot_waiting_outside_elevator(self) -> None:
        elevator_pose = [2.76, 2.6, 1.03, 0.0, 0.0, -1.5707963267948966]
        car_size = [1.9, 2.1, 2.3]
        passenger_pose = SimpleNamespace(
            position=SimpleNamespace(x=0.85, y=2.6, z=0.32),
        )

        self.assertFalse(
            _passenger_is_inside_elevator(passenger_pose, elevator_pose, car_size)
        )

    def test_call_elevator_moves_passenger_by_same_height(self) -> None:
        source_pose = [2.76, 2.6, 1.03, 0.0, 0.0, -1.5707963267948966]
        target_pose = [2.76, 2.6, 3.63, 0.0, 0.0, -1.5707963267948966]
        runtime = SimpleNamespace(
            call_elevator=lambda elevator_id, target_floor, open_doors: {
                "accepted": True,
                "current_floor": target_floor,
                "state": "idle",
                "message": "ok",
                "model_name": "dynamic_elevator_main",
                "source_pose": source_pose,
                "target_pose": target_pose,
                "car_size": [1.9, 2.1, 2.3],
            }
        )
        passenger_state = SimpleNamespace(
            success=True,
            pose=SimpleNamespace(
                position=SimpleNamespace(x=2.27, y=2.61, z=0.32),
            ),
            twist=SimpleNamespace(),
        )
        request = SimpleNamespace(
            elevator_id="elevator_main",
            target_floor=1,
            open_doors=False,
        )

        with patch(
            "building_generator_classic.control_server._apply_model_pose"
        ) as apply_elevator, patch(
            "building_generator_classic.control_server._apply_model_z_offset"
        ) as apply_passenger, patch("rospy.loginfo") as loginfo:
            response = _handle_call_elevator(
                runtime,
                request,
                lambda **values: SimpleNamespace(**values),
                set_model_state=object(),
                get_model_state=lambda model_name, frame: passenger_state,
            )

        self.assertTrue(response.accepted)
        apply_elevator.assert_called_once_with(
            ANY,
            "dynamic_elevator_main",
            target_pose,
        )
        apply_passenger.assert_called_once()
        passenger_args = apply_passenger.call_args.args
        self.assertEqual(passenger_args[:3], (ANY, "a1_gazebo", passenger_state))
        self.assertAlmostEqual(passenger_args[3], 2.6)
        diagnostic = json.loads(loginfo.call_args_list[-1].args[1])
        self.assertEqual(diagnostic["event"], "elevator_transport")
        self.assertEqual(diagnostic["target_floor"], 1)
        self.assertEqual(diagnostic["source_z"], 1.03)
        self.assertEqual(diagnostic["target_z"], 3.63)
        self.assertTrue(diagnostic["passenger_inside_elevator"])
        self.assertTrue(diagnostic["transport_applied"])
        self.assertAlmostEqual(diagnostic["passenger_requested_after_z"], 2.92)


if __name__ == "__main__":
    unittest.main()
