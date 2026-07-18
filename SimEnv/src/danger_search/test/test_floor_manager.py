#!/usr/bin/env python3
"""旧 FloorManager 的接口策略回归测试。"""
import unittest
from unittest.mock import patch

from danger_search.floor_manager import FloorManager


class FloorManagerTest(unittest.TestCase):
    def test_service_names_pass_through_interface_policy(self):
        checked = []

        def require(service, *, purpose):
            checked.append((service, purpose))
            return service

        with patch("danger_search.floor_manager.require_allowed_service", require):
            manager = FloorManager()

        self.assertEqual(
            [service for service, _purpose in checked],
            ["/set_door_state", "/call_elevator"],
        )
        self.assertEqual(manager._set_door_service, "/set_door_state")
        self.assertEqual(manager._call_elevator_service, "/call_elevator")


if __name__ == "__main__":
    unittest.main()
