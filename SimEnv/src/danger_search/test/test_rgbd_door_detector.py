import math
import unittest

import numpy as np

from danger_search.rgbd_door_detector import RgbdDoorDetector


class RgbdDoorDetectorTest(unittest.TestCase):
    def setUp(self):
        self.detector = RgbdDoorDetector(
            minimum_pixel_area=500,
            minimum_panel_width=0.65,
            maximum_panel_width=2.2,
            minimum_panel_height=0.8,
            maximum_plane_rmse=0.04,
        )
        self.camera_k = [100.0, 0.0, 80.0, 0.0, 100.0, 60.0, 0.0, 0.0, 1.0]
        # 光学坐标 x 向右、y 向下、z 向前；相机朝 world +X。
        self.camera_to_world = np.asarray(
            [
                [0.0, 0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [0.0, -1.0, 0.0, 1.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

    @staticmethod
    def _door_frame():
        rgb = np.full((120, 160, 3), 210, dtype=np.uint8)
        depth = np.full((120, 160), 4000, dtype=np.uint16)
        rgb[12:112, 50:110] = (46, 46, 50)
        depth[12:112, 50:110] = 2000
        return rgb, depth

    def test_detects_dark_vertical_panel_with_depth_geometry(self):
        rgb, depth = self._door_frame()

        observations = self.detector.detect_arrays(
            rgb,
            depth,
            self.camera_k,
            self.camera_to_world,
            stamp=12.5,
        )

        self.assertEqual(len(observations), 1)
        observation = observations[0]
        self.assertAlmostEqual(observation.world_xy[0], 2.0, places=2)
        self.assertAlmostEqual(observation.world_xy[1], 0.0, places=1)
        self.assertAlmostEqual(abs(observation.outward_yaw), math.pi, places=1)
        self.assertGreater(observation.width, 1.0)
        self.assertGreater(observation.height, 1.5)
        self.assertGreater(observation.depth_inlier_count, 1000)

    def test_gray_wall_does_not_merge_with_black_door_panel(self):
        """灰墙不能与黑门合成整幅超宽轮廓。"""
        rgb = np.full((120, 160, 3), 82, dtype=np.uint8)
        depth = np.full((120, 160), 2000, dtype=np.uint16)
        rgb[12:112, 50:110] = 20

        observations = self.detector.detect_arrays(
            rgb,
            depth,
            self.camera_k,
            self.camera_to_world,
        )

        self.assertEqual(len(observations), 1)
        self.assertGreater(observations[0].width, 1.0)
        self.assertLess(observations[0].width, 1.5)

    def test_rejects_dark_object_without_door_size(self):
        rgb = np.full((120, 160, 3), 210, dtype=np.uint8)
        depth = np.full((120, 160), 4000, dtype=np.uint16)
        rgb[70:100, 65:95] = (40, 40, 44)
        depth[70:100, 65:95] = 2000

        observations = self.detector.detect_arrays(
            rgb,
            depth,
            self.camera_k,
            self.camera_to_world,
        )

        self.assertEqual(observations, [])

    def test_rejects_black_region_without_valid_depth(self):
        rgb, depth = self._door_frame()
        depth[12:112, 50:110] = 0

        observations = self.detector.detect_arrays(
            rgb,
            depth,
            self.camera_k,
            self.camera_to_world,
        )

        self.assertEqual(observations, [])

    def test_opening_change_requires_panel_disappearance_and_deeper_surface(self):
        rgb, depth = self._door_frame()
        baseline = self.detector.detect_arrays(
            rgb,
            depth,
            self.camera_k,
            self.camera_to_world,
        )[0]
        opened_rgb = rgb.copy()
        opened_depth = depth.copy()
        x, y, width, height = baseline.bbox
        opened_rgb[y : y + height, x : x + width] = 180
        opened_depth[y : y + height, x : x + width] = 3500

        change = self.detector.assess_opening_change_arrays(
            baseline,
            opened_rgb,
            opened_depth,
        )

        self.assertTrue(change.confirmed)
        self.assertGreater(change.deeper_fraction, 0.5)

        same_depth = depth.copy()
        no_depth_change = self.detector.assess_opening_change_arrays(
            baseline,
            opened_rgb,
            same_depth,
        )
        self.assertFalse(no_depth_change.confirmed)

    def test_detects_open_door_from_two_dark_side_panels_and_deeper_center(self):
        rgb = np.full((120, 160, 3), 180, dtype=np.uint8)
        depth = np.full((120, 160), 3500, dtype=np.uint16)
        rgb[12:112, 18:55] = (46, 46, 50)
        rgb[12:112, 105:142] = (46, 46, 50)
        depth[12:112, 18:55] = 2000
        depth[12:112, 105:142] = 2000

        observations = self.detector.detect_arrays(
            rgb,
            depth,
            self.camera_k,
            self.camera_to_world,
            stamp=13.0,
        )

        openings = [item for item in observations if item.panel_state == "open"]
        self.assertEqual(len(openings), 1)
        self.assertAlmostEqual(openings[0].world_xy[0], 2.0, places=2)
        self.assertAlmostEqual(openings[0].world_xy[1], 0.0, places=1)
        self.assertGreater(openings[0].width, 0.65)

    def test_registered_door_plane_confirms_opening_without_closed_frame_baseline(self):
        closed_rgb = np.full((120, 160, 3), 210, dtype=np.uint8)
        closed_depth = np.full((120, 160), 4000, dtype=np.uint16)
        closed_rgb[25:95, 45:115] = 20
        closed_depth[25:95, 45:115] = 2000

        closed = self.detector.assess_registered_opening_arrays(
            closed_rgb,
            closed_depth,
            self.camera_k,
            self.camera_to_world,
            door_xy=(2.0, 0.0),
            door_yaw=math.pi,
            door_width=1.4,
        )
        opened = self.detector.assess_registered_opening_arrays(
            np.full((120, 160, 3), 180, dtype=np.uint8),
            np.full((120, 160), 3500, dtype=np.uint16),
            self.camera_k,
            self.camera_to_world,
            door_xy=(2.0, 0.0),
            door_yaw=math.pi,
            door_width=1.4,
        )

        self.assertFalse(closed.confirmed)
        self.assertTrue(opened.confirmed)
        self.assertGreater(opened.deeper_fraction, 0.9)

    def test_registered_door_plane_rejects_missing_depth(self):
        result = self.detector.assess_registered_opening_arrays(
            np.full((120, 160, 3), 180, dtype=np.uint8),
            np.zeros((120, 160), dtype=np.uint16),
            self.camera_k,
            self.camera_to_world,
            door_xy=(2.0, 0.0),
            door_yaw=math.pi,
            door_width=1.4,
        )

        self.assertFalse(result.confirmed)
        self.assertEqual(result.valid_depth_fraction, 0.0)

    def test_registered_door_plane_accepts_sparse_deeper_opening_echoes(self):
        """开门后的稀疏远端回波足够确认，但不能退化为无深度即开门。"""
        rgb = np.full((120, 160, 3), 180, dtype=np.uint8)
        depth = np.zeros((120, 160), dtype=np.uint16)
        # 注册门中央 ROI 为 x=[66, 94)，其中 6/28 列有更深回波，
        # 复现目标层实测约 21.4% 的有效深度占比。
        depth[:, 66:72] = 3500

        result = self.detector.assess_registered_opening_arrays(
            rgb,
            depth,
            self.camera_k,
            self.camera_to_world,
            door_xy=(2.0, 0.0),
            door_yaw=math.pi,
            door_width=1.4,
        )

        self.assertTrue(result.confirmed)
        self.assertAlmostEqual(result.valid_depth_fraction, 6.0 / 28.0)
        self.assertGreater(result.deeper_fraction, 0.9)


if __name__ == "__main__":
    unittest.main()
