import math
import unittest
from types import SimpleNamespace

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
        # 光学坐标 x 向右、y 向下、z 向前；相机朝 world +X，world +Y 向左。
        self.camera_to_world = np.asarray(
            [
                [0.0, 0.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, -1.0, 0.0, 1.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            RgbdDoorDetector(minimum_depth=2.0, maximum_depth=1.0)
        with self.assertRaises(ValueError):
            RgbdDoorDetector(maximum_plane_rmse=math.nan)

    def test_invalid_array_intrinsics_and_transform_fail_closed(self):
        rgb, depth = self._door_frame()
        self.assertEqual(
            self.detector.detect_arrays(
                rgb,
                depth[:10],
                self.camera_k,
                self.camera_to_world,
            ),
            [],
        )
        self.assertEqual(
            self.detector.detect_arrays(
                rgb,
                depth,
                [0.0, 0.0, 80.0, 0.0, 100.0, 60.0],
                self.camera_to_world,
            ),
            [],
        )
        invalid_transform = self.camera_to_world.copy()
        invalid_transform[0, 0] = math.nan
        self.assertEqual(
            self.detector.detect_arrays(
                rgb,
                depth,
                self.camera_k,
                invalid_transform,
            ),
            [],
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

    def test_oblique_panel_keeps_full_width_before_plane_residual_filter(self):
        """斜视门板的光轴深度梯度不能被误当成多个表面裁掉。"""
        rgb = np.full((120, 160, 3), 210, dtype=np.uint8)
        depth = np.full((120, 160), 4.0, dtype=np.float32)
        rgb[12:112, 55:105] = (46, 46, 50)
        angle = math.radians(41.0)
        normal_x = math.sin(angle)
        normal_z = math.cos(angle)
        plane_distance = 2.2 * normal_z
        for column in range(55, 105):
            ray_x = (column - self.camera_k[2]) / self.camera_k[0]
            depth[12:112, column] = plane_distance / (
                normal_z + normal_x * ray_x
            )

        diagnostics = {}
        observations = self.detector.detect_arrays(
            rgb,
            depth,
            self.camera_k,
            self.camera_to_world,
            diagnostics=diagnostics,
        )

        self.assertEqual(len(observations), 1)
        self.assertGreater(observations[0].width, 0.65)
        self.assertLess(observations[0].plane_rmse, 1e-4)
        self.assertEqual(diagnostics["passed_plane_candidate_pixels"], 1)

    def test_two_depth_surfaces_are_not_merged_into_one_door_panel(self):
        """同一深色轮廓内的前后表面不能合成一块虚假宽门板。"""
        rgb = np.full((120, 160, 3), 210, dtype=np.uint8)
        depth = np.full((120, 160), 4.0, dtype=np.float32)
        rgb[12:112, 40:120] = (46, 46, 50)
        depth[12:112, 40:80] = 2.0
        depth[12:112, 80:120] = 3.0

        observations = self.detector.detect_arrays(
            rgb,
            depth,
            self.camera_k,
            self.camera_to_world,
        )

        self.assertEqual(observations, [])

    def test_message_path_exposes_read_only_rejection_diagnostics(self):
        rgb, depth = self._door_frame()
        self.detector._bridge = SimpleNamespace(
            imgmsg_to_cv2=lambda message, desired_encoding: (
                rgb if desired_encoding == "bgr8" else depth
            )
        )
        stamp = SimpleNamespace(to_sec=lambda: 12.5)
        message = SimpleNamespace(header=SimpleNamespace(stamp=stamp))
        camera_info = SimpleNamespace(K=self.camera_k)

        observations = self.detector.detect_messages(
            message,
            message,
            camera_info,
            self.camera_to_world,
        )
        snapshot = self.detector.diagnostics()

        self.assertEqual(len(observations), 1)
        self.assertEqual(snapshot["frames"], 1)
        self.assertEqual(snapshot["last_observation_count"], 1)
        self.assertEqual(snapshot["last"]["accepted_panel"], 1)
        self.assertEqual(snapshot["cumulative"]["accepted_panel"], 1)
        self.assertEqual(
            snapshot["cumulative"]["panel_width_within_limits"], 1
        )
        self.assertEqual(len(snapshot["recent_panel_width_samples"]), 1)
        self.assertEqual(
            snapshot["recent_panel_width_samples"][0]["relation"],
            "within_limits",
        )
        sample = snapshot["recent_panel_width_samples"][0]
        self.assertGreater(sample["pixel_area"], 0.0)
        self.assertGreater(sample["valid_depth_count"], 0)
        self.assertGreater(sample["inlier_count"], 0)
        self.assertEqual(len(sample["bbox"]), 4)
        self.assertEqual(len(sample["world_xy"]), 2)
        self.assertTrue(math.isfinite(sample["relative_bearing"]))
        snapshot["last"]["accepted_panel"] = 99
        snapshot["recent_panel_width_samples"][0]["width"] = 99.0
        self.assertEqual(self.detector.diagnostics()["last"]["accepted_panel"], 1)
        self.assertNotEqual(
            self.detector.diagnostics()["recent_panel_width_samples"][0]["width"],
            99.0,
        )

    def test_message_path_records_missing_transform_without_fallback(self):
        self.detector._bridge = SimpleNamespace()

        self.assertEqual(
            self.detector.detect_messages(None, None, None, None),
            [],
        )

        snapshot = self.detector.diagnostics()
        self.assertEqual(snapshot["frames"], 1)
        self.assertEqual(snapshot["last"], {"missing_world_transform": 1})

        self.detector.reset_diagnostics()
        self.assertEqual(
            self.detector.diagnostics(),
            {
                "frames": 0,
                "last": {},
                "cumulative": {},
                "last_observation_count": 0,
                "recent_panel_width_samples": [],
            },
        )

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

    def test_registered_door_plane_accepts_dense_depth_with_partial_deeper_echoes(self):
        """有效深度完整且暗像素很少时，部分远端回波也能确认开口。"""
        rgb = np.full((120, 160, 3), 180, dtype=np.uint8)
        depth = np.full((120, 160), 2000, dtype=np.uint16)
        # 注册门中央 ROI 为 x=[66, 94)，其中 5/28 列命中轿厢深处，
        # 对应 Gazebo 返程电梯门实测约 0.17 的 deeper_fraction。
        depth[:, 66:71] = 3500

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
        self.assertAlmostEqual(result.valid_depth_fraction, 1.0)
        self.assertGreater(result.deeper_fraction, 0.15)


if __name__ == "__main__":
    unittest.main()
