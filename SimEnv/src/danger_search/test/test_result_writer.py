import json
from pathlib import Path
import tempfile
import unittest

from danger_search.result_writer import ResultWriter


class ResultWriterTest(unittest.TestCase):
    def test_writes_evaluation_schema(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            result_file = Path(temp_dir) / "detected_danger.json"
            ResultWriter(result_file).write(12.345, [[1.23456, 2.0, 0.15]])

            data = json.loads(result_file.read_text(encoding="utf-8"))
            self.assertEqual(data["exploration_time"], 12.35)
            self.assertEqual(data["detected_danger_sources"][0]["position"], [1.235, 2.0, 0.15])
            self.assertFalse((Path(temp_dir) / ".detected_danger.json.tmp").exists())

    def test_rejects_invalid_exploration_time(self):
        invalid_values = (-1.0, float("nan"), float("inf"), True, "12.3")
        with tempfile.TemporaryDirectory() as temp_dir:
            writer = ResultWriter(Path(temp_dir) / "detected_danger.json")
            for value in invalid_values:
                with self.subTest(value=value), self.assertRaises(ValueError):
                    writer.write(value, [])

    def test_rejects_invalid_coordinates_without_overwriting_result(self):
        invalid_positions = (
            [[float("nan"), 0.0, 0.0]],
            [[float("inf"), 0.0, 0.0]],
            [[True, 0.0, 0.0]],
            [["1.0", 0.0, 0.0]],
            [[1.0, 2.0]],
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            result_file = Path(temp_dir) / "detected_danger.json"
            result_file.write_text("原结果\n", encoding="utf-8")
            writer = ResultWriter(result_file)
            for positions in invalid_positions:
                with self.subTest(positions=positions), self.assertRaises(ValueError):
                    writer.write(1.0, positions)
                self.assertEqual(result_file.read_text(encoding="utf-8"), "原结果\n")
                self.assertFalse((Path(temp_dir) / ".detected_danger.json.tmp").exists())


if __name__ == "__main__":
    unittest.main()
