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


if __name__ == "__main__":
    unittest.main()
