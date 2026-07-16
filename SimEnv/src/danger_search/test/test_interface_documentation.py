"""比赛接口文档一致性测试。"""
from pathlib import Path
import unittest


SIMENV_ROOT = Path(__file__).resolve().parents[3]
REFERENCE_PATH = SIMENV_ROOT / "docs" / "reference.md"
DOCUMENT_PATHS = (
    SIMENV_ROOT / "README.md",
    SIMENV_ROOT / "docs" / "algorithm-interfaces.md",
    REFERENCE_PATH,
)
TRUTH_RESTRICTION = (
    "`/Odometry_gazebo`、`/ground_truth/base_w`、`/ground_truth/base_trunk` 和 "
    "`/ground_truth/*_foot` 是 Gazebo 真值通道，不作为正式比赛算法输入。"
    "即使本地调试时能够看到这些话题，参赛算法也不得订阅或读取。"
)


class InterfaceDocumentationTest(unittest.TestCase):
    def test_truth_restriction_is_listed_in_all_interface_documents(self):
        for path in DOCUMENT_PATHS:
            with self.subTest(path=path):
                content = path.read_text(encoding="utf-8")
                self.assertIn(TRUTH_RESTRICTION, content)

    def test_odometry_is_not_documented_as_an_algorithm_interface(self):
        for path in DOCUMENT_PATHS:
            with self.subTest(path=path):
                content = path.read_text(encoding="utf-8")
                self.assertNotIn("| `/Odometry_gazebo` |", content)

    def test_reference_does_not_document_nonexistent_odom_switch(self):
        content = REFERENCE_PATH.read_text(encoding="utf-8")
        self.assertNotIn("ENABLE_REFEREE_ODOM", content)


if __name__ == "__main__":
    unittest.main()
