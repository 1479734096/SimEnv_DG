"""离线工具与正式运行闭包隔离测试。"""

import importlib.util
from pathlib import Path
import tempfile
import unittest


SIMENV_ROOT = Path(__file__).resolve().parents[3]
AUDIT_PATH = SIMENV_ROOT / "tools" / "offline" / "check_production_boundary.py"


def _load_audit_module():
    spec = importlib.util.spec_from_file_location("offline_boundary_audit", AUDIT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载离线边界审计脚本: {AUDIT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class OfflineIsolationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.audit = _load_audit_module()

    def test_current_workspace_production_boundary_is_closed(self):
        self.assertEqual(self.audit.audit_workspace(SIMENV_ROOT), [])

    def test_install_parser_detects_extra_offline_entry(self):
        cmake = """
        catkin_install_python(PROGRAMS
          scripts/danger_search_node
          tools/offline/evaluate.py
          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        """
        self.assertEqual(
            self.audit.installed_python_programs(cmake),
            ("scripts/danger_search_node", "tools/offline/evaluate.py"),
        )

    def test_import_parser_exposes_offline_dependency(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad_module.py"
            path.write_text(
                "from tools.offline import evaluator\nimport building_obstacles.scorer\n",
                encoding="utf-8",
            )
            self.assertEqual(
                self.audit.imported_modules(path),
                ("tools.offline", "building_obstacles.scorer"),
            )


if __name__ == "__main__":
    unittest.main()
