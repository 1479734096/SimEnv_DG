"""定位 seed 矩阵 runner 静态契约测试。"""

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "tools" / "sim" / "run_localization_seed_matrix.sh"


def test_matrix_runner_help_does_not_start_ros():
    result = subprocess.run(
        ["bash", str(RUNNER), "--help"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert "20 个预登记 seed" in result.stdout


def test_matrix_runner_freezes_backend_seed_list_and_diagnostics():
    source = RUNNER.read_text(encoding="utf-8")
    assert "adaptive_shared_map" in source
    assert "SEEDS=(8127 1612 8072 6946 4962" in source
    assert "ELEVATOR_TEST_LOCALIZATION_DIAGNOSTICS=1" in source
    assert "ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=1" in source
    assert "ELEVATOR_TEST_RECORD_RGBD=0" in source
    assert "ELEVATOR_TEST_FULL_CYCLE=1" in source
    assert "ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=5" in source


def test_matrix_runner_stops_on_interrupt_or_global_preflight_failure():
    source = RUNNER.read_text(encoding="utf-8")
    assert '[[ "$runner_status" == "130" || "$runner_status" == "2" ]]' in source
    assert 'exit "$runner_status"' in source
    assert '"$prior_status" != "130" && "$prior_status" != "2"' in source
