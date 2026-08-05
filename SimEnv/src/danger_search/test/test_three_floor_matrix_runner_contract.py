"""三层完整闭环 seed 矩阵 runner 的静态契约测试。"""

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "tools" / "sim" / "run_three_floor_seed_matrix.sh"


def test_help_does_not_start_ros_and_documents_fixed_formal_budget():
    result = subprocess.run(
        ["bash", str(RUNNER), "--help"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert "3 层" in result.stdout
    assert "840 s 总仿真预算" in result.stdout
    assert "任务结束后才归档并读取裁判真值评分" in result.stdout


def test_runner_requires_multiple_preregistered_unique_seeds_before_preflight():
    one_seed = subprocess.run(
        ["bash", str(RUNNER), "8127"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    duplicate = subprocess.run(
        ["bash", str(RUNNER), "8127", "8127"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert one_seed.returncode == 2
    assert "2～20 个 seed" in one_seed.stderr
    assert duplicate.returncode == 2
    assert "seed 不得重复" in duplicate.stderr


def test_runner_freezes_formal_chain_and_post_run_truth_boundary():
    source = RUNNER.read_text(encoding="utf-8")

    assert 'GUI=true' in source
    assert 'ELEVATOR_TEST_FLOOR_COUNT=3' in source
    assert 'ELEVATOR_TEST_FLOOR_TIMEOUT=280' in source
    assert 'ELEVATOR_TEST_MIN_FLOOR_EXPLORE_TIME=60' in source
    assert 'ELEVATOR_TEST_MAX_RUNTIME=840' in source
    assert 'ELEVATOR_TEST_FULL_CYCLE=1' in source
    assert 'ELEVATOR_TEST_ENABLE_OFFLINE_TRUTH=0' in source
    assert 'ELEVATOR_TEST_LOCALIZATION_BACKEND=adaptive_shared_map' in source
    assert '"$RUNNER" 2>&1 | tee "$console_log"' in source
    assert 'archive_if_newer "$WORKSPACE_ROOT/results/danger_truth.json"' in source


def test_runner_does_not_retry_algorithm_failures_but_bounds_infrastructure_retries():
    source = RUNNER.read_text(encoding="utf-8")

    assert "MAX_INFRASTRUCTURE_ATTEMPTS=2" in source
    assert "algorithm_failure)" in source
    assert "记录为算法失败，不重跑" in source
    assert "连续两次基础设施失败，矩阵暂停" in source
    assert '[[ "$runner_status" == "130" ]]' in source
    assert "trap handle_signal INT TERM" in source
    assert 'runner_status=130' in source
