"""三层完整闭环 seed 矩阵离线汇总测试。"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "offline" / "summarize_three_floor_matrix.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("three_floor_matrix_summary", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载离线工具：{SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MODULE = _load_module()


def _write_run(
    root: Path,
    seed: int,
    *,
    attempt: int = 1,
    attempt_class: str = "success",
    score: float = 30.0,
    safety: dict[str, int] | None = None,
) -> Path:
    run_dir = root / f"seed_{seed}" / f"attempt_{attempt}"
    run_dir.mkdir(parents=True)
    (run_dir / "ATTEMPT_CLASS.txt").write_text(attempt_class + "\n", encoding="utf-8")
    (run_dir / "RUNNER_EXIT_CODE.txt").write_text(
        "0\n" if attempt_class == "success" else "1\n", encoding="utf-8"
    )
    (run_dir / "RUN_METADATA.txt").write_text(
        f"seed={seed}\nattempt={attempt}\nrunner_exit_code=0\n"
        f"attempt_class={attempt_class}\nwall_elapsed_seconds=100\n",
        encoding="utf-8",
    )
    (run_dir / "TEST_SUMMARY.txt").write_text(
        f"result={'通过' if attempt_class == 'success' else '失败'}\n"
        f"detail=合成证据\nseed={seed}\nfloor_count=3\nfull_cycle=1\n",
        encoding="utf-8",
    )
    outcome = "success" if attempt_class == "success" else "failed"
    (run_dir / "mission_timing.json").write_text(
        json.dumps(
            {
                "run_outcome": {
                    "category": "任务成功" if outcome == "success" else "电梯或换层失败",
                    "terminal_outcome": outcome,
                    "reason": "return_home_completed" if outcome == "success" else "floor_change_failed",
                },
                "sampled_duration": 500.0,
                "trajectory_distance": 60.0,
                "effective_motion_ratio": 0.5,
                "maximum_coverage_ratio": 0.7,
                "stuck_episode_count": 0,
                "recovery_episode_count": 0,
                "safety_event_counts": safety or {},
                "excluded_trajectory_segments": {},
            }
        ),
        encoding="utf-8",
    )
    (run_dir / "evaluation_result.json").write_text(
        json.dumps(
            {
                "metrics": {
                    "truth_count": 5,
                    "detected_count": 5,
                    "correct": 4,
                    "missed": 1,
                    "false_alarms": 1,
                },
                "scores": {"technical_objective_total": score},
            }
        ),
        encoding="utf-8",
    )
    for name in ("detected_danger.json", "danger_truth.json"):
        (run_dir / name).write_text("{}\n", encoding="utf-8")
    (run_dir / "result_files.sha256").write_text("合成哈希\n", encoding="utf-8")
    return run_dir


def test_selects_formal_result_instead_of_later_infrastructure_attempt():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        finalized = _write_run(root, 8127, attempt=1)
        later = root / "seed_8127" / "attempt_2"
        later.mkdir()
        (later / "ATTEMPT_CLASS.txt").write_text(
            "infrastructure_failure\n", encoding="utf-8"
        )

        row = MODULE.analyze_seed(root, 8127)

    assert row["run_dir"] == str(finalized.resolve())
    assert row["attempt_class"] == "success"


def test_summary_reports_effect_scores_and_structured_issues():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        _write_run(root, 8127, score=31.0, safety={"ground_support_hold": 2})
        _write_run(root, 6946, attempt_class="algorithm_failure", score=25.0)
        rows = [MODULE.analyze_seed(root, seed) for seed in (8127, 6946)]

        result = MODULE.summarize(rows, 2)

    assert result["matrix_complete"] is True
    assert result["success_count"] == 1
    assert result["algorithm_failure_count"] == 1
    assert result["average_objective_score"] == pytest.approx(28.0)
    assert result["average_recall"] == pytest.approx(0.8)
    assert result["safety_event_totals"] == {"ground_support_hold": 2}
    assert "算法失败" in rows[1]["issues"][0]


def test_missing_scoring_sidecars_keeps_matrix_incomplete():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        run_dir = _write_run(root, 8127)
        (run_dir / "evaluation_result.json").unlink()
        row = MODULE.analyze_seed(root, 8127)
        result = MODULE.summarize([row], 1)

    assert row["evidence_complete"] is False
    assert any("evaluation_result.json" in issue for issue in row["issues"])
    assert result["matrix_complete"] is False


def test_markdown_contains_process_effect_and_problem_sections():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        _write_run(root, 8127)
        _write_run(root, 6946, attempt_class="algorithm_failure")
        result = MODULE.summarize(
            [MODULE.analyze_seed(root, seed) for seed in (8127, 6946)], 2
        )
        path = root / "matrix_summary.md"

        MODULE.write_markdown(
            path,
            result,
            {"head": "a" * 40, "seeds": "8127 6946"},
        )
        text = path.read_text(encoding="utf-8")

    assert "## 整体效果" in text
    assert "## 逐轮经过" in text
    assert "## 出现的问题" in text
    assert "seed `6946`" in text
