"""命令仲裁等价测试（R1）。

`_publish_locked` 内联了一条五级零速归因优先级链（请求消毒 → 发布失败锁存 →
安全门过滤 → 门输出校验 → 零速归因解析）。R1 把这条链提取为纯函数
`arbitrate_command`，本文件用表驱动固化「提取前」的真实行为，并要求
提取后的纯函数与之逐条等价。

固化数据来自 `data/command_arbitration_equivalence.json`（396 个场景，
9 种请求 × 11 种门 × 2 种调用方 reason × 2 种锁存状态），由重构前的
`_publish_locked` 实测生成。数据文件与本测试一起进版本库，任何行为漂移
都会表现为逐场景 diff，而不是一个笼统的哈希不匹配。

覆盖的三条易被静默破坏的语义（均由表格实测确认，非推断）：

1. 归因遮蔽：非法请求（nan/inf/类型错误）叠加畸形门输出时，最终归因是
   `invalid_command_gate_output` 而非 `invalid_requested_command`。门是用
   消毒后的 (0,0,0) 调用的，其畸形输出覆盖了更早的非法请求归因。
2. epsilon 边界含等号：请求 1e-4 会被原样发出（不归零），但因
   `abs(v) <= 1e-4` 仍被归因为 `zero_command`。
3. 门被错误归责：把 1e-4 减半到 5e-5 的门并未归零，但因结果跌破 epsilon
   且 `emitted != requested`，归因变成 `ground_support_gate:<code>`。
"""
import itertools
import json
import unittest
from pathlib import Path

from danger_search.navigator import (
    PathFollower,
    arbitrate_command,
    sanitize_command_request,
)

DATA_PATH = (
    Path(__file__).resolve().parent / "data" / "command_arbitration_equivalence.json"
)

# 与生成 fixture 时完全一致的输入构造；顺序无关，按 (request, gate,
# zero_reason, latched) 四元组精确匹配。
REQUESTS = {
    "nonzero": (0.4, 0.0, 0.1),
    "zero": (0.0, 0.0, 0.0),
    "nonfinite": (float("nan"), 0.0, 0.0),
    "inf": (float("inf"), 0.0, 0.0),
    "typeerr": ("x", 0.0, 0.0),
    "noneval": (None, 0.0, 0.0),
    "eps_at": (1e-4, 0.0, 0.0),
    "eps_above": (1.1e-4, 0.0, 0.0),
    "lateral_only": (0.0, 0.3, 0.0),
}

GATE_KINDS = (
    "none",
    "pass",
    "zero_rc",
    "zero_norc",
    "raise",
    "short",
    "nan",
    "badtype",
    "nofilter",
    "noncallable",
    "halve",
)


def _make_gate(kind: str):
    """构造替身安全门；`none` 表示未安装门。"""
    if kind == "none":
        return None

    class _Gate:
        reason_code = None

    gate = _Gate()
    if kind == "pass":
        gate.filter_command = lambda command, **_kwargs: tuple(command)
    elif kind == "zero_rc":
        gate.reason_code = "safety_blocked"
        gate.filter_command = lambda command, **_kwargs: (0.0, 0.0, 0.0)
    elif kind == "zero_norc":
        gate.filter_command = lambda command, **_kwargs: (0.0, 0.0, 0.0)
    elif kind == "raise":
        gate.reason_code = "broken"

        def _raise(command, **_kwargs):
            raise RuntimeError("测试异常")

        gate.filter_command = _raise
    elif kind == "short":
        gate.reason_code = "shorty"
        gate.filter_command = lambda command, **_kwargs: (0.0, 0.0)
    elif kind == "nan":
        gate.reason_code = "nanny"
        gate.filter_command = lambda command, **_kwargs: (float("nan"), 0.0, 0.0)
    elif kind == "badtype":
        gate.reason_code = "badtype"
        gate.filter_command = lambda command, **_kwargs: "abc"
    elif kind == "nofilter":
        pass
    elif kind == "noncallable":
        gate.filter_command = 123
    elif kind == "halve":
        gate.reason_code = "halved"
        gate.filter_command = lambda command, **_kwargs: tuple(
            value / 2.0 for value in command
        )
    else:  # pragma: no cover - 防御：新增门类型必须显式登记
        raise AssertionError(f"未登记的门类型: {kind}")
    return gate


def _load_expected() -> dict:
    with DATA_PATH.open(encoding="utf-8") as handle:
        payload = json.load(handle)
    table = {}
    for row in payload["scenarios"]:
        key = (row["request"], row["gate"], row["zero_reason"], row["latched"])
        table[key] = (
            tuple(row["requested"]),
            tuple(row["emitted"]),
            row["resolved"],
        )
    return table


EXPECTED = _load_expected()


class ArbitrationFixtureIntegrityTest(unittest.TestCase):
    """先验证固化数据本身完整，避免等价测试空洞通过。"""

    def test_fixture_covers_full_cross_product(self):
        expected_keys = {
            (request, gate, zero_reason, latched)
            for request, gate, zero_reason, latched in itertools.product(
                REQUESTS, GATE_KINDS, (None, "caller_reason"), (False, True)
            )
        }
        self.assertEqual(set(EXPECTED), expected_keys)
        self.assertEqual(len(EXPECTED), 396)

    def test_fixture_contains_the_three_shadowed_semantics(self):
        # 1. 归因遮蔽：非法请求 + 畸形门 -> 门输出归因胜出
        self.assertEqual(
            EXPECTED[("nonfinite", "short", None, False)][2],
            "invalid_command_gate_output",
        )
        # 无门时才暴露非法请求归因
        self.assertEqual(
            EXPECTED[("nonfinite", "none", None, False)][2],
            "invalid_requested_command",
        )
        # 2. epsilon 含等号：1e-4 原样发出但归因 zero_command
        requested, emitted, resolved = EXPECTED[("eps_at", "none", None, False)]
        self.assertEqual(emitted, requested)
        self.assertEqual(resolved, "zero_command")
        # 3. 门未归零却被归责：1e-4 -> 5e-5
        _, emitted, resolved = EXPECTED[("eps_at", "halve", None, False)]
        self.assertEqual(emitted, (5e-05, 0.0, 0.0))
        self.assertEqual(resolved, "ground_support_gate:halved")


class PublishLockedMatchesFixtureTest(unittest.TestCase):
    """`_publish_locked` 的实际行为必须逐场景等于固化数据。

    重构前证明 fixture 准确；重构后证明行为未漂移。同一份断言在
    提取纯函数前后都必须通过。
    """

    def test_all_scenarios_match(self):
        mismatches = []
        for key, expected in sorted(EXPECTED.items(), key=lambda item: str(item[0])):
            request, gate_kind, zero_reason, latched = key
            linear_x, linear_y, angular_z = REQUESTS[request]
            follower = PathFollower(cmd_vel_topic="/cmd_vel")
            gate = _make_gate(gate_kind)
            if gate is not None:
                follower.set_command_gate(gate)
            if latched:
                follower._publish_failure_latched = True
                follower._publish_failure_reason = "command_publish_failure_latched"

            follower._publish_locked(
                linear_x, angular_z, linear_y=linear_y, zero_reason=zero_reason
            )

            actual = (
                tuple(follower._last_requested_command),
                tuple(follower._last_emitted_command),
                follower._debug_zero_command_reason,
            )
            if actual != expected:
                mismatches.append(f"{key}: 期望 {expected} 实际 {actual}")

        self.assertEqual(mismatches, [], "\n".join(mismatches))

    def test_latched_output_is_independent_of_gate(self):
        # 发布失败锁存优先于安全门：锁存后所有门类型输出必须一致。
        outputs = set()
        for gate_kind in GATE_KINDS:
            key = ("nonzero", gate_kind, None, True)
            outputs.add(EXPECTED[key][1:])
        self.assertEqual(len(outputs), 1, f"锁存态下输出随门变化: {outputs}")
        self.assertEqual(
            outputs.pop(), ((0.0, 0.0, 0.0), "command_publish_failure_latched")
        )


if __name__ == "__main__":  # pragma: no cover
    unittest.main()


class PureArbitrationMatchesFixtureTest(unittest.TestCase):
    """提取出的纯函数必须与固化数据、以及与方法路径三方一致。

    只驱动 `sanitize_command_request` + `arbitrate_command`，不构造
    `PathFollower`，因此这组断言证明仲裁逻辑确实脱离了对象状态。
    """

    def test_pure_functions_match_fixture(self):
        mismatches = []
        for key, expected in sorted(EXPECTED.items(), key=lambda item: str(item[0])):
            request, gate_kind, zero_reason, latched = key
            linear_x, linear_y, angular_z = REQUESTS[request]
            gate = _make_gate(gate_kind)

            requested, sanitized_reason = sanitize_command_request(
                linear_x, angular_z, linear_y=linear_y, zero_reason=zero_reason
            )
            emitted, resolved = arbitrate_command(
                requested,
                zero_reason=sanitized_reason,
                gate=gate,
                publish_failure_latched=latched,
                publish_failure_reason=(
                    "command_publish_failure_latched" if latched else None
                ),
                pose_yaw=0.0,
                wall_time=0.0,
                control_time_source=lambda: 0.0,
            )
            actual = (tuple(requested), tuple(emitted), resolved)
            if actual != expected:
                mismatches.append(f"{key}: 期望 {expected} 实际 {actual}")

        self.assertEqual(mismatches, [], "\n".join(mismatches))

    def test_pure_and_method_paths_agree_scenario_by_scenario(self):
        """双路等价：纯函数与 `_publish_locked` 逐场景同输出。

        固化数据可能整体过期，这条断言不依赖固化数据，直接对比两条路径。
        """
        disagreements = []
        for key in sorted(EXPECTED, key=str):
            request, gate_kind, zero_reason, latched = key
            linear_x, linear_y, angular_z = REQUESTS[request]

            follower = PathFollower(cmd_vel_topic="/cmd_vel")
            method_gate = _make_gate(gate_kind)
            if method_gate is not None:
                follower.set_command_gate(method_gate)
            if latched:
                follower._publish_failure_latched = True
                follower._publish_failure_reason = "command_publish_failure_latched"
            follower._publish_locked(
                linear_x, angular_z, linear_y=linear_y, zero_reason=zero_reason
            )
            method_out = (
                tuple(follower._last_requested_command),
                tuple(follower._last_emitted_command),
                follower._debug_zero_command_reason,
            )

            requested, sanitized_reason = sanitize_command_request(
                linear_x, angular_z, linear_y=linear_y, zero_reason=zero_reason
            )
            emitted, resolved = arbitrate_command(
                requested,
                zero_reason=sanitized_reason,
                gate=_make_gate(gate_kind),
                publish_failure_latched=latched,
                publish_failure_reason=(
                    "command_publish_failure_latched" if latched else None
                ),
                pose_yaw=0.0,
                wall_time=0.0,
                control_time_source=lambda: 0.0,
            )
            pure_out = (tuple(requested), tuple(emitted), resolved)
            if method_out != pure_out:
                disagreements.append(f"{key}: 方法 {method_out} 纯函数 {pure_out}")

        self.assertEqual(disagreements, [], "\n".join(disagreements))


class ArbitrationLazinessTest(unittest.TestCase):
    """任务时间必须保持惰性读取。

    原实现只在「有门 + filter 可调用 + 未锁存」时才调用 `get_time()`。
    若提取时改成传入求值结果，无门场景会新增一次任务时间读取；而
    `get_time()` 在 ROS 时间不可用时会抛异常，等于给无门路径凭空引入
    新的失败面。这组断言把惰性钉死。
    """

    def _count_control_time_reads(self, gate_kind, *, latched=False):
        calls = []

        def _counting_source():
            calls.append(1)
            return 0.0

        requested, reason = sanitize_command_request(0.4, 0.1, 0.0, None)
        arbitrate_command(
            requested,
            zero_reason=reason,
            gate=_make_gate(gate_kind),
            publish_failure_latched=latched,
            publish_failure_reason=None,
            pose_yaw=0.0,
            wall_time=0.0,
            control_time_source=_counting_source,
        )
        return len(calls)

    def test_no_gate_does_not_read_control_time(self):
        self.assertEqual(self._count_control_time_reads("none"), 0)

    def test_gate_without_filter_does_not_read_control_time(self):
        self.assertEqual(self._count_control_time_reads("nofilter"), 0)

    def test_non_callable_filter_does_not_read_control_time(self):
        self.assertEqual(self._count_control_time_reads("noncallable"), 0)

    def test_latched_does_not_read_control_time(self):
        self.assertEqual(self._count_control_time_reads("pass", latched=True), 0)

    def test_active_gate_reads_control_time_exactly_once(self):
        self.assertEqual(self._count_control_time_reads("pass"), 1)


class ArbitrationPurityTest(unittest.TestCase):
    """仲裁必须是纯函数：同输入同输出，且不改调用方传入的对象。"""

    def test_repeated_calls_are_stable(self):
        gate = _make_gate("zero_rc")
        requested, reason = sanitize_command_request(0.4, 0.1, 0.0, None)
        first = arbitrate_command(
            requested,
            zero_reason=reason,
            gate=gate,
            publish_failure_latched=False,
            publish_failure_reason=None,
            pose_yaw=0.0,
            wall_time=0.0,
            control_time_source=lambda: 0.0,
        )
        second = arbitrate_command(
            requested,
            zero_reason=reason,
            gate=gate,
            publish_failure_latched=False,
            publish_failure_reason=None,
            pose_yaw=0.0,
            wall_time=0.0,
            control_time_source=lambda: 0.0,
        )
        self.assertEqual(first, second)

    def test_requested_tuple_is_not_mutated(self):
        requested, reason = sanitize_command_request(0.4, 0.1, 0.0, None)
        before = tuple(requested)
        arbitrate_command(
            requested,
            zero_reason=reason,
            gate=_make_gate("halve"),
            publish_failure_latched=False,
            publish_failure_reason=None,
            pose_yaw=0.0,
            wall_time=0.0,
            control_time_source=lambda: 0.0,
        )
        self.assertEqual(tuple(requested), before)


if __name__ == "__main__":
    unittest.main()
