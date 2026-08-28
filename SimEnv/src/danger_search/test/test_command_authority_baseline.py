"""命令写入权重构（R1）基线契约。

本文件在重构前固化三件事，重构过程中不得放宽：

1. reason code 清单：所有命令写入调用点上的 reason 字面值集合。R1 允许新增
   reason code，但不允许删除或改名既有 code —— 断言用"冻结集是当前集的子集"。
2. 公开写入 API 面与 reason 参数默认值：默认值决定了不显式传 reason 时的归因，
   改动默认值会静默改变日志归因语义。
3. 零速归因优先级：``_publish_locked`` 内部的覆盖顺序。这是"dog 为什么停下"
   唯一的归因来源，R1 把 129/140 个写入点收口到单一仲裁入口时最容易破坏它。

测试全部离线运行：``get_time()`` 在未 ``init_node`` 时回落到 ``time.monotonic()``，
``_publisher`` 为 None 时 ``_publish_locked`` 在归因计算后提前返回，因此不需要 ROS。
"""
import ast
import json
import math
from pathlib import Path
import unittest

from danger_search.navigator import PathFollower

PACKAGE_DIR = Path(__file__).resolve().parents[1] / "danger_search"

# 命令写入 API：直接发 cmd_vel 的出口 + 改路径状态间接影响发送的出口。
# _publish / _publish_locked 是 navigator 内部发布原语，也纳入 reason 归因面。
COMMAND_WRITE_METHODS = frozenset(
    {
        "stop",
        "clear_path",
        "set_path",
        "recover",
        "rotate_in_place",
        "reverse_straight",
        "poll_result",
        "end_heading_alignment",
        "_publish",
        "_publish_locked",
    }
)

# reason 类关键字实参名（不同写入方法用了不同参数名）。
REASON_KEYWORDS = frozenset({"reason", "zero_reason"})


def _iter_package_sources():
    for path in sorted(PACKAGE_DIR.rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        yield path


def _collect_write_callsites():
    """AST 扫描全包，返回命令写入调用点。

    只按方法名匹配，不解析接收者类型：包内没有与这些名字冲突的其他 API
    （已人工核对），而按名字匹配能同时覆盖 ``obj.stop()`` 和裸 ``stop()``。
    """
    callsites = []
    for path in _iter_package_sources():
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            if isinstance(func, ast.Attribute):
                name = func.attr
            elif isinstance(func, ast.Name):
                name = func.id
            else:
                continue
            if name not in COMMAND_WRITE_METHODS:
                continue
            literal = None
            dynamic = False
            attributed = False
            for keyword in node.keywords:
                if keyword.arg not in REASON_KEYWORDS:
                    continue
                attributed = True
                if isinstance(keyword.value, ast.Constant) and isinstance(
                    keyword.value.value, str
                ):
                    literal = keyword.value.value
                elif isinstance(keyword.value, ast.Constant) and (
                    keyword.value.value is None
                ):
                    attributed = False
                else:
                    dynamic = True
            callsites.append(
                {
                    "file": path.name,
                    "line": node.lineno,
                    "method": name,
                    "literal": literal,
                    "dynamic": dynamic,
                    "attributed": attributed,
                }
            )
    return callsites


def _collect_write_signatures():
    """返回 navigator.py 中命令写入方法的 reason 类参数默认值。"""
    source = (PACKAGE_DIR / "navigator.py").read_text(encoding="utf-8")
    tree = ast.parse(source, filename="navigator.py")
    signatures = {}
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        if node.name not in COMMAND_WRITE_METHODS:
            continue
        defaults = {}
        args = node.args
        # 位置参数默认值右对齐
        positional = args.args
        for offset, default in enumerate(args.defaults):
            index = len(positional) - len(args.defaults) + offset
            param = positional[index].arg
            if param in REASON_KEYWORDS:
                defaults[param] = (
                    default.value if isinstance(default, ast.Constant) else "<dynamic>"
                )
        # 仅关键字参数默认值一一对应
        for param, default in zip(args.kwonlyargs, args.kw_defaults):
            if param.arg not in REASON_KEYWORDS:
                continue
            if default is None:
                defaults[param.arg] = "<required>"
            elif isinstance(default, ast.Constant):
                defaults[param.arg] = default.value
            else:
                defaults[param.arg] = "<dynamic>"
        signatures[node.name] = defaults
    return signatures


# 冻结清单：8470f6a 实测的 90 个去重 reason 字面值。
# return_home_retreat_unavailable 同时出现在 stop 与 clear_path（唯一跨方法复用）。
FROZEN_REASON_CODES = frozenset(
    {
        # _publish
        "rotate_in_place_zero",
        # _publish_locked
        "alignment_cancelled",
        "alignment_done",
        "alignment_hold",
        "alignment_rotate",
        "alignment_settle",
        "controller_pose_missing",
        "controller_pose_stale",
        "direct_command_expired",
        "goal_reached",
        "no_path",
        "recover_turn",
        "route_progress_timeout",
        # clear_path
        "door_crossing_failed",
        "door_open_wait_reached",
        "door_opening_geometry_refreshed",
        "door_pass_completed",
        "door_refreshed_wait_reached",
        "elevator_geometry_refresh_unavailable",
        "elevator_geometry_refreshed",
        "elevator_start_retreat_start",
        "elevator_trace_approach_start",
        "elevator_trace_candidate_retry",
        "elevator_transit_already_complete",
        "elevator_transit_completed",
        "elevator_visual_landmark_scan",
        "enter_return_home",
        "exit_backoff_complete",
        "exit_backoff_start",
        "initial_entry_completed",
        "initial_entry_opening_confirmed",
        "initial_entry_stuck",
        "main_entrance_clearance_blocked",
        "main_entrance_clearance_completed",
        "main_entrance_clearance_failed",
        "negative_obstacle_replan",
        "return_home_main_entrance",
        "return_home_retreat_started",
        "return_home_retreat_unavailable",
        # recover
        "elevator_enter_position_stuck",
        "return_home_stuck",
        # reverse_straight
        "elevator_start_clearance_retreat",
        "exit_cabin_backoff",
        "no_frontier_retreat",
        "return_home_retreat",
        # stop
        "door_opening_refresh",
        "door_pass_alignment_hold",
        "door_pass_blocked",
        "door_wait_point_blocked",
        "door_wait_point_blocked_safe_open",
        "elevator_geometry_refresh",
        "elevator_navigation_budget_exhausted",
        "elevator_navigation_plan_wait",
        "elevator_navigation_unreachable",
        "elevator_start_retreat_completed",
        "elevator_transit_goal_grace",
        "elevator_transit_missing_commit",
        "elevator_transit_path_lost",
        "elevator_transit_position_align_oscillation",
        "elevator_transit_unexpected_idle",
        "elevator_transit_unknown_stage",
        "elevator_visual_candidate_confirmation",
        "elevator_visual_landmark_found",
        "elevator_visual_landmark_scan_complete",
        "exit_backoff_aligned",
        "exit_backoff_distance_reached",
        "exit_backoff_failed",
        "exit_backoff_settle",
        "floor_transition_alignment_timeout",
        "frontier_planning",
        "initial_entry_heading_unavailable",
        "initial_entry_opening",
        "initial_entry_planning_constraint",
        "initial_entry_waiting_for_free_corridor",
        "invalid_path",
        "navigation_budget_exhausted",
        "navigation_target_unreachable",
        "negative_obstacle",
        "no_frontier_escape_scan_blocked",
        "no_frontier_retreat_completed",
        "no_frontier_retreat_unavailable",
        "return_home_plan_error",
        "return_home_plan_pending",
        "return_home_plan_stale",
        "return_home_plan_wait",
        "return_home_retreat_completed",
        "return_home_retreat_failed",
        "return_home_retreat_timeout",
        "unsafe_no_frontier_rotation_blocked",
        "unsafe_recover_rotation_blocked",
    }
)

# 冻结签名：reason 类参数的默认值。改这些值会静默改变未显式传 reason 时的归因。
FROZEN_REASON_DEFAULTS = {
    "stop": {"reason": "stop"},
    "clear_path": {"reason": "path_cleared"},
    "recover": {"reason": "stuck"},
    "reverse_straight": {"reason": "reverse_straight"},
    "_publish": {"zero_reason": None},
    "_publish_locked": {"zero_reason": None},
    # R1 补上：set_path 是唯一让 dog 动起来的非零命令入口，此前 14/14 调用点
    # 无归因，装载新路径也不写 _debug_replan_reason，导致快照里的 replan_reason
    # 停留在上一次清理的旧值。
    "set_path": {"reason": "path_set"},
    # 以下写入 API 仍无 reason 参数 —— 剩余归因空洞，见 R1 报告后续事项。
    "poll_result": {},
    "rotate_in_place": {},
    "end_heading_alignment": {},
}

# 无 reason 归因的调用点基线。8470f6a 实测 50 处；R1 把 set_path 的 14 处
# 全部补上归因后降到 36 处。断言用 <=，只允许继续减少。
UNATTRIBUTED_CALLSITE_BASELINE = 36


class CommandReasonInventoryTest(unittest.TestCase):
    """reason code 清单冻结。"""

    def test_frozen_reason_codes_all_still_present(self):
        callsites = _collect_write_callsites()
        current = {site["literal"] for site in callsites if site["literal"]}
        missing = sorted(FROZEN_REASON_CODES - current)
        self.assertEqual(
            missing,
            [],
            "以下 reason code 已从命令写入调用点消失（删除或改名）："
            f"{missing}。R1 允许新增 reason code，不允许删除或改名既有 code。",
        )

    def test_frozen_inventory_size_matches_measured_baseline(self):
        # 冻结清单本身的规模也要固化，防止有人"顺手"删掉冻结集里的条目
        # 让上一条断言变成空断言。
        self.assertEqual(len(FROZEN_REASON_CODES), 90)

    def test_cross_method_reason_reuse_is_explicit(self):
        """跨方法复用的 reason code 只允许已知的一个。

        同一个 code 出现在多个写入方法上会让日志无法区分是"清路径"还是"停车"，
        当前只有 return_home_retreat_unavailable 一例，新增必须显式登记。
        """
        callsites = _collect_write_callsites()
        by_code = {}
        for site in callsites:
            if not site["literal"]:
                continue
            by_code.setdefault(site["literal"], set()).add(site["method"])
        reused = {code: sorted(m) for code, m in by_code.items() if len(m) > 1}
        self.assertEqual(
            reused,
            {"return_home_retreat_unavailable": ["clear_path", "stop"]},
            f"跨方法复用的 reason code 发生变化：{reused}",
        )


class CommandWriteApiSurfaceTest(unittest.TestCase):
    """公开写入 API 面与 reason 参数默认值冻结。"""

    def test_reason_parameter_defaults_are_frozen(self):
        signatures = _collect_write_signatures()
        for method, expected in FROZEN_REASON_DEFAULTS.items():
            self.assertIn(method, signatures, f"命令写入方法 {method} 已消失")
            self.assertEqual(
                signatures[method],
                expected,
                f"{method} 的 reason 参数默认值发生变化，会静默改变归因语义",
            )

    def test_write_methods_exist_on_path_follower(self):
        # navigator 内部原语除外，其余必须是 PathFollower 的可调用属性。
        for method in sorted(COMMAND_WRITE_METHODS):
            self.assertTrue(
                callable(getattr(PathFollower, method, None)),
                f"PathFollower 缺少命令写入方法 {method}",
            )

    def test_unattributed_callsites_do_not_grow(self):
        """无 reason 归因的写入调用点只允许减少。

        R1 的目标就是把这些补上归因；断言用 <= 让改进不必回来改测试，
        但任何新增未归因写入点都会立刻失败。
        """
        callsites = _collect_write_callsites()
        unattributed = [
            site for site in callsites if not site["attributed"] and not site["dynamic"]
        ]
        self.assertLessEqual(
            len(unattributed),
            UNATTRIBUTED_CALLSITE_BASELINE,
            "新增了无 reason 归因的命令写入点："
            + str(sorted((s["file"], s["line"], s["method"]) for s in unattributed)),
        )


class ZeroCommandAttributionPriorityTest(unittest.TestCase):
    """零速归因优先级链冻结。

    ``_publish_locked`` 里的覆盖顺序（后者覆盖前者）：
      1. 调用方传入的 zero_reason
      2. invalid_requested_command（请求含 NaN/inf 或非数值）
      3. command_publish_failure_latched（发布失败已锁存）
      4. invalid_command_gate_output（门返回非法输出）
      5. ground_support_gate:<code>（门把非零请求改成了零速）
    其中第 5 条只在 ``emitted != requested`` 时生效 —— 这是关键语义：
    任务层自己请求的零速保留任务层的归因，不会被误记成安全门拦停。
    """

    @staticmethod
    def _follower():
        # _publisher 保持 None：_publish_locked 在归因计算后提前返回，无需 ROS。
        return PathFollower(cmd_vel_topic="/cmd_vel")

    class _ZeroingGate:
        """把任何命令改成零速的门。"""

        reason_code = "test_blocked"

        @staticmethod
        def filter_command(_command, **_kwargs):
            return (0.0, 0.0, 0.0)

    class _PassThroughGate:
        reason_code = None

        @staticmethod
        def filter_command(command, **_kwargs):
            return command

    class _MalformedGate:
        reason_code = "malformed"

        @staticmethod
        def filter_command(_command, **_kwargs):
            return (0.0, 0.0)

    def test_non_finite_request_is_attributed_to_invalid_request(self):
        follower = self._follower()
        follower._publish(math.nan, 0.2)
        self.assertEqual(follower._last_emitted_command, (0.0, 0.0, 0.0))
        self.assertEqual(
            follower._debug_zero_command_reason, "invalid_requested_command"
        )

    def test_gate_zeroing_non_zero_request_is_attributed_to_gate(self):
        follower = self._follower()
        follower.set_command_gate(self._ZeroingGate())
        follower._publish(0.4, 0.0)
        self.assertEqual(follower._last_emitted_command, (0.0, 0.0, 0.0))
        self.assertEqual(
            follower._debug_zero_command_reason, "ground_support_gate:test_blocked"
        )

    def test_caller_zero_request_keeps_caller_reason_even_with_gate(self):
        """任务层主动请求零速时，归因必须保留任务层的 reason。

        这是最关键的一条：如果这里被改成门归因，日志里所有"任务层主动停车"
        都会被误记成"安全门拦停"，dog 呆立的真实原因就彻底查不出来了。
        """
        follower = self._follower()
        follower.set_command_gate(self._ZeroingGate())
        follower._publish(0.0, 0.0, zero_reason="task_layer_hold")
        self.assertEqual(follower._debug_zero_command_reason, "task_layer_hold")

    def test_zero_request_without_reason_falls_back_to_zero_command(self):
        follower = self._follower()
        follower._publish(0.0, 0.0)
        self.assertEqual(follower._debug_zero_command_reason, "zero_command")

    def test_malformed_gate_output_only_observable_when_request_already_zero(self):
        """invalid_command_gate_output 的可观测条件。

        门返回非法输出时先记 invalid_command_gate_output，但只要请求是非零的，
        紧随的 emitted != requested 分支就会把它覆盖成 ground_support_gate:<code>。
        因此这个 code 只在请求本身已是零速时才可观测。
        """
        follower = self._follower()
        follower.set_command_gate(self._MalformedGate())
        follower._publish(0.0, 0.0, zero_reason="task_layer_hold")
        self.assertEqual(
            follower._debug_zero_command_reason, "invalid_command_gate_output"
        )

        other = self._follower()
        other.set_command_gate(self._MalformedGate())
        other._publish(0.4, 0.0)
        self.assertEqual(
            other._debug_zero_command_reason, "ground_support_gate:malformed"
        )

    def test_publish_failure_latch_outranks_gate_attribution(self):
        follower = self._follower()
        follower.set_command_gate(self._PassThroughGate())
        follower._publish_failure_latched = True
        follower._publish_failure_reason = "command_publish_failure_latched"
        follower._publish(0.5, 0.0)
        self.assertEqual(follower._last_emitted_command, (0.0, 0.0, 0.0))
        self.assertEqual(
            follower._debug_zero_command_reason, "command_publish_failure_latched"
        )

    def test_publish_failure_latch_has_no_reset_path(self):
        """发布失败锁存是 fail-closed 且不可复位。

        全文只有一处置 True，没有任何复位点。这是刻意设计（一次 /cmd_vel
        发布异常后不再假装还能控制），R1 不得顺手加"自动恢复"。
        """
        source = (PACKAGE_DIR / "navigator.py").read_text(encoding="utf-8")
        tree = ast.parse(source, filename="navigator.py")
        assignments = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Assign):
                continue
            for target in node.targets:
                if (
                    isinstance(target, ast.Attribute)
                    and target.attr == "_publish_failure_latched"
                ):
                    value = (
                        node.value.value
                        if isinstance(node.value, ast.Constant)
                        else "<dynamic>"
                    )
                    assignments.append((node.lineno, value))
        self.assertEqual(
            [value for _, value in assignments],
            [True],
            f"_publish_failure_latched 的赋值点发生变化：{assignments}。"
            "新增复位点会让发布失败后的零速锁存失效。",
        )


class ControlTimerPriorityTest(unittest.TestCase):
    """控制定时器决策优先级冻结：recover > align > path > keepalive。"""

    class _FakeTwist:
        def __init__(self):
            self.linear = type("V", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
            self.angular = type("V", (), {"x": 0.0, "y": 0.0, "z": 0.0})()

    class _FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    def _instrumented(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower._publisher = self._FakePublisher()
        calls = []
        for name in (
            "_control_step_locked",
            "_align_step_locked",
            "_keepalive_replay_locked",
        ):
            original = getattr(follower, name)

            def wrapper(*args, _name=name, _original=original, **kwargs):
                calls.append(_name)
                return _original(*args, **kwargs)

            setattr(follower, name, wrapper)
        return follower, calls

    def test_recover_outranks_align_and_path(self):
        follower, calls = self._instrumented()
        follower._recover_phase = "settle"
        follower._align_state = "rotate"
        follower._path = [(0.0, 0.0), (1.0, 0.0)]
        follower._on_control_timer()
        self.assertEqual(calls, ["_control_step_locked"])

    def test_align_outranks_path_when_not_recovering(self):
        follower, calls = self._instrumented()
        follower._recover_phase = "idle"
        follower._align_state = "rotate"
        follower._path = [(0.0, 0.0), (1.0, 0.0)]
        follower._on_control_timer()
        # 无位姿源：对准分支在取位姿前就归零退出，不会掉到路径分支。
        self.assertEqual(calls, [])
        self.assertNotIn("_control_step_locked", calls)

    def test_no_path_falls_back_to_keepalive(self):
        follower, calls = self._instrumented()
        follower._recover_phase = "idle"
        follower._align_state = "idle"
        follower._path = []
        follower._on_control_timer()
        self.assertEqual(calls, ["_keepalive_replay_locked"])


if __name__ == "__main__":
    unittest.main()


class PathInstallAttributionTest(unittest.TestCase):
    """``set_path`` 归因契约（R1 新增）。

    修复前 ``set_path`` 不写 ``_debug_replan_reason``：装上一条新路径后，
    快照里的 ``replan_reason`` 仍是上一次 ``clear_path`` 的旧值 —— 路径已经
    换了一条，归因还指向前一个原因，日志上无法回答“谁装的这条路径”。
    """

    def test_explicit_reason_is_recorded(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (1.0, 0.0)], reason="frontier_selected_path")
        self.assertEqual(
            follower.tracking_debug_snapshot()["replan_reason"],
            "frontier_selected_path",
        )

    def test_new_path_overwrites_stale_clear_reason(self):
        """这是修复的核心：装载新路径必须顶掉上一次清理的归因。"""
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (1.0, 0.0)], reason="frontier_selected_path")
        follower.clear_path(reason="door_pass_completed")
        self.assertEqual(
            follower.tracking_debug_snapshot()["replan_reason"],
            "door_pass_completed",
        )

        follower.set_path([(0.0, 0.0), (2.0, 0.0)], reason="probe_unknown_space_path")
        self.assertEqual(
            follower.tracking_debug_snapshot()["replan_reason"],
            "probe_unknown_space_path",
            "装载新路径后归因仍停留在上一次清理原因，'谁装的这条路径'无法回答",
        )

    def test_rejected_path_does_not_claim_installation(self):
        """被拒路径不得改写归因：它没有装上任何东西。

        归因写入刻意放在两道校验之后。若放在校验之前，一条非法路径会留下
        “我装了路径”的假证据，而实际路径状态未变。
        """
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (1.0, 0.0)], reason="frontier_selected_path")

        with self.assertRaises(ValueError):
            follower.set_path([()], reason="rejected_path")

        snapshot = follower.tracking_debug_snapshot()
        self.assertEqual(
            snapshot["replan_reason"],
            "frontier_selected_path",
            "被拒路径改写了归因，留下未发生的装载证据",
        )
        # 失败路径本身仍按既有语义归因零速。
        self.assertEqual(snapshot["zero_command_reason"], "invalid_path")

    def test_default_reason_is_explicit_not_none(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        self.assertEqual(
            follower.tracking_debug_snapshot()["replan_reason"], "path_set"
        )

    def test_all_set_path_callsites_are_attributed(self):
        """``set_path`` 的归因不允许回退。

        整体棘轮（``test_unattributed_callsites_do_not_grow``）只保证总数不增；
        这条把 set_path 单独钉死，防止新增调用点吃默认值而总数因别处改进被抵消。
        """
        callsites = [
            site for site in _collect_write_callsites() if site["method"] == "set_path"
        ]
        self.assertEqual(len(callsites), 16, "set_path 调用点数量变化，需复核归因覆盖")
        unattributed = [
            (site["file"], site["line"])
            for site in callsites
            if not site["attributed"] and not site["dynamic"]
        ]
        self.assertEqual(unattributed, [], f"set_path 出现无归因调用点：{unattributed}")

    def test_set_path_reasons_are_distinct_per_callsite(self):
        """15 个静态 reason 必须互不相同。

        另一个异步前沿统一安装入口使用显式动态 reason；静态调用点若复用同一
        code，会让“哪个状态机装的路径”重新变得不可区分。
        """
        literals = [
            site["literal"]
            for site in _collect_write_callsites()
            if site["method"] == "set_path" and site["literal"]
        ]
        duplicates = sorted({code for code in literals if literals.count(code) > 1})
        self.assertEqual(duplicates, [], f"set_path 出现重复 reason code：{duplicates}")
        self.assertEqual(len(literals), 15)


class CrossLayerPrivateAccessTest(unittest.TestCase):
    """状态机层不得读控制层私有成员。

    诊断载荷曾用 ``controller._last_goal_distance`` 与 ``controller._recover_until``
    绕过公开快照。前者是死字段（全仓库只被赋 ``math.inf``），使 ``path_length``
    恒为 ``Infinity``，``json.dumps`` 默认 ``allow_nan=True`` 会静默写出非法
    JSON 字面量。两者都有语义正确的公开替代（``route_progress`` /
    ``recover_remaining``），且同一函数早已持有该快照。

    本断言按 AST 检查，不依赖运行时构造 MissionManager。
    """

    # 状态机/任务层模块：这些不得触碰 controller 的私有成员。
    STATE_MACHINE_MODULES = (
        "mission_manager.py",
        "navigation_tasks.py",
        "exploration_runtime.py",
        "initial_entry_runtime.py",
        "return_home_runtime.py",
        "elevator_observation_runtime.py",
        "ground_support_coordinator.py",
    )

    # 被视为控制层句柄的属性名（self.controller / context.controller / 形参）。
    CONTROLLER_HOLDERS = frozenset({"controller"})

    def _violations(self):
        found = []
        for path in _iter_package_sources():
            if path.name not in self.STATE_MACHINE_MODULES:
                continue
            tree = ast.parse(path.read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                # 形如 <任意>.controller._private
                if not isinstance(node, ast.Attribute):
                    continue
                if not node.attr.startswith("_"):
                    continue
                base = node.value
                if not isinstance(base, ast.Attribute):
                    continue
                if base.attr not in self.CONTROLLER_HOLDERS:
                    continue
                found.append((path.name, node.lineno, node.attr))
        return sorted(found)

    def test_state_machines_do_not_read_controller_privates(self):
        violations = self._violations()
        self.assertEqual(
            violations,
            [],
            "状态机层读取了控制层私有成员，应改用 tracking_debug_snapshot() "
            f"暴露的公开字段：{violations}",
        )

    def test_public_snapshot_exposes_the_two_replacements(self):
        # 若这两个公开字段被改名/删除，上面的断言会逼人重新绕私有量；
        # 因此把替代品本身也钉住。
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        snapshot = follower.tracking_debug_snapshot()
        self.assertIn("route_progress", snapshot)
        self.assertIn("recover_remaining", snapshot)

    def test_route_progress_is_json_safe_with_and_without_path(self):
        follower = PathFollower(cmd_vel_topic="/cmd_vel")
        # 无路径
        json.dumps(follower.tracking_debug_snapshot()["route_progress"], allow_nan=False)
        # 有路径并推进若干步
        follower.set_path([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], reason="test_path")
        for _ in range(3):
            follower.tick((0.0, 0.0, 0.0))
        value = follower.tracking_debug_snapshot()["route_progress"]
        json.dumps(value, allow_nan=False)
