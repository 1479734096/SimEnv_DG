#!/usr/bin/env python3
"""审计当前 ROS master 上的完整正式运行图，不把局部节点检查外推为整机合规。"""

from __future__ import annotations

import argparse
from collections.abc import Iterable


FORBIDDEN_INPUT_PREFIXES = ("/ground_truth/",)
FORBIDDEN_INPUT_TOPICS = frozenset({"/Odometry_gazebo"})
EXPECTED_CMD_VEL_PUBLISHERS = frozenset({"/danger_search"})
PRODUCTION_ALGORITHM_NODES = frozenset({"/danger_search", "/scan_localization"})
TF_TOPICS = frozenset({"/tf", "/tf_static"})
EXPECTED_INTERNAL_PUBLISHERS = {
    "/danger_search/localization/odom": frozenset({"/scan_localization"}),
    "/danger_search/localization/cloud_registered": frozenset(
        {"/scan_localization"}
    ),
    "/danger_search/localization/health": frozenset({"/scan_localization"}),
    "/danger_search/localization/ready": frozenset({"/scan_localization"}),
}


def _normalized_state(entries: Iterable[tuple[str, Iterable[str]]]):
    return {
        str(topic): frozenset(str(node) for node in nodes)
        for topic, nodes in entries
    }


def audit_runtime_graph(*, publishers, subscribers) -> list[str]:
    """返回完整运行图违规；空列表才表示本次图检查通过。"""
    publisher_map = _normalized_state(publishers)
    subscriber_map = _normalized_state(subscribers)
    errors: list[str] = []

    for topic, nodes in sorted(subscriber_map.items()):
        forbidden = topic in FORBIDDEN_INPUT_TOPICS or topic.startswith(
            FORBIDDEN_INPUT_PREFIXES
        )
        if forbidden and nodes:
            errors.append(
                f"禁用输入 {topic} 存在订阅者: {', '.join(sorted(nodes))}"
            )

    cmd_publishers = publisher_map.get("/cmd_vel", frozenset())
    unexpected_cmd = cmd_publishers - EXPECTED_CMD_VEL_PUBLISHERS
    missing_cmd = EXPECTED_CMD_VEL_PUBLISHERS - cmd_publishers
    if unexpected_cmd:
        errors.append(
            "/cmd_vel 存在竞争 publisher: "
            + ", ".join(sorted(unexpected_cmd))
        )
    if missing_cmd:
        errors.append(
            "/cmd_vel 缺少唯一正式 publisher: "
            + ", ".join(sorted(missing_cmd))
        )

    joy_publishers = publisher_map.get("/joy", frozenset())
    if joy_publishers:
        errors.append(
            "正式运行存在 /joy publisher: "
            + ", ".join(sorted(joy_publishers))
        )

    for topic in TF_TOPICS:
        consumers = (
            subscriber_map.get(topic, frozenset())
            & PRODUCTION_ALGORITHM_NODES
        )
        if consumers:
            errors.append(
                f"参赛生产节点意外订阅 {topic}: "
                + ", ".join(sorted(consumers))
            )

    for topic, expected in EXPECTED_INTERNAL_PUBLISHERS.items():
        actual = publisher_map.get(topic, frozenset())
        if actual != expected:
            errors.append(
                f"内部话题 {topic} 生产者不匹配: "
                f"期望={','.join(sorted(expected))}，"
                f"实际={','.join(sorted(actual)) or '<none>'}"
            )
    return errors


def read_ros_master_state():
    """读取当前 master；ROS 依赖仅在实际运行审计时导入。"""
    import rosgraph

    master = rosgraph.Master("/danger_search_runtime_graph_audit")
    publishers, subscribers, _services = master.getSystemState()
    return publishers, subscribers


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    try:
        publishers, subscribers = read_ros_master_state()
    except Exception as exc:
        print(f"完整 ROS graph 审计无法连接 ROS master: {exc}")
        return 2
    errors = audit_runtime_graph(
        publishers=publishers,
        subscribers=subscribers,
    )
    publisher_map = _normalized_state(publishers)
    for topic in sorted(TF_TOPICS):
        nodes = publisher_map.get(topic, frozenset())
        print(
            f"{topic} publishers: "
            + (", ".join(sorted(nodes)) if nodes else "<none>")
        )
    if errors:
        print("完整 ROS graph 审计失败：")
        for error in errors:
            print(f"- {error}")
        return 1
    print("完整 ROS graph 审计通过。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
