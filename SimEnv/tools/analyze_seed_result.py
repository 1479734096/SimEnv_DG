#!/usr/bin/env python3
"""
快速分析 seed 测试结果
"""

import json
import re
import sys
from pathlib import Path

def analyze_seed_result(log_dir: Path) -> dict:
    """分析单个 seed 的测试结果"""
    result = {
        'success': False,
        'failure_mode': None,
        'duration': None,
        'key_events': [],
        'stuck_detected': False
    }

    mission_log = log_dir / "mission.log"
    if not mission_log.exists():
        result['failure_mode'] = 'no_mission_log'
        return result

    content = mission_log.read_text()

    # 检查是否成功完成
    if 'mission completed successfully' in content.lower() or \
       'full cycle completed' in content.lower():
        result['success'] = True
        result['failure_mode'] = None

    # 检查失败模式
    if 'return_home_stuck_exhausted' in content:
        result['failure_mode'] = 'return_home_stuck_exhausted'
        result['stuck_detected'] = True
    elif 'elevator_route_not_ready' in content:
        result['failure_mode'] = 'elevator_route_not_ready'
    elif 'wall_timeout' in content:
        result['failure_mode'] = 'wall_timeout'
    elif 'budget_exhausted' in content:
        result['failure_mode'] = 'budget_exhausted'

    # 提取关键事件
    for line in content.split('\n'):
        if any(kw in line.lower() for kw in ['floor', 'elevator', 'return', 'home', 'stuck']):
            result['key_events'].append(line.strip())

    return result

if __name__ == "__main__":
    log_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/phase1_quick_8127")

    if not log_dir.exists():
        print(f"❌ 日志目录不存在: {log_dir}")
        sys.exit(1)

    result = analyze_seed_result(log_dir)

    print(f"\n{'='*60}")
    print(f"Seed 测试结果分析")
    print(f"{'='*60}\n")

    if result['success']:
        print("✅ 测试成功")
    else:
        print("❌ 测试失败")
        if result['failure_mode']:
            print(f"失败模式: {result['failure_mode']}")
        if result['stuck_detected']:
            print("⚠️ 检测到卡滞问题 - 修复可能未生效")

    if result['key_events']:
        print("\n关键事件:")
        for event in result['key_events'][-10:]:  # 最后 10 个事件
            print(f"  {event}")

    print(f"\n{'='*60}\n")

    # 输出 JSON 供编排器使用
    print(json.dumps(result, indent=2))
