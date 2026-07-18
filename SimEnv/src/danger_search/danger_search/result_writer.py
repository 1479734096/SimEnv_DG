"""比赛结果文件的单向输出边界。

``MissionManager`` 在正常完成、显式失败或外部关闭时最终都会调用本模块。
本模块只负责把已确认目标序列序列化成公开 schema，不参与识别、去重或评分，
也绝不读取裁判真值文件。这样可保证生产决策与离线评估保持单向依赖。
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable, Sequence


class ResultWriter:
    """按 ``docs/evaluation.md`` 写出 ``detected_danger.json``。

    传入坐标必须已经位于比赛要求的 world/odom 数值坐标系；这里仅校验维数、
    转换数值并按固定精度输出，不擅自做坐标变换或目标合并。
    """

    def __init__(self, result_file: str | Path):
        self.result_file = Path(result_file)

    def write(self, exploration_time: float, positions: Iterable[Sequence[float]]) -> Path:
        """写入一次完整结果并返回文件路径。

        ``positions`` 可以是生成器，因此先一次性构造 JSON 数组。三维坐标保留
        毫米级小数，探索时间保留百分之一秒，以获得稳定且易比较的归档结果。
        """
        detected = []
        for position in positions:
            if len(position) != 3:
                raise ValueError(f"危险源位置必须是三维坐标: {position}")
            detected.append(
                {
                    "position": [
                        round(float(position[0]), 3),
                        round(float(position[1]), 3),
                        round(float(position[2]), 3),
                    ]
                }
            )

        payload = {
            "exploration_time": round(float(exploration_time), 2),
            "detected_danger_sources": detected,
        }
        self.result_file.parent.mkdir(parents=True, exist_ok=True)
        self.result_file.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        return self.result_file
