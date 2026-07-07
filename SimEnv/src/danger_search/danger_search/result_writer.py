"""比赛结果文件输出。

本模块只写参赛算法结果，不读取裁判真值文件。
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable, Sequence


class ResultWriter:
    """按 evaluation.md 要求写出 detected_danger.json。"""

    def __init__(self, result_file: str | Path):
        self.result_file = Path(result_file)

    def write(self, exploration_time: float, positions: Iterable[Sequence[float]]) -> Path:
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
